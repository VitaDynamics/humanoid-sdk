#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include <aorta.h>

#include "body_cmd_vendor/message/schemas/topic/lowlevel/body_cmd_generated.h"
#include "body_state_decoder.h"
#include "end_effector_decoder.h"

namespace py = pybind11;
namespace {
using Clock = std::chrono::steady_clock;
using Stamp = Clock::time_point;
int64_t Ns(Stamp from, Stamp to) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(to - from)
      .count();
}

float Float(py::handle value) {
  const double number = py::cast<double>(value);
  const float result = static_cast<float>(number);
  if (std::isfinite(number) && std::isinf(result))
    throw std::overflow_error("float too large to pack with f format");
  return result;
}

void WriteBody(flatbuffers::FlatBufferBuilder& builder,
               const std::array<lowlevel::BodyJointCmd, 31>& values,
               uint32_t protocol, const std::array<uint64_t, 6>& fields,
               uint8_t action, const std::optional<std::array<float, 4>>& quat,
               uint64_t runtime, uint64_t control, uint64_t stamp,
               const std::string& publisher_node, uint64_t header_sequence) {
  auto string = publisher_node.empty()
                    ? flatbuffers::Offset<flatbuffers::String>()
                    : builder.CreateString(publisher_node);
  // Keep the original Python oracle's primitive insertion order.
  aorta::sys::AortaHeaderBuilder header(builder);
  header.add_publish_stamp_ns(stamp);
  if (!publisher_node.empty()) header.add_publisher_node(string);
  header.add_sequence(header_sequence);
  const auto header_offset = header.Finish();
  const auto vector = builder.CreateVectorOfStructs(values.data(), values.size());
  lowlevel::BodyCmdBuilder body(builder);
  body.add_aorta_header(header_offset);
  body.add_joints(vector);
  builder.ForceDefaults(true);
  body.add_protocol_version(protocol);
  body.add_session_id(fields[0]);
  body.add_sequence(fields[1]);
  body.add_input_state_id(fields[2]);
  body.add_input_state_ts_real_us(fields[3]);
  body.add_ts_client_created_us(fields[4]);
  body.add_ts_client_send_us(fields[5]);
  body.add_action(static_cast<lowlevel::BodyCmdAction>(action));
  if (quat) {
    lowlevel::RootQuatWxyz value((*quat)[0], (*quat)[1], (*quat)[2], (*quat)[3]);
    body.add_root_quat(&value);
  }
  body.add_runtime_id(runtime);
  body.add_control_id(control);
  builder.ForceDefaults(false);
  builder.Finish(body.Finish());
}

class BodyAllocator final : public flatbuffers::DefaultAllocator {
 public:
  explicit BodyAllocator(size_t limit = 0) : limit_(limit) {}
  uint8_t* allocate(size_t size) override {
    if (limit_ && size > limit_)
      throw std::length_error("BodyCmd exceeds its cold layout capacity");
    maximum_ = std::max(maximum_, size);
    ++allocations_;
    return flatbuffers::DefaultAllocator::allocate(size);
  }
  size_t Maximum() const { return maximum_; }
  size_t Allocations() const { return allocations_; }

 private:
  const size_t limit_;
  size_t maximum_ = 0;
  size_t allocations_ = 0;
};

size_t BodyCapacity(const std::string& publisher_node) {
  // Only header zero/nonzero presence changes this fixed 31-axis layout.
  // These are private sizing buffers, never submitted as commands.
  std::array<lowlevel::BodyJointCmd, 31> values{};
  const std::array<uint64_t, 6> fields{};
  const std::optional<std::array<float, 4>> quat(std::array<float, 4>{1, 0, 0, 0});
  BodyAllocator allocation;
  for (uint64_t stamp : {uint64_t{0}, uint64_t{1}}) {
    for (uint64_t sequence : {uint64_t{0}, uint64_t{1}}) {
      flatbuffers::FlatBufferBuilder builder(256, &allocation);
      WriteBody(builder, values, 3, fields, 18, quat, 0, 0, stamp,
                publisher_node, sequence);
    }
  }
  return allocation.Maximum();
}

class HeaderContext {
 public:
  explicit HeaderContext(py::object binding) : binding_(std::move(binding)) {
    reserve_ = reinterpret_cast<decltype(&aorta_publisher_header_context_try_reserve)>(
        binding_.attr("reserve_address").cast<uintptr_t>());
    free_ = reinterpret_cast<decltype(&aorta_publisher_header_context_free)>(
        binding_.attr("free_address").cast<uintptr_t>());
    node_ = binding_.attr("publisher_node").cast<std::string>();
    const bool include = binding_.attr("include_publisher_node").cast<bool>();
    if (!reserve_ || !free_ || (!include && !node_.empty()))
      throw py::value_error("invalid public native header binding");
    body_capacity_ = BodyCapacity(node_);
    context_ = reinterpret_cast<aorta_header_context_t>(
        binding_.attr("acquire_context")().cast<uintptr_t>());
    if (!context_) throw py::value_error("native header context is null");
  }
  HeaderContext(const HeaderContext&) = delete;
  HeaderContext& operator=(const HeaderContext&) = delete;
  ~HeaderContext() { Close(); }
  aorta_result_t Close() {
    const auto context = context_;
    context_ = nullptr;  // Valid free consumes the handle even on INTERNAL.
    return context ? free_(context) : AORTA_OK;
  }
  aorta_result_t Reserve(aorta_native_header_t* out) {
    return context_ ? reserve_(context_, out) : AORTA_ERR_NOT_FOUND;
  }
  const std::string& Node() const { return node_; }
  size_t BodyCapacityBytes() const { return body_capacity_; }

 private:
  // Retain the same publisher and the library owning these function addresses.
  py::object binding_;
  decltype(&aorta_publisher_header_context_try_reserve) reserve_ = nullptr;
  decltype(&aorta_publisher_header_context_free) free_ = nullptr;
  aorta_header_context_t context_ = nullptr;
  std::string node_;
  size_t body_capacity_ = 0;
};
static_assert(sizeof(aorta_native_header_t) == 16);
static_assert(offsetof(aorta_native_header_t, sequence) == 0);
static_assert(offsetof(aorta_native_header_t, publish_stamp_ns) == 8);

py::tuple Encode(py::object command, uint64_t stamp, py::object node,
                 uint64_t header_sequence, bool measure,
                 const std::string* prepared_node = nullptr,
                 size_t body_capacity = 0) {
  const auto begin = measure ? Clock::now() : Stamp{};
  py::object joints = command.attr("joints");
  if (!py::isinstance<py::tuple>(joints) || py::len(joints) != 31)
    throw py::value_error("BodyCmd requires a 31-slot tuple");
  std::array<lowlevel::BodyJointCmd, 31> values;
  size_t joint_index = 0;
  for (auto joint : joints) {
    std::array<float, 5> row;
    const char *names[] = {"q", "dq", "tau", "kp", "kd"};
    for (size_t i = 0; i < row.size(); ++i)
      row[i] = Float(joint.attr(names[i]));
    values[joint_index++] = lowlevel::BodyJointCmd(row[0], row[1], row[2],
                                                  row[3], row[4]);
  }
  const auto protocol = command.attr("protocol_version").cast<uint32_t>();
  if (protocol != 3) throw py::value_error("BodyCmd requires protocol version 3");
  std::array<uint64_t, 6> fields;
  const char *names[] = {"session_id",           "sequence",
                         "input_state_id",       "input_state_ts_real_us",
                         "ts_client_created_us", "ts_client_send_us"};
  for (size_t i = 0; i < fields.size(); ++i)
    fields[i] = command.attr(names[i]).cast<uint64_t>();
  const auto action = command.attr("action").cast<uint8_t>();
  py::object runtime_value = command.attr("runtime_id");
  py::object control_value = command.attr("control_id");
  if (!PyLong_CheckExact(runtime_value.ptr()) || !PyLong_CheckExact(control_value.ptr()))
    throw py::value_error("BodyCmd runtime and control IDs must be uint64");
  const auto runtime = runtime_value.cast<uint64_t>();
  const auto control = control_value.cast<uint64_t>();
  if (runtime == 0 || control == 0)
    throw py::value_error("BodyCmd requires nonzero runtime and control IDs");
  std::optional<std::array<float, 4>> quat;
  py::object root = command.attr("root_quat");
  if (root.is_none() && action != 14) quat = std::array<float, 4>{1, 0, 0, 0};
  if (root.is_none() && action == 14)
    throw py::value_error("tracking BodyCmd requires root_quat");
  if (!root.is_none()) {
    if (py::len(root) != 4)
      throw py::value_error("root_quat requires four values");
    quat.emplace();
    for (size_t i = 0; i < 4; ++i) (*quat)[i] = Float(root[py::int_(i)]);
  }
  std::string node_storage;
  const auto& publisher_node = prepared_node ? *prepared_node
      : (node_storage = node.is_none() ? "" : node.cast<std::string>());
  const auto marshalled = measure ? Clock::now() : Stamp{};
  Stamp start{}, finish{};
  if (measure) {
    start = Clock::now();
  }
  BodyAllocator allocation(body_capacity);
  flatbuffers::FlatBufferBuilder builder(body_capacity ? body_capacity : 256,
                                         &allocation);
  WriteBody(builder, values, protocol, fields, action, quat, runtime, control,
            stamp, publisher_node, header_sequence);
  if (measure) finish = Clock::now();
  const auto acquired = measure ? Clock::now() : Stamp{};
  py::bytes bytes(reinterpret_cast<const char *>(builder.GetBufferPointer()),
                  builder.GetSize());
  if (!measure) return py::make_tuple(bytes, py::none());
  const auto copied = Clock::now();
  py::dict timing;
  timing["marshal_ns"] = Ns(begin, marshalled);
  timing["encode_native_ns"] = Ns(start, finish);
  timing["encode_return_bytes_ns"] = Ns(acquired, copied);
  timing["encode_buffer_capacity_bytes"] = allocation.Maximum();
  timing["encode_buffer_allocations"] = allocation.Allocations();
  return py::make_tuple(bytes, timing);
}

py::tuple EncodeWithHeader(py::object command, HeaderContext& context, bool measure) {
  aorta_native_header_t header{};  // Per-call storage also isolates getter reentry.
  const auto began = measure ? Clock::now() : Stamp{};
  const auto rc = context.Reserve(&header);
  const auto reserved = measure ? Clock::now() : Stamp{};
  if (rc != AORTA_OK) return py::make_tuple(rc, py::none(), py::none());
  if (header.publish_stamp_ns < 0)
    throw py::type_error("native header timestamp cannot be encoded as uint64");
  auto encoded = Encode(command, static_cast<uint64_t>(header.publish_stamp_ns),
                        py::none(), header.sequence, measure, &context.Node(),
                        context.BodyCapacityBytes());
  if (measure) {
    auto timing = encoded[1].cast<py::dict>();
    timing["header_reserve_ns"] = Ns(began, reserved);
    timing["header_sequence"] = header.sequence;
  }
  return py::make_tuple(rc, encoded[0], encoded[1]);
}
}  // namespace

PYBIND11_MODULE(_body_cmd_native, module) {
  BindBodyState(module);
  BindEndEffectorFeedback(module);
  module.attr("BODY_SCHEMA_SHA256") = BODY_CMD_SCHEMA_SHA256;
  module.attr("BODY_CMD_PROTOCOL_VERSION") = 3;
  py::class_<HeaderContext>(module, "HeaderContext")
      .def(py::init<py::object>())
      .def_property_readonly("body_capacity_bytes", &HeaderContext::BodyCapacityBytes)
      .def("close", &HeaderContext::Close);
  module.def("encode_with_header", &EncodeWithHeader, py::arg("command"),
             py::arg("context"), py::arg("measure") = false);
  module.def("encode", [](py::object command, uint64_t stamp, py::object node,
                           uint64_t sequence, bool measure) {
    return Encode(command, stamp, node, sequence, measure);
  }, py::arg("command"), py::arg("publish_stamp_ns"),
             py::arg("publisher_node"), py::arg("header_sequence"),
             py::arg("measure") = false);
}
