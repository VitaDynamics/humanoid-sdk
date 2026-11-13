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
#include <vector>

#include <aorta.h>

#include "body_cmd_vendor/message/schemas/topic/lowlevel/body_cmd_generated.h"
#include "body_state_decoder.h"
#include "end_effector_decoder.h"
#include "body_cmd_vendor/message/schemas/topic/lowlevel/end_effector_cmd_generated.h"

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
  explicit BodyAllocator(size_t limit = 0, const char* root = "BodyCmd")
      : limit_(limit), root_(root) {}
  uint8_t* allocate(size_t size) override {
    if (limit_ && size > limit_)
      throw std::length_error(std::string(root_) + " exceeds its cold layout capacity");
    maximum_ = std::max(maximum_, size);
    ++allocations_;
    return flatbuffers::DefaultAllocator::allocate(size);
  }
  size_t Maximum() const { return maximum_; }
  size_t Allocations() const { return allocations_; }

 private:
  const size_t limit_;
  const char* const root_;
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
  explicit HeaderContext(py::object binding, bool prepare_body = true)
      : binding_(std::move(binding)) {
    reserve_ = reinterpret_cast<decltype(&aorta_publisher_header_context_try_reserve)>(
        binding_.attr("reserve_address").cast<uintptr_t>());
    free_ = reinterpret_cast<decltype(&aorta_publisher_header_context_free)>(
        binding_.attr("free_address").cast<uintptr_t>());
    node_ = binding_.attr("publisher_node").cast<std::string>();
    const bool include = binding_.attr("include_publisher_node").cast<bool>();
    if (!reserve_ || !free_ || (!include && !node_.empty()))
      throw py::value_error("invalid public native header binding");
    if (prepare_body) body_capacity_ = BodyCapacity(node_);
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
  if (protocol != 3 && protocol != 4)
    throw py::value_error("BodyCmd requires protocol version 3 or 4");
  std::array<uint64_t, 6> fields;
  const char *names[] = {"session_id",           "sequence",
                         "input_state_id",       "input_state_ts_real_us",
                         "ts_client_created_us", "ts_client_send_us"};
  for (size_t i = 0; i < fields.size(); ++i)
    fields[i] = command.attr(names[i]).cast<uint64_t>();
  const auto action = command.attr("action").cast<uint8_t>();
  if (protocol == 4 && action != lowlevel::BodyCmdAction_EXTERNAL)
    throw py::value_error("BodyCmd protocol version 4 requires EXTERNAL action");
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

uint64_t EEId(py::handle value) {
  if (!PyLong_CheckExact(value.ptr()))
    throw py::value_error("EE identity must be uint64");
  return py::cast<uint64_t>(value);
}

template <typename Axis, typename Sample>
void WriteEE(flatbuffers::FlatBufferBuilder& builder, size_t n, size_t k,
             Axis axis, Sample sample, uint64_t runtime, uint32_t layout,
             uint8_t side, uint64_t control, const lowlevel::CommandKey& key,
             uint64_t created, uint64_t sent, uint64_t stamp,
             const std::string& node, uint64_t sequence) {
  const auto name = node.empty() ? flatbuffers::Offset<flatbuffers::String>()
                                 : builder.CreateString(node);
  aorta::sys::AortaHeaderBuilder header(builder);
  header.add_publish_stamp_ns(stamp);
  if (!node.empty()) header.add_publisher_node(name);
  header.add_sequence(sequence);
  const auto header_offset = header.Finish();
  lowlevel::SampleRef* samples;
  // Python StartVector retains struct alignment for K=0; C++ PreAlign(0)
  // skips it. Keep the original byte oracle even for an empty observation set.
  if (k == 0) builder.Align(flatbuffers::AlignOf<lowlevel::SampleRef>());
  const auto observations = builder.CreateUninitializedVectorOfStructs(k, &samples);
  for (size_t i = 0; i < k; ++i) samples[i] = sample(i);
  lowlevel::ee::AxisCommand* axes;
  const auto values = builder.CreateUninitializedVectorOfStructs(n, &axes);
  for (size_t i = 0; i < n; ++i) axes[i] = axis(i);
  lowlevel::ee::EndEffectorCmdBuilder command(builder);
  command.add_aorta_header(header_offset);
  command.add_input_observations(observations);
  command.add_axes(values);
  builder.ForceDefaults(true);
  command.add_protocol_version(2);
  command.add_runtime_id(runtime);
  command.add_key(&key);
  command.add_layout_id(layout);
  command.add_side(side);
  command.add_control_id(control);
  command.add_ts_client_created_us(created);
  command.add_ts_client_send_us(sent);
  builder.ForceDefaults(false);
  builder.Finish(command.Finish());
}

class EECommandContext {
 public:
  EECommandContext(py::object binding, uint64_t runtime, uint32_t layout,
                   uint8_t side, size_t n, py::tuple sources)
      : header_(std::move(binding), false), runtime_(runtime), layout_(layout),
        side_(side), n_(n) {
    if (!runtime || (side != 1 && side != 2) || !n || sources.size() > 5)
      throw py::value_error("invalid frozen EE command layout");
    for (auto value : sources) {
      const auto pair = py::cast<py::tuple>(value);
      if (pair.size() != 2) throw py::value_error("invalid observation source");
      const auto stream = pair[0].cast<uint8_t>();
      const auto source_side = pair[1].cast<uint8_t>();
      const std::pair<uint8_t, uint8_t> source(stream, source_side);
      if (!((stream == 1 && source_side == 0) ||
            ((stream == 2 || stream == 3) && (source_side == 1 || source_side == 2))) ||
          std::find(sources_.begin(), sources_.end(), source) != sources_.end())
        throw py::value_error("invalid or duplicate observation source");
      sources_.push_back(source);
    }
    BodyAllocator allocation(0, "EndEffectorCmd");
    // The same writer sizes the actual frozen N/K/node, including scratch,
    // vectors, tables and all zero/nonzero Header-presence combinations.
    for (uint64_t stamp : {uint64_t{0}, uint64_t{1}}) {
      for (uint64_t sequence : {uint64_t{0}, uint64_t{1}}) {
        flatbuffers::FlatBufferBuilder builder(256, &allocation);
        WriteEE(builder, n_, sources_.size(),
                [](size_t) { return lowlevel::ee::AxisCommand(); },
                [](size_t) { return lowlevel::SampleRef(); }, runtime_, layout_,
                side_, 0, lowlevel::CommandKey(), 0, 0, stamp, header_.Node(), sequence);
      }
    }
    capacity_ = allocation.Maximum();
  }
  aorta_result_t Close() { return header_.Close(); }
  size_t CapacityBytes() const { return capacity_; }

  py::tuple EncodeCommand(py::object command, bool measure) {
    aorta_native_header_t header{};
    const auto began = measure ? Clock::now() : Stamp{};
    const auto rc = header_.Reserve(&header);
    const auto reserved = measure ? Clock::now() : Stamp{};
    if (rc != AORTA_OK) return py::make_tuple(rc, py::none(), py::none());
    if (header.publish_stamp_ns < 0)
      throw py::type_error("native header timestamp cannot be encoded as uint64");
    if (command.attr("protocol_version").cast<uint32_t>() != 2 ||
        EEId(command.attr("runtime_id")) != runtime_ ||
        command.attr("layout_id").cast<uint32_t>() != layout_ ||
        command.attr("side").cast<uint8_t>() != side_)
      throw py::value_error("EE command differs from its frozen Info");
    const auto control = EEId(command.attr("control_id"));
    if (!control) throw py::value_error("EE command requires nonzero control_id");
    const auto key = command.attr("key");
    const lowlevel::CommandKey command_key(EEId(key.attr("session_id")),
                                              EEId(key.attr("sequence")));
    const auto created = EEId(command.attr("ts_client_created_us"));
    const auto sent = EEId(command.attr("ts_client_send_us"));
    const auto target = command.attr("target");
    const auto mode = target.attr("mode").cast<py::tuple>();
    const auto fields = target.attr("command_fields").cast<py::tuple>();
    if (mode.size() != n_ || fields.size() != n_)
      throw py::value_error("EE command axis count differs from frozen Info");
    const char* names[] = {"pos", "vel", "vel_limit", "tau", "current"};
    std::array<py::object, 5> columns;
    for (size_t i = 0; i < columns.size(); ++i) {
      columns[i] = target.attr(names[i]);
      if (!py::isinstance<py::tuple>(columns[i]) ||
          (py::len(columns[i]) && py::len(columns[i]) != n_))
        throw py::value_error("EE command column differs from frozen Info");
    }
    const auto observations = command.attr("input_observations").cast<py::tuple>();
    if (observations.size() != sources_.size())
      throw py::value_error("EE observations differ from startup K");
    BodyAllocator allocation(capacity_, "EndEffectorCmd");
    flatbuffers::FlatBufferBuilder builder(capacity_, &allocation);
    WriteEE(builder, n_, sources_.size(), [&](size_t index) {
      std::array<float, 5> row{};
      for (size_t i = 0; i < row.size(); ++i) {
        if (py::len(columns[i])) row[i] = Float(columns[i][py::int_(index)]);
        if (!std::isfinite(row[i])) throw py::value_error("EE command must be finite float32");
      }
      return lowlevel::ee::AxisCommand(fields[index].cast<uint32_t>(),
          mode[index].cast<uint16_t>(), row[0], row[1], row[2], row[3], row[4]);
    }, [&](size_t index) {
      const auto sample = observations[index];
      // Local R is checked before projecting the smaller wire SampleRef.
      if (EEId(sample.attr("runtime_id")) != runtime_)
        throw py::value_error("observation runtime_id differs from command runtime_id");
      const auto stream = sample.attr("stream").cast<uint8_t>();
      const auto side = sample.attr("side").cast<uint8_t>();
      if (std::pair<uint8_t, uint8_t>(stream, side) != sources_[index])
        throw py::value_error("EE observations differ from startup stream/side order");
      const auto validity = sample.attr("validity").cast<uint8_t>();
      if (validity & ~uint8_t{15}) throw py::value_error("unknown SampleRef validity bits");
      return lowlevel::SampleRef(stream, side, EEId(sample.attr("sample_id")),
                                    EEId(sample.attr("source_time_us")), validity);
    }, runtime_, layout_, side_, control, command_key, created, sent,
       static_cast<uint64_t>(header.publish_stamp_ns), header_.Node(), header.sequence);
    py::bytes payload(reinterpret_cast<const char*>(builder.GetBufferPointer()), builder.GetSize());
    if (!measure) return py::make_tuple(rc, payload, py::none());
    py::dict trace;
    trace["header_reserve_ns"] = Ns(began, reserved);
    trace["header_sequence"] = header.sequence;
    trace["encode_native_ns"] = Ns(reserved, Clock::now());
    trace["encode_buffer_capacity_bytes"] = allocation.Maximum();
    trace["encode_buffer_allocations"] = allocation.Allocations();
    return py::make_tuple(rc, payload, trace);
  }

 private:
  HeaderContext header_;
  const uint64_t runtime_;
  const uint32_t layout_;
  const uint8_t side_;
  const size_t n_;
  std::vector<std::pair<uint8_t, uint8_t>> sources_;  // Cold, immutable after construction.
  size_t capacity_ = 0;
};
}  // namespace

PYBIND11_MODULE(_body_cmd_native, module) {
  BindBodyState(module);
  BindEndEffectorFeedback(module);
  module.attr("BODY_SCHEMA_SHA256") = BODY_CMD_SCHEMA_SHA256;
  module.attr("BODY_CMD_PROTOCOL_VERSION") = 3;
  module.attr("BODY_CMD_MOTOR_THETA_PROTOCOL_VERSION") = 4;
  module.attr("EE_CMD_SCHEMA_SHA256") = EE_CMD_SCHEMA_SHA256;
  module.attr("EE_CMD_PROTOCOL_VERSION") = 2;
  py::class_<EECommandContext>(module, "EECommandContext")
      .def(py::init<py::object, uint64_t, uint32_t, uint8_t, size_t, py::tuple>())
      .def_property_readonly("capacity_bytes", &EECommandContext::CapacityBytes)
      .def("encode", &EECommandContext::EncodeCommand, py::arg("command"),
           py::arg("measure") = false)
      .def("close", &EECommandContext::Close);
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
