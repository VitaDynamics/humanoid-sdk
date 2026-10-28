#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>

#include "body_cmd_vendor/body_cmd_generated.h"
#include "body_state_decoder.h"

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

struct BodyEncodeScratch {
  std::array<lowlevel::BodyJointCmd, 31> joints;
};

py::tuple Encode(py::object command, uint64_t stamp, py::object node,
                 uint64_t header_sequence, bool measure,
                 BodyEncodeScratch* scratch) {
  const auto begin = measure ? Clock::now() : Stamp{};
  py::object joints = command.attr("joints");
  if (!py::isinstance<py::tuple>(joints) || py::len(joints) != 31)
    throw py::value_error("BodyCmd requires a 31-slot tuple");
  // Encode holds the GIL throughout; a publisher's scratch cannot race here.
  BodyEncodeScratch local;
  auto& values = scratch ? scratch->joints : local.joints;
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
  py::object ref = command.attr("descriptor_ref");
  if (ref.is_none()) throw py::value_error("BodyCmd requires DescriptorRef");
  py::object runtime_value = ref.attr("robot_runtime_id");
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
  std::string publisher_node = node.is_none() ? "" : node.cast<std::string>();
  const auto marshalled = measure ? Clock::now() : Stamp{};
  Stamp start{}, finish{};
  if (measure) {
    start = Clock::now();
  }
  flatbuffers::FlatBufferBuilder builder(256);
  auto string = publisher_node.empty()
                    ? flatbuffers::Offset<flatbuffers::String>()
                    : builder.CreateString(publisher_node);
  // Match aorta.header.build_aorta_header's primitive order, not the
  // generated Create* helper's reordered fields.
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
    lowlevel::RootQuatWxyz value((*quat)[0], (*quat)[1], (*quat)[2],
                                 (*quat)[3]);
    body.add_root_quat(&value);
  }
  const lowlevel::ee::DescriptorRef descriptor(runtime);
  body.add_descriptor_ref(&descriptor);
  body.add_control_id(control);
  builder.ForceDefaults(false);
  builder.Finish(body.Finish());
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
  return py::make_tuple(bytes, timing);
}
}  // namespace

PYBIND11_MODULE(_body_cmd_native, module) {
  BindBodyState(module);
  py::class_<BodyEncodeScratch>(module, "BodyEncodeScratch").def(py::init<>());
  module.attr("BODY_SCHEMA_SHA256") = BODY_CMD_SCHEMA_SHA256;
  module.attr("BODY_CMD_PROTOCOL_VERSION") = 3;
  module.def("encode", &Encode, py::arg("command"), py::arg("publish_stamp_ns"),
             py::arg("publisher_node"), py::arg("header_sequence"),
             py::arg("measure") = false, py::arg("scratch") = nullptr);
}
