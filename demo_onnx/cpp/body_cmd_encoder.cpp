#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

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
  // PyFloat_AsDouble matches struct.pack's numeric protocol, including bool
  // in unpopulated slots (the controller intentionally skips their validation).
  double number = PyFloat_AsDouble(value.ptr());
  if (PyErr_Occurred()) {
    if (!PyErr_ExceptionMatches(PyExc_TypeError)) throw py::error_already_set();
    PyErr_Clear();
    auto error = py::module_::import("struct").attr("error");
    PyErr_SetString(error.ptr(), "required argument is not a float");
    throw py::error_already_set();
  }
  const float result = static_cast<float>(number);
  if (std::isfinite(number) && std::isinf(result))
    throw std::overflow_error("float too large to pack with f format");
  return result;
}

template <class T>
T Unsigned(py::handle value, bool force = false) {
  py::int_ zero(0),
      maximum(static_cast<uint64_t>(std::numeric_limits<T>::max()));
  int above = PyObject_RichCompareBool(value.ptr(), zero.ptr(), Py_GE);
  if (above < 0) throw py::error_already_set();
  int below = PyObject_RichCompareBool(value.ptr(), maximum.ptr(), Py_LE);
  if (below < 0) throw py::error_already_set();
  if (!above || !below)
    throw py::type_error("BodyCmd unsigned field outside range");
  // Python's builder accepts a floating zero when the default is elided.
  int is_zero = PyObject_RichCompareBool(value.ptr(), zero.ptr(), Py_EQ);
  if (is_zero < 0) throw py::error_already_set();
  if (!force && is_zero) return 0;
  py::object index =
      py::reinterpret_steal<py::object>(PyNumber_Index(value.ptr()));
  if (!index) {
    if (!PyErr_ExceptionMatches(PyExc_TypeError)) throw py::error_already_set();
    PyErr_Clear();
    auto error = py::module_::import("struct").attr("error");
    PyErr_SetString(error.ptr(), "required argument is not an integer");
    throw py::error_already_set();
  }
  auto result = PyLong_AsUnsignedLongLong(index.ptr());
  if (PyErr_Occurred()) throw py::error_already_set();
  return static_cast<T>(result);
}

py::tuple Encode(py::object command, uint64_t stamp, py::object node,
                 uint64_t header_sequence, bool measure) {
  const auto begin = measure ? Clock::now() : Stamp{};
  py::object joints = command.attr("joints");
  if (!py::isinstance<py::tuple>(joints) || py::len(joints) != 31)
    throw py::value_error("BodyCmd requires a 31-slot tuple");
  std::vector<lowlevel::BodyJointCmd> values;
  values.reserve(31);
  for (auto joint : joints) {
    std::array<float, 5> row;
    const char* names[] = {"q", "dq", "tau", "kp", "kd"};
    for (size_t i = 0; i < row.size(); ++i)
      row[i] = Float(joint.attr(names[i]));
    values.emplace_back(row[0], row[1], row[2], row[3], row[4]);
  }
  const auto protocol = Unsigned<uint32_t>(command.attr("protocol_version"));
  std::array<uint64_t, 6> fields;
  const char* names[] = {"session_id",           "sequence",
                         "input_state_id",       "input_state_ts_real_us",
                         "ts_client_created_us", "ts_client_send_us"};
  for (size_t i = 0; i < fields.size(); ++i)
    fields[i] = Unsigned<uint64_t>(command.attr(names[i]));
  const auto action = Unsigned<uint8_t>(command.attr("action"), true);
  std::optional<std::array<float, 4>> quat;
  py::object root = command.attr("root_quat");
  if (!root.is_none()) {
    if (py::len(root) != 4)
      throw py::value_error("root_quat requires four values");
    quat.emplace();
    for (size_t i = 0; i < 4; ++i) (*quat)[i] = Float(root[py::int_(i)]);
  }
  std::string publisher_node = node.is_none() ? "" : node.cast<std::string>();
  const auto marshalled = measure ? Clock::now() : Stamp{};
  std::vector<uint8_t> encoded;
  Stamp start{}, finish{};
  bool gil_released = false;
  {
    py::gil_scoped_release release;
    if (measure) {
      start = Clock::now();
      gil_released = !PyGILState_Check();
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
    const auto vector = builder.CreateVectorOfStructs(values);
    lowlevel::BodyCmdBuilder body(builder);
    body.add_aorta_header(header_offset);
    body.add_joints(vector);
    body.add_protocol_version(protocol);
    body.add_session_id(fields[0]);
    body.add_sequence(fields[1]);
    body.add_input_state_id(fields[2]);
    body.add_input_state_ts_real_us(fields[3]);
    body.add_ts_client_created_us(fields[4]);
    body.add_ts_client_send_us(fields[5]);
    builder.ForceDefaults(true);
    body.add_action(static_cast<lowlevel::BodyCmdAction>(action));
    builder.ForceDefaults(false);
    if (quat) {
      lowlevel::RootQuatWxyz value((*quat)[0], (*quat)[1], (*quat)[2],
                                   (*quat)[3]);
      body.add_root_quat(&value);
    }
    builder.Finish(body.Finish());
    encoded.assign(builder.GetBufferPointer(),
                   builder.GetBufferPointer() + builder.GetSize());
    if (measure) finish = Clock::now();
  }
  const auto acquired = measure ? Clock::now() : Stamp{};
  py::bytes bytes(reinterpret_cast<const char*>(encoded.data()),
                  encoded.size());
  if (!measure) return py::make_tuple(bytes, py::none());
  const auto copied = Clock::now();
  py::dict timing;
  timing["marshal_ns"] = Ns(begin, marshalled);
  timing["encode_native_ns"] = Ns(start, finish);
  timing["encode_gil_reacquire_wall_ns"] = Ns(finish, acquired);
  timing["encode_return_bytes_ns"] = Ns(acquired, copied);
  timing["gil_released_during_encode"] = gil_released;
  return py::make_tuple(bytes, timing);
}
}  // namespace

PYBIND11_MODULE(_body_cmd_native, module) {
  BindBodyState(module);
  module.attr("BODY_SCHEMA_SHA256") =
      "c98a531aad2b43206048753b82d0bacec1ad65d2429536432766975cd211e71c";
  module.def("encode", &Encode, py::arg("command"), py::arg("publish_stamp_ns"),
             py::arg("publisher_node"), py::arg("header_sequence"),
             py::arg("measure") = false);
}
