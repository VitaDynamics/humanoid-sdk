#include "end_effector_decoder.h"

#include <cstddef>
#include <cstdint>
#include <stdexcept>

#include "body_cmd_vendor/message/schemas/topic/lowlevel/end_effector_state_generated.h"
#include "body_cmd_vendor/message/schemas/topic/lowlevel/end_effector_touch_generated.h"

namespace py = pybind11;
namespace {

py::bytes OwnInput(const py::buffer& input) {
  // Immutable bytes retain their owner without a copy. Borrowed/mutable input
  // is snapshotted before any DTO constructor can re-enter Python.
  auto owned = py::reinterpret_steal<py::bytes>(PyObject_Bytes(input.ptr()));
  if (!owned) throw py::error_already_set();
  return owned;
}

template <class Table>
const Table* CheckedView(const py::bytes& owned, size_t offset) {
  const auto* data =
      reinterpret_cast<const uint8_t*>(PyBytes_AS_STRING(owned.ptr()));
  const auto size = static_cast<size_t>(PyBytes_GET_SIZE(owned.ptr()));
  if (size >= FLATBUFFERS_MAX_BUFFER_SIZE)
    throw std::invalid_argument("invalid end-effector buffer size");
  if (offset == 0) {
    if (size < sizeof(flatbuffers::uoffset_t))
      throw std::invalid_argument("invalid end-effector root offset");
    offset = flatbuffers::ReadScalar<flatbuffers::uoffset_t>(data);
    if (offset == 0)
      throw std::invalid_argument("invalid end-effector root offset");
  }
  // Match Body's embedded-view contract; the supplied table may not be the
  // root.
  if (offset > size || size - offset < sizeof(flatbuffers::soffset_t) ||
      offset % alignof(flatbuffers::soffset_t))
    throw std::invalid_argument("invalid end-effector table offset");
  const auto* view = reinterpret_cast<const Table*>(data + offset);
  flatbuffers::Verifier verifier(data, size);
  if (!verifier.VerifyTable(view))
    throw std::invalid_argument("invalid end-effector FlatBuffer");
  return view;
}

template <class Vector, class Getter>
py::tuple Column(const Vector* values, Getter get) {
  py::tuple result(values->size());
  for (flatbuffers::uoffset_t i = 0; i < values->size(); ++i)
    result[i] = py::cast(get(values->Get(i)));
  return result;
}

py::object DecodeState(const py::buffer& input, size_t offset,
                       const py::object& state_type) {
  const auto owned = OwnInput(input);
  const auto* v = CheckedView<lowlevel::ee::EndEffectorState>(owned, offset);
  if (v->protocol_version() != 2)
    throw std::invalid_argument(
        "incompatible EndEffectorState protocol_version");
  const auto* axes = v->axes();
  if (!axes || axes->size() == 0)
    throw std::invalid_argument("EndEffectorState requires fixed axes");
  return state_type(
      py::arg("runtime_id") = v->runtime_id(),
      py::arg("layout_id") = v->layout_id(), py::arg("side") = v->side(),
      py::arg("state_id") = v->state_id(),
      py::arg("protocol_version") = v->protocol_version(),
      py::arg("ts_state_real_us") = v->ts_state_real_us(),
      py::arg("ts_state_pub_us") = v->ts_state_pub_us(),
      py::arg("ts_acquired_hw_us") = v->ts_acquired_hw_us(),
      py::arg("ts_ingested_us") = v->ts_ingested_us(),
      py::arg("ts_assembled_us") = v->ts_assembled_us(),
      py::arg("sample_status") = v->sample_status(),
      py::arg("valid_fields") =
          Column(axes, [](const auto* a) { return a->valid_fields(); }),
      py::arg("pos") = Column(axes, [](const auto* a) { return a->pos(); }),
      py::arg("vel") = Column(axes, [](const auto* a) { return a->vel(); }),
      py::arg("tau") = Column(axes, [](const auto* a) { return a->tau(); }),
      py::arg("current") =
          Column(axes, [](const auto* a) { return a->current(); }),
      py::arg("temperature_c") =
          Column(axes, [](const auto* a) { return a->temperature_c(); }),
      py::arg("error_code") =
          Column(axes, [](const auto* a) { return a->error_code(); }),
      py::arg("ready") = v->ready(),
      py::arg("output_allowed") = v->output_allowed(),
      py::arg("inhibit_reason") = v->inhibit_reason(),
      py::arg("touch_enabled") = v->touch_enabled(),
      py::arg("status_valid_mask") = v->status_valid_mask());
}

py::object DecodeTouch(const py::buffer& input, size_t offset,
                       const py::object& touch_type,
                       const py::object& stamp_type) {
  const auto owned = OwnInput(input);
  const auto* v = CheckedView<lowlevel::ee::EndEffectorTouch>(owned, offset);
  const auto* stamps = v->region_stamps();
  py::tuple regions(stamps ? stamps->size() : 0);
  for (flatbuffers::uoffset_t i = 0; stamps && i < stamps->size(); ++i) {
    const auto* s = stamps->Get(i);
    regions[i] =
        stamp_type(s->sample_seq(), s->ts_request_us(), s->ts_acquired_hw_us(),
                   s->ts_source_us(), s->ts_ingested_us(), s->status_flags(),
                   s->direction_flags(), s->validity());
  }
  const auto* payload = v->payload();
  // One bulk copy into independently owned bytes; no per-byte Python integers
  // or temporary tuple, and no borrowed transport buffer escapes this call.
  py::bytes bytes =
      payload ? py::bytes(reinterpret_cast<const char*>(payload->Data()),
                          payload->size())
              : py::bytes("");
  // The existing validate_touch retains the protocol/identity/layout gate.
  return touch_type(
      py::arg("runtime_id") = v->runtime_id(),
      py::arg("layout_id") = v->layout_id(), py::arg("side") = v->side(),
      py::arg("touch_state_id") = v->touch_state_id(),
      py::arg("region_stamps") = regions, py::arg("payload") = bytes,
      py::arg("protocol_version") = v->protocol_version(),
      py::arg("ts_state_pub_us") = v->ts_state_pub_us(),
      py::arg("publish_time_valid") = v->publish_time_valid());
}

}  // namespace

void BindEndEffectorFeedback(py::module_& module) {
  const auto types = py::module_::import("locomotion_aorta.end_effector_types");
  module.attr("EE_STATE_SCHEMA_SHA256") = EE_STATE_SCHEMA_SHA256;
  module.attr("EE_TOUCH_SCHEMA_SHA256") = EE_TOUCH_SCHEMA_SHA256;
  module.attr("EE_STATE_PROTOCOL_VERSION") = 2;
  module.attr("EE_TOUCH_PROTOCOL_VERSION") = 1;
  module.def(
      "decode_end_effector_state",
      [type = types.attr("EndEffectorState")](py::buffer input, size_t offset) {
        return DecodeState(input, offset, type);
      },
      py::arg("payload"), py::arg("table_offset") = 0);
  module.def(
      "decode_end_effector_touch",
      [type = types.attr("EndEffectorTouch"),
       stamp = types.attr("RegionStamp")](py::buffer input, size_t offset) {
        return DecodeTouch(input, offset, type, stamp);
      },
      py::arg("payload"), py::arg("table_offset") = 0);
}
