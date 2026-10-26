#include "body_state_decoder.h"

#include <pybind11/numpy.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <vector>

#include "body_cmd_vendor/body_state_generated.h"

namespace py = pybind11;
namespace {

struct Imu {
  uint8_t location, status;
  uint64_t sampled, acquired, ingested, assembled;
  std::optional<lowlevel::BodyImuState> state;
};

struct State {
  uint32_t protocol, roster;
  uint64_t id, real, pub, consumed, session, sequence, output;
  bool attitude, key_valid;
  uint64_t control;
  uint8_t source_type;
  std::optional<lowlevel::ee::DescriptorRef> descriptor;
  std::vector<lowlevel::BodyJointState> joints;
  std::vector<Imu> imus;
};

template <uint16_t N>
py::tuple Floats(const flatbuffers::Array<float, N> *values) {
  py::tuple result(N);
  for (uint16_t i = 0; i < N; ++i) result[i] = values->Get(i);
  return result;
}

State ReadState(const uint8_t *data, size_t size, size_t offset) {
  if (offset == 0) {
    if (size < sizeof(flatbuffers::uoffset_t))
      throw std::invalid_argument("invalid BodyState root offset");
    offset = flatbuffers::ReadScalar<flatbuffers::uoffset_t>(data);
    if (offset == 0)
      throw std::invalid_argument("invalid BodyState root offset");
  }
  State result;
  // A generated Python view can refer to an embedded table. Verify that
  // exact table offset rather than assuming the root starts at byte zero.
  if (size >= FLATBUFFERS_MAX_BUFFER_SIZE || offset > size ||
      size - offset < sizeof(flatbuffers::soffset_t) ||
      offset % alignof(flatbuffers::soffset_t))
    throw std::invalid_argument("invalid BodyState table offset");
  flatbuffers::Verifier verifier(data, size);
  const auto *view =
      reinterpret_cast<const lowlevel::BodyState *>(data + offset);
  if (!verifier.VerifyTable(view))
    throw std::invalid_argument("invalid BodyState FlatBuffer");
  if (view->protocol_version() != 3)
    throw std::invalid_argument(
        "incompatible BodyState protocol_version; expected BodyState 3");
  result.protocol = view->protocol_version();
  result.roster = view->roster_version();
  result.id = view->state_id();
  result.real = view->ts_state_real_us();
  result.pub = view->ts_state_pub_us();
  result.consumed = view->ts_consumed_us();
  result.attitude = view->attitude_valid();
  result.session = view->applied_session_id();
  result.sequence = view->applied_sequence();
  result.output = view->ts_first_output_us();
  result.control = view->ts_control_us();
  result.key_valid = view->applied_key_valid();
  result.source_type = view->association_source_type();
  if (view->descriptor_ref()) result.descriptor = *view->descriptor_ref();
  if (const auto *joints = view->joints()) {
    if (joints->size() != 31)
      throw std::invalid_argument(
          "body vector requires exactly 31 joint slots");
    result.joints.reserve(31);
    for (const auto *joint : *joints) result.joints.push_back(*joint);
  }
  if (const auto *imus = view->imu_states()) {
    result.imus.reserve(imus->size());
    for (const auto *sample : *imus) {
      Imu imu{static_cast<uint8_t>(sample->location()),
              static_cast<uint8_t>(sample->status()),
              sample->ts_sampled_us(),
              sample->ts_acquired_hw_us(),
              sample->ts_ingested_us(),
              sample->ts_assembled_us(),
              std::nullopt};
      if (sample->state()) imu.state = *sample->state();
      result.imus.push_back(imu);
    }
  }
  return result;
}

py::object Descriptor(const State &state, const py::object &descriptor_type) {
  if (!state.descriptor) return py::none();
  const auto runtime = state.descriptor->robot_runtime_id();
  if (!runtime)
    throw std::invalid_argument(
        "BodyState DescriptorRef requires nonzero robot_runtime_id");
  return descriptor_type(runtime);
}

py::object Decode(py::buffer input, size_t offset, const py::object &state_type,
                  const py::object &joint_type, const py::object &imu_type,
                  const py::object &sample_type, const py::object &descriptor_type) {
  // bytes is retained without copying. Mutable/borrowed buffers are copied
  // before decoding, so results never borrow mutable caller storage.
  py::bytes owned =
      py::reinterpret_steal<py::bytes>(PyObject_Bytes(input.ptr()));
  if (!owned) throw py::error_already_set();
  auto *data =
      reinterpret_cast<const uint8_t *>(PyBytes_AS_STRING(owned.ptr()));
  const auto size = static_cast<size_t>(PyBytes_GET_SIZE(owned.ptr()));
  State result = ReadState(data, size, offset);
  py::tuple joints(result.joints.size());
  for (size_t i = 0; i < result.joints.size(); ++i) {
    const auto &j = result.joints[i];
    joints[i] = joint_type(j.ts_acquired_hw_us(), j.ts_ingested_us(),
                           j.ts_assembled_us(), j.q(), j.dq(), j.tau_est(),
                           static_cast<uint32_t>(j.error_code()));
  }
  py::tuple imus(result.imus.size());
  for (size_t i = 0; i < result.imus.size(); ++i) {
    const auto &imu = result.imus[i];
    py::object state = py::none();
    if (imu.state) {
      state = imu_type(
          Floats(imu.state->quaternion()), Floats(imu.state->gyroscope()),
          Floats(imu.state->accelerometer()), Floats(imu.state->rpy()));
    }
    imus[i] = sample_type(imu.location, state, imu.status, imu.sampled,
                          imu.acquired, imu.ingested, imu.assembled);
  }
  py::object dto =
      state_type(result.id, joints, result.protocol, result.roster, result.real,
                 result.pub, result.consumed, result.attitude, imus,
                 result.session, result.sequence, result.output,
                 Descriptor(result, descriptor_type), result.control,
                 result.key_valid, result.source_type);
  return dto;
}

struct Arrays {
  State state;
  std::vector<float> q, dq, tau, imu;
  std::vector<uint64_t> joint_stamps, imu_stamps;
  std::vector<uint32_t> errors;
  std::vector<uint8_t> locations, status, present;
};

template <class T>
py::array View(std::vector<T> &data, py::ssize_t columns,
               const py::capsule &owner) {
  py::array result =
      columns == 1
          ? py::array(py::dtype::of<T>(),
                      {static_cast<py::ssize_t>(data.size())},
                      {static_cast<py::ssize_t>(sizeof(T))}, data.data(), owner)
          : py::array(
                py::dtype::of<T>(),
                {static_cast<py::ssize_t>(data.size() / columns), columns},
                {static_cast<py::ssize_t>(columns * sizeof(T)),
                 static_cast<py::ssize_t>(sizeof(T))},
                data.data(), owner);
  result.attr("setflags")(false);
  return result;
}

py::dict DecodeArrays(py::buffer input, size_t offset,
                      const py::object &descriptor_type) {
  py::bytes owned =
      py::reinterpret_steal<py::bytes>(PyObject_Bytes(input.ptr()));
  if (!owned) throw py::error_already_set();
  auto *data =
      reinterpret_cast<const uint8_t *>(PyBytes_AS_STRING(owned.ptr()));
  const auto size = static_cast<size_t>(PyBytes_GET_SIZE(owned.ptr()));
  auto arrays = std::make_unique<Arrays>();
  arrays->state = ReadState(data, size, offset);
  const auto joint_count = arrays->state.joints.size();
  arrays->q.reserve(joint_count);
  arrays->dq.reserve(joint_count);
  arrays->tau.reserve(joint_count);
  arrays->errors.reserve(joint_count);
  arrays->joint_stamps.reserve(joint_count * 3);
  const auto imu_count = arrays->state.imus.size();
  arrays->imu.reserve(imu_count * 13);
  arrays->imu_stamps.reserve(imu_count * 4);
  arrays->locations.reserve(imu_count);
  arrays->status.reserve(imu_count);
  arrays->present.reserve(imu_count);
  for (const auto &j : arrays->state.joints) {
    arrays->q.push_back(j.q());
    arrays->dq.push_back(j.dq());
    arrays->tau.push_back(j.tau_est());
    arrays->joint_stamps.insert(
        arrays->joint_stamps.end(),
        {j.ts_acquired_hw_us(), j.ts_ingested_us(), j.ts_assembled_us()});
    arrays->errors.push_back(static_cast<uint32_t>(j.error_code()));
  }
  for (const auto &i : arrays->state.imus) {
    arrays->locations.push_back(i.location);
    arrays->status.push_back(i.status);
    arrays->present.push_back(i.state.has_value());
    arrays->imu_stamps.insert(arrays->imu_stamps.end(),
                              {i.sampled, i.acquired, i.ingested, i.assembled});
    if (i.state) {
      for (uint16_t j = 0; j < 4; ++j)
        arrays->imu.push_back(i.state->quaternion()->Get(j));
      for (auto *values :
           {i.state->gyroscope(), i.state->accelerometer(), i.state->rpy()})
        for (uint16_t j = 0; j < 3; ++j) arrays->imu.push_back(values->Get(j));
    } else {
      arrays->imu.insert(arrays->imu.end(), 13, 0.0f);
    }
  }
  // All memory is independently owned; NumPy wrappers hold the capsule, not
  // a transport cache or the borrowed input.
  auto *a = arrays.get();
  py::capsule owner(a, [](void *p) { delete static_cast<Arrays *>(p); });
  arrays.release();
  const auto &v = a->state;
  py::dict out;
  out["q"] = View(a->q, 1, owner);
  out["dq"] = View(a->dq, 1, owner);
  out["tau_est"] = View(a->tau, 1, owner);
  out["joint_stamps_us"] = View(a->joint_stamps, 3, owner);
  out["error_code"] = View(a->errors, 1, owner);
  out["imu_values"] = View(a->imu, 13, owner);
  out["imu_stamps_us"] = View(a->imu_stamps, 4, owner);
  out["imu_location"] = View(a->locations, 1, owner);
  out["imu_status"] = View(a->status, 1, owner);
  out["imu_state_present"] = View(a->present, 1, owner);
  out["state_id"] = v.id;
  out["protocol_version"] = v.protocol;
  out["roster_version"] = v.roster;
  out["ts_state_real_us"] = v.real;
  out["ts_state_pub_us"] = v.pub;
  out["ts_consumed_us"] = v.consumed;
  out["attitude_valid"] = v.attitude;
  out["applied_session_id"] = v.session;
  out["applied_sequence"] = v.sequence;
  out["ts_first_output_us"] = v.output;
  out["descriptor_ref"] = Descriptor(v, descriptor_type);
  out["ts_control_us"] = v.control;
  out["applied_key_valid"] = v.key_valid;
  out["association_source_type"] = v.source_type;
  return out;
}
}  // namespace

void BindBodyState(py::module_ &module) {
  auto types = py::module_::import("locomotion_aorta.body_types");
  module.def(
      "decode_state_arrays",
      [descriptor = types.attr("DescriptorRef")](py::buffer input, size_t offset) {
        return DecodeArrays(input, offset, descriptor);
      },
      py::arg("payload"), py::arg("table_offset") = 0);
  module.attr("BODY_STATE_SCHEMA_SHA256") = BODY_STATE_SCHEMA_SHA256;
  module.attr("BODY_STATE_PROTOCOL_VERSION") = 3;
  module.def(
      "decode_state",
      [state = types.attr("BodyState"), joint = types.attr("BodyJointState"),
       imu = types.attr("BodyImuState"),
       sample = types.attr("BodyImuSample"),
       descriptor = types.attr("DescriptorRef")](py::buffer input, size_t offset) {
        return Decode(input, offset, state, joint, imu, sample, descriptor);
      },
      py::arg("payload"), py::arg("table_offset") = 0);
}
