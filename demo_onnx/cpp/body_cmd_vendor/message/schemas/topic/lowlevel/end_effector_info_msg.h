#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "aorta/array_view.h"
#include "aorta/typed_subscribe.h"
// IWYU pragma: begin_exports
#include "end_effector_info_generated.h"
#include "end_effector_info_schema_meta.h"
// IWYU pragma: end_exports

namespace aorta {
template <typename T>
struct TypedPublishTraits;
}  // namespace aorta

namespace lowlevel {
namespace ee {

struct ErrorInfoMsg;
struct AortaHeaderMsg;
struct EndEffectorCapabilityMsg;
struct EndEffectorInfoMsg;
struct TouchRegionInfoMsg;

#ifndef AORTA_TYPED_WRAPPER_TABLE_MSG_LOWLEVEL_EE_ERRORINFOMSG_DEFINED
#define AORTA_TYPED_WRAPPER_TABLE_MSG_LOWLEVEL_EE_ERRORINFOMSG_DEFINED 1
struct ErrorInfoMsg {
  std::optional<std::int32_t> code;
  std::optional<std::string> message;
  std::optional<std::string> details;
  std::optional<std::uint64_t> timestamp_ns;
};
#endif  // AORTA_TYPED_WRAPPER_TABLE_MSG_LOWLEVEL_EE_ERRORINFOMSG_DEFINED

#ifndef AORTA_TYPED_WRAPPER_TABLE_MSG_LOWLEVEL_EE_AORTAHEADERMSG_DEFINED
#define AORTA_TYPED_WRAPPER_TABLE_MSG_LOWLEVEL_EE_AORTAHEADERMSG_DEFINED 1
struct AortaHeaderMsg {
  std::optional<std::uint64_t> publish_stamp_ns;
  std::optional<std::string> publisher_node;
  std::optional<std::uint64_t> sequence;
};
#endif  // AORTA_TYPED_WRAPPER_TABLE_MSG_LOWLEVEL_EE_AORTAHEADERMSG_DEFINED

#ifndef AORTA_TYPED_WRAPPER_TABLE_MSG_LOWLEVEL_EE_ENDEFFECTORCAPABILITYMSG_DEFINED
#define AORTA_TYPED_WRAPPER_TABLE_MSG_LOWLEVEL_EE_ENDEFFECTORCAPABILITYMSG_DEFINED 1
struct EndEffectorCapabilityMsg {
  std::optional<::lowlevel::ee::EndEffectorSide> side;
  std::optional<bool> is_mock;
  std::optional<std::shared_ptr<EndEffectorInfoMsg>> info;
  std::optional<bool> ready;
  std::optional<::aorta::services::common::ServiceStatus> status;
  std::optional<std::string> reason;
};
#endif  // AORTA_TYPED_WRAPPER_TABLE_MSG_LOWLEVEL_EE_ENDEFFECTORCAPABILITYMSG_DEFINED

#ifndef AORTA_TYPED_WRAPPER_TABLE_MSG_LOWLEVEL_EE_ENDEFFECTORINFOMSG_DEFINED
#define AORTA_TYPED_WRAPPER_TABLE_MSG_LOWLEVEL_EE_ENDEFFECTORINFOMSG_DEFINED 1
struct EndEffectorInfoMsg {
  std::optional<std::uint32_t> protocol_version;
  std::optional<std::uint64_t> runtime_id;
  std::optional<std::uint32_t> layout_id;
  std::optional<std::uint8_t> side;
  std::optional<std::uint16_t> device_model;
  std::optional<std::string> firmware;
  std::optional<std::string> serial;
  std::optional<std::uint8_t> identity_valid_mask;
  std::optional<std::vector<std::string>> axis_names;
  std::optional<std::vector<std::uint8_t>> position_domain;
  std::optional<std::vector<std::uint8_t>> torque_domain;
  std::optional<std::vector<std::uint32_t>> supported_modes;
  std::optional<std::vector<std::uint32_t>> supported_command_fields;
  std::optional<std::vector<std::uint32_t>> supported_state_fields;
  std::optional<std::vector<std::uint32_t>> valid_limits;
  std::optional<std::vector<float>> pos_min;
  std::optional<std::vector<float>> pos_max;
  std::optional<std::vector<float>> vel_min;
  std::optional<std::vector<float>> vel_max;
  std::optional<std::vector<float>> vel_limit_max;
  std::optional<std::vector<float>> tau_min;
  std::optional<std::vector<float>> tau_max;
  std::optional<std::vector<float>> current_min;
  std::optional<std::vector<float>> current_max;
  std::optional<bool> touch_available;
  std::optional<std::vector<std::shared_ptr<TouchRegionInfoMsg>>> touch_regions;
  std::optional<std::vector<bool>> is_mock;
  std::optional<std::uint64_t> ts_descriptor_acquired_us;
  std::optional<std::uint64_t> ts_state_pub_us;
  std::optional<std::uint8_t> time_valid_mask;
};
#endif  // AORTA_TYPED_WRAPPER_TABLE_MSG_LOWLEVEL_EE_ENDEFFECTORINFOMSG_DEFINED

#ifndef AORTA_TYPED_WRAPPER_TABLE_MSG_LOWLEVEL_EE_TOUCHREGIONINFOMSG_DEFINED
#define AORTA_TYPED_WRAPPER_TABLE_MSG_LOWLEVEL_EE_TOUCHREGIONINFOMSG_DEFINED 1
struct TouchRegionInfoMsg {
  std::optional<std::uint16_t> region_id;
  std::optional<std::string> name;
  std::optional<std::string> frame;
  std::optional<std::uint16_t> sensor_type;
  std::optional<std::vector<std::uint32_t>> shape;
  std::optional<std::vector<std::string>> components;
  std::optional<std::uint8_t> dtype;
  std::optional<std::string> unit;
  std::optional<std::uint32_t> payload_offset;
  std::optional<std::uint32_t> payload_bytes;
  std::optional<std::uint16_t> encoding_version;
  std::optional<std::vector<float>> geometry;
  std::optional<bool> geometry_valid;
};
#endif  // AORTA_TYPED_WRAPPER_TABLE_MSG_LOWLEVEL_EE_TOUCHREGIONINFOMSG_DEFINED

}  // namespace ee
}  // namespace lowlevel

namespace aorta {
namespace detail {

#ifndef AORTA_DETAIL_SCRATCH_VECTOR_DEFINED
#define AORTA_DETAIL_SCRATCH_VECTOR_DEFINED 1
template <typename T>
class ScratchVector {
 public:
  ScratchVector() : data_(Take()) { data_.clear(); }
  ~ScratchVector() { Pool().push_back(std::move(data_)); }
  std::vector<T>& get() { return data_; }

 private:
  static std::vector<T> Take() {
    auto& pool = Pool();
    if (pool.empty()) {
      return {};
    }
    auto out = std::move(pool.back());
    pool.pop_back();
    return out;
  }

  static std::vector<std::vector<T>>& Pool() {
    thread_local std::vector<std::vector<T>> pool;
    return pool;
  }

  std::vector<T> data_;
};
#endif  // AORTA_DETAIL_SCRATCH_VECTOR_DEFINED

namespace typed_lowlevel_ee_end_effector_info {

inline flatbuffers::Offset<aorta::services::common::ErrorInfo> BuildErrorInfoTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::lowlevel::ee::ErrorInfoMsg& msg,
    flatbuffers::Offset<aorta::sys::AortaHeader> header);
inline flatbuffers::Offset<aorta::sys::AortaHeader> BuildAortaHeaderTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::lowlevel::ee::AortaHeaderMsg& msg,
    flatbuffers::Offset<aorta::sys::AortaHeader> header);
inline flatbuffers::Offset<lowlevel::ee::EndEffectorCapability> BuildEndEffectorCapabilityTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::lowlevel::ee::EndEffectorCapabilityMsg& msg,
    flatbuffers::Offset<aorta::sys::AortaHeader> header);
inline flatbuffers::Offset<lowlevel::ee::EndEffectorInfo> BuildEndEffectorInfoTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::lowlevel::ee::EndEffectorInfoMsg& msg,
    flatbuffers::Offset<aorta::sys::AortaHeader> header);
inline flatbuffers::Offset<lowlevel::ee::TouchRegionInfo> BuildTouchRegionInfoTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::lowlevel::ee::TouchRegionInfoMsg& msg,
    flatbuffers::Offset<aorta::sys::AortaHeader> header);

inline flatbuffers::Offset<aorta::services::common::ErrorInfo> BuildErrorInfoTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::lowlevel::ee::ErrorInfoMsg& msg,
    flatbuffers::Offset<aorta::sys::AortaHeader> header) {
  flatbuffers::Offset<flatbuffers::String> message_offset;
  if (msg.message.has_value()) {
    message_offset = builder.CreateString(msg.message.value());
  }
  flatbuffers::Offset<flatbuffers::String> details_offset;
  if (msg.details.has_value()) {
    details_offset = builder.CreateString(msg.details.value());
  }
  ::aorta::services::common::ErrorInfoBuilder root_builder(builder);
  (void)header;
  if (msg.code.has_value()) {
    root_builder.add_code(msg.code.value());
  }
  if (message_offset.o != 0) {
    root_builder.add_message(message_offset);
  }
  if (details_offset.o != 0) {
    root_builder.add_details(details_offset);
  }
  if (msg.timestamp_ns.has_value()) {
    root_builder.add_timestamp_ns(msg.timestamp_ns.value());
  }
  return root_builder.Finish();
}

inline flatbuffers::Offset<aorta::sys::AortaHeader> BuildAortaHeaderTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::lowlevel::ee::AortaHeaderMsg& msg,
    flatbuffers::Offset<aorta::sys::AortaHeader> header) {
  flatbuffers::Offset<flatbuffers::String> publisher_node_offset;
  if (msg.publisher_node.has_value()) {
    publisher_node_offset = builder.CreateString(msg.publisher_node.value());
  }
  ::aorta::sys::AortaHeaderBuilder root_builder(builder);
  (void)header;
  if (msg.publish_stamp_ns.has_value()) {
    root_builder.add_publish_stamp_ns(msg.publish_stamp_ns.value());
  }
  if (publisher_node_offset.o != 0) {
    root_builder.add_publisher_node(publisher_node_offset);
  }
  if (msg.sequence.has_value()) {
    root_builder.add_sequence(msg.sequence.value());
  }
  return root_builder.Finish();
}

inline flatbuffers::Offset<lowlevel::ee::EndEffectorCapability> BuildEndEffectorCapabilityTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::lowlevel::ee::EndEffectorCapabilityMsg& msg,
    flatbuffers::Offset<aorta::sys::AortaHeader> header) {
  flatbuffers::Offset<::lowlevel::ee::EndEffectorInfo> info_offset;
  if (msg.info.has_value()) {
    if (msg.info.value()) {
      info_offset = BuildEndEffectorInfoTyped(builder, *msg.info.value(), {});
    }
  }
  flatbuffers::Offset<flatbuffers::String> reason_offset;
  if (msg.reason.has_value()) {
    reason_offset = builder.CreateString(msg.reason.value());
  }
  ::lowlevel::ee::EndEffectorCapabilityBuilder root_builder(builder);
  (void)header;
  if (msg.side.has_value()) {
    root_builder.add_side(msg.side.value());
  }
  if (msg.is_mock.has_value()) {
    root_builder.add_is_mock(msg.is_mock.value());
  }
  if (info_offset.o != 0) {
    root_builder.add_info(info_offset);
  }
  if (msg.ready.has_value()) {
    root_builder.add_ready(msg.ready.value());
  }
  if (msg.status.has_value()) {
    root_builder.add_status(msg.status.value());
  }
  if (reason_offset.o != 0) {
    root_builder.add_reason(reason_offset);
  }
  return root_builder.Finish();
}

inline flatbuffers::Offset<lowlevel::ee::EndEffectorInfo> BuildEndEffectorInfoTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::lowlevel::ee::EndEffectorInfoMsg& msg,
    flatbuffers::Offset<aorta::sys::AortaHeader> header) {
  flatbuffers::Offset<flatbuffers::String> firmware_offset;
  if (msg.firmware.has_value()) {
    firmware_offset = builder.CreateString(msg.firmware.value());
  }
  flatbuffers::Offset<flatbuffers::String> serial_offset;
  if (msg.serial.has_value()) {
    serial_offset = builder.CreateString(msg.serial.value());
  }
  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>>> axis_names_offset;
  if (msg.axis_names.has_value()) {
    ::aorta::detail::ScratchVector<flatbuffers::Offset<flatbuffers::String>> offsets_scratch;
    auto& offsets = offsets_scratch.get();
    offsets.reserve(msg.axis_names.value().size());
    for (const auto& item : msg.axis_names.value()) {
      offsets.push_back(builder.CreateString(item));
    }
    axis_names_offset = builder.CreateVector(offsets);
  }
  flatbuffers::Offset<flatbuffers::Vector<std::uint8_t>> position_domain_offset;
  if (msg.position_domain.has_value()) {
    position_domain_offset = builder.CreateVector(msg.position_domain.value().data(), msg.position_domain.value().size());
  }
  flatbuffers::Offset<flatbuffers::Vector<std::uint8_t>> torque_domain_offset;
  if (msg.torque_domain.has_value()) {
    torque_domain_offset = builder.CreateVector(msg.torque_domain.value().data(), msg.torque_domain.value().size());
  }
  flatbuffers::Offset<flatbuffers::Vector<std::uint32_t>> supported_modes_offset;
  if (msg.supported_modes.has_value()) {
    supported_modes_offset = builder.CreateVector(msg.supported_modes.value().data(), msg.supported_modes.value().size());
  }
  flatbuffers::Offset<flatbuffers::Vector<std::uint32_t>> supported_command_fields_offset;
  if (msg.supported_command_fields.has_value()) {
    supported_command_fields_offset = builder.CreateVector(msg.supported_command_fields.value().data(), msg.supported_command_fields.value().size());
  }
  flatbuffers::Offset<flatbuffers::Vector<std::uint32_t>> supported_state_fields_offset;
  if (msg.supported_state_fields.has_value()) {
    supported_state_fields_offset = builder.CreateVector(msg.supported_state_fields.value().data(), msg.supported_state_fields.value().size());
  }
  flatbuffers::Offset<flatbuffers::Vector<std::uint32_t>> valid_limits_offset;
  if (msg.valid_limits.has_value()) {
    valid_limits_offset = builder.CreateVector(msg.valid_limits.value().data(), msg.valid_limits.value().size());
  }
  flatbuffers::Offset<flatbuffers::Vector<float>> pos_min_offset;
  if (msg.pos_min.has_value()) {
    pos_min_offset = builder.CreateVector(msg.pos_min.value().data(), msg.pos_min.value().size());
  }
  flatbuffers::Offset<flatbuffers::Vector<float>> pos_max_offset;
  if (msg.pos_max.has_value()) {
    pos_max_offset = builder.CreateVector(msg.pos_max.value().data(), msg.pos_max.value().size());
  }
  flatbuffers::Offset<flatbuffers::Vector<float>> vel_min_offset;
  if (msg.vel_min.has_value()) {
    vel_min_offset = builder.CreateVector(msg.vel_min.value().data(), msg.vel_min.value().size());
  }
  flatbuffers::Offset<flatbuffers::Vector<float>> vel_max_offset;
  if (msg.vel_max.has_value()) {
    vel_max_offset = builder.CreateVector(msg.vel_max.value().data(), msg.vel_max.value().size());
  }
  flatbuffers::Offset<flatbuffers::Vector<float>> vel_limit_max_offset;
  if (msg.vel_limit_max.has_value()) {
    vel_limit_max_offset = builder.CreateVector(msg.vel_limit_max.value().data(), msg.vel_limit_max.value().size());
  }
  flatbuffers::Offset<flatbuffers::Vector<float>> tau_min_offset;
  if (msg.tau_min.has_value()) {
    tau_min_offset = builder.CreateVector(msg.tau_min.value().data(), msg.tau_min.value().size());
  }
  flatbuffers::Offset<flatbuffers::Vector<float>> tau_max_offset;
  if (msg.tau_max.has_value()) {
    tau_max_offset = builder.CreateVector(msg.tau_max.value().data(), msg.tau_max.value().size());
  }
  flatbuffers::Offset<flatbuffers::Vector<float>> current_min_offset;
  if (msg.current_min.has_value()) {
    current_min_offset = builder.CreateVector(msg.current_min.value().data(), msg.current_min.value().size());
  }
  flatbuffers::Offset<flatbuffers::Vector<float>> current_max_offset;
  if (msg.current_max.has_value()) {
    current_max_offset = builder.CreateVector(msg.current_max.value().data(), msg.current_max.value().size());
  }
  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<::lowlevel::ee::TouchRegionInfo>>> touch_regions_offset;
  if (msg.touch_regions.has_value()) {
    ::aorta::detail::ScratchVector<flatbuffers::Offset<::lowlevel::ee::TouchRegionInfo>> offsets_scratch;
    auto& offsets = offsets_scratch.get();
    offsets.reserve(msg.touch_regions.value().size());
    for (const auto& item_ptr : msg.touch_regions.value()) {
      if (!item_ptr) {
        continue;
      }
      offsets.push_back(BuildTouchRegionInfoTyped(builder, *item_ptr, {}));
    }
    touch_regions_offset = builder.CreateVector(offsets);
  }
  flatbuffers::Offset<flatbuffers::Vector<uint8_t>> is_mock_offset;
  if (msg.is_mock.has_value()) {
    const auto& bool_values = msg.is_mock.value();
    uint8_t* values = nullptr;
    is_mock_offset = builder.CreateUninitializedVector<uint8_t>(bool_values.size(), &values);
    for (std::size_t i = 0; i < bool_values.size(); ++i) {
      values[i] = bool_values[i] ? 1 : 0;
    }
  }
  ::lowlevel::ee::EndEffectorInfoBuilder root_builder(builder);
  if (header.o != 0) {
    root_builder.add_aorta_header(header);
  }
  if (msg.protocol_version.has_value()) {
    root_builder.add_protocol_version(msg.protocol_version.value());
  }
  if (msg.runtime_id.has_value()) {
    root_builder.add_runtime_id(msg.runtime_id.value());
  }
  if (msg.layout_id.has_value()) {
    root_builder.add_layout_id(msg.layout_id.value());
  }
  if (msg.side.has_value()) {
    root_builder.add_side(msg.side.value());
  }
  if (msg.device_model.has_value()) {
    root_builder.add_device_model(msg.device_model.value());
  }
  if (firmware_offset.o != 0) {
    root_builder.add_firmware(firmware_offset);
  }
  if (serial_offset.o != 0) {
    root_builder.add_serial(serial_offset);
  }
  if (msg.identity_valid_mask.has_value()) {
    root_builder.add_identity_valid_mask(msg.identity_valid_mask.value());
  }
  if (axis_names_offset.o != 0) {
    root_builder.add_axis_names(axis_names_offset);
  }
  if (position_domain_offset.o != 0) {
    root_builder.add_position_domain(position_domain_offset);
  }
  if (torque_domain_offset.o != 0) {
    root_builder.add_torque_domain(torque_domain_offset);
  }
  if (supported_modes_offset.o != 0) {
    root_builder.add_supported_modes(supported_modes_offset);
  }
  if (supported_command_fields_offset.o != 0) {
    root_builder.add_supported_command_fields(supported_command_fields_offset);
  }
  if (supported_state_fields_offset.o != 0) {
    root_builder.add_supported_state_fields(supported_state_fields_offset);
  }
  if (valid_limits_offset.o != 0) {
    root_builder.add_valid_limits(valid_limits_offset);
  }
  if (pos_min_offset.o != 0) {
    root_builder.add_pos_min(pos_min_offset);
  }
  if (pos_max_offset.o != 0) {
    root_builder.add_pos_max(pos_max_offset);
  }
  if (vel_min_offset.o != 0) {
    root_builder.add_vel_min(vel_min_offset);
  }
  if (vel_max_offset.o != 0) {
    root_builder.add_vel_max(vel_max_offset);
  }
  if (vel_limit_max_offset.o != 0) {
    root_builder.add_vel_limit_max(vel_limit_max_offset);
  }
  if (tau_min_offset.o != 0) {
    root_builder.add_tau_min(tau_min_offset);
  }
  if (tau_max_offset.o != 0) {
    root_builder.add_tau_max(tau_max_offset);
  }
  if (current_min_offset.o != 0) {
    root_builder.add_current_min(current_min_offset);
  }
  if (current_max_offset.o != 0) {
    root_builder.add_current_max(current_max_offset);
  }
  if (msg.touch_available.has_value()) {
    root_builder.add_touch_available(msg.touch_available.value());
  }
  if (touch_regions_offset.o != 0) {
    root_builder.add_touch_regions(touch_regions_offset);
  }
  if (is_mock_offset.o != 0) {
    root_builder.add_is_mock(is_mock_offset);
  }
  if (msg.ts_descriptor_acquired_us.has_value()) {
    root_builder.add_ts_descriptor_acquired_us(msg.ts_descriptor_acquired_us.value());
  }
  if (msg.ts_state_pub_us.has_value()) {
    root_builder.add_ts_state_pub_us(msg.ts_state_pub_us.value());
  }
  if (msg.time_valid_mask.has_value()) {
    root_builder.add_time_valid_mask(msg.time_valid_mask.value());
  }
  return root_builder.Finish();
}

inline flatbuffers::Offset<lowlevel::ee::TouchRegionInfo> BuildTouchRegionInfoTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::lowlevel::ee::TouchRegionInfoMsg& msg,
    flatbuffers::Offset<aorta::sys::AortaHeader> header) {
  flatbuffers::Offset<flatbuffers::String> name_offset;
  if (msg.name.has_value()) {
    name_offset = builder.CreateString(msg.name.value());
  }
  flatbuffers::Offset<flatbuffers::String> frame_offset;
  if (msg.frame.has_value()) {
    frame_offset = builder.CreateString(msg.frame.value());
  }
  flatbuffers::Offset<flatbuffers::Vector<std::uint32_t>> shape_offset;
  if (msg.shape.has_value()) {
    shape_offset = builder.CreateVector(msg.shape.value().data(), msg.shape.value().size());
  }
  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>>> components_offset;
  if (msg.components.has_value()) {
    ::aorta::detail::ScratchVector<flatbuffers::Offset<flatbuffers::String>> offsets_scratch;
    auto& offsets = offsets_scratch.get();
    offsets.reserve(msg.components.value().size());
    for (const auto& item : msg.components.value()) {
      offsets.push_back(builder.CreateString(item));
    }
    components_offset = builder.CreateVector(offsets);
  }
  flatbuffers::Offset<flatbuffers::String> unit_offset;
  if (msg.unit.has_value()) {
    unit_offset = builder.CreateString(msg.unit.value());
  }
  flatbuffers::Offset<flatbuffers::Vector<float>> geometry_offset;
  if (msg.geometry.has_value()) {
    geometry_offset = builder.CreateVector(msg.geometry.value().data(), msg.geometry.value().size());
  }
  ::lowlevel::ee::TouchRegionInfoBuilder root_builder(builder);
  (void)header;
  if (msg.region_id.has_value()) {
    root_builder.add_region_id(msg.region_id.value());
  }
  if (name_offset.o != 0) {
    root_builder.add_name(name_offset);
  }
  if (frame_offset.o != 0) {
    root_builder.add_frame(frame_offset);
  }
  if (msg.sensor_type.has_value()) {
    root_builder.add_sensor_type(msg.sensor_type.value());
  }
  if (shape_offset.o != 0) {
    root_builder.add_shape(shape_offset);
  }
  if (components_offset.o != 0) {
    root_builder.add_components(components_offset);
  }
  if (msg.dtype.has_value()) {
    root_builder.add_dtype(msg.dtype.value());
  }
  if (unit_offset.o != 0) {
    root_builder.add_unit(unit_offset);
  }
  if (msg.payload_offset.has_value()) {
    root_builder.add_payload_offset(msg.payload_offset.value());
  }
  if (msg.payload_bytes.has_value()) {
    root_builder.add_payload_bytes(msg.payload_bytes.value());
  }
  if (msg.encoding_version.has_value()) {
    root_builder.add_encoding_version(msg.encoding_version.value());
  }
  if (geometry_offset.o != 0) {
    root_builder.add_geometry(geometry_offset);
  }
  if (msg.geometry_valid.has_value()) {
    root_builder.add_geometry_valid(msg.geometry_valid.value());
  }
  return root_builder.Finish();
}

}  // namespace typed_lowlevel_ee_end_effector_info
}  // namespace detail

template <>
struct TypedPublishTraits<::lowlevel::ee::EndEffectorInfo> {
  using Msg = ::lowlevel::ee::EndEffectorInfoMsg;
  static constexpr bool kHasAortaHeader = true;

  static flatbuffers::Offset<lowlevel::ee::EndEffectorInfo> Build(
      flatbuffers::FlatBufferBuilder& builder,
      flatbuffers::Offset<aorta::sys::AortaHeader> header,
      const Msg& msg) {
    return detail::typed_lowlevel_ee_end_effector_info::BuildEndEffectorInfoTyped(builder, msg, header);
  }
};

template <>
struct TypedSubscribeTraits<::lowlevel::ee::EndEffectorCapability> {
  static bool Verify(const uint8_t* data, std::size_t size) {
    if (data == nullptr || size == 0) {
      return false;
    }
    flatbuffers::Verifier verifier(data, size);
    return verifier.VerifyBuffer<lowlevel::ee::EndEffectorCapability>(nullptr);
  }

  static const lowlevel::ee::EndEffectorCapability* GetRoot(const uint8_t* data) {
    if (data == nullptr) {
      return nullptr;
    }
    return flatbuffers::GetRoot<lowlevel::ee::EndEffectorCapability>(data);
  }
};

template <>
struct TypedSubscribeTraits<::lowlevel::ee::EndEffectorInfo> {
  static bool Verify(const uint8_t* data, std::size_t size) {
    if (data == nullptr || size == 0) {
      return false;
    }
    flatbuffers::Verifier verifier(data, size);
    return ::lowlevel::ee::VerifyEndEffectorInfoBuffer(verifier);
  }

  static const lowlevel::ee::EndEffectorInfo* GetRoot(const uint8_t* data) {
    if (data == nullptr) {
      return nullptr;
    }
    return ::lowlevel::ee::GetEndEffectorInfo(data);
  }
};

}  // namespace aorta
