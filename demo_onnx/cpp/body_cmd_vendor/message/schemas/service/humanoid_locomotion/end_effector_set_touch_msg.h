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
#include "end_effector_set_touch_generated.h"
#include "end_effector_set_touch_schema_meta.h"
// IWYU pragma: end_exports

namespace aorta {
template <typename T>
struct TypedPublishTraits;
}  // namespace aorta

namespace aorta {
namespace services {
namespace humanoid_locomotion {

struct ErrorInfoMsg;
struct EndEffectorSetTouchRequestMsg;
struct EndEffectorSetTouchResponseMsg;
struct AortaHeaderMsg;
struct TouchRegionInfoMsg;

#ifndef AORTA_TYPED_WRAPPER_TABLE_MSG_AORTA_SERVICES_HUMANOID_LOCOMOTION_ERRORINFOMSG_DEFINED
#define AORTA_TYPED_WRAPPER_TABLE_MSG_AORTA_SERVICES_HUMANOID_LOCOMOTION_ERRORINFOMSG_DEFINED 1
struct ErrorInfoMsg {
  std::optional<std::int32_t> code;
  std::optional<std::string> message;
  std::optional<std::string> details;
  std::optional<std::uint64_t> timestamp_ns;
};
#endif  // AORTA_TYPED_WRAPPER_TABLE_MSG_AORTA_SERVICES_HUMANOID_LOCOMOTION_ERRORINFOMSG_DEFINED

#ifndef AORTA_TYPED_WRAPPER_TABLE_MSG_AORTA_SERVICES_HUMANOID_LOCOMOTION_ENDEFFECTORSETTOUCHREQUESTMSG_DEFINED
#define AORTA_TYPED_WRAPPER_TABLE_MSG_AORTA_SERVICES_HUMANOID_LOCOMOTION_ENDEFFECTORSETTOUCHREQUESTMSG_DEFINED 1
struct EndEffectorSetTouchRequestMsg {
  std::optional<std::uint32_t> protocol_version;
  std::optional<std::uint64_t> runtime_id;
  std::optional<std::uint32_t> layout_id;
  std::optional<std::uint8_t> side;
  std::optional<std::uint64_t> control_id;
  std::optional<bool> enable;
};
#endif  // AORTA_TYPED_WRAPPER_TABLE_MSG_AORTA_SERVICES_HUMANOID_LOCOMOTION_ENDEFFECTORSETTOUCHREQUESTMSG_DEFINED

#ifndef AORTA_TYPED_WRAPPER_TABLE_MSG_AORTA_SERVICES_HUMANOID_LOCOMOTION_ENDEFFECTORSETTOUCHRESPONSEMSG_DEFINED
#define AORTA_TYPED_WRAPPER_TABLE_MSG_AORTA_SERVICES_HUMANOID_LOCOMOTION_ENDEFFECTORSETTOUCHRESPONSEMSG_DEFINED 1
struct EndEffectorSetTouchResponseMsg {
  std::optional<std::uint32_t> protocol_version;
  std::optional<std::uint64_t> runtime_id;
  std::optional<std::uint8_t> side;
  std::optional<::aorta::services::common::ServiceStatus> status;
  std::optional<std::uint16_t> reason;
};
#endif  // AORTA_TYPED_WRAPPER_TABLE_MSG_AORTA_SERVICES_HUMANOID_LOCOMOTION_ENDEFFECTORSETTOUCHRESPONSEMSG_DEFINED

#ifndef AORTA_TYPED_WRAPPER_TABLE_MSG_AORTA_SERVICES_HUMANOID_LOCOMOTION_AORTAHEADERMSG_DEFINED
#define AORTA_TYPED_WRAPPER_TABLE_MSG_AORTA_SERVICES_HUMANOID_LOCOMOTION_AORTAHEADERMSG_DEFINED 1
struct AortaHeaderMsg {
  std::optional<std::uint64_t> publish_stamp_ns;
  std::optional<std::string> publisher_node;
  std::optional<std::uint64_t> sequence;
};
#endif  // AORTA_TYPED_WRAPPER_TABLE_MSG_AORTA_SERVICES_HUMANOID_LOCOMOTION_AORTAHEADERMSG_DEFINED

#ifndef AORTA_TYPED_WRAPPER_TABLE_MSG_AORTA_SERVICES_HUMANOID_LOCOMOTION_TOUCHREGIONINFOMSG_DEFINED
#define AORTA_TYPED_WRAPPER_TABLE_MSG_AORTA_SERVICES_HUMANOID_LOCOMOTION_TOUCHREGIONINFOMSG_DEFINED 1
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
#endif  // AORTA_TYPED_WRAPPER_TABLE_MSG_AORTA_SERVICES_HUMANOID_LOCOMOTION_TOUCHREGIONINFOMSG_DEFINED

}  // namespace humanoid_locomotion
}  // namespace services
}  // namespace aorta

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

namespace typed_aorta_services_humanoid_locomotion_end_effector_set_touch {

inline flatbuffers::Offset<aorta::services::common::ErrorInfo> BuildErrorInfoTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::aorta::services::humanoid_locomotion::ErrorInfoMsg& msg,
    flatbuffers::Offset<aorta::sys::AortaHeader> header);
inline flatbuffers::Offset<aorta::services::humanoid_locomotion::EndEffectorSetTouchRequest> BuildEndEffectorSetTouchRequestTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::aorta::services::humanoid_locomotion::EndEffectorSetTouchRequestMsg& msg,
    flatbuffers::Offset<aorta::sys::AortaHeader> header);
inline flatbuffers::Offset<aorta::services::humanoid_locomotion::EndEffectorSetTouchResponse> BuildEndEffectorSetTouchResponseTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::aorta::services::humanoid_locomotion::EndEffectorSetTouchResponseMsg& msg,
    flatbuffers::Offset<aorta::sys::AortaHeader> header);
inline flatbuffers::Offset<aorta::sys::AortaHeader> BuildAortaHeaderTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::aorta::services::humanoid_locomotion::AortaHeaderMsg& msg,
    flatbuffers::Offset<aorta::sys::AortaHeader> header);
inline flatbuffers::Offset<lowlevel::ee::TouchRegionInfo> BuildTouchRegionInfoTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::aorta::services::humanoid_locomotion::TouchRegionInfoMsg& msg,
    flatbuffers::Offset<aorta::sys::AortaHeader> header);

inline flatbuffers::Offset<aorta::services::common::ErrorInfo> BuildErrorInfoTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::aorta::services::humanoid_locomotion::ErrorInfoMsg& msg,
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

inline flatbuffers::Offset<aorta::services::humanoid_locomotion::EndEffectorSetTouchRequest> BuildEndEffectorSetTouchRequestTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::aorta::services::humanoid_locomotion::EndEffectorSetTouchRequestMsg& msg,
    flatbuffers::Offset<aorta::sys::AortaHeader> header) {
  ::aorta::services::humanoid_locomotion::EndEffectorSetTouchRequestBuilder root_builder(builder);
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
  if (msg.control_id.has_value()) {
    root_builder.add_control_id(msg.control_id.value());
  }
  if (msg.enable.has_value()) {
    root_builder.add_enable(msg.enable.value());
  }
  return root_builder.Finish();
}

inline flatbuffers::Offset<aorta::services::humanoid_locomotion::EndEffectorSetTouchResponse> BuildEndEffectorSetTouchResponseTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::aorta::services::humanoid_locomotion::EndEffectorSetTouchResponseMsg& msg,
    flatbuffers::Offset<aorta::sys::AortaHeader> header) {
  ::aorta::services::humanoid_locomotion::EndEffectorSetTouchResponseBuilder root_builder(builder);
  if (header.o != 0) {
    root_builder.add_aorta_header(header);
  }
  if (msg.protocol_version.has_value()) {
    root_builder.add_protocol_version(msg.protocol_version.value());
  }
  if (msg.runtime_id.has_value()) {
    root_builder.add_runtime_id(msg.runtime_id.value());
  }
  if (msg.side.has_value()) {
    root_builder.add_side(msg.side.value());
  }
  if (msg.status.has_value()) {
    root_builder.add_status(msg.status.value());
  }
  if (msg.reason.has_value()) {
    root_builder.add_reason(msg.reason.value());
  }
  return root_builder.Finish();
}

inline flatbuffers::Offset<aorta::sys::AortaHeader> BuildAortaHeaderTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::aorta::services::humanoid_locomotion::AortaHeaderMsg& msg,
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

inline flatbuffers::Offset<lowlevel::ee::TouchRegionInfo> BuildTouchRegionInfoTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::aorta::services::humanoid_locomotion::TouchRegionInfoMsg& msg,
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

}  // namespace typed_aorta_services_humanoid_locomotion_end_effector_set_touch
}  // namespace detail

template <>
struct TypedPublishTraits<::aorta::services::humanoid_locomotion::EndEffectorSetTouchRequest> {
  using Msg = ::aorta::services::humanoid_locomotion::EndEffectorSetTouchRequestMsg;
  static constexpr bool kHasAortaHeader = true;

  static flatbuffers::Offset<aorta::services::humanoid_locomotion::EndEffectorSetTouchRequest> Build(
      flatbuffers::FlatBufferBuilder& builder,
      flatbuffers::Offset<aorta::sys::AortaHeader> header,
      const Msg& msg) {
    return detail::typed_aorta_services_humanoid_locomotion_end_effector_set_touch::BuildEndEffectorSetTouchRequestTyped(builder, msg, header);
  }
};

template <>
struct TypedPublishTraits<::aorta::services::humanoid_locomotion::EndEffectorSetTouchResponse> {
  using Msg = ::aorta::services::humanoid_locomotion::EndEffectorSetTouchResponseMsg;
  static constexpr bool kHasAortaHeader = true;

  static flatbuffers::Offset<aorta::services::humanoid_locomotion::EndEffectorSetTouchResponse> Build(
      flatbuffers::FlatBufferBuilder& builder,
      flatbuffers::Offset<aorta::sys::AortaHeader> header,
      const Msg& msg) {
    return detail::typed_aorta_services_humanoid_locomotion_end_effector_set_touch::BuildEndEffectorSetTouchResponseTyped(builder, msg, header);
  }
};

template <>
struct TypedSubscribeTraits<::aorta::services::humanoid_locomotion::EndEffectorSetTouchRequest> {
  static bool Verify(const uint8_t* data, std::size_t size) {
    if (data == nullptr || size == 0) {
      return false;
    }
    flatbuffers::Verifier verifier(data, size);
    return ::aorta::services::humanoid_locomotion::VerifyEndEffectorSetTouchRequestBuffer(verifier);
  }

  static const aorta::services::humanoid_locomotion::EndEffectorSetTouchRequest* GetRoot(const uint8_t* data) {
    if (data == nullptr) {
      return nullptr;
    }
    return ::aorta::services::humanoid_locomotion::GetEndEffectorSetTouchRequest(data);
  }
};

template <>
struct TypedSubscribeTraits<::aorta::services::humanoid_locomotion::EndEffectorSetTouchResponse> {
  static bool Verify(const uint8_t* data, std::size_t size) {
    if (data == nullptr || size == 0) {
      return false;
    }
    flatbuffers::Verifier verifier(data, size);
    return verifier.VerifyBuffer<aorta::services::humanoid_locomotion::EndEffectorSetTouchResponse>(nullptr);
  }

  static const aorta::services::humanoid_locomotion::EndEffectorSetTouchResponse* GetRoot(const uint8_t* data) {
    if (data == nullptr) {
      return nullptr;
    }
    return flatbuffers::GetRoot<aorta::services::humanoid_locomotion::EndEffectorSetTouchResponse>(data);
  }
};

}  // namespace aorta
