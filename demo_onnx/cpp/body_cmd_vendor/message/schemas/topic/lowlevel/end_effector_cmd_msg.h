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
#include "end_effector_cmd_generated.h"
#include "end_effector_cmd_schema_meta.h"
// IWYU pragma: end_exports

namespace aorta {
template <typename T>
struct TypedPublishTraits;
}  // namespace aorta

namespace lowlevel {
namespace ee {

struct AortaHeaderMsg;
struct EndEffectorCmdMsg;
struct TouchRegionInfoMsg;

#ifndef AORTA_TYPED_WRAPPER_TABLE_MSG_LOWLEVEL_EE_AORTAHEADERMSG_DEFINED
#define AORTA_TYPED_WRAPPER_TABLE_MSG_LOWLEVEL_EE_AORTAHEADERMSG_DEFINED 1
struct AortaHeaderMsg {
  std::optional<std::uint64_t> publish_stamp_ns;
  std::optional<std::string> publisher_node;
  std::optional<std::uint64_t> sequence;
};
#endif  // AORTA_TYPED_WRAPPER_TABLE_MSG_LOWLEVEL_EE_AORTAHEADERMSG_DEFINED

#ifndef AORTA_TYPED_WRAPPER_TABLE_MSG_LOWLEVEL_EE_ENDEFFECTORCMDMSG_DEFINED
#define AORTA_TYPED_WRAPPER_TABLE_MSG_LOWLEVEL_EE_ENDEFFECTORCMDMSG_DEFINED 1
struct EndEffectorCmdMsg {
  std::optional<std::uint32_t> protocol_version;
  std::optional<std::uint64_t> runtime_id;
  std::optional<std::uint32_t> layout_id;
  std::optional<std::uint8_t> side;
  std::optional<std::uint64_t> control_id;
  std::optional<::lowlevel::CommandKey> key;
  std::optional<std::vector<::lowlevel::SampleRef>> input_observations;
  std::optional<std::uint64_t> ts_client_created_us;
  std::optional<std::uint64_t> ts_client_send_us;
  std::optional<std::vector<::lowlevel::ee::AxisCommand>> axes;
};
#endif  // AORTA_TYPED_WRAPPER_TABLE_MSG_LOWLEVEL_EE_ENDEFFECTORCMDMSG_DEFINED

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

namespace typed_lowlevel_ee_end_effector_cmd {

inline flatbuffers::Offset<aorta::sys::AortaHeader> BuildAortaHeaderTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::lowlevel::ee::AortaHeaderMsg& msg,
    flatbuffers::Offset<aorta::sys::AortaHeader> header);
inline flatbuffers::Offset<lowlevel::ee::EndEffectorCmd> BuildEndEffectorCmdTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::lowlevel::ee::EndEffectorCmdMsg& msg,
    flatbuffers::Offset<aorta::sys::AortaHeader> header);
inline flatbuffers::Offset<lowlevel::ee::TouchRegionInfo> BuildTouchRegionInfoTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::lowlevel::ee::TouchRegionInfoMsg& msg,
    flatbuffers::Offset<aorta::sys::AortaHeader> header);

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

inline flatbuffers::Offset<lowlevel::ee::EndEffectorCmd> BuildEndEffectorCmdTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::lowlevel::ee::EndEffectorCmdMsg& msg,
    flatbuffers::Offset<aorta::sys::AortaHeader> header) {
  flatbuffers::Offset<flatbuffers::Vector<const ::lowlevel::SampleRef *>> input_observations_offset;
  if (msg.input_observations.has_value()) {
    input_observations_offset = builder.CreateVectorOfStructs(msg.input_observations.value());
  }
  flatbuffers::Offset<flatbuffers::Vector<const ::lowlevel::ee::AxisCommand *>> axes_offset;
  if (msg.axes.has_value()) {
    axes_offset = builder.CreateVectorOfStructs(msg.axes.value());
  }
  ::lowlevel::ee::EndEffectorCmdBuilder root_builder(builder);
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
  if (msg.key.has_value()) {
    root_builder.add_key(&msg.key.value());
  }
  if (input_observations_offset.o != 0) {
    root_builder.add_input_observations(input_observations_offset);
  }
  if (msg.ts_client_created_us.has_value()) {
    root_builder.add_ts_client_created_us(msg.ts_client_created_us.value());
  }
  if (msg.ts_client_send_us.has_value()) {
    root_builder.add_ts_client_send_us(msg.ts_client_send_us.value());
  }
  if (axes_offset.o != 0) {
    root_builder.add_axes(axes_offset);
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

}  // namespace typed_lowlevel_ee_end_effector_cmd
}  // namespace detail

template <>
struct TypedPublishTraits<::lowlevel::ee::EndEffectorCmd> {
  using Msg = ::lowlevel::ee::EndEffectorCmdMsg;
  static constexpr bool kHasAortaHeader = true;

  static flatbuffers::Offset<lowlevel::ee::EndEffectorCmd> Build(
      flatbuffers::FlatBufferBuilder& builder,
      flatbuffers::Offset<aorta::sys::AortaHeader> header,
      const Msg& msg) {
    return detail::typed_lowlevel_ee_end_effector_cmd::BuildEndEffectorCmdTyped(builder, msg, header);
  }
};

template <>
struct TypedSubscribeTraits<::lowlevel::ee::EndEffectorCmd> {
  static bool Verify(const uint8_t* data, std::size_t size) {
    if (data == nullptr || size == 0) {
      return false;
    }
    flatbuffers::Verifier verifier(data, size);
    return ::lowlevel::ee::VerifyEndEffectorCmdBuffer(verifier);
  }

  static const lowlevel::ee::EndEffectorCmd* GetRoot(const uint8_t* data) {
    if (data == nullptr) {
      return nullptr;
    }
    return ::lowlevel::ee::GetEndEffectorCmd(data);
  }
};

}  // namespace aorta
