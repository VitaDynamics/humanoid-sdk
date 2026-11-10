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
#include "body_state_generated.h"
#include "body_state_schema_meta.h"
// IWYU pragma: end_exports

namespace aorta {
template <typename T>
struct TypedPublishTraits;
}  // namespace aorta

namespace lowlevel {

struct AortaHeaderMsg;
struct BodyImuSampleMsg;
struct BodyStateMsg;

#ifndef AORTA_TYPED_WRAPPER_TABLE_MSG_LOWLEVEL_AORTAHEADERMSG_DEFINED
#define AORTA_TYPED_WRAPPER_TABLE_MSG_LOWLEVEL_AORTAHEADERMSG_DEFINED 1
struct AortaHeaderMsg {
  std::optional<std::uint64_t> publish_stamp_ns;
  std::optional<std::string> publisher_node;
  std::optional<std::uint64_t> sequence;
};
#endif  // AORTA_TYPED_WRAPPER_TABLE_MSG_LOWLEVEL_AORTAHEADERMSG_DEFINED

#ifndef AORTA_TYPED_WRAPPER_TABLE_MSG_LOWLEVEL_BODYIMUSAMPLEMSG_DEFINED
#define AORTA_TYPED_WRAPPER_TABLE_MSG_LOWLEVEL_BODYIMUSAMPLEMSG_DEFINED 1
struct BodyImuSampleMsg {
  std::optional<std::uint64_t> ts_sampled_us;
  std::optional<std::uint64_t> ts_acquired_hw_us;
  std::optional<std::uint64_t> ts_ingested_us;
  std::optional<std::uint64_t> ts_assembled_us;
  std::optional<::lowlevel::BodyImuLocation> location;
  std::optional<::lowlevel::BodyImuState> state;
  std::optional<::lowlevel::BodySampleStatus> status;
};
#endif  // AORTA_TYPED_WRAPPER_TABLE_MSG_LOWLEVEL_BODYIMUSAMPLEMSG_DEFINED

#ifndef AORTA_TYPED_WRAPPER_TABLE_MSG_LOWLEVEL_BODYSTATEMSG_DEFINED
#define AORTA_TYPED_WRAPPER_TABLE_MSG_LOWLEVEL_BODYSTATEMSG_DEFINED 1
struct BodyStateMsg {
  std::optional<std::uint32_t> protocol_version;
  std::optional<std::uint32_t> roster_version;
  std::optional<std::uint64_t> state_id;
  std::optional<std::uint64_t> ts_state_real_us;
  std::optional<std::uint64_t> ts_state_pub_us;
  std::optional<std::uint64_t> ts_consumed_us;
  std::optional<bool> attitude_valid;
  std::optional<std::vector<std::shared_ptr<BodyImuSampleMsg>>> imu_states;
  std::optional<std::vector<::lowlevel::BodyJointState>> joints;
  std::optional<std::uint64_t> applied_session_id;
  std::optional<std::uint64_t> applied_sequence;
  std::optional<std::uint64_t> ts_first_output_us;
  std::optional<std::uint64_t> runtime_id;
  std::optional<std::uint64_t> ts_control_us;
  std::optional<bool> applied_key_valid;
  std::optional<std::uint8_t> association_source_type;
};
#endif  // AORTA_TYPED_WRAPPER_TABLE_MSG_LOWLEVEL_BODYSTATEMSG_DEFINED

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

namespace typed_lowlevel_body_state {

inline flatbuffers::Offset<aorta::sys::AortaHeader> BuildAortaHeaderTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::lowlevel::AortaHeaderMsg& msg,
    flatbuffers::Offset<aorta::sys::AortaHeader> header);
inline flatbuffers::Offset<lowlevel::BodyImuSample> BuildBodyImuSampleTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::lowlevel::BodyImuSampleMsg& msg,
    flatbuffers::Offset<aorta::sys::AortaHeader> header);
inline flatbuffers::Offset<lowlevel::BodyState> BuildBodyStateTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::lowlevel::BodyStateMsg& msg,
    flatbuffers::Offset<aorta::sys::AortaHeader> header);

inline flatbuffers::Offset<aorta::sys::AortaHeader> BuildAortaHeaderTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::lowlevel::AortaHeaderMsg& msg,
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

inline flatbuffers::Offset<lowlevel::BodyImuSample> BuildBodyImuSampleTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::lowlevel::BodyImuSampleMsg& msg,
    flatbuffers::Offset<aorta::sys::AortaHeader> header) {
  ::lowlevel::BodyImuSampleBuilder root_builder(builder);
  (void)header;
  if (msg.ts_sampled_us.has_value()) {
    root_builder.add_ts_sampled_us(msg.ts_sampled_us.value());
  }
  if (msg.ts_acquired_hw_us.has_value()) {
    root_builder.add_ts_acquired_hw_us(msg.ts_acquired_hw_us.value());
  }
  if (msg.ts_ingested_us.has_value()) {
    root_builder.add_ts_ingested_us(msg.ts_ingested_us.value());
  }
  if (msg.ts_assembled_us.has_value()) {
    root_builder.add_ts_assembled_us(msg.ts_assembled_us.value());
  }
  if (msg.location.has_value()) {
    root_builder.add_location(msg.location.value());
  }
  if (msg.state.has_value()) {
    root_builder.add_state(&msg.state.value());
  }
  if (msg.status.has_value()) {
    root_builder.add_status(msg.status.value());
  }
  return root_builder.Finish();
}

inline flatbuffers::Offset<lowlevel::BodyState> BuildBodyStateTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::lowlevel::BodyStateMsg& msg,
    flatbuffers::Offset<aorta::sys::AortaHeader> header) {
  flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<::lowlevel::BodyImuSample>>> imu_states_offset;
  if (msg.imu_states.has_value()) {
    ::aorta::detail::ScratchVector<flatbuffers::Offset<::lowlevel::BodyImuSample>> offsets_scratch;
    auto& offsets = offsets_scratch.get();
    offsets.reserve(msg.imu_states.value().size());
    for (const auto& item_ptr : msg.imu_states.value()) {
      if (!item_ptr) {
        continue;
      }
      offsets.push_back(BuildBodyImuSampleTyped(builder, *item_ptr, {}));
    }
    imu_states_offset = builder.CreateVector(offsets);
  }
  flatbuffers::Offset<flatbuffers::Vector<const ::lowlevel::BodyJointState *>> joints_offset;
  if (msg.joints.has_value()) {
    joints_offset = builder.CreateVectorOfStructs(msg.joints.value());
  }
  ::lowlevel::BodyStateBuilder root_builder(builder);
  if (header.o != 0) {
    root_builder.add_aorta_header(header);
  }
  if (msg.protocol_version.has_value()) {
    root_builder.add_protocol_version(msg.protocol_version.value());
  }
  if (msg.roster_version.has_value()) {
    root_builder.add_roster_version(msg.roster_version.value());
  }
  if (msg.state_id.has_value()) {
    root_builder.add_state_id(msg.state_id.value());
  }
  if (msg.ts_state_real_us.has_value()) {
    root_builder.add_ts_state_real_us(msg.ts_state_real_us.value());
  }
  if (msg.ts_state_pub_us.has_value()) {
    root_builder.add_ts_state_pub_us(msg.ts_state_pub_us.value());
  }
  if (msg.ts_consumed_us.has_value()) {
    root_builder.add_ts_consumed_us(msg.ts_consumed_us.value());
  }
  if (msg.attitude_valid.has_value()) {
    root_builder.add_attitude_valid(msg.attitude_valid.value());
  }
  if (imu_states_offset.o != 0) {
    root_builder.add_imu_states(imu_states_offset);
  }
  if (joints_offset.o != 0) {
    root_builder.add_joints(joints_offset);
  }
  if (msg.applied_session_id.has_value()) {
    root_builder.add_applied_session_id(msg.applied_session_id.value());
  }
  if (msg.applied_sequence.has_value()) {
    root_builder.add_applied_sequence(msg.applied_sequence.value());
  }
  if (msg.ts_first_output_us.has_value()) {
    root_builder.add_ts_first_output_us(msg.ts_first_output_us.value());
  }
  if (msg.runtime_id.has_value()) {
    root_builder.add_runtime_id(msg.runtime_id.value());
  }
  if (msg.ts_control_us.has_value()) {
    root_builder.add_ts_control_us(msg.ts_control_us.value());
  }
  if (msg.applied_key_valid.has_value()) {
    root_builder.add_applied_key_valid(msg.applied_key_valid.value());
  }
  if (msg.association_source_type.has_value()) {
    root_builder.add_association_source_type(msg.association_source_type.value());
  }
  return root_builder.Finish();
}

}  // namespace typed_lowlevel_body_state
}  // namespace detail

template <>
struct TypedPublishTraits<::lowlevel::BodyState> {
  using Msg = ::lowlevel::BodyStateMsg;
  static constexpr bool kHasAortaHeader = true;

  static flatbuffers::Offset<lowlevel::BodyState> Build(
      flatbuffers::FlatBufferBuilder& builder,
      flatbuffers::Offset<aorta::sys::AortaHeader> header,
      const Msg& msg) {
    return detail::typed_lowlevel_body_state::BuildBodyStateTyped(builder, msg, header);
  }
};

template <>
struct TypedSubscribeTraits<::lowlevel::BodyImuSample> {
  static bool Verify(const uint8_t* data, std::size_t size) {
    if (data == nullptr || size == 0) {
      return false;
    }
    flatbuffers::Verifier verifier(data, size);
    return verifier.VerifyBuffer<lowlevel::BodyImuSample>(nullptr);
  }

  static const lowlevel::BodyImuSample* GetRoot(const uint8_t* data) {
    if (data == nullptr) {
      return nullptr;
    }
    return flatbuffers::GetRoot<lowlevel::BodyImuSample>(data);
  }
};

template <>
struct TypedSubscribeTraits<::lowlevel::BodyState> {
  static bool Verify(const uint8_t* data, std::size_t size) {
    if (data == nullptr || size == 0) {
      return false;
    }
    flatbuffers::Verifier verifier(data, size);
    return ::lowlevel::VerifyBodyStateBuffer(verifier);
  }

  static const lowlevel::BodyState* GetRoot(const uint8_t* data) {
    if (data == nullptr) {
      return nullptr;
    }
    return ::lowlevel::GetBodyState(data);
  }
};

}  // namespace aorta
