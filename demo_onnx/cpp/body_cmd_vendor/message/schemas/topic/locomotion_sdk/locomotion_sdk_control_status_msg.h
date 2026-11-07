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
#include "control_status_generated.h"
#include "control_status_schema_meta.h"
// IWYU pragma: end_exports

namespace aorta {
template <typename T>
struct TypedPublishTraits;
}  // namespace aorta

namespace locomotion_sdk {

struct AortaHeaderMsg;
struct ControlStatusMsg;

#ifndef AORTA_TYPED_WRAPPER_TABLE_MSG_LOCOMOTION_SDK_AORTAHEADERMSG_DEFINED
#define AORTA_TYPED_WRAPPER_TABLE_MSG_LOCOMOTION_SDK_AORTAHEADERMSG_DEFINED 1
struct AortaHeaderMsg {
  std::optional<std::uint64_t> publish_stamp_ns;
  std::optional<std::string> publisher_node;
  std::optional<std::uint64_t> sequence;
};
#endif  // AORTA_TYPED_WRAPPER_TABLE_MSG_LOCOMOTION_SDK_AORTAHEADERMSG_DEFINED

#ifndef AORTA_TYPED_WRAPPER_TABLE_MSG_LOCOMOTION_SDK_CONTROLSTATUSMSG_DEFINED
#define AORTA_TYPED_WRAPPER_TABLE_MSG_LOCOMOTION_SDK_CONTROLSTATUSMSG_DEFINED 1
struct ControlStatusMsg {
  std::optional<std::uint32_t> protocol_version;
  std::optional<std::uint64_t> status_seq;
  std::optional<std::string> current_action;
  std::optional<::locomotion_sdk::ControlActionPhase> action_phase;
  std::optional<std::uint64_t> client_id;
  std::optional<std::uint64_t> control_id;
  std::optional<std::uint64_t> generation;
  std::optional<std::uint64_t> last_accepted_cmd_id;
  std::optional<std::uint32_t> last_valid_age_ms;
  std::optional<std::uint32_t> estimated_input_period_us;
  std::optional<bool> timeout_hold;
  std::optional<std::uint32_t> last_reject_code;
  std::optional<std::uint64_t> reject_count;
  std::optional<std::uint64_t> ingress_drop_count;
  std::optional<std::uint64_t> stamp_monotonic_ns;
  std::optional<std::uint64_t> runtime_id;
};
#endif  // AORTA_TYPED_WRAPPER_TABLE_MSG_LOCOMOTION_SDK_CONTROLSTATUSMSG_DEFINED

}  // namespace locomotion_sdk

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

namespace typed_locomotion_sdk_control_status {

inline flatbuffers::Offset<aorta::sys::AortaHeader> BuildAortaHeaderTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::locomotion_sdk::AortaHeaderMsg& msg,
    flatbuffers::Offset<aorta::sys::AortaHeader> header);
inline flatbuffers::Offset<locomotion_sdk::ControlStatus> BuildControlStatusTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::locomotion_sdk::ControlStatusMsg& msg,
    flatbuffers::Offset<aorta::sys::AortaHeader> header);

inline flatbuffers::Offset<aorta::sys::AortaHeader> BuildAortaHeaderTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::locomotion_sdk::AortaHeaderMsg& msg,
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

inline flatbuffers::Offset<locomotion_sdk::ControlStatus> BuildControlStatusTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::locomotion_sdk::ControlStatusMsg& msg,
    flatbuffers::Offset<aorta::sys::AortaHeader> header) {
  flatbuffers::Offset<flatbuffers::String> current_action_offset;
  if (msg.current_action.has_value()) {
    current_action_offset = builder.CreateString(msg.current_action.value());
  }
  ::locomotion_sdk::ControlStatusBuilder root_builder(builder);
  if (header.o != 0) {
    root_builder.add_aorta_header(header);
  }
  if (msg.protocol_version.has_value()) {
    root_builder.add_protocol_version(msg.protocol_version.value());
  }
  if (msg.status_seq.has_value()) {
    root_builder.add_status_seq(msg.status_seq.value());
  }
  if (current_action_offset.o != 0) {
    root_builder.add_current_action(current_action_offset);
  }
  if (msg.action_phase.has_value()) {
    root_builder.add_action_phase(msg.action_phase.value());
  }
  if (msg.client_id.has_value()) {
    root_builder.add_client_id(msg.client_id.value());
  }
  if (msg.control_id.has_value()) {
    root_builder.add_control_id(msg.control_id.value());
  }
  if (msg.generation.has_value()) {
    root_builder.add_generation(msg.generation.value());
  }
  if (msg.last_accepted_cmd_id.has_value()) {
    root_builder.add_last_accepted_cmd_id(msg.last_accepted_cmd_id.value());
  }
  if (msg.last_valid_age_ms.has_value()) {
    root_builder.add_last_valid_age_ms(msg.last_valid_age_ms.value());
  }
  if (msg.estimated_input_period_us.has_value()) {
    root_builder.add_estimated_input_period_us(msg.estimated_input_period_us.value());
  }
  if (msg.timeout_hold.has_value()) {
    root_builder.add_timeout_hold(msg.timeout_hold.value());
  }
  if (msg.last_reject_code.has_value()) {
    root_builder.add_last_reject_code(msg.last_reject_code.value());
  }
  if (msg.reject_count.has_value()) {
    root_builder.add_reject_count(msg.reject_count.value());
  }
  if (msg.ingress_drop_count.has_value()) {
    root_builder.add_ingress_drop_count(msg.ingress_drop_count.value());
  }
  if (msg.stamp_monotonic_ns.has_value()) {
    root_builder.add_stamp_monotonic_ns(msg.stamp_monotonic_ns.value());
  }
  if (msg.runtime_id.has_value()) {
    root_builder.add_runtime_id(msg.runtime_id.value());
  }
  return root_builder.Finish();
}

}  // namespace typed_locomotion_sdk_control_status
}  // namespace detail

template <>
struct TypedPublishTraits<::locomotion_sdk::ControlStatus> {
  using Msg = ::locomotion_sdk::ControlStatusMsg;
  static constexpr bool kHasAortaHeader = true;

  static flatbuffers::Offset<locomotion_sdk::ControlStatus> Build(
      flatbuffers::FlatBufferBuilder& builder,
      flatbuffers::Offset<aorta::sys::AortaHeader> header,
      const Msg& msg) {
    return detail::typed_locomotion_sdk_control_status::BuildControlStatusTyped(builder, msg, header);
  }
};

template <>
struct TypedSubscribeTraits<::locomotion_sdk::ControlStatus> {
  static bool Verify(const uint8_t* data, std::size_t size) {
    if (data == nullptr || size == 0) {
      return false;
    }
    flatbuffers::Verifier verifier(data, size);
    return ::locomotion_sdk::VerifyControlStatusBuffer(verifier);
  }

  static const locomotion_sdk::ControlStatus* GetRoot(const uint8_t* data) {
    if (data == nullptr) {
      return nullptr;
    }
    return ::locomotion_sdk::GetControlStatus(data);
  }
};

}  // namespace aorta
