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
#include "control_ack_generated.h"
#include "control_ack_schema_meta.h"
// IWYU pragma: end_exports

namespace aorta {
template <typename T>
struct TypedPublishTraits;
}  // namespace aorta

namespace locomotion_sdk {

struct AortaHeaderMsg;
struct ControlAckMsg;
struct ControlRequestMsg;

#ifndef AORTA_TYPED_WRAPPER_TABLE_MSG_LOCOMOTION_SDK_AORTAHEADERMSG_DEFINED
#define AORTA_TYPED_WRAPPER_TABLE_MSG_LOCOMOTION_SDK_AORTAHEADERMSG_DEFINED 1
struct AortaHeaderMsg {
  std::optional<std::uint64_t> publish_stamp_ns;
  std::optional<std::string> publisher_node;
  std::optional<std::uint64_t> sequence;
};
#endif  // AORTA_TYPED_WRAPPER_TABLE_MSG_LOCOMOTION_SDK_AORTAHEADERMSG_DEFINED

#ifndef AORTA_TYPED_WRAPPER_TABLE_MSG_LOCOMOTION_SDK_CONTROLACKMSG_DEFINED
#define AORTA_TYPED_WRAPPER_TABLE_MSG_LOCOMOTION_SDK_CONTROLACKMSG_DEFINED 1
struct ControlAckMsg {
  std::optional<std::uint32_t> protocol_version;
  std::optional<::locomotion_sdk::ControlOperation> operation;
  std::optional<std::string> request_id;
  std::optional<bool> accepted;
  std::optional<::locomotion_sdk::ControlErrorCode> error_code;
  std::optional<std::string> reason;
  std::optional<std::uint64_t> control_id;
  std::optional<std::uint64_t> runtime_id;
  std::optional<std::uint64_t> client_id;
};
#endif  // AORTA_TYPED_WRAPPER_TABLE_MSG_LOCOMOTION_SDK_CONTROLACKMSG_DEFINED

#ifndef AORTA_TYPED_WRAPPER_TABLE_MSG_LOCOMOTION_SDK_CONTROLREQUESTMSG_DEFINED
#define AORTA_TYPED_WRAPPER_TABLE_MSG_LOCOMOTION_SDK_CONTROLREQUESTMSG_DEFINED 1
struct ControlRequestMsg {
  std::optional<std::uint32_t> protocol_version;
  std::optional<::locomotion_sdk::ControlOperation> operation;
  std::optional<std::string> request_id;
  std::optional<std::uint64_t> client_id;
  std::optional<std::string> action_name;
  std::optional<std::uint64_t> control_id;
  std::optional<std::uint64_t> runtime_id;
};
#endif  // AORTA_TYPED_WRAPPER_TABLE_MSG_LOCOMOTION_SDK_CONTROLREQUESTMSG_DEFINED

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

namespace typed_locomotion_sdk_control_ack {

inline flatbuffers::Offset<aorta::sys::AortaHeader> BuildAortaHeaderTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::locomotion_sdk::AortaHeaderMsg& msg,
    flatbuffers::Offset<aorta::sys::AortaHeader> header);
inline flatbuffers::Offset<locomotion_sdk::ControlAck> BuildControlAckTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::locomotion_sdk::ControlAckMsg& msg,
    flatbuffers::Offset<aorta::sys::AortaHeader> header);
inline flatbuffers::Offset<locomotion_sdk::ControlRequest> BuildControlRequestTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::locomotion_sdk::ControlRequestMsg& msg,
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

inline flatbuffers::Offset<locomotion_sdk::ControlAck> BuildControlAckTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::locomotion_sdk::ControlAckMsg& msg,
    flatbuffers::Offset<aorta::sys::AortaHeader> header) {
  flatbuffers::Offset<flatbuffers::String> request_id_offset;
  if (msg.request_id.has_value()) {
    request_id_offset = builder.CreateString(msg.request_id.value());
  }
  flatbuffers::Offset<flatbuffers::String> reason_offset;
  if (msg.reason.has_value()) {
    reason_offset = builder.CreateString(msg.reason.value());
  }
  ::locomotion_sdk::ControlAckBuilder root_builder(builder);
  if (header.o != 0) {
    root_builder.add_aorta_header(header);
  }
  if (msg.protocol_version.has_value()) {
    root_builder.add_protocol_version(msg.protocol_version.value());
  }
  if (msg.operation.has_value()) {
    root_builder.add_operation(msg.operation.value());
  }
  if (request_id_offset.o != 0) {
    root_builder.add_request_id(request_id_offset);
  }
  if (msg.accepted.has_value()) {
    root_builder.add_accepted(msg.accepted.value());
  }
  if (msg.error_code.has_value()) {
    root_builder.add_error_code(msg.error_code.value());
  }
  if (reason_offset.o != 0) {
    root_builder.add_reason(reason_offset);
  }
  if (msg.control_id.has_value()) {
    root_builder.add_control_id(msg.control_id.value());
  }
  if (msg.runtime_id.has_value()) {
    root_builder.add_runtime_id(msg.runtime_id.value());
  }
  if (msg.client_id.has_value()) {
    root_builder.add_client_id(msg.client_id.value());
  }
  return root_builder.Finish();
}

inline flatbuffers::Offset<locomotion_sdk::ControlRequest> BuildControlRequestTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::locomotion_sdk::ControlRequestMsg& msg,
    flatbuffers::Offset<aorta::sys::AortaHeader> header) {
  flatbuffers::Offset<flatbuffers::String> request_id_offset;
  if (msg.request_id.has_value()) {
    request_id_offset = builder.CreateString(msg.request_id.value());
  }
  flatbuffers::Offset<flatbuffers::String> action_name_offset;
  if (msg.action_name.has_value()) {
    action_name_offset = builder.CreateString(msg.action_name.value());
  }
  ::locomotion_sdk::ControlRequestBuilder root_builder(builder);
  if (header.o != 0) {
    root_builder.add_aorta_header(header);
  }
  if (msg.protocol_version.has_value()) {
    root_builder.add_protocol_version(msg.protocol_version.value());
  }
  if (msg.operation.has_value()) {
    root_builder.add_operation(msg.operation.value());
  }
  if (request_id_offset.o != 0) {
    root_builder.add_request_id(request_id_offset);
  }
  if (msg.client_id.has_value()) {
    root_builder.add_client_id(msg.client_id.value());
  }
  if (action_name_offset.o != 0) {
    root_builder.add_action_name(action_name_offset);
  }
  if (msg.control_id.has_value()) {
    root_builder.add_control_id(msg.control_id.value());
  }
  if (msg.runtime_id.has_value()) {
    root_builder.add_runtime_id(msg.runtime_id.value());
  }
  return root_builder.Finish();
}

}  // namespace typed_locomotion_sdk_control_ack
}  // namespace detail

template <>
struct TypedPublishTraits<::locomotion_sdk::ControlAck> {
  using Msg = ::locomotion_sdk::ControlAckMsg;
  static constexpr bool kHasAortaHeader = true;

  static flatbuffers::Offset<locomotion_sdk::ControlAck> Build(
      flatbuffers::FlatBufferBuilder& builder,
      flatbuffers::Offset<aorta::sys::AortaHeader> header,
      const Msg& msg) {
    return detail::typed_locomotion_sdk_control_ack::BuildControlAckTyped(builder, msg, header);
  }
};

template <>
struct TypedSubscribeTraits<::locomotion_sdk::ControlAck> {
  static bool Verify(const uint8_t* data, std::size_t size) {
    if (data == nullptr || size == 0) {
      return false;
    }
    flatbuffers::Verifier verifier(data, size);
    return ::locomotion_sdk::VerifyControlAckBuffer(verifier);
  }

  static const locomotion_sdk::ControlAck* GetRoot(const uint8_t* data) {
    if (data == nullptr) {
      return nullptr;
    }
    return ::locomotion_sdk::GetControlAck(data);
  }
};

}  // namespace aorta
