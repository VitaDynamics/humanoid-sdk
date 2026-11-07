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
#include "body_cmd_generated.h"
#include "body_cmd_schema_meta.h"
// IWYU pragma: end_exports

namespace aorta {
template <typename T>
struct TypedPublishTraits;
}  // namespace aorta

namespace lowlevel {

struct AortaHeaderMsg;
struct BodyCmdMsg;

#ifndef AORTA_TYPED_WRAPPER_TABLE_MSG_LOWLEVEL_AORTAHEADERMSG_DEFINED
#define AORTA_TYPED_WRAPPER_TABLE_MSG_LOWLEVEL_AORTAHEADERMSG_DEFINED 1
struct AortaHeaderMsg {
  std::optional<std::uint64_t> publish_stamp_ns;
  std::optional<std::string> publisher_node;
  std::optional<std::uint64_t> sequence;
};
#endif  // AORTA_TYPED_WRAPPER_TABLE_MSG_LOWLEVEL_AORTAHEADERMSG_DEFINED

#ifndef AORTA_TYPED_WRAPPER_TABLE_MSG_LOWLEVEL_BODYCMDMSG_DEFINED
#define AORTA_TYPED_WRAPPER_TABLE_MSG_LOWLEVEL_BODYCMDMSG_DEFINED 1
struct BodyCmdMsg {
  std::optional<std::uint32_t> protocol_version;
  std::optional<std::uint64_t> session_id;
  std::optional<std::uint64_t> sequence;
  std::optional<std::uint64_t> input_state_id;
  std::optional<std::uint64_t> input_state_ts_real_us;
  std::optional<std::uint64_t> ts_client_created_us;
  std::optional<std::uint64_t> ts_client_send_us;
  std::optional<std::vector<::lowlevel::BodyJointCmd>> joints;
  std::optional<::lowlevel::BodyCmdAction> action;
  std::optional<::lowlevel::RootQuatWxyz> root_quat;
  std::optional<std::uint64_t> runtime_id;
  std::optional<std::uint64_t> control_id;
};
#endif  // AORTA_TYPED_WRAPPER_TABLE_MSG_LOWLEVEL_BODYCMDMSG_DEFINED

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

namespace typed_lowlevel_body_cmd {

inline flatbuffers::Offset<aorta::sys::AortaHeader> BuildAortaHeaderTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::lowlevel::AortaHeaderMsg& msg,
    flatbuffers::Offset<aorta::sys::AortaHeader> header);
inline flatbuffers::Offset<lowlevel::BodyCmd> BuildBodyCmdTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::lowlevel::BodyCmdMsg& msg,
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

inline flatbuffers::Offset<lowlevel::BodyCmd> BuildBodyCmdTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::lowlevel::BodyCmdMsg& msg,
    flatbuffers::Offset<aorta::sys::AortaHeader> header) {
  flatbuffers::Offset<flatbuffers::Vector<const ::lowlevel::BodyJointCmd *>> joints_offset;
  if (msg.joints.has_value()) {
    joints_offset = builder.CreateVectorOfStructs(msg.joints.value());
  }
  ::lowlevel::BodyCmdBuilder root_builder(builder);
  if (header.o != 0) {
    root_builder.add_aorta_header(header);
  }
  if (msg.protocol_version.has_value()) {
    root_builder.add_protocol_version(msg.protocol_version.value());
  }
  if (msg.session_id.has_value()) {
    root_builder.add_session_id(msg.session_id.value());
  }
  if (msg.sequence.has_value()) {
    root_builder.add_sequence(msg.sequence.value());
  }
  if (msg.input_state_id.has_value()) {
    root_builder.add_input_state_id(msg.input_state_id.value());
  }
  if (msg.input_state_ts_real_us.has_value()) {
    root_builder.add_input_state_ts_real_us(msg.input_state_ts_real_us.value());
  }
  if (msg.ts_client_created_us.has_value()) {
    root_builder.add_ts_client_created_us(msg.ts_client_created_us.value());
  }
  if (msg.ts_client_send_us.has_value()) {
    root_builder.add_ts_client_send_us(msg.ts_client_send_us.value());
  }
  if (joints_offset.o != 0) {
    root_builder.add_joints(joints_offset);
  }
  if (msg.action.has_value()) {
    root_builder.add_action(msg.action.value());
  }
  if (msg.root_quat.has_value()) {
    root_builder.add_root_quat(&msg.root_quat.value());
  }
  if (msg.runtime_id.has_value()) {
    root_builder.add_runtime_id(msg.runtime_id.value());
  }
  if (msg.control_id.has_value()) {
    root_builder.add_control_id(msg.control_id.value());
  }
  return root_builder.Finish();
}

}  // namespace typed_lowlevel_body_cmd
}  // namespace detail

template <>
struct TypedPublishTraits<::lowlevel::BodyCmd> {
  using Msg = ::lowlevel::BodyCmdMsg;
  static constexpr bool kHasAortaHeader = true;

  static flatbuffers::Offset<lowlevel::BodyCmd> Build(
      flatbuffers::FlatBufferBuilder& builder,
      flatbuffers::Offset<aorta::sys::AortaHeader> header,
      const Msg& msg) {
    return detail::typed_lowlevel_body_cmd::BuildBodyCmdTyped(builder, msg, header);
  }
};

template <>
struct TypedSubscribeTraits<::lowlevel::BodyCmd> {
  static bool Verify(const uint8_t* data, std::size_t size) {
    if (data == nullptr || size == 0) {
      return false;
    }
    flatbuffers::Verifier verifier(data, size);
    return ::lowlevel::VerifyBodyCmdBuffer(verifier);
  }

  static const lowlevel::BodyCmd* GetRoot(const uint8_t* data) {
    if (data == nullptr) {
      return nullptr;
    }
    return ::lowlevel::GetBodyCmd(data);
  }
};

}  // namespace aorta
