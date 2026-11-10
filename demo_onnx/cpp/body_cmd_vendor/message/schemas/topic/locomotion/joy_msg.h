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
#include "joy_generated.h"
#include "joy_schema_meta.h"
// IWYU pragma: end_exports

namespace aorta {
template <typename T>
struct TypedPublishTraits;
}  // namespace aorta

namespace locomotion {

struct AortaHeaderMsg;
struct JoyMsg;

#ifndef AORTA_TYPED_WRAPPER_TABLE_MSG_LOCOMOTION_AORTAHEADERMSG_DEFINED
#define AORTA_TYPED_WRAPPER_TABLE_MSG_LOCOMOTION_AORTAHEADERMSG_DEFINED 1
struct AortaHeaderMsg {
  std::optional<std::uint64_t> publish_stamp_ns;
  std::optional<std::string> publisher_node;
  std::optional<std::uint64_t> sequence;
};
#endif  // AORTA_TYPED_WRAPPER_TABLE_MSG_LOCOMOTION_AORTAHEADERMSG_DEFINED

#ifndef AORTA_TYPED_WRAPPER_TABLE_MSG_LOCOMOTION_JOYMSG_DEFINED
#define AORTA_TYPED_WRAPPER_TABLE_MSG_LOCOMOTION_JOYMSG_DEFINED 1
struct JoyMsg {
  std::optional<std::vector<float>> axes;
  std::optional<std::vector<std::int32_t>> buttons;
};
#endif  // AORTA_TYPED_WRAPPER_TABLE_MSG_LOCOMOTION_JOYMSG_DEFINED

}  // namespace locomotion

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

namespace typed_locomotion_joy {

inline flatbuffers::Offset<aorta::sys::AortaHeader> BuildAortaHeaderTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::locomotion::AortaHeaderMsg& msg,
    flatbuffers::Offset<aorta::sys::AortaHeader> header);
inline flatbuffers::Offset<locomotion::Joy> BuildJoyTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::locomotion::JoyMsg& msg,
    flatbuffers::Offset<aorta::sys::AortaHeader> header);

inline flatbuffers::Offset<aorta::sys::AortaHeader> BuildAortaHeaderTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::locomotion::AortaHeaderMsg& msg,
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

inline flatbuffers::Offset<locomotion::Joy> BuildJoyTyped(
    flatbuffers::FlatBufferBuilder& builder,
    const ::locomotion::JoyMsg& msg,
    flatbuffers::Offset<aorta::sys::AortaHeader> header) {
  flatbuffers::Offset<flatbuffers::Vector<float>> axes_offset;
  if (msg.axes.has_value()) {
    axes_offset = builder.CreateVector(msg.axes.value().data(), msg.axes.value().size());
  }
  flatbuffers::Offset<flatbuffers::Vector<std::int32_t>> buttons_offset;
  if (msg.buttons.has_value()) {
    buttons_offset = builder.CreateVector(msg.buttons.value().data(), msg.buttons.value().size());
  }
  ::locomotion::JoyBuilder root_builder(builder);
  if (header.o != 0) {
    root_builder.add_aorta_header(header);
  }
  if (axes_offset.o != 0) {
    root_builder.add_axes(axes_offset);
  }
  if (buttons_offset.o != 0) {
    root_builder.add_buttons(buttons_offset);
  }
  return root_builder.Finish();
}

}  // namespace typed_locomotion_joy
}  // namespace detail

template <>
struct TypedPublishTraits<::locomotion::Joy> {
  using Msg = ::locomotion::JoyMsg;
  static constexpr bool kHasAortaHeader = true;

  static flatbuffers::Offset<locomotion::Joy> Build(
      flatbuffers::FlatBufferBuilder& builder,
      flatbuffers::Offset<aorta::sys::AortaHeader> header,
      const Msg& msg) {
    return detail::typed_locomotion_joy::BuildJoyTyped(builder, msg, header);
  }
};

template <>
struct TypedSubscribeTraits<::locomotion::Joy> {
  static bool Verify(const uint8_t* data, std::size_t size) {
    if (data == nullptr || size == 0) {
      return false;
    }
    flatbuffers::Verifier verifier(data, size);
    return ::locomotion::VerifyJoyBuffer(verifier);
  }

  static const locomotion::Joy* GetRoot(const uint8_t* data) {
    if (data == nullptr) {
      return nullptr;
    }
    return ::locomotion::GetJoy(data);
  }
};

}  // namespace aorta
