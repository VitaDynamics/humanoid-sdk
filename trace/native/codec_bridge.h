#pragma once
// Private, same-wheel interface between native codecs/inference and the writer.
// The sink copies POD synchronously; no borrowed Python or codec buffer
// escapes.
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <pybind11/pybind11.h>
#include <stdexcept>

namespace locomotion_trace {
constexpr size_t kValues = 256, kTimes = 128, kIntegers = 32, kValidity = 256;
struct Record {
  uint64_t sequence{}, timestamp_us{}, duration_us{}, epoch{1};
  std::array<uint64_t, 12> ids{};
  std::array<double, kValues> values{};
  std::array<uint64_t, kTimes> times_us{};
  std::array<uint64_t, kIntegers> integers{};
  std::array<uint8_t, kValidity> validity{};
  uint32_t stream{}, stage{}, clock{};
  uint16_t kind{}, n_values{}, n_times{}, n_integers{}, n_validity{};
  uint8_t outcome{};
  bool time_valid{true}, duration_valid{};
};
enum Stage : uint32_t {
  BodyEncode,
  EEEncode,
  BodyDecode,
  EEDecode,
  TouchDecode,
  Inference,
  StageCount
};
struct Reference {
  uint64_t cookie[2]{}, event{}, kind{}, runtime{}, sample{}, publication{};
};
struct Sink {
  uint32_t version{1}, record_size{sizeof(Record)};
  void *owner{};
  void (*command)(void *, const Record &, const Record &, uint64_t,
                  uint64_t) noexcept {};
  Reference (*decoded)(void *, const Record &, Stage, uint64_t) noexcept {};
  void (*inference)(void *, uint64_t, uint64_t, bool) noexcept {};
};
constexpr const char *kCapsule = "locomotion_aorta.trace.codec_sink.v1";
inline Sink *GetSink(pybind11::handle capsule) {
  if (capsule.is_none())
    return nullptr;
  if (!PyCapsule_IsValid(capsule.ptr(), kCapsule))
    throw pybind11::value_error("invalid native trace sink");
  auto *sink =
      static_cast<Sink *>(PyCapsule_GetPointer(capsule.ptr(), kCapsule));
  if (!sink || sink->version != 1 || sink->record_size != sizeof(Record))
    throw pybind11::value_error("native trace sink version mismatch");
  return sink;
}
inline uint64_t NowUs() {
  return std::chrono::duration_cast<std::chrono::microseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
inline void Value(Record &r, double value, bool valid = true) {
  if (r.n_values == kValues)
    throw std::length_error("trace values overflow");
  r.values[r.n_values] = valid ? value : 0;
  r.validity[r.n_values++] = valid && std::isfinite(value);
}
inline void Integer(Record &r, uint64_t value) {
  if (r.n_integers == kIntegers)
    throw std::length_error("trace integers overflow");
  r.integers[r.n_integers++] = value;
}
inline void Time(Record &r, uint64_t value) {
  if (r.n_times == kTimes)
    throw std::length_error("trace times overflow");
  r.times_us[r.n_times++] = value;
}
inline pybind11::object ReferenceObject(const Reference &ref) {
  if (!ref.event)
    return pybind11::none();
  return pybind11::make_tuple(ref.cookie[0], ref.cookie[1], ref.event, ref.kind,
                              ref.runtime, ref.sample, ref.publication);
}
inline void Attach(pybind11::object dto, const Reference &ref) {
  if (ref.event) {
    auto name = pybind11::str("_trace_ref");
    auto value = ReferenceObject(ref);
    if (PyObject_GenericSetAttr(dto.ptr(), name.ptr(), value.ptr()))
      throw pybind11::error_already_set();
  }
}
} // namespace locomotion_trace
