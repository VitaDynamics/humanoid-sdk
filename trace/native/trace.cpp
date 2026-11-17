#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#include <pthread.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <zlib.h>
#include <zstd.h>

#include "codec_bridge.h"
#include "trace_generated.h"

namespace py = pybind11;
namespace vt = vita::trace;
namespace {
using locomotion_trace::kValues;
using locomotion_trace::kTimes;
using locomotion_trace::kIntegers;
using locomotion_trace::kValidity;
using locomotion_trace::Integer;
using locomotion_trace::Time;
using locomotion_trace::NowUs;
constexpr size_t kIds = 12, kMaxBatch = 64;
constexpr uint32_t kMaxBlock = 4 * 1024 * 1024;
constexpr char kMagic[] = "FTRACE13";

uint64_t NativeThreadToken() {
  // ctypes may recreate PyThreadState on every foreign callback. Native TLS
  // survives those callbacks but not the OS thread's lifetime. Tokens are
  // process-unique, so recycled OS/Python thread IDs cannot inherit a ring.
  static std::atomic<uint64_t> next{1};
  thread_local const uint64_t token = [] {
    auto value = next.load(std::memory_order_relaxed);
    while (value != UINT64_MAX) {
      if (next.compare_exchange_weak(value, value + 1,
                                     std::memory_order_relaxed))
        return value;
    }
    return uint64_t{0};
  }();
  return token;
}

struct Counter {
  std::atomic<uint64_t> value{0};
  std::atomic<bool> valid{true};
  void Add(uint64_t amount = 1) noexcept {
    const uint64_t old = value.fetch_add(amount, std::memory_order_relaxed);
    if (old > UINT64_MAX - amount) valid.store(false, std::memory_order_relaxed);
  }
};

// All storage is owned by the ring. No Python references survive Submit().
using Record = locomotion_trace::Record;
static_assert(std::is_trivially_copyable_v<Record>);

struct Producer {
  explicit Producer(size_t capacity) : capacity(capacity), slots(new Record[capacity]) {}
  const size_t capacity;
  std::unique_ptr<Record[]> slots;
  alignas(64) std::atomic<uint64_t> head{0};
  alignas(64) std::atomic<uint64_t> tail{0};
  std::atomic<pid_t> owner{0};
  Counter attempted, enqueued, dropped, persisted, wrong_thread, invalid;
  uint64_t next_sequence{0};  // The single registered producer writes this.
  std::atomic<uint64_t> peak{0};
};

struct State {
  std::vector<std::unique_ptr<Producer>> producers;
  std::atomic<bool> closing{false}, failed{false}, finished{false}, ready{false};
  std::atomic<uint32_t> active_submissions{0};
  std::mutex completion_mutex;
  std::condition_variable completion;
  std::string error;
  int fd{-1};
  bool compress{};
  uint64_t flush_us{}, poll_us{};
  std::vector<int> requested_cpus;
  int requested_nice{};
  ~State() { if (fd >= 0) ::close(fd); }
};

void WriteAll(int fd, const void* pointer, size_t length) {
  auto* data = static_cast<const uint8_t*>(pointer);
  while (length) {
    const auto written = ::write(fd, data, length);
    if (written < 0 && errno == EINTR) continue;
    if (written <= 0) throw std::system_error(errno, std::generic_category(), "trace write");
    data += written;
    length -= static_cast<size_t>(written);
  }
}

template <class T> void Put(std::array<uint8_t, 32>& header, size_t offset, T value) {
  for (size_t i = 0; i < sizeof(T); ++i) header[offset + i] = uint8_t(value >> (8 * i));
}

bool Offset(uint64_t value, uint64_t base, uint32_t* out) {
  if (value < base || value - base > UINT32_MAX) return false;
  *out = static_cast<uint32_t>(value - base);
  return true;
}

flatbuffers::Offset<vt::Event> EncodeEvent(flatbuffers::FlatBufferBuilder& b,
                                         uint32_t producer, const Record& r,
                                         uint64_t base) {
  const auto numbers = b.CreateVector(r.values.data(), r.n_values);
  const auto integers = b.CreateVector(r.integers.data(), r.n_integers);
  const auto validity = b.CreateVector(r.validity.data(), r.n_validity);
  std::array<uint32_t, kTimes> offsets{};
  std::array<uint16_t, kTimes> extended_indices{};
  std::array<uint64_t, kTimes> extended_values{};
  size_t extended_count = 0;
  for (uint16_t i = 0; i < r.n_times; ++i) {
    if (!Offset(r.times_us[i], base, &offsets[i])) {
      extended_indices[extended_count] = i;
      extended_values[extended_count++] = r.times_us[i];
    }
  }
  const auto times = b.CreateVector(offsets.data(), r.n_times);
  const auto time_indices = b.CreateVector(extended_indices.data(), extended_count);
  const auto time_values = b.CreateVector(extended_values.data(), extended_count);
  uint32_t timestamp = 0;
  const bool extended = r.time_valid && !Offset(r.timestamp_us, base, &timestamp);
  vt::EventBuilder e(b);
  e.add_kind(static_cast<vt::RecordKind>(r.kind));
  e.add_producer(producer); e.add_sequence(r.sequence); e.add_stream(r.stream);
  e.add_stage(r.stage); e.add_clock(r.clock); e.add_epoch(r.epoch);
  e.add_time_offset_us(timestamp); e.add_time_valid(r.time_valid);
  e.add_time_extended(extended);
  if (extended) e.add_time_extended_us(r.timestamp_us);
  e.add_duration_valid(r.duration_valid);
  if (r.duration_us > UINT32_MAX) {
    e.add_duration_extended(true); e.add_duration_extended_us(r.duration_us);
  } else {
    e.add_duration_us(static_cast<uint32_t>(r.duration_us));
  }
  e.add_outcome(static_cast<vt::Outcome>(r.outcome));
  e.add_iteration_id(r.ids[0]); e.add_inference_id(r.ids[1]);
  e.add_span_id(r.ids[2]); e.add_parent_span_id(r.ids[3]);
  e.add_attempt_id(r.ids[4]); e.add_group_id(r.ids[5]);
  e.add_runtime_id(r.ids[6]); e.add_control_id(r.ids[7]);
  e.add_session_id(r.ids[8]); e.add_command_sequence(r.ids[9]);
  e.add_state_id(r.ids[10]); e.add_reference_id(r.ids[11]);
  e.add_values(numbers); e.add_integers(integers); e.add_validity(validity);
  e.add_time_offsets_us(times); e.add_time_extended_indices(time_indices);
  e.add_time_extended_values_us(time_values);
  return e.Finish();
}

struct Pending { uint32_t producer; Record record; };

void WriteChunk(State& s, flatbuffers::FlatBufferBuilder& b, uint64_t sequence,
                const std::vector<Pending>& pending, const std::string& metadata = {}) {
  b.Clear();
  uint64_t base = UINT64_MAX;
  for (const auto& p : pending) if (p.record.time_valid) base = std::min(base, p.record.timestamp_us);
  if (base == UINT64_MAX) base = 0;
  std::vector<flatbuffers::Offset<vt::Event>> events;
  events.reserve(pending.size());  // Writer-only allocation, never a producer.
  for (const auto& p : pending) events.push_back(EncodeEvent(b, p.producer, p.record, base));
  const auto event_vector = b.CreateVector(events);
  const auto meta = b.CreateString(metadata);
  vt::ChunkBuilder chunk(b);
  chunk.add_major(1); chunk.add_minor(3); chunk.add_sequence(sequence);
  chunk.add_base_us(base); chunk.add_events(event_vector); chunk.add_metadata(meta);
  vt::FinishChunkBuffer(b, chunk.Finish());
  const auto raw_size = b.GetSize();
  if (raw_size > kMaxBlock) throw std::runtime_error("trace block exceeds declared bound");
  std::vector<uint8_t> compressed;
  const uint8_t* bytes = b.GetBufferPointer();
  size_t size = raw_size;
  if (s.compress) {
    compressed.resize(ZSTD_compressBound(raw_size));
    size = ZSTD_compress(compressed.data(), compressed.size(), bytes, raw_size, 1);
    if (ZSTD_isError(size)) throw std::runtime_error("trace Zstd encoding failed");
    bytes = compressed.data();
  }
  std::array<uint8_t, 32> header{};
  Put(header, 0, uint32_t(size)); Put(header, 4, uint32_t(raw_size));
  Put(header, 8, uint32_t(crc32(0, b.GetBufferPointer(), raw_size)));
  header[12] = s.compress ? 1 : 0;
  Put(header, 16, sequence); Put(header, 24, base);
  WriteAll(s.fd, header.data(), header.size());
  WriteAll(s.fd, bytes, size);
  // Only a whole, successfully written, verifiable block counts as persisted.
  for (const auto& p : pending) s.producers[p.producer]->persisted.Add();
}

std::string Health(const State& s, const char* type) {
  std::string json = "{\"record_type\":\"" + std::string(type) + "\",\"producers\":[";
  bool comma = false;
  for (const auto& p : s.producers) {
    if (comma) json += ',';
    comma = true;
    const uint64_t enqueued = p->enqueued.value.load(), persisted = p->persisted.value.load();
    json += "{\"attempted_events\":" + std::to_string(p->attempted.value.load()) +
      ",\"enqueued_events\":" + std::to_string(enqueued) +
      ",\"dropped_events\":" + std::to_string(p->dropped.value.load()) +
      ",\"persisted_events\":" + std::to_string(persisted) +
      ",\"unpersisted_events\":" + std::to_string(enqueued >= persisted ? enqueued - persisted : 0) +
      ",\"wrong_thread_events\":" + std::to_string(p->wrong_thread.value.load()) +
      ",\"invalid_events\":" + std::to_string(p->invalid.value.load()) +
      ",\"queue_peak_packets\":" + std::to_string(p->peak.load()) +
      ",\"capacity_packets\":" + std::to_string(p->capacity) +
      ",\"slot_bytes\":" + std::to_string(sizeof(Record)) +
      ",\"counters_valid\":" +
      (p->attempted.valid.load() && p->enqueued.valid.load() && p->dropped.valid.load() &&
       p->persisted.valid.load() && p->wrong_thread.valid.load() && p->invalid.valid.load() ? "true" : "false") + "}";
  }
  json += "],\"failed\":" + std::string(s.failed.load() ? "true" : "false") + "}";
  return json;
}

void ConfigureWriter(State& s) {
  if (!s.requested_cpus.empty()) {
    cpu_set_t allowed, requested;
    CPU_ZERO(&allowed); CPU_ZERO(&requested);
    if (pthread_getaffinity_np(pthread_self(), sizeof(allowed), &allowed))
      throw std::runtime_error("cannot inspect writer cpuset");
    for (int cpu : s.requested_cpus) {
      if (cpu < 0 || cpu >= CPU_SETSIZE || !CPU_ISSET(cpu, &allowed))
        throw std::runtime_error("writer CPU is outside allowed cpuset");
      CPU_SET(cpu, &requested);
    }
    if (pthread_setaffinity_np(pthread_self(), sizeof(requested), &requested))
      throw std::runtime_error("cannot set writer affinity");
  }
  // Linux setpriority on the writer TID, not the entire process.
  if (s.requested_nice && setpriority(PRIO_PROCESS, syscall(SYS_gettid), s.requested_nice))
    throw std::runtime_error("cannot set writer priority");
}

void RunWriter(std::shared_ptr<State> s, std::string metadata) noexcept {
  try {
    ConfigureWriter(*s);
    flatbuffers::FlatBufferBuilder builder(256 * 1024);
    std::vector<Pending> pending;
    pending.reserve(kMaxBatch);
    uint64_t sequence = 0, last_flush = NowUs();
    WriteChunk(*s, builder, sequence++, pending, metadata);
    s->ready.store(true, std::memory_order_release);
    s->completion.notify_all();
    for (;;) {
      bool any = false;
      for (uint32_t i = 0; i < s->producers.size(); ++i) {
        auto& p = *s->producers[i];
        const uint64_t tail = p.tail.load(std::memory_order_relaxed);
        const uint64_t head = p.head.load(std::memory_order_acquire);
        if (tail == head) continue;
        pending.push_back({i, p.slots[tail % p.capacity]});
        p.tail.store(tail + 1, std::memory_order_release);
        any = true;
        if (pending.size() == kMaxBatch) {
          WriteChunk(*s, builder, sequence++, pending); pending.clear();
          last_flush = NowUs();
        }
      }
      const uint64_t now = NowUs();
      if (!pending.empty() && now - last_flush >= s->flush_us) {
        WriteChunk(*s, builder, sequence++, pending); pending.clear(); last_flush = now;
      }
      if (s->closing.load(std::memory_order_seq_cst) &&
          !s->active_submissions.load(std::memory_order_seq_cst)) {
        bool empty = true;
        for (const auto& p : s->producers)
          empty = empty && p->head.load(std::memory_order_acquire) == p->tail.load();
        if (empty) break;
      }
      if (!any) std::this_thread::sleep_for(std::chrono::microseconds(s->poll_us));
    }
    if (!pending.empty()) { WriteChunk(*s, builder, sequence++, pending); pending.clear(); }
    WriteChunk(*s, builder, sequence, pending, Health(*s, "run_end"));
  } catch (const std::exception& e) {
    s->error = e.what();
    s->failed.store(true, std::memory_order_release);
  } catch (...) {
    s->error = "unknown trace writer failure";
    s->failed.store(true, std::memory_order_release);
  }
  {
    std::lock_guard<std::mutex> lock(s->completion_mutex);
    s->finished.store(true, std::memory_order_release);
  }
  s->completion.notify_all();
}

template <class Fill>
bool SubmitRecord(State &s, uint32_t producer, Fill &&fill) {
  if (producer >= s.producers.size())
    throw py::value_error("unknown producer");
  auto &p = *s.producers[producer];
  s.active_submissions.fetch_add(1, std::memory_order_seq_cst);
  struct InFlight {
    State *s;
    ~InFlight() {
      s->active_submissions.fetch_sub(1, std::memory_order_seq_cst);
    }
  } in_flight{&s};
  // Closing establishes the end of the accounting window. A racing call
  // either belongs to that window or returns without modifying its ledger.
  if (s.closing.load(std::memory_order_seq_cst))
    return false;
  p.attempted.Add();
  const pid_t tid = static_cast<pid_t>(syscall(SYS_gettid));
  pid_t owner = 0;
  p.owner.compare_exchange_strong(owner, tid, std::memory_order_relaxed);
  if (p.owner.load(std::memory_order_relaxed) != tid) {
    p.wrong_thread.Add();
    p.dropped.Add();
    return false;
  }
  if (p.next_sequence == UINT64_MAX) {
    p.attempted.valid.store(false);
    p.dropped.Add();
    return false;
  }
  const uint64_t sequence = p.next_sequence++;
  if (s.failed.load(std::memory_order_acquire)) {
    p.dropped.Add();
    return false;
  }
  const uint64_t head = p.head.load(std::memory_order_relaxed);
  const uint64_t tail = p.tail.load(std::memory_order_acquire);
  if (head - tail >= p.capacity) {
    p.dropped.Add();
    return false;
  }
  Record &r = p.slots[head % p.capacity];
  try {
    fill(r);
  } catch (...) {
    p.dropped.Add();
    throw;
  }
  r.sequence = sequence;
  p.enqueued.Add();
  p.head.store(head + 1, std::memory_order_release);
  const uint64_t size = head - tail + 1;
  if (size > p.peak.load(std::memory_order_relaxed))
    p.peak.store(size, std::memory_order_relaxed);
  return true;
}

class Session {
 public:
  Session(const std::string& path, size_t producers, size_t capacity, bool compress,
          const std::string& metadata, uint64_t flush_us, uint64_t poll_us,
          const std::vector<int>& cpus, int nice, size_t memory_budget)
      : s_(std::make_shared<State>()) {
    if (!producers || producers > 32 || capacity < 2 || capacity > 65536 ||
        !flush_us || flush_us > 10000000 || !poll_us || poll_us > 100000 ||
        nice < 0 || nice > 19 || metadata.size() > 256 * 1024)
      throw py::value_error("invalid bounded trace configuration");
    if (producers * capacity > memory_budget / sizeof(Record))
      throw py::value_error("trace queues exceed declared memory budget");
    if (!s_->closing.is_lock_free() || !s_->active_submissions.is_lock_free())
      throw std::runtime_error("trace atomics are not lock-free on this host");
    for (size_t i = 0; i < producers; ++i) {
      auto p = std::make_unique<Producer>(capacity);
      if (!p->head.is_lock_free() || !p->owner.is_lock_free())
        throw std::runtime_error("trace producer atomics are not lock-free");
      s_->producers.push_back(std::move(p));
    }
    s_->compress = compress; s_->flush_us = flush_us; s_->poll_us = poll_us;
    s_->requested_cpus = cpus; s_->requested_nice = nice;
    s_->fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
    if (s_->fd < 0) throw std::system_error(errno, std::generic_category(), "create trace");
    WriteAll(s_->fd, kMagic, 8);
    worker_ = std::thread(RunWriter, s_, metadata);
  }
  ~Session() { Close(200000); }

  bool Submit(uint32_t producer, uint16_t kind, uint64_t timestamp_us,
              uint64_t duration_us, bool duration_valid, uint32_t stream,
              uint32_t stage, uint32_t clock, uint64_t epoch, uint8_t outcome,
              const std::array<uint64_t, kIds>& ids, py::object values,
              py::object integers, py::object times_us, py::object validity,
              bool time_valid) {
    if (!kind || kind > 21 || outcome > 11) throw py::value_error("unknown trace kind/outcome");
    return SubmitRecord(*s_, producer, [&](Record& r) {
      r.kind = kind; r.timestamp_us = timestamp_us;
      r.time_valid = time_valid; r.duration_us = duration_us; r.duration_valid = duration_valid;
      r.stream = stream; r.stage = stage; r.clock = clock; r.epoch = epoch;
      r.outcome = outcome; r.ids = ids;
      r.n_values = CopyValues(values, r.values);
      r.n_integers = CopyBuffer(integers, r.integers);
      r.n_times = CopyBuffer(times_us, r.times_us);
      r.n_validity = CopyBuffer(validity, r.validity);
    });
  }

  bool Close(uint64_t timeout_us) {
    s_->closing.store(true, std::memory_order_seq_cst);
    if (!worker_.joinable()) return s_->finished.load(std::memory_order_acquire);
    std::unique_lock<std::mutex> lock(s_->completion_mutex);
    const bool done = s_->completion.wait_for(lock, std::chrono::microseconds(timeout_us),
      [&] { return s_->finished.load(std::memory_order_acquire); });
    lock.unlock();
    if (done) worker_.join();
    else worker_.detach();  // Worker owns State/FD/queues until it really exits.
    return done;
  }
  std::shared_ptr<State> SharedState() const { return s_; }
  std::string HealthJson() const { return Health(*s_, "recorder_health"); }
  void NoteInvalid(uint32_t producer) {
    if (producer < s_->producers.size() && !s_->closing.load())
      s_->producers[producer]->invalid.Add();
  }
  bool WaitReady(uint64_t timeout_us) {
    std::unique_lock<std::mutex> lock(s_->completion_mutex);
    s_->completion.wait_for(lock, std::chrono::microseconds(timeout_us), [&] {
      return s_->ready.load(std::memory_order_acquire) || s_->finished.load(std::memory_order_acquire);
    });
    return s_->ready.load(std::memory_order_acquire);
  }
  py::dict Status() const {
    py::dict d;
    const bool finished = s_->finished.load(std::memory_order_acquire);
    d["finished"] = finished; d["failed"] = s_->failed.load(std::memory_order_acquire);
    d["error"] = finished ? s_->error : "";
    d["slot_bytes"] = sizeof(Record);
    return d;
  }

 private:
  friend class SdkCapture;
  template <class T, size_t N> static uint16_t CopyBuffer(py::object source, std::array<T, N>& dest) {
    if (source.is_none()) return 0;
    const auto b = py::buffer(source).request();
    if (b.ndim != 1 || b.size > static_cast<py::ssize_t>(N) ||
        b.itemsize != sizeof(T) || b.strides[0] != sizeof(T) ||
        b.format != py::format_descriptor<T>::format())
      throw py::value_error("trace requires bounded contiguous native-endian typed buffers");
    std::memcpy(dest.data(), b.ptr, static_cast<size_t>(b.size) * sizeof(T));
    return static_cast<uint16_t>(b.size);
  }
  static uint16_t CopyValues(py::object source, std::array<double, kValues>& dest) {
    if (source.is_none()) return 0;
    const auto b = py::buffer(source).request();
    if (b.ndim != 1 || b.size > static_cast<py::ssize_t>(kValues) || b.strides[0] != b.itemsize)
      throw py::value_error("trace numeric values require bounded contiguous buffers");
    if (b.format == py::format_descriptor<double>::format() && b.itemsize == sizeof(double))
      std::memcpy(dest.data(), b.ptr, static_cast<size_t>(b.size) * sizeof(double));
    else if (b.format == py::format_descriptor<float>::format() && b.itemsize == sizeof(float)) {
      const auto* input = static_cast<const float*>(b.ptr);
      for (py::ssize_t i = 0; i < b.size; ++i) dest[static_cast<size_t>(i)] = input[i];
    } else throw py::value_error("trace values must be float32 or float64");
    return static_cast<uint16_t>(b.size);
  }
  std::shared_ptr<State> s_;
  std::thread worker_;
};

#include "sdk_capture.h"

py::dict DecodeChunk(py::bytes input) {
  std::string bytes = input;
  if (bytes.size() > kMaxBlock) throw py::value_error("trace block exceeds bound");
  flatbuffers::Verifier verifier(reinterpret_cast<const uint8_t*>(bytes.data()), bytes.size(), 32, 100000);
  if (!vt::VerifyChunkBuffer(verifier)) throw py::value_error("invalid trace FlatBuffer");
  const vt::Chunk* chunk = vt::GetChunk(bytes.data());
  if (chunk->major() != 1 || chunk->minor() != 3)
    throw py::value_error("unsupported trace format version");
  py::dict result;
  result["major"] = chunk->major(); result["minor"] = chunk->minor();
  result["sequence"] = chunk->sequence(); result["base_us"] = chunk->base_us();
  result["metadata"] = chunk->metadata() ? chunk->metadata()->str() : "";
  py::list events;
  if (chunk->events()) for (const auto* e : *chunk->events()) {
    if (e->kind() == vt::RecordKind::UNKNOWN || static_cast<uint16_t>(e->kind()) > 21 ||
        static_cast<uint8_t>(e->outcome()) > 11 ||
        (e->values() && e->values()->size() > kValues) ||
        (e->integers() && e->integers()->size() > kIntegers) ||
        (e->validity() && e->validity()->size() > kValidity) ||
        (e->time_offsets_us() && e->time_offsets_us()->size() > kTimes))
      throw py::value_error("trace record exceeds declared layout");
    py::dict row;
    row["kind"] = static_cast<uint16_t>(e->kind()); row["producer"] = e->producer();
    row["sequence"] = e->sequence(); row["stream"] = e->stream(); row["stage"] = e->stage();
    row["clock"] = e->clock(); row["epoch"] = e->epoch(); row["outcome"] = static_cast<uint8_t>(e->outcome());
    if (e->time_valid()) {
      if (!e->time_extended() && chunk->base_us() > UINT64_MAX - e->time_offset_us())
        throw py::value_error("trace time overflows uint64");
      row["timestamp_us"] = e->time_extended() ? e->time_extended_us() : chunk->base_us() + e->time_offset_us();
    } else row["timestamp_us"] = py::none();
    if (e->duration_valid()) row["duration_us"] = e->duration_extended() ? e->duration_extended_us() : uint64_t(e->duration_us());
    else row["duration_us"] = py::none();
    row["iteration_id"] = e->iteration_id(); row["inference_id"] = e->inference_id();
    row["span_id"] = e->span_id(); row["parent_span_id"] = e->parent_span_id();
    row["attempt_id"] = e->attempt_id(); row["group_id"] = e->group_id();
    row["runtime_id"] = e->runtime_id(); row["control_id"] = e->control_id();
    row["session_id"] = e->session_id(); row["command_sequence"] = e->command_sequence();
    row["state_id"] = e->state_id(); row["reference_id"] = e->reference_id();
    py::list values, integers, times, validity;
    if (e->values()) for (auto x : *e->values()) values.append(x);
    if (e->integers()) for (auto x : *e->integers()) integers.append(x);
    if (e->validity()) for (auto x : *e->validity()) validity.append(x);
    std::vector<uint64_t> restored;
    if (e->time_offsets_us()) for (auto x : *e->time_offsets_us()) {
      if (chunk->base_us() > UINT64_MAX - x) throw py::value_error("trace point time overflow");
      restored.push_back(chunk->base_us() + x);
    }
    const auto* indices = e->time_extended_indices();
    const auto* extensions = e->time_extended_values_us();
    if ((indices ? indices->size() : 0) != (extensions ? extensions->size() : 0))
      throw py::value_error("invalid extended trace times");
    uint16_t previous = 0;
    if (indices) for (uint32_t i = 0; i < indices->size(); ++i) {
      const auto index = indices->Get(i);
      if (index >= restored.size() || (i && index <= previous))
        throw py::value_error("duplicate/out-of-range extended trace time");
      previous = index; restored[index] = extensions->Get(i);
    }
    for (auto x : restored) times.append(x);
    row["values"] = values; row["integers"] = integers; row["times_us"] = times; row["validity"] = validity;
    events.append(row);
  }
  result["events"] = events;
  return result;
}
}  // namespace

PYBIND11_MODULE(_trace_native, m) {
  m.attr("FORMAT_MAJOR") = 1; m.attr("FORMAT_MINOR") = 3;
  m.attr("SLOT_BYTES") = sizeof(Record);
  m.attr("CAPTURE_API_VERSION") = 4;
  m.def("thread_token", &NativeThreadToken);
  m.def("monotonic_us", &NowUs);
  m.def("decode_chunk", &DecodeChunk);
  py::class_<Session>(m, "Session")
    .def(py::init<const std::string&, size_t, size_t, bool, const std::string&, uint64_t, uint64_t,
                  const std::vector<int>&, int, size_t>())
    .def("submit", &Session::Submit)
    .def("note_invalid", &Session::NoteInvalid)
    .def("wait_ready", &Session::WaitReady, py::call_guard<py::gil_scoped_release>())
    .def("close", &Session::Close, py::call_guard<py::gil_scoped_release>())
    .def("health_json", &Session::HealthJson)
    .def("status", &Session::Status);
  BindSdkCapture(m);
}
