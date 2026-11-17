// Internal implementation included by trace.cpp after the queue/session types.
// All Python access is synchronous under the GIL. Only owned POD reaches the
// writer; no Python references, callbacks, or control locks cross that
// boundary.
#pragma once

class SdkCapture : public std::enable_shared_from_this<SdkCapture> {
public:
  SdkCapture(Session &session, uint32_t producer,
             std::array<uint32_t, locomotion_trace::StageCount> stages,
             std::array<uint64_t, 2> cookie)
      : state_(session.SharedState()), producer_(producer),
        native_stages_(stages), cookie_(cookie) {
    if (producer >= state_->producers.size())
      throw py::value_error("unknown producer");
    if (!busy_.is_lock_free())
      throw py::value_error("capture guard atomics are not lock-free");
  }
  py::capsule SinkCapsule() {
    sink_.owner = this;
    sink_.command = [](void *p, const Record &wanted, const Record &wire,
                       uint64_t begin, uint64_t end) noexcept {
      static_cast<SdkCapture *>(p)->NativeCommand(wanted, wire, begin, end);
    };
    sink_.decoded = [](void *p, const Record &data,
                       locomotion_trace::Stage stage, uint64_t group) noexcept {
      return static_cast<SdkCapture *>(p)->NativeDecoded(data, stage, group);
    };
    sink_.inference = [](void *p, uint64_t begin, uint64_t end,
                         bool failed) noexcept {
      static_cast<SdkCapture *>(p)->NativeInference(begin, end, failed);
    };
    auto keep =
        std::make_unique<std::shared_ptr<SdkCapture>>(shared_from_this());
    py::capsule capsule(&sink_, locomotion_trace::kCapsule,
                        [](PyObject *value) {
                          delete static_cast<std::shared_ptr<SdkCapture> *>(
                              PyCapsule_GetContext(value));
                        });
    if (PyCapsule_SetContext(capsule.ptr(), keep.get()))
      throw py::error_already_set();
    keep.release();
    return capsule;
  }
  void NativeCommand(const Record &wanted, const Record &wire, uint64_t begin,
                     uint64_t end) noexcept {
    Guard guard(*this);
    if (!guard)
      return;
    try {
      if (call_depth_ && calls_[call_depth_ - 1].kind == wanted.kind) {
        auto &call = calls_[call_depth_ - 1];
        call.command = wanted;
        call.has_command = true;
      }
      auto r = wire;
      r.kind = 8;
      r.stage = native_stages_[wanted.kind == 4 ? locomotion_trace::BodyEncode
                                                : locomotion_trace::EEEncode];
      r.timestamp_us = begin;
      r.duration_valid = end >= begin;
      r.duration_us = end >= begin ? end - begin + 1 : 0;
      r.outcome = 1;
      Context(r);
      Emit(r);
    } catch (...) {
      Invalid();
    }
  }
  locomotion_trace::Reference NativeDecoded(const Record &data,
                                            locomotion_trace::Stage stage,
                                            uint64_t group) noexcept {
    Guard guard(*this);
    if (!guard)
      return {};
    try {
      if (stage >= locomotion_trace::StageCount ||
          data.n_times < (stage == locomotion_trace::TouchDecode ? 1 : 2)) {
        Invalid();
        return {};
      }
      auto r = data;
      r.stage = native_stages_[stage];
      r.timestamp_us = NowUs();
      r.outcome = 0;
      const auto id = group ? group : Identity();
      if (!id)
        return {};
      Context(r);
      r.ids[5] = r.ids[4];
      r.ids[4] = id;
      Emit(r);
      const uint64_t kind =
          stage == locomotion_trace::TouchDecode ? 11 : data.kind;
      const uint64_t publication = stage == locomotion_trace::TouchDecode
                                       ? data.times_us[data.n_times - 1]
                                       : data.times_us[1];
      return {{cookie_[0], cookie_[1]},
              id,
              kind,
              data.ids[6],
              data.ids[10],
              publication};
    } catch (...) {
      Invalid();
      return {};
    }
  }
  void NativeInference(uint64_t begin, uint64_t end, bool failed) noexcept {
    Guard guard(*this);
    if (!guard)
      return;
    try {
      auto r = Event(8, native_stages_[locomotion_trace::Inference], 0,
                     failed ? 3 : 1);
      r.timestamp_us = begin;
      r.duration_valid = end >= begin;
      r.duration_us = end >= begin ? end - begin + 1 : 0;
      if (!r.ids[1])
        r.ids[1] = Identity();
      Emit(r);
    } catch (...) {
      Invalid();
    }
  }

  bool Active() const { return busy_.load() || call_depth_ || span_depth_; }
  uint64_t AllocateId() {
    Guard guard(*this);
    return guard ? Identity() : 0;
  }
  uint64_t Vector(uint32_t stage, py::buffer source, uint64_t iteration,
                  uint64_t inference, uint64_t reference) {
    Guard guard(*this);
    if (!guard) return 0;
    try {
      const auto b = source.request();
      const bool f32 = b.format == py::format_descriptor<float>::format() && b.itemsize == 4;
      const bool f64 = b.format == py::format_descriptor<double>::format() && b.itemsize == 8;
      if (b.ndim != 1 || b.size < 0 || b.size > 8192 || b.strides[0] != b.itemsize || (!f32 && !f64))
        throw py::value_error("trace vector requires a contiguous float32/64 buffer with at most 8192 values");
      const auto id = Identity(), stamp = NowUs();
      if (!id) return 0;
      for (size_t offset = 0; offset == 0 || offset < size_t(b.size); offset += kValues) {
        auto r = Event(16, stage, 0, 1);
        Context(r);
        r.timestamp_us = stamp;
        r.ids[4] = id;
        if (iteration) r.ids[0] = iteration;
        if (inference) r.ids[1] = inference;
        r.ids[11] = reference;
        Integer(r, offset);
        Integer(r, b.size);
        const auto n = std::min(kValues, size_t(b.size)-offset);
        for (size_t i = 0; i < n; ++i) {
          double value;
          if (f32) {
            float input;
            std::memcpy(&input, static_cast<const char *>(b.ptr)+(offset+i)*4, 4);
            value = input;
          } else {
            std::memcpy(&value, static_cast<const char *>(b.ptr)+(offset+i)*8, 8);
          }
          r.values[i] = value;
          r.validity[i] = std::isfinite(value);
        }
        r.n_values = r.n_validity = n;
        Emit(r);
      }
      return id;
    } catch (const std::exception &) {
      Invalid();
      return 0;
    }
  }
  uint64_t Mark(uint16_t kind, uint32_t stage, uint32_t stream, uint8_t outcome,
                uint64_t iteration, uint64_t inference, uint64_t reference,
                py::object values, py::object integers, py::object times,
                py::object validity) {
    Guard guard(*this);
    if (!guard)
      return 0;
    try {
      if ((kind != 9 && kind != 11 && kind != 12 && kind != 16 &&
           kind != 17 && kind != 18 && kind != 20) || outcome > 11)
        throw py::value_error("invalid user trace event kind/outcome");
      auto r = Event(kind, stage, stream, outcome);
      Context(r);
      r.ids[4] = Identity();
      if (iteration) r.ids[0] = iteration;
      if (inference) r.ids[1] = inference;
      r.ids[11] = reference;
      r.n_values = Session::CopyValues(values, r.values);
      r.n_integers = Session::CopyBuffer(integers, r.integers);
      r.n_times = Session::CopyBuffer(times, r.times_us);
      r.n_validity = Session::CopyBuffer(validity, r.validity);
      Emit(r);
      return r.ids[4];
    } catch (const std::exception &) {
      Invalid();
      return 0;
    }
  }
  // Explicit transferable context, not a TLS span pushed on the submitter.
  std::array<uint64_t, 7> AsyncBegin(uint32_t stage, uint64_t iteration,
                                    uint64_t inference, uint64_t parent) {
    Guard guard(*this);
    if (!guard) return {};
    const auto id = Identity();
    return {id, NowUs(), stage, iteration, inference ? inference : id,
            parent, NativeThreadToken()};
  }
  void AsyncEnd(const std::array<uint64_t, 7> &context, bool failed) {
    Guard guard(*this);
    if (!guard) return;
    auto r = Event(15, context[2], 0, failed ? 3 : 1);
    if (!context[0] || r.timestamp_us < context[1]) {
      Invalid();
      return;
    }
    r.duration_us = r.timestamp_us - context[1] + 1;
    r.duration_valid = true;
    r.timestamp_us = context[1];
    r.ids[0] = context[3];
    r.ids[1] = context[4];
    r.ids[2] = context[0];
    r.ids[3] = context[5];
    // Keep the originating thread lifetime token as explicit provenance.
    Integer(r, context[6]);
    Emit(r);
  }
  void Invalid() {
    if (!state_->closing.load())
      state_->producers[producer_]->invalid.Add();
  }
  uint64_t Begin(uint16_t kind, uint32_t stage, uint32_t stream) {
    Guard guard(*this);
    if (!guard)
      return 0;
    if (call_depth_ == calls_.size()) {
      Invalid();
      return 0;
    }
    const auto id = Identity();
    if (!id)
      return 0;
    auto r = Event(kind, stage, stream, 0);
    r.ids[4] = id;
    auto &f = calls_[call_depth_++];
    static_cast<Frame &>(f) = {
        id,    r.timestamp_us, kind,
        stage, stream,         {r.ids[0], r.ids[1], r.ids[2], r.ids[3]}};
    f.has_command = false;
    Emit(r);
    return id;
  }
  void End(uint64_t token, bool failed, py::object subject, py::object required,
           py::object targets, py::dict indices) {
    if (!token)
      return;
    Guard guard(*this);
    if (!guard)
      return;
    if (!call_depth_ || calls_[call_depth_ - 1].id != token) {
      Invalid();
      return;
    }
    // Keep owned POD while reading return-key scalar getters. The guard counts
    // nested capture as loss without preventing the original nested operation.
    const CallFrame f = calls_[--call_depth_];
    auto r = Event(f.kind, f.stage, f.stream, failed ? 3 : 1);
    for (size_t i = 0; i < 4; ++i)
      r.ids[i] = f.span[i];
    r.ids[4] = f.id;
    if (r.timestamp_us >= f.start) {
      r.duration_valid = true;
      r.duration_us = r.timestamp_us - f.start + 1;
    } else
      Invalid();
    try {
      if (!failed) {
        if ((f.kind == 4 || f.kind == 6) && f.has_command) {
          const auto key = f.kind == 6 ? subject.attr("key") : subject;
          if (U(subject, "runtime_id") != f.command.ids[6] ||
              U(subject, "control_id") != f.command.ids[7] ||
              U(key, "session_id") != f.command.ids[8] ||
              U(key, "sequence") != f.command.ids[9]) {
            Invalid();
            return;
          }
          const auto point = r.timestamp_us, elapsed = r.duration_us;
          const auto ids = r.ids;
          r = f.command;
          r.timestamp_us = point;
          r.duration_us = elapsed;
          r.duration_valid = true;
          r.outcome = 1;
          r.stage = f.stage;
          for (size_t i = 0; i < 5; ++i)
            r.ids[i] = ids[i];
          if (f.kind == 4)
            BodyScope(r, required, targets, indices);
        } else if (f.kind == 4)
          BodyCommand(r, subject, required, targets, indices);
        else if (f.kind == 3)
          BodyState(r, subject);
        else if (f.kind == 6)
          EECommand(r, subject);
        else if (f.kind == 10)
          Status(r, subject);
      }
      Emit(r);
      if (!failed && f.kind == 6) {
        for (auto ref : subject.attr("input_observations")) {
          auto link = Event(17, f.stage, f.stream, 1);
          CommandKey(link, subject);
          link.ids[10] = U(ref, "sample_id");
          for (const char *name : {"runtime_id", "stream", "side", "validity"})
            Integer(link, U(ref, name));
          Time(link, U(ref, "source_time_us"));
          Emit(link);
        }
      }
    } catch (const std::exception &) {
      Invalid();
    }
  }
  uint64_t BeginSpan(uint32_t stage, uint64_t iteration, uint64_t inference) {
    Guard guard(*this);
    if (!guard)
      return 0;
    if (span_depth_ == spans_.size()) {
      Invalid();
      return 0;
    }
    const uint64_t id = Identity();
    if (!id)
      return 0;
    const uint64_t parent = span_depth_ ? spans_[span_depth_ - 1].id : 0;
    spans_[span_depth_++] = {
        id,    NowUs(), 15,
        stage, 0,       {iteration, inference ? inference : id, id, parent}};
    return id;
  }
  void EndSpan(uint64_t token, bool failed) {
    if (!token)
      return;
    Guard guard(*this);
    if (!guard)
      return;
    if (!span_depth_ || spans_[span_depth_ - 1].id != token) {
      Invalid();
      return;
    }
    const Frame f = spans_[--span_depth_];
    const uint64_t end = NowUs();
    if (end < f.start) {
      Invalid();
      return;
    }
    auto r = Event(15, f.stage, 0, failed ? 3 : 1);
    r.timestamp_us = f.start;
    r.duration_valid = true;
    r.duration_us = end - f.start + 1;
    for (size_t i = 0; i < 4; ++i)
      r.ids[i] = f.span[i];
    Emit(r);
  }
  void Reserved(py::object context, uint32_t side, uint32_t stage) {
    Guard guard(*this);
    if (!guard)
      return;
    try {
      auto r = Event(8, stage, 1 + side, 1);
      r.ids[4] = call_depth_ ? calls_[call_depth_ - 1].id : 0;
      r.ids[6] = U(context, "runtime_id");
      r.ids[7] = U(context, "control_id");
      r.ids[8] = U(context, "session_id");
      r.ids[9] = U(context, "sequence");
      Emit(r);
    } catch (const std::exception &) {
      Invalid();
    }
  }
  void BodyFeedback(py::object subject, uint32_t stage, bool accepted) {
    Guard guard(*this);
    if (!guard)
      return;
    try {
      auto r = Event(5, stage, 1, accepted ? 1 : 2);
      BodyState(r, subject);
      Emit(r);
    } catch (const std::exception &) {
      Invalid();
    }
  }
  void EndpointFeedback(py::object subject, uint32_t role, py::object received,
                        uint32_t stage, bool accepted) {
    Guard guard(*this);
    if (!guard)
      return;
    try {
      auto r = Event(7, stage, 1 + U(subject, "side"), accepted ? 1 : 2);
      if (Link(r, subject, 7)) {
        Integer(r, role);
        Time(r, received.is_none() ? 0 : SecondsUs(received));
        Emit(r);
        return;
      }
      r.ids[6] = U(subject, "runtime_id");
      r.ids[10] = U(subject, "state_id");
      auto fields = subject.attr("valid_fields").cast<py::sequence>();
      Integer(r, fields.size());
      for (const char *name :
           {"side", "layout_id", "sample_status", "status_valid_mask", "ready",
            "output_allowed", "inhibit_reason", "touch_enabled"})
        Integer(r, U(subject, name));
      Integer(r, role);
      for (auto field : fields)
        Integer(r, field.cast<uint64_t>());
      const char *names[] = {"pos",     "vel",           "tau",
                             "current", "temperature_c", "error_code"};
      const uint8_t bits[] = {1, 2, 8, 16, 32, 128};
      for (size_t c = 0; c < 6; ++c)
        Column(r, subject.attr(names[c]), fields, bits[c]);
      for (const char *name :
           {"ts_state_real_us", "ts_state_pub_us", "ts_acquired_hw_us",
            "ts_ingested_us", "ts_assembled_us"})
        Time(r, U(subject, name));
      Time(r, received.is_none() ? 0 : SecondsUs(received));
      Emit(r);
    } catch (const std::exception &) {
      Invalid();
    }
  }
  void Touch(py::object subject, uint32_t stage, bool accepted) {
    Guard guard(*this);
    if (!guard)
      return;
    try {
      auto linked = Event(11, stage, 1 + U(subject, "side"), accepted ? 1 : 2);
      if (Link(linked, subject, 11)) {
        Integer(linked, U(subject, "layout_id"));
        Integer(linked, U(subject, "side"));
        Integer(linked, U(subject, "publish_time_valid"));
        Time(linked, U(subject, "ts_state_pub_us"));
        Emit(linked);
        return;
      }
      const auto runtime = U(subject, "runtime_id"),
                 sample = U(subject, "touch_state_id");
      const auto side = U(subject, "side"), layout = U(subject, "layout_id");
      size_t index = 0;
      for (auto region : subject.attr("region_stamps")) {
        auto r = Event(2, stage, 1 + side, 0);
        r.ids[6] = runtime;
        r.ids[10] = sample;
        Integer(r, layout);
        Integer(r, side);
        Integer(r, index++);
        for (const char *name :
             {"sample_seq", "status_flags", "direction_flags", "validity"})
          Integer(r, U(region, name));
        Integer(r, U(subject, "publish_time_valid"));
        for (const char *name : {"ts_request_us", "ts_acquired_hw_us",
                                 "ts_source_us", "ts_ingested_us"})
          Time(r, U(region, name));
        Time(r, U(subject, "ts_state_pub_us"));
        Emit(r);
      }
      const auto payload = subject.attr("payload").cast<py::bytes>();
      char *bytes;
      Py_ssize_t size;
      if (PyBytes_AsStringAndSize(payload.ptr(), &bytes, &size))
        throw py::error_already_set();
      for (size_t offset = 0; offset < size_t(size); offset += 216) {
        auto r = Event(11, stage, 1 + side, 0);
        r.ids[6] = runtime;
        r.ids[10] = sample;
        Integer(r, layout);
        Integer(r, side);
        Integer(r, offset);
        Integer(r, size);
        Words(r, bytes + offset, std::min(size_t(216), size_t(size) - offset));
        Emit(r);
      }
    } catch (const std::exception &) {
      Invalid();
    }
  }
  void Info(py::bytes raw, uint32_t stage, uint16_t kind) {
    Guard guard(*this);
    if (!guard)
      return;
    try {
      char *bytes;
      Py_ssize_t size;
      if (PyBytes_AsStringAndSize(raw.ptr(), &bytes, &size))
        throw py::error_already_set();
      if (size > 256 * 1024 || (kind != 2 && kind != 13 && kind != 14))
        throw py::value_error("Info exceeds cold bound");
      const auto id = Identity();
      for (size_t offset = 0; offset < size_t(size); offset += 216) {
        auto r = Event(kind, stage, 0, 1);
        r.ids[4] = id;
        Integer(r, offset);
        Integer(r, size);
        Words(r, bytes + offset, std::min(size_t(216), size_t(size) - offset));
        Emit(r);
      }
    } catch (const std::exception &) {
      Invalid();
    }
  }

private:
  struct Frame {
    uint64_t id{}, start{};
    uint16_t kind{};
    uint32_t stage{}, stream{};
    std::array<uint64_t, 4> span{};
  };
  struct CallFrame : Frame {
    bool has_command{false};
    Record command{};
  };
  struct Guard {
    explicit Guard(SdkCapture &capture) : c(capture) {
      if (c.state_->closing.load())
        return;
      auto &p = *c.state_->producers[c.producer_];
      pid_t zero = 0, tid = static_cast<pid_t>(syscall(SYS_gettid));
      p.owner.compare_exchange_strong(zero, tid, std::memory_order_relaxed);
      if (p.owner.load(std::memory_order_relaxed) != tid) {
        p.wrong_thread.Add();
        c.Invalid();
        return;
      }
      bool expected = false;
      held = c.busy_.compare_exchange_strong(expected, true);
      if (!held)
        c.Invalid();
    }
    ~Guard() {
      if (held)
        c.busy_.store(false);
    }
    explicit operator bool() const { return held; }
    SdkCapture &c;
    bool held{false};
  };
  uint64_t Identity() {
    if (next_id_ >= (uint64_t(1) << 56)) {
      Invalid();
      return 0;
    }
    return ((uint64_t(producer_) + 1) << 56) | next_id_++;
  }
  void Context(Record &r) const {
    if (span_depth_)
      for (size_t i = 0; i < 4; ++i)
        r.ids[i] = spans_[span_depth_ - 1].span[i];
    r.ids[4] = call_depth_ ? calls_[call_depth_ - 1].id : 0;
  }
  bool Link(Record &r, py::handle subject, uint64_t kind) {
    if (!py::hasattr(subject, "_trace_ref"))
      return false;
    auto value = subject.attr("_trace_ref");
    if (!py::isinstance<py::tuple>(value) || py::len(value) != 7)
      return false;
    auto ref = value.cast<py::tuple>();
    if (ref[0].cast<uint64_t>() != cookie_[0] ||
        ref[1].cast<uint64_t>() != cookie_[1] ||
        ref[3].cast<uint64_t>() != kind)
      return false;
    const auto runtime = U(subject, "runtime_id"),
               sample = U(subject, kind == 11 ? "touch_state_id" : "state_id");
    if (runtime != ref[4].cast<uint64_t>() ||
        sample != ref[5].cast<uint64_t>() ||
        U(subject, "ts_state_pub_us") != ref[6].cast<uint64_t>())
      return false;
    r.ids[6] = runtime;
    r.ids[10] = sample;
    r.ids[11] = ref[2].cast<uint64_t>();
    return true;
  }
  Record Event(uint16_t kind, uint32_t stage, uint32_t stream,
               uint8_t outcome) {
    Record r{};
    r.timestamp_us = NowUs();
    r.kind = kind;
    r.stage = stage;
    r.stream = stream;
    r.outcome = outcome;
    if (span_depth_) {
      for (size_t i = 0; i < 4; ++i)
        r.ids[i] = spans_[span_depth_ - 1].span[i];
    }
    return r;
  }
  void Emit(Record &r) {
    if (r.n_values + r.n_times > kValidity) {
      Invalid();
      return;
    }
    for (size_t i = 0; i < r.n_times; ++i)
      r.validity[r.n_values + i] = r.times_us[i] != 0;
    r.n_validity = r.n_values + r.n_times;
    SubmitRecord(*state_, producer_, [&](Record &slot) { slot = r; });
  }
  static uint64_t U(py::handle source, const char *field) {
    return source.attr(field).cast<uint64_t>();
  }
  static uint64_t OptionalU(py::handle source, const char *field) {
    return py::hasattr(source, field) ? U(source, field) : 0;
  }
  static uint64_t SecondsUs(py::handle source) {
    const double value = source.cast<double>() * 1000000.;
    if (!std::isfinite(value) || value < 0 || value >= double(UINT64_MAX))
      throw py::value_error("invalid receipt time");
    return static_cast<uint64_t>(value);
  }
  static void Integer(Record &r, uint64_t value) {
    if (r.n_integers == kIntegers)
      throw py::value_error("trace integer bound");
    r.integers[r.n_integers++] = value;
  }
  static void Time(Record &r, uint64_t value) {
    if (r.n_times == kTimes)
      throw py::value_error("trace time bound");
    r.times_us[r.n_times++] = value;
  }
  static void Number(Record &r, py::handle input, bool valid = true) {
    if (r.n_values == kValues)
      throw py::value_error("trace value bound");
    const double value = valid ? input.cast<double>() : 0;
    if (valid && (PyBool_Check(input.ptr()) || !std::isfinite(value)))
      throw py::value_error("invalid trace numeric value");
    r.validity[r.n_values] = valid;
    r.values[r.n_values++] = value;
  }
  static void Column(Record &r, py::object column, py::sequence fields,
                     uint8_t bit) {
    for (size_t i = 0; i < fields.size(); ++i) {
      const bool valid = fields[i].cast<uint64_t>() & bit;
      if (valid)
        Number(r, column[py::int_(i)]);
      else
        Number(r, py::none(), false);
    }
  }
  static void Words(Record &r, const char *bytes, size_t size) {
    for (size_t i = 0; i < size; i += 8) {
      uint64_t word = 0;
      for (size_t j = 0; j < 8 && i + j < size; ++j)
        word |= uint64_t(uint8_t(bytes[i + j])) << (j * 8);
      Integer(r, word);
    }
  }
  static void CommandKey(Record &r, py::handle command) {
    r.ids[6] = U(command, "runtime_id");
    r.ids[7] = U(command, "control_id");
    const auto key = command.attr("key");
    r.ids[8] = U(key, "session_id");
    r.ids[9] = U(key, "sequence");
  }
  static void BodyScope(Record &r, py::object required, py::object targets,
                        py::dict indices) {
    Integer(r, required.is_none() ? 0 : required.cast<uint64_t>());
    Integer(r, !required.is_none());
    uint64_t slots = 0;
    if (!targets.is_none())
      for (auto name : targets) {
        const auto index = indices[name].cast<uint64_t>();
        if (index >= 31)
          throw py::value_error("invalid target index");
        slots |= uint64_t(1) << index;
      }
    Integer(r, slots);
    Integer(r, !targets.is_none());
  }
  static void BodyCommand(Record &r, py::handle command, py::object required,
                          py::object targets, py::dict indices) {
    r.ids[6] = U(command, "runtime_id");
    r.ids[7] = U(command, "control_id");
    r.ids[8] = U(command, "session_id");
    r.ids[9] = U(command, "sequence");
    r.ids[10] = U(command, "input_state_id");
    const auto joints = command.attr("joints");
    if (py::len(joints) != 31)
      throw py::value_error("Body trace requires 31 slots");
    Integer(r, 31);
    Integer(r, U(command, "protocol_version"));
    BodyScope(r, required, targets, indices);
    for (const char *field : {"q", "dq", "tau", "kp", "kd"})
      for (auto joint : joints)
        Number(r, joint.attr(field));
    for (const char *field : {"ts_client_created_us", "ts_client_send_us",
                              "input_state_ts_real_us"})
      Time(r, U(command, field));
  }
  template <class T>
  static void NumericBuffer(Record &r, const py::buffer_info &b) {
    const auto *data = static_cast<const char *>(b.ptr);
    for (py::ssize_t i = 0; i < b.size; ++i) {
      T item;
      std::memcpy(&item, data + i * sizeof(T), sizeof(T));
      const double value = item;
      if (r.n_values == kValues || !std::isfinite(value))
        throw py::value_error("invalid trace column");
      r.validity[r.n_values] = 1;
      r.values[r.n_values++] = value;
    }
  }
  static bool BufferColumn(Record &r, py::object column) {
    if (!PyObject_CheckBuffer(column.ptr()))
      return false;
    const auto b = py::buffer(column).request();
    if (b.ndim != 1 || b.size != 31 || b.strides[0] != b.itemsize)
      return false;
    if (b.format == py::format_descriptor<double>::format() &&
        b.itemsize == sizeof(double))
      NumericBuffer<double>(r, b);
    else if (b.format == py::format_descriptor<float>::format() &&
             b.itemsize == sizeof(float))
      NumericBuffer<float>(r, b);
    else if (b.format == py::format_descriptor<uint32_t>::format() &&
             b.itemsize == sizeof(uint32_t))
      NumericBuffer<uint32_t>(r, b);
    else if (U64Buffer(b))
      NumericBuffer<uint64_t>(r, b);
    else
      return false;
    return true;
  }
  static bool U64Buffer(const py::buffer_info &b) {
    // NumPy uses native unsigned long (L) on LP64; array('Q') uses Q.
    // Non-native endian prefixes deliberately take the numeric fallback.
    return b.itemsize == 8 && (b.format == "Q" || b.format == "L");
  }
  static bool StampBuffer(Record &r, py::object stamps) {
    if (!PyObject_CheckBuffer(stamps.ptr()))
      return false;
    const auto b = py::buffer(stamps).request();
    if (b.ndim != 2 || b.shape[0] != 31 || b.shape[1] != 3 ||
        b.strides[0] != 24 || b.strides[1] != 8 || !U64Buffer(b))
      return false;
    const auto *data = static_cast<const char *>(b.ptr);
    for (size_t i = 0; i < 93; ++i) {
      uint64_t value;
      std::memcpy(&value, data + i * 8, 8);
      Time(r, value);
    }
    return true;
  }
  void BodyState(Record &r, py::handle subject) {
    if (Link(r, subject, 5)) {
      r.ids[8] = U(subject, "applied_session_id");
      r.ids[9] = U(subject, "applied_sequence");
      for (const char *f : {"received_monotonic_ns", "received_realtime_ns",
                            "pull_context_receive_stamp_ns"})
        Time(r, OptionalU(subject, f) / 1000);
      return;
    }
    r.ids[6] = U(subject, "runtime_id");
    r.ids[8] = U(subject, "applied_session_id");
    r.ids[9] = U(subject, "applied_sequence");
    r.ids[10] = U(subject, "state_id");
    Integer(r, 31);
    Integer(r, U(subject, "applied_key_valid"));
    Integer(r, U(subject, "association_source_type"));
    for (const char *field : {"q", "dq", "tau_est", "error_code"}) {
      if (py::hasattr(subject, field) && !subject.attr(field).is_none()) {
        const auto column = subject.attr(field).cast<py::object>();
        if (py::len(column) != 31)
          throw py::value_error("Body trace requires 31 slots");
        if (!BufferColumn(r, column))
          for (auto value : column)
            Number(r, value);
      } else {
        const auto joints = subject.attr("joints");
        if (py::len(joints) != 31)
          throw py::value_error("Body trace requires 31 slots");
        for (auto joint : joints)
          Number(r, joint.attr(field));
      }
    }
    for (const char *field :
         {"ts_state_real_us", "ts_state_pub_us", "ts_consumed_us",
          "ts_control_us", "ts_first_output_us"})
      Time(r, U(subject, field));
    for (const char *field : {"received_monotonic_ns", "received_realtime_ns",
                              "pull_context_receive_stamp_ns"})
      Time(r, OptionalU(subject, field) / 1000);
    if (py::hasattr(subject, "joint_stamps_us") &&
        !subject.attr("joint_stamps_us").is_none()) {
      const auto stamps = subject.attr("joint_stamps_us").cast<py::object>();
      if (!StampBuffer(r, stamps))
        for (auto row : stamps)
          for (auto value : row)
            Time(r, value.cast<uint64_t>());
    } else
      for (auto joint : subject.attr("joints")) {
        for (const char *field :
             {"ts_acquired_hw_us", "ts_ingested_us", "ts_assembled_us"})
          Time(r, U(joint, field));
      }
  }
  static void EECommand(Record &r, py::handle subject) {
    CommandKey(r, subject);
    const auto target = subject.attr("target");
    const auto modes = target.attr("mode").cast<py::sequence>();
    const auto fields = target.attr("command_fields").cast<py::sequence>();
    Integer(r, modes.size());
    const char *names[] = {"pos", "vel", "vel_limit", "tau", "current"};
    const uint8_t bits[] = {1, 2, 4, 8, 16};
    for (size_t c = 0; c < 5; ++c)
      Column(r, target.attr(names[c]), fields, bits[c]);
    for (auto mode : modes)
      Integer(r, mode.cast<uint64_t>());
    for (auto field : fields)
      Integer(r, field.cast<uint64_t>());
    Time(r, U(subject, "ts_client_created_us"));
    Time(r, U(subject, "ts_client_send_us"));
  }
  static void Status(Record &r, py::handle subject) {
    r.ids[6] = U(subject, "runtime_id");
    r.ids[7] = U(subject, "control_id");
    for (const char *field :
         {"client_id", "generation", "status_seq", "reject_count",
          "ingress_drop_count", "timeout_hold"})
      Integer(r, U(subject, field));
  }
  std::shared_ptr<State> state_;
  const uint32_t producer_;
  const std::array<uint32_t, locomotion_trace::StageCount> native_stages_;
  const std::array<uint64_t, 2> cookie_;
  locomotion_trace::Sink sink_{};
  std::atomic<bool> busy_{false};
  uint64_t next_id_{1};
  std::array<CallFrame, 8> calls_{};
  std::array<Frame, 8> spans_{};
  size_t call_depth_{0}, span_depth_{0};
};

void BindSdkCapture(py::module_ &m) {
  py::class_<SdkCapture, std::shared_ptr<SdkCapture>>(m, "SdkCapture")
      .def(py::init<Session &, uint32_t,
                    std::array<uint32_t, locomotion_trace::StageCount>,
                    std::array<uint64_t, 2>>())
      .def("sink", &SdkCapture::SinkCapsule)
      .def_property_readonly("active", &SdkCapture::Active)
      .def("begin", &SdkCapture::Begin)
      .def("end", &SdkCapture::End)
      .def("begin_span", &SdkCapture::BeginSpan)
      .def("end_span", &SdkCapture::EndSpan)
      .def("allocate_id", &SdkCapture::AllocateId)
      .def("vector", &SdkCapture::Vector)
      .def("mark", &SdkCapture::Mark)
      .def("async_begin", &SdkCapture::AsyncBegin)
      .def("async_end", &SdkCapture::AsyncEnd)
      .def("reserved", &SdkCapture::Reserved)
      .def("body_feedback", &SdkCapture::BodyFeedback)
      .def("endpoint_feedback", &SdkCapture::EndpointFeedback)
      .def("touch", &SdkCapture::Touch)
      .def("info", &SdkCapture::Info, py::arg("raw"), py::arg("stage"), py::arg("kind") = 2);
}
