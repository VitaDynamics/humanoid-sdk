// Internal implementation included by trace.cpp after the queue/session types.
// All Python access is synchronous under the GIL. Only owned POD reaches the
// writer; no Python references, callbacks, or control locks cross that
// boundary.
#pragma once

class SdkCapture {
public:
  SdkCapture(Session &session, uint32_t producer)
      : state_(session.SharedState()), producer_(producer) {
    if (producer >= state_->producers.size())
      throw py::value_error("unknown producer");
    if (!busy_.is_lock_free())
      throw py::value_error("capture guard atomics are not lock-free");
  }
  bool Active() const { return busy_.load() || call_depth_ || span_depth_; }
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
    f = {id,    r.timestamp_us, kind,
         stage, stream,         {r.ids[0], r.ids[1], r.ids[2], r.ids[3]}};
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
    const Frame f = calls_[--call_depth_];
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
        if (f.kind == 4)
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
  void BodyFeedback(py::object subject, uint32_t stage) {
    Guard guard(*this);
    if (!guard)
      return;
    try {
      auto r = Event(5, stage, 1, 1);
      BodyState(r, subject);
      Emit(r);
    } catch (const std::exception &) {
      Invalid();
    }
  }
  void EndpointFeedback(py::object subject, uint32_t role, py::object received,
                        uint32_t stage) {
    Guard guard(*this);
    if (!guard)
      return;
    try {
      auto r = Event(7, stage, 1 + U(subject, "side"), 0);
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
  void Touch(py::object subject, uint32_t stage) {
    Guard guard(*this);
    if (!guard)
      return;
    try {
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
  void Info(py::bytes raw, uint32_t stage) {
    Guard guard(*this);
    if (!guard)
      return;
    try {
      char *bytes;
      Py_ssize_t size;
      if (PyBytes_AsStringAndSize(raw.ptr(), &bytes, &size))
        throw py::error_already_set();
      if (size > 256 * 1024)
        throw py::value_error("Info exceeds cold bound");
      const auto id = Identity();
      for (size_t offset = 0; offset < size_t(size); offset += 216) {
        auto r = Event(2, stage, 0, 1);
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
  static void BodyState(Record &r, py::handle subject) {
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
  std::atomic<bool> busy_{false};
  uint64_t next_id_{1};
  std::array<Frame, 8> calls_{}, spans_{};
  size_t call_depth_{0}, span_depth_{0};
};

void BindSdkCapture(py::module_ &m) {
  py::class_<SdkCapture>(m, "SdkCapture")
      .def(py::init<Session &, uint32_t>())
      .def_property_readonly("active", &SdkCapture::Active)
      .def("begin", &SdkCapture::Begin)
      .def("end", &SdkCapture::End)
      .def("begin_span", &SdkCapture::BeginSpan)
      .def("end_span", &SdkCapture::EndSpan)
      .def("reserved", &SdkCapture::Reserved)
      .def("body_feedback", &SdkCapture::BodyFeedback)
      .def("endpoint_feedback", &SdkCapture::EndpointFeedback)
      .def("touch", &SdkCapture::Touch)
      .def("info", &SdkCapture::Info);
}
