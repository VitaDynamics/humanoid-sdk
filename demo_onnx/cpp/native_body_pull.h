#pragma once

#include <aorta.h>
#include <pybind11/pybind11.h>

#include <memory>
#include <utility>
#include <vector>

namespace body_pull {
namespace py = pybind11;

inline void Check(aorta_result_t rc) {
  if (rc == AORTA_OK) return;
  auto error = py::module_::import("aorta.errors")
                   .attr("exception_for_code")(rc, "native Body pull");
  PyErr_SetObject(reinterpret_cast<PyObject*>(Py_TYPE(error.ptr())), error.ptr());
  throw py::error_already_set();
}

class Slots : public std::enable_shared_from_this<Slots> {
 public:
  explicit Slots(py::object binding) : binding_(std::move(binding)) {
    receive_ = reinterpret_cast<decltype(&aorta_pull_slot_try_recv)>(
        binding_.attr("try_recv_address").cast<uintptr_t>());
    release_ = reinterpret_cast<decltype(&aorta_pull_borrow_release)>(
        binding_.attr("release_address").cast<uintptr_t>());
    close_ = reinterpret_cast<decltype(&aorta_pull_slot_try_close)>(
        binding_.attr("try_close_address").cast<uintptr_t>());
    free_ = reinterpret_cast<decltype(&aorta_pull_slot_free)>(
        binding_.attr("free_address").cast<uintptr_t>());
    if (!receive_ || !release_ || !close_ || !free_)
      throw py::value_error("invalid public native RX binding");
    const auto count = binding_.attr("slot_count").cast<size_t>();
    if (count < 2)
      throw py::value_error("Body keep-last plus candidate requires at least two RX slots");
    // Allocate every local slot record BEFORE acquiring any real Core handle.
    slots_.resize(count);
    try {
      for (auto& slot : slots_) {
        slot.handle = reinterpret_cast<aorta_pull_slot_t>(
            binding_.attr("acquire_slot")().cast<uintptr_t>());
        if (!slot.handle) throw py::value_error("null native RX slot");
      }
    } catch (...) {
      // No borrow is exposed during cold construction. Each acquired handle
      // has this one owner, so the original drained free is sufficient.
      for (auto& slot : slots_)
        if (slot.handle) free_(slot.handle);
      throw;
    }
  }
  Slots(const Slots&) = delete;
  Slots& operator=(const Slots&) = delete;
  ~Slots() { Close(); }

  void Stop() { closed_ = true; }
  aorta_result_t Close() {
    closed_ = true;
    if (active_) return AORTA_ERR_BUSY;
    if (release_error_ != AORTA_OK) return release_error_;
    aorta_result_t first = AORTA_OK;
    for (auto& slot : slots_) {
      if (!slot.handle) continue;
      aorta_result_t rc = close_(slot.handle);
      if (rc == AORTA_OK) rc = free_(slot.handle);
      if (rc == AORTA_OK) slot.handle = nullptr;
      else if (first == AORTA_OK) first = rc;
    }
    return first;
  }

  size_t Available() const {
    if (closed_) Check(AORTA_ERR_NOT_FOUND);
    for (size_t i = 0; i < slots_.size(); ++i)
      if (!slots_[i].borrow.borrow_id) return i;
    Check(AORTA_ERR_BUSY);  // Do not pop or misreport capacity as TIMEOUT.
    return 0;
  }
  aorta_result_t Receive(size_t index) {
    auto& slot = slots_.at(index);
    ++active_;
    auto rc = receive_(slot.handle, &slot.borrow);
    --active_;
    // A real release callback may synchronously re-enter close. The original
    // Core gate and this admission check must both precede exposure.
    if (rc == AORTA_OK && closed_) {
      Release(index);
      return AORTA_ERR_NOT_FOUND;
    }
    return rc;
  }
  void Release(size_t index) noexcept {
    auto& token = slots_[index].borrow;
    if (!token.borrow_id) return;
    ++active_;
    const auto rc = release_(&token);
    --active_;
    if (rc != AORTA_OK) {
      release_error_ = rc;
      closed_ = true;
      // Never lose the original token/Core after an unsuccessful release.
      // Explicit close reports the original error; this bounded failed owner
      // remains retained rather than falsely reporting drained or unloading.
      failed_owner_ = shared_from_this();
    }
  }
  const aorta_pull_borrow_t& Borrow(size_t index) const {
    return slots_.at(index).borrow;
  }

 private:
  struct Slot {
    aorta_pull_slot_t handle = nullptr;
    aorta_pull_borrow_t borrow{};
  };
  // The one public binding owns the SAME library and original receiver.
  py::object binding_;
  decltype(&aorta_pull_slot_try_recv) receive_ = nullptr;
  decltype(&aorta_pull_borrow_release) release_ = nullptr;
  decltype(&aorta_pull_slot_try_close) close_ = nullptr;
  decltype(&aorta_pull_slot_free) free_ = nullptr;
  std::vector<Slot> slots_;
  bool closed_ = false;
  size_t active_ = 0;
  aorta_result_t release_error_ = AORTA_OK;
  std::shared_ptr<Slots> failed_owner_;
};

class Sample {
 public:
  Sample(std::shared_ptr<Slots> slots, size_t index)
      : slots_(std::move(slots)), index_(index) {}
  Sample(const Sample&) = delete;
  Sample& operator=(const Sample&) = delete;
  ~Sample() { slots_->Release(index_); }
  std::pair<const uint8_t*, size_t> Bytes() const {
    const auto& sample = slots_->Borrow(index_);
    if (!sample.borrow_id) throw py::value_error("native RX borrow was released");
    if (sample.payload.slice_count != 1 || !sample.payload.slices)
      throw py::value_error("native Body decoder requires a contiguous sample; multi-slice remains unsupported");
    const auto& slice = sample.payload.slices[0];
    if (slice.len != sample.payload.total_len || (!slice.data && slice.len))
      throw py::value_error("invalid native RX contiguous span");
    return {slice.data, slice.len};
  }
  int64_t ReceiveStamp() const {
    return slots_->Borrow(index_).context.receive_stamp_ns;
  }

 private:
  std::shared_ptr<Slots> slots_;
  size_t index_;
};

class Reader {
 public:
  explicit Reader(py::object binding)
      : slots_(std::make_shared<Slots>(std::move(binding))) {}
  Reader(const Reader&) = delete;
  Reader& operator=(const Reader&) = delete;
  ~Reader() { slots_->Stop(); }
  py::object TryReceive() {
    const auto index = slots_->Available();
    const auto rc = slots_->Receive(index);
    if (rc == AORTA_ERR_TIMEOUT) return py::none();
    Check(rc);
    struct Pending {
      Slots& slots;
      size_t index;
      bool armed = true;
      ~Pending() { if (armed) slots.Release(index); }
    } pending{*slots_, index};
    // Empty probes allocate no sample owner. A successful borrow remains
    // stack-owned until shared ownership is established, including bad_alloc.
    auto sample = std::make_shared<Sample>(slots_, index);
    pending.armed = false;
    return py::cast(std::move(sample));
  }
  void Stop() { slots_->Stop(); }
  aorta_result_t Close() { return slots_->Close(); }

 private:
  std::shared_ptr<Slots> slots_;
};
}  // namespace body_pull
