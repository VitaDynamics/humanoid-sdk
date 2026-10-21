#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace production_mock {
constexpr size_t kSlots = 31;
struct Request {
  uint32_t version = 0;
  int operation = 0;
  std::string request_id, client_id, lease_id, action;
};
struct Ack {
  std::string request_id, lease_id, reason;
  int operation = 0, error = 0;
  bool accepted = false;
};
struct Status {
  uint64_t status_seq = 1, generation = 0, accepted_sequence = 0;
  uint64_t reject_count = 0, last_valid_ns = 0;
  int phase = 0, reject_code = 0;
  std::string request_id, client_id, lease_id;
};
struct Command {
  uint64_t session = 0, sequence = 0, input_state_id = 0;
  uint64_t input_state_real_us = 0, client_created_us = 0, client_send_us = 0;
  std::array<float, kSlots> q{}, dq{}, tau{}, kp{}, kd{};
};
struct Output {
  uint64_t session = 0, sequence = 0, first_output_us = 0;
  std::array<float, kSlots> q{};
  std::array<std::array<float, 4>, 2> imu_q{
      {std::array<float, 4>{1, 0, 0, 0}, std::array<float, 4>{1, 0, 0, 0}}};
};
class Protocol {
 public:
  Protocol(uint64_t seed, uint64_t command_timeout_ns,
           uint64_t prepare_timeout_ns, int reject_operation = 0);
  Ack RequestControl(const Request&, uint64_t now_ns);
  bool Accept(const Command&, uint64_t now_ns);
  bool Tick(uint64_t now_ns, uint64_t wall_us);
  void Watchdog(uint64_t now_ns);
  const Status& status() const { return status_; }
  const Output& output() const { return output_; }
  void SetInitialQ(const std::array<float, kSlots>& q) { output_.q = q; }
  void SetInitialImu(const std::array<std::array<float, 4>, 2>& q) {
    output_.imu_q = q;
  }
  uint64_t overwritten() const { return overwritten_; }

 private:
  Ack Reply(const Request&, int error, const std::string& reason) const;
  void Clear();
  Status status_;
  Output output_;
  Command pending_;
  bool has_pending_ = false;
  uint64_t generation_, command_timeout_ns_, prepare_timeout_ns_,
      prepared_ns_ = 0;
  uint64_t overwritten_ = 0;
  int reject_operation_;
};

Request DecodeRequest(const std::vector<uint8_t>&);
std::vector<uint8_t> EncodeAck(const Ack&, uint64_t wall_ns,
                               uint64_t publication_id);
std::vector<uint8_t> EncodeStatus(const Status&, uint64_t now_ns,
                                  uint64_t wall_ns, uint64_t publication_id);
std::vector<uint8_t> EncodeInfo();
bool ValidInfoRequest(const std::vector<uint8_t>&);
std::vector<uint8_t> EncodeState(uint64_t state_id, const Output&,
                                 uint64_t wall_us);
void StampState(std::vector<uint8_t>&, uint64_t wall_ns);
}  // namespace production_mock
