#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace production_mock {
constexpr size_t kSlots = 31;
struct JointLimits {
  float q_min, q_max, dq_min, dq_max, tau_min, tau_max;
  float kp_min, kp_max, kd_min, kd_max;
};
// Synthetic capabilities, shared by RobotInfo and admission. Real robots
// supply their per-joint firmware limits through this same public contract.
inline constexpr JointLimits kJointLimits{-10.f, 10.f, -50.f, 50.f, -100.f,
                                           100.f, 0.f, 600.f, 0.f, 60.f};
uint64_t NextApplyTick(uint64_t accepted_wall_ns, uint64_t period_ns);
struct Request {
  uint32_t version = 0;
  int operation = 0;
  std::string request_id, action;
  uint64_t client_id = 0, control_id = 0, runtime = 0;
};
struct Ack {
  std::string request_id, reason;
  int operation = 0, error = 0;
  bool accepted = false;
  uint64_t client_id = 0, control_id = 0, runtime = 0;
};
struct Status {
  uint64_t status_seq = 1, generation = 0, accepted_sequence = 0;
  uint64_t reject_count = 0, last_valid_ns = 0;
  int phase = 0, reject_code = 0;
  std::string request_id;
  uint64_t client_id = 0, control_id = 0, runtime = 0;
};
struct Command {
  uint64_t runtime = 0, control_id = 0;
  uint64_t session = 0, sequence = 0, input_state_id = 0;
  uint64_t input_state_real_us = 0, client_created_us = 0, client_send_us = 0;
  std::array<float, kSlots> q{}, dq{}, tau{}, kp{}, kd{};
};
// Synthetic EE fixture declarations and bounded endpoint snapshots. Permission
// remains exclusively in Protocol::status_; endpoint keys are not leases.
struct EEAxis {
  uint32_t modes = 0, command_fields = 0, state_fields = 0, valid_limits = 0;
  uint8_t position_domain = 0, torque_domain = 0;
  float pos_min = 0, pos_max = 0, vel_min = 0, vel_max = 0;
  float vel_limit_max = 0, tau_min = 0, tau_max = 0, current_min = 0, current_max = 0;
};
struct EEInfo {
  uint8_t side = 0;
  uint16_t device_model = 0;
  uint32_t layout_id = 0;
  bool touch_available = false;
  std::vector<EEAxis> axes;
  std::vector<std::pair<uint8_t, uint8_t>> observation_sources;
};
struct EEKey {
  uint64_t session = 0, sequence = 0;
};
struct EEObservation {
  uint64_t runtime = 0, sample_id = 0, source_us = 0;
  uint8_t stream = 0, side = 0, validity = 0;
};
struct EECommand {
  uint32_t version = 0, layout_id = 0;
  uint64_t runtime = 0;
  uint8_t side = 0;
  uint64_t client_created_us = 0, client_send_us = 0;
  uint64_t control_id = 0;
  EEKey key;
  std::vector<uint16_t> modes;
  std::vector<uint32_t> fields;
  // POS, VEL, VEL_LIMIT, TAU, CURRENT: one stored value per axis.
  std::array<std::vector<float>, 5> columns;
  std::vector<EEObservation> observations;
};
struct EESnapshot {
  EEInfo info;
  std::vector<float> position;
  std::vector<EEKey> output_keys;
  std::vector<bool> output_key_valid;
  EEKey received, accepted, applied;
  bool received_valid = false, accepted_valid = false, applied_valid = false;
  bool touch_enabled = false, output_allowed = false;
  uint8_t inhibit_reason = 1;
  uint16_t reject_code = 0;
  uint64_t reject_count = 0, overwritten = 0, timeouts = 0;
  uint64_t received_us = 0, accepted_us = 0, output_us = 0;
  uint64_t runtime = 0;
};
struct TouchRequest {
  uint32_t version = 0, layout_id = 0;
  uint64_t runtime = 0;
  uint8_t side = 0;
  uint64_t control_id = 0;
  bool enabled = false;
};
struct TouchReply {
  uint64_t runtime = 0;
  uint8_t side = 0;
  uint16_t reason = 0;
  bool success = false, enabled = false, enabled_valid = false;
};
struct Output {
  uint64_t runtime = 0, control_us = 0;
  bool applied_key_valid = false;
  uint8_t association_source_type = 0;
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
  void ConfigureEE(const EEInfo&, const std::vector<float>& initial_position);
  bool AcceptEE(const EECommand&, uint64_t now_ns, uint64_t wall_us);
  size_t TickEE(uint64_t now_ns, uint64_t wall_us);
  EESnapshot SnapshotEE(uint8_t side, uint64_t now_ns);
  TouchReply SetTouch(const TouchRequest&, uint64_t now_ns);
  uint64_t runtime_id() const { return runtime_id_; }

 private:
  Ack Reply(const Request&, int error, const std::string& reason) const;
  void InvalidateApplied();
  void Clear();
  bool OwnedEE(uint64_t control_id) const;
  void InvalidateEE();
  struct Endpoint {
    EESnapshot snapshot;
    EECommand pending;
    bool configured = false, has_pending = false, timed_out = false;
    uint64_t last_valid_ns = 0;
  };
  std::array<Endpoint, 2> endpoints_;
  uint64_t runtime_id_;
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
std::vector<uint8_t> EncodeInfo(uint64_t runtime);
bool ValidInfoRequest(const std::vector<uint8_t>&);
std::vector<uint8_t> EncodeState(uint64_t state_id, const Output&,
                                 uint64_t wall_us);
void StampState(std::vector<uint8_t>&, uint64_t wall_ns);
EEInfo DecodeEEInfo(const std::vector<uint8_t>&);
EECommand DecodeEECommand(const std::vector<uint8_t>&);
TouchRequest DecodeTouchRequest(const std::vector<uint8_t>&);
std::vector<uint8_t> EncodeEEInfo(const std::vector<uint8_t>& fixture,
                                 uint64_t runtime, uint64_t startup_us, uint64_t wall_us);
std::vector<uint8_t> EncodeEEState(const EESnapshot&, uint64_t sample_id, uint64_t wall_us);
std::vector<uint8_t> EncodeEETouch(const std::vector<uint8_t>& fixture,
                                  const EESnapshot&, uint64_t sample_id, uint64_t wall_us);
std::vector<uint8_t> EncodeTouchReply(const TouchReply&);
}  // namespace production_mock
