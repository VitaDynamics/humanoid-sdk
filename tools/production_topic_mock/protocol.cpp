#include "protocol.h"

#include <cmath>
#include <cstdlib>
#include <limits>
#include <memory>
#include <stdexcept>

// BFBS --gen-all headers flatten shared types. Compile each codec separately,
// rather than including conflicting generated definitions in one translation
// unit.
#if defined(PRODUCTION_MOCK_REQUEST_CODEC)
#include "control_request_generated.h"
namespace production_mock {
Request DecodeRequest(const std::vector<uint8_t>& bytes) {
  flatbuffers::Verifier verifier(bytes.data(), bytes.size());
  if (!locomotion_sdk::VerifyControlRequestBuffer(verifier))
    throw std::runtime_error("invalid ControlRequest FlatBuffer");
  const auto* r = locomotion_sdk::GetControlRequest(bytes.data());
  auto str = [](const flatbuffers::String* s) {
    // Structural errors belong to the verifier. Identity errors must reach
    // RequestControl's1002 reply, including absent/overlong identifiers.
    return s ? s->str() : std::string{};
  };
  return {r->protocol_version(),
          static_cast<int>(r->operation()),
          str(r->request_id()),
          str(r->client_id()),
          r->lease_id() ? str(r->lease_id()) : "",
          r->action_name() ? str(r->action_name()) : ""};
}
}  // namespace production_mock
#elif defined(PRODUCTION_MOCK_ACK_CODEC)
#include "control_ack_generated.h"
namespace production_mock {
std::vector<uint8_t> EncodeAck(const Ack& r, uint64_t wall_ns,
                               uint64_t publication_id) {
  locomotion_sdk::ControlAckT ack;
  ack.protocol_version = 1;
  ack.operation = static_cast<decltype(ack.operation)>(r.operation);
  ack.request_id = r.request_id;
  ack.lease_id = r.lease_id;
  ack.accepted = r.accepted;
  ack.error_code = static_cast<decltype(ack.error_code)>(r.error);
  ack.reason = r.reason;
  ack.lease_deadline_monotonic_ns = 0;
  ack.aorta_header = std::make_unique<aorta::sys::AortaHeaderT>();
  ack.aorta_header->sequence = publication_id;
  ack.aorta_header->publish_stamp_ns = wall_ns;
  ack.aorta_header->publisher_node = "production_topic_mock";
  flatbuffers::FlatBufferBuilder b;
  b.Finish(locomotion_sdk::ControlAck::Pack(b, &ack));
  return {b.GetBufferPointer(), b.GetBufferPointer() + b.GetSize()};
}
}  // namespace production_mock
#elif defined(PRODUCTION_MOCK_STATUS_CODEC)
#include "control_status_generated.h"
namespace production_mock {
std::vector<uint8_t> EncodeStatus(const Status& s, uint64_t now_ns,
                                  uint64_t wall_ns, uint64_t publication_id) {
  locomotion_sdk::ControlStatusT status;
  status.protocol_version = 1;
  status.status_seq = s.status_seq;
  status.current_action =
      (s.phase == 4 || s.phase == 5) ? "EXTERNAL" : "PASSIVE";
  status.action_phase = static_cast<decltype(status.action_phase)>(s.phase);
  status.client_id = s.client_id;
  status.lease_id = s.lease_id;
  status.generation = s.generation;
  status.last_accepted_cmd_id =
      static_cast<uint16_t>(s.accepted_sequence & 0xffff);
  status.reject_count = s.reject_count;
  status.last_reject_code =
      static_cast<decltype(status.last_reject_code)>(s.reject_code);
  status.last_valid_age_ms = s.last_valid_ns && now_ns >= s.last_valid_ns
                                 ? (now_ns - s.last_valid_ns) / 1000000
                                 : 0;
  status.timeout_hold = s.phase == 5;
  status.stamp_monotonic_ns = now_ns;
  status.aorta_header = std::make_unique<aorta::sys::AortaHeaderT>();
  status.aorta_header->sequence = publication_id;
  status.aorta_header->publish_stamp_ns = wall_ns;
  status.aorta_header->publisher_node = "production_topic_mock";
  flatbuffers::FlatBufferBuilder b;
  b.Finish(locomotion_sdk::ControlStatus::Pack(b, &status));
  return {b.GetBufferPointer(), b.GetBufferPointer() + b.GetSize()};
}
}  // namespace production_mock
#elif defined(PRODUCTION_MOCK_INFO_CODEC)
#include "get_robot_info_generated.h"
namespace production_mock {
bool ValidInfoRequest(const std::vector<uint8_t>& bytes) {
  flatbuffers::Verifier verifier(bytes.data(), bytes.size());
  return aorta::services::humanoid_locomotion::VerifyGetRobotInfoRequestBuffer(
      verifier);
}
std::vector<uint8_t> EncodeInfo() {
  static const std::array<const char*, kSlots> names{
      "left_hip_pitch_joint",
      "left_hip_roll_joint",
      "left_hip_yaw_joint",
      "left_knee_joint",
      "left_ankle_pitch_joint",
      "left_ankle_roll_joint",
      "right_hip_pitch_joint",
      "right_hip_roll_joint",
      "right_hip_yaw_joint",
      "right_knee_joint",
      "right_ankle_pitch_joint",
      "right_ankle_roll_joint",
      "waist_yaw_joint",
      "waist_pitch_joint",
      "waist_roll_joint",
      "left_shoulder_pitch_joint",
      "left_shoulder_roll_joint",
      "left_shoulder_yaw_joint",
      "left_elbow_joint",
      "left_wrist_roll_joint",
      "left_wrist_yaw_joint",
      "left_wrist_pitch_joint",
      "right_shoulder_pitch_joint",
      "right_shoulder_roll_joint",
      "right_shoulder_yaw_joint",
      "right_elbow_joint",
      "right_wrist_roll_joint",
      "right_wrist_yaw_joint",
      "right_wrist_pitch_joint",
      "neck_pitch_joint",
      "neck_yaw_joint"};
  lowlevel::RobotInfoT info;
  info.protocol_version = 1;
  info.body = std::make_unique<lowlevel::BodyInfoT>();
  info.body->slot_count = kSlots;
  info.body->roster_version = 1;
  for (size_t j = 0; j < kSlots; ++j) {
    const bool present = j != 14;
    info.body->joint_names.push_back(names[j]);
    info.body->joints.emplace_back(j, lowlevel::BodySlotRole_ACTIVE, false,
                                   present, present ? 31 : 0, present ? 31 : 0,
                                   kJointLimits.q_min, kJointLimits.q_max,
                                   kJointLimits.dq_min, kJointLimits.dq_max,
                                   kJointLimits.tau_min, kJointLimits.tau_max,
                                   kJointLimits.kp_min, kJointLimits.kp_max,
                                   kJointLimits.kd_min, kJointLimits.kd_max);
    auto identity = std::make_unique<lowlevel::BodyMotorIdentityT>();
    identity->bus = "SYNTHETIC_NO_CAN";
    identity->node_id = j + 1;
    identity->present = present;
    info.body->motor_identities.push_back(std::move(identity));
  }
  for (int side = 1; side <= 2; ++side) {
    auto ee = std::make_unique<lowlevel::EndEffectorCapabilityT>();
    ee->side = static_cast<lowlevel::EndEffectorSide>(side);
    const char* requested =
        std::getenv(side == 1 ? "PRODUCTION_MOCK_EE_LEFT_KIND"
                              : "PRODUCTION_MOCK_EE_RIGHT_KIND");
    const int kind = requested ? std::stoi(requested) : 0;
    if (kind < 0 || kind > 2)
      throw std::runtime_error("invalid synthetic EE kind");
    ee->kind = static_cast<lowlevel::EndEffectorKind>(kind);
    ee->ready = true;
    ee->status = aorta::services::common::ServiceStatus_SUCCESS;
    info.end_effectors.push_back(std::move(ee));
  }
  aorta::services::humanoid_locomotion::GetRobotInfoResponseT reply;
  reply.status = aorta::services::common::ServiceStatus_SUCCESS;
  reply.info = std::make_unique<lowlevel::RobotInfoT>(std::move(info));
  flatbuffers::FlatBufferBuilder b;
  b.Finish(aorta::services::humanoid_locomotion::GetRobotInfoResponse::Pack(
      b, &reply));
  return {b.GetBufferPointer(), b.GetBufferPointer() + b.GetSize()};
}
}  // namespace production_mock
#else
namespace production_mock {
uint64_t NextApplyTick(uint64_t accepted_wall_ns, uint64_t period_ns) {
  if (!period_ns) throw std::invalid_argument("nonzero apply period required");
  const auto remainder = accepted_wall_ns % period_ns;
  const auto delta = remainder ? period_ns - remainder : 0;
  if (accepted_wall_ns > std::numeric_limits<uint64_t>::max() - delta)
    throw std::overflow_error("synthetic apply tick overflow");
  return accepted_wall_ns + delta;
}
Protocol::Protocol(uint64_t seed, uint64_t stream_timeout,
                   uint64_t prepare_timeout, int reject_operation)
    : generation_(seed),
      command_timeout_ns_(stream_timeout),
      prepare_timeout_ns_(prepare_timeout),
      reject_operation_(reject_operation) {
  if (!seed || !stream_timeout || !prepare_timeout)
    throw std::invalid_argument("nonzero generation/timeouts required");
}
Ack Protocol::Reply(const Request& r, int error,
                    const std::string& reason) const {
  return {r.request_id, error ? "" : status_.lease_id,
          reason,       r.operation,
          error,        error == 0};
}
void Protocol::Clear() {
  const auto seq = status_.status_seq + 1;
  const auto rejects = status_.reject_count;
  status_ = Status{};
  status_.status_seq = seq;
  status_.reject_count = rejects;
  has_pending_ = false;
  prepared_ns_ = 0;
  InvalidateApplied();
}
void Protocol::InvalidateApplied() {
  // Hold the last synthetic positions, while invalidating evidence of a
  // current command. Cancelling does not teleport the simulated joints.
  output_.session = output_.sequence = output_.first_output_us = 0;
}
void Protocol::Watchdog(uint64_t now) {
  if (status_.phase == 2 && now - prepared_ns_ >= prepare_timeout_ns_) Clear();
  if (status_.phase == 4 &&
      now - status_.last_valid_ns >= command_timeout_ns_) {
    status_.phase = 5;
    ++status_.status_seq;
    has_pending_ = false;
    InvalidateApplied();
  }
}
Ack Protocol::RequestControl(const Request& r, uint64_t now) {
  Watchdog(now);
  if (r.version != 2) return Reply(r, 1001, "unsupported request version");
  if (r.request_id.empty() || r.client_id.empty() ||
      r.request_id.size() > 256 || r.client_id.size() > 256 ||
      r.operation < 1 || r.operation > 3)
    return Reply(r, 1002, "invalid request identity/operation");
  if (r.action != "EXTERNAL")
    return Reply(r, 1003, "only EXTERNAL is simulated");
  if (r.operation == reject_operation_)
    return Reply(r, 1003, "scripted rejection");
  if (r.operation == 1) {
    if (!status_.client_id.empty()) {
      if (r.client_id != status_.client_id) return Reply(r, 1008, "mock busy");
      if (r.request_id != status_.request_id)
        return Reply(r, 1004, "request conflict");
      return Reply(r, 0, "idempotent PREPARE");
    }
    if (generation_ == std::numeric_limits<uint64_t>::max())
      return Reply(r, 1009, "generation exhausted");
    status_.request_id = r.request_id;
    status_.client_id = r.client_id;
    status_.lease_id = "mock-lease-" + std::to_string(++generation_);
    status_.phase = 2;
    ++status_.status_seq;
    prepared_ns_ = now;
    return Reply(r, 0, "armed; no automatic running lease expiry");
  }
  if (r.operation == 3 && status_.client_id.empty())
    return Reply(r, 0, "already idle");
  if (status_.client_id != r.client_id)
    return Reply(r, 1005, "identity mismatch");
  if (status_.request_id != r.request_id)
    return Reply(r, 1004, "request conflict");
  if (r.lease_id != status_.lease_id) return Reply(r, 1006, "lease mismatch");
  if (r.operation == 3) {
    Ack ack = Reply(r, 0, "cancelled to PASSIVE");
    Clear();
    return ack;
  }
  if (status_.phase == 4 || status_.phase == 5)
    return Reply(r, 0, "idempotent COMMIT");
  if (status_.phase != 2) return Reply(r, 1007, "not armed");
  if (generation_ == std::numeric_limits<uint64_t>::max())
    return Reply(r, 1009, "generation exhausted");
  status_.generation = ++generation_;
  status_.phase = 4;
  status_.last_valid_ns = now;
  ++status_.status_seq;
  return Reply(r, 0, "EXTERNAL RUNNING");
}
bool Protocol::Accept(const Command& cmd, uint64_t now) {
  Watchdog(now);
  bool valid = (status_.phase == 4 || status_.phase == 5) &&
               cmd.session == status_.generation &&
               cmd.sequence > status_.accepted_sequence && cmd.sequence &&
               cmd.input_state_real_us;
  for (size_t j = 0; j < kSlots && valid; ++j) {
    if (j == 14) continue;
    valid = std::isfinite(cmd.q[j]) && std::isfinite(cmd.dq[j]) &&
            std::isfinite(cmd.tau[j]) && std::isfinite(cmd.kp[j]) &&
            std::isfinite(cmd.kd[j]) &&
            cmd.q[j] >= kJointLimits.q_min && cmd.q[j] <= kJointLimits.q_max &&
            cmd.dq[j] >= kJointLimits.dq_min && cmd.dq[j] <= kJointLimits.dq_max &&
            cmd.tau[j] >= kJointLimits.tau_min && cmd.tau[j] <= kJointLimits.tau_max &&
            cmd.kp[j] >= kJointLimits.kp_min && cmd.kp[j] <= kJointLimits.kp_max &&
            cmd.kd[j] >= kJointLimits.kd_min && cmd.kd[j] <= kJointLimits.kd_max;
  }
  if (!valid) {
    ++status_.reject_count;
    status_.reject_code =
        1;  // Synthetic body admission diagnostic, not ControlErrorCode.
    ++status_.status_seq;
    return false;
  }
  if (has_pending_) ++overwritten_;
  pending_ = cmd;
  has_pending_ = true;
  status_.accepted_sequence = cmd.sequence;
  status_.last_valid_ns = now;
  status_.phase = 4;
  status_.reject_code = 0;
  ++status_.status_seq;
  return true;
}
bool Protocol::Tick(uint64_t now, uint64_t wall_us) {
  Watchdog(now);
  if (!has_pending_ || status_.phase != 4) return false;
  output_.session = pending_.session;
  output_.sequence = pending_.sequence;
  output_.q = pending_.q;
  output_.first_output_us = wall_us;
  has_pending_ = false;
  return true;
}
}  // namespace production_mock
#endif
