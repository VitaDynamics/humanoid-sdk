#include "protocol.h"

#include <algorithm>
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
    : runtime_id_(seed), generation_(seed),
      command_timeout_ns_(stream_timeout),
      prepare_timeout_ns_(prepare_timeout),
      reject_operation_(reject_operation) {
  if (!seed || !stream_timeout || !prepare_timeout)
    throw std::invalid_argument("nonzero generation/timeouts required");
  output_.runtime = runtime_id_;
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
  InvalidateEE();
}
void Protocol::InvalidateApplied() {
  // Hold the last synthetic positions, while invalidating evidence of a
  // current command. Cancelling does not teleport the simulated joints.
  output_.session = output_.sequence = output_.first_output_us = 0;
  output_.applied_key_valid = false;
  output_.association_source_type = 0;
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
  output_.control_us = wall_us;
  output_.applied_key_valid = true;
  output_.association_source_type = 1;  // EXTERNAL, from this accepted synthetic frame.
  has_pending_ = false;
  return true;
}

void Protocol::ConfigureEE(const EEInfo& info, const std::vector<float>& initial) {
  if (status_.phase != 0 || !status_.client_id.empty() || info.side < 1 || info.side > 2 ||
      info.axes.empty() || info.axes.size() > 64 || initial.size() != info.axes.size() ||
      info.device_model < 1 || info.device_model > 3 || !info.layout_id)
    throw std::invalid_argument("invalid synthetic EE fixture or active owner");
  auto& endpoint = endpoints_[info.side - 1];
  if (endpoint.configured) throw std::invalid_argument("EE fixture is fixed at startup");
  for (size_t i = 0; i < initial.size(); ++i) {
    const auto& a = info.axes[i];
    if (!(a.valid_limits & 1) || !(a.state_fields & 1) || !std::isfinite(initial[i]) ||
        !std::isfinite(a.pos_min) || !std::isfinite(a.pos_max) ||
        initial[i] < a.pos_min || initial[i] > a.pos_max)
      throw std::invalid_argument("invalid synthetic measured position");
  }
  std::vector<std::pair<uint8_t, uint8_t>> seen;
  for (const auto& source : info.observation_sources) {
    const auto [stream, side] = source;
    if (!((stream == 1 && side == 0) || ((stream == 2 || stream == 3) && (side == 1 || side == 2))) ||
        std::find(seen.begin(), seen.end(), source) != seen.end())
      throw std::invalid_argument("invalid or duplicate synthetic observation source");
    seen.push_back(source);
  }
  endpoint.configured = true;
  endpoint.snapshot.info = info;
  endpoint.snapshot.position = initial;
  endpoint.snapshot.output_keys.resize(initial.size());
  endpoint.snapshot.output_key_valid.assign(initial.size(), false);
  endpoint.snapshot.runtime = runtime_id_;
}

bool Protocol::OwnedEE(const std::string& client, const std::string& lease) const {
  return (status_.phase == 4 || status_.phase == 5) && !client.empty() && !lease.empty() &&
         client == status_.client_id && lease == status_.lease_id;
}

void Protocol::InvalidateEE() {
  for (auto& endpoint : endpoints_) {
    endpoint.has_pending = false;
    endpoint.last_valid_ns = 0;
    endpoint.timed_out = false;
    auto& s = endpoint.snapshot;
    s.received_valid = s.accepted_valid = s.applied_valid = false;
    std::fill(s.output_key_valid.begin(), s.output_key_valid.end(), false);
    s.output_allowed = false;
    s.inhibit_reason = 1;
    s.received = s.accepted = s.applied = {};
    s.received_us = s.accepted_us = s.output_us = 0;
  }
}

bool Protocol::AcceptEE(const EECommand& cmd, uint64_t now, uint64_t wall_us) {
  Watchdog(now);  // EE traffic never refreshes Body's original watchdog.
  if (cmd.side < 1 || cmd.side > 2 || !endpoints_[cmd.side - 1].configured) return false;
  auto& e = endpoints_[cmd.side - 1];
  auto& s = e.snapshot;
  s.received = cmd.key;
  s.received_valid = true;
  s.received_us = wall_us;
  auto reject = [&](uint16_t reason) { ++s.reject_count; s.reject_code = reason; return false; };
  if (cmd.version != 1) return reject(8);
  if (cmd.runtime != runtime_id_ || cmd.epoch != s.epoch) return reject(9);
  if (cmd.layout_id != s.info.layout_id) return reject(10);
  if (!OwnedEE(cmd.client_id, cmd.lease_id) || cmd.key.session != status_.generation || !cmd.key.sequence)
    return reject(11);
  // Sequence spaces are per endpoint on receive, since independent topics may
  // arrive out of order. There is no endpoint session table or second owner.
  if (s.accepted_valid && cmd.key.sequence <= s.accepted.sequence) return false;
  const auto n = s.info.axes.size();
  if (cmd.modes.size() != n || cmd.fields.size() != n ||
      cmd.observations.size() != s.info.observation_sources.size()) return reject(2);
  for (size_t i = 0; i < cmd.observations.size(); ++i) {
    const auto& o = cmd.observations[i];
    if (o.runtime != runtime_id_ || o.epoch != s.epoch || (o.validity & ~15) ||
        std::make_pair(o.stream, o.side) != s.info.observation_sources[i]) return reject(9);
    // Actual IDs/time/validity are preserved, not regenerated or made into a
    // time/permission gate. Reusing one actual sample is legal.
  }
  constexpr std::array<uint32_t, 5> bits{1, 2, 4, 8, 16};
  bool requested = false;
  for (size_t i = 0; i < n; ++i) {
    const auto& a = s.info.axes[i];
    if ((cmd.modes[i] != 1 && cmd.modes[i] != 2) || !(a.modes & (1u << cmd.modes[i]))) return reject(5);
    if ((cmd.fields[i] & ~uint32_t{95}) || (cmd.fields[i] & ~a.command_fields)) return reject(12);
    requested |= cmd.fields[i] != 0;
  }
  if (!requested) return reject(12);
  for (size_t col = 0; col < bits.size(); ++col) {
    const auto bit = bits[col];
    const bool supported = std::any_of(s.info.axes.begin(), s.info.axes.end(),
        [bit](const auto& a) { return a.command_fields & bit; });
    if (cmd.columns[col].size() != (supported ? n : 0)) return reject(2);
    for (size_t i = 0; i < cmd.columns[col].size(); ++i) {
      const auto v = cmd.columns[col][i];
      if (!std::isfinite(v)) return reject(3);
      if (!(cmd.fields[i] & bit)) continue;
      const auto& a = s.info.axes[i];
      if (!(a.valid_limits & bit)) return reject(4);
      const std::array<float, 5> minimum{a.pos_min,a.vel_min,0.f,a.tau_min,a.current_min};
      const std::array<float, 5> maximum{a.pos_max,a.vel_max,a.vel_limit_max,a.tau_max,a.current_max};
      if (!std::isfinite(minimum[col]) || !std::isfinite(maximum[col]) ||
          v < minimum[col] || v > maximum[col]) return reject(4);
    }
  }
  if (e.has_pending) ++s.overwritten;
  e.pending = cmd;
  e.has_pending = true;
  e.last_valid_ns = now;
  e.timed_out = false;
  s.accepted = cmd.key;
  s.accepted_valid = true;
  s.accepted_us = wall_us;
  s.reject_code = 0;
  return true;
}

EESnapshot Protocol::SnapshotEE(uint8_t side, uint64_t now) {
  Watchdog(now);
  if (side < 1 || side > 2 || !endpoints_[side - 1].configured)
    throw std::invalid_argument("unconfigured synthetic endpoint");
  auto& e = endpoints_[side - 1];
  if (e.last_valid_ns && now - e.last_valid_ns >= command_timeout_ns_ && !e.timed_out) {
    e.timed_out = true;
    e.has_pending = false;
    e.snapshot.applied_valid = false;
    std::fill(e.snapshot.output_key_valid.begin(), e.snapshot.output_key_valid.end(), false);
    ++e.snapshot.timeouts;
  }
  auto result = e.snapshot;
  // Synthetic configured fixtures are ready; this is not hardware discovery.
  result.output_allowed = status_.phase == 4 || status_.phase == 5;
  result.inhibit_reason = result.output_allowed ? 0 : 1;
  return result;
}

size_t Protocol::TickEE(uint64_t now, uint64_t wall_us) {
  Watchdog(now);
  size_t applied = 0;
  for (size_t side = 1; side <= 2; ++side) {
    auto& e = endpoints_[side - 1];
    if (!e.configured) continue;
    const auto snapshot = SnapshotEE(static_cast<uint8_t>(side), now);
    if (!snapshot.output_allowed || !e.has_pending) continue;
    for (size_t i = 0; i < e.snapshot.position.size(); ++i) {
      if (e.pending.fields[i] & 1) e.snapshot.position[i] = e.pending.columns[0][i];
      if (e.pending.fields[i]) {
        e.snapshot.output_keys[i] = e.pending.key;
        e.snapshot.output_key_valid[i] = true;
      }
    }
    e.snapshot.applied = e.pending.key;
    e.snapshot.applied_valid = true;
    e.snapshot.output_us = wall_us;
    e.has_pending = false;
    ++applied;
  }
  return applied;
}

TouchReply Protocol::SetTouch(const TouchRequest& request, uint64_t now) {
  Watchdog(now);
  TouchReply reply;
  reply.runtime = request.runtime;  // Reply binding echoes even a rejected request.
  reply.epoch = request.epoch;
  reply.side = request.side;
  if (request.side < 1 || request.side > 2 || !endpoints_[request.side - 1].configured) {
    reply.reason = 3;
    return reply;
  }
  auto& s = endpoints_[request.side - 1].snapshot;
  if (request.version != 1 || request.runtime != runtime_id_ || request.epoch != s.epoch ||
      request.layout_id != s.info.layout_id) reply.reason = 3;
  else if (!OwnedEE(request.client_id, request.lease_id)) reply.reason = 2;
  else if (!s.info.touch_available) reply.reason = 1;
  else {
    s.touch_enabled = request.enabled;
    reply.success = true;
    reply.enabled = s.touch_enabled;
    reply.enabled_valid = true;
  }
  // No motion sequence reservation, pending output or heartbeat update.
  return reply;
}
}  // namespace production_mock
#endif
