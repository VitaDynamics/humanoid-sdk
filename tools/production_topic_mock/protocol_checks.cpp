#include <cassert>
#include <limits>
#include <memory>
#include <type_traits>
#include <utility>

#include "aorta_contract.h"
#include "control_request_generated.h"
#include "protocol.h"
using namespace production_mock;
static_assert(std::is_same_v<
              decltype(Request::version),
              decltype(std::declval<const locomotion_sdk::ControlRequest &>()
                           .protocol_version())>);
int main() {
  aorta_contract::ValidateGroupLength(
      std::string(aorta_contract::MAX_GROUP_LEN, 'a'));
  bool overlong_rejected = false;
  try {
    aorta_contract::ValidateGroupLength(
        std::string(aorta_contract::MAX_GROUP_LEN + 1, 'a'));
  } catch (const std::invalid_argument &) {
    overlong_rejected = true;
  }
  assert(overlong_rejected);
  assert(NextApplyTick(2000001, 2000000) == 4000000);
  assert(NextApplyTick(4000000, 2000000) == 4000000);
  Protocol p(10, 100, 50);
  Request r{4, 1, "req", "EXTERNAL", 7, 0, 10};
  auto nonempty_prepare = r;
  nonempty_prepare.control_id = 1;
  assert(p.RequestControl(nonempty_prepare, 998).error == 1006);
  assert(p.status().phase == 0);
  for (const uint32_t version : {uint32_t{4}, uint32_t{0x80000003},
                                 std::numeric_limits<uint32_t>::max()}) {
    locomotion_sdk::ControlRequestT wire;
    wire.protocol_version = version;
    wire.operation = locomotion_sdk::ControlOperation_PREPARE;
    wire.request_id = "version-check";
    wire.client_id = 7;
    wire.runtime_id = 10;
    wire.action_name = "EXTERNAL";
    flatbuffers::FlatBufferBuilder builder;
    builder.Finish(locomotion_sdk::ControlRequest::Pack(builder, &wire));
    const std::vector<uint8_t> bytes(
        builder.GetBufferPointer(),
        builder.GetBufferPointer() + builder.GetSize());
    const auto decoded = DecodeRequest(bytes);
    assert(decoded.version == version);
    if (version != 4) {
      assert(p.RequestControl(decoded, 999).error == 1001);
      assert(p.status().phase == 0);
    }
  }
  auto prep = p.RequestControl(r, 1000);
  assert(prep.accepted && prep.control_id != 0 && prep.runtime == 10);
  auto foreign_runtime = r;
  foreign_runtime.runtime = 11;
  assert(p.RequestControl(foreign_runtime, 1000).error == 1005);
  assert(p.status().phase == 2 && p.status().control_id == prep.control_id);
  assert(p.status().phase == 2);
  auto duplicate = p.RequestControl(r, 1001);
  assert(duplicate.control_id == prep.control_id);
  auto other = r;
  other.client_id = 8;
  assert(p.RequestControl(other, 1002).error == 1008);
  other = r;
  other.request_id = "other";
  assert(p.RequestControl(other, 1002).error == 1004);
  r.operation = 2;
  r.control_id = prep.control_id + 1;
  assert(p.RequestControl(r, 1003).error == 1006);
  r.control_id = prep.control_id;
  assert(p.RequestControl(r, 1004).accepted);
  auto generation = p.status().generation;
  assert(generation && p.status().phase == 4);
  assert(p.RequestControl(r, 1005).accepted &&
         p.status().generation == generation);
  Command cmd;
  cmd.session = generation;
  cmd.runtime = 10; cmd.control_id = prep.control_id;
  cmd.sequence = 1;
  cmd.input_state_real_us = 1;
  cmd.kp.fill(1);
  cmd.kd.fill(1);
  auto wrong_body = cmd;
  wrong_body.runtime = 11;
  assert(!p.Accept(wrong_body, 1009));
  wrong_body = cmd;
  wrong_body.control_id = prep.control_id + 1;
  assert(!p.Accept(wrong_body, 1009));
  assert(p.status().accepted_sequence == 0);
  assert(p.Accept(cmd, 1010));
  assert(!p.Accept(cmd, 1011));
  assert(p.Tick(1012, 2000));
  assert(p.output().sequence == 1 && p.output().first_output_us == 2000);
  assert(!p.Tick(1013, 2001));
  cmd.sequence = 2;
  cmd.q[0] = std::numeric_limits<float>::quiet_NaN();
  assert(!p.Accept(cmd, 1020));
  cmd.q[0] = 0;
  cmd.session = 0;
  assert(!p.Accept(cmd, 1021));
  cmd.session = generation;
  assert(p.Accept(cmd, 1022));
  cmd.sequence = 3;
  assert(p.Accept(cmd, 1023));
  assert(p.overwritten() == 1);
  assert(p.Tick(1024, 2002));
  assert(p.output().sequence == 3);
  p.Watchdog(1123);
  assert(p.status().phase == 5);
  assert(p.output().session == 0 && p.output().sequence == 0 &&
         p.output().first_output_us == 0);
  cmd.sequence = 4;
  assert(p.Accept(cmd, 1124) && p.status().phase == 4);
  r.operation = 3;
  assert(p.RequestControl(r, 1125).accepted);
  assert(p.status().phase == 0);
  assert(p.status().runtime == 10 && p.output().runtime == 10);
  assert(p.output().session == 0 && p.output().sequence == 0 &&
         p.output().first_output_us == 0);
  assert(!p.Tick(1126, 2003));
  assert(!p.Accept(cmd, 1127));
  assert(p.RequestControl(r, 1128).accepted);
  r.operation = 1;
  r.control_id = 0;
  r.version = 1;
  assert(p.RequestControl(r, 1130).error == 1001);
  r.version = 3;
  r.action = "OTHER";
  assert(p.RequestControl(r, 1131).error == 1003);
  r.action = "EXTERNAL";
  assert(p.RequestControl(r, 1140).accepted);
  p.Watchdog(1190);
  assert(p.status().phase == 0);
  Protocol rejected(20, 100, 50, 1);
  auto rejected_request = r; rejected_request.runtime = 20;
  assert(!rejected.RequestControl(rejected_request, 1200).accepted);
  Protocol exhausted(std::numeric_limits<uint64_t>::max(), 100, 50);
  auto exhausted_request = r; exhausted_request.runtime = std::numeric_limits<uint64_t>::max();
  assert(exhausted.RequestControl(exhausted_request, 1200).error == 1009);
  Protocol limits(50, 1000, 1000);
  Request lr{3, 1, "limits", "EXTERNAL", 7, 0, 50};
  auto la = limits.RequestControl(lr, 1000);
  assert(la.accepted);
  lr.operation = 2;
  lr.control_id = la.control_id;
  assert(limits.RequestControl(lr, 1001).accepted);
  Command lc;
  lc.runtime = 50; lc.control_id = la.control_id;
  lc.session = limits.status().generation;
  lc.sequence = 1;
  lc.input_state_real_us = 1;
  lc.dq[0] = kJointLimits.dq_max;
  lc.tau[0] = kJointLimits.tau_min;
  assert(limits.Accept(lc, 1002));
  ++lc.sequence;
  lc.dq[0] = kJointLimits.dq_min;
  lc.tau[0] = kJointLimits.tau_max;
  assert(limits.Accept(lc, 1003));
  ++lc.sequence;
  lc.dq[0] = kJointLimits.dq_max + 1;
  assert(!limits.Accept(lc, 1004));
  lc.dq[0] = kJointLimits.dq_min - 1;
  assert(!limits.Accept(lc, 1005));
  lc.dq[0] = 0;
  lc.tau[0] = kJointLimits.tau_max + 1;
  assert(!limits.Accept(lc, 1006));
  lc.tau[0] = kJointLimits.tau_min - 1;
  assert(!limits.Accept(lc, 1007));

  // EE and typed touch borrow the original Body owner, not a second session.
  Protocol shared(100, 100, 50);
  EEInfo hand;
  hand.side = 1; hand.device_model = 1; hand.layout_id = 7;
  hand.touch_available = true;
  EEAxis axis;
  axis.modes = (1u << 1) | (1u << 2);
  axis.command_fields = 1 | 4 | 8 | 64;
  axis.state_fields = 1;
  axis.valid_limits = 1 | 4 | 8;
  axis.pos_min = 0; axis.pos_max = 1; axis.vel_limit_max = 1;
  axis.tau_min = -1; axis.tau_max = 1;
  hand.axes = {axis}; hand.observation_sources = {{2,1}};
  shared.ConfigureEE(hand, {.03f});
  auto gripper = hand;
  gripper.side = 2; gripper.device_model = 3;
  gripper.touch_available = false; gripper.observation_sources = {{2,2}};
  shared.ConfigureEE(gripper, {.02f});
  Request owner{3, 1, "shared", "EXTERNAL", 7, 0, 100};
  auto armed = shared.RequestControl(owner, 1000);
  assert(armed.accepted);
  owner.operation = 2; owner.control_id = armed.control_id;
  assert(shared.RequestControl(owner, 1001).accepted);
  EECommand ec;
  ec.version = 2; ec.runtime = shared.runtime_id();
  ec.layout_id = 7; ec.side = 1; ec.control_id = owner.control_id;
  ec.key = {shared.status().generation, 1}; ec.modes = {2}; ec.fields = {1 | 4 | 8};
  ec.columns = {std::vector<float>{.25f}, std::vector<float>{0.f}, std::vector<float>{0.f},
                std::vector<float>{-.5f}, std::vector<float>{0.f}};
  ec.observations = {{shared.runtime_id(),30,100,2,1,7}};
  auto bad = ec;
  bad.control_id = owner.control_id + 1;
  assert(!shared.AcceptEE(bad,1002,100));
  bad = ec; bad.runtime += 1;
  assert(!shared.AcceptEE(bad,1003,100));
  bad = ec; bad.observations[0].side = 2;
  assert(!shared.AcceptEE(bad,1004,100));
  bad = ec; bad.columns[0].clear();
  assert(!shared.AcceptEE(bad,1005,100));
  bad = ec; bad.columns[3][0] = -2;
  assert(!shared.AcceptEE(bad,1006,100));
  bad = ec; bad.columns[3][0] = std::numeric_limits<float>::quiet_NaN();
  assert(!shared.AcceptEE(bad,1007,100));
  assert(!shared.SnapshotEE(1,1007).accepted_valid);
  assert(shared.AcceptEE(ec,1010,200));
  auto before = shared.SnapshotEE(1,1010);
  assert(before.accepted_valid && before.accepted.sequence == 1);
  assert(!before.output_key_valid[0] && before.position[0] == .03f);
  assert(shared.status().accepted_sequence == 0 && shared.status().last_valid_ns == 1001);
  // Independent topics may deliver a globally later right key first.
  auto right = ec; right.side = 2; right.observations[0].side = 2; right.key.sequence = 9;
  assert(shared.AcceptEE(right,1011,201));
  ec.key.sequence = 2; ec.columns[0][0] = .5f;
  assert(shared.AcceptEE(ec,1012,202));
  assert(shared.SnapshotEE(1,1012).overwritten == 1);
  assert(shared.TickEE(1013,203) == 2);
  auto applied = shared.SnapshotEE(1,1013);
  assert(applied.position[0] == .5f && applied.output_key_valid[0]);
  assert(applied.output_keys[0].session == ec.key.session && applied.output_keys[0].sequence == 2);
  assert(!shared.AcceptEE(ec,1014,204));
  // set_touch changes the same published snapshot without consuming a key,
  // changing pending motion, or refreshing the original Body heartbeat.
  TouchRequest tr{2,7,shared.runtime_id(),1,owner.control_id,true};
  auto rejected_touch = tr; rejected_touch.runtime += 1;
  auto rejected_reply = shared.SetTouch(rejected_touch,1015);
  assert(!rejected_reply.success && rejected_reply.runtime == rejected_touch.runtime);
  ec.key.sequence = 3;
  assert(shared.AcceptEE(ec,1016,205));
  const auto last_body = shared.status().last_valid_ns;
  auto switched = shared.SetTouch(tr,1017);
  assert(switched.success && switched.enabled && switched.enabled_valid);
  assert(shared.SnapshotEE(1,1017).touch_enabled);
  assert(shared.SnapshotEE(1,1017).accepted.sequence == 3);
  assert(shared.TickEE(1018,206) == 1);
  assert(shared.status().last_valid_ns == last_body);
  tr.side = 2;
  assert(!shared.SetTouch(tr,1019).success);
  shared.Watchdog(1101);
  assert(shared.status().phase == 5 && !shared.output().applied_key_valid);
  // Body TIMEOUT_HOLD retains the same owner; EE-only remains possible.
  ec.key.sequence = 4;
  assert(shared.AcceptEE(ec,1102,207));
  assert(shared.TickEE(1103,208) == 1);
  assert(shared.status().phase == 5 && shared.status().last_valid_ns == last_body);
  assert(shared.SnapshotEE(1,1202).timeouts == 1);
  assert(!shared.SnapshotEE(1,1203).output_key_valid[0]);
  ec.key.sequence = 5;
  assert(shared.AcceptEE(ec,1204,209));
  owner.operation = 3;
  assert(shared.RequestControl(owner,1205).accepted);
  assert(shared.TickEE(1206,210) == 0);
  const auto cancelled = shared.SnapshotEE(1,1206);
  assert(!cancelled.output_allowed && !cancelled.accepted_valid && !cancelled.output_key_valid[0]);
  assert(cancelled.position[0] == .5f); // No teleported zero on CANCEL.
  assert(!shared.AcceptEE(ec,1207,211));
  tr.side = 1;
  assert(!shared.SetTouch(tr,1208).success);
}
