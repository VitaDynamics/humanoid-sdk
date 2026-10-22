#include <cassert>
#include <limits>
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
  Request r{2, 1, "req", "client", "", "EXTERNAL"};
  for (const uint32_t version : {uint32_t{2}, uint32_t{0x80000002},
                                 std::numeric_limits<uint32_t>::max()}) {
    locomotion_sdk::ControlRequestT wire;
    wire.protocol_version = version;
    wire.operation = locomotion_sdk::ControlOperation_PREPARE;
    wire.request_id = "version-check";
    wire.client_id = "client";
    wire.action_name = "EXTERNAL";
    flatbuffers::FlatBufferBuilder builder;
    builder.Finish(locomotion_sdk::ControlRequest::Pack(builder, &wire));
    const std::vector<uint8_t> bytes(
        builder.GetBufferPointer(),
        builder.GetBufferPointer() + builder.GetSize());
    const auto decoded = DecodeRequest(bytes);
    assert(decoded.version == version);
    if (version != 2) {
      assert(p.RequestControl(decoded, 999).error == 1001);
      assert(p.status().phase == 0);
    }
  }
  auto prep = p.RequestControl(r, 1000);
  assert(prep.accepted && !prep.lease_id.empty());
  assert(p.status().phase == 2);
  auto duplicate = p.RequestControl(r, 1001);
  assert(duplicate.lease_id == prep.lease_id);
  auto other = r;
  other.client_id = "other";
  assert(p.RequestControl(other, 1002).error == 1008);
  other = r;
  other.request_id = "other";
  assert(p.RequestControl(other, 1002).error == 1004);
  r.operation = 2;
  r.lease_id = "wrong";
  assert(p.RequestControl(r, 1003).error == 1006);
  r.lease_id = prep.lease_id;
  assert(p.RequestControl(r, 1004).accepted);
  auto generation = p.status().generation;
  assert(generation && p.status().phase == 4);
  assert(p.RequestControl(r, 1005).accepted &&
         p.status().generation == generation);
  Command cmd;
  cmd.session = generation;
  cmd.sequence = 1;
  cmd.input_state_real_us = 1;
  cmd.kp.fill(1);
  cmd.kd.fill(1);
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
  assert(p.output().session == 0 && p.output().sequence == 0 &&
         p.output().first_output_us == 0);
  assert(!p.Tick(1126, 2003));
  assert(!p.Accept(cmd, 1127));
  assert(p.RequestControl(r, 1128).accepted);
  r.operation = 1;
  r.lease_id = "";
  r.version = 1;
  assert(p.RequestControl(r, 1130).error == 1001);
  r.version = 2;
  r.action = "OTHER";
  assert(p.RequestControl(r, 1131).error == 1003);
  r.action = "EXTERNAL";
  assert(p.RequestControl(r, 1140).accepted);
  p.Watchdog(1190);
  assert(p.status().phase == 0);
  Protocol rejected(20, 100, 50, 1);
  assert(!rejected.RequestControl(r, 1200).accepted);
  Protocol exhausted(std::numeric_limits<uint64_t>::max(), 100, 50);
  assert(exhausted.RequestControl(r, 1200).error == 1009);
  Protocol limits(50, 1000, 1000);
  Request lr{2, 1, "limits", "client", "", "EXTERNAL"};
  auto la = limits.RequestControl(lr, 1000);
  assert(la.accepted);
  lr.operation = 2;
  lr.lease_id = la.lease_id;
  assert(limits.RequestControl(lr, 1001).accepted);
  Command lc;
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
}
