#include <array>
#include <memory>
#include <stdexcept>

#include "body_state_generated.h"
#include "protocol.h"

namespace production_mock {
std::vector<uint8_t> EncodeState(uint64_t id, const Output& output,
                                 uint64_t wall_us) {
  lowlevel::BodyStateT state;
  state.protocol_version = 3;
  // This Mock's own startup/tick snapshot, never inferred from a nonzero key.
  state.descriptor_ref = std::make_unique<lowlevel::ee::DescriptorRef>(
      output.runtime);
  state.ts_control_us = output.control_us;
  state.applied_key_valid = output.applied_key_valid;
  state.association_source_type = output.association_source_type;
  state.roster_version = 1;
  state.state_id = id;
  state.ts_state_real_us = state.ts_state_pub_us = state.ts_consumed_us =
      wall_us;
  state.attitude_valid = true;
  state.applied_session_id = output.session;
  state.applied_sequence = output.sequence;
  state.ts_first_output_us = output.first_output_us;
  state.aorta_header = std::make_unique<aorta::sys::AortaHeaderT>();
  state.aorta_header->sequence = id;
  state.aorta_header->publish_stamp_ns = wall_us * 1000;
  state.aorta_header->publisher_node = "production_topic_mock";
  for (size_t j = 0; j < kSlots; ++j)
    state.joints.emplace_back(wall_us, wall_us, wall_us, output.q[j], 0.f, 0.f,
                              lowlevel::JointError_NONE);
  for (int j = 0; j < 2; ++j) {
    auto sample = std::make_unique<lowlevel::BodyImuSampleT>();
    sample->location = static_cast<lowlevel::BodyImuLocation>(j);
    const auto& q = output.imu_q[j];
    const std::array<float, 3> zero{0, 0, 0}, acceleration{0, 0, 9.81f};
    sample->state =
        std::make_unique<lowlevel::BodyImuState>(q, zero, acceleration, zero);
    sample->status = lowlevel::BodySampleStatus_OK;
    sample->ts_sampled_us = sample->ts_acquired_hw_us = sample->ts_ingested_us =
        sample->ts_assembled_us = wall_us;
    state.imu_states.push_back(std::move(sample));
  }
  flatbuffers::FlatBufferBuilder b;
  b.Finish(lowlevel::BodyState::Pack(b, &state));
  return {b.GetBufferPointer(), b.GetBufferPointer() + b.GetSize()};
}
void StampState(std::vector<uint8_t>& bytes, uint64_t wall_ns) {
  auto* state = lowlevel::GetMutableBodyState(bytes.data());
  if (!state->mutate_ts_state_pub_us(wall_ns / 1000) ||
      !state->mutable_aorta_header()->mutate_publish_stamp_ns(wall_ns))
    throw std::runtime_error("BodyState publication stamp is absent");
}
}  // namespace production_mock
