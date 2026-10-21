#include <array>
#include <chrono>
#include <limits>
#include <vector>
#include "body_state_generated.h"

// Separate translation unit: flattened BFBS headers duplicate common types.
std::vector<uint8_t> MakeState(uint64_t sequence) {
  const auto wall_us = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  lowlevel::BodyStateT state;
  state.protocol_version = 2;
  state.state_id = sequence;
  state.roster_version = 1;
  state.ts_state_real_us = state.ts_state_pub_us = state.ts_consumed_us = wall_us;
  state.attitude_valid = true;
  state.aorta_header = std::make_unique<aorta::sys::AortaHeaderT>();
  state.aorta_header->sequence = sequence;
  state.aorta_header->publish_stamp_ns = wall_us * 1000;
  state.aorta_header->publisher_node = "sdk_ci_mock";
  std::vector<lowlevel::BodyJointState> joints(31);
  for (size_t j = 0; j < joints.size(); ++j) {
    const bool present = j != 14;
    const float q = present ? static_cast<float>(sequence % 1024) * .0001f + j * .01f
                            : std::numeric_limits<float>::quiet_NaN();
    joints[j] = lowlevel::BodyJointState(
        0, wall_us, wall_us, q, .1f, .2f, lowlevel::JointError_NONE);
  }
  state.joints = std::move(joints);
  for (int j = 0; j < 2; ++j) {
    auto imu = std::make_unique<lowlevel::BodyImuSampleT>();
    imu->location = static_cast<lowlevel::BodyImuLocation>(j);
    std::array<float, 4> quat{1, 0, 0, 0};
    std::array<float, 3> gyro{0, 0, 0}, accel{0, 0, 9.81f}, rpy{0, 0, 0};
    imu->state = std::make_unique<lowlevel::BodyImuState>(quat, gyro, accel, rpy);
    imu->status = lowlevel::BodySampleStatus_OK;
    imu->ts_ingested_us = imu->ts_assembled_us = wall_us;
    state.imu_states.push_back(std::move(imu));
  }
  flatbuffers::FlatBufferBuilder builder;
  builder.Finish(lowlevel::BodyState::Pack(builder, &state));
  return {builder.GetBufferPointer(), builder.GetBufferPointer() + builder.GetSize()};
}
