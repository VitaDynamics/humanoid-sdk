#include <array>
#include <chrono>
#include <vector>

#include "humanoid_low_state_generated.h"

// Separate translation unit: both BFBS-generated headers contain common types.
std::vector<uint8_t> MakeState(uint16_t sequence) {
  const auto wall_us = std::chrono::duration_cast<std::chrono::microseconds>(
                           std::chrono::system_clock::now().time_since_epoch())
                           .count();
  lowlevel::HumanoidLowStateT state;
  state.state_id = sequence;
  state.ts_state_real = state.ts_state_pub = state.ts_consumed_us = wall_us;
  state.attitude_valid = true;
  state.aorta_header = std::make_unique<aorta::sys::AortaHeaderT>();
  state.aorta_header->sequence = sequence;
  state.aorta_header->publish_stamp_ns = wall_us * 1000;
  state.aorta_header->publisher_node = "sdk_ci_mock";
  for (size_t j = 0; j < 44; ++j) {
    const float q = static_cast<float>(sequence % 1024) * .0001f + j * .01f;
    auto m = std::make_unique<lowlevel::HumanoidMotorStateT>();
    m->ts_ingested_us = m->ts_assembled_us = wall_us;
    m->q = m->q_raw = q;
    m->dq = m->dq_raw = .1f;
    m->tau_est = .2f;
    m->temperature = 30;
    m->mode = 1;
    m->slot_index = j;
    m->slot_role = j < 42 ? lowlevel::HumanoidSlotRole_ACTIVE
                          : lowlevel::HumanoidSlotRole_RESERVED;
    m->thermal_integral = {1, 2, 3};
    state.motor_states.push_back(std::move(m));
    state.motor_state.emplace_back(1, q, .1f, 0.f, .2f, q, .1f, 0.f, 30, 0);
    state.relative_motor_cmd.emplace_back(j < 42 ? 1 : 0, q, 0, 0, 0, 0);
  }
  for (int j = 0; j < 2; ++j) {
    auto imu = std::make_unique<lowlevel::ImuSampleT>();
    imu->location = static_cast<lowlevel::ImuLocation>(j);
    std::array<float, 4> quat{1, 0, 0, 0};
    std::array<float, 3> gyro{0, 0, 0}, accel{0, 0, 9.81f}, rpy{0, 0, 0};
    imu->state = std::make_unique<lowlevel::ImuState>(quat, gyro, accel, rpy);
    imu->status = 1;
    imu->ts_ingested_us = imu->ts_assembled_us = wall_us;
    state.imu_states.push_back(std::move(imu));
  }
  flatbuffers::FlatBufferBuilder builder;
  builder.Finish(lowlevel::HumanoidLowState::Pack(builder, &state));
  return {builder.GetBufferPointer(),
          builder.GetBufferPointer() + builder.GetSize()};
}
