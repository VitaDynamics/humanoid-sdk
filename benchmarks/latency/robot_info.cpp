#include <atomic>
#include <chrono>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>
#include "aorta_abi.h"
#include "get_robot_info_generated.h"

namespace {
std::atomic<uint64_t> requests{0}, errors{0};
std::vector<uint8_t> MakeInfo(uint64_t runtime) {
  // Synthetic benchmark topology/capabilities, never reported as robot readback.
  lowlevel::RobotInfoT info;
  info.protocol_version = 2;
  info.ts_info_us = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  info.body = std::make_unique<lowlevel::BodyInfoT>();
  info.body->slot_count = 31;
  info.body->roster_version = 1;
  for (uint16_t j = 0; j < 31; ++j) {
    const bool present = j != 14;
    info.body->joint_names.push_back("mock_joint_" + std::to_string(j));
    info.body->joints.emplace_back(j, lowlevel::BodySlotRole_ACTIVE, !present,
        present ? 31 : 0, present ? 31 : 0, -3.f, 3.f, -10.f, 10.f,
        -20.f, 20.f, 0.f, 200.f, 0.f, 10.f);
    auto identity = std::make_unique<lowlevel::BodyMotorIdentityT>();
    identity->bus = "mock_can";
    identity->node_id = j + 1;
    identity->present = present;
    info.body->motor_identities.push_back(std::move(identity));
  }
  for (int side = 1; side <= 2; ++side) {
    auto entry = std::make_unique<lowlevel::EndEffectorCapabilityT>();
    entry->side = static_cast<lowlevel::ee::EndEffectorSide>(side);
    entry->is_mock = true;  // no attached EE Info in this synthetic reply
    entry->ready = false;
    entry->status = aorta::services::common::ServiceStatus_UNAVAILABLE;
    entry->reason = "synthetic endpoint not configured";
    info.end_effectors.push_back(std::move(entry));
  }
  aorta::services::humanoid_locomotion::GetRobotInfoResponseT reply;
  reply.status = aorta::services::common::ServiceStatus_SUCCESS;
  reply.runtime_id = runtime;
  reply.info = std::make_unique<lowlevel::RobotInfoT>(std::move(info));
  flatbuffers::FlatBufferBuilder builder;
  builder.Finish(aorta::services::humanoid_locomotion::GetRobotInfoResponse::Pack(builder, &reply));
  return {builder.GetBufferPointer(), builder.GetBufferPointer() + builder.GetSize()};
}
void ReplyInfo(const aorta_bytes_view_t *view, aorta_service_request_t request, void *opaque) noexcept {
  ++requests;
  try {
    if (!view || view->total_len > 65536) throw std::runtime_error("invalid request payload");
    std::vector<uint8_t> bytes;
    bytes.reserve(view->total_len);
    for (size_t i = 0; i < view->slice_count; ++i) {
      const auto &s = view->slices[i];
      if (s.len > view->total_len - bytes.size()) throw std::runtime_error("invalid request slice");
      if (s.len) bytes.insert(bytes.end(), s.data, s.data + s.len);
    }
    flatbuffers::Verifier verifier(bytes.data(), bytes.size());
    if (bytes.size() != view->total_len ||
        !aorta::services::humanoid_locomotion::VerifyGetRobotInfoRequestBuffer(verifier))
      throw std::runtime_error("invalid GetRobotInfoRequest");
    const auto reply = MakeInfo(*static_cast<const uint64_t *>(opaque));
    // The pinned FFI copies synchronously when release is null; reply remains alive here.
    aorta_owned_bytes_slice_t slice{reply.data(), reply.size(), nullptr, nullptr};
    aorta_owned_bytes_view_t owned{&slice, 1, reply.size()};
    if (aorta_service_request_reply(request, &owned) != 0) ++errors;
  } catch (const std::exception &e) {
    ++errors;
    aorta_service_request_reply_error(request, e.what());
  }
}
}  // namespace
uint64_t InfoRequests() { return requests.load(); }
uint64_t InfoErrors() { return errors.load(); }
aorta_service_t StartInfoService(aorta_node_t node, const std::string &endpoint,
                                const std::string &path, const std::string &hash,
                                uint64_t *runtime) {
  std::ifstream file(path, std::ios::binary);
  std::vector<uint8_t> bfbs((std::istreambuf_iterator<char>(file)), {});
  if (bfbs.empty()) throw std::runtime_error("missing info BFBS");
  aorta_service_options_t options{};
  options.service_schema_hash_hex = reinterpret_cast<const uint8_t *>(hash.data());
  options.service_schema_hash_len = hash.size();
  options.service_schema_bfbs = bfbs.data();
  options.service_schema_bfbs_len = bfbs.size();
  static const std::string request = "aorta.services.humanoid_locomotion.GetRobotInfoRequest";
  static const std::string response = "aorta.services.humanoid_locomotion.GetRobotInfoResponse";
  options.request_root_type = request.c_str(); options.request_root_type_len = request.size();
  options.response_root_type = response.c_str(); options.response_root_type_len = response.size();
  aorta_service_t service = 0;
  const int rc = aorta_service_new_with_options(node, endpoint.c_str(), &options, ReplyInfo,
                                                runtime, &service);
  if (rc != 0) throw std::runtime_error("Aorta info service result=" + std::to_string(rc));
  return service;
}
