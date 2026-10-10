#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

#include "aorta_abi.h"
#include "low_cmd_generated.h"

std::vector<uint8_t> MakeState(uint16_t sequence);
using Clock = std::chrono::steady_clock;
uint64_t Now() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             Clock::now().time_since_epoch())
      .count();
}
void Check(int rc) {
  if (rc != 0) throw std::runtime_error("Aorta result=" + std::to_string(rc));
}
struct Reply {
  uint64_t session, sequence, cmd_id, callback_ns, copied_ns;
  bool valid;
};
struct Capture {
  std::mutex mutex;
  std::vector<Reply> replies;
  uint64_t session;
  size_t errors = 0;
};
void Receive(const aorta_bytes_view_t *view, const aorta_schema_hash_view_t *,
             const aorta_msg_context_t *, void *opaque) noexcept {
  auto &capture = *static_cast<Capture *>(opaque);
  const auto entered = Now();
  try {
    if (!view || view->total_len > 65536)
      throw std::runtime_error("invalid payload");
    std::vector<uint8_t> bytes;
    bytes.reserve(view->total_len);
    for (size_t j = 0; j < view->slice_count; ++j) {
      const auto &s = view->slices[j];
      if (s.len > view->total_len - bytes.size())
        throw std::runtime_error("invalid slice");
      if (s.len) bytes.insert(bytes.end(), s.data, s.data + s.len);
    }
    flatbuffers::Verifier verifier(bytes.data(), bytes.size());
    if (bytes.size() != view->total_len ||
        !lowlevel::VerifyLowCmdBuffer(verifier))
      throw std::runtime_error("invalid LowCmd FlatBuffer");
    const auto *cmd = lowlevel::GetLowCmd(bytes.data());
    const auto *motors = cmd->motor_cmd();
    if (!motors || motors->size() != 42)
      throw std::runtime_error("expected 42 slots");
    std::array<lowlevel::MotorCmd, 42> copied;
    for (size_t j = 0; j < copied.size(); ++j) copied[j] = *motors->Get(j);
    Reply row{cmd->external_session_id(),
              cmd->external_sequence(),
              cmd->cmd_id(),
              entered,
              Now(),
              true};
    row.valid = row.session == capture.session && row.sequence > 0 &&
                row.sequence <= 65000 && row.cmd_id == row.sequence &&
                cmd->external_version() == 1;
    for (size_t j = 0; j < copied.size(); ++j) {
      const float q =
          static_cast<float>(row.sequence % 1024) * .0001f + j * .01f;
      const auto &m = copied[j];
      row.valid = row.valid && m.mode() == 1 && std::isfinite(m.q()) &&
                  std::abs(m.q() - q) < 1e-6f && m.dq() == .1f &&
                  m.tau() == .2f && m.kp() == 1.f && m.kd() == .5f;
    }
    std::lock_guard<std::mutex> lock(capture.mutex);
    if (capture.replies.size() >= 65000)
      ++capture.errors;
    else
      capture.replies.push_back(row);
  } catch (...) {
    std::lock_guard<std::mutex> lock(capture.mutex);
    ++capture.errors;
  }
}

int main(int argc, char **argv) {
  aorta_node_t node = 0;
  Capture capture;
  try {
    if (argc != 6)
      throw std::runtime_error("mock DIRECTORY SESSION RATE SECONDS SCHEMA");
    const std::filesystem::path directory(argv[1]);
    capture.session = std::stoull(argv[2]);
    const int hz = std::stoi(argv[3]);
    const double seconds = std::stod(argv[4]);
    if (!capture.session || hz < 1 || hz > 1000 || !std::isfinite(seconds) ||
        seconds < 1 || seconds * hz + 1 > 65000)
      throw std::runtime_error("invalid bounded benchmark options");
    if (aorta_abi_version() != 8)
      throw std::runtime_error("Aorta ABI mismatch");
    const std::string prefix =
        "/bench/sdk_latency/" + std::to_string(capture.session);
    capture.replies.reserve(65000);
    Check(aorta_node_new("sdk_ci_mock", "default", &node));
    auto command_qos = aorta_qos_preset(AORTA_QOS_PRESET_REALTIME_CONTROL);
    aorta_sub_t subscriber = 0;
    Check(aorta_subscriber_new(node, (prefix + "/command").c_str(),
                               &command_qos, nullptr, Receive, &capture,
                               &subscriber));
    std::ifstream schema_file(argv[5], std::ios::binary);
    std::vector<uint8_t> schema((std::istreambuf_iterator<char>(schema_file)),
                                {});
    if (schema.empty()) throw std::runtime_error("missing schema BFBS");
    aorta_pub_options_t options{};
    options.schema_bfbs = schema.data();
    options.schema_bfbs_len = schema.size();
    auto state_qos = aorta_qos_preset(AORTA_QOS_PRESET_SENSOR_DATA);
    aorta_pub_t publisher = 0;
    Check(aorta_publisher_new(node, (prefix + "/lowstate").c_str(), &state_qos,
                              &options, &publisher));
    uint8_t matched = 0;
    Check(aorta_publisher_wait_for_matching(publisher, 15000, &matched));
    if (!matched) throw std::runtime_error("state subscriber did not match");
    const auto ready_deadline = Clock::now() + std::chrono::seconds(15);
    while (!std::filesystem::exists(directory / "client.ready")) {
      if (Clock::now() > ready_deadline)
        throw std::runtime_error("client not ready");
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    struct Sent {
      uint64_t sequence, publish_ns, return_ns, size;
    };
    std::vector<Sent> sent;
    sent.reserve(65000);
    auto next = Clock::now();
    for (uint64_t seq = 1; seq <= 65000; ++seq) {
      std::this_thread::sleep_until(next);
      auto bytes = MakeState(seq);
      const auto begin = Now();
      Check(aorta_publisher_publish(publisher, bytes.data(), bytes.size()));
      sent.push_back({seq, begin, Now(), bytes.size()});
      if (begin - sent.front().publish_ns >=
          static_cast<uint64_t>(seconds * 1e9))
        break;
      next += std::chrono::nanoseconds(1000000000 / hz);
      if (next < Clock::now()) next = Clock::now();  // no catch-up bursts
    }
    std::this_thread::sleep_for(
        std::chrono::seconds(3));  // bounded reply drain
    Check(aorta_node_free(node));
    node = 0;  // callbacks quiesce before Capture storage is released
    std::ofstream sent_file(directory / "sent.csv"),
        received(directory / "received.csv");
    sent_file << "sequence,publish_ns,return_ns,bytes\n";
    for (const auto &r : sent)
      sent_file << r.sequence << ',' << r.publish_ns << ',' << r.return_ns
                << ',' << r.size << '\n';
    std::lock_guard<std::mutex> lock(capture.mutex);
    received << "session,sequence,cmd_id,callback_ns,copied_ns,valid\n";
    for (const auto &r : capture.replies)
      received << r.session << ',' << r.sequence << ',' << r.cmd_id << ','
               << r.callback_ns << ',' << r.copied_ns << ',' << r.valid << '\n';
    sent_file.close();
    received.close();
    if (!sent_file || !received || capture.errors)
      throw std::runtime_error("capture/write errors");
    std::cout << "sent=" << sent.size()
              << " received=" << capture.replies.size() << '\n';
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    if (node) aorta_node_free(node);
    return 1;
  }
}
