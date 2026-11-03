#include <array>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <mutex>
#include <memory>
#include <stdexcept>
#include <thread>
#include <vector>

#include "aorta_abi.h"
#include "body_cmd_generated.h"
#include "protocol.h"

std::vector<uint8_t> MakeState(uint64_t sequence, uint64_t runtime);
aorta_service_t StartInfoService(aorta_node_t node, const std::string &endpoint,
                                const std::string &bfbs, const std::string &hash,
                                uint64_t *runtime);
uint64_t InfoRequests();
uint64_t InfoErrors();
using Clock = std::chrono::steady_clock;
uint64_t Now() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             Clock::now().time_since_epoch())
      .count();
}
uint64_t RealtimeNow() {
  timespec stamp{};
  if (clock_gettime(CLOCK_REALTIME, &stamp) != 0)
    throw std::runtime_error("CLOCK_REALTIME failed");
  return static_cast<uint64_t>(stamp.tv_sec) * 1000000000ULL + stamp.tv_nsec;
}
void Check(int rc) {
  if (rc != 0) throw std::runtime_error("Aorta result=" + std::to_string(rc));
}
struct Reply {
  uint64_t session, sequence, input_state_id, callback_ns, copied_ns;
  uint64_t callback_realtime_ns, copied_realtime_ns;
  bool valid;
};
struct Capture {
  std::mutex mutex;
  std::vector<Reply> replies;
  uint64_t session = 0, runtime = 0;
  std::unique_ptr<production_mock::Protocol> protocol;
  aorta_pub_t ack = 0;
  uint64_t ack_sequence = 0;
  size_t errors = 0;
};

void ReceiveControl(const aorta_bytes_view_t *view, const aorta_schema_hash_view_t *,
                    const aorta_msg_context_t *, void *opaque) noexcept {
  auto &capture = *static_cast<Capture *>(opaque);
  try {
    if (!view || view->total_len > 65536) throw std::runtime_error("invalid control payload");
    std::vector<uint8_t> bytes;
    for (size_t i = 0; i < view->slice_count; ++i) {
      const auto &slice = view->slices[i];
      if (slice.len > view->total_len - bytes.size() || (slice.len && !slice.data))
        throw std::runtime_error("invalid control slice");
      if (slice.len) bytes.insert(bytes.end(), slice.data, slice.data + slice.len);
    }
    if (bytes.size() != view->total_len) throw std::runtime_error("invalid control length");
    const auto request = production_mock::DecodeRequest(bytes);
    std::lock_guard<std::mutex> lock(capture.mutex);
    auto proposed = *capture.protocol;
    const auto reply = proposed.RequestControl(request, Now());
    const auto payload = production_mock::EncodeAck(reply, RealtimeNow(), ++capture.ack_sequence);
    if (aorta_publisher_publish(capture.ack, payload.data(), payload.size()) != 0)
      throw std::runtime_error("control acknowledgement publish failed");
    *capture.protocol = std::move(proposed);
  } catch (...) {
    std::lock_guard<std::mutex> lock(capture.mutex);
    ++capture.errors;
  }
}
void Receive(const aorta_bytes_view_t *view, const aorta_schema_hash_view_t *,
             const aorta_msg_context_t *, void *opaque) noexcept {
  auto &capture = *static_cast<Capture *>(opaque);
  const auto entered = Now();
  try {
    const auto entered_realtime = RealtimeNow();
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
        !lowlevel::VerifyBodyCmdBuffer(verifier))
      throw std::runtime_error("invalid BodyCmd FlatBuffer");
    const auto *cmd = lowlevel::GetBodyCmd(bytes.data());
    const auto *motors = cmd->joints();
    if (!motors || motors->size() != 31)
      throw std::runtime_error("expected31 slots");
    std::array<lowlevel::BodyJointCmd, 31> copied;
    for (size_t j = 0; j < copied.size(); ++j) copied[j] = *motors->Get(j);
    Reply row{cmd->session_id(),
              cmd->sequence(),
              cmd->input_state_id(),
              entered,
              Now(),
              entered_realtime,
              RealtimeNow(),
              true};
    row.valid = row.session > 0 && row.sequence > 0 &&
                row.sequence <= 65000 &&
                cmd->protocol_version() == 3 &&
                cmd->runtime_id() == capture.runtime &&
                cmd->input_state_id() > 0 && cmd->input_state_id() <= 65000 &&
                cmd->input_state_ts_real_us() > 0 &&
                cmd->action() == lowlevel::BodyCmdAction_EXTERNAL;
    for (size_t j = 0; j < copied.size(); ++j) {
      if (j == 14) continue;  // declared uninstalled: no physical validation
      const float q =
          static_cast<float>(row.input_state_id % 1024) * .0001f + j * .01f;
      const auto &m = copied[j];
      row.valid = row.valid && std::isfinite(m.q()) &&
                  std::abs(m.q() - q) < 1e-6f && m.dq() == .1f &&
                  m.tau() == .2f && m.kp() == 1.f && m.kd() == .5f;
    }
    std::lock_guard<std::mutex> lock(capture.mutex);
    production_mock::Command command;
    command.runtime = cmd->runtime_id();
    command.control_id = cmd->control_id();
    command.session = row.session; command.sequence = row.sequence;
    command.input_state_id = row.input_state_id;
    command.input_state_real_us = cmd->input_state_ts_real_us();
    for (size_t j = 0; j < copied.size(); ++j) {
      command.q[j] = copied[j].q(); command.dq[j] = copied[j].dq();
      command.tau[j] = copied[j].tau(); command.kp[j] = copied[j].kp(); command.kd[j] = copied[j].kd();
    }
    row.valid = row.valid && capture.protocol->Accept(command, Now());
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
  aorta_service_t info_service = 0;
  Capture capture;
  try {
    if (argc != 8)
      throw std::runtime_error("mock DIRECTORY SESSION RATE SECONDS STATE_SCHEMA INFO_SCHEMA INFO_HASH");
    const std::filesystem::path directory(argv[1]);
    capture.session = std::stoull(argv[2]);
    const int hz = std::stoi(argv[3]);
    const double seconds = std::stod(argv[4]);
    if (!capture.session || hz < 1 || hz > 1000 || !std::isfinite(seconds) ||
        seconds < 1 || seconds * hz + 1 > 65000)
      throw std::runtime_error("invalid bounded benchmark options");
    if (aorta_abi_version() != AORTA_EXPECTED_ABI)
      throw std::runtime_error("Aorta ABI mismatch");
    const std::string prefix =
        "/bench/sdk_latency/" + std::to_string(capture.session);
    capture.replies.reserve(65000);
    capture.protocol = std::make_unique<production_mock::Protocol>(capture.session, 1000000000ULL, 10000000000ULL);
    capture.runtime = capture.protocol->runtime_id();
    Check(aorta_node_new("sdk_ci_mock", "default", &node));
    info_service = StartInfoService(node, prefix + "/get_robot_info", argv[6], argv[7], &capture.runtime);
    const auto schema_dir = std::filesystem::path(argv[5]).parent_path();
    const auto control_publisher = [&](const char *key, const char *root,
                                       const std::string &topic, aorta_qos_preset_t preset) {
      std::ifstream file(schema_dir / (std::string(key) + ".bfbs"), std::ios::binary);
      std::vector<uint8_t> schema((std::istreambuf_iterator<char>(file)), {});
      if (schema.empty()) throw std::runtime_error("missing control schema BFBS");
      aorta_pub_options_t options{};
      options.schema_root_type = root;
      options.schema_bfbs = schema.data(); options.schema_bfbs_len = schema.size();
      const auto qos = aorta_qos_preset(preset);
      aorta_pub_t publisher = 0;
      Check(aorta_publisher_new(node, topic.c_str(), &qos, &options, &publisher));
      return publisher;
    };
    capture.ack = control_publisher("control_ack", "locomotion_sdk.ControlAck",
                                   prefix + "/control/ack", AORTA_QOS_PRESET_REALTIME_CONTROL);
    const auto status_pub = control_publisher("control_status", "locomotion_sdk.ControlStatus",
                                               prefix + "/control/status", AORTA_QOS_PRESET_STATE_UPDATE);
    const auto request_qos = aorta_qos_preset(AORTA_QOS_PRESET_REALTIME_CONTROL);
    aorta_sub_t request_subscriber = 0;
    Check(aorta_subscriber_new(node, (prefix + "/control/request").c_str(), &request_qos,
                               nullptr, ReceiveControl, &capture, &request_subscriber));
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
    options.schema_root_type = "lowlevel.BodyState";
    options.schema_bfbs = schema.data();
    options.schema_bfbs_len = schema.size();
    auto state_qos = aorta_qos_preset(AORTA_QOS_PRESET_SENSOR_DATA);
    aorta_pub_t publisher = 0;
    Check(aorta_publisher_new(node, (prefix + "/lowstate").c_str(), &state_qos,
                              &options, &publisher));
    uint8_t matched = 0;
    Check(aorta_publisher_wait_for_matching(publisher, 15000, &matched));
    if (!matched) throw std::runtime_error("state subscriber did not match");
    uint64_t status_sequence = 0;
    const auto publish_status = [&] {
      production_mock::Status status;
      const auto now = Now();
      {
        std::lock_guard<std::mutex> lock(capture.mutex);
        capture.protocol->Watchdog(now);
        status = capture.protocol->status();
      }
      const auto bytes = production_mock::EncodeStatus(status, now, RealtimeNow(), ++status_sequence);
      Check(aorta_publisher_publish(status_pub, bytes.data(), bytes.size()));
    };
    const auto ready_deadline = Clock::now() + std::chrono::seconds(15);
    while (!std::filesystem::exists(directory / "client.ready")) {
      if (Clock::now() > ready_deadline)
        throw std::runtime_error("client not ready");
      publish_status();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    struct Sent {
      uint64_t sequence, publish_ns, return_ns, size;
      uint64_t publish_realtime_ns, return_realtime_ns;
    };
    std::vector<Sent> sent;
    sent.reserve(65000);
    auto next = Clock::now();
    auto next_status = next;
    for (uint64_t seq = 1; seq <= 65000; ++seq) {
      std::this_thread::sleep_until(next);
      if (Clock::now() >= next_status) {
        publish_status();
        next_status = Clock::now() + std::chrono::milliseconds(20);
      }
      auto bytes = MakeState(seq, capture.runtime);
      const auto begin = Now();
      const auto begin_realtime = RealtimeNow();
      Check(aorta_publisher_publish(publisher, bytes.data(), bytes.size()));
      sent.push_back(
          {seq, begin, Now(), bytes.size(), begin_realtime, RealtimeNow()});
      if (begin - sent.front().publish_ns >=
          static_cast<uint64_t>(seconds * 1e9))
        break;
      next += std::chrono::nanoseconds(1000000000 / hz);
      if (next < Clock::now()) next = Clock::now();  // no catch-up bursts
    }
    std::this_thread::sleep_for(
        std::chrono::seconds(3));  // bounded reply drain
    // Keep the original control provider alive until the client has sent
    // CANCEL, observed IDLE and closed. Reuse the existing stop/client files.
    { std::ofstream stop(directory / "stop"); if (!stop) throw std::runtime_error("cannot signal stop"); }
    const auto close_deadline = Clock::now() + std::chrono::seconds(15);
    while (!std::filesystem::exists(directory / "client.json")) {
      if (Clock::now() > close_deadline) throw std::runtime_error("client did not close");
      publish_status();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    Check(aorta_service_free(info_service));
    info_service = 0;
    Check(aorta_node_free(node));
    node = 0;  // callbacks quiesce before Capture storage is released
    std::ofstream sent_file(directory / "sent.csv"),
        received(directory / "received.csv");
    sent_file << "sequence,publish_ns,return_ns,bytes,publish_realtime_ns,return_realtime_ns\n";
    for (const auto &r : sent)
      sent_file << r.sequence << ',' << r.publish_ns << ',' << r.return_ns
                << ',' << r.size << ',' << r.publish_realtime_ns << ','
                << r.return_realtime_ns << '\n';
    std::lock_guard<std::mutex> lock(capture.mutex);
    received << "session,sequence,input_state_id,callback_ns,copied_ns,valid,callback_realtime_ns,copied_realtime_ns\n";
    for (const auto &r : capture.replies)
      received << r.session << ',' << r.sequence << ',' << r.input_state_id << ','
               << r.callback_ns << ',' << r.copied_ns << ',' << r.valid << ','
               << r.callback_realtime_ns << ',' << r.copied_realtime_ns << '\n';
    sent_file.close();
    received.close();
    std::ofstream info_receipt(directory / "robot_info.json");
    info_receipt << "{\"requests\":" << InfoRequests() << ",\"errors\":" << InfoErrors() << "}\n";
    info_receipt.close();
    if (!sent_file || !received || !info_receipt || capture.errors || InfoErrors() || !InfoRequests())
      throw std::runtime_error("capture/write errors");
    std::cout << "sent=" << sent.size()
              << " received=" << capture.replies.size() << '\n';
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    if (info_service) aorta_service_free(info_service);
    if (node) aorta_node_free(node);
    return 1;
  }
}
