#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

#include "aorta_abi.h"
#include "aorta_contract.h"
#include "body_cmd_generated.h"
#include "protocol.h"

extern "C" {
aorta_result_t aorta_subscriber_free(aorta_sub_t);
aorta_result_t aorta_publisher_free(aorta_pub_t);
const char* aorta_last_error();
}
extern char** environ;
namespace pm = production_mock;
namespace {
using Clock = std::chrono::steady_clock;
volatile std::sig_atomic_t stopped = 0;
void Stop(int) { stopped = 1; }
uint64_t Mono() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             Clock::now().time_since_epoch())
      .count();
}
uint64_t Wall() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}
bool Checked(int rc, const std::string& call) {
  if (!rc) return true;
  // The last-error pointer is thread-local and the next FFI call (including
  // cleanup) may replace it. Copy it before doing anything else with Aorta.
  const char* raw = aorta_last_error();
  const std::string error = raw ? raw : "";
  std::cerr << call << " rc=" << rc << " last_error=" << error << '\n';
  return false;
}
void Check(int rc, const std::string& call) {
  if (!Checked(rc, call)) throw std::runtime_error("Aorta endpoint failed; see named call above");
}
std::vector<uint8_t> Read(const std::filesystem::path& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("cannot read " + path.string());
  return {std::istreambuf_iterator<char>(f), {}};
}
std::vector<uint8_t> Copy(const aorta_bytes_view_t* view) {
  if (!view || view->total_len > 65536 || (view->slice_count && !view->slices))
    throw std::runtime_error("invalid payload size/slices");
  std::vector<uint8_t> bytes;
  bytes.reserve(view->total_len);
  for (size_t i = 0; i < view->slice_count; ++i) {
    const auto& slice = view->slices[i];
    if (slice.len > view->total_len - bytes.size() ||
        (slice.len && !slice.data))
      throw std::runtime_error("invalid payload slice");
    if (slice.len)
      bytes.insert(bytes.end(), slice.data, slice.data + slice.len);
  }
  if (bytes.size() != view->total_len)
    throw std::runtime_error("payload length mismatch");
  return bytes;
}
std::string Quoted(const std::string& value) {
  std::string result = "\"";
  for (char ch : value) result += ch == '"' ? "\"\"" : std::string(1, ch);
  return result + "\"";
}
constexpr size_t kRows = 1000000;
struct Received {
  uint64_t session, sequence, state_id, state_real_us, created_us, send_us;
  uint64_t received_ns, accepted_ns;
  bool valid;
};
struct Applied {
  uint64_t session, sequence, wall_ns, dispatch_ns;
};
struct Sent {
  uint64_t state_id, published_ns, returned_ns, real_us, session, sequence,
      first_us;
};
struct ControlRow {
  pm::Request request;
  pm::Ack ack;
  uint64_t wall_ns;
  bool ack_sent;
};
struct Context {
  std::mutex mutex;
  pm::Protocol protocol;
  aorta_pub_t ack = 0;
  std::vector<Received> received;
  std::vector<Applied> applied;
  std::vector<ControlRow> controls;
  uint64_t parse_errors = 0, info_requests = 0, info_errors = 0;
  uint64_t ack_sequence = 0;
  uint64_t apply_period_ns = 0, pending_tick_ns = 0;
  std::atomic<bool> failed{false};
  std::atomic<bool> endpoint_failed{false};
  Context(uint64_t seed, uint64_t timeout, uint64_t prepare, int reject)
      : protocol(seed, timeout, prepare, reject) {
    received.reserve(kRows);
    applied.reserve(kRows);
    controls.reserve(kRows);
  }
};
void ReceiveCommand(const aorta_bytes_view_t* view,
                    const aorta_schema_hash_view_t*, const aorta_msg_context_t*,
                    void* opaque) noexcept {
  auto& ctx = *static_cast<Context*>(opaque);
  const auto received = Wall();
  try {
    auto bytes = Copy(view);
    flatbuffers::Verifier verifier(bytes.data(), bytes.size());
    if (!lowlevel::VerifyBodyCmdBuffer(verifier))
      throw std::runtime_error("invalid BodyCmd FlatBuffer");
    const auto* value = lowlevel::GetBodyCmd(bytes.data());
    if (value->protocol_version() != 2 ||
        value->action() != lowlevel::BodyCmdAction_EXTERNAL ||
        !value->joints() || value->joints()->size() != pm::kSlots)
      throw std::runtime_error("invalid BodyCmd protocol/action/slot count");
    pm::Command cmd;
    cmd.session = value->session_id();
    cmd.sequence = value->sequence();
    cmd.input_state_id = value->input_state_id();
    cmd.input_state_real_us = value->input_state_ts_real_us();
    cmd.client_created_us = value->ts_client_created_us();
    cmd.client_send_us = value->ts_client_send_us();
    for (size_t j = 0; j < pm::kSlots; ++j) {
      const auto* joint = value->joints()->Get(j);
      cmd.q[j] = joint->q();
      cmd.dq[j] = joint->dq();
      cmd.tau[j] = joint->tau();
      cmd.kp[j] = joint->kp();
      cmd.kd[j] = joint->kd();
    }
    std::lock_guard<std::mutex> lock(ctx.mutex);
    if (ctx.received.size() >= kRows) {
      ctx.failed = true;
      return;
    }
    const bool valid = ctx.protocol.Accept(cmd, Mono());
    const auto accepted = Wall();
    if (valid) ctx.pending_tick_ns = pm::NextApplyTick(accepted, ctx.apply_period_ns);
    ctx.received.push_back({cmd.session, cmd.sequence, cmd.input_state_id,
                            cmd.input_state_real_us, cmd.client_created_us,
                            cmd.client_send_us, received, accepted, valid});
  } catch (const std::exception& e) {
    std::lock_guard<std::mutex> lock(ctx.mutex);
    ++ctx.parse_errors;
    std::cerr << "ReceiveCommand: " << e.what() << '\n';
  } catch (...) {
    std::lock_guard<std::mutex> lock(ctx.mutex);
    ++ctx.parse_errors;
    std::cerr << "ReceiveCommand: unknown exception\n";
  }
}
void ReceiveRequest(const aorta_bytes_view_t* view,
                    const aorta_schema_hash_view_t*, const aorta_msg_context_t*,
                    void* opaque) noexcept {
  auto& ctx = *static_cast<Context*>(opaque);
  try {
    const auto request = pm::DecodeRequest(Copy(view));
    std::lock_guard<std::mutex> lock(ctx.mutex);
    if (ctx.controls.size() >= kRows) {
      ctx.failed = true;
      return;
    }
    // Commit the proposed state only after its ACK was published.
    auto proposed = ctx.protocol;
    const auto reply = proposed.RequestControl(request, Mono());
    const auto wall = Wall();
    const auto bytes = pm::EncodeAck(reply, wall, ++ctx.ack_sequence);
    const bool sent = Checked(aorta_publisher_publish(ctx.ack, bytes.data(), bytes.size()),
                              "aorta_publisher_publish(/sdk/foot_humanoid/control/ack)");
    if (sent)
      ctx.protocol = std::move(proposed);
    else
      ctx.endpoint_failed = true;
    ctx.controls.push_back({request, reply, wall, sent});
  } catch (const std::exception& e) {
    std::lock_guard<std::mutex> lock(ctx.mutex);
    ++ctx.parse_errors;
    std::cerr << "ReceiveRequest: " << e.what() << '\n';
  } catch (...) {
    std::lock_guard<std::mutex> lock(ctx.mutex);
    ++ctx.parse_errors;
    std::cerr << "ReceiveRequest: unknown exception\n";
  }
}
void ReplyInfo(const aorta_bytes_view_t* view, aorta_service_request_t request,
               void* opaque) noexcept {
  auto& ctx = *static_cast<Context*>(opaque);
  bool reply_consumed = false;
  try {
    if (!pm::ValidInfoRequest(Copy(view)))
      throw std::runtime_error("invalid info request");
    const auto bytes = pm::EncodeInfo();
    aorta_owned_bytes_slice_t slice{bytes.data(), bytes.size(), nullptr,
                                    nullptr};
    aorta_owned_bytes_view_t owned{&slice, 1, bytes.size()};
    // This FFI call consumes the request on both success and failure.
    reply_consumed = true;
    const bool replied = Checked(aorta_service_request_reply(request, &owned),
                                  "aorta_service_request_reply(get_robot_info)");
    std::lock_guard<std::mutex> lock(ctx.mutex);
    if (replied)
      ++ctx.info_requests;
    else {
      ++ctx.info_errors;
      ctx.endpoint_failed = true;
    }
  } catch (const std::exception& e) {
    std::cerr << "ReplyInfo: " << e.what() << '\n';
    if (!reply_consumed && !Checked(aorta_service_request_reply_error(request, e.what()),
                                      "aorta_service_request_reply_error(get_robot_info)"))
      ctx.endpoint_failed = true;
    std::lock_guard<std::mutex> lock(ctx.mutex);
    ++ctx.info_errors;
  }
}
std::string ExpectedConfig(int port) {
  return "{\"mode\":\"peer\",\"scouting\":{\"multicast\":{\"enabled\":false},"
         "\"gossip\":{\"enabled\":false}},\"transport\":{\"shared_memory\":{"
         "\"enabled\":false},"
         "\"unicast\":{\"compression\":{\"enabled\":false}}},\"listen\":{"
         "\"endpoints\":["
         "\"tcp/127.0.0.1:" +
         std::to_string(port) + "\"]},\"connect\":{\"endpoints\":[]}}";
}
void CheckEnvironment(const std::filesystem::path& directory,
                      const std::string& group, int port) {
  pm::aorta_contract::ValidateGroupLength(group);
  const char* effective = std::getenv("AORTA_GROUP");
  const char* uri = std::getenv("ZENOH_SESSION_CONFIG_URI");
  const char* logging = std::getenv("AORTA_LOG_TOPIC_ENABLED");
  if (group.rfind(pm::aorta_contract::GROUP_PREFIX, 0) != 0 ||
      group.find_first_not_of(
          "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_") !=
          std::string::npos ||
      !effective || group != effective || !logging ||
      std::string(logging) != "false" || !uri ||
      std::filesystem::path(uri) != directory / "mock-session.json")
    throw std::runtime_error(
        "use the isolated runner; unsafe group/session environment");
  for (char** item = environ; *item; ++item) {
    const std::string key(*item, std::string(*item).find('='));
    if ((key.rfind("ZENOH_", 0) == 0 || key.rfind("AORTA_", 0) == 0) &&
        key != "AORTA_GROUP" && key != "AORTA_LOG_TOPIC_ENABLED" &&
        key != "ZENOH_SESSION_CONFIG_URI")
      throw std::runtime_error("unexpected transport environment: " + key);
  }
  const auto raw = Read(directory / "mock-session.json");
  if (std::string(raw.begin(), raw.end()) != ExpectedConfig(port))
    throw std::runtime_error(
        "mock peer profile differs from strict loopback configuration");
}
struct Resources {
  aorta_node_t node = 0;
  aorta_service_t service = 0;
  std::vector<aorta_sub_t> subscribers;
  std::vector<aorta_pub_t> publishers;
  bool Close() {
    bool ok = true;
    for (auto sub : subscribers)
      ok = Checked(aorta_subscriber_free(sub), "aorta_subscriber_free") && ok;
    subscribers.clear();
    if (service) ok = Checked(aorta_service_free(service), "aorta_service_free") && ok;
    service = 0;
    for (auto pub : publishers)
      ok = Checked(aorta_publisher_free(pub), "aorta_publisher_free") && ok;
    publishers.clear();
    if (node) ok = Checked(aorta_node_free(node), "aorta_node_free") && ok;
    node = 0;
    return ok;
  }
  ~Resources() { Close(); }
};
aorta_pub_t Publisher(Resources& r, const std::filesystem::path& schemas,
                      const std::string& key, const char* topic,
                      const char* root, aorta_qos_preset_t preset) {
  const auto bfbs = Read(schemas / (key + ".bfbs"));
  if (bfbs.empty()) throw std::runtime_error("empty BFBS");
  aorta_pub_options_t options{};
  options.schema_bfbs = bfbs.data();
  options.schema_bfbs_len = bfbs.size();
  options.schema_root_type = root;
  const auto qos = aorta_qos_preset(preset);
  aorta_pub_t pub = 0;
  Check(aorta_publisher_new(r.node, topic, &qos, &options, &pub),
        std::string("aorta_publisher_new(") + topic + ")");
  r.publishers.push_back(pub);
  return pub;
}
void Advance(uint64_t& next, uint64_t period, uint64_t now) {
  next += ((now - next) / period + 1) * period;
}
}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc != 11 || std::string(argv[1]) != "--production-topics")
      throw std::runtime_error(
          "use run.py --mode production-topics; not a default benchmark mode");
    const auto directory = std::filesystem::absolute(argv[2]);
    const std::string group(argv[3]);
    const int port = std::stoi(argv[4]);
    const double hz = std::stod(argv[5]), apply_hz = std::stod(argv[6]),
                 seconds = std::stod(argv[7]);
    const uint64_t stream_ms = std::stoull(argv[8]),
                   prepare_ms = std::stoull(argv[9]);
    const int reject = std::stoi(argv[10]);
    if (port < 1024 || port > 65535 || !std::isfinite(hz) || hz < 1 ||
        hz > 2000 || !std::isfinite(apply_hz) || apply_hz < 1 ||
        apply_hz > 2000 || !std::isfinite(seconds) || seconds < 1 ||
        seconds > 600 || hz * seconds + 1 > kRows || stream_ms < 1 ||
        stream_ms > 60000 || prepare_ms < 1 || prepare_ms > 60000 ||
        reject < 0 || reject > 3)
      throw std::runtime_error("invalid bounded mock parameters");
    CheckEnvironment(directory, group, port);
    for (const auto* name : {"states.csv", "received.csv", "applied.csv",
                             "control.csv", "mock-result.json"})
      if (std::filesystem::exists(directory / name))
        throw std::runtime_error("refusing to overwrite a previous capture");
    if (aorta_abi_version() != AORTA_EXPECTED_ABI)
      throw std::runtime_error("Aorta ABI mismatch");
    Context context(Wall(), stream_ms * 1000000, prepare_ms * 1000000, reject);
    context.apply_period_ns = static_cast<uint64_t>(1e9 / apply_hz);
    std::ifstream initial(directory / "initial-q.txt");
    std::array<float, pm::kSlots> q{};
    for (auto& v : q)
      if (!(initial >> v) || !std::isfinite(v) || std::abs(v) > 10)
        throw std::runtime_error(
            "initial-q requires31 finite values in synthetic info range");
    std::string trailing;
    if (initial >> trailing) throw std::runtime_error("extra initial-q values");
    context.protocol.SetInitialQ(q);
    std::ifstream imu_file(directory / "initial-imu.txt");
    std::array<std::array<float, 4>, 2> imu_q{};
    for (auto& quat : imu_q) {
      float norm = 0;
      for (auto& v : quat) {
        if (!(imu_file >> v) || !std::isfinite(v))
          throw std::runtime_error("invalid initial IMU fixture");
        norm += v * v;
      }
      if (std::abs(norm - 1.f) > .001f)
        throw std::runtime_error("IMU quaternion must be normalized");
    }
    if (imu_file >> trailing)
      throw std::runtime_error("extra initial IMU values");
    context.protocol.SetInitialImu(imu_q);
    // Declare resources after context: callbacks are drained before context
    // dies.
    Resources resources;
    const auto start = Mono();
    std::vector<Sent> sent;
    int endpoint_rc = 0;
    try {
      Check(aorta_node_new("production_topic_mock", group.c_str(),
                           &resources.node), "aorta_node_new(production_topic_mock)");
      const auto schemas =
          std::filesystem::absolute(argv[0]).parent_path() / "generated";
      auto state_pub = Publisher(
          resources, schemas, "body_state", "/sdk/foot_humanoid/body_state",
          "lowlevel.BodyState", AORTA_QOS_PRESET_SENSOR_DATA);
      context.ack = Publisher(
          resources, schemas, "control_ack", "/sdk/foot_humanoid/control/ack",
          "locomotion_sdk.ControlAck", AORTA_QOS_PRESET_REALTIME_CONTROL);
      auto status_pub = Publisher(resources, schemas, "control_status",
                                  "/sdk/foot_humanoid/control/status",
                                  "locomotion_sdk.ControlStatus",
                                  AORTA_QOS_PRESET_STATE_UPDATE);
      const auto command_qos =
          aorta_qos_preset(AORTA_QOS_PRESET_REALTIME_CONTROL);
      const auto request_qos =
          aorta_qos_preset(AORTA_QOS_PRESET_REALTIME_CONTROL);
      aorta_sub_t commands = 0, requests = 0;
      Check(aorta_subscriber_new(resources.node, "/sdk/foot_humanoid/body_cmd",
                                 &command_qos, nullptr, ReceiveCommand,
                                 &context, &commands),
            "aorta_subscriber_new(/sdk/foot_humanoid/body_cmd)");
      resources.subscribers.push_back(commands);
      Check(aorta_subscriber_new(
          resources.node, "/sdk/foot_humanoid/control/request", &request_qos,
          nullptr, ReceiveRequest, &context, &requests),
          "aorta_subscriber_new(/sdk/foot_humanoid/control/request)");
      resources.subscribers.push_back(requests);
      const auto info_bfbs = Read(schemas / "get_robot_info.bfbs");
      const auto hash_bytes = Read(schemas / "get_robot_info.sha256");
      aorta_service_options_t options{};
      options.service_schema_hash_hex = hash_bytes.data();
      options.service_schema_hash_len = hash_bytes.size();
      options.service_schema_bfbs = info_bfbs.data();
      options.service_schema_bfbs_len = info_bfbs.size();
      static const std::string request_root =
          "aorta.services.humanoid_locomotion.GetRobotInfoRequest";
      static const std::string response_root =
          "aorta.services.humanoid_locomotion.GetRobotInfoResponse";
      options.request_root_type = request_root.c_str();
      options.request_root_type_len = request_root.size();
      options.response_root_type = response_root.c_str();
      options.response_root_type_len = response_root.size();
      Check(aorta_service_new_with_options(
          resources.node, "/sdk/foot_humanoid/get_robot_info", &options,
          ReplyInfo, &context, &resources.service),
          "aorta_service_new_with_options(/sdk/foot_humanoid/get_robot_info)");
      std::signal(SIGINT, Stop);
      std::signal(SIGTERM, Stop);
      const auto finish = start + static_cast<uint64_t>(seconds * 1e9);
      uint64_t next_state = start, next_apply = start, next_status = start,
               state_id = 0;
      uint64_t status_publication = 0;
      const uint64_t state_period = static_cast<uint64_t>(1e9 / hz);
      const uint64_t apply_period = static_cast<uint64_t>(1e9 / apply_hz);
      sent.reserve(static_cast<size_t>(hz * seconds + 1));
      while (!stopped && !context.failed && !context.endpoint_failed && Mono() < finish) {
        const auto now = Mono();
        if (now >= next_apply) {
          std::lock_guard<std::mutex> lock(context.mutex);
          const auto wall = Wall();
          const auto tick = context.pending_tick_ns;
          if (tick && wall >= tick && context.protocol.Tick(Mono(), tick / 1000)) {
            const auto& out = context.protocol.output();
            if (context.applied.size() >= kRows)
              context.failed = true;
            else
              context.applied.push_back({out.session, out.sequence, tick, wall});
            context.pending_tick_ns = 0;
          }
          Advance(next_apply, apply_period, now);
        }
        if (now >= next_state) {
          pm::Output output;
          {
            std::lock_guard<std::mutex> lock(context.mutex);
            output = context.protocol.output();
          }
          const auto real_us = Wall() / 1000;
          auto bytes = pm::EncodeState(++state_id, output, real_us);
          const auto wall = Wall();
          pm::StampState(bytes, wall);
          Check(aorta_publisher_publish(state_pub, bytes.data(), bytes.size()),
                "aorta_publisher_publish(/sdk/foot_humanoid/body_state)");
          if (sent.size() >= kRows)
            context.failed = true;
          else
            sent.push_back({state_id, wall, Wall(), real_us, output.session,
                            output.sequence, output.first_output_us});
          Advance(next_state, state_period, now);
        }
        if (now >= next_status) {
          pm::Status status;
          {
            std::lock_guard<std::mutex> lock(context.mutex);
            context.protocol.Watchdog(Mono());
            status = context.protocol.status();
          }
          const auto bytes =
              pm::EncodeStatus(status, Mono(), Wall(), ++status_publication);
          Check(
              aorta_publisher_publish(status_pub, bytes.data(), bytes.size()),
              "aorta_publisher_publish(/sdk/foot_humanoid/control/status)");
          Advance(next_status, 20000000, now);
        }
        std::this_thread::sleep_until(
            Clock::time_point(std::chrono::nanoseconds(
                std::min(next_state, std::min(next_apply, next_status)))));
      }
    } catch (const std::exception& e) {
      endpoint_rc = 2;
      std::cerr << e.what() << '\n';
    }
    // Drain callbacks and close every resource before deciding the verdict.
    if (!resources.Close() || context.endpoint_failed) endpoint_rc = 2;
    std::ofstream states(directory / "states.csv"),
        received(directory / "received.csv");
    std::ofstream applied(directory / "applied.csv"),
        controls(directory / "control.csv");
    states << "state_id,publish_ns,publish_return_ns,state_real_us,applied_"
              "session_id,applied_sequence,ts_first_output_us\n";
    for (const auto& r : sent)
      states << r.state_id << ',' << r.published_ns << ',' << r.returned_ns
             << ',' << r.real_us << ',' << r.session << ',' << r.sequence << ','
             << r.first_us << '\n';
    received
        << "session_id,sequence,input_state_id,input_state_ts_real_us,client_"
           "created_us,client_send_us,received_ns,accepted_ns,valid\n";
    for (const auto& r : context.received)
      received << r.session << ',' << r.sequence << ',' << r.state_id << ','
               << r.state_real_us << ',' << r.created_us << ',' << r.send_us
               << ',' << r.received_ns << ',' << r.accepted_ns << ',' << r.valid
               << '\n';
    applied << "session_id,sequence,first_output_ns,actual_dispatch_ns\n";
    for (const auto& r : context.applied)
      applied << r.session << ',' << r.sequence << ',' << r.wall_ns << ',' << r.dispatch_ns << '\n';
    controls << "request_id,client_id,operation,lease_id,accepted,error_code,"
                "reason,wall_ns,ack_sent\n";
    for (const auto& r : context.controls)
      controls << Quoted(r.request.request_id) << ','
               << Quoted(r.request.client_id) << ',' << r.request.operation
               << ',' << Quoted(r.ack.lease_id) << ',' << r.ack.accepted << ','
               << r.ack.error << ',' << Quoted(r.ack.reason) << ',' << r.wall_ns
               << ',' << r.ack_sent << '\n';
    states.flush();
    received.flush();
    applied.flush();
    controls.flush();
    if (!states || !received || !applied || !controls)
      throw std::runtime_error("capture write failed");
    std::ofstream result(directory / "mock-result.json");
    result << "{\"synthetic\":true,\"capture_overflow\":"
           << (context.failed ? "true" : "false")
           << ",\"endpoint_rc\":" << endpoint_rc
           << ",\"parse_errors\":" << context.parse_errors
           << ",\"info_requests\":" << context.info_requests
           << ",\"info_errors\":" << context.info_errors
           << ",\"overwritten_before_apply\":" << context.protocol.overwritten()
           << ",\"duration_ns\":" << Mono() - start
           << ",\"apply_period_ns\":" << context.apply_period_ns
           << ",\"control_phase_at_stop\":" << context.protocol.status().phase
           << ",\"stopped_by_signal\":" << (stopped ? "true" : "false")
           << "}\n";
    result.flush();
    if (!result) throw std::runtime_error("result write failed");
    return context.failed || endpoint_rc ? 2 : 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 2;
  }
}
