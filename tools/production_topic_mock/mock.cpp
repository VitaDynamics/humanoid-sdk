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
#include <limits>
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
const char *aorta_last_error();
}
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
bool Checked(int rc, const std::string &call) {
  if (!rc) return true;
  // The last-error pointer is thread-local and the next FFI call (including
  // cleanup) may replace it. Copy it before doing anything else with Aorta.
  const char *raw = aorta_last_error();
  const std::string error = raw ? raw : "";
  std::cerr << call << " rc=" << rc << " last_error=" << error << '\n';
  return false;
}
void Check(int rc, const std::string &call) {
  if (!Checked(rc, call))
    throw std::runtime_error("Aorta endpoint failed; see named call above");
}
std::vector<uint8_t> Read(const std::filesystem::path &path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("cannot read " + path.string());
  return {std::istreambuf_iterator<char>(f), {}};
}
std::vector<uint8_t> Copy(const aorta_bytes_view_t *view) {
  if (!view || view->total_len > 65536 || (view->slice_count && !view->slices))
    throw std::runtime_error("invalid payload size/slices");
  std::vector<uint8_t> bytes;
  bytes.reserve(view->total_len);
  for (size_t i = 0; i < view->slice_count; ++i) {
    const auto &slice = view->slices[i];
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
struct Context {
  std::mutex mutex;
  pm::Protocol protocol;
  aorta_pub_t ack = 0;
  uint64_t accepted = 0, rejected = 0, applied = 0, controls = 0;
  uint64_t parse_errors = 0, info_requests = 0, info_errors = 0;
  uint64_t ee_accepted = 0, ee_rejected = 0, ee_applied = 0, touch_requests = 0;
  uint64_t ack_sequence = 0;
  uint64_t apply_period_ns = 0, pending_tick_ns = 0;
  std::atomic<bool> endpoint_failed{false};
  Context(uint64_t seed, uint64_t timeout, uint64_t prepare, int reject)
      : protocol(seed, timeout, prepare, reject) {}
};
void ReceiveCommand(const aorta_bytes_view_t *view,
                    const aorta_schema_hash_view_t *,
                    const aorta_msg_context_t *, void *opaque) noexcept {
  auto &ctx = *static_cast<Context *>(opaque);
  try {
    auto bytes = Copy(view);
    flatbuffers::Verifier verifier(bytes.data(), bytes.size());
    if (!lowlevel::VerifyBodyCmdBuffer(verifier))
      throw std::runtime_error("invalid BodyCmd FlatBuffer");
    const auto *value = lowlevel::GetBodyCmd(bytes.data());
    if (value->protocol_version() != 3 || !value->runtime_id() ||
        value->action() != lowlevel::BodyCmdAction_EXTERNAL ||
        !value->joints() || value->joints()->size() != pm::kSlots)
      throw std::runtime_error("invalid BodyCmd protocol/action/slot count");
    pm::Command cmd;
    cmd.runtime = value->runtime_id();
    cmd.control_id = value->control_id();
    cmd.session = value->session_id();
    cmd.sequence = value->sequence();
    cmd.input_state_id = value->input_state_id();
    cmd.input_state_real_us = value->input_state_ts_real_us();
    cmd.client_created_us = value->ts_client_created_us();
    cmd.client_send_us = value->ts_client_send_us();
    for (size_t j = 0; j < pm::kSlots; ++j) {
      const auto *joint = value->joints()->Get(j);
      cmd.q[j] = joint->q();
      cmd.dq[j] = joint->dq();
      cmd.tau[j] = joint->tau();
      cmd.kp[j] = joint->kp();
      cmd.kd[j] = joint->kd();
    }
    std::lock_guard<std::mutex> lock(ctx.mutex);
    const bool valid = ctx.protocol.Accept(cmd, Mono());
    const auto accepted = Wall();
    if (valid)
      ctx.pending_tick_ns = pm::NextApplyTick(accepted, ctx.apply_period_ns);
    if (valid)
      ++ctx.accepted;
    else
      ++ctx.rejected;
  } catch (const std::exception &e) {
    std::lock_guard<std::mutex> lock(ctx.mutex);
    ++ctx.parse_errors;
    std::cerr << "ReceiveCommand: " << e.what() << '\n';
  } catch (...) {
    std::lock_guard<std::mutex> lock(ctx.mutex);
    ++ctx.parse_errors;
    std::cerr << "ReceiveCommand: unknown exception\n";
  }
}
void ReceiveRequest(const aorta_bytes_view_t *view,
                    const aorta_schema_hash_view_t *,
                    const aorta_msg_context_t *, void *opaque) noexcept {
  auto &ctx = *static_cast<Context *>(opaque);
  try {
    const auto request = pm::DecodeRequest(Copy(view));
    std::lock_guard<std::mutex> lock(ctx.mutex);
    // Commit the proposed state only after its ACK was published.
    auto proposed = ctx.protocol;
    const auto reply = proposed.RequestControl(request, Mono());
    const auto wall = Wall();
    const auto bytes = pm::EncodeAck(reply, wall, ++ctx.ack_sequence);
    const bool sent =
        Checked(aorta_publisher_publish(ctx.ack, bytes.data(), bytes.size()),
                "aorta_publisher_publish(/sdk/foot_humanoid/control/ack)");
    if (sent)
      ctx.protocol = std::move(proposed);
    else
      ctx.endpoint_failed = true;
    ++ctx.controls;
  } catch (const std::exception &e) {
    std::lock_guard<std::mutex> lock(ctx.mutex);
    ++ctx.parse_errors;
    std::cerr << "ReceiveRequest: " << e.what() << '\n';
  } catch (...) {
    std::lock_guard<std::mutex> lock(ctx.mutex);
    ++ctx.parse_errors;
    std::cerr << "ReceiveRequest: unknown exception\n";
  }
}
void ReplyInfo(const aorta_bytes_view_t *view, aorta_service_request_t request,
               void *opaque) noexcept {
  auto &ctx = *static_cast<Context *>(opaque);
  bool reply_consumed = false;
  try {
    if (!pm::ValidInfoRequest(Copy(view)))
      throw std::runtime_error("invalid info request");
    uint64_t runtime = 0;
    {
      std::lock_guard<std::mutex> lock(ctx.mutex);
      runtime = ctx.protocol.runtime_id();
    }
    const auto bytes = pm::EncodeInfo(runtime);
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
  } catch (const std::exception &e) {
    std::cerr << "ReplyInfo: " << e.what() << '\n';
    if (!reply_consumed &&
        !Checked(aorta_service_request_reply_error(request, e.what()),
                 "aorta_service_request_reply_error(get_robot_info)"))
      ctx.endpoint_failed = true;
    std::lock_guard<std::mutex> lock(ctx.mutex);
    ++ctx.info_errors;
  }
}
struct EEContext {
  Context* shared = nullptr;
  uint8_t side = 0;
};
void ReceiveEE(const aorta_bytes_view_t* view, const aorta_schema_hash_view_t*,
               const aorta_msg_context_t*, void* opaque) noexcept {
  auto& endpoint = *static_cast<EEContext*>(opaque);
  auto& ctx = *endpoint.shared;
  try {
    const auto command = pm::DecodeEECommand(Copy(view));
    std::lock_guard<std::mutex> lock(ctx.mutex);
    if (command.side == endpoint.side && ctx.protocol.AcceptEE(command, Mono(), Wall()/1000))
      ++ctx.ee_accepted;
    else ++ctx.ee_rejected;
  } catch (const std::exception& error) {
    std::lock_guard<std::mutex> lock(ctx.mutex);
    ++ctx.parse_errors;
    std::cerr << "ReceiveEE: " << error.what() << '\n';
  } catch (...) {
    std::lock_guard<std::mutex> lock(ctx.mutex);
    ++ctx.parse_errors;
  }
}
void ReplyTouch(const aorta_bytes_view_t* view, aorta_service_request_t request,
                void* opaque) noexcept {
  auto& endpoint = *static_cast<EEContext*>(opaque);
  auto& ctx = *endpoint.shared;
  bool consumed = false;
  try {
    const auto value = pm::DecodeTouchRequest(Copy(view));
    pm::TouchReply reply;
    {
      std::lock_guard<std::mutex> lock(ctx.mutex);
      if (value.side == endpoint.side) reply = ctx.protocol.SetTouch(value, Mono());
      else {reply.runtime=value.runtime;reply.side=value.side;reply.reason=3;}
      ++ctx.touch_requests;
    }
    // The same Protocol snapshot is updated before responding. A transport
    // failure does not pretend the already-confirmed synthetic switch undid.
    const auto bytes = pm::EncodeTouchReply(reply);
    aorta_owned_bytes_slice_t slice{bytes.data(),bytes.size(),nullptr,nullptr};
    aorta_owned_bytes_view_t owned{&slice,1,bytes.size()};
    consumed = true;
    if (!Checked(aorta_service_request_reply(request,&owned), "set_touch reply")) ctx.endpoint_failed=true;
  } catch (const std::exception& error) {
    if (!consumed && !Checked(aorta_service_request_reply_error(request,error.what()),"set_touch error")) ctx.endpoint_failed=true;
    std::lock_guard<std::mutex> lock(ctx.mutex);
    ++ctx.parse_errors;
  } catch (...) {
    if (!consumed && !Checked(aorta_service_request_reply_error(request,"unknown set_touch error"),"set_touch error")) ctx.endpoint_failed=true;
    ctx.endpoint_failed=true;
  }
}
struct Resources {
  aorta_node_t node = 0;
  aorta_service_t service = 0;
  std::vector<aorta_service_t> ee_services;
  std::vector<aorta_sub_t> subscribers;
  std::vector<aorta_pub_t> publishers;
  bool Close() {
    bool ok = true;
    for (auto sub : subscribers)
      ok = Checked(aorta_subscriber_free(sub), "aorta_subscriber_free") && ok;
    subscribers.clear();
    if (service)
      ok = Checked(aorta_service_free(service), "aorta_service_free") && ok;
    service = 0;
    for (auto handle : ee_services)
      ok = Checked(aorta_service_free(handle), "aorta_service_free(EE)") && ok;
    ee_services.clear();
    for (auto pub : publishers)
      ok = Checked(aorta_publisher_free(pub), "aorta_publisher_free") && ok;
    publishers.clear();
    if (node) ok = Checked(aorta_node_free(node), "aorta_node_free") && ok;
    node = 0;
    return ok;
  }
  ~Resources() { Close(); }
};
aorta_pub_t Publisher(Resources &r, const std::filesystem::path &schemas,
                      const std::string &key, const char *topic,
                      const char *root, aorta_qos_preset_t preset) {
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
struct EEStreams {
  uint8_t side = 0;
  std::vector<uint8_t> info_bytes, touch_bytes;
  std::array<aorta_pub_t,3> publishers{};
  std::array<uint64_t,3> periods{}, next{}, samples{};
};
std::vector<EEStreams> ConfigureEE(Context& context,const std::filesystem::path& directory) {
  std::vector<EEStreams> result;
  const auto path=directory/"ee-config.txt";
  if (!std::filesystem::exists(path)) return result;
  std::ifstream file(path);
  if (!file) throw std::runtime_error("cannot open EE config");
  std::string side;
  while (file >> side) {
    if (result.size()>=2 || (side!="left" && side!="right")) throw std::runtime_error("EE fixture side/count");
    EEStreams stream;stream.side=side=="left" ? 1 : 2;
    stream.info_bytes=Read(directory/("ee-"+side+"-info.bin"));
    auto info=pm::DecodeEEInfo(stream.info_bytes);
    if (info.side!=stream.side) throw std::runtime_error("EE fixture topic side mismatch");
    for (size_t role=0;role<3;++role) {
      double hz=0;
      if (!(file>>hz) || !std::isfinite(hz) || hz>2000 || hz<0 || (role<2 && hz==0))
        throw std::runtime_error("EE explicit fixture frequency");
      if (hz>0 && (!std::isfinite(1e9/hz) || 1e9/hz >= std::numeric_limits<uint64_t>::max()))
        throw std::runtime_error("EE fixture period overflow");
      stream.periods[role]=hz>0 ? static_cast<uint64_t>(1e9/hz) : 0;
    }
    if (info.touch_available != (stream.periods[2]!=0)) throw std::runtime_error("EE touch rate/capability mismatch");
    std::vector<float> initial(info.axes.size());
    for (auto& value:initial) if (!(file>>value)) throw std::runtime_error("EE initial position absent");
    size_t count=0;
    if (!(file>>count) || count>5) throw std::runtime_error("EE fixed K bound");
    for (size_t i=0;i<count;++i) {
      unsigned kind=0,where=0;
      if (!(file>>kind>>where) || kind>255 || where>255) throw std::runtime_error("EE K field");
      info.observation_sources.emplace_back(kind,where);
    }
    context.protocol.ConfigureEE(info,initial);
    if (info.touch_available) {
      stream.touch_bytes=Read(directory/("ee-"+side+"-touch.bin"));
      // Fail a malformed template before creating any Aorta endpoints.
      (void)pm::EncodeEETouch(stream.touch_bytes,context.protocol.SnapshotEE(stream.side,Mono()),1,Wall()/1000);
    }
    result.push_back(std::move(stream));
  }
  if (!file.eof()) throw std::runtime_error("truncated EE fixture config");
  return result;
}
void OpenEE(Resources& resources,const std::filesystem::path& schemas,
            EEStreams& stream,EEContext& context) {
  const auto prefix=std::string("/sdk/foot_humanoid/end_effector/")+(stream.side==1 ? "left" : "right");
  const std::array<const char*,3> roles{"state","info","touch"};
  const std::array<const char*,3> roots{"lowlevel.ee.EndEffectorState","lowlevel.ee.EndEffectorInfo","lowlevel.ee.EndEffectorTouch"};
  for (size_t role=0;role<3;++role) if (stream.periods[role]) {
    stream.publishers[role]=Publisher(resources,schemas,"end_effector_"+std::string(roles[role]),
        (prefix+"/"+roles[role]).c_str(),roots[role],role==1 ? AORTA_QOS_PRESET_STATE_UPDATE : AORTA_QOS_PRESET_SENSOR_DATA);
  }
  auto qos=aorta_qos_preset(AORTA_QOS_PRESET_REALTIME_CONTROL);
  aorta_sub_t sub=0;
  Check(aorta_subscriber_new(resources.node,(prefix+"/cmd").c_str(),&qos,nullptr,ReceiveEE,&context,&sub),"EE cmd subscriber");
  resources.subscribers.push_back(sub);
  const auto bfbs=Read(schemas/"end_effector_set_touch.bfbs");
  const auto hash=Read(schemas/"end_effector_set_touch.sha256");
  aorta_service_options_t options{};
  options.service_schema_hash_hex=hash.data();options.service_schema_hash_len=hash.size();
  options.service_schema_bfbs=bfbs.data();options.service_schema_bfbs_len=bfbs.size();
  const std::string request="aorta.services.humanoid_locomotion.EndEffectorSetTouchRequest";
  const std::string response="aorta.services.humanoid_locomotion.EndEffectorSetTouchResponse";
  options.request_root_type=request.c_str();options.request_root_type_len=request.size();
  options.response_root_type=response.c_str();options.response_root_type_len=response.size();
  aorta_service_t service=0;
  Check(aorta_service_new_with_options(resources.node,(prefix+"/set_touch").c_str(),&options,ReplyTouch,&context,&service),"typed set_touch service");
  resources.ee_services.push_back(service);
}
void Advance(uint64_t &next, uint64_t period, uint64_t now) {
  next += ((now - next) / period + 1) * period;
}
}  // namespace

int main(int argc, char **argv) {
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
        seconds > 600 || stream_ms < 1 || stream_ms > 60000 || prepare_ms < 1 ||
        prepare_ms > 60000 || reject < 0 || reject > 3)
      throw std::runtime_error("invalid bounded mock parameters");
    pm::aorta_contract::ValidateGroupLength(group);
    const char *effective = std::getenv("AORTA_GROUP");
    if (group.rfind(pm::aorta_contract::GROUP_PREFIX, 0) != 0 || !effective ||
        group != effective)
      throw std::runtime_error("use the isolated runner and its private group");
    if (aorta_abi_version() != AORTA_EXPECTED_ABI)
      throw std::runtime_error("Aorta ABI mismatch");
    Context context(Wall(), stream_ms * 1000000, prepare_ms * 1000000, reject);
    context.apply_period_ns = static_cast<uint64_t>(1e9 / apply_hz);
    std::ifstream initial(directory / "initial-q.txt");
    std::array<float, pm::kSlots> q{};
    for (auto &v : q)
      if (!(initial >> v) || !std::isfinite(v) || std::abs(v) > 10)
        throw std::runtime_error(
            "initial-q requires31 finite values in synthetic info range");
    std::string trailing;
    if (initial >> trailing) throw std::runtime_error("extra initial-q values");
    context.protocol.SetInitialQ(q);
    std::ifstream imu_file(directory / "initial-imu.txt");
    std::array<std::array<float, 4>, 2> imu_q{};
    for (auto &quat : imu_q) {
      float norm = 0;
      for (auto &v : quat) {
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
    auto ee_streams = ConfigureEE(context,directory);
    std::array<EEContext,2> ee_contexts{{{&context,1},{&context,2}}};
    const auto ee_started_us = Wall()/1000;
    // Declare resources after context: callbacks are drained before context
    // dies.
    Resources resources;
    const auto start = Mono();
    int endpoint_rc = 0;
    try {
      Check(aorta_node_new("production_topic_mock", group.c_str(),
                           &resources.node),
            "aorta_node_new(production_topic_mock)");
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
                resources.node, "/sdk/foot_humanoid/control/request",
                &request_qos, nullptr, ReceiveRequest, &context, &requests),
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
      Check(
          aorta_service_new_with_options(
              resources.node, "/sdk/foot_humanoid/get_robot_info", &options,
              ReplyInfo, &context, &resources.service),
          "aorta_service_new_with_options(/sdk/foot_humanoid/get_robot_info)");
      for (auto& stream:ee_streams) OpenEE(resources,schemas,stream,ee_contexts[stream.side-1]);
      std::signal(SIGINT, Stop);
      std::signal(SIGTERM, Stop);
      const auto finish = start + static_cast<uint64_t>(seconds * 1e9);
      uint64_t next_state = start, next_apply = start, next_status = start,
               state_id = 0;
      uint64_t status_publication = 0;
      const uint64_t state_period = static_cast<uint64_t>(1e9 / hz);
      const uint64_t apply_period = static_cast<uint64_t>(1e9 / apply_hz);
      while (!stopped && !context.endpoint_failed && Mono() < finish) {
        const auto now = Mono();
        if (now >= next_apply) {
          std::lock_guard<std::mutex> lock(context.mutex);
          const auto wall = Wall();
          const auto tick = context.pending_tick_ns;
          if (tick && wall >= tick &&
              context.protocol.Tick(Mono(), tick / 1000)) {
            ++context.applied;
            context.pending_tick_ns = 0;
          }
          context.ee_applied += context.protocol.TickEE(Mono(),wall/1000);
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
          Check(aorta_publisher_publish(status_pub, bytes.data(), bytes.size()),
                "aorta_publisher_publish(/sdk/foot_humanoid/control/status)");
          Advance(next_status, 20000000, now);
        }
        uint64_t next_ee=finish;
        for (auto& stream:ee_streams) {
          pm::EESnapshot snapshot;
          {std::lock_guard<std::mutex> lock(context.mutex);snapshot=context.protocol.SnapshotEE(stream.side,Mono());}
          for (size_t role=0;role<3;++role) {
            if (!stream.periods[role]) continue;
            if (now>=stream.next[role]) {
              if (role!=2 || snapshot.touch_enabled) {
                const auto us=Wall()/1000;
                const auto id=++stream.samples[role];
                const auto bytes=role==0 ? pm::EncodeEEState(snapshot,id,us) : role==1 ?
                    pm::EncodeEEInfo(stream.info_bytes,snapshot.runtime,ee_started_us,us) :
                    pm::EncodeEETouch(stream.touch_bytes,snapshot,id,us);
                Check(aorta_publisher_publish(stream.publishers[role],bytes.data(),bytes.size()),"synthetic EE publish");
              }
              stream.next[role]=now+stream.periods[role];
            }
            next_ee=std::min(next_ee,stream.next[role]);
          }
        }
        std::this_thread::sleep_until(
            Clock::time_point(std::chrono::nanoseconds(
                std::min(next_ee,std::min(next_state, std::min(next_apply, next_status))))));
      }
    } catch (const std::exception &e) {
      endpoint_rc = 2;
      std::cerr << e.what() << '\n';
    }
    // Drain callbacks and close every resource before deciding the verdict.
    if (!resources.Close() || context.endpoint_failed) endpoint_rc = 2;
    std::ofstream result(directory / "mock-result.json");
    result << "{\"synthetic\":true,\"accepted\":" << context.accepted
           << ",\"ee_accepted\":" << context.ee_accepted
           << ",\"ee_rejected\":" << context.ee_rejected
           << ",\"ee_applied\":" << context.ee_applied
           << ",\"touch_requests\":" << context.touch_requests
           << ",\"rejected\":" << context.rejected
           << ",\"applied\":" << context.applied
           << ",\"controls\":" << context.controls
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
    return endpoint_rc ? 2 : 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 2;
  }
}
