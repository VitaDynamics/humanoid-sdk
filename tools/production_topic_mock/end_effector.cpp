// Compile one real generated root per TU: --gen-all duplicates shared types.
#include "protocol.h"
#include <algorithm>
#include <limits>
#include <memory>
#include <stdexcept>

#if defined(PRODUCTION_MOCK_EE_INFO_CODEC)
#include "end_effector_info_generated.h"
#elif defined(PRODUCTION_MOCK_EE_CMD_CODEC)
#include "end_effector_cmd_generated.h"
#elif defined(PRODUCTION_MOCK_EE_STATE_CODEC)
#include "end_effector_state_generated.h"
#elif defined(PRODUCTION_MOCK_EE_TOUCH_CODEC)
#include "end_effector_touch_generated.h"
#elif defined(PRODUCTION_MOCK_EE_SERVICE_CODEC)
#include "end_effector_set_touch_generated.h"
#endif

namespace production_mock {
namespace {
template<class Table, class T> std::vector<uint8_t> Pack(const T& obj) {
  flatbuffers::FlatBufferBuilder builder;
  builder.Finish(Table::Pack(builder, &obj));
  return {builder.GetBufferPointer(), builder.GetBufferPointer()+builder.GetSize()};
}
#if defined(PRODUCTION_MOCK_EE_INFO_CODEC) || defined(PRODUCTION_MOCK_EE_STATE_CODEC) || defined(PRODUCTION_MOCK_EE_TOUCH_CODEC)
std::unique_ptr<aorta::sys::AortaHeaderT> Header(uint64_t id, uint64_t us) {
  auto header=std::make_unique<aorta::sys::AortaHeaderT>();
  header->sequence=id;
  header->publish_stamp_ns=us*1000;
  header->publisher_node="production_topic_mock_SYNTHETIC";
  return header;
}
#endif
#if defined(PRODUCTION_MOCK_EE_CMD_CODEC)
template<class T> std::vector<T> CopyVector(const flatbuffers::Vector<T>* value) {
  if (value && value->size()>64) throw std::invalid_argument("EE vector bound");
  return value ? std::vector<T>(value->begin(),value->end()) : std::vector<T>{};
}
#endif
}  // namespace

#if defined(PRODUCTION_MOCK_EE_INFO_CODEC)
EEInfo DecodeEEInfo(const std::vector<uint8_t>& bytes) {
  flatbuffers::Verifier verify(bytes.data(),bytes.size());
  if (!lowlevel::ee::VerifyEndEffectorInfoBuffer(verify)) throw std::invalid_argument("invalid EE Info");
  const auto* wire=lowlevel::ee::GetEndEffectorInfo(bytes.data());
  if (wire->protocol_version()!=1 || !wire->axis_names() || !wire->axis_names()->size() || wire->axis_names()->size()>64)
    throw std::invalid_argument("EE Info version/axis count");
  EEInfo out;
  out.side=wire->side(); out.device_model=wire->device_model(); out.layout_id=wire->layout_id();
  out.touch_available=wire->touch_available();
  const auto n=wire->axis_names()->size();
  auto required=[n](const auto* v) {if (!v || v->size()!=n) throw std::invalid_argument("EE capability shape");};
  required(wire->supported_modes()); required(wire->supported_command_fields());
  required(wire->supported_state_fields()); required(wire->valid_limits());
  required(wire->position_domain()); required(wire->torque_domain());
  auto number=[n](const auto* v,size_t i,bool supported) {
    if ((v ? v->size() : 0)!=(supported ? n : 0)) throw std::invalid_argument("EE limit column shape");
    return supported ? v->Get(i) : 0.f;
  };
  uint32_t all=0;
  for (size_t i=0;i<n;++i) all|=wire->supported_command_fields()->Get(i)|wire->supported_state_fields()->Get(i);
  for (size_t i=0;i<n;++i) {
    EEAxis a;
    a.modes=wire->supported_modes()->Get(i); a.command_fields=wire->supported_command_fields()->Get(i);
    a.state_fields=wire->supported_state_fields()->Get(i); a.valid_limits=wire->valid_limits()->Get(i);
    a.position_domain=wire->position_domain()->Get(i); a.torque_domain=wire->torque_domain()->Get(i);
    a.pos_min=number(wire->pos_min(),i,all&1); a.pos_max=number(wire->pos_max(),i,all&1);
    a.vel_min=number(wire->vel_min(),i,all&2); a.vel_max=number(wire->vel_max(),i,all&2);
    a.vel_limit_max=number(wire->vel_limit_max(),i,all&4);
    a.tau_min=number(wire->tau_min(),i,all&8); a.tau_max=number(wire->tau_max(),i,all&8);
    a.current_min=number(wire->current_min(),i,all&16); a.current_max=number(wire->current_max(),i,all&16);
    out.axes.push_back(a);
  }
  return out;
}
std::vector<uint8_t> EncodeEEInfo(const std::vector<uint8_t>& fixture,
    uint64_t runtime,uint64_t startup_us,uint64_t wall_us) {
  (void)DecodeEEInfo(fixture);
  std::unique_ptr<lowlevel::ee::EndEffectorInfoT> info(lowlevel::ee::GetEndEffectorInfo(fixture.data())->UnPack());
  info->descriptor_ref=std::make_unique<lowlevel::ee::DescriptorRef>(runtime,1);
  info->firmware="SYNTHETIC_NO_DEVICE"; info->serial="SYNTHETIC_NO_DEVICE"; info->identity_valid_mask=0;
  info->ts_descriptor_acquired_us=startup_us; info->ts_state_pub_us=wall_us; info->time_valid_mask=3;
  info->aorta_header=Header(0,wall_us);
  return Pack<lowlevel::ee::EndEffectorInfo>(*info);
}
#elif defined(PRODUCTION_MOCK_EE_CMD_CODEC)
EECommand DecodeEECommand(const std::vector<uint8_t>& bytes) {
  flatbuffers::Verifier verify(bytes.data(),bytes.size());
  if (!lowlevel::ee::VerifyEndEffectorCmdBuffer(verify)) throw std::invalid_argument("invalid EE Cmd");
  const auto* w=lowlevel::ee::GetEndEffectorCmd(bytes.data());
  if (!w->descriptor_ref() || !w->control_ref() || !w->key()) throw std::invalid_argument("missing EE identity");
  EECommand out;
  out.version=w->protocol_version(); out.layout_id=w->layout_id(); out.side=w->side();
  out.runtime=w->descriptor_ref()->robot_runtime_id(); out.epoch=w->descriptor_ref()->descriptor_epoch();
  const auto* c=w->control_ref();
  if (!c->client_id() || !c->lease_id() || c->client_id()->size()>256 || c->lease_id()->size()>256)
    throw std::invalid_argument("invalid EE control identity");
  out.client_id=c->client_id()->str(); out.lease_id=c->lease_id()->str();
  out.key={w->key()->session_id(),w->key()->sequence()};
  out.modes=CopyVector(w->mode()); out.fields=CopyVector(w->command_fields());
  out.columns={CopyVector(w->pos()),CopyVector(w->vel()),CopyVector(w->vel_limit()),CopyVector(w->tau_cmd()),CopyVector(w->current_cmd())};
  out.client_created_us=w->ts_client_created_us(); out.client_send_us=w->ts_client_send_us();
  out.sdk_time_valid_mask=w->sdk_time_valid_mask();
  if (w->input_observations()) {
    if (w->input_observations()->size()>5) throw std::invalid_argument("EE observation bound");
    for (const auto* o:*w->input_observations()) {
      const auto& ref=o->descriptor_ref();
      out.observations.push_back({ref.robot_runtime_id(),ref.descriptor_epoch(),o->sample_id(),o->source_time_us(),o->stream(),o->side(),o->validity()});
    }
  }
  return out;
}
#elif defined(PRODUCTION_MOCK_EE_STATE_CODEC)
std::vector<uint8_t> EncodeEEState(const EESnapshot& s,uint64_t id,uint64_t us) {
  lowlevel::ee::EndEffectorStateT out;
  out.protocol_version=1; out.descriptor_ref=std::make_unique<lowlevel::ee::DescriptorRef>(s.runtime,s.epoch);
  out.layout_id=s.info.layout_id; out.side=s.info.side; out.state_id=id;
  out.ts_state_real_us=out.ts_state_pub_us=out.ts_ingested_us=out.ts_assembled_us=us;
  out.time_valid_mask=27; // Host synthetic sample, not a hardware acquisition clock.
  out.sample_status=lowlevel::ee::SampleStatus_OK;
  out.pos=s.position;
  const auto n=s.info.axes.size();
  uint32_t all=0;
  for (const auto& a:s.info.axes) {all|=a.state_fields;out.valid_fields.push_back(a.state_fields&1);}
  const auto unknown=std::numeric_limits<float>::quiet_NaN();
  if (all&2) out.vel.assign(n,unknown);
  if (all&8) out.tau.assign(n,unknown);
  if (all&16) out.current.assign(n,unknown);
  if (all&32) out.temperature_c.assign(n,unknown);
  if (all&128) out.error_code.assign(n,0); // invalid, not "no fault measured".
  out.is_mock.assign(n,true);
  out.ready=true;out.output_allowed=s.output_allowed;out.inhibit_reason=s.inhibit_reason;
  out.touch_enabled=s.touch_enabled;out.status_valid_mask=15;
  if (s.received_valid) {out.received_key=std::make_unique<lowlevel::ee::CommandKey>(s.received.session,s.received.sequence);out.key_valid_mask|=1;}
  if (s.accepted_valid) {out.host_target_key=std::make_unique<lowlevel::ee::CommandKey>(s.accepted.session,s.accepted.sequence);out.key_valid_mask|=2;}
  for (size_t i=0;i<n;++i) {
    out.actual_output_key.emplace_back(s.output_keys[i].session,s.output_keys[i].sequence);
    out.actual_output_key_valid.push_back(s.output_key_valid[i]);
  }
  // No simulated CAN ACK, feedback-carried key or complete progress history.
  out.progress_complete=false;
  out.reject_count=s.reject_count;out.last_reject_code=s.reject_code;
  out.ingress_drop_count=s.overwritten;out.timeout_count=s.timeouts;
  out.diagnostic_valid_mask=60;
  out.raw_valid_fields.assign(n,0);
  out.aorta_header=Header(id,us);
  return Pack<lowlevel::ee::EndEffectorState>(out);
}
#elif defined(PRODUCTION_MOCK_EE_TOUCH_CODEC)
std::vector<uint8_t> EncodeEETouch(const std::vector<uint8_t>& bytes,
    const EESnapshot& s,uint64_t id,uint64_t us) {
  flatbuffers::Verifier verify(bytes.data(),bytes.size());
  if (!lowlevel::ee::VerifyEndEffectorTouchBuffer(verify)) throw std::invalid_argument("invalid synthetic Touch fixture");
  std::unique_ptr<lowlevel::ee::EndEffectorTouchT> out(lowlevel::ee::GetEndEffectorTouch(bytes.data())->UnPack());
  if (out->protocol_version!=1 || out->side!=s.info.side || out->layout_id!=s.info.layout_id)
    throw std::invalid_argument("synthetic Touch fixture identity");
  out->descriptor_ref=std::make_unique<lowlevel::ee::DescriptorRef>(s.runtime,s.epoch);
  out->touch_state_id=id;out->ts_state_pub_us=us;out->publish_time_valid=true;
  for (auto& stamp:out->region_stamps) {
    // Keep payload validity only. Re-emitting a synthetic sample cannot create
    // hardware times, a device sample sequence, or physical contact evidence.
    stamp=lowlevel::ee::RegionStamp(0,0,0,0,0,0,0,stamp.validity()&1);
  }
  out->aorta_header=Header(id,us);
  return Pack<lowlevel::ee::EndEffectorTouch>(*out);
}
#elif defined(PRODUCTION_MOCK_EE_SERVICE_CODEC)
TouchRequest DecodeTouchRequest(const std::vector<uint8_t>& bytes) {
  flatbuffers::Verifier verify(bytes.data(),bytes.size());
  if (!aorta::services::humanoid_locomotion::VerifyEndEffectorSetTouchRequestBuffer(verify))
    throw std::invalid_argument("invalid typed set_touch request");
  const auto* w=aorta::services::humanoid_locomotion::GetEndEffectorSetTouchRequest(bytes.data());
  if (!w->descriptor_ref() || !w->control_ref() || !w->control_ref()->client_id() || !w->control_ref()->lease_id())
    throw std::invalid_argument("set_touch identity missing");
  const auto* control=w->control_ref();
  if (control->client_id()->size()>256 || control->lease_id()->size()>256) throw std::invalid_argument("set_touch identity size");
  return {w->protocol_version(),w->layout_id(),w->descriptor_ref()->robot_runtime_id(),
          w->descriptor_ref()->descriptor_epoch(),w->side(),control->client_id()->str(),control->lease_id()->str(),w->enable()};
}
std::vector<uint8_t> EncodeTouchReply(const TouchReply& value) {
  aorta::services::humanoid_locomotion::EndEffectorSetTouchResponseT out;
  out.protocol_version=1;
  out.descriptor_ref=std::make_unique<lowlevel::ee::DescriptorRef>(value.runtime,value.epoch);
  out.side=value.side;out.reason=value.reason;
  out.status=value.success ? aorta::services::common::ServiceStatus_SUCCESS : aorta::services::common::ServiceStatus_INTERNAL_ERROR;
  out.enabled=value.enabled;out.enabled_valid=value.enabled_valid;
  return Pack<aorta::services::humanoid_locomotion::EndEffectorSetTouchResponse>(out);
}
#endif
}  // namespace production_mock
