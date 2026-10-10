// Test-only declaration subset of the cbindgen C API at Aorta
// 71008719f5958df7f09d522f1d585c6748956ce1 (ABI 8).
// No middleware implementation: link the pinned aorta-sdk wheel's shared
// library.
#pragma once
#include <cstddef>
#include <cstdint>

extern "C" {
enum aorta_priority_t {
  AORTA_PRIORITY_REALTIME,
  AORTA_PRIORITY_HIGH,
  AORTA_PRIORITY_NORMAL,
  AORTA_PRIORITY_LOW,
  AORTA_PRIORITY_BACKGROUND
};
enum aorta_reliability_t {
  AORTA_RELIABILITY_BEST_EFFORT,
  AORTA_RELIABILITY_RELIABLE
};
enum aorta_qos_preset_t {
  AORTA_QOS_PRESET_DEFAULT,
  AORTA_QOS_PRESET_REALTIME_CONTROL,
  AORTA_QOS_PRESET_SENSOR_DATA,
  AORTA_QOS_PRESET_STATE_UPDATE,
  AORTA_QOS_PRESET_BULK_TRANSFER
};
enum aorta_schema_attach_policy_t {
  AORTA_SCHEMA_ATTACH_INHERIT,
  AORTA_SCHEMA_ATTACH_ALWAYS,
  AORTA_SCHEMA_ATTACH_NEVER
};
struct aorta_bytes_slice_t {
  const uint8_t *data;
  uintptr_t len;
};
struct aorta_bytes_view_t {
  const aorta_bytes_slice_t *slices;
  uintptr_t slice_count;
  uintptr_t total_len;
};
struct aorta_string_view_t {
  const uint8_t *data;
  uintptr_t len;
};
struct aorta_msg_context_t {
  aorta_string_view_t topic;
  int64_t receive_stamp_ns;
  int32_t priority;
  aorta_string_view_t publisher_node;
  uint64_t sequence;
};
struct aorta_qos_t {
  aorta_priority_t priority;
  aorta_reliability_t reliability;
  uint8_t congestion_control;
  uint8_t express;
};
struct aorta_pub_options_t {
  aorta_schema_attach_policy_t schema_attach_policy;
  uint8_t congestion_control;
  const uint8_t *schema_bfbs;
  uintptr_t schema_bfbs_len;
  uintptr_t max_message_size;
};
struct aorta_schema_hash_view_t {
  const uint8_t *hex;
  uintptr_t len;
};
// Subscriber options are not passed; Aorta's actual default options are used.
struct aorta_sub_options_t;
using aorta_node_t = uint64_t;
using aorta_pub_t = uint64_t;
using aorta_sub_t = uint64_t;
using aorta_result_t = int32_t;
using aorta_on_msg_cb = void (*)(const aorta_bytes_view_t *,
                                 const aorta_schema_hash_view_t *,
                                 const aorta_msg_context_t *, void *);
uint32_t aorta_abi_version();
aorta_qos_t aorta_qos_preset(aorta_qos_preset_t);
aorta_result_t aorta_node_new(const char *, const char *, aorta_node_t *);
aorta_result_t aorta_node_free(aorta_node_t);
aorta_result_t aorta_publisher_new(aorta_node_t, const char *,
                                   const aorta_qos_t *,
                                   const aorta_pub_options_t *, aorta_pub_t *);
aorta_result_t aorta_publisher_publish(aorta_pub_t, const uint8_t *, uintptr_t);
aorta_result_t aorta_publisher_wait_for_matching(aorta_pub_t, uint64_t,
                                                 uint8_t *);
aorta_result_t aorta_subscriber_new(aorta_node_t, const char *,
                                    const aorta_qos_t *,
                                    const aorta_sub_options_t *,
                                    aorta_on_msg_cb, void *, aorta_sub_t *);
}
static_assert(sizeof(void *) == 8 && sizeof(aorta_qos_t) == 12);
static_assert(sizeof(aorta_pub_options_t) == 32);
static_assert(offsetof(aorta_pub_options_t, schema_bfbs) == 8);
static_assert(sizeof(aorta_bytes_view_t) == 24);
static_assert(sizeof(aorta_msg_context_t) == 56);
