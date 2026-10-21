// Deliberate FFI failures against the real callback bodies, with no Node or IPC.
#define main ProductionMockEntryForCallbackChecks
#include "mock.cpp"
#undef main

#include <cassert>

namespace {
int publish_result = 0, reply_result = 0, free_result = 0;
int error_replies = 0;
void RequestBytes(Context& context, const std::vector<uint8_t>& bytes) {
  aorta_bytes_slice_t slice{bytes.data(), bytes.size()};
  aorta_bytes_view_t view{&slice, 1, bytes.size()};
  ReceiveRequest(&view, nullptr, nullptr, &context);
}
void InfoBytes(Context& context, const std::vector<uint8_t>& bytes) {
  aorta_bytes_slice_t slice{bytes.data(), bytes.size()};
  aorta_bytes_view_t view{&slice, 1, bytes.size()};
  ReplyInfo(&view, 123, &context);
}
}  // namespace

extern "C" {
const char* aorta_last_error() { return "injected callback FFI failure"; }
aorta_result_t aorta_publisher_publish(aorta_pub_t, const uint8_t*, uintptr_t) {
  return publish_result;
}
aorta_result_t aorta_service_request_reply(aorta_service_request_t,
                                           const aorta_owned_bytes_view_t*) {
  return reply_result;
}
aorta_result_t aorta_service_request_reply_error(aorta_service_request_t,
                                                 const char*) {
  ++error_replies;
  return 0;
}
aorta_result_t aorta_subscriber_free(aorta_sub_t) { return free_result; }
aorta_result_t aorta_service_free(aorta_service_t) { return free_result; }
aorta_result_t aorta_publisher_free(aorta_pub_t) { return free_result; }
aorta_result_t aorta_node_free(aorta_node_t) { return free_result; }
}

int main(int argc, char** argv) {
  assert(argc == 5);
  Context context(10, 5000000000, 5000000000, 0);
  for (int i : {2, 3}) {
    RequestBytes(context, Read(argv[i]));
    assert(context.controls.back().ack.error == 1002);
    assert(context.controls.back().ack_sent);
    assert(context.protocol.status().client_id.empty());
    assert(context.parse_errors == 0);
  }
  publish_result = -1;
  RequestBytes(context, Read(argv[1]));
  assert(context.endpoint_failed && !context.controls.back().ack_sent);
  assert(context.protocol.status().phase == 0);
  assert(context.protocol.status().lease_id.empty());
  publish_result = 0;
  RequestBytes(context, Read(argv[1]));
  assert(context.protocol.status().phase == 2);
  assert(context.protocol.status().lease_id == "mock-lease-11");

  reply_result = -1;
  InfoBytes(context, Read(argv[4]));
  assert(context.endpoint_failed && context.info_errors == 1);
  assert(error_replies == 0);  // A consumed reply handle is never retried.
  reply_result = 0;
  InfoBytes(context, {0, 0, 0, 0});
  assert(context.info_errors == 2 && error_replies == 1);

  const std::vector<uint8_t> bad{0, 0, 0, 0};
  aorta_bytes_slice_t slice{bad.data(), bad.size()};
  aorta_bytes_view_t view{&slice, 1, bad.size()};
  ReceiveCommand(&view, nullptr, nullptr, &context);
  assert(context.parse_errors == 1);

  Resources resources;
  resources.node = resources.service = 1;
  resources.publishers.push_back(2);
  resources.subscribers.push_back(3);
  free_result = -1;
  assert(!resources.Close());
  assert(resources.Close());  // Every handle was consumed exactly once.
}
