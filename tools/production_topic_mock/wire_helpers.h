#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "flatbuffers/flatbuffers.h"

namespace production_mock {
namespace detail {

// Include no generated root here: each codec TU selects its own gen-all root.
template <class Table, class T>
std::vector<uint8_t> Pack(const T& value) {
  flatbuffers::FlatBufferBuilder builder;
  builder.Finish(Table::Pack(builder, &value));
  return {builder.GetBufferPointer(),
          builder.GetBufferPointer() + builder.GetSize()};
}

template <class Header>
std::unique_ptr<Header> MakeHeader(uint64_t sequence, uint64_t stamp_ns,
                                   const char* publisher_node) {
  auto header = std::make_unique<Header>();
  header->sequence = sequence;
  header->publish_stamp_ns = stamp_ns;
  header->publisher_node = publisher_node;
  return header;
}

}  // namespace detail
}  // namespace production_mock
