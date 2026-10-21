#pragma once

#include <cstddef>
#include <stdexcept>
#include <string>

namespace production_mock::aorta_contract {
// Aorta v2026.10.20 (309b8691), core/src/key_expr.rs::MAX_GROUP_LEN.
// This private Rust constant is not exported by the C ABI. Python reads this
// same declaration; do not give the two prechecks independent limits.
inline constexpr std::size_t MAX_GROUP_LEN = 32;
inline constexpr const char GROUP_PREFIX[] = "mock-sdkmain-";

inline void ValidateGroupLength(const std::string& group) {
  if (group.size() > MAX_GROUP_LEN)
    throw std::invalid_argument(
        "Aorta group exceeds max length " + std::to_string(MAX_GROUP_LEN) +
        " (utf-8 bytes): got " + std::to_string(group.size()));
}
}  // namespace production_mock::aorta_contract
