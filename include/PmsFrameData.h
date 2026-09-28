#pragma once

#include <stdint.h>

namespace pms_frame {

constexpr uint8_t ATMOSPHERIC_PM1_OFFSET = 10;
constexpr uint8_t ATMOSPHERIC_PM25_OFFSET = 12;
constexpr uint8_t ATMOSPHERIC_PM10_OFFSET = 14;

inline uint16_t readUint16(const uint8_t* const frame,
                           const uint8_t offset) {
  return static_cast<uint16_t>(
      (static_cast<uint16_t>(frame[offset]) << 8U) |
      frame[offset + 1U]);
}

}  // namespace pms_frame
