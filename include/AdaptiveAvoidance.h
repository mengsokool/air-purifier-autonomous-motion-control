#pragma once

#include <stdint.h>

class AdaptiveAvoidance {
 public:
  static int16_t steeringMagnitude(uint16_t distanceCm,
                                   uint16_t activationDistanceCm,
                                   uint16_t emergencyDistanceCm,
                                   uint8_t pathErrorDeg,
                                   int16_t minimumPercent,
                                   int16_t maximumPercent);
  static int32_t baseSpeed(int32_t normalSpeed, int32_t crawlSpeed,
                           int16_t steeringPercent,
                           int16_t adaptiveStartPercent,
                           int16_t adaptiveMaximumPercent);
  static int16_t aggressiveSteering(int16_t steeringPercent,
                                    int16_t innerBrakeBoostPercent,
                                    int16_t maximumMagnitudePercent = 100);
};
