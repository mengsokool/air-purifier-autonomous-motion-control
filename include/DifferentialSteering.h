#pragma once

#include <stdint.h>

struct DifferentialWheelTargets {
  int32_t leftTps;
  int32_t rightTps;
};

class DifferentialSteering {
 public:
  static int32_t pathOffsetPercent(uint8_t targetAngleDeg,
                                   uint8_t centreAngleDeg,
                                   int8_t angleToTurnSign);
  static int32_t offsetPercent(uint16_t leftClearanceCm,
                               uint16_t rightClearanceCm,
                               uint16_t wallTargetCm,
                               int32_t maximumPercent);
  static DifferentialWheelTargets wheelTargets(int32_t cruiseTps,
                                                int32_t steerOffsetTps,
                                                int32_t minimumForwardTps);
};
