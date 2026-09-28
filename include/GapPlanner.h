#pragma once

#include <stdint.h>

#include "RobotTypes.h"

class GapPlanner {
 public:
  static uint16_t applyDirectionalPenalty(uint16_t distanceCm,
                                          uint8_t angleDifferenceDeg,
                                          uint8_t penaltyRadiusDeg,
                                          uint16_t openDistanceCm,
                                          uint8_t centreRetainPercent,
                                          uint8_t edgeRetainPercent);
  static GapSelection selectBestCorridor(const uint16_t* distancesCm,
                                         const bool* valid,
                                         uint8_t pointCount,
                                         uint8_t minAngleDeg,
                                         uint8_t stepDeg,
                                         uint8_t centreAngleDeg,
                                         uint16_t openDistanceCm,
                                         uint8_t corridorPoints);
  static GapSelection select(const uint16_t* distancesCm,
                             const bool* valid,
                             uint8_t pointCount,
                             uint8_t minAngleDeg,
                             uint8_t stepDeg,
                             uint8_t centreAngleDeg,
                             uint16_t openDistanceCm,
                             uint8_t minimumGapPoints);
};
