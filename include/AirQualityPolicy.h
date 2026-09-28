#pragma once

#include <stdint.h>

#include "RobotTypes.h"

enum class AirQualityLevel : uint8_t {
  GOOD,
  FAIR,
  POOR,
};

class AirQualityPolicy {
 public:
  // Returns the user-facing quality based on whichever pollutant is worse.
  static AirQualityLevel classify(uint16_t pm25, uint16_t pm10);

  // Applies independent PM2.5/PM10 hysteresis and returns the next fan level.
  // Latest samples are used only for rising transitions (fast attack). The
  // moving averages still govern falling transitions (stable release).
  static FanLevel nextFanLevel(FanLevel current, uint16_t pm25Average,
                               uint16_t pm10Average,
                               uint16_t pm25Latest = 0,
                               uint16_t pm10Latest = 0);
};
