#pragma once

#include <math.h>
#include <stdint.h>

class MotionGeometry {
 public:
  static uint32_t linearMillimetresToTicks(const float distanceMm,
                                           const float wheelDiameterMm,
                                           const uint32_t ticksPerWheelRev) {
    if (distanceMm <= 0.0F || wheelDiameterMm <= 0.0F ||
        ticksPerWheelRev == 0) {
      return 0;
    }
    const float wheelCircumferenceMm =
        static_cast<float>(M_PI) * wheelDiameterMm;
    return static_cast<uint32_t>(
        lroundf(distanceMm / wheelCircumferenceMm *
                static_cast<float>(ticksPerWheelRev)));
  }

  static uint32_t inPlaceTurnDegreesToTicks(const float angleDeg,
                                            const float wheelDiameterMm,
                                            const float trackWidthMm,
                                            const uint32_t ticksPerWheelRev) {
    if (angleDeg <= 0.0F || wheelDiameterMm <= 0.0F ||
        trackWidthMm <= 0.0F || ticksPerWheelRev == 0) {
      return 0;
    }
    // Each wheel follows a circle with radius trackWidth/2. Pi cancels when
    // dividing the turn arc by wheel circumference.
    return static_cast<uint32_t>(
        lroundf((trackWidthMm * angleDeg) /
                (360.0F * wheelDiameterMm) *
                static_cast<float>(ticksPerWheelRev)));
  }
};
