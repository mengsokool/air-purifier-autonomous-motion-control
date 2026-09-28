#pragma once

#include <stdint.h>

enum class RadarMotionResult : uint8_t {
  INSUFFICIENT,
  SCENE_CHANGED,
  STUCK
};

class RadarMotionVerifier {
 public:
  static RadarMotionResult verify(uint32_t reportedWheelTravelTicks,
                                  uint32_t requiredWheelTravelTicks,
                                  uint8_t comparableAnchors,
                                  uint8_t stableAnchors,
                                  uint8_t forwardComparableAnchors,
                                  uint8_t forwardStableAnchors,
                                  uint8_t minimumComparableAnchors,
                                  bool forwardEvidenceRequired);
};
