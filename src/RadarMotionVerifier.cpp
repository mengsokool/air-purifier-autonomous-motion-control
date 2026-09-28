#include "RadarMotionVerifier.h"

RadarMotionResult RadarMotionVerifier::verify(
    const uint32_t reportedWheelTravelTicks,
    const uint32_t requiredWheelTravelTicks,
    const uint8_t comparableAnchors, const uint8_t stableAnchors,
    const uint8_t forwardComparableAnchors,
    const uint8_t forwardStableAnchors,
    const uint8_t minimumComparableAnchors,
    const bool forwardEvidenceRequired) {
  if (reportedWheelTravelTicks < requiredWheelTravelTicks ||
      comparableAnchors < minimumComparableAnchors) {
    return RadarMotionResult::INSUFFICIENT;
  }
  if (forwardEvidenceRequired &&
      forwardComparableAnchors == 0) {
    return RadarMotionResult::INSUFFICIENT;
  }
  // One genuinely changing ray is enough to prove chassis motion. Requiring
  // every comparable real echo to remain stable avoids declaring a moving
  // robot stuck merely because it travels parallel to corridor walls.
  // forwardStableAnchors is intentionally retained in the interface for
  // telemetry and diagnostics. A changed forward ray is positive motion
  // evidence, not a reason to classify the result as insufficient.
  (void)forwardStableAnchors;
  return stableAnchors == comparableAnchors
             ? RadarMotionResult::STUCK
             : RadarMotionResult::SCENE_CHANGED;
}
