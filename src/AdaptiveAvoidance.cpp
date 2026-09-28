#include "AdaptiveAvoidance.h"

int16_t AdaptiveAvoidance::steeringMagnitude(
    const uint16_t distanceCm, const uint16_t activationDistanceCm,
    const uint16_t emergencyDistanceCm, const uint8_t pathErrorDeg,
    const int16_t minimumPercent, const int16_t maximumPercent) {
  int32_t magnitude = minimumPercent;
  if (distanceCm <= emergencyDistanceCm) {
    magnitude = maximumPercent;
  } else if (distanceCm < activationDistanceCm &&
             activationDistanceCm > emergencyDistanceCm) {
    const int32_t dangerRange = activationDistanceCm - emergencyDistanceCm;
    const int32_t danger = activationDistanceCm - distanceCm;
    magnitude += (static_cast<int32_t>(maximumPercent - minimumPercent) *
                  danger) / dangerRange;
  }
  const uint8_t boundedError = pathErrorDeg > 75U ? 75U : pathErrorDeg;
  magnitude += (static_cast<int32_t>(boundedError) * 20L) / 75L;
  if (magnitude > maximumPercent) return maximumPercent;
  if (magnitude < minimumPercent) return minimumPercent;
  return static_cast<int16_t>(magnitude);
}

int32_t AdaptiveAvoidance::baseSpeed(
    const int32_t normalSpeed, const int32_t crawlSpeed,
    const int16_t steeringPercent, const int16_t adaptiveStartPercent,
    const int16_t adaptiveMaximumPercent) {
  const int32_t magnitude = steeringPercent < 0
                                ? -static_cast<int32_t>(steeringPercent)
                                : static_cast<int32_t>(steeringPercent);
  if (magnitude <= adaptiveStartPercent) return normalSpeed;
  if (magnitude >= adaptiveMaximumPercent) return crawlSpeed;
  return normalSpeed -
         ((normalSpeed - crawlSpeed) * (magnitude - adaptiveStartPercent)) /
             (adaptiveMaximumPercent - adaptiveStartPercent);
}

int16_t AdaptiveAvoidance::aggressiveSteering(
    const int16_t steeringPercent, const int16_t innerBrakeBoostPercent,
    const int16_t maximumMagnitudePercent) {
  if (steeringPercent == 0) return 0;
  const int8_t sign = steeringPercent < 0 ? -1 : 1;
  int32_t magnitude = steeringPercent < 0
                          ? -static_cast<int32_t>(steeringPercent)
                          : static_cast<int32_t>(steeringPercent);
  magnitude += innerBrakeBoostPercent;
  int32_t safeMaximum = maximumMagnitudePercent;
  if (safeMaximum < 0) safeMaximum = -safeMaximum;
  if (safeMaximum > 100) safeMaximum = 100;
  if (magnitude > safeMaximum) magnitude = safeMaximum;
  return static_cast<int16_t>(sign * magnitude);
}
