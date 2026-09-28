#include "DifferentialSteering.h"

int32_t DifferentialSteering::pathOffsetPercent(
    const uint8_t targetAngleDeg, const uint8_t centreAngleDeg,
    const int8_t angleToTurnSign) {
  return (static_cast<int32_t>(targetAngleDeg) - centreAngleDeg) *
         angleToTurnSign;
}

int32_t DifferentialSteering::offsetPercent(
    const uint16_t leftClearanceCm, const uint16_t rightClearanceCm,
    const uint16_t wallTargetCm, const int32_t maximumPercent) {
  int32_t offset = static_cast<int32_t>(rightClearanceCm) - leftClearanceCm;
  if (leftClearanceCm < wallTargetCm) offset += wallTargetCm - leftClearanceCm;
  if (rightClearanceCm < wallTargetCm) offset -= wallTargetCm - rightClearanceCm;
  if (offset > maximumPercent) return maximumPercent;
  if (offset < -maximumPercent) return -maximumPercent;
  return offset;
}

DifferentialWheelTargets DifferentialSteering::wheelTargets(
    const int32_t cruiseTps, const int32_t steerOffsetTps,
    const int32_t minimumForwardTps) {
  int32_t leftTps = cruiseTps + steerOffsetTps;
  int32_t rightTps = cruiseTps - steerOffsetTps;
  if (leftTps < minimumForwardTps) leftTps = minimumForwardTps;
  if (rightTps < minimumForwardTps) rightTps = minimumForwardTps;
  return {leftTps, rightTps};
}
