#include "StablePathPlanner.h"

void StablePathPlanner::reset(const uint8_t centreAngleDeg) {
  targetAngleDeg_ = centreAngleDeg;
  candidateAngleDeg_ = centreAngleDeg;
  candidateWins_ = 0;
  steeringPercent_ = 0;
}

void StablePathPlanner::observeTarget(
    const uint8_t rawTargetAngleDeg, const uint8_t centreAngleDeg,
    const bool corridorMode, const uint8_t confirmationWins,
    const uint8_t maximumStepDeg) {
  if (corridorMode) {
    targetAngleDeg_ = centreAngleDeg;
    candidateAngleDeg_ = centreAngleDeg;
    candidateWins_ = 0;
    return;
  }
  if (rawTargetAngleDeg != candidateAngleDeg_) {
    candidateAngleDeg_ = rawTargetAngleDeg;
    candidateWins_ = 1;
  } else if (candidateWins_ < confirmationWins) {
    ++candidateWins_;
  }
  if (candidateWins_ < confirmationWins) return;

  const int16_t error = static_cast<int16_t>(candidateAngleDeg_) -
                        static_cast<int16_t>(targetAngleDeg_);
  if (error > static_cast<int16_t>(maximumStepDeg)) {
    targetAngleDeg_ = static_cast<uint8_t>(targetAngleDeg_ + maximumStepDeg);
  } else if (error < -static_cast<int16_t>(maximumStepDeg)) {
    targetAngleDeg_ = static_cast<uint8_t>(targetAngleDeg_ - maximumStepDeg);
  } else {
    targetAngleDeg_ = candidateAngleDeg_;
  }
  candidateWins_ = 0;
}

int16_t StablePathPlanner::desiredSteeringPercent(
    const int32_t wallSteeringPercent, const uint8_t centreAngleDeg,
    const int8_t angleToTurnSign, const int16_t maximumPercent,
    const int16_t deadbandPercent) const {
  int32_t desired = wallSteeringPercent +
                    (static_cast<int32_t>(targetAngleDeg_) - centreAngleDeg) *
                        angleToTurnSign;
  if (desired > maximumPercent) desired = maximumPercent;
  if (desired < -maximumPercent) desired = -maximumPercent;
  if (desired >= -deadbandPercent && desired <= deadbandPercent) desired = 0;
  return static_cast<int16_t>(desired);
}

int16_t StablePathPlanner::slewSteering(
    const int16_t desiredPercent, const int16_t maximumStepPercent) {
  if (desiredPercent > steeringPercent_ + maximumStepPercent) {
    steeringPercent_ += maximumStepPercent;
  } else if (desiredPercent < steeringPercent_ - maximumStepPercent) {
    steeringPercent_ -= maximumStepPercent;
  } else {
    steeringPercent_ = desiredPercent;
  }
  return steeringPercent_;
}

void StablePathPlanner::forceSteering(const int16_t steeringPercent) {
  steeringPercent_ = steeringPercent;
}
