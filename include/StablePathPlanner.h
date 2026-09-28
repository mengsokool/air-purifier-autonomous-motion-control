#pragma once

#include <stdint.h>

class StablePathPlanner {
 public:
  void reset(uint8_t centreAngleDeg);
  void observeTarget(uint8_t rawTargetAngleDeg, uint8_t centreAngleDeg,
                     bool corridorMode, uint8_t confirmationWins,
                     uint8_t maximumStepDeg);
  int16_t desiredSteeringPercent(int32_t wallSteeringPercent,
                                 uint8_t centreAngleDeg,
                                 int8_t angleToTurnSign,
                                 int16_t maximumPercent,
                                 int16_t deadbandPercent) const;
  int16_t slewSteering(int16_t desiredPercent, int16_t maximumStepPercent);
  void forceSteering(int16_t steeringPercent);
  uint8_t targetAngleDeg() const { return targetAngleDeg_; }
  int16_t steeringPercent() const { return steeringPercent_; }

 private:
  uint8_t targetAngleDeg_ = 90;
  uint8_t candidateAngleDeg_ = 90;
  uint8_t candidateWins_ = 0;
  int16_t steeringPercent_ = 0;
};
