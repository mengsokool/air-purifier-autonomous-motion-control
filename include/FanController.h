#pragma once

#include <Arduino.h>

#include "RobotTypes.h"

class FanController {
 public:
  void begin();
  void startStartupDustShake(uint32_t nowMs);
  void setChargingLock(bool locked) { chargingLocked_ = locked; }
  void update(uint32_t nowMs, bool sensorReady, bool sensorHealthy,
              uint16_t pm25Average, uint16_t pm10Average,
              uint16_t pm25Latest, uint16_t pm10Latest);
  bool startupDustShakeActive() const { return startupDustShakeActive_; }
  FanLevel level() const { return level_; }
  uint8_t percent() const { return currentPercent_; }

 private:
  FanLevel level_ = FanLevel::FAILSAFE;
  uint8_t targetPercent_ = 0;
  uint8_t currentPercent_ = 0;
  uint32_t lastPwmUpdateMs_ = 0;
  bool startupDustShakeActive_ = false;
  bool chargingLocked_ = false;
  uint32_t startupFullThrottleStartedMs_ = 0;
  bool ledcReady_ = false;

  void setLevel(FanLevel level);
  void writePercent(uint8_t percent);
};
