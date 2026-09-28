#pragma once

#include <Arduino.h>
#include <Preferences.h>

#include "RobotTypes.h"

class BatterySocEstimator {
 public:
  void begin();
  void update(uint32_t nowMs, bool batteryValid, float batteryVoltage,
              bool chargerPresent, BatteryChargeState chargeState,
              float sessionChargedMah, bool cutoff);

  bool valid() const { return valid_; }
  uint8_t percent() const;
  float precisePercent() const { return percent_; }

 private:
  Preferences preferences_;
  bool valid_ = false;
  bool storedValid_ = false;
  bool previousChargerPresent_ = false;
  float percent_ = 0.0F;
  float storedPercent_ = 0.0F;
  float chargeStartPercent_ = 0.0F;
  float lastPersistedPercent_ = 0.0F;
  uint32_t lastUpdateMs_ = 0;
  uint32_t lastPersistMs_ = 0;

  void initializeFromVoltage(float voltage, bool chargerPresent);
  void persistIfNeeded(uint32_t nowMs, bool force);
  static float clampPercent(float percent);
};
