#pragma once

#include <Arduino.h>

#include "RobotTypes.h"

class BatteryChargeManager {
 public:
  void begin();
  void update(uint32_t nowMs, bool chargerPresent, bool batteryValid,
              float batteryVoltage,
              bool chargeValid, float chargeCurrentMa);

  BatteryChargeState state() const { return state_; }
  bool wasFullyCharged() const { return wasFullyCharged_; }
  bool historyValid() const { return historyValid_; }
  float sessionChargedMah() const { return sessionChargedMah_; }
  float lastFullVoltage() const { return lastFullVoltage_; }
  float lastFullCurrentMa() const { return lastFullCurrentMa_; }
  float lastFullChargedMah() const { return lastFullChargedMah_; }
  uint32_t lastFullDurationSeconds() const {
    return lastFullDurationSeconds_;
  }
  uint32_t fullConfirmationCount() const { return fullConfirmationCount_; }

 private:
  BatteryChargeState state_ = BatteryChargeState::DISCHARGING;
  bool wasFullyCharged_ = false;
  bool historyValid_ = false;
  uint32_t stateStartedMs_ = 0;
  uint32_t fullCandidateSinceMs_ = 0;
  uint32_t sessionStartedMs_ = 0;
  uint32_t lastIntegrationMs_ = 0;
  float sessionChargedMah_ = 0.0F;
  float lastFullVoltage_ = 0.0F;
  float lastFullCurrentMa_ = 0.0F;
  float lastFullChargedMah_ = 0.0F;
  uint32_t lastFullDurationSeconds_ = 0;
  uint32_t fullConfirmationCount_ = 0;

  void transitionTo(BatteryChargeState next, uint32_t nowMs);
  void startSession(uint32_t nowMs);
  void updateSession(uint32_t nowMs, float chargeCurrentMa);
  void saveFullState(bool full, float voltage, float currentMa = 0.0F,
                     uint32_t nowMs = 0);
};
