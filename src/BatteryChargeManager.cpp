#include "BatteryChargeManager.h"

#include <Preferences.h>

#include "Config.h"

namespace {
Preferences chargePreferences;
constexpr uint8_t CHARGE_HISTORY_VERSION = 2;

bool validStoredVoltage(const float voltage) {
  return isfinite(voltage) && voltage >= 10.0F && voltage <= 16.0F;
}

bool validStoredCurrent(const float currentMa) {
  return isfinite(currentMa) && currentMa >= -50.0F && currentMa <= 3500.0F;
}
}

void BatteryChargeManager::begin() {
  chargePreferences.begin("charge-state", false);
  const uint8_t storedVersion = chargePreferences.getUChar("version", 0);
  if (storedVersion != CHARGE_HISTORY_VERSION) {
    // Legacy data did not contain enough evidence to trust a remembered FULL.
    chargePreferences.putUChar("version", CHARGE_HISTORY_VERSION);
    chargePreferences.putBool("was-full", false);
    wasFullyCharged_ = false;
    Serial.println(F("Charge history initialized; legacy FULL memory cleared"));
    return;
  }

  wasFullyCharged_ = chargePreferences.getBool("was-full", false);
  lastFullVoltage_ = chargePreferences.getFloat("full-v", 0.0F);
  lastFullCurrentMa_ = chargePreferences.getFloat("full-ma", 0.0F);
  lastFullChargedMah_ = chargePreferences.getFloat("full-mah", 0.0F);
  lastFullDurationSeconds_ = chargePreferences.getUInt("full-sec", 0);
  fullConfirmationCount_ = chargePreferences.getUInt("full-count", 0);
  historyValid_ = validStoredVoltage(lastFullVoltage_) &&
                  validStoredCurrent(lastFullCurrentMa_) &&
                  isfinite(lastFullChargedMah_) &&
                  lastFullChargedMah_ >= 0.0F;
  if (!historyValid_) {
    wasFullyCharged_ = false;
    chargePreferences.putBool("was-full", false);
    Serial.println(F("Charge history invalid; remembered FULL cleared"));
    return;
  }
  Serial.printf(
      "Charge history: full=%u last=%.2fV %+.0fmA added=%.0fmAh "
      "duration=%lus count=%lu\n",
      wasFullyCharged_ ? 1U : 0U, lastFullVoltage_, lastFullCurrentMa_,
      lastFullChargedMah_,
      static_cast<unsigned long>(lastFullDurationSeconds_),
      static_cast<unsigned long>(fullConfirmationCount_));
}

void BatteryChargeManager::update(const uint32_t nowMs,
                                  const bool chargerPresent,
                                  const bool batteryValid,
                                  const float batteryVoltage,
                                  const bool chargeValid,
                                  const float chargeCurrentMa) {
  if (batteryValid && wasFullyCharged_ &&
      batteryVoltage < config::charge::RECHARGE_V) {
    saveFullState(false, batteryVoltage);
    transitionTo(BatteryChargeState::DISCHARGING, nowMs);
  }

  // INA219 current alone cannot prove that an adapter is plugged in. The
  // dedicated adapter detector is authoritative for charging mode.
  if (!chargerPresent) {
    fullCandidateSinceMs_ = 0;
    sessionStartedMs_ = 0;
    lastIntegrationMs_ = 0;
    sessionChargedMah_ = 0.0F;
    transitionTo(BatteryChargeState::DISCHARGING, nowMs);
    return;
  }

  if (state_ == BatteryChargeState::DISCHARGING) {
    startSession(nowMs);
    transitionTo(BatteryChargeState::CHARGER_DETECTED, nowMs);
  }

  // An absent/bad monitor must never produce FULL; retain a visible detect
  // state until both battery voltage and charge-current samples are usable.
  if (!batteryValid || !chargeValid) return;

  const float chargeCurrent = config::charge::CURRENT_SIGN_INVERTED
                                  ? -chargeCurrentMa : chargeCurrentMa;
  updateSession(nowMs, chargeCurrent);
  const bool fullCurrentValid =
      chargeCurrent >= config::charge::FULL_MIN_CURRENT_MA &&
      chargeCurrent <= config::charge::FULL_CURRENT_MA;

  switch (state_) {
    case BatteryChargeState::DISCHARGING:
      transitionTo(BatteryChargeState::CHARGER_DETECTED, nowMs);
      break;
    case BatteryChargeState::CHARGER_DETECTED:
      if (nowMs - stateStartedMs_ < config::charge::DETECT_CONFIRM_MS) break;
      if (wasFullyCharged_ && batteryVoltage >= config::charge::RECENT_FULL_MIN_V &&
          batteryVoltage >= config::charge::ABSORPTION_V &&
          fullCurrentValid) {
        transitionTo(BatteryChargeState::VERIFY_FULL, nowMs);
      } else if (batteryVoltage >= config::charge::ABSORPTION_V) {
        if (wasFullyCharged_) saveFullState(false, batteryVoltage);
        transitionTo(BatteryChargeState::ABSORPTION, nowMs);
      } else if (chargeCurrent >= config::charge::CHARGER_DETECT_CURRENT_MA) {
        if (wasFullyCharged_) saveFullState(false, batteryVoltage);
        transitionTo(BatteryChargeState::CHARGING, nowMs);
      }
      break;
    case BatteryChargeState::CHARGING:
      if (batteryVoltage >= config::charge::ABSORPTION_V) {
        transitionTo(BatteryChargeState::ABSORPTION, nowMs);
      }
      break;
    case BatteryChargeState::ABSORPTION:
      if (batteryVoltage < config::charge::ABSORPTION_V) {
        transitionTo(BatteryChargeState::CHARGING, nowMs);
      } else if (fullCurrentValid) {
        if (fullCandidateSinceMs_ == 0) fullCandidateSinceMs_ = nowMs;
        if (nowMs - fullCandidateSinceMs_ >= config::charge::FULL_CONFIRM_MS) {
          saveFullState(true, batteryVoltage, chargeCurrent, nowMs);
          transitionTo(BatteryChargeState::FULL, nowMs);
        }
      } else {
        fullCandidateSinceMs_ = 0;
      }
      break;
    case BatteryChargeState::VERIFY_FULL:
      if (batteryVoltage < config::charge::ABSORPTION_V ||
          !fullCurrentValid) {
        saveFullState(false, batteryVoltage);
        transitionTo(BatteryChargeState::CHARGING, nowMs);
      } else if (nowMs - stateStartedMs_ >= config::charge::QUICK_VERIFY_MS) {
        transitionTo(BatteryChargeState::FULL, nowMs);
      }
      break;
    case BatteryChargeState::FULL:
      if (!fullCurrentValid ||
          batteryVoltage < config::charge::ABSORPTION_V) {
        saveFullState(false, batteryVoltage);
        transitionTo(BatteryChargeState::CHARGING, nowMs);
      }
      break;
  }
}

void BatteryChargeManager::startSession(const uint32_t nowMs) {
  sessionStartedMs_ = nowMs;
  lastIntegrationMs_ = nowMs;
  sessionChargedMah_ = 0.0F;
}

void BatteryChargeManager::updateSession(const uint32_t nowMs,
                                         const float chargeCurrentMa) {
  if (sessionStartedMs_ == 0) startSession(nowMs);
  uint32_t elapsedMs = nowMs - lastIntegrationMs_;
  lastIntegrationMs_ = nowMs;
  if (elapsedMs > config::charge::MAX_INTEGRATION_STEP_MS) {
    elapsedMs = config::charge::MAX_INTEGRATION_STEP_MS;
  }
  if (chargeCurrentMa > 0.0F && isfinite(chargeCurrentMa)) {
    sessionChargedMah_ +=
        chargeCurrentMa * static_cast<float>(elapsedMs) / 3600000.0F;
  }
}

void BatteryChargeManager::transitionTo(const BatteryChargeState next,
                                        const uint32_t nowMs) {
  if (state_ == next) return;
  state_ = next;
  stateStartedMs_ = nowMs;
  fullCandidateSinceMs_ = 0;
}

void BatteryChargeManager::saveFullState(const bool full, const float voltage,
                                         const float currentMa,
                                         const uint32_t nowMs) {
  if (wasFullyCharged_ == full && !full) return;
  wasFullyCharged_ = full;
  chargePreferences.putBool("was-full", full);
  if (!full) return;

  lastFullVoltage_ = voltage;
  lastFullCurrentMa_ = currentMa;
  lastFullChargedMah_ = sessionChargedMah_;
  lastFullDurationSeconds_ = sessionStartedMs_ == 0
                                 ? 0
                                 : (nowMs - sessionStartedMs_) / 1000UL;
  ++fullConfirmationCount_;
  historyValid_ = true;
  // These writes occur only after a confirmed full cycle, never every sample.
  chargePreferences.putFloat("full-v", lastFullVoltage_);
  chargePreferences.putFloat("full-ma", lastFullCurrentMa_);
  chargePreferences.putFloat("full-mah", lastFullChargedMah_);
  chargePreferences.putUInt("full-sec", lastFullDurationSeconds_);
  chargePreferences.putUInt("full-count", fullConfirmationCount_);
}
