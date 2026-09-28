#include "BatterySocEstimator.h"

#include <math.h>

#include "BatterySocModel.h"
#include "Config.h"

namespace {
constexpr uint8_t SOC_STORAGE_VERSION = 1;
}

void BatterySocEstimator::begin() {
  preferences_.begin("battery-soc", false);
  const uint8_t version = preferences_.getUChar("version", 0);
  storedPercent_ = preferences_.getFloat("percent", -1.0F);
  storedValid_ = version == SOC_STORAGE_VERSION && isfinite(storedPercent_) &&
                 storedPercent_ >= 0.0F && storedPercent_ <= 100.0F;
  if (!storedValid_) {
    preferences_.putUChar("version", SOC_STORAGE_VERSION);
    storedPercent_ = 0.0F;
  }
}

void BatterySocEstimator::initializeFromVoltage(const float voltage,
                                                const bool chargerPresent) {
  const float voltagePercent = BatterySocModel::voltageToPercent(voltage);
  if (storedValid_) {
    // Never let surface-charge recovery raise a persisted estimate by more
    // than five points, but immediately accept evidence that the battery is
    // lower than memory says.
    percent_ = min(voltagePercent, storedPercent_ + 5.0F);
  } else {
    percent_ = voltagePercent;
  }
  percent_ = clampPercent(percent_);
  chargeStartPercent_ = percent_;
  previousChargerPresent_ = chargerPresent;
  lastPersistedPercent_ = storedValid_ ? storedPercent_ : percent_;
  valid_ = true;
}

void BatterySocEstimator::update(const uint32_t nowMs,
                                 const bool batteryValid,
                                 const float batteryVoltage,
                                 const bool chargerPresent,
                                 const BatteryChargeState chargeState,
                                 const float sessionChargedMah,
                                 const bool cutoff) {
  if (!valid_) {
    // With no saved anchor, voltage observed after a charger is already
    // connected is not an open-circuit SOC measurement. Keep percentage
    // unknown until a real FULL event anchors it at 100%.
    if (chargerPresent && !storedValid_ &&
        chargeState != BatteryChargeState::FULL) {
      return;
    }
    if (!batteryValid || !isfinite(batteryVoltage) || batteryVoltage < 1.0F) {
      return;
    }
    initializeFromVoltage(batteryVoltage, chargerPresent);
    lastUpdateMs_ = nowMs;
  }

  if (chargerPresent && !previousChargerPresent_) {
    chargeStartPercent_ = percent_;
  }
  previousChargerPresent_ = chargerPresent;

  uint32_t elapsedMs = nowMs - lastUpdateMs_;
  lastUpdateMs_ = nowMs;
  if (elapsedMs > 5000UL) elapsedMs = 5000UL;

  if (chargerPresent) {
    const float chargedGain = BatterySocModel::chargedMahToPercent(
        sessionChargedMah, config::battery::NOMINAL_CAPACITY_MAH,
        config::battery::CHARGE_COULOMB_EFFICIENCY);
    float target = chargeStartPercent_ + chargedGain;
    if (chargeState == BatteryChargeState::ABSORPTION) {
      target = max(target, 90.0F);
    } else if (chargeState == BatteryChargeState::VERIFY_FULL) {
      target = max(target, 98.0F);
    } else if (chargeState == BatteryChargeState::FULL) {
      target = 100.0F;
    }
    // Charge progress is monotonic; noisy sensor data must never move it back.
    percent_ = max(percent_, clampPercent(target));
  } else if (batteryValid && isfinite(batteryVoltage)) {
    const float voltageTarget = BatterySocModel::voltageToPercent(batteryVoltage);
    const float downPerMs = config::battery::SOC_MAX_DROP_PERCENT_PER_MIN / 60000.0F;
    const float upPerMs = config::battery::SOC_MAX_RECOVERY_PERCENT_PER_MIN / 60000.0F;
    if (voltageTarget < percent_) {
      percent_ = max(voltageTarget,
                     percent_ - downPerMs * static_cast<float>(elapsedMs));
    } else {
      percent_ = min(voltageTarget,
                     percent_ + upPerMs * static_cast<float>(elapsedMs));
    }
  }

  if (batteryValid && batteryVoltage <= config::battery::WARNING_V) {
    percent_ = min(percent_, 10.0F);
  }
  if (cutoff) percent_ = 0.0F;
  percent_ = clampPercent(percent_);
  persistIfNeeded(nowMs, cutoff || chargeState == BatteryChargeState::FULL);
}

uint8_t BatterySocEstimator::percent() const {
  if (!valid_) return 0;
  return static_cast<uint8_t>(percent_ + 0.5F);
}

void BatterySocEstimator::persistIfNeeded(const uint32_t nowMs,
                                          const bool force) {
  const float difference = fabs(percent_ - lastPersistedPercent_);
  // FULL and cutoff call this on every loop; write only on the actual change.
  if (storedValid_ && difference < 0.1F) return;
  if (!force &&
      (difference < config::battery::SOC_PERSIST_STEP_PERCENT ||
       nowMs - lastPersistMs_ < config::battery::SOC_PERSIST_MIN_INTERVAL_MS)) {
    return;
  }
  preferences_.putFloat("percent", percent_);
  lastPersistedPercent_ = percent_;
  lastPersistMs_ = nowMs;
  storedValid_ = true;
}

float BatterySocEstimator::clampPercent(const float percent) {
  if (percent < 0.0F) return 0.0F;
  if (percent > 100.0F) return 100.0F;
  return percent;
}
