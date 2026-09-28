#include "BatteryMonitor.h"

#include "Config.h"

void BatteryMonitor::begin() {
  pinMode(config::pin::BATTERY_SENSE, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(config::pin::BATTERY_SENSE, ADC_11db);
}

void BatteryMonitor::update(const uint32_t nowMs) {
  if (!config::battery::MONITOR_ENABLED ||
      nowMs - lastSampleMs_ < config::battery::SAMPLE_INTERVAL_MS) {
    return;
  }
  lastSampleMs_ = nowMs;
  adcSum_ += static_cast<uint32_t>(
      analogReadMilliVolts(config::pin::BATTERY_SENSE));
  ++sampleCount_;

  if (sampleCount_ < config::battery::FILTER_SAMPLES) {
    return;
  }

  const float averageMillivolts =
      static_cast<float>(adcSum_) / static_cast<float>(sampleCount_);
  const float dividerRatio =
      (config::battery::DIVIDER_R_TOP_OHM +
       config::battery::DIVIDER_R_BOTTOM_OHM) /
      config::battery::DIVIDER_R_BOTTOM_OHM;
  filteredVoltage_ =
      (averageMillivolts / 1000.0F) * dividerRatio *
      config::battery::CALIBRATION_MULTIPLIER;
  hasReading_ = true;
  adcSum_ = 0;
  sampleCount_ = 0;
}

void BatteryMonitor::updateProtection(
    const uint32_t nowMs, const bool voltageValid,
    const float authoritativeVoltage) {
  protectionVoltageValid_ = voltageValid;
  protectionVoltage_ = voltageValid ? authoritativeVoltage : 0.0F;
  if (!config::battery::PROTECTION_ENABLED) {
    lowSinceMs_ = 0;
    return;
  }

  if (!voltageValid || authoritativeVoltage < 1.0F) {
    lowSinceMs_ = 0;
    return;
  }

  if (authoritativeVoltage <= config::battery::CUTOFF_V) {
    if (lowSinceMs_ == 0) {
      lowSinceMs_ = nowMs;
    } else if (nowMs - lowSinceMs_ >= config::battery::LOW_CONFIRM_MS) {
      cutoffLatched_ = true;
    }
  } else {
    lowSinceMs_ = 0;
  }

}

bool BatteryMonitor::warning() const {
  return config::battery::MONITOR_ENABLED &&
         config::battery::PROTECTION_ENABLED &&
         protectionVoltageValid_ && protectionVoltage_ > 0.1F &&
         protectionVoltage_ <= config::battery::WARNING_V;
}
