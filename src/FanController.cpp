#include "FanController.h"

#include "AirQualityPolicy.h"
#include "Config.h"

void FanController::begin() {
  pinMode(config::pin::FAN_PWM, OUTPUT);
  ledcReady_ = ledcSetup(config::fan::PWM_CHANNEL,
                          config::fan::PWM_FREQUENCY_HZ,
                          config::fan::PWM_RESOLUTION_BITS) > 0.0;
  if (ledcReady_) {
    ledcAttachPin(config::pin::FAN_PWM, config::fan::PWM_CHANNEL);
  }
  if (!ledcReady_) {
    Serial.println(F("Fan LEDC setup failed"));
    digitalWrite(config::pin::FAN_PWM, HIGH);
    return;
  }
  writePercent(0);
  setLevel(FanLevel::FAILSAFE);
}

void FanController::startStartupDustShake(const uint32_t nowMs) {
  if (!ledcReady_) {
    return;
  }
  startupDustShakeActive_ = true;
  startupFullThrottleStartedMs_ = nowMs;
  lastPwmUpdateMs_ = nowMs;
  setLevel(FanLevel::HIGH_SPEED);

  // The fan controller has its own soft-start and needs an immediate 100%
  // command to start reliably. Bypass the firmware slew only for this launch
  // burst; normal level changes still use the smooth PWM ramp below.
  currentPercent_ = config::fan::HIGH_PERCENT;
  writePercent(currentPercent_);
}

void FanController::update(const uint32_t nowMs,
                           const bool sensorReady,
                           const bool sensorHealthy,
                           const uint16_t pm25Average,
                           const uint16_t pm10Average,
                           const uint16_t pm25Latest,
                           const uint16_t pm10Latest) {
  if (chargingLocked_) {
    // Remove the purifier load while charging so INA219 current can taper with
    // the battery and FULL detection is not masked by fan consumption.
    startupDustShakeActive_ = false;
    targetPercent_ = 0;
  }

  if (startupDustShakeActive_ &&
      nowMs - startupFullThrottleStartedMs_ >=
          config::fan::STARTUP_FULL_THROTTLE_HOLD_MS) {
    startupDustShakeActive_ = false;
  }

  if (chargingLocked_) {
    targetPercent_ = 0;
  } else if (startupDustShakeActive_) {
    setLevel(FanLevel::HIGH_SPEED);
  } else if (!sensorReady) {
    // Once the launch burst is over, keep airflow quiet while PMS3003 warms
    // up. There is no valid dust reading yet, so LOW is the intended state.
    setLevel(FanLevel::LOW_SPEED);
  } else if (!sensorHealthy) {
    setLevel(FanLevel::FAILSAFE);
  } else {
    setLevel(AirQualityPolicy::nextFanLevel(
        level_, pm25Average, pm10Average, pm25Latest, pm10Latest));
  }

  // Rate-limit the ramp so it has the same physical duration regardless of
  // CPU load or how quickly loop() happens to run.
  if (!ledcReady_ ||
      nowMs - lastPwmUpdateMs_ < config::fan::PWM_UPDATE_MS) {
    return;
  }
  lastPwmUpdateMs_ = nowMs;

  // Smooth PWM slew ramping to avoid power supply voltage sag and inrush.
  const uint8_t slewStep = config::fan::SLEW_PERCENT_PER_UPDATE;
  if (currentPercent_ < targetPercent_) {
    if (targetPercent_ - currentPercent_ <= slewStep) {
      currentPercent_ = targetPercent_;
    } else {
      currentPercent_ = static_cast<uint8_t>(currentPercent_ + slewStep);
    }
    writePercent(currentPercent_);
  } else if (currentPercent_ > targetPercent_) {
    if (currentPercent_ - targetPercent_ <= slewStep) {
      currentPercent_ = targetPercent_;
    } else {
      currentPercent_ = static_cast<uint8_t>(currentPercent_ - slewStep);
    }
    writePercent(currentPercent_);
  }

}

void FanController::writePercent(const uint8_t percent) {
  const uint32_t duty =
      (static_cast<uint32_t>(percent) * config::fan::PWM_MAX_DUTY + 50U) /
      100U;
  ledcWrite(config::fan::PWM_CHANNEL, duty);
}

void FanController::setLevel(const FanLevel level) {
  level_ = level;
  switch (level_) {
    case FanLevel::LOW_SPEED:
      targetPercent_ = config::fan::LOW_PERCENT;
      break;
    case FanLevel::MEDIUM_SPEED:
      targetPercent_ = config::fan::MEDIUM_PERCENT;
      break;
    case FanLevel::HIGH_SPEED:
      targetPercent_ = config::fan::HIGH_PERCENT;
      break;
    case FanLevel::FAILSAFE:
      targetPercent_ = config::fan::FAILSAFE_PERCENT;
      break;
  }
}
