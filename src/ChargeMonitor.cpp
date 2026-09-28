#include "ChargeMonitor.h"

#include "Config.h"

void ChargeMonitor::begin() {
  if (!wire_.begin(config::pin::INA219_SDA, config::pin::INA219_SCL,
                   config::i2c::FREQUENCY_HZ)) {
    Serial.println(F("INA219 Wire1 setup failed"));
    return;
  }
  ready_ = ina219_.begin(&wire_);
  if (!ready_) {
    Serial.println(F("INA219 charger monitor not detected"));
    return;
  }
  ina219_.setCalibration_32V_2A();
}

void ChargeMonitor::update(const uint32_t nowMs) {
  if (!ready_ ||
      nowMs - lastSampleMs_ < config::charge::SAMPLE_INTERVAL_MS) {
    return;
  }
  lastSampleMs_ = nowMs;
  busVoltageV_ = ina219_.getBusVoltage_V();
  currentMa_ = ina219_.getCurrent_mA();
  powerMw_ = ina219_.getPower_mW();
  hasReading_ = true;
}
