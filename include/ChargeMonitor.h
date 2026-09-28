#pragma once

#include <Arduino.h>
#include <Adafruit_INA219.h>
#include <Wire.h>

#include "Config.h"

class ChargeMonitor {
 public:
  void begin();
  void update(uint32_t nowMs);

  bool ready() const { return ready_; }
  bool hasReading() const { return hasReading_; }
  float busVoltage() const { return busVoltageV_; }
  float currentMilliamp() const { return currentMa_; }
  float powerMilliwatt() const { return powerMw_; }

 private:
  TwoWire wire_{1};
  Adafruit_INA219 ina219_{config::charge::I2C_ADDRESS};
  bool ready_ = false;
  bool hasReading_ = false;
  uint32_t lastSampleMs_ = 0;
  float busVoltageV_ = 0.0F;
  float currentMa_ = 0.0F;
  float powerMw_ = 0.0F;
};
