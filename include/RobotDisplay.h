#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

#include "RobotTypes.h"

struct RobotDisplayData {
  RobotState state;
  uint16_t warmupSeconds;
  bool pmsHealthy;
  uint16_t pm25;
  uint16_t pm10;
  FanLevel fanLevel;
  uint8_t fanPercent;
  bool batteryValid;
  bool batteryWarning;
  float batteryVoltage;
  bool batteryPercentValid;
  uint8_t batteryPercent;
  bool chargeMonitorReady;
  bool chargeReadingValid;
  bool adapterPresent;
  BatteryChargeState chargeState;
  float chargeCurrentMa;
  bool frontValid;
  uint16_t frontCm;
  bool radarReady;
  bool frontAvoidActive;
  bool sideAvoidActive;
  int8_t corridorCorrectionSign;
  int16_t steeringPercent;
};

class RobotDisplay {
 public:
  void begin();
  void update(uint32_t nowMs, const RobotDisplayData& data);
  bool ready() const { return ready_; }

 private:
  hd44780_I2Cexp lcd_;
  bool ready_ = false;
  uint32_t lastRefreshMs_ = 0;
  uint32_t lastBacklightToggleMs_ = 0;
  bool backlightOn_ = true;

  void updateBacklight(uint32_t nowMs, bool blink);
  void writeLine(uint8_t row, const char* text);
  void writeLineWithIcon(uint8_t row, const char* text, uint8_t icon);
  void showStartup(const RobotDisplayData& data);
  void showSummary(const RobotDisplayData& data);
  void showCharging(const RobotDisplayData& data);
  void showCritical(const RobotDisplayData& data);
};
