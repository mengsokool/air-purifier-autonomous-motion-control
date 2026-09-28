#include "RobotDisplay.h"

#include <stdio.h>
#include <string.h>

#include <Wire.h>

#include "AirQualityPolicy.h"
#include "Config.h"

namespace {
constexpr uint8_t ICON_FACE = 0;
constexpr uint8_t ICON_AIR = 1;
constexpr uint8_t ICON_FAN = 2;
constexpr uint8_t ICON_ROBOT = 3;
constexpr uint8_t ICON_BATTERY = 4;

uint8_t faceIcon[8] = {
    0b00000, 0b01010, 0b00000, 0b00000,
    0b10001, 0b01110, 0b00000, 0b00000,
};
uint8_t airIcon[8] = {
    0b00100, 0b10101, 0b01110, 0b11111,
    0b01110, 0b10101, 0b00100, 0b00000,
};
uint8_t fanIcon[8] = {
    0b00100, 0b10101, 0b01110, 0b00100,
    0b01110, 0b10101, 0b00100, 0b00000,
};
uint8_t robotIcon[8] = {
    0b01110, 0b10101, 0b11111, 0b10001,
    0b11111, 0b01010, 0b10001, 0b00000,
};
uint8_t batteryIcon[8] = {
    0b01110, 0b10001, 0b10101, 0b10101,
    0b10101, 0b10001, 0b11111, 0b00000,
};

const char* fanName(const FanLevel level) {
  switch (level) {
    case FanLevel::LOW_SPEED: return "QUIET";
    case FanLevel::MEDIUM_SPEED: return "NORMAL";
    case FanLevel::HIGH_SPEED: return "STRONG";
    case FanLevel::FAILSAFE: return "STRONG";
  }
  return "CHECKING";
}

const char* airQualityName(const RobotDisplayData& data) {
  if (!data.pmsHealthy) return "WAIT";
  switch (AirQualityPolicy::classify(data.pm25, data.pm10)) {
    case AirQualityLevel::GOOD: return "GOOD";
    case AirQualityLevel::FAIR: return "FAIR";
    case AirQualityLevel::POOR: return "POOR";
  }
  return "WAIT";
}

const char* movementName(const RobotDisplayData& data) {
  switch (data.state) {
    case RobotState::STARTUP: return "GETTING READY";
    case RobotState::BRAKE: return "PAUSING";
    case RobotState::SCAN: return "CHECKING PATH";
    case RobotState::TURN:
      return data.steeringPercent < 0 ? "LEFT TURN" : "RIGHT TURN";
    case RobotState::ESCAPE_REVERSE: return "BACKING UP";
    case RobotState::ESCAPE_SCAN: return "FINDING A WAY";
    case RobotState::ESCAPE_TURN: return "TRYING NEW WAY";
    case RobotState::LOW_BATTERY: return "PAUSED";
    case RobotState::FAULT: return "NEEDS HELP";
    case RobotState::DRIVE:
      if (data.frontAvoidActive || data.sideAvoidActive ||
          data.corridorCorrectionSign != 0) {
        return "MOVING SAFELY";
      }
      return "MOVING";
  }
  return "CHECKING";
}
}  // namespace

void RobotDisplay::begin() {
  Wire.begin(config::pin::LCD_SDA, config::pin::LCD_SCL,
             config::i2c::FREQUENCY_HZ);
  ready_ = lcd_.begin(config::display::COLUMNS,
                      config::display::ROWS) == 0;
  if (!ready_) {
    Serial.println(F("LCD 20x4 not detected"));
    return;
  }
  lcd_.createChar(ICON_FACE, faceIcon);
  lcd_.createChar(ICON_AIR, airIcon);
  lcd_.createChar(ICON_FAN, fanIcon);
  lcd_.createChar(ICON_ROBOT, robotIcon);
  lcd_.createChar(ICON_BATTERY, batteryIcon);
  lcd_.backlight();
  backlightOn_ = true;
  lcd_.clear();
  writeLineWithIcon(0, "HELLO!", ICON_FACE);
  writeLine(1, "AIR BOT IS AWAKE");
  writeLine(2, "GETTING READY...");
  writeLine(3, "PLEASE WAIT");
}

void RobotDisplay::updateBacklight(const uint32_t nowMs, const bool blink) {
  if (!blink) {
    if (!backlightOn_) {
      lcd_.backlight();
      backlightOn_ = true;
    }
    lastBacklightToggleMs_ = nowMs;
    return;
  }

  if (nowMs - lastBacklightToggleMs_ <
      config::display::LOW_BATTERY_BLINK_MS) {
    return;
  }
  lastBacklightToggleMs_ = nowMs;
  backlightOn_ = !backlightOn_;
  if (backlightOn_) {
    lcd_.backlight();
  } else {
    lcd_.noBacklight();
  }
}

void RobotDisplay::writeLine(const uint8_t row, const char* text) {
  char line[config::display::COLUMNS + 1];
  memset(line, ' ', config::display::COLUMNS);
  line[config::display::COLUMNS] = '\0';
  const size_t length = strlen(text);
  const size_t copyLength = length < config::display::COLUMNS
                                ? length
                                : config::display::COLUMNS;
  memcpy(line, text, copyLength);
  lcd_.setCursor(0, row);
  lcd_.print(line);
}

void RobotDisplay::writeLineWithIcon(const uint8_t row, const char* text,
                                     const uint8_t icon) {
  writeLine(row, text);
  lcd_.setCursor(config::display::COLUMNS - 1U, row);
  lcd_.write(icon);
}

void RobotDisplay::showStartup(const RobotDisplayData& data) {
  char line[48];
  writeLineWithIcon(0, "HELLO!", ICON_FACE);
  if (data.warmupSeconds > 0) {
    snprintf(line, sizeof(line), "GETTING READY %3us", data.warmupSeconds);
  } else {
    snprintf(line, sizeof(line), "GETTING READY...");
  }
  writeLine(1, line);
  writeLine(2, data.radarReady ? "ROOM CHECK: DONE" : "CHECKING THE ROOM");
  if (data.batteryWarning) {
    writeLineWithIcon(3, "BATTERY LOW", ICON_BATTERY);
  } else {
    writeLine(3, "PLEASE WAIT...");
  }
}

void RobotDisplay::showSummary(const RobotDisplayData& data) {
  char line[48];

  if (data.pmsHealthy) {
    snprintf(line, sizeof(line), "%-4s 25:%3u 10:%3u",
             airQualityName(data), data.pm25 > 999U ? 999U : data.pm25,
             data.pm10 > 999U ? 999U : data.pm10);
  } else {
    snprintf(line, sizeof(line), "AIR CHECKING...");
  }
  writeLineWithIcon(0, line, ICON_AIR);

  snprintf(line, sizeof(line), "FAN %-8s", fanName(data.fanLevel));
  writeLineWithIcon(1, line, ICON_FAN);

  if (data.frontValid) {
    snprintf(line, sizeof(line), "%-13s %3ucm", movementName(data),
             data.frontCm > 999U ? 999U : data.frontCm);
  } else {
    snprintf(line, sizeof(line), "%-13s  ---", movementName(data));
  }
  writeLineWithIcon(2, line, ICON_ROBOT);

  if (!data.batteryValid) {
    snprintf(line, sizeof(line), "BATTERY CHECKING...");
  } else if (data.batteryPercentValid) {
    snprintf(line, sizeof(line), "BATTERY %3u%% %4.1fV",
             static_cast<unsigned>(data.batteryPercent),
             data.batteryVoltage);
  } else {
    snprintf(line, sizeof(line), "BATTERY %-3s %4.1fV",
             data.batteryWarning ? "LOW" : "OK", data.batteryVoltage);
  }
  writeLineWithIcon(3, line, ICON_BATTERY);
}

void RobotDisplay::showCharging(const RobotDisplayData& data) {
  char line[48];
  switch (data.chargeState) {
    case BatteryChargeState::CHARGER_DETECTED:
      writeLineWithIcon(0, "CHARGER CONNECTED", ICON_BATTERY);
      break;
    case BatteryChargeState::ABSORPTION:
      writeLineWithIcon(0, "ALMOST FULL...", ICON_BATTERY);
      break;
    case BatteryChargeState::VERIFY_FULL:
      writeLineWithIcon(0, "CHECKING FULL...", ICON_BATTERY);
      break;
    case BatteryChargeState::FULL:
      writeLineWithIcon(0, "CHARGE COMPLETE", ICON_BATTERY);
      break;
    case BatteryChargeState::DISCHARGING:
    case BatteryChargeState::CHARGING:
      writeLineWithIcon(0, "CHARGING...", ICON_BATTERY);
      break;
  }
  if (data.batteryValid) {
    if (data.batteryPercentValid) {
      snprintf(line, sizeof(line), "BATTERY %3u%% %4.1fV",
               static_cast<unsigned>(data.batteryPercent),
               data.batteryVoltage);
    } else {
      snprintf(line, sizeof(line), "BATTERY       %4.1fV", data.batteryVoltage);
    }
    writeLine(1, line);
  } else {
    writeLine(1, "CHECKING BATTERY");
  }
  if (data.chargeState == BatteryChargeState::FULL) {
    writeLineWithIcon(2, "READY TO USE", ICON_FACE);
    writeLine(3, "UNPLUG WHEN READY");
  } else if (data.chargeReadingValid) {
    snprintf(line, sizeof(line), "CURRENT       %4.2fA",
             data.chargeCurrentMa / 1000.0F);
    writeLine(2, line);
    writeLineWithIcon(3, "ROBOT IS RESTING", ICON_FACE);
  } else {
    writeLine(2, "CHECKING CURRENT");
    writeLineWithIcon(3, "ROBOT IS RESTING", ICON_FACE);
  }
}

void RobotDisplay::showCritical(const RobotDisplayData& data) {
  if (data.state == RobotState::LOW_BATTERY) {
    writeLineWithIcon(0, "BATTERY IS LOW", ICON_BATTERY);
    writeLine(1, "PLEASE PLUG IN");
    writeLineWithIcon(2, "ROBOT IS PAUSED", ICON_FACE);
    writeLine(3, "SAFE TO RECHARGE");
  } else {
    writeLineWithIcon(0, "I NEED SOME HELP", ICON_FACE);
    writeLine(1, "CHECK MY WHEELS");
    writeLine(2, "TURN OFF, THEN ON");
    writeLine(3, "ROBOT IS PAUSED");
  }
}

void RobotDisplay::update(const uint32_t nowMs,
                          const RobotDisplayData& data) {
  if (!ready_) return;

  // Blink only while running from a low battery. Charging restores a steady
  // backlight so the charge screen remains easy to read.
  updateBacklight(nowMs, data.batteryWarning && !data.adapterPresent);

  const bool critical = data.state == RobotState::LOW_BATTERY ||
                        data.state == RobotState::FAULT;
  const uint16_t refreshMs = critical
                                 ? config::display::STARTUP_REFRESH_MS
                                 : config::display::NORMAL_REFRESH_MS;
  if (lastRefreshMs_ != 0 && nowMs - lastRefreshMs_ < refreshMs) return;
  lastRefreshMs_ = nowMs;

  if (critical) {
    showCritical(data);
  } else if (data.adapterPresent) {
    showCharging(data);
  } else if (data.state == RobotState::STARTUP && !data.adapterPresent) {
    showStartup(data);
  } else {
    showSummary(data);
  }
}
