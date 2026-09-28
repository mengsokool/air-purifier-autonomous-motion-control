#pragma once

#include <stdint.h>

enum class RobotState : uint8_t {
  STARTUP,
  DRIVE,
  BRAKE,
  SCAN,
  TURN,
  ESCAPE_REVERSE,
  ESCAPE_SCAN,
  ESCAPE_TURN,
  LOW_BATTERY,
  FAULT
};

enum class FanLevel : uint8_t {
  LOW_SPEED,
  MEDIUM_SPEED,
  HIGH_SPEED,
  FAILSAFE
};

enum class TurnDirection : int8_t {
  LEFT = -1,
  NONE = 0,
  RIGHT = 1
};

enum class BatteryChargeState : uint8_t {
  DISCHARGING,
  CHARGER_DETECTED,
  CHARGING,
  ABSORPTION,
  VERIFY_FULL,
  FULL
};

struct GapSelection {
  bool found;
  uint8_t targetAngleDeg;
  uint8_t startIndex;
  uint8_t endIndex;
  uint32_t score;
};
