#include <Arduino.h>

#include "Config.h"
#include "RobotController.h"

namespace {
RobotController robot;
}

void setup() {
  robot.begin();
}

void loop() {
  robot.update();
}
