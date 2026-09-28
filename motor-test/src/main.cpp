#include <Arduino.h>

namespace {
// These pins intentionally match the main robot firmware and the L298N wiring.
constexpr uint8_t LEFT_ENA = 4;
constexpr uint8_t LEFT_IN1 = 11;
constexpr uint8_t LEFT_IN2 = 12;
constexpr uint8_t RIGHT_ENB = 14;
constexpr uint8_t RIGHT_IN3 = 15;
constexpr uint8_t RIGHT_IN4 = 16;
// Verified on the assembled chassis: 'w' drives both wheels forward with
// both H-bridge channels non-inverted.
constexpr bool LEFT_INVERTED = false;
constexpr bool RIGHT_INVERTED = false;

// Begin gently. Increase only after verifying motor direction and wiring.
constexpr uint8_t TEST_PWM = 255;
constexpr uint32_t MOVE_MS = 2200;
constexpr uint32_t TURN_MS = 1600;
constexpr uint32_t STOP_MS = 1400;

enum class TestStep : uint8_t {
  WAIT_FOR_START,
  FORWARD,
  STOP_AFTER_FORWARD,
  TURN_LEFT,
  STOP_AFTER_LEFT,
  TURN_RIGHT,
  STOP_AFTER_RIGHT,
  REVERSE,
  STOP_AFTER_REVERSE,
  COMPLETE,
  MANUAL_FORWARD,
  MANUAL_REVERSE,
  MANUAL_LEFT,
  MANUAL_RIGHT
};

TestStep step = TestStep::WAIT_FOR_START;
uint32_t stepStartedMs = 0;
uint32_t lastPromptMs = 0;

HardwareSerial& console = Serial0;

void printStartPrompt() {
  console.println(F("Keys: w=forward s=reverse a=left d=right x=stop t=auto test"));
  console.println(F("Keep wheels lifted; ENA/ENB jumpers must be removed."));
}

void setMotor(const uint8_t enablePin, const uint8_t in1Pin,
              const uint8_t in2Pin, const int16_t command) {
  if (command > 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    analogWrite(enablePin, static_cast<uint8_t>(command));
  } else if (command < 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    analogWrite(enablePin, static_cast<uint8_t>(-command));
  } else {
    analogWrite(enablePin, 0);
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
  }
}

void stopMotors() {
  setMotor(LEFT_ENA, LEFT_IN1, LEFT_IN2, 0);
  setMotor(RIGHT_ENB, RIGHT_IN3, RIGHT_IN4, 0);
}

void setDrive(const int16_t left, const int16_t right) {
  setMotor(LEFT_ENA, LEFT_IN1, LEFT_IN2,
           LEFT_INVERTED ? -left : left);
  setMotor(RIGHT_ENB, RIGHT_IN3, RIGHT_IN4,
           RIGHT_INVERTED ? -right : right);
}

void enterStep(const TestStep next) {
  step = next;
  stepStartedMs = millis();
  switch (step) {
    case TestStep::FORWARD:
      console.println(F("TEST: FORWARD"));
      setDrive(TEST_PWM, TEST_PWM);
      break;
    case TestStep::TURN_LEFT:
      console.println(F("TEST: TURN LEFT (in place)"));
      setDrive(-TEST_PWM, TEST_PWM);
      break;
    case TestStep::TURN_RIGHT:
      console.println(F("TEST: TURN RIGHT (in place)"));
      setDrive(TEST_PWM, -TEST_PWM);
      break;
    case TestStep::REVERSE:
      console.println(F("TEST: REVERSE"));
      setDrive(-TEST_PWM, -TEST_PWM);
      break;
    case TestStep::STOP_AFTER_FORWARD:
    case TestStep::STOP_AFTER_LEFT:
    case TestStep::STOP_AFTER_RIGHT:
    case TestStep::STOP_AFTER_REVERSE:
      console.println(F("TEST: STOP"));
      stopMotors();
      break;
    case TestStep::COMPLETE:
      console.println(F("TEST COMPLETE. Use w/a/s/d, or t to run again."));
      stopMotors();
      lastPromptMs = millis();
      break;
    case TestStep::MANUAL_FORWARD:
      console.println(F("MANUAL: FORWARD (x to stop)"));
      setDrive(TEST_PWM, TEST_PWM);
      break;
    case TestStep::MANUAL_REVERSE:
      console.println(F("MANUAL: REVERSE (x to stop)"));
      setDrive(-TEST_PWM, -TEST_PWM);
      break;
    case TestStep::MANUAL_LEFT:
      console.println(F("MANUAL: TURN LEFT (x to stop)"));
      setDrive(-TEST_PWM, TEST_PWM);
      break;
    case TestStep::MANUAL_RIGHT:
      console.println(F("MANUAL: TURN RIGHT (x to stop)"));
      setDrive(TEST_PWM, -TEST_PWM);
      break;
    case TestStep::WAIT_FOR_START:
      stopMotors();
      break;
  }
}

bool elapsed(const uint32_t durationMs) {
  return millis() - stepStartedMs >= durationMs;
}
}  // namespace

void setup() {
  // Serial0 is the USB-to-UART bridge shown as /dev/cu.usbserial-* on the
  // DevKitC-1. It remains usable even when native USB CDC is enabled.
  console.begin(115200);
  pinMode(LEFT_ENA, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_ENB, OUTPUT);
  pinMode(RIGHT_IN3, OUTPUT);
  pinMode(RIGHT_IN4, OUTPUT);
  stopMotors();

  printStartPrompt();
}

void loop() {
  if ((step == TestStep::WAIT_FOR_START || step == TestStep::COMPLETE) &&
      millis() - lastPromptMs >= 2000) {
    lastPromptMs = millis();
    printStartPrompt();
  }

  if (console.available() > 0) {
    const char command = static_cast<char>(console.read());
    switch (command) {
      case 'w': case 'W': enterStep(TestStep::MANUAL_FORWARD); break;
      case 's': case 'S': enterStep(TestStep::MANUAL_REVERSE); break;
      case 'a': case 'A': enterStep(TestStep::MANUAL_LEFT); break;
      case 'd': case 'D': enterStep(TestStep::MANUAL_RIGHT); break;
      case 'x': case 'X': enterStep(TestStep::WAIT_FOR_START); break;
      case 't': case 'T': enterStep(TestStep::FORWARD); break;
    }
  }

  switch (step) {
    case TestStep::FORWARD:
      if (elapsed(MOVE_MS)) enterStep(TestStep::STOP_AFTER_FORWARD);
      break;
    case TestStep::STOP_AFTER_FORWARD:
      if (elapsed(STOP_MS)) enterStep(TestStep::TURN_LEFT);
      break;
    case TestStep::TURN_LEFT:
      if (elapsed(TURN_MS)) enterStep(TestStep::STOP_AFTER_LEFT);
      break;
    case TestStep::STOP_AFTER_LEFT:
      if (elapsed(STOP_MS)) enterStep(TestStep::TURN_RIGHT);
      break;
    case TestStep::TURN_RIGHT:
      if (elapsed(TURN_MS)) enterStep(TestStep::STOP_AFTER_RIGHT);
      break;
    case TestStep::STOP_AFTER_RIGHT:
      if (elapsed(STOP_MS)) enterStep(TestStep::REVERSE);
      break;
    case TestStep::REVERSE:
      if (elapsed(MOVE_MS)) enterStep(TestStep::STOP_AFTER_REVERSE);
      break;
    case TestStep::STOP_AFTER_REVERSE:
      if (elapsed(STOP_MS)) enterStep(TestStep::COMPLETE);
      break;
    case TestStep::WAIT_FOR_START:
    case TestStep::COMPLETE:
    case TestStep::MANUAL_FORWARD:
    case TestStep::MANUAL_REVERSE:
    case TestStep::MANUAL_LEFT:
    case TestStep::MANUAL_RIGHT:
      break;
  }
}
