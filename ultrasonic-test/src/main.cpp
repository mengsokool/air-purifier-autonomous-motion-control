#include <Arduino.h>
#include <ESP32Servo.h>

#include "Config.h"

namespace {
constexpr uint8_t SAMPLE_COUNT = 5;
constexpr uint8_t TEST_ANGLES[] = {30, 60, 90, 120, 150};
constexpr uint32_t ECHO_TIMEOUT_US = 25000UL;
constexpr uint16_t SERVO_SETTLE_MS = 400;
constexpr uint16_t BETWEEN_PINGS_MS = 70;

Servo scannerServo;

void stopMotors() {
  pinMode(config::pin::MOTOR_LEFT_PWM, OUTPUT);
  pinMode(config::pin::MOTOR_LEFT_IN1, OUTPUT);
  pinMode(config::pin::MOTOR_LEFT_IN2, OUTPUT);
  pinMode(config::pin::MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(config::pin::MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(config::pin::MOTOR_RIGHT_IN2, OUTPUT);
  analogWrite(config::pin::MOTOR_LEFT_PWM, 0);
  analogWrite(config::pin::MOTOR_RIGHT_PWM, 0);
  digitalWrite(config::pin::MOTOR_LEFT_IN1, LOW);
  digitalWrite(config::pin::MOTOR_LEFT_IN2, LOW);
  digitalWrite(config::pin::MOTOR_RIGHT_IN1, LOW);
  digitalWrite(config::pin::MOTOR_RIGHT_IN2, LOW);
}

uint16_t readDistanceCm() {
  digitalWrite(config::pin::ULTRASONIC_TRIGGER, LOW);
  delayMicroseconds(3);
  digitalWrite(config::pin::ULTRASONIC_TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(config::pin::ULTRASONIC_TRIGGER, LOW);

  const uint32_t durationUs =
      pulseIn(config::pin::ULTRASONIC_ECHO, HIGH, ECHO_TIMEOUT_US);
  if (durationUs == 0) return 0;
  const uint32_t distanceCm = (durationUs + 29UL) / 58UL;
  if (distanceCm < config::sonar::MIN_VALID_CM ||
      distanceCm > config::sonar::MAX_VALID_CM) {
    return 0;
  }
  return static_cast<uint16_t>(distanceCm);
}

uint16_t readMedianCm(uint8_t& validSamples) {
  uint16_t samples[SAMPLE_COUNT]{};
  validSamples = 0;
  for (uint8_t i = 0; i < SAMPLE_COUNT; ++i) {
    const uint16_t distanceCm = readDistanceCm();
    if (distanceCm != 0) {
      samples[validSamples++] = distanceCm;
    }
    delay(BETWEEN_PINGS_MS);
  }

  if (validSamples == 0) return 0;
  for (uint8_t i = 1; i < validSamples; ++i) {
    const uint16_t value = samples[i];
    uint8_t j = i;
    while (j > 0 && samples[j - 1] > value) {
      samples[j] = samples[j - 1];
      --j;
    }
    samples[j] = value;
  }
  return samples[validSamples / 2U];
}
}  // namespace

void setup() {
  Serial.begin(115200);
  delay(500);
  stopMotors();

  pinMode(config::pin::ULTRASONIC_TRIGGER, OUTPUT);
  digitalWrite(config::pin::ULTRASONIC_TRIGGER, LOW);
  pinMode(config::pin::ULTRASONIC_ECHO, INPUT);

  scannerServo.setPeriodHertz(50);
  scannerServo.attach(config::pin::SCANNER_SERVO,
                      config::servo::MIN_PULSE_US,
                      config::servo::MAX_PULSE_US);
  scannerServo.write(config::servo::CENTER_DEG);

  Serial.println();
  Serial.println(F("=== HC-SR04 + SERVO QUICK TEST ==="));
  Serial.printf("TRIG=GPIO%u ECHO=GPIO%u SERVO=GPIO%u\n",
                config::pin::ULTRASONIC_TRIGGER,
                config::pin::ULTRASONIC_ECHO,
                config::pin::SCANNER_SERVO);
  Serial.println(F("Each result is the median of 5 pings. Motors are disabled."));
}

void loop() {
  Serial.println(F("--- NEW SWEEP ---"));
  for (const uint8_t angle : TEST_ANGLES) {
    scannerServo.write(angle);
    delay(SERVO_SETTLE_MS);

    uint8_t validSamples = 0;
    const uint16_t distanceCm = readMedianCm(validSamples);
    if (distanceCm == 0) {
      Serial.printf("angle=%3u  NO ECHO  valid=%u/%u\n",
                    angle, validSamples, SAMPLE_COUNT);
    } else {
      Serial.printf("angle=%3u  distance=%3ucm  valid=%u/%u\n",
                    angle, distanceCm, validSamples, SAMPLE_COUNT);
    }
  }
  scannerServo.write(config::servo::CENTER_DEG);
  Serial.println(F("Sweep complete; restarting in 2 seconds."));
  delay(2000);
}
