#include <Arduino.h>
#include <Adafruit_INA219.h>
#include <ESP32Servo.h>
#include <Wire.h>

#include "AsyncUltrasonic.h"
#include "Config.h"
#include "Pms3003.h"
#include "QuadratureEncoders.h"

// Reuse the production sensor drivers so this test validates the same parser,
// sonar timing, and PCNT quadrature configuration as the main firmware.
#include "../../src/AsyncUltrasonic.cpp"
#include "../../src/Pms3003.cpp"
#include "../../src/QuadratureEncoders.cpp"

namespace {
// With USB CDC disabled, Arduino's Serial is UART0 (the USB-to-UART bridge).
HardwareSerial& console = Serial;
HardwareSerial pmsSerial(config::serial::PMS_UART_NUMBER);
Pms3003 pms(pmsSerial);
AsyncUltrasonic sonar;
QuadratureEncoders encoders;
Servo scannerServo;
TwoWire inaWire(1);
Adafruit_INA219 ina219(config::charge::I2C_ADDRESS);

bool inaReady = false;
uint8_t scannerAngle = 90;
int8_t scannerDirection = 1;
uint16_t sonarDistanceCm = 0;
bool sonarValid = false;
uint32_t lastServoMoveMs = 0;
uint32_t lastPingMs = 0;
uint32_t lastReportMs = 0;
EncoderCounts previousCounts{0, 0};

float readBatteryVoltage() {
  const uint32_t millivolts =
      static_cast<uint32_t>(analogReadMilliVolts(config::pin::BATTERY_SENSE));
  const float dividerRatio =
      (config::battery::DIVIDER_R_TOP_OHM +
       config::battery::DIVIDER_R_BOTTOM_OHM) /
      config::battery::DIVIDER_R_BOTTOM_OHM;
  return (static_cast<float>(millivolts) / 1000.0F) * dividerRatio *
         config::battery::CALIBRATION_MULTIPLIER;
}

void scanI2c() {
  const int sdaLevel = digitalRead(config::pin::LCD_SDA);
  const int sclLevel = digitalRead(config::pin::LCD_SCL);
  console.printf("I2C pins: SDA GPIO%u=%s, SCL GPIO%u=%s\n",
                 config::pin::LCD_SDA, sdaLevel ? "HIGH" : "LOW",
                 config::pin::LCD_SCL, sclLevel ? "HIGH" : "LOW");
  if (sdaLevel == LOW || sclLevel == LOW) {
    console.println(F("I2C FAULT: a bus wire is held LOW; scan skipped."));
    return;
  }

  console.print(F("LCD I2C devices:"));
  bool found = false;
  constexpr uint8_t expectedAddresses[] = {0x27, 0x3F};
  for (const uint8_t address : expectedAddresses) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      console.printf(" 0x%02X", address);
      found = true;
    }
  }
  if (!found) console.print(F(" none"));
  console.println();

  console.print(F("INA219 I2C1 devices:"));
  inaWire.beginTransmission(config::charge::I2C_ADDRESS);
  if (inaWire.endTransmission() == 0) {
    console.printf(" 0x%02X", config::charge::I2C_ADDRESS);
  } else {
    console.print(F(" none"));
  }
  console.println();
}

void updateSonar(const uint32_t nowMs) {
  sonar.update();
  uint16_t resultCm = 0;
  bool resultValid = false;
  if (sonar.takeResult(resultCm, resultValid)) {
    sonarDistanceCm = resultCm;
    sonarValid = resultValid;
  }

  if (nowMs - lastServoMoveMs >= 1000) {
    lastServoMoveMs = nowMs;
    scannerAngle = static_cast<uint8_t>(scannerAngle + scannerDirection * 15);
    if (scannerAngle >= 165) {
      scannerAngle = 165;
      scannerDirection = -1;
    } else if (scannerAngle <= 15) {
      scannerAngle = 15;
      scannerDirection = 1;
    }
    scannerServo.write(scannerAngle);
  }

  if (nowMs - lastServoMoveMs >= 250 && !sonar.busy() &&
      nowMs - lastPingMs >= 250) {
    if (sonar.request()) lastPingMs = nowMs;
  }
}

void report(const uint32_t nowMs) {
  if (nowMs - lastReportMs < 1000) return;
  lastReportMs = nowMs;
  const EncoderCounts counts = encoders.read();
  const int32_t leftDelta = counts.left - previousCounts.left;
  const int32_t rightDelta = counts.right - previousCounts.right;
  previousCounts = counts;

  console.println(F("--- SENSOR REPORT ---"));
  console.printf("PMS3003: frames=%lu healthy=%u warmup=%us PM1=%u PM2.5=%u PM10=%u\n",
                 static_cast<unsigned long>(pms.validFrames()),
                 pms.healthy(nowMs) ? 1U : 0U,
                 pms.warmedUp(nowMs) ? 0U :
                     static_cast<unsigned>((config::pms::WARMUP_MS - nowMs + 999U) / 1000U),
                 pms.pm1Average(), pms.pm25Average(), pms.pm10Average());
  console.printf("Sonar: angle=%u distance=%ucm valid=%u\n", scannerAngle,
                 sonarDistanceCm, sonarValid ? 1U : 0U);
  console.printf("Encoders: left=%ld (%+ld/s) right=%ld (%+ld/s)\n",
                 static_cast<long>(counts.left), static_cast<long>(leftDelta),
                 static_cast<long>(counts.right), static_cast<long>(rightDelta));
  console.printf("Battery module: %.2fV\n", readBatteryVoltage());
  const bool adapterPresent =
      digitalRead(config::pin::CHARGER_PRESENT) ==
      (config::charge::PRESENT_ACTIVE_HIGH ? HIGH : LOW);
  console.printf("Adapter detect: %s (GPIO%u)\n",
                 adapterPresent ? "CONNECTED" : "not connected",
                 config::pin::CHARGER_PRESENT);
  if (inaReady) {
    console.printf("INA219: bus=%.2fV current=%+.0fmA power=%.2fW\n",
                   ina219.getBusVoltage_V(), ina219.getCurrent_mA(),
                   ina219.getPower_mW() / 1000.0F);
  } else {
    console.println(F("INA219: NOT DETECTED (expect I2C 0x40)"));
  }
}
}  // namespace

void setup() {
  console.begin(115200);
  delay(500);  // Give the USB-to-UART monitor time to attach after reset.
  console.println();
  console.println(F("=== ESP32-S3 SENSOR TEST STARTING ==="));
  console.println(F("Console: UART0, 115200 baud"));

  console.println(F("Starting PMS3003 UART..."));
  pmsSerial.begin(config::pms::BAUD, SERIAL_8N1, config::pin::PMS_RX,
                  config::pin::PMS_TX);
  console.println(F("Starting I2C..."));
  Wire.begin(config::pin::LCD_SDA, config::pin::LCD_SCL,
             config::i2c::FREQUENCY_HZ);
  Wire.setTimeOut(20);
  inaWire.begin(config::pin::INA219_SDA, config::pin::INA219_SCL,
                config::i2c::FREQUENCY_HZ);
  inaWire.setTimeOut(20);
  console.println(F("Starting battery ADC..."));
  analogReadResolution(12);
  analogSetPinAttenuation(config::pin::BATTERY_SENSE, ADC_11db);
  pinMode(config::pin::CHARGER_PRESENT, INPUT);
  console.println(F("Starting scanner servo..."));
  scannerServo.attach(config::pin::SCANNER_SERVO);
  scannerServo.write(scannerAngle);
  console.println(F("Starting HC-SR04..."));
  sonar.begin();
  console.println(F("Starting wheel encoders..."));
  encoders.begin();
  console.println(F("Starting PMS3003 parser..."));
  pms.begin();
  console.println(F("Starting INA219..."));
  inaReady = ina219.begin(&inaWire);
  if (inaReady) ina219.setCalibration_32V_2A();

  console.println(F("Sensor test ready: motors remain disabled."));
  console.println(F("Turn each wheel by hand to verify encoder counts."));
  scanI2c();
}

void loop() {
  const uint32_t nowMs = millis();
  pms.update(nowMs);
  updateSonar(nowMs);
  report(nowMs);
}
