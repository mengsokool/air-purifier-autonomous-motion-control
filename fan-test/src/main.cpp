#include <Arduino.h>

namespace {
constexpr uint8_t FAN_PWM_PIN = 5;
constexpr uint8_t FAN_PWM_CHANNEL = 7;
constexpr uint32_t FAN_PWM_FREQUENCY_HZ = 500UL;
constexpr uint8_t FAN_PWM_RESOLUTION_BITS = 10;
constexpr uint16_t FAN_PWM_MAX_DUTY =
    (1U << FAN_PWM_RESOLUTION_BITS) - 1U;
constexpr uint8_t TEST_PERCENT = 25;
constexpr uint16_t TEST_DUTY =
    static_cast<uint16_t>(
        (static_cast<uint32_t>(TEST_PERCENT) * FAN_PWM_MAX_DUTY + 50U) /
        100U);
constexpr uint32_t REPORT_INTERVAL_MS = 1000UL;

uint32_t lastReportMs = 0;
}  // namespace

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(FAN_PWM_PIN, OUTPUT);
  const double actualFrequency =
      ledcSetup(FAN_PWM_CHANNEL, FAN_PWM_FREQUENCY_HZ,
                FAN_PWM_RESOLUTION_BITS);
  if (actualFrequency <= 0.0) {
    digitalWrite(FAN_PWM_PIN, LOW);
    Serial.println("ERROR: LEDC setup failed; GPIO5 forced LOW");
    return;
  }

  ledcAttachPin(FAN_PWM_PIN, FAN_PWM_CHANNEL);
  ledcWrite(FAN_PWM_CHANNEL, TEST_DUTY);

  Serial.println("=== FAN PWM TEST ===");
  Serial.printf("GPIO=%u frequency=%luHz resolution=%ubit\n",
                static_cast<unsigned>(FAN_PWM_PIN),
                static_cast<unsigned long>(FAN_PWM_FREQUENCY_HZ),
                static_cast<unsigned>(FAN_PWM_RESOLUTION_BITS));
  Serial.printf("command=%u%% duty=%u/%u\n",
                static_cast<unsigned>(TEST_PERCENT),
                static_cast<unsigned>(TEST_DUTY),
                static_cast<unsigned>(FAN_PWM_MAX_DUTY));
}
void loop() {
  const uint32_t nowMs = millis();
  if (nowMs - lastReportMs >= REPORT_INTERVAL_MS) {
    lastReportMs = nowMs;
    Serial.printf("fan command steady at %u%%, uptime=%lums\n",
                  static_cast<unsigned>(TEST_PERCENT),
                  static_cast<unsigned long>(nowMs));
  }
}
