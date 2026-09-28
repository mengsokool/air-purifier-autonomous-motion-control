#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

namespace {
constexpr uint8_t I2C_SDA_PIN = 8;
constexpr uint8_t I2C_SCL_PIN = 9;
constexpr uint32_t I2C_FREQUENCY_HZ = 100000UL;
constexpr uint8_t TEST_INA219_ADDRESS = 0x40;
constexpr uint32_t SAMPLE_INTERVAL_MS = 1000UL;
constexpr uint32_t RETRY_INTERVAL_MS = 2000UL;

TwoWire inaWire(1);
Adafruit_INA219 ina219(TEST_INA219_ADDRESS);
bool inaReady = false;
uint32_t lastSampleMs = 0;
uint32_t lastRetryMs = 0;

bool addressResponds(const uint8_t address) {
  inaWire.beginTransmission(address);
  return inaWire.endTransmission() == 0;
}

void scanI2cBus() {
  uint8_t count = 0;
  Serial.println("I2C scan:");
  for (uint8_t address = 1; address < 127; ++address) {
    if (!addressResponds(address)) continue;
    Serial.printf("  found 0x%02X", static_cast<unsigned>(address));
    if (address >= 0x40 && address <= 0x4F) {
      Serial.print("  (possible INA219)");
    }
    Serial.println();
    ++count;
  }
  if (count == 0) {
    Serial.println("  no I2C devices found");
  }
}

bool startIna219() {
  if (!addressResponds(TEST_INA219_ADDRESS)) {
    Serial.println("ERROR: no response from INA219 at 0x40");
    Serial.println("Check 3.3V, GND, SDA=GPIO8, SCL=GPIO9 and A0/A1.");
    return false;
  }
  if (!ina219.begin(&inaWire)) {
    Serial.println("ERROR: INA219 begin failed");
    return false;
  }
  ina219.setCalibration_32V_2A();
  Serial.println("INA219 ready: address 0x40, calibration 32V/2A");
  return true;
}
}  // namespace

void setup() {
  Serial.begin(115200);
  delay(750);

  Serial.println();
  Serial.println("=== INA219 CHARGER MONITOR TEST ===");
  Serial.printf("I2C SDA=GPIO%u SCL=GPIO%u frequency=%luHz\n",
                static_cast<unsigned>(I2C_SDA_PIN),
                static_cast<unsigned>(I2C_SCL_PIN),
                static_cast<unsigned long>(I2C_FREQUENCY_HZ));

  inaWire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQUENCY_HZ);
  scanI2cBus();
  inaReady = startIna219();
  lastRetryMs = millis();
}

void loop() {
  const uint32_t nowMs = millis();

  if (!inaReady) {
    if (nowMs - lastRetryMs >= RETRY_INTERVAL_MS) {
      lastRetryMs = nowMs;
      Serial.println("Retrying INA219...");
      scanI2cBus();
      inaReady = startIna219();
    }
    return;
  }

  if (nowMs - lastSampleMs < SAMPLE_INTERVAL_MS) return;
  lastSampleMs = nowMs;

  if (!addressResponds(TEST_INA219_ADDRESS)) {
    Serial.println("ERROR: INA219 disconnected; waiting to reconnect");
    inaReady = false;
    lastRetryMs = nowMs;
    return;
  }

  const float shuntMv = ina219.getShuntVoltage_mV();
  const float busV = ina219.getBusVoltage_V();
  const float currentMa = ina219.getCurrent_mA();
  const float powerMw = ina219.getPower_mW();
  const float loadV = busV + shuntMv / 1000.0F;

  Serial.printf(
      "bus=%.3fV shunt=%+.3fmV load=%.3fV current=%+.1fmA power=%.3fW\n",
      busV, shuntMv, loadV, currentMa, powerMw / 1000.0F);
}
