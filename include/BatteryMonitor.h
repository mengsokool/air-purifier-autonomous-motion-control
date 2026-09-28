#pragma once

#include <Arduino.h>

class BatteryMonitor {
 public:
  void begin();
  void update(uint32_t nowMs);
  void updateProtection(uint32_t nowMs, bool voltageValid,
                        float authoritativeVoltage);

  float voltage() const { return filteredVoltage_; }
  bool hasReading() const { return hasReading_; }
  bool warning() const;
  bool cutoff() const { return cutoffLatched_; }

 private:
  uint32_t lastSampleMs_ = 0;
  uint32_t lowSinceMs_ = 0;
  uint32_t adcSum_ = 0;
  uint8_t sampleCount_ = 0;
  float filteredVoltage_ = 0.0F;
  float protectionVoltage_ = 0.0F;
  bool hasReading_ = false;
  bool protectionVoltageValid_ = false;
  bool cutoffLatched_ = false;
};
