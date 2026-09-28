#pragma once

#include <Arduino.h>

class AsyncUltrasonic {
 public:
  void begin();
  void update();
  bool request();
  bool takeResult(uint16_t& distanceCm, bool& valid);
  bool busy() const;

 private:
  enum class State : uint8_t { IDLE, WAITING_ECHO, READY };

  static volatile uint32_t echoRiseUs_;
  static volatile uint32_t echoPulseUs_;
  static volatile bool echoComplete_;
  static void echoIsr();

  State state_ = State::IDLE;
  uint32_t requestedAtUs_ = 0;
  uint16_t resultCm_ = 0;
  bool resultValid_ = false;
};
