#pragma once

#include <Arduino.h>

struct EncoderCounts {
  int32_t left;
  int32_t right;
};

// Dual x4 quadrature decoder using ESP32-S3 PCNT hardware. Each wheel uses
// both PCNT channels: A edges gated by B and B edges gated by A.
class QuadratureEncoders {
 public:
  void begin();
  EncoderCounts read() const;
  void reset();

 private:
  static void handleLeftLimit(void*);
  static void handleRightLimit(void*);
};
