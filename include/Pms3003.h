#pragma once

#include <Arduino.h>

#include "Config.h"

class Pms3003 {
 public:
  explicit Pms3003(Stream& serial);

  void begin();
  void update(uint32_t nowMs);
  bool healthy(uint32_t nowMs) const;
  bool warmedUp(uint32_t nowMs) const;
  bool hasReading() const { return sampleCount_ > 0; }
  uint16_t pm1Average() const;
  uint16_t pm25Average() const;
  uint16_t pm10Average() const;
  uint16_t pm1Latest() const { return latestPm1_; }
  uint16_t pm25Latest() const { return latestPm25_; }
  uint16_t pm10Latest() const { return latestPm10_; }
  uint32_t rawBytes() const { return rawBytes_; }
  uint32_t validFrames() const { return validFrames_; }
  uint32_t invalidFrames() const { return invalidFrames_; }

 private:
  static constexpr uint8_t HEADER_SIZE = 4;
  static constexpr uint8_t SHORT_PAYLOAD_SIZE = 20;
  static constexpr uint8_t LONG_PAYLOAD_SIZE = 28;
  static constexpr uint8_t MAX_FRAME_SIZE =
      HEADER_SIZE + LONG_PAYLOAD_SIZE;
  Stream& serial_;
  uint8_t frame_[MAX_FRAME_SIZE]{};
  uint8_t frameIndex_ = 0;
  uint8_t expectedFrameSize_ = 0;
  uint32_t startedAtMs_ = 0;
  uint32_t lastValidAtMs_ = 0;
  uint32_t rawBytes_ = 0;
  uint32_t validFrames_ = 0;
  uint32_t invalidFrames_ = 0;
  uint16_t latestPm1_ = 0;
  uint16_t latestPm25_ = 0;
  uint16_t latestPm10_ = 0;
  uint16_t pm1Samples_[config::pms::MOVING_AVERAGE_SAMPLES]{};
  uint16_t pm25Samples_[config::pms::MOVING_AVERAGE_SAMPLES]{};
  uint16_t pm10Samples_[config::pms::MOVING_AVERAGE_SAMPLES]{};
  uint8_t sampleIndex_ = 0;
  uint8_t sampleCount_ = 0;
  uint32_t pm1SampleSum_ = 0;
  uint32_t pm25SampleSum_ = 0;
  uint32_t pm10SampleSum_ = 0;

  void consume(uint8_t byteValue, uint32_t nowMs);
  void resetFrame();
  bool validateFrame(uint8_t frameSize) const;
  void acceptFrame(uint32_t nowMs);
};
