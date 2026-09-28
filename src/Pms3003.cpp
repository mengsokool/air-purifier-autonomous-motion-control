#include "Pms3003.h"

#include "Config.h"
#include "PmsFrameData.h"

Pms3003::Pms3003(Stream& serial) : serial_(serial) {}

void Pms3003::begin() {
  startedAtMs_ = millis();
}

void Pms3003::update(const uint32_t nowMs) {
  while (serial_.available() > 0) {
    consume(static_cast<uint8_t>(serial_.read()), nowMs);
  }
}

void Pms3003::consume(const uint8_t byteValue, const uint32_t nowMs) {
  ++rawBytes_;

  if (frameIndex_ == 0 && byteValue != 0x42) {
    return;
  }
  if (frameIndex_ == 1 && byteValue != 0x4D) {
    frameIndex_ = byteValue == 0x42 ? 1 : 0;
    return;
  }

  frame_[frameIndex_++] = byteValue;

  if (frameIndex_ == HEADER_SIZE) {
    const uint16_t payloadSize =
        static_cast<uint16_t>(
            (static_cast<uint16_t>(frame_[2]) << 8U) |
            frame_[3]);
    if (payloadSize != SHORT_PAYLOAD_SIZE &&
        payloadSize != LONG_PAYLOAD_SIZE) {
      ++invalidFrames_;
      resetFrame();
      return;
    }
    expectedFrameSize_ =
        static_cast<uint8_t>(HEADER_SIZE + payloadSize);
  }

  if (expectedFrameSize_ == 0 ||
      frameIndex_ < expectedFrameSize_) {
    return;
  }

  if (validateFrame(expectedFrameSize_)) {
    acceptFrame(nowMs);
  } else {
    ++invalidFrames_;
  }
  resetFrame();
}

void Pms3003::resetFrame() {
  frameIndex_ = 0;
  expectedFrameSize_ = 0;
}

bool Pms3003::validateFrame(const uint8_t frameSize) const {
  if (frameSize < 6U || frameSize > MAX_FRAME_SIZE) {
    return false;
  }
  const uint16_t frameLength =
      static_cast<uint16_t>(
          (static_cast<uint16_t>(frame_[2]) << 8U) |
          frame_[3]);
  if (frameLength + HEADER_SIZE != frameSize) {
    return false;
  }

  uint16_t checksum = 0;
  for (uint8_t i = 0; i < frameSize - 2U; ++i) {
    checksum = static_cast<uint16_t>(checksum + frame_[i]);
  }
  const uint16_t expected =
      static_cast<uint16_t>(
          (static_cast<uint16_t>(frame_[frameSize - 2U]) << 8U) |
          frame_[frameSize - 1U]);
  return checksum == expected;
}

void Pms3003::acceptFrame(const uint32_t nowMs) {
  // Atmospheric-environment PM1.0, PM2.5 and PM10 occupy bytes 10..15.
  latestPm1_ =
      pms_frame::readUint16(frame_, pms_frame::ATMOSPHERIC_PM1_OFFSET);
  latestPm25_ =
      pms_frame::readUint16(frame_, pms_frame::ATMOSPHERIC_PM25_OFFSET);
  latestPm10_ =
      pms_frame::readUint16(frame_, pms_frame::ATMOSPHERIC_PM10_OFFSET);

  if (sampleCount_ == config::pms::MOVING_AVERAGE_SAMPLES) {
    pm1SampleSum_ -= pm1Samples_[sampleIndex_];
    pm25SampleSum_ -= pm25Samples_[sampleIndex_];
    pm10SampleSum_ -= pm10Samples_[sampleIndex_];
  } else {
    ++sampleCount_;
  }

  pm1Samples_[sampleIndex_] = latestPm1_;
  pm25Samples_[sampleIndex_] = latestPm25_;
  pm10Samples_[sampleIndex_] = latestPm10_;
  pm1SampleSum_ += latestPm1_;
  pm25SampleSum_ += latestPm25_;
  pm10SampleSum_ += latestPm10_;
  sampleIndex_ =
      static_cast<uint8_t>((sampleIndex_ + 1U) %
                           config::pms::MOVING_AVERAGE_SAMPLES);
  lastValidAtMs_ = nowMs;
  ++validFrames_;
}

bool Pms3003::healthy(const uint32_t nowMs) const {
  return hasReading() && (nowMs - lastValidAtMs_ <= config::pms::STALE_MS);
}

bool Pms3003::warmedUp(const uint32_t nowMs) const {
  return nowMs - startedAtMs_ >= config::pms::WARMUP_MS;
}

uint16_t Pms3003::pm1Average() const {
  return sampleCount_ == 0
             ? 0
             : static_cast<uint16_t>(pm1SampleSum_ / sampleCount_);
}

uint16_t Pms3003::pm25Average() const {
  return sampleCount_ == 0
             ? 0
             : static_cast<uint16_t>(pm25SampleSum_ / sampleCount_);
}

uint16_t Pms3003::pm10Average() const {
  return sampleCount_ == 0
             ? 0
             : static_cast<uint16_t>(pm10SampleSum_ / sampleCount_);
}
