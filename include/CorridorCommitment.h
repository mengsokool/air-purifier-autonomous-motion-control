#pragma once

#include <stdint.h>

class CorridorCommitment {
 public:
  int8_t update(uint32_t nowMs,
                uint16_t leftClearanceCm,
                uint16_t rightClearanceCm,
                bool corridorActive,
                bool pairedReadingsFresh,
                uint8_t deadbandCm,
                uint8_t progressPercent,
                uint16_t minimumCommitMs,
                uint16_t maximumCommitMs,
                uint16_t settleMs);
  void reset();
  int8_t sign() const { return sign_; }
  uint16_t baselineErrorCm() const { return baselineErrorCm_; }

 private:
  void beginSettling(uint32_t nowMs);

  int8_t sign_ = 0;
  uint16_t baselineErrorCm_ = 0;
  uint32_t commitStartedMs_ = 0;
  bool settling_ = false;
  uint32_t settleStartedMs_ = 0;
};
