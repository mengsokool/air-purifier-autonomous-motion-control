#include "CorridorCommitment.h"

namespace {
uint16_t absoluteDifference(const uint16_t left, const uint16_t right) {
  return left >= right ? static_cast<uint16_t>(left - right)
                       : static_cast<uint16_t>(right - left);
}

int8_t correctionSign(const uint16_t left, const uint16_t right) {
  // Positive steering speeds the left wheel and curves right, away from a
  // closer left wall. Negative steering does the symmetric correction.
  return left < right ? 1 : -1;
}
}  // namespace

int8_t CorridorCommitment::update(
    const uint32_t nowMs, const uint16_t leftClearanceCm,
    const uint16_t rightClearanceCm, const bool corridorActive,
    const bool pairedReadingsFresh, const uint8_t deadbandCm,
    const uint8_t progressPercent, const uint16_t minimumCommitMs,
    const uint16_t maximumCommitMs, const uint16_t settleMs) {
  if (!corridorActive) {
    reset();
    return 0;
  }

  if (settling_) {
    if (nowMs - settleStartedMs_ < settleMs) {
      return 0;
    }
    settling_ = false;
  }

  if (!pairedReadingsFresh) {
    if (sign_ != 0 &&
        nowMs - commitStartedMs_ >= maximumCommitMs) {
      beginSettling(nowMs);
    }
    return sign_;
  }

  const uint16_t errorCm =
      absoluteDifference(leftClearanceCm, rightClearanceCm);
  if (sign_ == 0) {
    if (errorCm <= deadbandCm) {
      return 0;
    }
    sign_ = correctionSign(leftClearanceCm, rightClearanceCm);
    baselineErrorCm_ = errorCm;
    commitStartedMs_ = nowMs;
    return sign_;
  }

  if (nowMs - commitStartedMs_ < minimumCommitMs) {
    return sign_;
  }

  const uint8_t boundedProgressPercent =
      progressPercent > 100U ? 100U : progressPercent;
  uint16_t targetErrorCm = static_cast<uint16_t>(
      (static_cast<uint32_t>(baselineErrorCm_) *
       (100U - boundedProgressPercent)) /
      100U);
  if (targetErrorCm < deadbandCm) {
    targetErrorCm = deadbandCm;
  }
  const int8_t newestSign =
      errorCm > deadbandCm
          ? correctionSign(leftClearanceCm, rightClearanceCm)
          : 0;
  const bool enoughProgress = errorCm <= targetErrorCm;
  const bool crossedCentre = newestSign != 0 && newestSign != sign_;
  const bool timedOut =
      nowMs - commitStartedMs_ >= maximumCommitMs;
  if (enoughProgress || crossedCentre || timedOut) {
    beginSettling(nowMs);
    return 0;
  }
  return sign_;
}

void CorridorCommitment::reset() {
  sign_ = 0;
  baselineErrorCm_ = 0;
  commitStartedMs_ = 0;
  settling_ = false;
  settleStartedMs_ = 0;
}

void CorridorCommitment::beginSettling(const uint32_t nowMs) {
  sign_ = 0;
  baselineErrorCm_ = 0;
  commitStartedMs_ = 0;
  settling_ = true;
  settleStartedMs_ = nowMs;
}
