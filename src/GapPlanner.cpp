#include "GapPlanner.h"

uint16_t GapPlanner::applyDirectionalPenalty(
    const uint16_t distanceCm, const uint8_t angleDifferenceDeg,
    const uint8_t penaltyRadiusDeg, const uint16_t openDistanceCm,
    const uint8_t centreRetainPercent, const uint8_t edgeRetainPercent) {
  if (penaltyRadiusDeg == 0 || angleDifferenceDeg > penaltyRadiusDeg) {
    return distanceCm;
  }
  const uint8_t retainRange = edgeRetainPercent > centreRetainPercent
                                  ? edgeRetainPercent - centreRetainPercent
                                  : 0;
  const uint16_t retainedPercent =
      static_cast<uint16_t>(centreRetainPercent) +
      (static_cast<uint16_t>(angleDifferenceDeg) * retainRange) /
          penaltyRadiusDeg;
  const uint16_t adjustedDistance = static_cast<uint16_t>(
      (static_cast<uint32_t>(distanceCm) * retainedPercent) / 100UL);
  return distanceCm >= openDistanceCm && adjustedDistance < openDistanceCm
             ? openDistanceCm
             : adjustedDistance;
}

GapSelection GapPlanner::selectBestCorridor(
    const uint16_t* distancesCm, const bool* valid, const uint8_t pointCount,
    const uint8_t minAngleDeg, const uint8_t stepDeg,
    const uint8_t centreAngleDeg, const uint16_t openDistanceCm,
    const uint8_t corridorPoints) {
  GapSelection best{false, centreAngleDeg, 0, 0, 0};
  if (corridorPoints == 0 || corridorPoints > pointCount) return best;
  for (uint8_t start = 0; start + corridorPoints <= pointCount; ++start) {
    uint32_t distanceSum = 0;
    uint16_t minimumDistance = UINT16_MAX;
    bool corridorOpen = true;
    for (uint8_t offset = 0; offset < corridorPoints; ++offset) {
      const uint8_t index = static_cast<uint8_t>(start + offset);
      if (!valid[index] || distancesCm[index] < openDistanceCm) {
        corridorOpen = false;
        break;
      }
      distanceSum += distancesCm[index];
      if (distancesCm[index] < minimumDistance) minimumDistance = distancesCm[index];
    }
    if (!corridorOpen) continue;
    // Measure the complete contiguous opening around this chassis-width
    // window. Two windows can have similar depth, but the wider opening gives
    // the robot more tolerance for sonar error and steering overshoot.
    uint8_t runStart = start;
    while (runStart > 0 && valid[runStart - 1U] &&
           distancesCm[runStart - 1U] >= openDistanceCm) {
      --runStart;
    }
    uint8_t runEnd =
        static_cast<uint8_t>(start + corridorPoints - 1U);
    while (runEnd + 1U < pointCount && valid[runEnd + 1U] &&
           distancesCm[runEnd + 1U] >= openDistanceCm) {
      ++runEnd;
    }
    const uint8_t runWidth =
        static_cast<uint8_t>(runEnd - runStart + 1U);
    const uint8_t middleIndex = static_cast<uint8_t>(start + corridorPoints / 2U);
    const uint8_t targetAngle = static_cast<uint8_t>(minAngleDeg + middleIndex * stepDeg);
    const uint8_t turnPenalty = targetAngle > centreAngleDeg
                                    ? targetAngle - centreAngleDeg
                                    : centreAngleDeg - targetAngle;
    // Bottleneck clearance dominates, average depth breaks close calls, and
    // full opening width rewards forgiving routes. The modest heading penalty
    // keeps a clear straight route preferable without hiding a safer side gap.
    uint32_t score = static_cast<uint32_t>(minimumDistance) * 6UL +
                     (distanceSum / corridorPoints) * 3UL +
                     static_cast<uint32_t>(runWidth) * 80UL;
    const uint32_t penalty = static_cast<uint32_t>(turnPenalty) * 3UL;
    score = score > penalty ? score - penalty : 0;
    if (!best.found || score > best.score) {
      best = {true, targetAngle, start,
              static_cast<uint8_t>(start + corridorPoints - 1U), score};
    }
  }
  return best;
}

GapSelection GapPlanner::select(
    const uint16_t* distancesCm, const bool* valid, const uint8_t pointCount,
    const uint8_t minAngleDeg, const uint8_t stepDeg,
    const uint8_t centreAngleDeg, const uint16_t openDistanceCm,
    const uint8_t minimumGapPoints) {
  GapSelection best{false, centreAngleDeg, 0, 0, 0};
  uint8_t index = 0;
  while (index < pointCount) {
    while (index < pointCount && (!valid[index] || distancesCm[index] < openDistanceCm)) ++index;
    if (index >= pointCount) break;
    const uint8_t start = index;
    uint32_t distanceSum = 0;
    uint16_t minimumDistance = UINT16_MAX;
    while (index < pointCount && valid[index] && distancesCm[index] >= openDistanceCm) {
      distanceSum += distancesCm[index];
      if (distancesCm[index] < minimumDistance) minimumDistance = distancesCm[index];
      ++index;
    }
    const uint8_t end = static_cast<uint8_t>(index - 1U);
    const uint8_t width = static_cast<uint8_t>(end - start + 1U);
    if (width < minimumGapPoints) continue;
    const uint8_t middleIndex = static_cast<uint8_t>(start + ((width - 1U) / 2U));
    const uint8_t targetAngle = static_cast<uint8_t>(minAngleDeg + middleIndex * stepDeg);
    const uint8_t turnPenalty = targetAngle > centreAngleDeg
                                    ? targetAngle - centreAngleDeg
                                    : centreAngleDeg - targetAngle;
    uint32_t score = static_cast<uint32_t>(width) * (distanceSum / width) +
                     static_cast<uint32_t>(minimumDistance) * 2UL;
    const uint32_t penalty = static_cast<uint32_t>(turnPenalty) * 2UL;
    score = score > penalty ? score - penalty : 0;
    if (!best.found || score > best.score) best = {true, targetAngle, start, end, score};
  }
  return best;
}
