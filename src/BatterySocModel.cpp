#include "BatterySocModel.h"

#include <stddef.h>

namespace {
struct VoltageSocPoint {
  float voltage;
  float percent;
};

// Generic resting-voltage curve for a 12 V (6-cell) sealed lead-acid battery.
// The endpoints deliberately match the firmware's conservative cutoff region.
constexpr VoltageSocPoint CURVE[] = {
    {10.80F, 0.0F},  {11.31F, 10.0F}, {11.58F, 20.0F},
    {11.75F, 30.0F}, {11.90F, 40.0F}, {12.06F, 50.0F},
    {12.20F, 60.0F}, {12.32F, 70.0F}, {12.42F, 80.0F},
    {12.50F, 90.0F}, {12.73F, 100.0F},
};
constexpr size_t CURVE_SIZE = sizeof(CURVE) / sizeof(CURVE[0]);
}  // namespace

float BatterySocModel::voltageToPercent(const float voltage) {
  if (voltage <= CURVE[0].voltage) return 0.0F;
  if (voltage >= CURVE[CURVE_SIZE - 1U].voltage) return 100.0F;
  for (size_t i = 1; i < CURVE_SIZE; ++i) {
    if (voltage > CURVE[i].voltage) continue;
    const VoltageSocPoint& low = CURVE[i - 1U];
    const VoltageSocPoint& high = CURVE[i];
    const float fraction =
        (voltage - low.voltage) / (high.voltage - low.voltage);
    return low.percent + fraction * (high.percent - low.percent);
  }
  return 100.0F;
}

float BatterySocModel::chargedMahToPercent(const float chargedMah,
                                           const float capacityMah,
                                           const float chargeEfficiency) {
  if (chargedMah <= 0.0F || capacityMah <= 0.0F ||
      chargeEfficiency <= 0.0F) {
    return 0.0F;
  }
  return chargedMah * chargeEfficiency * 100.0F / capacityMah;
}
