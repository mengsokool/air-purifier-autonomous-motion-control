#pragma once

class BatterySocModel {
 public:
  // Approximate 12 V sealed lead-acid state of charge from terminal voltage.
  // Most useful after the battery has rested; loaded readings are smoothed by
  // BatterySocEstimator before being shown to the user.
  static float voltageToPercent(float voltage);
  static float chargedMahToPercent(float chargedMah, float capacityMah,
                                   float chargeEfficiency);
};
