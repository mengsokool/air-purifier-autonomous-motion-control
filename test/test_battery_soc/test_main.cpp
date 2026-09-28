#include <unity.h>

#include "BatterySocModel.h"

void test_voltage_curve_clamps_at_empty_and_full() {
  TEST_ASSERT_FLOAT_WITHIN(0.01F, 0.0F,
                           BatterySocModel::voltageToPercent(10.0F));
  TEST_ASSERT_FLOAT_WITHIN(0.01F, 100.0F,
                           BatterySocModel::voltageToPercent(13.5F));
}

void test_voltage_curve_uses_piecewise_interpolation() {
  TEST_ASSERT_FLOAT_WITHIN(0.1F, 50.0F,
                           BatterySocModel::voltageToPercent(12.06F));
  TEST_ASSERT_FLOAT_WITHIN(0.2F, 65.0F,
                           BatterySocModel::voltageToPercent(12.26F));
}

void test_charged_mah_respects_capacity_and_efficiency() {
  TEST_ASSERT_FLOAT_WITHIN(
      0.01F, 42.5F,
      BatterySocModel::chargedMahToPercent(6000.0F, 12000.0F, 0.85F));
}

void test_invalid_charge_inputs_return_zero() {
  TEST_ASSERT_FLOAT_WITHIN(
      0.01F, 0.0F,
      BatterySocModel::chargedMahToPercent(1000.0F, 0.0F, 0.85F));
  TEST_ASSERT_FLOAT_WITHIN(
      0.01F, 0.0F,
      BatterySocModel::chargedMahToPercent(-1.0F, 12000.0F, 0.85F));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_voltage_curve_clamps_at_empty_and_full);
  RUN_TEST(test_voltage_curve_uses_piecewise_interpolation);
  RUN_TEST(test_charged_mah_respects_capacity_and_efficiency);
  RUN_TEST(test_invalid_charge_inputs_return_zero);
  return UNITY_END();
}
