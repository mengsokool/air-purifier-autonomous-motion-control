#include <unity.h>

#include "AirQualityPolicy.h"

void test_pm10_alone_can_raise_fan_to_medium() {
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(FanLevel::MEDIUM_SPEED),
      static_cast<uint8_t>(AirQualityPolicy::nextFanLevel(
          FanLevel::LOW_SPEED, 10, 90)));
}

void test_pm10_alone_can_raise_fan_to_high() {
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(FanLevel::HIGH_SPEED),
      static_cast<uint8_t>(AirQualityPolicy::nextFanLevel(
          FanLevel::LOW_SPEED, 10, 160)));
}

void test_fan_does_not_drop_until_both_pollutants_are_low() {
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(FanLevel::MEDIUM_SPEED),
      static_cast<uint8_t>(AirQualityPolicy::nextFanLevel(
          FanLevel::MEDIUM_SPEED, 20, 70)));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(FanLevel::LOW_SPEED),
      static_cast<uint8_t>(AirQualityPolicy::nextFanLevel(
          FanLevel::MEDIUM_SPEED, 10, 20)));
}

void test_high_hysteresis_handles_pm10() {
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(FanLevel::HIGH_SPEED),
      static_cast<uint8_t>(AirQualityPolicy::nextFanLevel(
          FanLevel::HIGH_SPEED, 20, 130)));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(FanLevel::MEDIUM_SPEED),
      static_cast<uint8_t>(AirQualityPolicy::nextFanLevel(
          FanLevel::HIGH_SPEED, 20, 60)));
}

void test_latest_sample_raises_fan_without_waiting_for_long_average() {
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(FanLevel::HIGH_SPEED),
      static_cast<uint8_t>(AirQualityPolicy::nextFanLevel(
          FanLevel::LOW_SPEED, 7, 7, 55, 60)));
}

void test_latest_clean_sample_cannot_drop_fan_early() {
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(FanLevel::HIGH_SPEED),
      static_cast<uint8_t>(AirQualityPolicy::nextFanLevel(
          FanLevel::HIGH_SPEED, 60, 65, 5, 5)));
}

void test_display_quality_uses_worse_pollutant() {
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(AirQualityLevel::GOOD),
      static_cast<uint8_t>(AirQualityPolicy::classify(10, 20)));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(AirQualityLevel::FAIR),
      static_cast<uint8_t>(AirQualityPolicy::classify(20, 90)));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(AirQualityLevel::POOR),
      static_cast<uint8_t>(AirQualityPolicy::classify(20, 160)));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_pm10_alone_can_raise_fan_to_medium);
  RUN_TEST(test_pm10_alone_can_raise_fan_to_high);
  RUN_TEST(test_fan_does_not_drop_until_both_pollutants_are_low);
  RUN_TEST(test_high_hysteresis_handles_pm10);
  RUN_TEST(test_latest_sample_raises_fan_without_waiting_for_long_average);
  RUN_TEST(test_latest_clean_sample_cannot_drop_fan_early);
  RUN_TEST(test_display_quality_uses_worse_pollutant);
  return UNITY_END();
}
