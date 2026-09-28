#include "AirQualityPolicy.h"

#include "Config.h"

AirQualityLevel AirQualityPolicy::classify(const uint16_t pm25,
                                           const uint16_t pm10) {
  if (pm25 >= config::fan::PM25_MEDIUM_TO_HIGH ||
      pm10 >= config::fan::PM10_MEDIUM_TO_HIGH) {
    return AirQualityLevel::POOR;
  }
  if (pm25 <= config::fan::PM25_MEDIUM_TO_LOW &&
      pm10 <= config::fan::PM10_MEDIUM_TO_LOW) {
    return AirQualityLevel::GOOD;
  }
  return AirQualityLevel::FAIR;
}

FanLevel AirQualityPolicy::nextFanLevel(const FanLevel current,
                                        const uint16_t pm25Average,
                                        const uint16_t pm10Average,
                                        const uint16_t pm25Latest,
                                        const uint16_t pm10Latest) {
  const uint16_t pm25Rise =
      pm25Latest > pm25Average ? pm25Latest : pm25Average;
  const uint16_t pm10Rise =
      pm10Latest > pm10Average ? pm10Latest : pm10Average;
  const bool enterHigh =
      pm25Rise >= config::fan::PM25_MEDIUM_TO_HIGH ||
      pm10Rise >= config::fan::PM10_MEDIUM_TO_HIGH;
  const bool enterMedium =
      pm25Rise >= config::fan::PM25_LOW_TO_MEDIUM ||
      pm10Rise >= config::fan::PM10_LOW_TO_MEDIUM;
  const bool mayEnterLow =
      pm25Average <= config::fan::PM25_MEDIUM_TO_LOW &&
      pm10Average <= config::fan::PM10_MEDIUM_TO_LOW;
  const bool mayLeaveHigh =
      pm25Average <= config::fan::PM25_HIGH_TO_MEDIUM &&
      pm10Average <= config::fan::PM10_HIGH_TO_MEDIUM;

  switch (current) {
    case FanLevel::LOW_SPEED:
      if (enterHigh) return FanLevel::HIGH_SPEED;
      if (enterMedium) return FanLevel::MEDIUM_SPEED;
      return FanLevel::LOW_SPEED;

    case FanLevel::MEDIUM_SPEED:
    case FanLevel::FAILSAFE:
      if (enterHigh) return FanLevel::HIGH_SPEED;
      if (mayEnterLow) return FanLevel::LOW_SPEED;
      return FanLevel::MEDIUM_SPEED;

    case FanLevel::HIGH_SPEED:
      if (mayEnterLow) return FanLevel::LOW_SPEED;
      if (mayLeaveHigh) return FanLevel::MEDIUM_SPEED;
      return FanLevel::HIGH_SPEED;
  }
  return FanLevel::FAILSAFE;
}
