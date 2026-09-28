#include <unity.h>

#include "PmsFrameData.h"

namespace {

// Captured from the tested PMS3003. Atmospheric values are PM1=10,
// PM2.5=14 and PM10=14 micrograms per cubic metre.
constexpr uint8_t FRAME[] = {
    0x42, 0x4D, 0x00, 0x14, 0x00, 0x0A, 0x00, 0x0E,
    0x00, 0x0E, 0x00, 0x0A, 0x00, 0x0E, 0x00, 0x0E,
    0x01, 0xF8, 0x00, 0x05, 0x97, 0x00, 0x02, 0x84,
};

void test_reads_all_atmospheric_particle_values() {
  TEST_ASSERT_EQUAL_UINT16(
      10, pms_frame::readUint16(
              FRAME, pms_frame::ATMOSPHERIC_PM1_OFFSET));
  TEST_ASSERT_EQUAL_UINT16(
      14, pms_frame::readUint16(
              FRAME, pms_frame::ATMOSPHERIC_PM25_OFFSET));
  TEST_ASSERT_EQUAL_UINT16(
      14, pms_frame::readUint16(
              FRAME, pms_frame::ATMOSPHERIC_PM10_OFFSET));
}

}  // namespace

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_reads_all_atmospheric_particle_values);
  return UNITY_END();
}
