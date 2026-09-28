#include "QuadratureEncoders.h"

#include <driver/pcnt.h>
#include <esp_err.h>

#include "Config.h"

namespace {
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;
constexpr int16_t COUNTER_HIGH_LIMIT = 30000;
constexpr int16_t COUNTER_LOW_LIMIT = -30000;

// PCNT's counter is 16-bit. These bases retain the total position across the
// infrequent hardware-limit interrupts, while PCNT counts every edge itself.
volatile int32_t leftBaseCount = 0;
volatile int32_t rightBaseCount = 0;

void configureWheel(const pcnt_unit_t unit, const int pulseA, const int pulseB) {
  // Channel 0 counts A's rising and falling edges. B selects direction.
  pcnt_config_t channelA{};
  channelA.pulse_gpio_num = pulseA;
  channelA.ctrl_gpio_num = pulseB;
  channelA.lctrl_mode = PCNT_MODE_KEEP;
  channelA.hctrl_mode = PCNT_MODE_REVERSE;
  channelA.pos_mode = PCNT_COUNT_INC;
  channelA.neg_mode = PCNT_COUNT_DEC;
  channelA.counter_h_lim = COUNTER_HIGH_LIMIT;
  channelA.counter_l_lim = COUNTER_LOW_LIMIT;
  channelA.unit = unit;
  channelA.channel = PCNT_CHANNEL_0;
  ESP_ERROR_CHECK(pcnt_unit_config(&channelA));

  // Channel 1 counts B's edges with the complementary A direction gate.
  // The two channels share the unit's counter, giving true x4 decoding.
  pcnt_config_t channelB = channelA;
  channelB.pulse_gpio_num = pulseB;
  channelB.ctrl_gpio_num = pulseA;
  channelB.lctrl_mode = PCNT_MODE_REVERSE;
  channelB.hctrl_mode = PCNT_MODE_KEEP;
  channelB.channel = PCNT_CHANNEL_1;
  ESP_ERROR_CHECK(pcnt_unit_config(&channelB));

  ESP_ERROR_CHECK(pcnt_set_filter_value(
      unit, config::motor::ENCODER_GLITCH_FILTER_CYCLES));
  ESP_ERROR_CHECK(pcnt_filter_enable(unit));
  ESP_ERROR_CHECK(pcnt_event_enable(unit, PCNT_EVT_H_LIM));
  ESP_ERROR_CHECK(pcnt_event_enable(unit, PCNT_EVT_L_LIM));
  ESP_ERROR_CHECK(pcnt_counter_clear(unit));
}

int32_t totalCount(const pcnt_unit_t unit, const volatile int32_t& base) {
  int16_t hardwareCount = 0;
  ESP_ERROR_CHECK(pcnt_get_counter_value(unit, &hardwareCount));
  return base + static_cast<int32_t>(hardwareCount);
}
}  // namespace

void QuadratureEncoders::begin() {
  pinMode(config::pin::ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(config::pin::ENCODER_LEFT_B, INPUT_PULLUP);
  pinMode(config::pin::ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(config::pin::ENCODER_RIGHT_B, INPUT_PULLUP);

  configureWheel(PCNT_UNIT_0, config::pin::ENCODER_LEFT_A,
                 config::pin::ENCODER_LEFT_B);
  configureWheel(PCNT_UNIT_1, config::pin::ENCODER_RIGHT_A,
                 config::pin::ENCODER_RIGHT_B);

  ESP_ERROR_CHECK(pcnt_isr_service_install(0));
  ESP_ERROR_CHECK(pcnt_isr_handler_add(PCNT_UNIT_0, handleLeftLimit, nullptr));
  ESP_ERROR_CHECK(pcnt_isr_handler_add(PCNT_UNIT_1, handleRightLimit, nullptr));
  ESP_ERROR_CHECK(pcnt_intr_enable(PCNT_UNIT_0));
  ESP_ERROR_CHECK(pcnt_intr_enable(PCNT_UNIT_1));
}

EncoderCounts QuadratureEncoders::read() const {
  EncoderCounts result{};
  portENTER_CRITICAL(&encoderMux);
  result.left = totalCount(PCNT_UNIT_0, leftBaseCount);
  result.right = totalCount(PCNT_UNIT_1, rightBaseCount);
  portEXIT_CRITICAL(&encoderMux);
  if (config::motor::ENCODER_LEFT_INVERTED) {
    result.left = -result.left;
  }
  if (config::motor::ENCODER_RIGHT_INVERTED) {
    result.right = -result.right;
  }
  return result;
}

void QuadratureEncoders::reset() {
  ESP_ERROR_CHECK(pcnt_counter_pause(PCNT_UNIT_0));
  ESP_ERROR_CHECK(pcnt_counter_pause(PCNT_UNIT_1));
  portENTER_CRITICAL(&encoderMux);
  leftBaseCount = 0;
  rightBaseCount = 0;
  ESP_ERROR_CHECK(pcnt_counter_clear(PCNT_UNIT_0));
  ESP_ERROR_CHECK(pcnt_counter_clear(PCNT_UNIT_1));
  portEXIT_CRITICAL(&encoderMux);
  ESP_ERROR_CHECK(pcnt_counter_resume(PCNT_UNIT_0));
  ESP_ERROR_CHECK(pcnt_counter_resume(PCNT_UNIT_1));
}

void QuadratureEncoders::handleLeftLimit(void*) {
  uint32_t status = 0;
  pcnt_get_event_status(PCNT_UNIT_0, &status);
  portENTER_CRITICAL_ISR(&encoderMux);
  if (status & PCNT_EVT_H_LIM) {
    leftBaseCount += COUNTER_HIGH_LIMIT;
  } else if (status & PCNT_EVT_L_LIM) {
    leftBaseCount += COUNTER_LOW_LIMIT;
  }
  pcnt_counter_clear(PCNT_UNIT_0);
  portEXIT_CRITICAL_ISR(&encoderMux);
}

void QuadratureEncoders::handleRightLimit(void*) {
  uint32_t status = 0;
  pcnt_get_event_status(PCNT_UNIT_1, &status);
  portENTER_CRITICAL_ISR(&encoderMux);
  if (status & PCNT_EVT_H_LIM) {
    rightBaseCount += COUNTER_HIGH_LIMIT;
  } else if (status & PCNT_EVT_L_LIM) {
    rightBaseCount += COUNTER_LOW_LIMIT;
  }
  pcnt_counter_clear(PCNT_UNIT_1);
  portEXIT_CRITICAL_ISR(&encoderMux);
}
