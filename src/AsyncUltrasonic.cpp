#include "AsyncUltrasonic.h"

#include "Config.h"

volatile uint32_t AsyncUltrasonic::echoRiseUs_ = 0;
volatile uint32_t AsyncUltrasonic::echoPulseUs_ = 0;
volatile bool AsyncUltrasonic::echoComplete_ = false;

void AsyncUltrasonic::begin() {
  pinMode(config::pin::ULTRASONIC_TRIGGER, OUTPUT);
  digitalWrite(config::pin::ULTRASONIC_TRIGGER, LOW);
  pinMode(config::pin::ULTRASONIC_ECHO, INPUT);
  attachInterrupt(digitalPinToInterrupt(config::pin::ULTRASONIC_ECHO),
                  echoIsr, CHANGE);
}

void AsyncUltrasonic::echoIsr() {
  const uint32_t nowUs = micros();
  if (digitalRead(config::pin::ULTRASONIC_ECHO) == HIGH) {
    echoRiseUs_ = nowUs;
  } else if (echoRiseUs_ != 0) {
    echoPulseUs_ = nowUs - echoRiseUs_;
    echoComplete_ = true;
    echoRiseUs_ = 0;
  }
}

bool AsyncUltrasonic::request() {
  if (state_ != State::IDLE) {
    return false;
  }

  noInterrupts();
  echoComplete_ = false;
  echoPulseUs_ = 0;
  echoRiseUs_ = 0;
  interrupts();

  digitalWrite(config::pin::ULTRASONIC_TRIGGER, LOW);
  delayMicroseconds(3);
  digitalWrite(config::pin::ULTRASONIC_TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(config::pin::ULTRASONIC_TRIGGER, LOW);

  requestedAtUs_ = micros();
  state_ = State::WAITING_ECHO;
  return true;
}

void AsyncUltrasonic::update() {
  if (state_ != State::WAITING_ECHO) {
    return;
  }

  bool complete = false;
  uint32_t pulseUs = 0;
  noInterrupts();
  complete = echoComplete_;
  pulseUs = echoPulseUs_;
  interrupts();

  if (complete) {
    // Speed of sound round trip: distance cm = microseconds / 58.
    const uint32_t cm = (pulseUs + 29UL) / 58UL;
    resultValid_ =
        cm >= config::sonar::MIN_VALID_CM && cm <= config::sonar::MAX_VALID_CM;
    resultCm_ =
        resultValid_ ? static_cast<uint16_t>(cm) : config::sonar::MAX_VALID_CM;
    state_ = State::READY;
  } else if (micros() - requestedAtUs_ >= config::sonar::ECHO_TIMEOUT_US) {
    resultCm_ = config::sonar::MAX_VALID_CM;
    resultValid_ = false;
    state_ = State::READY;
  }
}

bool AsyncUltrasonic::takeResult(uint16_t& distanceCm, bool& valid) {
  if (state_ != State::READY) {
    return false;
  }
  distanceCm = resultCm_;
  valid = resultValid_;
  state_ = State::IDLE;
  return true;
}

bool AsyncUltrasonic::busy() const {
  return state_ != State::IDLE;
}
