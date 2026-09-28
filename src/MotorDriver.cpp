#include "MotorDriver.h"

#include <math.h>
#include <stdlib.h>

#include "Config.h"

namespace {
int16_t clampCommand(const int16_t command) {
  if (command > 255) {
    return 255;
  }
  if (command < -255) {
    return -255;
  }
  return command;
}

int32_t rampSpeed(const int32_t current,
                  const int32_t target,
                  const int32_t accelerationStep) {
  if (current == target) {
    return current;
  }

  // Crossing through zero must still stop before reversing.
  if (current > 0 && target < 0) {
    const int32_t next =
        current - config::motor::DECEL_RAMP_STEP_TPS;
    return next > 0 ? next : 0;
  }
  if (current < 0 && target > 0) {
    const int32_t next =
        current + config::motor::DECEL_RAMP_STEP_TPS;
    return next < 0 ? next : 0;
  }

  const bool reducingMagnitude =
      (current > 0 && target >= 0 && target < current) ||
      (current < 0 && target <= 0 && target > current);
  const int32_t step =
      reducingMagnitude
          ? config::motor::DECEL_RAMP_STEP_TPS
          : accelerationStep;
  if (target > current) {
    const int32_t next = current + step;
    return next > target ? target : next;
  }
  const int32_t next = current - step;
  return next < target ? target : next;
}
}  // namespace

void MotorDriver::begin() {
  pinMode(config::pin::MOTOR_LEFT_PWM, OUTPUT);
  pinMode(config::pin::MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(config::pin::MOTOR_LEFT_IN1, OUTPUT);
  pinMode(config::pin::MOTOR_LEFT_IN2, OUTPUT);
  pinMode(config::pin::MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(config::pin::MOTOR_RIGHT_IN2, OUTPUT);

  const bool leftPwmReady =
      ledcSetup(config::motor::PWM_LEFT_CHANNEL,
                config::motor::PWM_FREQUENCY_HZ,
                config::motor::PWM_RESOLUTION_BITS) > 0.0;
  const bool rightPwmReady =
      ledcSetup(config::motor::PWM_RIGHT_CHANNEL,
                config::motor::PWM_FREQUENCY_HZ,
                config::motor::PWM_RESOLUTION_BITS) > 0.0;
  ledcReady_ = leftPwmReady && rightPwmReady;
  if (ledcReady_) {
    ledcAttachPin(config::pin::MOTOR_LEFT_PWM,
                  config::motor::PWM_LEFT_CHANNEL);
    ledcAttachPin(config::pin::MOTOR_RIGHT_PWM,
                  config::motor::PWM_RIGHT_CHANNEL);
  } else {
    Serial.println(F("Motor LEDC setup failed"));
    digitalWrite(config::pin::MOTOR_LEFT_PWM, LOW);
    digitalWrite(config::pin::MOTOR_RIGHT_PWM, LOW);
  }

  encoders_.begin();
  lastControlCounts_ = encoders_.read();
  markedCounts_ = lastControlCounts_;
  lastControlMs_ = millis();
  lastMotionMs_ = lastControlMs_;
  wheelImbalanceStartedMs_ = 0;
  emergencyStop();
}

void MotorDriver::reset() {
  targetLeftTps_ = 0;
  targetRightTps_ = 0;
  activeLeftTps_ = 0;
  activeRightTps_ = 0;
  measuredLeftTps_ = 0;
  measuredRightTps_ = 0;
  appliedLeftPwm_ = 0;
  appliedRightPwm_ = 0;
  leftIntegral_ = 0.0F;
  rightIntegral_ = 0.0F;
  lastControlMs_ = millis();
  lastMotionMs_ = lastControlMs_;
  wheelImbalanceStartedMs_ = 0;
  emergencyStop();
}

void MotorDriver::driveForward(const int32_t ticksPerSecond) {
  if (targetLeftTps_ != ticksPerSecond || targetRightTps_ != ticksPerSecond) {
    syncStartCounts_ = encoders_.read();
    lastMotionMs_ = millis();
  }
  targetLeftTps_ = ticksPerSecond;
  targetRightTps_ = ticksPerSecond;
}

void MotorDriver::driveReverse(const int32_t ticksPerSecond) {
  if (targetLeftTps_ != -ticksPerSecond || targetRightTps_ != -ticksPerSecond) {
    syncStartCounts_ = encoders_.read();
    lastMotionMs_ = millis();
  }
  targetLeftTps_ = -ticksPerSecond;
  targetRightTps_ = -ticksPerSecond;
}

void MotorDriver::driveDifferential(const int32_t leftTps, const int32_t rightTps) {
  const bool wasDifferential =
      targetLeftTps_ != targetRightTps_;
  const bool returningToStraight =
      wasDifferential &&
      leftTps == rightTps &&
      ((leftTps >= 0 && activeLeftTps_ >= 0 &&
        activeRightTps_ >= 0) ||
       (leftTps <= 0 && activeLeftTps_ <= 0 &&
        activeRightTps_ <= 0));
  if (returningToStraight) {
    // Remove residual crab steering immediately. Match both active wheel
    // speeds and PWM at the slower channel, then let them accelerate together.
    const int32_t commonActiveMagnitude =
        min(labs(activeLeftTps_), labs(activeRightTps_));
    const int32_t requestedMagnitude = labs(leftTps);
    const int32_t recoveryMagnitude =
        min(commonActiveMagnitude, requestedMagnitude);
    const int32_t direction = leftTps < 0 ? -1 : 1;
    activeLeftTps_ = direction * recoveryMagnitude;
    activeRightTps_ = direction * recoveryMagnitude;

    const int16_t commonPwmMagnitude =
        min(abs(appliedLeftPwm_), abs(appliedRightPwm_));
    appliedLeftPwm_ =
        static_cast<int16_t>(direction * commonPwmMagnitude);
    appliedRightPwm_ = appliedLeftPwm_;
    leftIntegral_ = 0.0F;
    rightIntegral_ = 0.0F;
  }
  if (targetLeftTps_ != leftTps || targetRightTps_ != rightTps) {
    syncStartCounts_ = encoders_.read();
    lastMotionMs_ = millis();
  }
  targetLeftTps_ = leftTps;
  targetRightTps_ = rightTps;
}

void MotorDriver::turnLeft(const int32_t ticksPerSecond) {
  if (targetLeftTps_ != -ticksPerSecond || targetRightTps_ != ticksPerSecond) {
    syncStartCounts_ = encoders_.read();
    lastMotionMs_ = millis();
  }
  targetLeftTps_ = -ticksPerSecond;
  targetRightTps_ = ticksPerSecond;
}

void MotorDriver::turnRight(const int32_t ticksPerSecond) {
  if (targetLeftTps_ != ticksPerSecond || targetRightTps_ != -ticksPerSecond) {
    syncStartCounts_ = encoders_.read();
    lastMotionMs_ = millis();
  }
  targetLeftTps_ = ticksPerSecond;
  targetRightTps_ = -ticksPerSecond;
}

void MotorDriver::stop() {
  targetLeftTps_ = 0;
  targetRightTps_ = 0;
}

void MotorDriver::emergencyStop() {
  targetLeftTps_ = 0;
  targetRightTps_ = 0;
  activeLeftTps_ = 0;
  activeRightTps_ = 0;
  appliedLeftPwm_ = 0;
  appliedRightPwm_ = 0;
  leftIntegral_ = 0.0F;
  rightIntegral_ = 0.0F;
  wheelImbalanceStartedMs_ = 0;
  apply();
}

int16_t MotorDriver::slew(const int16_t current, const int16_t target) {
  const int16_t step = config::motor::PWM_SLEW_PER_UPDATE;
  if (target > current + step) {
    return current + step;
  }
  if (target < current - step) {
    return current - step;
  }
  return target;
}

void MotorDriver::setSoftStartDuration(const float durationSeconds) {
  if (durationSeconds <= 0.1F) {
    rampStepPerUpdate_ = config::motor::CRUISE_TPS;
    return;
  }
  const float updatesCount = (durationSeconds * 1000.0F) / static_cast<float>(config::motor::CONTROL_UPDATE_MS);
  rampStepPerUpdate_ = static_cast<int32_t>(static_cast<float>(config::motor::CRUISE_TPS) / updatesCount);
  if (rampStepPerUpdate_ < 10) rampStepPerUpdate_ = 10;
}

void MotorDriver::update(const uint32_t nowMs) {
  const uint32_t elapsedMs = nowMs - lastControlMs_;
  if (elapsedMs < config::motor::CONTROL_UPDATE_MS) {
    return;
  }
  lastControlMs_ = nowMs;

  const EncoderCounts counts = encoders_.read();
  const int32_t leftDelta = counts.left - lastControlCounts_.left;
  const int32_t rightDelta = counts.right - lastControlCounts_.right;
  lastControlCounts_ = counts;

  const int32_t rawLeftTps = (leftDelta * 1000L) / static_cast<int32_t>(elapsedMs);
  const int32_t rawRightTps = (rightDelta * 1000L) / static_cast<int32_t>(elapsedMs);

  // Low-pass filter (alpha = 0.35) to eliminate single-sample encoder delta noise spikes
  measuredLeftTps_ += static_cast<int32_t>(0.35F * static_cast<float>(rawLeftTps - measuredLeftTps_));
  measuredRightTps_ += static_cast<int32_t>(0.35F * static_cast<float>(rawRightTps - measuredRightTps_));

  if (leftDelta != 0 || rightDelta != 0) {
    lastMotionMs_ = nowMs;
  }

  // Smooth Electronic Soft-Start Ramping (Eliminates heavy current/voltage spikes during acceleration)
  activeLeftTps_ =
      rampSpeed(activeLeftTps_, targetLeftTps_, rampStepPerUpdate_);
  activeRightTps_ =
      rampSpeed(activeRightTps_, targetRightTps_, rampStepPerUpdate_);

  int32_t effectiveTargetLeft = activeLeftTps_;
  int32_t effectiveTargetRight = activeRightTps_;

  // Prevent back-EMF / directional reversal spikes: if active target crosses 0, force PWM to 0 first
  if ((targetLeftTps_ > 0 && activeLeftTps_ < 0) || (targetLeftTps_ < 0 && activeLeftTps_ > 0)) {
    effectiveTargetLeft = 0;
  }
  if ((targetRightTps_ > 0 && activeRightTps_ < 0) || (targetRightTps_ < 0 && activeRightTps_ > 0)) {
    effectiveTargetRight = 0;
  }

  // Laser-Straight Encoder Cross-Coupled Lock
  if (config::motor::TPS_FEEDBACK_ENABLED &&
      activeLeftTps_ != 0 && activeLeftTps_ == activeRightTps_) {
    const int32_t leftTravel = counts.left - syncStartCounts_.left;
    const int32_t rightTravel = counts.right - syncStartCounts_.right;
    const int32_t tickDiff = leftTravel - rightTravel;
    // A cumulative encoder error can be much larger than the small active
    // target during soft-start. The old unbounded correction could therefore
    // cross zero and reverse one wheel even though both requested targets were
    // forward. Limit cross-coupling to 25% of the active speed; the individual
    // PI loops still correct wheel speed without ever changing direction.
    const int32_t activeMagnitude = labs(activeLeftTps_);
    const int32_t maximumSyncCorrection =
        max(1L, activeMagnitude / 4L);
    const int32_t syncCorrection =
        constrain(tickDiff * 2L,
                  -maximumSyncCorrection,
                  maximumSyncCorrection);
    effectiveTargetLeft -= syncCorrection;
    effectiveTargetRight += syncCorrection;

    const int32_t minimumSameDirectionMagnitude =
        max(1L, activeMagnitude / 2L);
    if (activeLeftTps_ > 0) {
      effectiveTargetLeft =
          max(minimumSameDirectionMagnitude, effectiveTargetLeft);
      effectiveTargetRight =
          max(minimumSameDirectionMagnitude, effectiveTargetRight);
    } else {
      effectiveTargetLeft =
          min(-minimumSameDirectionMagnitude, effectiveTargetLeft);
      effectiveTargetRight =
          min(-minimumSameDirectionMagnitude, effectiveTargetRight);
    }
  }

  const float dtSeconds = static_cast<float>(elapsedMs) / 1000.0F;
  if (!config::motor::TPS_FEEDBACK_ENABLED) {
    leftIntegral_ = 0.0F;
    rightIntegral_ = 0.0F;
  }
  const int32_t feedbackLeftTps =
      config::motor::TPS_FEEDBACK_ENABLED
          ? measuredLeftTps_
          : effectiveTargetLeft;
  const int32_t feedbackRightTps =
      config::motor::TPS_FEEDBACK_ENABLED
          ? measuredRightTps_
          : effectiveTargetRight;
  const int16_t requestedLeftPwm =
      calculateControl(effectiveTargetLeft, feedbackLeftTps, dtSeconds,
                       leftIntegral_);
  const int16_t requestedRightPwm =
      calculateControl(effectiveTargetRight, feedbackRightTps, dtSeconds,
                       rightIntegral_);

  appliedLeftPwm_ = slew(appliedLeftPwm_, requestedLeftPwm);
  appliedRightPwm_ = slew(appliedRightPwm_, requestedRightPwm);
  apply();
}

int16_t MotorDriver::calculateControl(const int32_t targetTps,
                                      const int32_t measuredTps,
                                      const float dtSeconds,
                                      float& integral) {
  if (targetTps == 0) {
    integral = 0.0F;
    return 0;
  }

  const float error =
      static_cast<float>(targetTps - measuredTps);
  integral += error * config::motor::SPEED_KI * dtSeconds;
  integral = constrain(integral, -config::motor::INTEGRAL_LIMIT,
                       config::motor::INTEGRAL_LIMIT);

  const int8_t direction = targetTps > 0 ? 1 : -1;
  const float targetFraction =
      min(1.0F, fabs(static_cast<float>(targetTps)) /
                    static_cast<float>(config::motor::MAX_EXPECTED_TPS));
  const float feedForward =
      static_cast<float>(direction) *
      (static_cast<float>(config::motor::MIN_EFFECTIVE_PWM) +
       targetFraction *
           static_cast<float>(config::motor::MAX_DRIVE_PWM -
                              config::motor::MIN_EFFECTIVE_PWM));
  float output =
      feedForward + config::motor::SPEED_KP * error + integral;

  // Prevent H-Bridge directional flipping during speed regulation
  if (targetTps > 0 && output < 0.0F) {
    output = 0.0F;
  } else if (targetTps < 0 && output > 0.0F) {
    output = 0.0F;
  }

  return static_cast<int16_t>(
      constrain(output,
                -static_cast<float>(config::motor::MAX_DRIVE_PWM),
                static_cast<float>(config::motor::MAX_DRIVE_PWM)));
}

void MotorDriver::markPosition() {
  markedCounts_ = encoders_.read();
}

uint32_t MotorDriver::averageAbsoluteTicksSinceMark() const {
  const EncoderCounts counts = encoders_.read();
  const uint32_t leftTravel =
      static_cast<uint32_t>(labs(counts.left - markedCounts_.left));
  const uint32_t rightTravel =
      static_cast<uint32_t>(labs(counts.right - markedCounts_.right));
  return (leftTravel + rightTravel) / 2UL;
}

bool MotorDriver::stalled(const uint32_t nowMs) const {
  if (!config::motor::TPS_FEEDBACK_ENABLED) {
    return false;
  }
  if (targetLeftTps_ == 0 && targetRightTps_ == 0) {
    return false;
  }
  const bool requestingMotion =
      abs(appliedLeftPwm_) >= config::motor::STALL_MIN_PWM ||
      abs(appliedRightPwm_) >= config::motor::STALL_MIN_PWM;
  return requestingMotion &&
         (nowMs - lastMotionMs_ >= config::motor::STALL_TIMEOUT_MS);
}

bool MotorDriver::wheelImbalanceStalled(const uint32_t nowMs) {
  if (!config::motor::TPS_FEEDBACK_ENABLED) {
    wheelImbalanceStartedMs_ = 0;
    return false;
  }
  const int32_t leftTarget = labs(targetLeftTps_);
  const int32_t rightTarget = labs(targetRightTps_);
  const bool sameDirection =
      (targetLeftTps_ > 0 && targetRightTps_ > 0) ||
      (targetLeftTps_ < 0 && targetRightTps_ < 0);
  const bool bothWheelsCommanded =
      sameDirection &&
      leftTarget >= config::motor::WHEEL_JAM_MIN_TARGET_TPS &&
      rightTarget >= config::motor::WHEEL_JAM_MIN_TARGET_TPS &&
      abs(appliedLeftPwm_) >= config::motor::STALL_MIN_PWM &&
      abs(appliedRightPwm_) >= config::motor::STALL_MIN_PWM;
  if (!bothWheelsCommanded) {
    wheelImbalanceStartedMs_ = 0;
    return false;
  }

  const int32_t leftMeasured = labs(measuredLeftTps_);
  const int32_t rightMeasured = labs(measuredRightTps_);
  const bool leftNearlyStopped =
      leftMeasured * 100L <
      leftTarget *
          config::motor::WHEEL_JAM_LOW_RESPONSE_PERCENT;
  const bool rightNearlyStopped =
      rightMeasured * 100L <
      rightTarget *
          config::motor::WHEEL_JAM_LOW_RESPONSE_PERCENT;
  const bool leftStillMoving =
      leftMeasured * 100L >=
      leftTarget *
          config::motor::WHEEL_JAM_MOVING_RESPONSE_PERCENT;
  const bool rightStillMoving =
      rightMeasured * 100L >=
      rightTarget *
          config::motor::WHEEL_JAM_MOVING_RESPONSE_PERCENT;
  const bool asymmetricJam =
      (leftNearlyStopped && rightStillMoving) ||
      (rightNearlyStopped && leftStillMoving);
  if (!asymmetricJam) {
    wheelImbalanceStartedMs_ = 0;
    return false;
  }

  if (wheelImbalanceStartedMs_ == 0) {
    wheelImbalanceStartedMs_ = nowMs;
    return false;
  }
  return nowMs - wheelImbalanceStartedMs_ >=
         config::motor::WHEEL_JAM_TIMEOUT_MS;
}

void MotorDriver::applyChannel(const uint8_t pwmChannel,
                               const uint8_t in1Pin,
                               const uint8_t in2Pin,
                               const int16_t rawCommand,
                               const bool inverted) {
  int16_t command = clampCommand(rawCommand);
  if (inverted) {
    command = -command;
  }

  if (command > 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    ledcWrite(pwmChannel, static_cast<uint8_t>(command));
  } else if (command < 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    ledcWrite(pwmChannel, static_cast<uint8_t>(-command));
  } else {
    ledcWrite(pwmChannel, 0);
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
  }
}

void MotorDriver::apply() {
  if (!ledcReady_) {
    digitalWrite(config::pin::MOTOR_LEFT_IN1, LOW);
    digitalWrite(config::pin::MOTOR_LEFT_IN2, LOW);
    digitalWrite(config::pin::MOTOR_RIGHT_IN1, LOW);
    digitalWrite(config::pin::MOTOR_RIGHT_IN2, LOW);
    return;
  }

  applyChannel(config::motor::PWM_LEFT_CHANNEL,
               config::pin::MOTOR_LEFT_IN1,
               config::pin::MOTOR_LEFT_IN2, appliedLeftPwm_,
               config::motor::LEFT_INVERTED);
  applyChannel(config::motor::PWM_RIGHT_CHANNEL,
               config::pin::MOTOR_RIGHT_IN1,
               config::pin::MOTOR_RIGHT_IN2, appliedRightPwm_,
               config::motor::RIGHT_INVERTED);
}
