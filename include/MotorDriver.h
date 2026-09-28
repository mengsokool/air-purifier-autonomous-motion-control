#pragma once

#include <Arduino.h>

#include "QuadratureEncoders.h"

class MotorDriver {
 public:
  void begin();
  void update(uint32_t nowMs);

  void driveForward(int32_t ticksPerSecond);
  void driveReverse(int32_t ticksPerSecond);
  void driveDifferential(int32_t leftTps, int32_t rightTps);
  void turnLeft(int32_t ticksPerSecond);
  void turnRight(int32_t ticksPerSecond);
  void stop();
  void emergencyStop();
  void reset();
  void setSoftStartDuration(float durationSeconds);

  int32_t leftTargetTps() const { return targetLeftTps_; }
  int32_t rightTargetTps() const { return targetRightTps_; }
  int32_t leftMeasuredTps() const { return measuredLeftTps_; }
  int32_t rightMeasuredTps() const { return measuredRightTps_; }
  int16_t leftAppliedPwm() const { return appliedLeftPwm_; }
  int16_t rightAppliedPwm() const { return appliedRightPwm_; }
  EncoderCounts encoderCounts() const { return encoders_.read(); }

  void markPosition();
  uint32_t averageAbsoluteTicksSinceMark() const;
  bool stalled(uint32_t nowMs) const;
  bool wheelImbalanceStalled(uint32_t nowMs);

 private:
  QuadratureEncoders encoders_;
  int32_t targetLeftTps_ = 0;
  int32_t targetRightTps_ = 0;
  int32_t activeLeftTps_ = 0;
  int32_t activeRightTps_ = 0;
  int32_t rampStepPerUpdate_ = 120;
  int32_t measuredLeftTps_ = 0;
  int32_t measuredRightTps_ = 0;
  int16_t appliedLeftPwm_ = 0;
  int16_t appliedRightPwm_ = 0;
  float leftIntegral_ = 0.0F;
  float rightIntegral_ = 0.0F;
  uint32_t lastControlMs_ = 0;
  uint32_t lastMotionMs_ = 0;
  uint32_t wheelImbalanceStartedMs_ = 0;
  EncoderCounts lastControlCounts_{0, 0};
  EncoderCounts markedCounts_{0, 0};
  EncoderCounts syncStartCounts_{0, 0};

  static int16_t slew(int16_t current, int16_t target);
  static int16_t calculateControl(int32_t targetTps,
                                  int32_t measuredTps,
                                  float dtSeconds,
                                  float& integral);
  static void applyChannel(uint8_t pwmChannel,
                           uint8_t in1Pin,
                           uint8_t in2Pin,
                           int16_t command,
                           bool inverted);
  void apply();
  bool ledcReady_ = false;
};
