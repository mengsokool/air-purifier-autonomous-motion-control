#pragma once

#include <Arduino.h>
#include <ESP32Servo.h>

#include "AsyncUltrasonic.h"
#include "BatteryMonitor.h"
#include "BatterySocEstimator.h"
#include "BatteryChargeManager.h"
#include "ChargeMonitor.h"
#include "CorridorCommitment.h"
#include "FanController.h"
#include "MotorDriver.h"
#include "Pms3003.h"
#include "RobotDisplay.h"
#include "RobotTypes.h"
#include "StablePathPlanner.h"

class RobotController {
 public:
  RobotController();

  void begin();
  void update();

 private:
  MotorDriver motors_;
  AsyncUltrasonic sonar_;
  Servo scannerServo_;
  Pms3003 pms_;
  FanController fan_;
  BatteryMonitor battery_;
  BatterySocEstimator batterySoc_;
  ChargeMonitor charge_;
  BatteryChargeManager chargeManager_;
  RobotDisplay display_;
  bool chargerPresent_ = false;

  RobotState state_ = RobotState::STARTUP;
  uint32_t stateEnteredMs_ = 0;
  uint32_t lastDrivePingMs_ = 0;
  uint32_t avoidWindowStartedMs_ = 0;
  uint8_t avoidEvents_ = 0;
  bool forceEscape_ = false;
  uint16_t frontDistanceCm_ = config::sonar::MAX_VALID_CM;
  bool frontDistanceValid_ = false;
  uint32_t frontDistanceUpdatedMs_ = 0;

  uint16_t leftClearanceCm_ = config::sonar::MAX_VALID_CM;
  uint16_t centerClearanceCm_ = config::sonar::MAX_VALID_CM;
  uint16_t rightClearanceCm_ = config::sonar::MAX_VALID_CM;
  uint32_t centerClearanceUpdatedMs_ = 0;
  uint8_t forwardThreatHitCount_ = 0;
  uint8_t forwardThreatLastAngleDeg_ = config::servo::CENTER_DEG;
  uint32_t forwardThreatLastHitMs_ = 0;
  uint32_t forwardThreatConfirmedMs_ = 0;
  uint16_t latestLeftRayCm_ = config::sonar::MAX_VALID_CM;
  uint16_t latestRightRayCm_ = config::sonar::MAX_VALID_CM;
  uint32_t latestLeftRayMs_ = 0;
  uint32_t latestRightRayMs_ = 0;
  StablePathPlanner pathPlanner_;
  CorridorCommitment corridorCommitment_;
  int16_t wheelSteeringPercent_ = 0;
  uint32_t lastSteeringUpdateMs_ = 0;
  bool rollingFrontAvoidActive_ = false;
  // -1 = committed physical-left escape, +1 = physical-right escape.
  // Keep this stable for the complete avoidance manoeuvre so individual
  // ultrasonic echoes cannot reverse steering every scan.
  int8_t rollingAvoidSteerSign_ = 0;
  bool rollingAvoidDirectionCorrectionUsed_ = false;
  uint32_t rollingAvoidStartedMs_ = 0;
  uint32_t rollingAvoidReleaseStartedMs_ = 0;
  uint32_t straightRecoveryUntilMs_ = 0;
  uint32_t rollingAvoidLastProgressMs_ = 0;
  uint16_t rollingAvoidBestClearanceCm_ = 0;
  uint32_t lastEscapeCompletedMs_ = 0;
  uint8_t escapeAttemptCount_ = 0;
  bool straightAssistActive_ = false;
  bool sideAvoidActive_ = false;
  int8_t sideAvoidSteerSign_ = 0;
  uint16_t sideAvoidDistanceCm_ = config::sonar::MAX_VALID_CM;
  uint16_t sideAvoidReleaseCm_ = config::sonar::SIDE_AVOID_RELEASE_CM;
  uint32_t sideAvoidLastSeenMs_ = 0;
  uint32_t sideAvoidStartedMs_ = 0;
  uint32_t sideAvoidLastProgressMs_ = 0;
  uint16_t sideAvoidBestDistanceCm_ = 0;
  bool radarStuckWatchActive_ = false;
  int8_t radarStuckWatchSteerSign_ = 0;
  uint32_t radarStuckWatchStartedMs_ = 0;
  EncoderCounts radarStuckStartCounts_{0, 0};
  uint16_t radarStuckBaselineCm_[
      config::sonar::RADAR_STUCK_ANCHOR_COUNT]{};
  uint32_t radarStuckBaselineUpdatedMs_[
      config::sonar::RADAR_STUCK_ANCHOR_COUNT]{};
  uint8_t failedAngleCenterDeg_[
      config::sonar::FAILED_ANGLE_SECTOR_COUNT]{};
  uint8_t failedAngleRadiusDeg_[
      config::sonar::FAILED_ANGLE_SECTOR_COUNT]{};
  uint32_t failedAngleUpdatedMs_[
      config::sonar::FAILED_ANGLE_SECTOR_COUNT]{};

  // One entry for each actual 5-degree radar ray.
  uint16_t spatialMap_[config::sonar::RADAR_POINT_COUNT]{};
  uint32_t spatialMapUpdatedMs_[config::sonar::RADAR_POINT_COUNT]{};

  void updateRadarTask(uint32_t nowMs);
  uint16_t getSpatialClearance(uint8_t minAngle, uint8_t maxAngle) const;
  uint16_t getSpatialMedianClearance(uint8_t minAngle,
                                     uint8_t maxAngle) const;
  GapSelection selectLiveGap(
      uint32_t nowMs,
      bool penalizeFailedAngleSectors = false,
      uint8_t corridorPoints =
          config::sonar::RADAR_CORRIDOR_POINTS,
      bool allowNarrowFallback = true) const;
  bool frontReadingFresh(uint32_t nowMs) const;
  void rememberFailedAngle(uint32_t nowMs);

  uint16_t scanDistances_[config::sonar::SCAN_POINT_COUNT]{};
  bool scanValid_[config::sonar::SCAN_POINT_COUNT]{};

  uint8_t driveSweepAngle_ = config::servo::CENTER_DEG;
  uint8_t discreteAngleIdx_ = config::sonar::RADAR_CENTER_INDEX;
  int8_t discreteAngleDir_ = 1;        // +1 or -1 for stepping sequence
  uint8_t escapeScanStep_ = 0;
  bool radarThreatScanActive_ = false;
  uint8_t threatSequenceIdx_ = 0;
  uint8_t threatFocusAngleDeg_ = config::servo::CENTER_DEG;
  uint16_t threatFocusDistanceCm_ = config::sonar::MAX_VALID_CM;
  bool radarAnomalyFocusActive_ = false;
  uint8_t radarAnomalyFocusAngleDeg_ = config::servo::CENTER_DEG;
  uint8_t discreteSampleCount_ = 0;
  uint8_t discreteInvalidCount_ = 0;
  uint16_t discreteSampleBuf_[config::sonar::RADAR_SAMPLE_BUFFER_SIZE]{};
  uint8_t discreteClearMisses_[config::sonar::RADAR_POINT_COUNT]{};
  uint32_t radarCoverageMask_ = 0;
  uint8_t radarRevision_ = 0;
  uint8_t plannerRevision_ = UINT8_MAX;
  uint8_t currentServoAngle_ = config::servo::CENTER_DEG;
  uint32_t lastServoStepMs_ = 0;
  uint16_t servoSettleDurationMs_ = 0;

  uint8_t selectedAngleDeg_ = config::servo::CENTER_DEG;
  uint8_t livePathAngleDeg_ = config::servo::CENTER_DEG;
  TurnDirection selectedTurn_ = TurnDirection::NONE;
  uint32_t selectedTurnTargetTicks_ = 0;
  TurnDirection escapeTurn_ = TurnDirection::RIGHT;
  uint32_t escapeTurnTargetTicks_ = 0;

  uint8_t faultRetryCount_ = 0;
  uint32_t lastFaultMs_ = 0;

  void transitionTo(RobotState next, uint32_t nowMs);
  void handleStartup(uint32_t nowMs);
  void handleDrive(uint32_t nowMs);
  void handleBrake(uint32_t nowMs);
  void handleScan(uint32_t nowMs);
  void handleTurn(uint32_t nowMs);
  void handleEscapeReverse(uint32_t nowMs);
  void handleEscapeScan(uint32_t nowMs);
  void handleEscapeTurn(uint32_t nowMs);
  void handleFault(uint32_t nowMs);
  void handleLowBattery();
  void prepareSafeStartup(uint32_t nowMs);

  void beginScan(uint32_t nowMs);
  void finishScan(uint32_t nowMs);
  void recordAvoidEvent(uint32_t nowMs);
  static uint16_t medianSamples(const uint16_t* values, uint8_t count);
  uint32_t lastStatusReportMs_ = 0;
  void updateDisplay(uint32_t nowMs);
  void reportStatus(uint32_t nowMs);
  bool inaBatteryVoltageValid() const;
  bool selectedBatteryVoltageValid() const;
  float selectedBatteryVoltage() const;
};
