#include "RobotController.h"

#include <math.h>

#include "AdaptiveAvoidance.h"
#include "Config.h"
#include "DifferentialSteering.h"
#include "GapPlanner.h"
#include "MotionGeometry.h"
#include "RadarFilter.h"
#include "RadarMotionVerifier.h"

namespace {
HardwareSerial pmsSerial(config::serial::PMS_UART_NUMBER);
}

RobotController::RobotController()
    : pms_(pmsSerial) {}

bool RobotController::inaBatteryVoltageValid() const {
  const float voltage = charge_.busVoltage();
  return charge_.hasReading() && isfinite(voltage) &&
         voltage >= config::battery::VALID_MIN_V &&
         voltage <= config::battery::VALID_MAX_V;
}

bool RobotController::selectedBatteryVoltageValid() const {
  return inaBatteryVoltageValid() || battery_.hasReading();
}

float RobotController::selectedBatteryVoltage() const {
  return inaBatteryVoltageValid() ? charge_.busVoltage()
                                  : battery_.voltage();
}

void RobotController::begin() {
  Serial.begin(config::logging::SERIAL_BAUD);
  pmsSerial.begin(config::pms::BAUD, SERIAL_8N1, config::pin::PMS_RX,
                  config::pin::PMS_TX);

  motors_.begin();
  sonar_.begin();
  // Keep ESP32Servo on timer 0 (channels 0/1). Motor PWM owns timer 1
  // (channels 2/3), and fan PWM owns timer 3 (channel 7).
  ESP32PWM::allocateTimer(0);
  scannerServo_.setPeriodHertz(50);
  scannerServo_.attach(config::pin::SCANNER_SERVO,
                       config::servo::MIN_PULSE_US,
                       config::servo::MAX_PULSE_US);
  scannerServo_.write(config::servo::CENTER_DEG);
  pms_.begin();
  fan_.begin();
  battery_.begin();
  display_.begin();
  charge_.begin();
  chargeManager_.begin();
  batterySoc_.begin();
  // The external 33 kOhm divider leg holds this LOW while no adapter is present.
  pinMode(config::pin::CHARGER_PRESENT, INPUT);
  stateEnteredMs_ = millis();
  avoidWindowStartedMs_ = stateEnteredMs_;
  fan_.startStartupDustShake(stateEnteredMs_);

}

void RobotController::updateRadarTask(const uint32_t nowMs) {
  const uint32_t fullRadarCoverageMask =
      (1UL << config::sonar::RADAR_POINT_COUNT) - 1UL;
  const bool startupCoverageComplete =
      state_ == RobotState::STARTUP &&
      radarCoverageMask_ == fullRadarCoverageMask;
  const bool chassisTurning =
      state_ == RobotState::TURN ||
      state_ == RobotState::ESCAPE_TURN;
  const bool fastEscapeScan =
      state_ == RobotState::ESCAPE_SCAN;
  const bool threatScanRequested =
      state_ == RobotState::DRIVE &&
      rollingFrontAvoidActive_;
  if (state_ != RobotState::DRIVE &&
      radarAnomalyFocusActive_) {
    radarAnomalyFocusActive_ = false;
    discreteSampleCount_ = 0;
    discreteInvalidCount_ = 0;
  }
  if (threatScanRequested != radarThreatScanActive_) {
    radarThreatScanActive_ = threatScanRequested;
    radarAnomalyFocusActive_ = false;
    threatSequenceIdx_ = 0;
    discreteSampleCount_ = 0;
    discreteInvalidCount_ = 0;
  }

  uint8_t targetAngle = config::servo::CENTER_DEG;
  if (startupCoverageComplete || chassisTurning) {
    // A startup sweep is useful only if the sensor returns to the forward
    // direction before the chassis is allowed to move. While the chassis is
    // turning, keep looking forward instead of mixing rotating coordinate
    // frames into the panoramic map.
    targetAngle = config::servo::CENTER_DEG;
  } else if (radarThreatScanActive_) {
    const bool physicalLeftSelected = rollingAvoidSteerSign_ < 0;
    const bool lowServoAnglesArePhysicalLeft =
        config::servo::ANGLE_TO_TURN_SIGN > 0;
    const bool scanLowAngles =
        physicalLeftSelected == lowServoAnglesArePhysicalLeft;
    const uint8_t selectedNearAngle = scanLowAngles ? 60U : 120U;
    const uint8_t selectedFarAngle = scanLowAngles ? 45U : 135U;
    const uint8_t oppositeNearAngle = scanLowAngles ? 120U : 60U;
    const uint8_t oppositeFarAngle = scanLowAngles ? 135U : 45U;
    switch (threatSequenceIdx_) {
      case 0:
        targetAngle = config::servo::CENTER_DEG;
        break;
      case 1:
        targetAngle = 85;
        break;
      case 2:
        targetAngle = 95;
        break;
      case 3:
        targetAngle = selectedNearAngle;
        break;
      case 4:
        targetAngle = selectedFarAngle;
        break;
      case 5:
        targetAngle = oppositeFarAngle;
        break;
      default:
        targetAngle = oppositeNearAngle;
        break;
    }
  } else if (fastEscapeScan) {
    // Scan symmetrically outwards from 90 degrees:
    // 90, 85, 95, 80, 100 ... This sees both possible exits before
    // committing and reaches a usable seven-ray corridor much sooner than
    // sweeping all the way to one side first.
    const uint8_t offset =
        static_cast<uint8_t>((escapeScanStep_ + 1U) / 2U);
    const int16_t signedIndex =
        escapeScanStep_ == 0
            ? config::sonar::RADAR_CENTER_INDEX
            : (escapeScanStep_ & 1U)
                  ? static_cast<int16_t>(
                        config::sonar::RADAR_CENTER_INDEX) -
                        offset
                  : static_cast<int16_t>(
                        config::sonar::RADAR_CENTER_INDEX) +
                        offset;
    targetAngle = config::sonar::radarAngleAt(
        static_cast<uint8_t>(signedIndex));
  } else if (radarAnomalyFocusActive_) {
    // A suspicious new close return gets a short stationary stare. The
    // normal sweep cursor remains unchanged and resumes after the median.
    targetAngle = radarAnomalyFocusAngleDeg_;
  } else {
    targetAngle = config::sonar::radarAngleAt(discreteAngleIdx_);
  }
  const uint8_t mapAngleIdx =
      static_cast<uint8_t>(
          (targetAngle - config::sonar::RADAR_MIN_DEG) /
          config::sonar::RADAR_STEP_DEG);
  const uint8_t requiredSamples =
      chassisTurning
          ? 1U
      : radarAnomalyFocusActive_
          ? config::sonar::ANOMALY_FOCUS_SAMPLES
      : state_ == RobotState::STARTUP
          ? config::sonar::STARTUP_SAMPLES_PER_ANGLE
      : fastEscapeScan
          ? config::sonar::ESCAPE_SAMPLES_PER_ANGLE
          : radarThreatScanActive_ &&
              targetAngle != threatFocusAngleDeg_
          ? config::sonar::THREAT_SIDE_SAMPLES_PER_ANGLE
          : config::sonar::RADAR_SAMPLES_PER_ANGLE;

  // Send one destination to the servo and let its internal controller move
  // smoothly. Repeated one-degree commands made the mechanism visibly jerk.
  if (currentServoAngle_ != targetAngle) {
    const uint8_t travelDegrees =
        currentServoAngle_ < targetAngle
            ? static_cast<uint8_t>(targetAngle - currentServoAngle_)
            : static_cast<uint8_t>(currentServoAngle_ - targetAngle);
    currentServoAngle_ = targetAngle;
    driveSweepAngle_ = targetAngle;
    scannerServo_.write(targetAngle);
    lastServoStepMs_ = nowMs;
    servoSettleDurationMs_ =
        static_cast<uint16_t>(
            config::sonar::RADAR_SERVO_SETTLE_MS +
            static_cast<uint16_t>(travelDegrees) *
                config::sonar::SERVO_TRAVEL_MS_PER_DEG);
    discreteSampleCount_ = 0;
    discreteInvalidCount_ = 0;
    // A ping started at the previous angle may finish while the servo moves.
    // Discard it instead of later attributing that echo to the new angle.
    uint16_t discardedCm = 0;
    bool discardedValid = false;
    sonar_.takeResult(discardedCm, discardedValid);
    return;
  }

  // The commanded angle has arrived, but the physical horn and sensor bracket
  // still need time to stop vibrating. Pinging during that movement produces
  // repeatable short reflections from the floor or chassis.
  if (nowMs - lastServoStepMs_ < servoSettleDurationMs_) {
    // Also drain a late result during the mechanical settling window. By the
    // time this window ends, the next accepted ping is guaranteed to have
    // been transmitted and received at the current target angle.
    uint16_t discardedCm = 0;
    bool discardedValid = false;
    sonar_.takeResult(discardedCm, discardedValid);
    return;
  }

  // Fire only after the previous acoustic energy has had time to decay.
  if (!sonar_.busy() &&
      nowMs - lastDrivePingMs_ >= config::sonar::DRIVE_SAMPLE_MS) {
    if (sonar_.request()) {
      lastDrivePingMs_ = nowMs;
    }
  }

  // 3. Collect samples at the target angle and compute a stable median.
  uint16_t measuredCm = 0;
  bool valid = false;
  if (sonar_.takeResult(measuredCm, valid)) {
    if (valid && measuredCm >= config::sonar::MIN_VALID_CM && measuredCm <= config::sonar::MAX_VALID_CM) {
      discreteInvalidCount_ = 0;
      if (discreteSampleCount_ < requiredSamples) {
        discreteSampleBuf_[discreteSampleCount_++] = measuredCm;
      }
    } else {
      // An open ray commonly produces no echo. The old implementation waited
      // forever at that angle, which made the sweep appear random and biased.
      // Confirm three consecutive misses, then record maximum clearance.
      const uint8_t invalidLimit =
          radarAnomalyFocusActive_
              ? config::sonar::FOCUS_INVALID_LIMIT
              : config::sonar::NORMAL_INVALID_LIMIT;
      if (discreteInvalidCount_ < invalidLimit) {
        ++discreteInvalidCount_;
      }
      if (discreteInvalidCount_ >= invalidLimit) {
        for (uint8_t i = 0;
             i < requiredSamples; ++i) {
          discreteSampleBuf_[i] = config::sonar::MAX_VALID_CM;
        }
        discreteSampleCount_ = requiredSamples;
        discreteInvalidCount_ = 0;
      }
    }

    if (discreteSampleCount_ >= requiredSamples) {
      const uint16_t cleanCm =
          requiredSamples == 2
              ? min(discreteSampleBuf_[0], discreteSampleBuf_[1])
              : medianSamples(discreteSampleBuf_, requiredSamples);
      const uint16_t previousCm = spatialMap_[mapAngleIdx];
      const bool previousFresh =
          previousCm > 0 &&
          nowMs - spatialMapUpdatedMs_[mapAngleIdx] <=
              config::sonar::SPATIAL_FRESH_MS;

      const bool normalDrivePan =
          state_ == RobotState::DRIVE &&
          !radarThreatScanActive_ &&
          !radarAnomalyFocusActive_;
      if (normalDrivePan &&
          RadarFilter::shouldFocus(
              previousCm, cleanCm, previousFresh,
              config::sonar::ANOMALY_FOCUS_MAX_CM,
              config::sonar::ANOMALY_DROP_CM)) {
        radarAnomalyFocusActive_ = true;
        radarAnomalyFocusAngleDeg_ = targetAngle;
        // Preserve the triggering measurement as sample one of the median.
        discreteSampleBuf_[0] = cleanCm;
        discreteSampleCount_ = 1;
        discreteInvalidCount_ = 0;
        return;
      }
      const bool completedAnomalyFocus =
          radarAnomalyFocusActive_;
      if (completedAnomalyFocus) {
        radarAnomalyFocusActive_ = false;
      }

      // A narrow object such as a table leg can disappear between adjacent
      // beams. Hold a previously close return until two consecutive visits
      // confirm that the ray is clear.
      const int16_t activeSteerPercent =
          pathPlanner_.steeringPercent();
      const bool robotTurning =
          activeSteerPercent >=
                  config::sonar::TURNING_STEER_THRESHOLD_PERCENT ||
          activeSteerPercent <=
                  -config::sonar::TURNING_STEER_THRESHOLD_PERCENT;
      if (!robotTurning &&
          previousFresh && cleanCm > previousCm &&
          discreteClearMisses_[mapAngleIdx] + 1U <
              config::sonar::CLEAR_CONFIRM_VISITS) {
        ++discreteClearMisses_[mapAngleIdx];
        spatialMapUpdatedMs_[mapAngleIdx] = nowMs;
      } else {
        spatialMap_[mapAngleIdx] = cleanCm;
        spatialMapUpdatedMs_[mapAngleIdx] = nowMs;
        discreteClearMisses_[mapAngleIdx] = 0;
      }
      radarCoverageMask_ |=
          static_cast<uint32_t>(1UL << mapAngleIdx);
      ++radarRevision_;

      if (targetAngle >= config::sonar::FORWARD_MIN_DEG &&
          targetAngle <= config::sonar::FORWARD_MAX_DEG) {
        if (cleanCm <= config::sonar::FRONT_AVOID_CM) {
          const uint8_t angleDifference =
              static_cast<uint8_t>(
                  targetAngle >= forwardThreatLastAngleDeg_
                      ? targetAngle - forwardThreatLastAngleDeg_
                      : forwardThreatLastAngleDeg_ - targetAngle);
          const bool corroboratesPreviousRay =
              forwardThreatLastHitMs_ != 0 &&
              nowMs - forwardThreatLastHitMs_ <=
                  config::sonar::THREAT_CONFIRM_WINDOW_MS &&
              // A repeat at the same angle can repeat the same false echo.
              // Confirmation must come from a neighbouring physical ray.
              angleDifference >= config::sonar::RADAR_STEP_DEG &&
              angleDifference <=
                  config::sonar::RADAR_STEP_DEG * 2U;
          forwardThreatHitCount_ =
              corroboratesPreviousRay
                  ? static_cast<uint8_t>(
                        min(3, forwardThreatHitCount_ + 1))
                  : 1;
          forwardThreatLastAngleDeg_ = targetAngle;
          forwardThreatLastHitMs_ = nowMs;
          const bool veryCloseCentralRay =
              targetAngle >= 85 && targetAngle <= 95 &&
              cleanCm <= config::sonar::THREAT_SINGLE_RAY_CM;
          if (forwardThreatHitCount_ >= 2 ||
              veryCloseCentralRay) {
            forwardThreatConfirmedMs_ = nowMs;
          }
        } else {
          forwardThreatHitCount_ = 0;
        }
      }

      const bool lowSideRay = targetAngle <= 70;
      const bool highSideRay = targetAngle >= 110;
      if (lowSideRay || highSideRay) {
        const bool rayIsPhysicalLeft =
            config::servo::ANGLE_TO_TURN_SIGN > 0
                ? lowSideRay
                : highSideRay;
        if (rayIsPhysicalLeft) {
          latestLeftRayCm_ = cleanCm;
          latestLeftRayMs_ = nowMs;
        } else {
          latestRightRayCm_ = cleanCm;
          latestRightRayMs_ = nowMs;
        }
      }

      if (cleanCm <= config::sonar::FRONT_AVOID_CM &&
          (cleanCm < threatFocusDistanceCm_ ||
           targetAngle == threatFocusAngleDeg_)) {
        threatFocusAngleDeg_ = targetAngle;
        threatFocusDistanceCm_ = cleanCm;
      } else if (targetAngle == threatFocusAngleDeg_ &&
                 cleanCm >=
                     config::sonar::FRONT_AVOID_RELEASE_CM) {
        threatFocusAngleDeg_ = config::servo::CENTER_DEG;
        threatFocusDistanceCm_ = config::sonar::MAX_VALID_CM;
      }

      if (targetAngle == 90) {
        frontDistanceCm_ = spatialMap_[mapAngleIdx];
        frontDistanceValid_ = true;
        frontDistanceUpdatedMs_ = nowMs;
      }

      // Convert low/high servo angles into physical robot-left/right using
      // the single calibrated servo orientation flag.
      const uint16_t lowAngleClearance =
          getSpatialClearance(15, 70);
      const uint16_t highAngleClearance =
          getSpatialClearance(110, 165);
      if (config::servo::ANGLE_TO_TURN_SIGN > 0) {
        leftClearanceCm_ = lowAngleClearance;
        rightClearanceCm_ = highAngleClearance;
      } else {
        leftClearanceCm_ = highAngleClearance;
        rightClearanceCm_ = lowAngleClearance;
      }
      centerClearanceCm_ =
          getSpatialMedianClearance(config::sonar::FORWARD_MIN_DEG,
                                    config::sonar::FORWARD_MAX_DEG);
      if (targetAngle >= config::sonar::FORWARD_MIN_DEG &&
          targetAngle <= config::sonar::FORWARD_MAX_DEG) {
        centerClearanceUpdatedMs_ = nowMs;
      }

      if (chassisTurning) {
        // Keep the normal sweep cursor unchanged. It resumes from the same
        // place after the turn, while centre readings remain fresh.
      } else if (fastEscapeScan) {
        ++escapeScanStep_;
        if (escapeScanStep_ >=
            config::sonar::RADAR_POINT_COUNT) {
          escapeScanStep_ = 0;
        }
      } else if (radarThreatScanActive_) {
        ++threatSequenceIdx_;
        if (threatSequenceIdx_ >=
            config::sonar::THREAT_SEQUENCE_COUNT) {
          threatSequenceIdx_ = 0;
        }
      } else if (discreteAngleDir_ > 0) {
        if (discreteAngleIdx_ + 1U <
            config::sonar::RADAR_POINT_COUNT) {
          ++discreteAngleIdx_;
        } else {
          discreteAngleDir_ = -1;
          --discreteAngleIdx_;
        }
      } else {
        if (discreteAngleIdx_ > 0) {
          --discreteAngleIdx_;
        } else {
          discreteAngleDir_ = 1;
          ++discreteAngleIdx_;
        }
      }

      discreteSampleCount_ = 0;
      discreteInvalidCount_ = 0;
    }
  }
}

uint16_t RobotController::getSpatialClearance(const uint8_t minAngle, const uint8_t maxAngle) const {
  uint16_t minimum = config::sonar::MAX_VALID_CM;
  bool found = false;
  const uint32_t nowMs = millis();
  const int16_t activeSteerPercent = pathPlanner_.steeringPercent();
  const uint16_t freshLimitMs =
      activeSteerPercent >=
                  config::sonar::TURNING_STEER_THRESHOLD_PERCENT ||
              activeSteerPercent <=
                  -config::sonar::TURNING_STEER_THRESHOLD_PERCENT
          ? config::sonar::TURNING_SPATIAL_FRESH_MS
          : config::sonar::SPATIAL_FRESH_MS;
  for (uint8_t i = 0; i < config::sonar::RADAR_POINT_COUNT; ++i) {
    const uint8_t angle = config::sonar::radarAngleAt(i);
    if (angle < minAngle || angle > maxAngle) {
      continue;
    }
    if (spatialMap_[i] > 0 &&
        spatialMapUpdatedMs_[i] != 0 &&
        (nowMs - spatialMapUpdatedMs_[i] <=
         freshLimitMs)) {
      if (spatialMap_[i] < minimum) {
        minimum = spatialMap_[i];
      }
      found = true;
    }
  }
  if (!found) {
    return config::sonar::MAX_VALID_CM;
  }
  return minimum;
}

uint16_t RobotController::getSpatialMedianClearance(
    const uint8_t minAngle,
    const uint8_t maxAngle) const {
  uint16_t values[config::sonar::SAMPLES_PER_ANGLE]{};
  uint8_t count = 0;
  const uint32_t nowMs = millis();
  const int16_t activeSteerPercent = pathPlanner_.steeringPercent();
  const uint16_t freshLimitMs =
      activeSteerPercent >=
                  config::sonar::TURNING_STEER_THRESHOLD_PERCENT ||
              activeSteerPercent <=
                  -config::sonar::TURNING_STEER_THRESHOLD_PERCENT
          ? config::sonar::TURNING_SPATIAL_FRESH_MS
          : config::sonar::SPATIAL_FRESH_MS;

  for (uint8_t i = 0; i < config::sonar::RADAR_POINT_COUNT; ++i) {
    const uint8_t angle = config::sonar::radarAngleAt(i);
    if (angle < minAngle || angle > maxAngle ||
        spatialMap_[i] == 0 || spatialMapUpdatedMs_[i] == 0 ||
        nowMs - spatialMapUpdatedMs_[i] > freshLimitMs) {
      continue;
    }
    if (count < config::sonar::SAMPLES_PER_ANGLE) {
      values[count++] = spatialMap_[i];
    }
  }

  if (count == 0) {
    return config::sonar::MAX_VALID_CM;
  }
  // For the 85/90/95-degree corridor, two agreeing rays win. A single short
  // reflection can no longer force a turn, while a real wide obstacle still
  // blocks the corridor. The exact 90-degree emergency check remains active.
  return medianSamples(values, count);
}

GapSelection RobotController::selectLiveGap(
    const uint32_t nowMs,
    const bool penalizeFailedAngleSectors,
    const uint8_t corridorPoints,
    const bool allowNarrowFallback) const {
  uint16_t distances[config::sonar::RADAR_POINT_COUNT]{};
  bool valid[config::sonar::RADAR_POINT_COUNT]{};

  const int16_t activeSteerPercent = pathPlanner_.steeringPercent();
  const uint16_t freshLimitMs =
      activeSteerPercent >=
                  config::sonar::TURNING_STEER_THRESHOLD_PERCENT ||
              activeSteerPercent <=
                  -config::sonar::TURNING_STEER_THRESHOLD_PERCENT
          ? config::sonar::TURNING_SPATIAL_FRESH_MS
          : config::sonar::SPATIAL_FRESH_MS;
  for (uint8_t i = 0; i < config::sonar::RADAR_POINT_COUNT; ++i) {
    const uint8_t angle = config::sonar::radarAngleAt(i);
    distances[i] = spatialMap_[i];
    valid[i] =
        distances[i] > 0 &&
        spatialMapUpdatedMs_[i] != 0 &&
        nowMs - spatialMapUpdatedMs_[i] <=
            freshLimitMs;
    if (valid[i] && penalizeFailedAngleSectors) {
      for (uint8_t sector = 0;
           sector < config::sonar::FAILED_ANGLE_SECTOR_COUNT;
           ++sector) {
        if (failedAngleUpdatedMs_[sector] == 0 ||
            nowMs - failedAngleUpdatedMs_[sector] >
                config::sonar::FAILED_ANGLE_MEMORY_MS) {
          continue;
        }
        const uint8_t difference =
            static_cast<uint8_t>(
                angle >= failedAngleCenterDeg_[sector]
                    ? angle - failedAngleCenterDeg_[sector]
                    : failedAngleCenterDeg_[sector] - angle);
        distances[i] =
            GapPlanner::applyDirectionalPenalty(
                distances[i],
                difference,
                failedAngleRadiusDeg_[sector],
                config::sonar::OPEN_SPACE_CM,
                config::sonar::
                    FAILED_ANGLE_CENTER_RETAIN_PERCENT,
                config::sonar::
                    FAILED_ANGLE_EDGE_RETAIN_PERCENT);
      }
    }
  }

  GapSelection best = GapPlanner::selectBestCorridor(
      distances,
      valid,
      config::sonar::RADAR_POINT_COUNT,
      config::sonar::RADAR_MIN_DEG,
      config::sonar::RADAR_STEP_DEG,
      config::servo::CENTER_DEG,
      config::sonar::OPEN_SPACE_CM,
      corridorPoints);
  if (!best.found && allowNarrowFallback) {
    best = GapPlanner::select(
        distances,
        valid,
        config::sonar::RADAR_POINT_COUNT,
        config::sonar::RADAR_MIN_DEG,
        config::sonar::RADAR_STEP_DEG,
        config::servo::CENTER_DEG,
        config::sonar::OPEN_SPACE_CM,
        config::sonar::RADAR_MIN_GAP_POINTS);
  }
  if (!best.found && allowNarrowFallback) {
    // Last-resort rolling escape direction. A single fresh open ray is less
    // desirable than a real corridor, but still better than reversing when
    // the robot has usable space to curve around an obstacle.
    best = GapPlanner::select(
        distances,
        valid,
        config::sonar::RADAR_POINT_COUNT,
        config::sonar::RADAR_MIN_DEG,
        config::sonar::RADAR_STEP_DEG,
        config::servo::CENTER_DEG,
        config::sonar::OPEN_SPACE_CM,
        1);
  }
  return best;
}

bool RobotController::frontReadingFresh(const uint32_t nowMs) const {
  return frontDistanceValid_ &&
         nowMs - frontDistanceUpdatedMs_ <= config::sonar::FRONT_FRESH_MS;
}

void RobotController::rememberFailedAngle(const uint32_t nowMs) {
  uint8_t failedAngle = livePathAngleDeg_;
  const bool recentlyEscaped =
      lastEscapeCompletedMs_ != 0 &&
      nowMs - lastEscapeCompletedMs_ <
          config::motor::ESCAPE_REPEAT_WINDOW_MS;
  if (recentlyEscaped &&
      selectedAngleDeg_ != config::servo::CENTER_DEG) {
    // DRIVE resets the live planner to 90 degrees. If the newly selected
    // escape heading immediately traps the robot again, remember that actual
    // committed heading instead of incorrectly blacklisting straight ahead.
    failedAngle = selectedAngleDeg_;
  }
  if (failedAngle == config::servo::CENTER_DEG &&
      threatFocusAngleDeg_ != config::servo::CENTER_DEG) {
    failedAngle = threatFocusAngleDeg_;
  }
  if (failedAngle == config::servo::CENTER_DEG &&
      wheelSteeringPercent_ != 0) {
    const bool physicalLeft = wheelSteeringPercent_ < 0;
    if (config::servo::ANGLE_TO_TURN_SIGN > 0) {
      failedAngle = physicalLeft ? 60 : 120;
    } else {
      failedAngle = physicalLeft ? 120 : 60;
    }
  }
  if (failedAngle == config::servo::CENTER_DEG) {
    return;
  }

  int8_t selectedSector = -1;
  int8_t replacementSector = -1;
  uint32_t oldestTimestamp = UINT32_MAX;
  for (uint8_t sector = 0;
       sector < config::sonar::FAILED_ANGLE_SECTOR_COUNT;
       ++sector) {
    const bool active =
        failedAngleUpdatedMs_[sector] != 0 &&
        nowMs - failedAngleUpdatedMs_[sector] <=
            config::sonar::FAILED_ANGLE_MEMORY_MS;
    if (!active) {
      if (replacementSector < 0) {
        replacementSector = static_cast<int8_t>(sector);
      }
      continue;
    }
    const uint8_t difference =
        static_cast<uint8_t>(
            failedAngle >= failedAngleCenterDeg_[sector]
                ? failedAngle - failedAngleCenterDeg_[sector]
                : failedAngleCenterDeg_[sector] - failedAngle);
    if (difference <=
        failedAngleRadiusDeg_[sector] +
            config::sonar::RADAR_STEP_DEG * 2U) {
      selectedSector = static_cast<int8_t>(sector);
      break;
    }
    if (failedAngleUpdatedMs_[sector] < oldestTimestamp) {
      oldestTimestamp = failedAngleUpdatedMs_[sector];
      replacementSector = static_cast<int8_t>(sector);
    }
  }

  if (selectedSector < 0) {
    selectedSector = replacementSector >= 0 ? replacementSector : 0;
    failedAngleCenterDeg_[selectedSector] = failedAngle;
    failedAngleRadiusDeg_[selectedSector] =
        config::sonar::FAILED_ANGLE_INITIAL_RADIUS_DEG;
  } else {
    const uint8_t expandedRadius =
        static_cast<uint8_t>(
            failedAngleRadiusDeg_[selectedSector] +
            config::sonar::FAILED_ANGLE_EXPAND_DEG);
    failedAngleRadiusDeg_[selectedSector] =
        min(expandedRadius,
            config::sonar::FAILED_ANGLE_MAX_RADIUS_DEG);
  }
  failedAngleUpdatedMs_[selectedSector] = nowMs;

}

void RobotController::update() {
  const uint32_t nowMs = millis();
  const bool previousChargerPresent = chargerPresent_;
  chargerPresent_ = digitalRead(config::pin::CHARGER_PRESENT) ==
                    (config::charge::PRESENT_ACTIVE_HIGH ? HIGH : LOW);
  if (chargerPresent_ != previousChargerPresent) {
    // A person may move the robot or place an obstacle nearby while charging.
    // Never reuse the spatial map from before an adapter transition.
    prepareSafeStartup(nowMs);
  }

  // Stop at the GPIO boundary before any drive-state code can issue a new
  // motion command. The charge-lock relay is the independent hardware layer.
  if (chargerPresent_) {
    motors_.emergencyStop();
  }

  if (!chargerPresent_) {
    sonar_.update();
    // Charging mode never needs to move the scanning servo or chassis.
    updateRadarTask(nowMs);
  }

  pms_.update(nowMs);
  battery_.update(nowMs);
  charge_.update(nowMs);
  // INA219 VIN- is permanently connected to battery positive, so its bus
  // voltage remains the authoritative battery reading even with no adapter.
  // The ADC divider is retained only as a fallback if INA219 is unavailable.
  const bool batteryVoltageValid = selectedBatteryVoltageValid();
  const float batteryVoltage = selectedBatteryVoltage();
  battery_.updateProtection(nowMs, batteryVoltageValid, batteryVoltage);
  chargeManager_.update(nowMs, chargerPresent_, batteryVoltageValid,
                        batteryVoltage, charge_.hasReading(),
                        charge_.currentMilliamp());
  batterySoc_.update(nowMs, batteryVoltageValid, batteryVoltage,
                     chargerPresent_, chargeManager_.state(),
                     chargeManager_.sessionChargedMah(), battery_.cutoff());

  fan_.setChargingLock(chargerPresent_);
  fan_.update(nowMs, pms_.warmedUp(nowMs), pms_.healthy(nowMs),
              pms_.pm25Average(), pms_.pm10Average(),
              pms_.pm25Latest(), pms_.pm10Latest());

  if (chargerPresent_) {
    if (state_ != RobotState::STARTUP) {
      transitionTo(RobotState::STARTUP, nowMs);
    }
    motors_.emergencyStop();
    updateDisplay(nowMs);
    reportStatus(nowMs);
    return;
  }

  if (battery_.cutoff() && state_ != RobotState::LOW_BATTERY) {
    transitionTo(RobotState::LOW_BATTERY, nowMs);
  }

  switch (state_) {
    case RobotState::STARTUP:
      handleStartup(nowMs);
      break;
    case RobotState::DRIVE:
      handleDrive(nowMs);
      break;
    case RobotState::BRAKE:
      handleBrake(nowMs);
      break;
    case RobotState::SCAN:
      handleScan(nowMs);
      break;
    case RobotState::TURN:
      handleTurn(nowMs);
      break;
    case RobotState::ESCAPE_REVERSE:
      handleEscapeReverse(nowMs);
      break;
    case RobotState::ESCAPE_SCAN:
      handleEscapeScan(nowMs);
      break;
    case RobotState::ESCAPE_TURN:
      handleEscapeTurn(nowMs);
      break;
    case RobotState::LOW_BATTERY:
      handleLowBattery();
      break;
    case RobotState::FAULT:
      handleFault(nowMs);
      break;
  }

  motors_.update(nowMs);
  updateDisplay(nowMs);
  reportStatus(nowMs);
}

void RobotController::prepareSafeStartup(const uint32_t nowMs) {
  motors_.emergencyStop();
  if (state_ != RobotState::STARTUP) {
    transitionTo(RobotState::STARTUP, nowMs);
  }
  // Restart the grace period even if charging already left us in STARTUP.
  stateEnteredMs_ = nowMs;
  pathPlanner_.reset(config::servo::CENTER_DEG);
  corridorCommitment_.reset();
  wheelSteeringPercent_ = 0;
  livePathAngleDeg_ = config::servo::CENTER_DEG;
  frontDistanceCm_ = config::sonar::MAX_VALID_CM;
  frontDistanceValid_ = false;
  frontDistanceUpdatedMs_ = 0;
  leftClearanceCm_ = config::sonar::MAX_VALID_CM;
  centerClearanceCm_ = config::sonar::MAX_VALID_CM;
  rightClearanceCm_ = config::sonar::MAX_VALID_CM;
  centerClearanceUpdatedMs_ = 0;
  latestLeftRayMs_ = 0;
  latestRightRayMs_ = 0;
  forwardThreatHitCount_ = 0;
  forwardThreatLastHitMs_ = 0;
  forwardThreatConfirmedMs_ = 0;
  rollingFrontAvoidActive_ = false;
  rollingAvoidSteerSign_ = 0;
  sideAvoidActive_ = false;
  sideAvoidSteerSign_ = 0;
  radarThreatScanActive_ = false;
  radarAnomalyFocusActive_ = false;
  threatSequenceIdx_ = 0;
  discreteAngleIdx_ = config::sonar::RADAR_CENTER_INDEX;
  discreteAngleDir_ = 1;
  discreteSampleCount_ = 0;
  discreteInvalidCount_ = 0;
  radarCoverageMask_ = 0;
  radarRevision_ = 0;
  plannerRevision_ = UINT8_MAX;
  for (uint8_t i = 0; i < config::sonar::RADAR_POINT_COUNT; ++i) {
    spatialMapUpdatedMs_[i] = 0;
    discreteClearMisses_[i] = 0;
  }
}

void RobotController::transitionTo(const RobotState next,
                                   const uint32_t nowMs) {
  if (state_ == next) {
    return;
  }
  if ((state_ == RobotState::SCAN || state_ == RobotState::DRIVE) && next != RobotState::SCAN && next != RobotState::DRIVE) {
    driveSweepAngle_ = config::servo::CENTER_DEG;
  }
  if (next == RobotState::DRIVE &&
      state_ == RobotState::ESCAPE_TURN) {
    lastEscapeCompletedMs_ = nowMs;
  }
  if (next == RobotState::ESCAPE_REVERSE) {
    const bool repeatedTrap =
        lastEscapeCompletedMs_ != 0 &&
        nowMs - lastEscapeCompletedMs_ <
            config::motor::ESCAPE_REPEAT_WINDOW_MS;
    if (repeatedTrap) {
      escapeAttemptCount_ =
          min(static_cast<uint8_t>(escapeAttemptCount_ + 1U),
              config::motor::ESCAPE_MAX_ATTEMPTS);
    } else {
      escapeAttemptCount_ = 1;
    }
  }
  // Preserve the complete radar map collected while stationary at startup.
  // Recovery transitions still clear their old map before returning to DRIVE.
  if (next == RobotState::DRIVE && state_ != RobotState::STARTUP) {
    pathPlanner_.reset(config::servo::CENTER_DEG);
    wheelSteeringPercent_ = 0;
    livePathAngleDeg_ = config::servo::CENTER_DEG;
    rollingFrontAvoidActive_ = false;
    rollingAvoidSteerSign_ = 0;
    rollingAvoidDirectionCorrectionUsed_ = false;
    rollingAvoidStartedMs_ = 0;
    rollingAvoidReleaseStartedMs_ = 0;
    straightRecoveryUntilMs_ = 0;
    rollingAvoidLastProgressMs_ = 0;
    rollingAvoidBestClearanceCm_ = 0;
    straightAssistActive_ = false;
    sideAvoidActive_ = false;
    sideAvoidSteerSign_ = 0;
    sideAvoidDistanceCm_ = config::sonar::MAX_VALID_CM;
    sideAvoidReleaseCm_ = config::sonar::SIDE_AVOID_RELEASE_CM;
    sideAvoidLastSeenMs_ = 0;
    sideAvoidStartedMs_ = 0;
    sideAvoidLastProgressMs_ = 0;
    sideAvoidBestDistanceCm_ = 0;
    radarStuckWatchActive_ = false;
    radarStuckWatchSteerSign_ = 0;
    radarStuckWatchStartedMs_ = 0;
    radarStuckStartCounts_ = motors_.encoderCounts();
    for (uint8_t i = 0;
         i < config::sonar::RADAR_STUCK_ANCHOR_COUNT;
         ++i) {
      radarStuckBaselineCm_[i] = 0;
      radarStuckBaselineUpdatedMs_[i] = 0;
    }
    frontDistanceCm_ = config::sonar::MAX_VALID_CM;
    frontDistanceValid_ = false;
    frontDistanceUpdatedMs_ = 0;
    leftClearanceCm_ = config::sonar::MAX_VALID_CM;
    centerClearanceCm_ = config::sonar::MAX_VALID_CM;
    centerClearanceUpdatedMs_ = 0;
    rightClearanceCm_ = config::sonar::MAX_VALID_CM;
    forwardThreatHitCount_ = 0;
    forwardThreatLastAngleDeg_ = config::servo::CENTER_DEG;
    forwardThreatLastHitMs_ = 0;
    forwardThreatConfirmedMs_ = 0;
    latestLeftRayCm_ = config::sonar::MAX_VALID_CM;
    latestRightRayCm_ = config::sonar::MAX_VALID_CM;
    latestLeftRayMs_ = 0;
    latestRightRayMs_ = 0;
    plannerRevision_ = UINT8_MAX;
    lastSteeringUpdateMs_ = nowMs;
    // Clear stale spatial map entries so old readings from previous state don't cause false braking
    for (uint8_t i = 0;
         i < config::sonar::RADAR_POINT_COUNT; ++i) {
      spatialMapUpdatedMs_[i] = 0;
    }
    radarCoverageMask_ = 0;
    radarRevision_ = 0;
    discreteAngleIdx_ = config::sonar::RADAR_CENTER_INDEX;
    discreteAngleDir_ = 1;
    radarThreatScanActive_ = false;
    threatSequenceIdx_ = 0;
    threatFocusAngleDeg_ = config::servo::CENTER_DEG;
    threatFocusDistanceCm_ = config::sonar::MAX_VALID_CM;
    discreteSampleCount_ = 0;
    discreteInvalidCount_ = 0;
    for (uint8_t i = 0; i < config::sonar::RADAR_POINT_COUNT; ++i) {
      discreteClearMisses_[i] = 0;
    }
  } else if (next == RobotState::DRIVE) {
    pathPlanner_.reset(config::servo::CENTER_DEG);
    wheelSteeringPercent_ = 0;
    livePathAngleDeg_ = config::servo::CENTER_DEG;
    straightAssistActive_ = false;
    plannerRevision_ = UINT8_MAX;
    radarThreatScanActive_ = false;
    threatSequenceIdx_ = 0;
    discreteSampleCount_ = 0;
    discreteInvalidCount_ = 0;
  }
  if (next == RobotState::ESCAPE_REVERSE) {
    forceEscape_ = false;
  }
  if (next == RobotState::ESCAPE_SCAN) {
    motors_.emergencyStop();
    pathPlanner_.reset(config::servo::CENTER_DEG);
    wheelSteeringPercent_ = 0;
    frontDistanceCm_ = config::sonar::MAX_VALID_CM;
    frontDistanceValid_ = false;
    frontDistanceUpdatedMs_ = 0;
    leftClearanceCm_ = config::sonar::MAX_VALID_CM;
    centerClearanceCm_ = config::sonar::MAX_VALID_CM;
    centerClearanceUpdatedMs_ = 0;
    rightClearanceCm_ = config::sonar::MAX_VALID_CM;
    forwardThreatHitCount_ = 0;
    forwardThreatLastAngleDeg_ = config::servo::CENTER_DEG;
    forwardThreatLastHitMs_ = 0;
    forwardThreatConfirmedMs_ = 0;
    latestLeftRayCm_ = config::sonar::MAX_VALID_CM;
    latestRightRayCm_ = config::sonar::MAX_VALID_CM;
    latestLeftRayMs_ = 0;
    latestRightRayMs_ = 0;
    sideAvoidActive_ = false;
    sideAvoidSteerSign_ = 0;
    sideAvoidDistanceCm_ = config::sonar::MAX_VALID_CM;
    sideAvoidReleaseCm_ = config::sonar::SIDE_AVOID_RELEASE_CM;
    sideAvoidLastSeenMs_ = 0;
    sideAvoidStartedMs_ = 0;
    sideAvoidLastProgressMs_ = 0;
    sideAvoidBestDistanceCm_ = 0;
    for (uint8_t i = 0;
         i < config::sonar::RADAR_POINT_COUNT; ++i) {
      spatialMapUpdatedMs_[i] = 0;
    }
    radarCoverageMask_ = 0;
    radarRevision_ = 0;
    discreteAngleIdx_ = config::sonar::RADAR_CENTER_INDEX;
    discreteAngleDir_ = 1;
    escapeScanStep_ = 0;
    radarThreatScanActive_ = false;
    threatSequenceIdx_ = 0;
    threatFocusAngleDeg_ = config::servo::CENTER_DEG;
    threatFocusDistanceCm_ = config::sonar::MAX_VALID_CM;
    discreteSampleCount_ = 0;
    discreteInvalidCount_ = 0;
    for (uint8_t i = 0; i < config::sonar::RADAR_POINT_COUNT; ++i) {
      discreteClearMisses_[i] = 0;
    }
  }
  if (next == RobotState::TURN ||
      next == RobotState::ESCAPE_REVERSE ||
      next == RobotState::ESCAPE_TURN) {
    motors_.markPosition();
  }
  if (next == RobotState::TURN || next == RobotState::ESCAPE_TURN) {
    motors_.setSoftStartDuration(
        config::motor::TURN_SOFT_START_SECONDS);
  } else if (state_ == RobotState::STARTUP && next == RobotState::DRIVE) {
    // Get up to clear-path cruise promptly without applying an instantaneous
    // current step to the drivetrain.
    motors_.setSoftStartDuration(1.5F);
  } else if (next == RobotState::DRIVE ||
             next == RobotState::ESCAPE_REVERSE) {
    motors_.setSoftStartDuration(1.0F);  // 1-second ramp for all normal operations
  }

  state_ = next;
  stateEnteredMs_ = nowMs;
}

void RobotController::handleStartup(const uint32_t nowMs) {
  motors_.stop();
  const uint32_t fullRadarCoverageMask =
      (1UL << config::sonar::RADAR_POINT_COUNT) - 1UL;
  const bool initialMapComplete =
      radarCoverageMask_ == fullRadarCoverageMask;
  const bool scannerCenteredAndSettled =
      currentServoAngle_ == config::servo::CENTER_DEG &&
      nowMs - lastServoStepMs_ >= servoSettleDurationMs_;
  const bool freshCenteredReading =
      frontDistanceValid_ &&
      frontDistanceUpdatedMs_ >= lastServoStepMs_ &&
      frontReadingFresh(nowMs);
  if (config::safety::MOTION_ENABLED &&
      pms_.warmedUp(nowMs) &&
      nowMs - stateEnteredMs_ >= config::safety::STARTUP_GRACE_MS &&
      !fan_.startupDustShakeActive() &&
      initialMapComplete &&
      scannerCenteredAndSettled &&
      freshCenteredReading) {
    // Keep the chassis stopped while the display says PMS is warming up.
    // Mobility and the user-visible warm-up countdown must share the same
    // readiness condition so the robot cannot depart prematurely.
    transitionTo(RobotState::DRIVE, nowMs);
  }
}

void RobotController::handleDrive(const uint32_t nowMs) {
  if (escapeAttemptCount_ != 0 &&
      lastEscapeCompletedMs_ != 0 &&
      nowMs - lastEscapeCompletedMs_ >=
          config::motor::ESCAPE_STABLE_RESET_MS) {
    escapeAttemptCount_ = 0;
    lastEscapeCompletedMs_ = 0;
  }

  if (motors_.stalled(nowMs)) {
    motors_.emergencyStop();
    rememberFailedAngle(nowMs);
    recordAvoidEvent(nowMs);
    transitionTo(RobotState::ESCAPE_REVERSE, nowMs);
    return;
  }
  if (motors_.wheelImbalanceStalled(nowMs)) {
    motors_.emergencyStop();
    rememberFailedAngle(nowMs);
    recordAvoidEvent(nowMs);
    transitionTo(RobotState::ESCAPE_REVERSE, nowMs);
    return;
  }

  const GapSelection liveGap = selectLiveGap(nowMs);
  const uint32_t fullRadarCoverageMask =
      (1UL << config::sonar::RADAR_POINT_COUNT) - 1UL;
  const bool radarCoverageComplete =
      radarCoverageMask_ == fullRadarCoverageMask;
  const bool freshFront = frontReadingFresh(nowMs);
  const bool freshForwardCorridor =
      centerClearanceUpdatedMs_ != 0 &&
      nowMs - centerClearanceUpdatedMs_ <=
          config::sonar::FRONT_FRESH_MS;
  // A clearly open centred corridor is the strongest evidence that the robot
  // should continue straight. Wide-angle side echoes often see walls, floor,
  // or the chassis even though none of them blocks forward travel.
  // Use the higher threshold only to release an avoidance already in
  // progress. During normal driving, anything above the activation distance
  // remains straight; this removes the old 60..85 cm steering dead zone.
  const uint16_t requiredForwardClearanceCm =
      rollingFrontAvoidActive_
          ? config::sonar::FRONT_AVOID_RELEASE_CM
          : config::sonar::FRONT_AVOID_CM;
  const bool centeredForwardClearlyOpen =
      freshFront &&
      freshForwardCorridor &&
      frontDistanceCm_ > requiredForwardClearanceCm &&
      centerClearanceCm_ > requiredForwardClearanceCm;

  // Forward clearance has priority over all side-map guidance. Wide-angle
  // sonar returns often hit the sensor bracket, chassis, or floor and must not
  // make the robot hook left/right when its actual driving corridor is open.
  // Clear aggressive avoidance latches on an open-front control cycle. Do not
  // reset the steering accumulator here: gentle side guidance below is still
  // allowed to curve away from a nearby left/right object.
  if (centeredForwardClearlyOpen) {
    rollingFrontAvoidActive_ = false;
    rollingAvoidSteerSign_ = 0;
    rollingAvoidStartedMs_ = 0;
    rollingAvoidReleaseStartedMs_ = 0;
    rollingAvoidLastProgressMs_ = 0;
    rollingAvoidBestClearanceCm_ = 0;
    forwardThreatHitCount_ = 0;
    forwardThreatLastHitMs_ = 0;
    forwardThreatConfirmedMs_ = 0;
    livePathAngleDeg_ = config::servo::CENTER_DEG;
  }
  const int32_t sideDifferenceCm =
      static_cast<int32_t>(leftClearanceCm_) -
      static_cast<int32_t>(rightClearanceCm_);
  const uint32_t absoluteSideDifferenceCm =
      static_cast<uint32_t>(
          sideDifferenceCm < 0 ? -sideDifferenceCm : sideDifferenceCm);
  const bool avoidWasActive = rollingFrontAvoidActive_;
  const bool confirmedForwardThreat =
      forwardThreatConfirmedMs_ != 0 &&
      nowMs - forwardThreatConfirmedMs_ <=
          config::sonar::THREAT_CONFIRMED_FRESH_MS;
  const bool forwardCorridorClearlyReleased =
      freshFront &&
      freshForwardCorridor &&
      frontDistanceCm_ >=
          config::sonar::FRONT_AVOID_RELEASE_CM &&
      centerClearanceCm_ >=
          config::sonar::FRONT_AVOID_RELEASE_CM;
  if (avoidWasActive && forwardCorridorClearlyReleased) {
    if (rollingAvoidReleaseStartedMs_ == 0) {
      rollingAvoidReleaseStartedMs_ = nowMs;
    }
  } else {
    rollingAvoidReleaseStartedMs_ = 0;
  }
  const bool releaseConfirmed =
      rollingAvoidReleaseStartedMs_ != 0 &&
      nowMs - rollingAvoidReleaseStartedMs_ >=
          config::sonar::FRONT_AVOID_RELEASE_CONFIRM_MS;
  // Once avoidance starts, keep the curve committed until the whole forward
  // corridor has remained clear. A momentary missed echo must not cancel the
  // turn and send the robot straight back toward the obstacle.
  rollingFrontAvoidActive_ =
      confirmedForwardThreat ||
      (avoidWasActive && !releaseConfirmed);
  if (!avoidWasActive && rollingFrontAvoidActive_) {
    rollingAvoidStartedMs_ = nowMs;
    rollingAvoidLastProgressMs_ = nowMs;
    rollingAvoidBestClearanceCm_ =
        freshForwardCorridor
            ? centerClearanceCm_
            : (freshFront
                   ? frontDistanceCm_
                   : config::sonar::MIN_VALID_CM);
    threatSequenceIdx_ = 0;
    const int32_t gapDirection =
        liveGap.found
            ? DifferentialSteering::pathOffsetPercent(
                  liveGap.targetAngleDeg,
                  config::servo::CENTER_DEG,
                  config::servo::ANGLE_TO_TURN_SIGN)
            : 0;
    if (gapDirection < 0) {
      rollingAvoidSteerSign_ = -1;
    } else if (gapDirection > 0) {
      rollingAvoidSteerSign_ = 1;
    } else {
      // A tied map still needs one decisive direction. Prefer the side with
      // greater measured clearance and keep it until the obstacle is passed.
      rollingAvoidSteerSign_ =
          leftClearanceCm_ >= rightClearanceCm_ ? -1 : 1;
    }
    rollingAvoidDirectionCorrectionUsed_ = false;
  }
  if (avoidWasActive && !rollingFrontAvoidActive_) {
    // Once the selected opening has rotated into the forward direction, drop
    // all residual turn command immediately. Equal wheel targets re-enable
    // the encoder cross-coupled straight-line lock.
    pathPlanner_.reset(config::servo::CENTER_DEG);
    livePathAngleDeg_ = config::servo::CENTER_DEG;
    rollingAvoidStartedMs_ = 0;
    rollingAvoidSteerSign_ = 0;
    rollingAvoidDirectionCorrectionUsed_ = false;
    rollingAvoidReleaseStartedMs_ = 0;
    straightRecoveryUntilMs_ =
        nowMs + config::sonar::POST_AVOID_STRAIGHT_MS;
    rollingAvoidLastProgressMs_ = 0;
    rollingAvoidBestClearanceCm_ = 0;
  }
  if (rollingFrontAvoidActive_) {
    const uint16_t currentAvoidClearance =
        freshForwardCorridor
            ? centerClearanceCm_
            : (freshFront
                   ? frontDistanceCm_
                   : rollingAvoidBestClearanceCm_);
    if (currentAvoidClearance >=
        rollingAvoidBestClearanceCm_ +
            config::sonar::FRONT_PROGRESS_CM) {
      rollingAvoidBestClearanceCm_ = currentAvoidClearance;
      rollingAvoidLastProgressMs_ = nowMs;
    }
  }

  // The anti-hunting latch must not turn an initial bad echo into a permanent
  // wrong-way decision. Once both sides have been revisited by the threat
  // sequence, permit exactly one correction when the opposite corridor is
  // open by a decisive margin. Further flips remain locked out for the rest
  // of this avoidance manoeuvre.
  const bool leftDecisionRayFresh =
      latestLeftRayMs_ != 0 &&
      nowMs - latestLeftRayMs_ <= config::sonar::SIDE_RAY_FRESH_MS;
  const bool rightDecisionRayFresh =
      latestRightRayMs_ != 0 &&
      nowMs - latestRightRayMs_ <= config::sonar::SIDE_RAY_FRESH_MS;
  const uint32_t decisionRaySkewMs =
      latestLeftRayMs_ >= latestRightRayMs_
          ? latestLeftRayMs_ - latestRightRayMs_
          : latestRightRayMs_ - latestLeftRayMs_;
  if (rollingFrontAvoidActive_ &&
      !rollingAvoidDirectionCorrectionUsed_ &&
      rollingAvoidSteerSign_ != 0 &&
      nowMs - rollingAvoidStartedMs_ >=
          config::sonar::AVOID_DIRECTION_RECHECK_MS &&
      leftDecisionRayFresh && rightDecisionRayFresh &&
      decisionRaySkewMs <= config::sonar::CORRIDOR_PAIR_MAX_SKEW_MS) {
    const uint16_t selectedSideCm =
        rollingAvoidSteerSign_ < 0
            ? leftClearanceCm_
            : rightClearanceCm_;
    const uint16_t oppositeSideCm =
        rollingAvoidSteerSign_ < 0
            ? rightClearanceCm_
            : leftClearanceCm_;
    const bool selectedSideBlocked =
        selectedSideCm < config::sonar::FRONT_AVOID_RELEASE_CM;
    const bool oppositeSideClearlyOpen =
        oppositeSideCm >= config::sonar::FRONT_AVOID_RELEASE_CM &&
        static_cast<uint32_t>(oppositeSideCm) >=
            static_cast<uint32_t>(selectedSideCm) +
                config::sonar::AVOID_DIRECTION_SWITCH_MARGIN_CM;
    if (selectedSideBlocked && oppositeSideClearlyOpen) {
      rollingAvoidSteerSign_ =
          static_cast<int8_t>(-rollingAvoidSteerSign_);
      rollingAvoidDirectionCorrectionUsed_ = true;
      pathPlanner_.reset(config::servo::CENTER_DEG);
      plannerRevision_ = UINT8_MAX;
    }
  }

  const bool narrowCorridorMode =
      !rollingFrontAvoidActive_ &&
      freshFront &&
      frontDistanceCm_ >= config::sonar::CORRIDOR_FRONT_CM &&
      leftClearanceCm_ < config::sonar::STEER_CM &&
      rightClearanceCm_ < config::sonar::STEER_CM &&
      absoluteSideDifferenceCm <= config::sonar::CORRIDOR_BALANCE_CM;

  // Once a close side wall is seen, keep steering away long enough for the
  // detailed radar to revisit that side. Without this latch, the wall could
  // disappear from the fresh map halfway through the avoidance curve.
  const uint16_t sideActivationCm =
      centeredForwardClearlyOpen
          ? config::sonar::SIDE_STUCK_CM
          : config::sonar::SIDE_AVOID_CM;
  const bool closeLeftSide =
      latestLeftRayMs_ != 0 &&
      nowMs - latestLeftRayMs_ <=
          config::sonar::SIDE_RAY_FRESH_MS &&
      latestLeftRayCm_ <= sideActivationCm;
  const bool closeRightSide =
      latestRightRayMs_ != 0 &&
      nowMs - latestRightRayMs_ <=
          config::sonar::SIDE_RAY_FRESH_MS &&
      latestRightRayCm_ <= sideActivationCm;
  const bool sideAvoidWasActive = sideAvoidActive_;
  const int8_t previousSideSteerSign = sideAvoidSteerSign_;
  if (!narrowCorridorMode && (closeLeftSide || closeRightSide)) {
    const int8_t newSteerSign =
        closeLeftSide && closeRightSide
            ? (latestLeftRayCm_ <= latestRightRayCm_ ? 1 : -1)
            : (closeLeftSide ? 1 : -1);
    const uint16_t currentCommittedSideCm =
        previousSideSteerSign > 0
            ? latestLeftRayCm_
            : latestRightRayCm_;
    const bool currentSideResolved =
        sideAvoidWasActive &&
        currentCommittedSideCm >= sideAvoidReleaseCm_;
    const bool commitTimeComplete =
        sideAvoidWasActive &&
        nowMs - sideAvoidStartedMs_ >=
            config::sonar::SIDE_DIRECTION_COMMIT_MS;
    // Keep solving the current side. The opposite side may be observed and
    // mapped, but it cannot reverse steering until this side is safely clear.
    const bool acceptNewDirection =
        !sideAvoidWasActive ||
        newSteerSign == previousSideSteerSign ||
        (currentSideResolved && commitTimeComplete);
    if (acceptNewDirection) {
      sideAvoidActive_ = true;
      sideAvoidSteerSign_ = newSteerSign;
      sideAvoidReleaseCm_ =
          centeredForwardClearlyOpen
              ? config::sonar::SIDE_STUCK_RELEASE_CM
              : config::sonar::SIDE_AVOID_RELEASE_CM;
    }
    sideAvoidDistanceCm_ =
        sideAvoidSteerSign_ > 0
            ? latestLeftRayCm_
            : latestRightRayCm_;
    const bool committedSideStillClose =
        sideAvoidSteerSign_ > 0 ? closeLeftSide : closeRightSide;
    if (committedSideStillClose) {
      sideAvoidLastSeenMs_ = nowMs;
    }
    if (!sideAvoidWasActive ||
        (acceptNewDirection &&
         newSteerSign != previousSideSteerSign)) {
      sideAvoidStartedMs_ = nowMs;
      sideAvoidLastProgressMs_ = nowMs;
      sideAvoidBestDistanceCm_ = sideAvoidDistanceCm_;
    } else if (sideAvoidDistanceCm_ >=
               sideAvoidBestDistanceCm_ +
                   config::sonar::SIDE_PROGRESS_CM) {
      sideAvoidBestDistanceCm_ = sideAvoidDistanceCm_;
      sideAvoidLastProgressMs_ = nowMs;
    }
  } else if (sideAvoidActive_) {
    const uint16_t committedSideCm =
        sideAvoidSteerSign_ > 0
            ? latestLeftRayCm_
            : latestRightRayCm_;
    const uint32_t committedSideMs =
        sideAvoidSteerSign_ > 0
            ? latestLeftRayMs_
            : latestRightRayMs_;
    const bool committedSideFresh =
        committedSideMs != 0 &&
        nowMs - committedSideMs <=
            config::sonar::SIDE_RAY_FRESH_MS;
    const bool resolutionTimeComplete =
        nowMs - sideAvoidStartedMs_ >=
            config::sonar::SIDE_DIRECTION_COMMIT_MS;
    const bool resolved =
        committedSideFresh &&
        committedSideCm >= sideAvoidReleaseCm_ &&
        resolutionTimeComplete;
    const bool observationExpired =
        nowMs - sideAvoidLastSeenMs_ >=
            config::sonar::SIDE_AVOID_HOLD_MS;
    if (resolved || observationExpired) {
      sideAvoidActive_ = false;
      sideAvoidSteerSign_ = 0;
      sideAvoidDistanceCm_ = config::sonar::MAX_VALID_CM;
      sideAvoidReleaseCm_ = config::sonar::SIDE_AVOID_RELEASE_CM;
      sideAvoidStartedMs_ = 0;
      sideAvoidLastProgressMs_ = 0;
      sideAvoidBestDistanceCm_ = 0;
    }
  }

  const uint8_t liveGapErrorDeg =
      liveGap.found
          ? static_cast<uint8_t>(
                liveGap.targetAngleDeg >= config::servo::CENTER_DEG
                    ? liveGap.targetAngleDeg -
                          config::servo::CENTER_DEG
                    : config::servo::CENTER_DEG -
                          liveGap.targetAngleDeg)
          : 0;
  const bool openPathGuidanceNeeded =
      radarCoverageComplete &&
      !centeredForwardClearlyOpen &&
      liveGap.found &&
      liveGapErrorDeg >
          config::sonar::OPEN_PATH_GUIDANCE_DEADBAND_DEG;
  const bool straightAssistWasActive = straightAssistActive_;
  straightAssistActive_ =
      !sideAvoidActive_ &&
      !rollingFrontAvoidActive_ &&
      !openPathGuidanceNeeded &&
      centeredForwardClearlyOpen;
  if (straightAssistActive_ && !straightAssistWasActive) {
    pathPlanner_.reset(config::servo::CENTER_DEG);
    livePathAngleDeg_ = config::servo::CENTER_DEG;
  }
  const bool corridorMode =
      narrowCorridorMode || straightAssistActive_;
  const bool rollingAvoidMode =
      rollingFrontAvoidActive_ &&
      (!freshFront ||
       frontDistanceCm_ > config::sonar::EMERGENCY_CM);

  uint8_t observedPathAngle =
      liveGap.found
          ? liveGap.targetAngleDeg
          : pathPlanner_.targetAngleDeg();
  if (rollingAvoidMode &&
      observedPathAngle == config::servo::CENTER_DEG) {
    // A blocked forward ray must produce a committed side choice even when
    // the corridor scores tie. Prefer the side with more measured clearance.
    const bool preferPhysicalLeft =
        leftClearanceCm_ >= rightClearanceCm_;
    if (config::servo::ANGLE_TO_TURN_SIGN > 0) {
      observedPathAngle = preferPhysicalLeft ? 60 : 120;
    } else {
      observedPathAngle = preferPhysicalLeft ? 120 : 60;
    }
  }

  // Only accept a changed path after it wins several fresh radar updates.
  // A genuinely centred open corridor locks to 90 degrees; a confirmed open
  // corridor outside the deadband remains eligible for gentle guidance.
  if (plannerRevision_ != radarRevision_) {
    pathPlanner_.observeTarget(
        observedPathAngle,
        config::servo::CENTER_DEG,
        corridorMode,
        rollingAvoidMode
            ? config::sonar::AVOID_PATH_CONFIRMATION_WINS
            : config::sonar::PATH_CONFIRMATION_WINS,
        rollingAvoidMode
            ? config::sonar::AVOID_PATH_TARGET_STEP_DEG
            : config::sonar::PATH_TARGET_STEP_DEG);
    plannerRevision_ = radarRevision_;
  }
  livePathAngleDeg_ = pathPlanner_.targetAngleDeg();

  const bool emergencyFront =
      freshFront &&
      frontDistanceCm_ <= config::sonar::EMERGENCY_CM;
  const bool escapeRecoveryGrace =
      lastEscapeCompletedMs_ != 0 &&
      nowMs - lastEscapeCompletedMs_ <
          config::sonar::ESCAPE_RECOVERY_GRACE_MS;
  const bool rollingAvoidFailed =
      !escapeRecoveryGrace &&
      rollingFrontAvoidActive_ &&
      rollingAvoidStartedMs_ != 0 &&
      nowMs - rollingAvoidStartedMs_ >=
          config::sonar::AVOID_FAILSAFE_MS &&
      rollingAvoidLastProgressMs_ != 0 &&
      nowMs - rollingAvoidLastProgressMs_ >=
          config::sonar::FRONT_PROGRESS_TIMEOUT_MS &&
      freshForwardCorridor &&
      centerClearanceCm_ <= config::sonar::AVOID_FAILSAFE_CM;
  const uint16_t currentThreatSideRayCm =
      sideAvoidSteerSign_ > 0
          ? latestLeftRayCm_
          : latestRightRayCm_;
  const uint32_t currentThreatSideRayMs =
      sideAvoidSteerSign_ > 0
          ? latestLeftRayMs_
          : latestRightRayMs_;
  const bool freshThreatSideRay =
      sideAvoidActive_ &&
      currentThreatSideRayMs != 0 &&
      nowMs - currentThreatSideRayMs <=
          config::sonar::SIDE_RAY_FRESH_MS;

  // The encoders only establish that the driven wheels claim motion. Actual
  // chassis motion is verified independently from changes in real radar
  // echoes. This also runs while driving straight, not only during avoidance.
  const int32_t maximumCommandedTps =
      max(labs(motors_.leftTargetTps()),
          labs(motors_.rightTargetTps()));
  const int32_t maximumReportedTps =
      max(labs(motors_.leftMeasuredTps()),
          labs(motors_.rightMeasuredTps()));
  const bool wheelsClaimMotion =
      maximumCommandedTps >=
          config::sonar::RADAR_STUCK_MIN_COMMAND_TPS &&
      maximumReportedTps >=
          config::sonar::RADAR_STUCK_MIN_REPORTED_TPS;
  const int8_t radarSteerSign =
      wheelSteeringPercent_ <=
              -config::sonar::RADAR_STUCK_TURN_THRESHOLD_PERCENT
          ? -1
          : (wheelSteeringPercent_ >=
                     config::sonar::RADAR_STUCK_TURN_THRESHOLD_PERCENT
                 ? 1
                 : 0);
  const auto radarMotionAnchorAngle =
      [radarSteerSign](const uint8_t index) -> uint8_t {
    if (radarSteerSign == 0) {
      return config::sonar::radarStuckAnchorAngle(index);
    }
    if (index == 0) return 85;
    if (index == 1) return 90;
    if (index == 2) return 95;
    const bool physicalLeftSelected = radarSteerSign < 0;
    const bool lowAnglesArePhysicalLeft =
        config::servo::ANGLE_TO_TURN_SIGN > 0;
    const bool useLowAngles =
        physicalLeftSelected == lowAnglesArePhysicalLeft;
    if (index == 3) return useLowAngles ? 60 : 120;
    return useLowAngles ? 45 : 135;
  };
  bool radarMotionStuck = false;
  if (!wheelsClaimMotion || escapeRecoveryGrace) {
    radarStuckWatchActive_ = false;
    radarStuckWatchSteerSign_ = 0;
    radarStuckWatchStartedMs_ = 0;
  } else if (!radarStuckWatchActive_ ||
             radarStuckWatchSteerSign_ !=
                 radarSteerSign) {
    uint8_t availableAnchors = 0;
    for (uint8_t i = 0;
         i < config::sonar::RADAR_STUCK_ANCHOR_COUNT;
         ++i) {
      const uint8_t angle =
          radarMotionAnchorAngle(i);
      const uint8_t mapIndex =
          static_cast<uint8_t>(
              (angle - config::sonar::RADAR_MIN_DEG) /
              config::sonar::RADAR_STEP_DEG);
      const bool usable =
          spatialMapUpdatedMs_[mapIndex] != 0 &&
          nowMs - spatialMapUpdatedMs_[mapIndex] <=
              config::sonar::RADAR_STUCK_ANCHOR_FRESH_MS &&
          spatialMap_[mapIndex] >=
              config::sonar::MIN_VALID_CM &&
          spatialMap_[mapIndex] <=
              config::sonar::RADAR_STUCK_MAX_ANCHOR_CM;
      radarStuckBaselineCm_[i] =
          usable ? spatialMap_[mapIndex] : 0;
      radarStuckBaselineUpdatedMs_[i] =
          usable ? spatialMapUpdatedMs_[mapIndex] : 0;
      if (usable) {
        ++availableAnchors;
      }
    }
    if (availableAnchors >=
        config::sonar::
            RADAR_STUCK_MIN_COMPARABLE_ANCHORS) {
      radarStuckWatchActive_ = true;
      radarStuckWatchSteerSign_ = radarSteerSign;
      radarStuckWatchStartedMs_ = nowMs;
      radarStuckStartCounts_ = motors_.encoderCounts();
    }
  } else if (nowMs - radarStuckWatchStartedMs_ >=
             config::sonar::RADAR_STUCK_TIMEOUT_MS) {
    const EncoderCounts currentCounts =
        motors_.encoderCounts();
    const uint32_t leftReportedTravel =
        static_cast<uint32_t>(
            labs(currentCounts.left -
                  radarStuckStartCounts_.left));
    const uint32_t rightReportedTravel =
        static_cast<uint32_t>(
            labs(currentCounts.right -
                  radarStuckStartCounts_.right));
    const uint32_t requiredWheelTravel =
        MotionGeometry::linearMillimetresToTicks(
            config::sonar::RADAR_STUCK_MIN_WHEEL_TRAVEL_MM,
            config::motor::WHEEL_DIAMETER_MM,
            config::motor::ENCODER_TICKS_PER_WHEEL_REV);
    uint8_t comparableAnchors = 0;
    uint8_t stableAnchors = 0;
    uint8_t forwardComparableAnchors = 0;
    uint8_t forwardStableAnchors = 0;
    for (uint8_t i = 0;
         i < config::sonar::RADAR_STUCK_ANCHOR_COUNT;
         ++i) {
      if (radarStuckBaselineCm_[i] == 0) {
        continue;
      }
      const uint8_t angle =
          radarMotionAnchorAngle(i);
      const uint8_t mapIndex =
          static_cast<uint8_t>(
              (angle - config::sonar::RADAR_MIN_DEG) /
              config::sonar::RADAR_STEP_DEG);
      const bool reobserved =
          spatialMapUpdatedMs_[mapIndex] >
              radarStuckBaselineUpdatedMs_[i] &&
          nowMs - spatialMapUpdatedMs_[mapIndex] <=
              config::sonar::RADAR_STUCK_ANCHOR_FRESH_MS &&
          spatialMap_[mapIndex] <=
              config::sonar::RADAR_STUCK_MAX_ANCHOR_CM;
      if (!reobserved) {
        continue;
      }
      ++comparableAnchors;
      const bool forwardAnchor =
          angle >= config::sonar::FORWARD_MIN_DEG &&
          angle <= config::sonar::FORWARD_MAX_DEG;
      if (forwardAnchor) {
        ++forwardComparableAnchors;
      }
      const uint16_t currentCm = spatialMap_[mapIndex];
      const uint16_t baselineCm =
          radarStuckBaselineCm_[i];
      const uint16_t differenceCm =
          currentCm >= baselineCm
              ? currentCm - baselineCm
              : baselineCm - currentCm;
      if (differenceCm <=
          config::sonar::RADAR_STUCK_STABLE_DELTA_CM) {
        ++stableAnchors;
        if (forwardAnchor) {
          ++forwardStableAnchors;
        }
      }
    }
    const uint32_t maximumReportedTravel =
        max(leftReportedTravel, rightReportedTravel);
    const RadarMotionResult motionResult =
        RadarMotionVerifier::verify(
            maximumReportedTravel, requiredWheelTravel,
            comparableAnchors, stableAnchors,
            forwardComparableAnchors, forwardStableAnchors,
            config::sonar::RADAR_STUCK_MIN_COMPARABLE_ANCHORS,
            radarSteerSign == 0);
    radarMotionStuck =
        motionResult == RadarMotionResult::STUCK;
    if (!radarMotionStuck) {
      // The scene changed, so the chassis made real progress. Re-acquire a
      // new fingerprint instead of allowing the old window to fire later.
      radarStuckWatchActive_ = false;
      radarStuckWatchStartedMs_ = 0;
    }
  }
  const bool sideEmergency =
      !escapeRecoveryGrace &&
      freshThreatSideRay &&
      sideAvoidStartedMs_ != 0 &&
      nowMs - sideAvoidStartedMs_ >=
          config::sonar::SIDE_EMERGENCY_CONFIRM_MS &&
      currentThreatSideRayCm <=
          config::sonar::SIDE_EMERGENCY_CM;
  const bool sideAvoidFailed =
      !escapeRecoveryGrace &&
      freshThreatSideRay &&
      sideAvoidStartedMs_ != 0 &&
      nowMs - sideAvoidStartedMs_ >=
          config::sonar::SIDE_STUCK_GRACE_MS &&
      nowMs - sideAvoidLastProgressMs_ >=
          config::sonar::SIDE_PROGRESS_TIMEOUT_MS &&
      currentThreatSideRayCm <=
          config::sonar::SIDE_STUCK_CM;

  // Reverse either for an emergency-close 90-degree return, or when a
  // committed rolling curve has had enough time but still cannot open the
  // forward corridor. This handles traction and motor-response edge cases
  // before physical contact or stall detection.
  if (emergencyFront || rollingAvoidFailed ||
      radarMotionStuck ||
      sideEmergency || sideAvoidFailed) {
    motors_.emergencyStop();
    rememberFailedAngle(nowMs);
    if (liveGap.found &&
        liveGap.targetAngleDeg != config::servo::CENTER_DEG) {
      const int16_t signedError =
          (static_cast<int16_t>(liveGap.targetAngleDeg) -
           config::servo::CENTER_DEG) *
          config::servo::ANGLE_TO_TURN_SIGN;
      escapeTurn_ =
          signedError < 0 ? TurnDirection::LEFT : TurnDirection::RIGHT;
    } else {
      escapeTurn_ =
          leftClearanceCm_ >= rightClearanceCm_
              ? TurnDirection::LEFT
              : TurnDirection::RIGHT;
    }
    recordAvoidEvent(nowMs);
    transitionTo(RobotState::ESCAPE_REVERSE, nowMs);
    return;
  }

  const bool criticalSide =
      !corridorMode && sideAvoidActive_;
  const bool safeRollingCruise =
      radarCoverageComplete &&
      freshFront &&
      frontDistanceCm_ > config::sonar::SLOW_CM &&
      !rollingAvoidMode &&
      (!criticalSide || corridorMode);
  int32_t driveTps =
      safeRollingCruise
          ? config::motor::CRUISE_TPS
          : config::motor::APPROACH_TPS;

  int32_t wallSteeringPercent = 0;
  const bool bothSideRaysFresh =
      latestLeftRayMs_ != 0 && latestRightRayMs_ != 0 &&
      nowMs - latestLeftRayMs_ <=
          config::sonar::SIDE_RAY_FRESH_MS &&
      nowMs - latestRightRayMs_ <=
          config::sonar::SIDE_RAY_FRESH_MS;
  const uint32_t sideRaySkewMs =
      latestLeftRayMs_ >= latestRightRayMs_
          ? latestLeftRayMs_ - latestRightRayMs_
          : latestRightRayMs_ - latestLeftRayMs_;
  const bool pairedSideReadingsFresh =
      bothSideRaysFresh &&
      sideRaySkewMs <=
          config::sonar::CORRIDOR_PAIR_MAX_SKEW_MS;
  const int8_t corridorCorrectionSign =
      corridorCommitment_.update(
          nowMs, leftClearanceCm_, rightClearanceCm_,
          narrowCorridorMode, pairedSideReadingsFresh,
          config::sonar::CENTERING_DEADBAND_CM,
          config::sonar::CORRIDOR_PROGRESS_PERCENT,
          config::sonar::CORRIDOR_MIN_COMMIT_MS,
          config::sonar::CORRIDOR_MAX_COMMIT_MS,
          config::sonar::CORRIDOR_SETTLE_MS);
  if (narrowCorridorMode) {
    if (corridorCorrectionSign != 0) {
      int32_t correctionMagnitude =
          static_cast<int32_t>(absoluteSideDifferenceCm) * 2L;
      if (correctionMagnitude <
          config::sonar::CORRIDOR_MIN_STEER_PERCENT) {
        correctionMagnitude =
            config::sonar::CORRIDOR_MIN_STEER_PERCENT;
      }
      if (correctionMagnitude >
          config::sonar::CORRIDOR_MAX_STEER_PERCENT) {
        correctionMagnitude =
            config::sonar::CORRIDOR_MAX_STEER_PERCENT;
      }
      wallSteeringPercent =
          corridorCorrectionSign * correctionMagnitude;
    }
  } else if (straightAssistActive_) {
    // Keep the selected path centred, but gently curve away when one side is
    // genuinely close. Positive offset makes the left wheel faster and turns
    // right, so a low left clearance produces the required right-hand escape.
    const bool sideGuidanceNeeded =
        leftClearanceCm_ < config::sonar::SIDE_GUIDANCE_CM ||
        rightClearanceCm_ < config::sonar::SIDE_GUIDANCE_CM;
    wallSteeringPercent =
        sideGuidanceNeeded
            ? DifferentialSteering::offsetPercent(
                  leftClearanceCm_, rightClearanceCm_,
                  config::sonar::SIDE_GUIDANCE_CM,
                  config::sonar::SIDE_GUIDANCE_MAX_STEER_PERCENT)
            : 0;
  } else {
    wallSteeringPercent =
        DifferentialSteering::offsetPercent(
            leftClearanceCm_,
            rightClearanceCm_,
            config::sonar::STEER_CM,
            config::sonar::MAX_STEER_PERCENT);
  }

  const bool gentleOpenPathGuidance =
      openPathGuidanceNeeded &&
      !criticalSide &&
      !rollingAvoidMode &&
      freshFront &&
      frontDistanceCm_ >=
          config::sonar::FRONT_AVOID_RELEASE_CM;
  const int16_t maximumSteerPercent =
      criticalSide || rollingAvoidMode
          ? config::sonar::CRITICAL_MAX_STEER_PERCENT
          : (gentleOpenPathGuidance
                 ? config::sonar::OPEN_PATH_MAX_STEER_PERCENT
                 : config::sonar::PATH_MAX_STEER_PERCENT);
  int16_t desiredSteerPercent =
      pathPlanner_.desiredSteeringPercent(
          wallSteeringPercent,
          config::servo::CENTER_DEG,
          config::servo::ANGLE_TO_TURN_SIGN,
          maximumSteerPercent,
          config::sonar::STEER_DEADBAND_PERCENT);

  int16_t sideAvoidCommand = 0;
  if (criticalSide) {
    const int16_t sideMagnitude =
        AdaptiveAvoidance::steeringMagnitude(
            sideAvoidDistanceCm_,
            config::sonar::SIDE_AVOID_CM,
            config::sonar::MIN_VALID_CM,
            0,
            config::sonar::AVOID_MIN_STEER_PERCENT,
            config::sonar::SIDE_RESOLUTION_MAX_STEER_PERCENT);
    sideAvoidCommand =
        sideAvoidSteerSign_ * sideMagnitude;
  }

  int32_t avoidPathDirection = 0;
  int16_t frontAvoidCommand = 0;
  if (rollingAvoidMode) {
    avoidPathDirection =
        DifferentialSteering::pathOffsetPercent(
            observedPathAngle,
            config::servo::CENTER_DEG,
            config::servo::ANGLE_TO_TURN_SIGN);
    // The best corridor was selected when avoidance began. Do not reverse
    // that choice on every new echo; this is the key anti-hunting behaviour.
    int8_t frontSteerSign = rollingAvoidSteerSign_;
    if (frontSteerSign == 0) {
      frontSteerSign =
          avoidPathDirection < 0
              ? -1
              : (avoidPathDirection > 0
                     ? 1
                     : (sideDifferenceCm >= 0 ? -1 : 1));
    }
    const int16_t rawPathError =
        static_cast<int16_t>(observedPathAngle) -
        static_cast<int16_t>(config::servo::CENTER_DEG);
    const uint8_t pathErrorDeg =
        static_cast<uint8_t>(rawPathError < 0
                                 ? -rawPathError
                                 : rawPathError);
    const int16_t frontMagnitude =
        AdaptiveAvoidance::steeringMagnitude(
            centerClearanceCm_,
            config::sonar::FRONT_AVOID_CM,
            config::sonar::EMERGENCY_CM,
            pathErrorDeg,
            config::sonar::AVOID_MIN_STEER_PERCENT,
            config::sonar::CRITICAL_MAX_STEER_PERCENT);
    frontAvoidCommand = frontSteerSign * frontMagnitude;
  }

  // Blend front and side risks by keeping the stronger command. Opposing
  // hazards must never cancel into a dangerous straight command.
  if (rollingAvoidMode && criticalSide) {
    desiredSteerPercent =
        abs(frontAvoidCommand) >= abs(sideAvoidCommand)
            ? frontAvoidCommand
            : sideAvoidCommand;
  } else if (rollingAvoidMode) {
    desiredSteerPercent = frontAvoidCommand;
  } else if (criticalSide) {
    desiredSteerPercent = sideAvoidCommand;
  }

  if (rollingAvoidMode || criticalSide) {
    // A blocked forward corridor is time-critical. Bypass the comfort slew
    // and command a decisive rolling curve immediately; both wheels still
    // remain in forward motion through DifferentialSteering::wheelTargets().
    pathPlanner_.forceSteering(desiredSteerPercent);
    lastSteeringUpdateMs_ = nowMs;
  } else if (nowMs - lastSteeringUpdateMs_ >=
             config::sonar::STEERING_UPDATE_MS) {
    pathPlanner_.slewSteering(
        desiredSteerPercent,
        criticalSide
            ? config::sonar::AVOID_STEER_SLEW_STEP_PERCENT
            : config::sonar::STEER_SLEW_STEP_PERCENT);
    lastSteeringUpdateMs_ = nowMs;
  }

  const bool closeSideOverridesStraightRecovery =
      freshThreatSideRay &&
      currentThreatSideRayCm <=
          config::sonar::POST_AVOID_SIDE_OVERRIDE_CM;
  const bool straightRecoveryActive =
      straightRecoveryUntilMs_ != 0 &&
      nowMs < straightRecoveryUntilMs_ &&
      !rollingAvoidMode &&
      !closeSideOverridesStraightRecovery;
  if (straightRecoveryActive) {
    // Cancel residual path/wall commands and immediately request equal wheel
    // targets. MotorDriver then matches both active speeds/PWM at the slower
    // channel, clears both PI integrators and restarts encoder cross-lock.
    pathPlanner_.reset(config::servo::CENTER_DEG);
    desiredSteerPercent = 0;
    lastSteeringUpdateMs_ = nowMs;
  } else if (straightRecoveryUntilMs_ != 0 &&
             (nowMs >= straightRecoveryUntilMs_ ||
              rollingAvoidMode ||
              closeSideOverridesStraightRecovery)) {
    straightRecoveryUntilMs_ = 0;
  }

  if ((rollingAvoidMode || criticalSide) &&
      !straightRecoveryActive) {
    driveTps = AdaptiveAvoidance::baseSpeed(
        config::motor::APPROACH_TPS,
        config::motor::AVOID_CRAWL_TPS,
        pathPlanner_.steeringPercent(),
        config::sonar::AVOID_MIN_STEER_PERCENT,
        config::sonar::CRITICAL_MAX_STEER_PERCENT);
  }

  const bool activeAvoidance =
      (rollingAvoidMode || criticalSide) &&
      !straightRecoveryActive;
  const int16_t wheelSteeringPercent =
      criticalSide && !rollingAvoidMode
          ? pathPlanner_.steeringPercent()
      : activeAvoidance
          ? AdaptiveAvoidance::aggressiveSteering(
                pathPlanner_.steeringPercent(),
                config::sonar::AVOID_INNER_BRAKE_BOOST_PERCENT,
                config::sonar::ROLLING_MAX_STEER_PERCENT)
          : pathPlanner_.steeringPercent();
  wheelSteeringPercent_ = wheelSteeringPercent;
  const int32_t targetSteerOffset =
      (static_cast<int32_t>(wheelSteeringPercent) *
       driveTps) /
      100L;
  const int32_t minForwardTps =
      activeAvoidance
          ? (driveTps *
             config::sonar::ROLLING_MIN_INNER_WHEEL_PERCENT) /
                100L
          : (driveTps * 15L) / 100L;
  const DifferentialWheelTargets wheelTargets =
      DifferentialSteering::wheelTargets(
          driveTps,
          targetSteerOffset,
          minForwardTps);

  motors_.driveDifferential(wheelTargets.leftTps, wheelTargets.rightTps);
}

void RobotController::handleBrake(const uint32_t nowMs) {
  motors_.stop();

  if (forceEscape_) {
    forceEscape_ = false;
    escapeTurn_ = escapeTurn_ == TurnDirection::RIGHT
                      ? TurnDirection::LEFT
                      : TurnDirection::RIGHT;
    transitionTo(RobotState::ESCAPE_REVERSE, nowMs);
    return;
  }

  // Stop and wait 2.0s (2000ms = 1 full back-and-forth radar sweep cycle) for fresh spatial map
  if (nowMs - stateEnteredMs_ >= 2000UL) {
    finishScan(nowMs);
  }
}

void RobotController::beginScan(const uint32_t nowMs) {
  transitionTo(RobotState::SCAN, nowMs);
}

void RobotController::handleScan(const uint32_t nowMs) {
  handleBrake(nowMs);
}

void RobotController::finishScan(const uint32_t nowMs) {
  // Populate 4 discrete gap planner bins directly from live shared spatial map
  scanDistances_[0] = getSpatialClearance(15, 50);   // 30° bin
  scanValid_[0] = true;
  scanDistances_[1] = getSpatialClearance(51, 90);   // 70° bin
  scanValid_[1] = true;
  scanDistances_[2] = getSpatialClearance(91, 130);  // 110° bin
  scanValid_[2] = true;
  scanDistances_[3] = getSpatialClearance(131, 165); // 150° bin
  scanValid_[3] = true;

  const GapSelection gap =
      GapPlanner::select(scanDistances_, scanValid_,
                         config::sonar::SCAN_POINT_COUNT,
                         config::sonar::SCAN_MIN_DEG,
                         config::sonar::SCAN_STEP_DEG,
                         config::servo::CENTER_DEG,
                         config::sonar::OPEN_SPACE_CM,
                         config::sonar::MIN_GAP_POINTS);

  if (!gap.found) {
    if (frontDistanceValid_ && frontDistanceCm_ > config::sonar::STEER_CM) {
      transitionTo(RobotState::DRIVE, nowMs);
      return;
    }
    escapeTurn_ = escapeTurn_ == TurnDirection::RIGHT
                      ? TurnDirection::LEFT
                      : TurnDirection::RIGHT;
    transitionTo(RobotState::ESCAPE_REVERSE, nowMs);
    return;
  }

  selectedAngleDeg_ = gap.targetAngleDeg;
  const int16_t servoError =
      static_cast<int16_t>(selectedAngleDeg_) - config::servo::CENTER_DEG;
  const int16_t signedError =
      servoError * config::servo::ANGLE_TO_TURN_SIGN;
  if (abs(signedError) < 10) {
    selectedTurn_ = TurnDirection::NONE;
    selectedTurnTargetTicks_ = 0;
  } else {
    selectedTurn_ =
        signedError < 0 ? TurnDirection::LEFT : TurnDirection::RIGHT;
    selectedTurnTargetTicks_ =
        MotionGeometry::inPlaceTurnDegreesToTicks(
            fabs(static_cast<float>(signedError)),
            config::motor::WHEEL_DIAMETER_MM,
            config::motor::TRACK_WIDTH_MM,
            config::motor::ENCODER_TICKS_PER_WHEEL_REV);
  }

  transitionTo(RobotState::TURN, nowMs);
}

void RobotController::handleTurn(const uint32_t nowMs) {
  if (selectedTurn_ == TurnDirection::NONE) {
    motors_.stop();
    frontDistanceValid_ = false;
    transitionTo(RobotState::DRIVE, nowMs);
    return;
  }

  if (selectedTurn_ == TurnDirection::LEFT) {
    motors_.turnLeft(config::motor::TURN_TPS);
  } else {
    motors_.turnRight(config::motor::TURN_TPS);
  }

  const uint32_t travelledTicks = motors_.averageAbsoluteTicksSinceMark();
  const uint32_t minimumCommitTicks =
      MotionGeometry::inPlaceTurnDegreesToTicks(
          config::motor::TURN_MIN_COMMIT_DEG,
          config::motor::WHEEL_DIAMETER_MM,
          config::motor::TRACK_WIDTH_MM,
          config::motor::ENCODER_TICKS_PER_WHEEL_REV);
  const uint32_t requiredCommitTicks =
      selectedTurnTargetTicks_ < minimumCommitTicks
          ? selectedTurnTargetTicks_
          : minimumCommitTicks;
  const uint32_t brakeLeadTicks =
      MotionGeometry::inPlaceTurnDegreesToTicks(
          config::motor::TURN_BRAKE_LEAD_DEG,
          config::motor::WHEEL_DIAMETER_MM,
          config::motor::TRACK_WIDTH_MM,
          config::motor::ENCODER_TICKS_PER_WHEEL_REV);
  const uint32_t brakingTargetTicks =
      selectedTurnTargetTicks_ > brakeLeadTicks
          ? selectedTurnTargetTicks_ - brakeLeadTicks
          : selectedTurnTargetTicks_;
  const bool newlyConfirmedClear =
      frontDistanceUpdatedMs_ > stateEnteredMs_ &&
      frontReadingFresh(nowMs) &&
      frontDistanceCm_ >= config::sonar::CLEAR_CM;

  if (travelledTicks >= brakingTargetTicks ||
      (travelledTicks >= requiredCommitTicks && newlyConfirmedClear) ||
      nowMs - stateEnteredMs_ >= config::motor::TURN_TIMEOUT_MS) {
    motors_.emergencyStop();
    frontDistanceValid_ = false;
    transitionTo(RobotState::DRIVE, nowMs);
  }
}

void RobotController::handleEscapeReverse(const uint32_t nowMs) {
  float reverseMillimetres =
      config::motor::ESCAPE_REVERSE_MM;
  if (escapeAttemptCount_ > 1) {
    reverseMillimetres +=
        static_cast<float>(escapeAttemptCount_ - 1U) *
        config::motor::ESCAPE_REVERSE_STEP_MM;
  }
  if (reverseMillimetres >
      config::motor::ESCAPE_REVERSE_MAX_MM) {
    reverseMillimetres =
        config::motor::ESCAPE_REVERSE_MAX_MM;
  }
  const uint32_t reverseTargetTicks =
      MotionGeometry::linearMillimetresToTicks(
          reverseMillimetres,
          config::motor::WHEEL_DIAMETER_MM,
          config::motor::ENCODER_TICKS_PER_WHEEL_REV);
  motors_.driveReverse(config::motor::ESCAPE_TPS);

  const uint32_t travelledTicks = motors_.averageAbsoluteTicksSinceMark();
  if (travelledTicks >= reverseTargetTicks ||
      nowMs - stateEnteredMs_ >= config::motor::ESCAPE_REVERSE_TIMEOUT_MS) {
    motors_.emergencyStop();
    transitionTo(RobotState::ESCAPE_SCAN, nowMs);
  }
}

void RobotController::handleEscapeScan(const uint32_t nowMs) {
  motors_.emergencyStop();

  const uint32_t fullRadarCoverageMask =
      (1UL << config::sonar::RADAR_POINT_COUNT) - 1UL;
  const bool scanComplete =
      radarCoverageMask_ == fullRadarCoverageMask;
  const bool scanTimedOut =
      nowMs - stateEnteredMs_ >= config::motor::ESCAPE_SCAN_TIMEOUT_MS;

  bool hasFailedSector = false;
  int8_t newestFailedSector = -1;
  uint32_t newestFailedTimestamp = 0;
  for (uint8_t sector = 0;
       sector < config::sonar::FAILED_ANGLE_SECTOR_COUNT;
       ++sector) {
    if (failedAngleUpdatedMs_[sector] == 0 ||
        nowMs - failedAngleUpdatedMs_[sector] >
            config::sonar::FAILED_ANGLE_MEMORY_MS) {
      continue;
    }
    hasFailedSector = true;
    if (failedAngleUpdatedMs_[sector] >= newestFailedTimestamp) {
      newestFailedTimestamp = failedAngleUpdatedMs_[sector];
      newestFailedSector = static_cast<int8_t>(sector);
    }
  }

  GapSelection bestGap =
      selectLiveGap(
          nowMs,
          hasFailedSector,
          config::sonar::ESCAPE_CORRIDOR_POINTS,
          false);
  constexpr uint8_t balancedLeftIndex =
      (60 - config::sonar::RADAR_MIN_DEG) /
      config::sonar::RADAR_STEP_DEG;
  constexpr uint8_t balancedRightIndex =
      (120 - config::sonar::RADAR_MIN_DEG) /
      config::sonar::RADAR_STEP_DEG;
  const bool balancedCoverage =
      (radarCoverageMask_ &
       (1UL << balancedLeftIndex)) != 0 &&
      (radarCoverageMask_ &
       (1UL << balancedRightIndex)) != 0;
  const bool earlyCorridorReady =
      nowMs - stateEnteredMs_ >=
          config::sonar::ESCAPE_EARLY_SCAN_MIN_MS &&
      balancedCoverage &&
      bestGap.found;
  if (!scanComplete && !scanTimedOut &&
      !earlyCorridorReady) {
    return;
  }

  float escapeTurnDegrees = config::motor::ESCAPE_TURN_DEG;
  if (bestGap.found &&
      bestGap.targetAngleDeg != config::servo::CENTER_DEG) {
    const int16_t signedError =
        (static_cast<int16_t>(bestGap.targetAngleDeg) -
         config::servo::CENTER_DEG) *
        config::servo::ANGLE_TO_TURN_SIGN;
    escapeTurn_ =
        signedError < 0 ? TurnDirection::LEFT : TurnDirection::RIGHT;
    escapeTurnDegrees =
        fabs(static_cast<float>(signedError)) +
        config::motor::ESCAPE_TURN_MARGIN_DEG;
    if (escapeTurnDegrees < config::motor::ESCAPE_MIN_TURN_DEG) {
      escapeTurnDegrees = config::motor::ESCAPE_MIN_TURN_DEG;
    }
    if (escapeTurnDegrees > config::motor::ESCAPE_TURN_DEG) {
      escapeTurnDegrees = config::motor::ESCAPE_TURN_DEG;
    }
  } else {
    if (newestFailedSector >= 0) {
      const int16_t failedSignedDirection =
          (static_cast<int16_t>(
               failedAngleCenterDeg_[newestFailedSector]) -
           config::servo::CENTER_DEG) *
          config::servo::ANGLE_TO_TURN_SIGN;
      escapeTurn_ =
          failedSignedDirection < 0
              ? TurnDirection::RIGHT
              : TurnDirection::LEFT;
    } else {
      escapeTurn_ =
          leftClearanceCm_ >= rightClearanceCm_
              ? TurnDirection::LEFT
              : TurnDirection::RIGHT;
    }
  }

  if (bestGap.found) {
    selectedAngleDeg_ = bestGap.targetAngleDeg;
  } else if (escapeTurn_ == TurnDirection::LEFT) {
    selectedAngleDeg_ =
        config::servo::ANGLE_TO_TURN_SIGN > 0 ? 60 : 120;
  } else {
    selectedAngleDeg_ =
        config::servo::ANGLE_TO_TURN_SIGN > 0 ? 120 : 60;
  }
  escapeTurnTargetTicks_ =
      MotionGeometry::inPlaceTurnDegreesToTicks(
          escapeTurnDegrees,
          config::motor::WHEEL_DIAMETER_MM,
          config::motor::TRACK_WIDTH_MM,
          config::motor::ENCODER_TICKS_PER_WHEEL_REV);
  transitionTo(RobotState::ESCAPE_TURN, nowMs);
}

void RobotController::handleEscapeTurn(const uint32_t nowMs) {
  if (escapeTurnTargetTicks_ == 0) {
    escapeTurnTargetTicks_ =
        MotionGeometry::inPlaceTurnDegreesToTicks(
            config::motor::ESCAPE_TURN_DEG,
            config::motor::WHEEL_DIAMETER_MM,
            config::motor::TRACK_WIDTH_MM,
            config::motor::ENCODER_TICKS_PER_WHEEL_REV);
  }
  const uint32_t minimumTurnTicks =
      MotionGeometry::inPlaceTurnDegreesToTicks(
          config::motor::ESCAPE_MIN_TURN_DEG,
          config::motor::WHEEL_DIAMETER_MM,
          config::motor::TRACK_WIDTH_MM,
          config::motor::ENCODER_TICKS_PER_WHEEL_REV);
  const uint32_t brakeLeadTicks =
      MotionGeometry::inPlaceTurnDegreesToTicks(
          config::motor::TURN_BRAKE_LEAD_DEG,
          config::motor::WHEEL_DIAMETER_MM,
          config::motor::TRACK_WIDTH_MM,
          config::motor::ENCODER_TICKS_PER_WHEEL_REV);
  const uint32_t brakingTargetTicks =
      escapeTurnTargetTicks_ > brakeLeadTicks
          ? escapeTurnTargetTicks_ - brakeLeadTicks
          : escapeTurnTargetTicks_;
  if (escapeTurn_ == TurnDirection::LEFT) {
    motors_.turnLeft(config::motor::TURN_TPS);
  } else {
    motors_.turnRight(config::motor::TURN_TPS);
  }

  const uint32_t travelledTicks = motors_.averageAbsoluteTicksSinceMark();
  const bool newlyConfirmedClear =
      frontDistanceUpdatedMs_ > stateEnteredMs_ &&
      frontReadingFresh(nowMs) &&
      frontDistanceCm_ >= config::sonar::CLEAR_CM;
  const bool allowEarlyClear =
      escapeAttemptCount_ <= 1;
  if (travelledTicks >= brakingTargetTicks ||
      (allowEarlyClear &&
       travelledTicks >= minimumTurnTicks &&
       newlyConfirmedClear) ||
      nowMs - stateEnteredMs_ >= config::motor::ESCAPE_TURN_TIMEOUT_MS) {
    motors_.emergencyStop();
    frontDistanceValid_ = false;
    transitionTo(RobotState::DRIVE, nowMs);
  }
}

void RobotController::handleLowBattery() {
  motors_.emergencyStop();
  if (!battery_.cutoff()) {
    transitionTo(RobotState::STARTUP, millis());
  }
}

void RobotController::handleFault(const uint32_t nowMs) {
  motors_.emergencyStop();

  // Reset retry counter if 30 seconds have passed without faults
  if (nowMs - lastFaultMs_ > 30000UL) {
    faultRetryCount_ = 0;
  }

  // After 3 seconds cool-down: attempt Auto-Recovery
  if (nowMs - stateEnteredMs_ >= 3000UL) {
    if (faultRetryCount_ < 3) {
      ++faultRetryCount_;
      lastFaultMs_ = nowMs;
      motors_.reset();
      frontDistanceValid_ = false;
      escapeTurn_ = (escapeTurn_ == TurnDirection::RIGHT)
                        ? TurnDirection::LEFT
                        : TurnDirection::RIGHT;
      transitionTo(RobotState::ESCAPE_REVERSE, nowMs);
    }
  }
}

void RobotController::recordAvoidEvent(const uint32_t nowMs) {
  if (nowMs - avoidWindowStartedMs_ > config::safety::AVOID_WINDOW_MS) {
    avoidWindowStartedMs_ = nowMs;
    avoidEvents_ = 0;
  }
  ++avoidEvents_;
  if (avoidEvents_ >= config::safety::MAX_AVOID_EVENTS) {
    forceEscape_ = true;
    avoidEvents_ = 0;
    avoidWindowStartedMs_ = nowMs;
  }
}

uint16_t RobotController::medianSamples(const uint16_t* values,
                                        const uint8_t count) {
  uint16_t sorted[config::sonar::SAMPLES_PER_ANGLE]{};
  for (uint8_t i = 0; i < count; ++i) {
    sorted[i] = values[i];
  }
  for (uint8_t i = 1; i < count; ++i) {
    const uint16_t value = sorted[i];
    uint8_t j = i;
    while (j > 0 && sorted[j - 1U] > value) {
      sorted[j] = sorted[j - 1U];
      --j;
    }
    sorted[j] = value;
  }
  return sorted[count / 2U];
}

void RobotController::updateDisplay(const uint32_t nowMs) {
  const uint32_t cappedElapsedMs =
      nowMs < static_cast<uint32_t>(config::pms::WARMUP_MS)
          ? nowMs
          : static_cast<uint32_t>(config::pms::WARMUP_MS);
  const uint32_t warmupRemainingMs = pms_.warmedUp(nowMs)
                                         ? 0
                                         : config::pms::WARMUP_MS -
                                               cappedElapsedMs;
  const uint32_t fullRadarCoverageMask =
      (1UL << config::sonar::RADAR_POINT_COUNT) - 1UL;
  const bool radarReady =
      radarCoverageMask_ == fullRadarCoverageMask &&
      currentServoAngle_ == config::servo::CENTER_DEG &&
      frontReadingFresh(nowMs);
  const bool freshFront = frontDistanceValid_ && frontReadingFresh(nowMs);
  const bool displayedBatteryValid = selectedBatteryVoltageValid();
  const float displayedBatteryVoltage = selectedBatteryVoltage();
  const RobotDisplayData displayData{
      state_,
      static_cast<uint16_t>((warmupRemainingMs + 999UL) / 1000UL),
      pms_.healthy(nowMs),
      pms_.pm25Average(),
      pms_.pm10Average(),
      fan_.level(),
      fan_.percent(),
      displayedBatteryValid,
      battery_.warning(),
      displayedBatteryVoltage,
      batterySoc_.valid(),
      batterySoc_.percent(),
      charge_.ready(),
      charge_.hasReading(),
      chargerPresent_,
      chargeManager_.state(),
      charge_.currentMilliamp(),
      freshFront,
      frontDistanceCm_,
      radarReady,
      rollingFrontAvoidActive_,
      sideAvoidActive_,
      corridorCommitment_.sign(),
      wheelSteeringPercent_,
  };
  display_.update(nowMs, displayData);
}

void RobotController::reportStatus(const uint32_t nowMs) {
  constexpr uint32_t REPORT_INTERVAL_MS = 1000;
  if (nowMs - lastStatusReportMs_ < REPORT_INTERVAL_MS) {
    return;
  }
  lastStatusReportMs_ = nowMs;
  const bool chargeVoltageValid = charge_.hasReading() &&
                                  charge_.busVoltage() > 1.0F;
  const float displayedBatteryVoltage = selectedBatteryVoltage();
  Serial.printf("state=%u adapter=%u drive_locked=%u pm25=%u pm10=%u pms_ok=%u "
                "fan=%u/%u%% battery=%.2fV "
                "soc=%s%u%% divider=%.2fV charge_v=%s%.2fV charge_state=%u "
                "charge=%s%+.0fmA %.1fW session=%.1fmAh "
                "full_mem=%u history=%u full_count=%lu front=%ucm "
                "servo=%u left=%u center=%u right=%u path=%u steer=%d%% "
                "avoid_dir=%d avoid_fix=%u side_dir=%d threat_scan=%u radar_focus=%u "
                "corridor_fix=%d/%ucm motion_watch=%u tps_ctl=%u "
                "wheel_tps=%ld/%ld pwm=%d/%d\n",
                static_cast<unsigned>(state_),
                chargerPresent_ ? 1U : 0U,
                chargerPresent_ ? 1U : 0U,
                static_cast<unsigned>(pms_.pm25Average()),
                static_cast<unsigned>(pms_.pm10Average()),
                pms_.healthy(nowMs) ? 1U : 0U,
                static_cast<unsigned>(fan_.level()),
                static_cast<unsigned>(fan_.percent()),
                displayedBatteryVoltage,
                batterySoc_.valid() ? "EST:" : "?",
                static_cast<unsigned>(batterySoc_.percent()),
                battery_.voltage(), chargeVoltageValid ? "" : "?",
                charge_.busVoltage(),
                static_cast<unsigned>(chargeManager_.state()),
                charge_.hasReading() ? "" : "?",
                charge_.currentMilliamp(), charge_.powerMilliwatt() / 1000.0F,
                chargeManager_.sessionChargedMah(),
                chargeManager_.wasFullyCharged() ? 1U : 0U,
                chargeManager_.historyValid() ? 1U : 0U,
                static_cast<unsigned long>(
                    chargeManager_.fullConfirmationCount()),
                static_cast<unsigned>(frontDistanceCm_),
                static_cast<unsigned>(currentServoAngle_),
                static_cast<unsigned>(leftClearanceCm_),
                static_cast<unsigned>(centerClearanceCm_),
                static_cast<unsigned>(rightClearanceCm_),
                static_cast<unsigned>(livePathAngleDeg_),
                static_cast<int>(wheelSteeringPercent_),
                static_cast<int>(rollingAvoidSteerSign_),
                rollingAvoidDirectionCorrectionUsed_ ? 1U : 0U,
                static_cast<int>(sideAvoidSteerSign_),
                radarThreatScanActive_ ? 1U : 0U,
                radarAnomalyFocusActive_ ? 1U : 0U,
                static_cast<int>(corridorCommitment_.sign()),
                static_cast<unsigned>(
                    corridorCommitment_.baselineErrorCm()),
                radarStuckWatchActive_ ? 1U : 0U,
                config::motor::TPS_FEEDBACK_ENABLED ? 1U : 0U,
                static_cast<long>(motors_.leftMeasuredTps()),
                static_cast<long>(motors_.rightMeasuredTps()),
                static_cast<int>(motors_.leftAppliedPwm()),
                static_cast<int>(motors_.rightAppliedPwm()));
}
