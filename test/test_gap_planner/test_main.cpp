#include <unity.h>

#include "AdaptiveAvoidance.h"
#include "CorridorCommitment.h"
#include "DifferentialSteering.h"
#include "GapPlanner.h"
#include "MotionGeometry.h"
#include "RadarFilter.h"
#include "RadarMotionVerifier.h"
#include "StablePathPlanner.h"

void test_selects_middle_of_widest_gap() {
  const uint16_t distances[] = {20, 25, 80, 100, 120, 110, 90, 20, 15};
  const bool valid[] = {true, true, true, true, true, true, true, true, true};

  const GapSelection result =
      GapPlanner::select(distances, valid, 9, 50, 10, 90, 55, 3);

  TEST_ASSERT_TRUE(result.found);
  TEST_ASSERT_EQUAL_UINT8(90, result.targetAngleDeg);
  TEST_ASSERT_EQUAL_UINT8(2, result.startIndex);
  TEST_ASSERT_EQUAL_UINT8(6, result.endIndex);
}

void test_rejects_narrow_opening() {
  const uint16_t distances[] = {20, 20, 100, 100, 20, 20, 20};
  const bool valid[] = {true, true, true, true, true, true, true};

  const GapSelection result =
      GapPlanner::select(distances, valid, 7, 60, 10, 90, 55, 3);

  TEST_ASSERT_FALSE(result.found);
}

void test_invalid_points_split_gap() {
  const uint16_t distances[] = {100, 100, 100, 120, 120, 120, 120};
  const bool valid[] = {true, true, false, true, true, true, true};

  const GapSelection result =
      GapPlanner::select(distances, valid, 7, 60, 10, 90, 55, 3);

  TEST_ASSERT_TRUE(result.found);
  TEST_ASSERT_EQUAL_UINT8(100, result.targetAngleDeg);
  TEST_ASSERT_EQUAL_UINT8(3, result.startIndex);
  TEST_ASSERT_EQUAL_UINT8(6, result.endIndex);
}

void test_selects_deepest_three_ray_corridor() {
  const uint16_t distances[] =
      {220, 210, 190, 90, 80, 75, 70, 65, 60};
  const bool valid[] =
      {true, true, true, true, true, true, true, true, true};

  const GapSelection result =
      GapPlanner::selectBestCorridor(
          distances, valid, 9, 30, 15, 90, 35, 3);

  TEST_ASSERT_TRUE(result.found);
  TEST_ASSERT_EQUAL_UINT8(45, result.targetAngleDeg);
  TEST_ASSERT_EQUAL_UINT8(0, result.startIndex);
  TEST_ASSERT_EQUAL_UINT8(2, result.endIndex);
}

void test_corridor_requires_consecutive_open_rays() {
  const uint16_t distances[] =
      {220, 20, 220, 20, 220, 20, 220, 20, 220};
  const bool valid[] =
      {true, true, true, true, true, true, true, true, true};

  const GapSelection result =
      GapPlanner::selectBestCorridor(
          distances, valid, 9, 30, 15, 90, 35, 3);

  TEST_ASSERT_FALSE(result.found);
}

void test_corridor_score_prefers_wider_safe_opening() {
  // The right-hand opening is slightly deeper but only exactly chassis-wide.
  // A six-ray opening on the left is the safer navigation choice because it
  // tolerates steering and ultrasonic measurement error.
  const uint16_t distances[] =
      {150, 150, 150, 150, 150, 150, 10, 170, 170, 170, 10, 10, 10};
  const bool valid[] =
      {true, true, true, true, true, true, true,
       true, true, true, true, true, true};

  const GapSelection result =
      GapPlanner::selectBestCorridor(
          distances, valid, 13, 30, 10, 90, 35, 3);

  TEST_ASSERT_TRUE(result.found);
  TEST_ASSERT_EQUAL_UINT8(70, result.targetAngleDeg);
  TEST_ASSERT_LESS_THAN_UINT8(6, result.endIndex);
}

void test_radar_focuses_on_new_close_distance_drop() {
  TEST_ASSERT_TRUE(
      RadarFilter::shouldFocus(240, 80, true, 110, 25));
  TEST_ASSERT_TRUE(
      RadarFilter::shouldFocus(0, 70, false, 110, 25));
}

void test_radar_does_not_pause_for_distant_or_small_changes() {
  TEST_ASSERT_FALSE(
      RadarFilter::shouldFocus(240, 180, true, 110, 25));
  TEST_ASSERT_FALSE(
      RadarFilter::shouldFocus(100, 85, true, 110, 25));
  TEST_ASSERT_FALSE(
      RadarFilter::shouldFocus(70, 95, true, 110, 25));
}

void test_escape_corridor_rejects_sonar_only_slit() {
  const uint16_t distances[] =
      {20, 100, 100, 100, 100, 100, 20, 20, 20};
  const bool valid[] =
      {true, true, true, true, true, true, true, true, true};

  const GapSelection result =
      GapPlanner::selectBestCorridor(
          distances, valid, 9, 50, 5, 90, 35, 7);

  TEST_ASSERT_FALSE(result.found);
}

void test_escape_corridor_accepts_chassis_width_run() {
  const uint16_t distances[] =
      {20, 100, 100, 100, 100, 100, 100, 100, 20};
  const bool valid[] =
      {true, true, true, true, true, true, true, true, true};

  const GapSelection result =
      GapPlanner::selectBestCorridor(
          distances, valid, 9, 50, 5, 90, 35, 7);

  TEST_ASSERT_TRUE(result.found);
  TEST_ASSERT_EQUAL_UINT8(70, result.targetAngleDeg);
}

void test_failed_heading_penalty_does_not_split_open_corridor() {
  TEST_ASSERT_EQUAL_UINT16(
      35,
      GapPlanner::applyDirectionalPenalty(
          40, 0, 20, 35, 55, 85));
  TEST_ASSERT_EQUAL_UINT16(
      165,
      GapPlanner::applyDirectionalPenalty(
          300, 0, 20, 35, 55, 85));
  TEST_ASSERT_EQUAL_UINT16(
      255,
      GapPlanner::applyDirectionalPenalty(
          300, 20, 20, 35, 55, 85));
}

void test_failed_heading_penalty_keeps_obstacle_blocked() {
  TEST_ASSERT_LESS_THAN_UINT16(
      35,
      GapPlanner::applyDirectionalPenalty(
          20, 0, 20, 35, 55, 85));
}

void test_motion_geometry_converts_wheel_travel() {
  const uint32_t ticks =
      MotionGeometry::linearMillimetresToTicks(204.2F, 65.0F, 660);
  TEST_ASSERT_UINT32_WITHIN(2, 660, ticks);
}

void test_motion_geometry_converts_in_place_turn() {
  const uint32_t ticks =
      MotionGeometry::inPlaceTurnDegreesToTicks(90.0F, 65.0F, 230.0F, 660);
  TEST_ASSERT_UINT32_WITHIN(2, 584, ticks);
}

void test_positive_steering_offset_matches_robot_hardware() {
  const DifferentialWheelTargets targets =
      DifferentialSteering::wheelTargets(1000, 300, 150);

  TEST_ASSERT_EQUAL_INT32(1300, targets.leftTps);
  TEST_ASSERT_EQUAL_INT32(700, targets.rightTps);
  TEST_ASSERT_TRUE(targets.leftTps > targets.rightTps);
}

void test_negative_steering_offset_matches_robot_hardware() {
  const DifferentialWheelTargets targets =
      DifferentialSteering::wheelTargets(1000, -300, 150);

  TEST_ASSERT_EQUAL_INT32(700, targets.leftTps);
  TEST_ASSERT_EQUAL_INT32(1300, targets.rightTps);
  TEST_ASSERT_TRUE(targets.rightTps > targets.leftTps);
}

void test_steering_keeps_both_wheels_moving_forward() {
  const DifferentialWheelTargets targets =
      DifferentialSteering::wheelTargets(1000, 1200, 150);

  TEST_ASSERT_EQUAL_INT32(2200, targets.leftTps);
  TEST_ASSERT_EQUAL_INT32(150, targets.rightTps);
}

void test_close_left_wall_strengthens_early_correction() {
  const int32_t offset =
      DifferentialSteering::offsetPercent(80, 100, 90, 75);

  TEST_ASSERT_EQUAL_INT32(30, offset);
}

void test_close_right_wall_strengthens_early_correction() {
  const int32_t offset =
      DifferentialSteering::offsetPercent(100, 80, 90, 75);

  TEST_ASSERT_EQUAL_INT32(-30, offset);
}

void test_wall_correction_is_safely_limited() {
  TEST_ASSERT_EQUAL_INT32(
      75, DifferentialSteering::offsetPercent(20, 300, 90, 75));
  TEST_ASSERT_EQUAL_INT32(
      -75, DifferentialSteering::offsetPercent(300, 20, 90, 75));
}

void test_left_path_produces_left_steering_offset() {
  const int32_t offset =
      DifferentialSteering::pathOffsetPercent(45, 90, 1);
  const DifferentialWheelTargets targets =
      DifferentialSteering::wheelTargets(1000, offset * 10, 150);

  TEST_ASSERT_TRUE(offset < 0);
  TEST_ASSERT_TRUE(targets.leftTps < targets.rightTps);
}

void test_right_path_produces_right_steering_offset() {
  const int32_t offset =
      DifferentialSteering::pathOffsetPercent(135, 90, 1);
  const DifferentialWheelTargets targets =
      DifferentialSteering::wheelTargets(1000, offset * 10, 150);

  TEST_ASSERT_TRUE(offset > 0);
  TEST_ASSERT_TRUE(targets.leftTps > targets.rightTps);
}

void test_path_change_requires_repeated_wins() {
  StablePathPlanner planner;
  planner.reset(90);

  planner.observeTarget(45, 90, false, 3, 5);
  planner.observeTarget(45, 90, false, 3, 5);
  TEST_ASSERT_EQUAL_UINT8(90, planner.targetAngleDeg());

  planner.observeTarget(45, 90, false, 3, 5);
  TEST_ASSERT_EQUAL_UINT8(85, planner.targetAngleDeg());
}

void test_normal_path_can_respond_in_two_wins() {
  StablePathPlanner planner;
  planner.reset(90);

  planner.observeTarget(45, 90, false, 2, 10);
  planner.observeTarget(45, 90, false, 2, 10);

  TEST_ASSERT_EQUAL_UINT8(80, planner.targetAngleDeg());
}

void test_urgent_path_change_moves_on_first_observation() {
  StablePathPlanner planner;
  planner.reset(90);

  planner.observeTarget(45, 90, false, 1, 12);

  TEST_ASSERT_EQUAL_UINT8(78, planner.targetAngleDeg());
}

void test_corridor_mode_locks_target_straight_ahead() {
  StablePathPlanner planner;
  planner.reset(90);
  for (uint8_t i = 0; i < 3; ++i) {
    planner.observeTarget(45, 90, false, 3, 5);
  }
  TEST_ASSERT_EQUAL_UINT8(85, planner.targetAngleDeg());

  planner.observeTarget(45, 90, true, 3, 5);
  TEST_ASSERT_EQUAL_UINT8(90, planner.targetAngleDeg());
}

void test_steering_slew_prevents_instant_zigzag_reversal() {
  StablePathPlanner planner;
  planner.reset(90);

  TEST_ASSERT_EQUAL_INT16(6, planner.slewSteering(45, 6));
  TEST_ASSERT_EQUAL_INT16(12, planner.slewSteering(45, 6));
  TEST_ASSERT_EQUAL_INT16(6, planner.slewSteering(-45, 6));
  TEST_ASSERT_EQUAL_INT16(0, planner.slewSteering(-45, 6));
  TEST_ASSERT_EQUAL_INT16(-6, planner.slewSteering(-45, 6));
}

void test_small_steering_error_is_deadbanded_to_zero() {
  StablePathPlanner planner;
  planner.reset(90);

  TEST_ASSERT_EQUAL_INT16(
      0, planner.desiredSteeringPercent(4, 90, 1, 45, 5));
}

void test_emergency_rolling_avoid_can_force_steering() {
  StablePathPlanner planner;
  planner.reset(90);

  planner.forceSteering(-65);

  TEST_ASSERT_EQUAL_INT16(-65, planner.steeringPercent());
}

void test_reset_clears_residual_crab_steering() {
  StablePathPlanner planner;
  planner.reset(90);
  planner.forceSteering(65);

  planner.reset(90);

  TEST_ASSERT_EQUAL_UINT8(90, planner.targetAngleDeg());
  TEST_ASSERT_EQUAL_INT16(0, planner.steeringPercent());
}

void test_adaptive_avoidance_increases_as_distance_closes() {
  const int16_t farSteer =
      AdaptiveAvoidance::steeringMagnitude(
          120, 120, 30, 0, 35, 95);
  const int16_t mediumSteer =
      AdaptiveAvoidance::steeringMagnitude(
          75, 120, 30, 0, 35, 95);
  const int16_t nearSteer =
      AdaptiveAvoidance::steeringMagnitude(
          30, 120, 30, 0, 35, 95);

  TEST_ASSERT_EQUAL_INT16(35, farSteer);
  TEST_ASSERT_GREATER_THAN_INT16(farSteer, mediumSteer);
  TEST_ASSERT_EQUAL_INT16(95, nearSteer);
}

void test_adaptive_avoidance_adds_path_angle_authority() {
  const int16_t straightGap =
      AdaptiveAvoidance::steeringMagnitude(
          100, 120, 30, 0, 35, 95);
  const int16_t sideGap =
      AdaptiveAvoidance::steeringMagnitude(
          100, 120, 30, 60, 35, 95);

  TEST_ASSERT_GREATER_THAN_INT16(straightGap, sideGap);
  TEST_ASSERT_LESS_OR_EQUAL_INT16(95, sideGap);
}

void test_adaptive_avoidance_slows_base_speed_for_hard_turns() {
  TEST_ASSERT_EQUAL_INT32(
      3000, AdaptiveAvoidance::baseSpeed(3000, 1200, 35, 35, 95));
  const int32_t mediumSpeed =
      AdaptiveAvoidance::baseSpeed(3000, 1200, 65, 35, 95);
  TEST_ASSERT_LESS_THAN_INT32(3000, mediumSpeed);
  TEST_ASSERT_GREATER_THAN_INT32(1200, mediumSpeed);
  TEST_ASSERT_EQUAL_INT32(
      1200, AdaptiveAvoidance::baseSpeed(3000, 1200, 95, 35, 95));
}

void test_aggressive_avoidance_can_stop_inside_wheel() {
  TEST_ASSERT_EQUAL_INT16(
      65, AdaptiveAvoidance::aggressiveSteering(35, 30));
  TEST_ASSERT_EQUAL_INT16(
      -95, AdaptiveAvoidance::aggressiveSteering(-65, 30));
  TEST_ASSERT_EQUAL_INT16(
      100, AdaptiveAvoidance::aggressiveSteering(85, 30));
  TEST_ASSERT_EQUAL_INT16(
      -100, AdaptiveAvoidance::aggressiveSteering(-95, 30));
}

void test_tyre_safe_rolling_avoidance_limits_steering() {
  TEST_ASSERT_EQUAL_INT16(
      65, AdaptiveAvoidance::aggressiveSteering(35, 30, 75));
  TEST_ASSERT_EQUAL_INT16(
      75, AdaptiveAvoidance::aggressiveSteering(70, 30, 75));
  TEST_ASSERT_EQUAL_INT16(
      -75, AdaptiveAvoidance::aggressiveSteering(-95, 30, 75));

  const DifferentialWheelTargets rightTurn =
      DifferentialSteering::wheelTargets(1000, 750, 250);
  TEST_ASSERT_EQUAL_INT32(1750, rightTurn.leftTps);
  TEST_ASSERT_EQUAL_INT32(250, rightTurn.rightTps);
}

void test_corridor_commits_until_current_side_is_mostly_corrected() {
  CorridorCommitment correction;
  TEST_ASSERT_EQUAL_INT8(
      1, correction.update(1000, 40, 70, true, true,
                           8, 60, 350, 1800, 450));
  TEST_ASSERT_EQUAL_UINT16(30, correction.baselineErrorCm());

  // A contradictory echo during the minimum commitment cannot reverse the
  // wheels immediately.
  TEST_ASSERT_EQUAL_INT8(
      1, correction.update(1200, 75, 55, true, true,
                           8, 60, 350, 1800, 450));

  // Baseline 30 cm reduced to 12 cm means 60% of the original error is gone.
  TEST_ASSERT_EQUAL_INT8(
      0, correction.update(1400, 54, 66, true, true,
                           8, 60, 350, 1800, 450));
  // Stay neutral through the settling period, even if the other side now
  // appears closer.
  TEST_ASSERT_EQUAL_INT8(
      0, correction.update(1600, 75, 45, true, true,
                           8, 60, 350, 1800, 450));
  TEST_ASSERT_EQUAL_INT8(
      -1, correction.update(1900, 75, 45, true, true,
                            8, 60, 350, 1800, 450));
}

void test_corridor_does_not_start_from_unpaired_side_echoes() {
  CorridorCommitment correction;
  TEST_ASSERT_EQUAL_INT8(
      0, correction.update(1000, 30, 80, true, false,
                           8, 60, 350, 1800, 450));
  TEST_ASSERT_EQUAL_INT8(
      1, correction.update(1100, 30, 80, true, true,
                           8, 60, 350, 1800, 450));
  TEST_ASSERT_EQUAL_INT8(
      0, correction.update(3000, 30, 80, true, false,
                           8, 60, 350, 1800, 450));
}

void test_radar_motion_verifier_requires_real_scene_evidence() {
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(RadarMotionResult::INSUFFICIENT),
      static_cast<uint8_t>(RadarMotionVerifier::verify(
          500, 1000, 5, 5, 1, 1, 3, true)));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(RadarMotionResult::INSUFFICIENT),
      static_cast<uint8_t>(RadarMotionVerifier::verify(
          1500, 1000, 4, 4, 0, 0, 3, true)));
}

void test_radar_motion_verifier_detects_wheel_spin_without_chassis_motion() {
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(RadarMotionResult::STUCK),
      static_cast<uint8_t>(RadarMotionVerifier::verify(
          1500, 1000, 4, 4, 1, 1, 3, true)));
  // While turning, any three re-observed real rays should change. Forward
  // evidence is not mandatory because the chassis heading itself is moving.
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(RadarMotionResult::STUCK),
      static_cast<uint8_t>(RadarMotionVerifier::verify(
          1500, 1000, 3, 3, 0, 0, 3, false)));
}

void test_radar_motion_verifier_accepts_one_changing_ray_as_motion() {
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(RadarMotionResult::SCENE_CHANGED),
      static_cast<uint8_t>(RadarMotionVerifier::verify(
          1500, 1000, 5, 4, 1, 0, 3, true)));
}

void test_side_resolution_uses_sixty_percent_without_stopping_inner_wheel() {
  const int16_t correction =
      AdaptiveAvoidance::steeringMagnitude(
          5, 75, 5, 0, 35, 60);
  TEST_ASSERT_EQUAL_INT16(60, correction);

  const DifferentialWheelTargets rightCorrection =
      DifferentialSteering::wheelTargets(1000, 600, 250);
  TEST_ASSERT_EQUAL_INT32(1600, rightCorrection.leftTps);
  TEST_ASSERT_EQUAL_INT32(400, rightCorrection.rightTps);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_selects_middle_of_widest_gap);
  RUN_TEST(test_rejects_narrow_opening);
  RUN_TEST(test_invalid_points_split_gap);
  RUN_TEST(test_selects_deepest_three_ray_corridor);
  RUN_TEST(test_corridor_requires_consecutive_open_rays);
  RUN_TEST(test_corridor_score_prefers_wider_safe_opening);
  RUN_TEST(test_radar_focuses_on_new_close_distance_drop);
  RUN_TEST(test_radar_does_not_pause_for_distant_or_small_changes);
  RUN_TEST(test_escape_corridor_rejects_sonar_only_slit);
  RUN_TEST(test_escape_corridor_accepts_chassis_width_run);
  RUN_TEST(test_failed_heading_penalty_does_not_split_open_corridor);
  RUN_TEST(test_failed_heading_penalty_keeps_obstacle_blocked);
  RUN_TEST(test_motion_geometry_converts_wheel_travel);
  RUN_TEST(test_motion_geometry_converts_in_place_turn);
  RUN_TEST(test_positive_steering_offset_matches_robot_hardware);
  RUN_TEST(test_negative_steering_offset_matches_robot_hardware);
  RUN_TEST(test_steering_keeps_both_wheels_moving_forward);
  RUN_TEST(test_close_left_wall_strengthens_early_correction);
  RUN_TEST(test_close_right_wall_strengthens_early_correction);
  RUN_TEST(test_wall_correction_is_safely_limited);
  RUN_TEST(test_left_path_produces_left_steering_offset);
  RUN_TEST(test_right_path_produces_right_steering_offset);
  RUN_TEST(test_path_change_requires_repeated_wins);
  RUN_TEST(test_normal_path_can_respond_in_two_wins);
  RUN_TEST(test_urgent_path_change_moves_on_first_observation);
  RUN_TEST(test_corridor_mode_locks_target_straight_ahead);
  RUN_TEST(test_steering_slew_prevents_instant_zigzag_reversal);
  RUN_TEST(test_small_steering_error_is_deadbanded_to_zero);
  RUN_TEST(test_emergency_rolling_avoid_can_force_steering);
  RUN_TEST(test_reset_clears_residual_crab_steering);
  RUN_TEST(test_adaptive_avoidance_increases_as_distance_closes);
  RUN_TEST(test_adaptive_avoidance_adds_path_angle_authority);
  RUN_TEST(test_adaptive_avoidance_slows_base_speed_for_hard_turns);
  RUN_TEST(test_aggressive_avoidance_can_stop_inside_wheel);
  RUN_TEST(test_tyre_safe_rolling_avoidance_limits_steering);
  RUN_TEST(test_corridor_commits_until_current_side_is_mostly_corrected);
  RUN_TEST(test_corridor_does_not_start_from_unpaired_side_echoes);
  RUN_TEST(test_radar_motion_verifier_requires_real_scene_evidence);
  RUN_TEST(test_radar_motion_verifier_detects_wheel_spin_without_chassis_motion);
  RUN_TEST(test_radar_motion_verifier_accepts_one_changing_ray_as_motion);
  RUN_TEST(test_side_resolution_uses_sixty_percent_without_stopping_inner_wheel);
  return UNITY_END();
}
