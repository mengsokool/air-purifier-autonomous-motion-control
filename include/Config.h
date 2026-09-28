#pragma once

#include <stdint.h>

namespace config
{

    // ---------------------------------------------------------------------------
    // Pin map for an ESP32-S3-WROOM-1 N16R8 board compatible with DevKitC-1.
    // GPIO19/20 remain free for native USB. GPIO0, GPIO3, GPIO45 and GPIO46
    // are boot/strapping pins and are intentionally not connected to devices.
    // All peripheral logic signals must be 3.3 V; level-shift 5 V devices.
    // ---------------------------------------------------------------------------
    namespace pin
    {
        constexpr int8_t PMS_RX = 18; // PMS3003 TX -> UART1 RX.
        constexpr int8_t PMS_TX = -1; // PMS3003 RX is not required.
        // The LCD and INA219 use separate I2C controllers in the assembled robot.
        // The 5 V LCD remains behind its level shifter on Wire (I2C0).
        constexpr uint8_t LCD_SDA = 17;
        constexpr uint8_t LCD_SCL = 10;
        // INA219 is a 3.3 V device on Wire1 (I2C1), without the LCD level shifter.
        constexpr uint8_t INA219_SDA = 8;
        constexpr uint8_t INA219_SCL = 9;
        constexpr uint8_t ULTRASONIC_ECHO = 7;
        constexpr uint8_t ULTRASONIC_TRIGGER = 6;
        constexpr uint8_t FAN_PWM = 5;
        constexpr uint8_t MOTOR_LEFT_PWM = 4;
        constexpr uint8_t MOTOR_LEFT_IN1 = 11;
        constexpr uint8_t MOTOR_LEFT_IN2 = 12;
        constexpr uint8_t SCANNER_SERVO = 13;
        constexpr uint8_t MOTOR_RIGHT_PWM = 14;
        constexpr uint8_t MOTOR_RIGHT_IN1 = 15;
        constexpr uint8_t MOTOR_RIGHT_IN2 = 16;
        constexpr uint8_t BATTERY_SENSE = 1;
        // Live PWM/encoder correlation plus the observed physical turn prove
        // the encoder cable groups are crossed: GPIO38/39 is the physical
        // left wheel and GPIO2/21 is the physical right wheel.
        constexpr uint8_t ENCODER_LEFT_A = 38;
        constexpr uint8_t ENCODER_LEFT_B = 39;
        constexpr uint8_t ENCODER_RIGHT_A = 2;
        constexpr uint8_t ENCODER_RIGHT_B = 21;
        // 18 V adapter detector: 180 kOhm from adapter+ to this pin and
        // 33 kOhm from this pin to GND. HIGH means an adapter is connected.
        // GPIO42 is intentionally only a runtime status input; the power-OR
        // wiring boots the ESP32 while charging even when the main switch is off.
        constexpr uint8_t CHARGER_PRESENT = 42;
    } // namespace pin

    namespace serial
    {
        constexpr uint8_t PMS_UART_NUMBER = 1;
    } // namespace serial

    namespace i2c
    {
        constexpr uint32_t FREQUENCY_HZ = 100000UL;
    } // namespace i2c

    namespace motor
    {
        // Reserve LEDC timer 1 (channels 2 and 3) exclusively for the drive
        // motors.  Do not use analogWrite() here: Arduino allocates ESP32-S3
        // channels from 7 downward, which collides with FAN_PWM_CHANNEL 7.
        constexpr uint8_t PWM_LEFT_CHANNEL = 2;
        constexpr uint8_t PWM_RIGHT_CHANNEL = 3;
        constexpr uint32_t PWM_FREQUENCY_HZ = 1000UL;
        constexpr uint8_t PWM_RESOLUTION_BITS = 8;

        // Verified on the assembled chassis: equal positive targets with both
        // channels non-inverted drive both tyre contact patches forward.
        constexpr bool LEFT_INVERTED = false;
        constexpr bool RIGHT_INVERTED = false;
        // Encoder signs must make both counters increase while the robot drives
        // forward. Change an encoder inversion flag instead of swapping random pins.
        constexpr bool ENCODER_LEFT_INVERTED = true;
        constexpr bool ENCODER_RIGHT_INVERTED = false;

        // PCNT samples inputs at the 80 MHz APB clock. Reject pulses shorter
        // than 1 us (80 cycles), which removes cable/brush noise without
        // affecting the millisecond-scale pulses from the wheel encoders.
        constexpr uint16_t ENCODER_GLITCH_FILTER_CYCLES = 80;

        // JGB37-520 12 RPM: 11 pulses/motor rev * 810:1 gearbox * x4 quadrature
        // decoding is nominally 35,640 ticks/output-shaft revolution. The configured
        // 35,600 value is the measured/calibrated near-equivalent.
        constexpr uint32_t ENCODER_TICKS_PER_WHEEL_REV = 35600UL;
        constexpr float WHEEL_DIAMETER_MM = 65.0F;
        constexpr float TRACK_WIDTH_MM = 230.0F;

        // Temporary diagnostic mode. Encoder counts remain available for
        // turn/reverse distance and logging, but measured TPS does not adjust
        // PWM or trigger automatic stall/imbalance recovery.
        constexpr bool TPS_FEEDBACK_ENABLED = false;

        // Run at the gearbox's nominal full speed whenever the forward path
        // is confirmed clear.  Approach and avoidance retain lower targets.
        constexpr float CRUISE_RPM = 12.0F;
        constexpr float APPROACH_RPM = 5.0F;
        // At the real 12-14 V supply, 2.2 RPM does not leave enough torque to
        // overcome the loaded drivetrain's static friction during avoidance.
        constexpr float AVOID_CRAWL_RPM = 3.5F;
        // Lower in-place turn speed improves encoder stopping accuracy and
        // prevents the chassis from coasting far past the selected heading.
        constexpr float TURN_RPM = 5.0F;
        constexpr float ESCAPE_RPM = 9.0F;
        constexpr float MAX_EXPECTED_RPM = 12.0F;

        constexpr int32_t rpmToTicksPerSecond(const float rpm)
        {
            return static_cast<int32_t>(
                (rpm * static_cast<float>(ENCODER_TICKS_PER_WHEEL_REV)) / 60.0F);
        }
        constexpr int32_t CRUISE_TPS = rpmToTicksPerSecond(CRUISE_RPM);
        constexpr int32_t APPROACH_TPS = rpmToTicksPerSecond(APPROACH_RPM);
        constexpr int32_t AVOID_CRAWL_TPS =
            rpmToTicksPerSecond(AVOID_CRAWL_RPM);
        constexpr int32_t TURN_TPS = rpmToTicksPerSecond(TURN_RPM);
        constexpr int32_t ESCAPE_TPS = rpmToTicksPerSecond(ESCAPE_RPM);
        constexpr int32_t MAX_EXPECTED_TPS = rpmToTicksPerSecond(MAX_EXPECTED_RPM);

        // PI speed controller tuned for smooth dynamic 12V PWM modulation.
        constexpr float SPEED_KP = 0.35F;
        constexpr float SPEED_KI = 0.15F;
        constexpr float INTEGRAL_LIMIT = 150.0F;
        // Loaded motors need enough initial voltage to get through gearbox and
        // tyre static friction.  Lower values make low-speed avoidance chatter.
        constexpr uint8_t MIN_EFFECTIVE_PWM = 55;
        // The installed supply is 12-14 V, not the 21 V test supply.  The old
        // 21 V compensation capped PWM at 145 and made the real robot run at
        // only 57% duty.  Allow full duty so an unobstructed cruise command can
        // use all of the available supply; obstacle modes still reduce speed.
        constexpr float MOTOR_SUPPLY_V = 14.0F;
        constexpr float MOTOR_RATED_V = 12.0F;
        constexpr uint8_t MAX_DRIVE_PWM = 255;
        constexpr uint8_t PWM_SLEW_PER_UPDATE = 12;    // Prompt but controlled response to an avoidance command
        constexpr int32_t DEFAULT_RAMP_STEP_TPS = 120; // Soft-start speed ramp step per update
        constexpr int32_t DECEL_RAMP_STEP_TPS = MAX_EXPECTED_TPS;
        constexpr uint16_t ZERO_CROSS_DWELL_MS = 60; // Zero-crossing deadtime before motor reversal
        constexpr uint16_t CONTROL_UPDATE_MS = 20;
        constexpr uint16_t STALL_TIMEOUT_MS = 2500;
        constexpr uint8_t STALL_MIN_PWM = 70;
        // Detect one wheel caught on a loose object while the other spins. This is
        // separate from total stall detection, which sees any encoder movement as
        // motion and would otherwise miss this failure.
        constexpr uint16_t WHEEL_JAM_TIMEOUT_MS = 2500;
        constexpr int32_t WHEEL_JAM_MIN_TARGET_TPS =
            rpmToTicksPerSecond(1.0F);
        constexpr uint8_t WHEEL_JAM_LOW_RESPONSE_PERCENT = 12;
        constexpr uint8_t WHEEL_JAM_MOVING_RESPONSE_PERCENT = 35;

        constexpr uint16_t BRAKE_SETTLE_MS = 1200;
        constexpr float ESCAPE_REVERSE_MM = 100.0F;
        constexpr float ESCAPE_REVERSE_STEP_MM = 50.0F;
        constexpr float ESCAPE_REVERSE_MAX_MM = 200.0F;
        constexpr uint16_t ESCAPE_REPEAT_WINDOW_MS = 20000;
        constexpr uint16_t ESCAPE_STABLE_RESET_MS = 25000;
        constexpr uint8_t ESCAPE_MAX_ATTEMPTS = 3;
        constexpr float ESCAPE_TURN_DEG = 80.0F;
        constexpr float ESCAPE_MIN_TURN_DEG = 45.0F;
        constexpr float ESCAPE_TURN_MARGIN_DEG = 15.0F;
        constexpr float HARD_AVOID_TURN_DEG = 35.0F;
        constexpr float TURN_MIN_COMMIT_DEG = 30.0F;
        constexpr float TURN_BRAKE_LEAD_DEG = 6.0F;
        constexpr float TURN_SOFT_START_SECONDS = 0.35F;
        constexpr uint16_t TURN_TIMEOUT_MS = 6500;
        constexpr uint16_t ESCAPE_REVERSE_TIMEOUT_MS = 5000;
        constexpr uint16_t ESCAPE_SCAN_TIMEOUT_MS = 6500;
        constexpr uint16_t ESCAPE_TURN_TIMEOUT_MS = 6500;
    } // namespace motor

    namespace sonar
    {
        constexpr uint16_t MIN_VALID_CM = 5;
        constexpr uint16_t MAX_VALID_CM = 300;
        constexpr uint32_t ECHO_TIMEOUT_US = 19000UL;
        // HC-SR04 needs time for late echoes to decay before the next trigger.
        constexpr uint16_t DRIVE_SAMPLE_MS = 40;
        // A no-echo result normally means open space. Fast pan accepts it
        // immediately; an anomaly focus uses its own stricter miss count.
        constexpr uint8_t NORMAL_INVALID_LIMIT = 1;
        constexpr uint8_t FOCUS_INVALID_LIMIT = 3;
        constexpr uint8_t RADAR_MIN_DEG = 15;
        constexpr uint8_t RADAR_MAX_DEG = 165;
        constexpr uint8_t RADAR_STEP_DEG = 5;
        constexpr uint8_t RADAR_POINT_COUNT =
            ((RADAR_MAX_DEG - RADAR_MIN_DEG) / RADAR_STEP_DEG) + 1;
        constexpr uint8_t RADAR_CENTER_INDEX =
            (90 - RADAR_MIN_DEG) / RADAR_STEP_DEG;
        constexpr uint8_t radarAngleAt(const uint8_t index)
        {
            return static_cast<uint8_t>(
                RADAR_MIN_DEG + index * RADAR_STEP_DEG);
        }
        constexpr uint8_t RADAR_SAMPLES_PER_ANGLE = 1;
        constexpr uint8_t STARTUP_SAMPLES_PER_ANGLE = 3;
        constexpr uint8_t ESCAPE_SAMPLES_PER_ANGLE = 2;
        constexpr uint8_t THREAT_SIDE_SAMPLES_PER_ANGLE = 2;
        constexpr uint8_t ANOMALY_FOCUS_SAMPLES = 5;
        constexpr uint8_t RADAR_SAMPLE_BUFFER_SIZE =
            ANOMALY_FOCUS_SAMPLES;
        // Pause only for a newly appearing near object. Ordinary distant
        // geometry changes should not interrupt the fast sweep.
        constexpr uint16_t ANOMALY_FOCUS_MAX_CM = 110;
        constexpr uint16_t ANOMALY_DROP_CM = 25;
        // During a committed rolling avoid, keep both physical sides fresh.
        // Scanning only the initially selected side made one early bad echo
        // self-reinforcing: the opposite (possibly open) side went stale and
        // could never win the planner again.
        constexpr uint8_t THREAT_SEQUENCE_COUNT = 7;
        // The command angle can arrive before the horn and sensor bracket stop
        // vibrating. Do not ping until the mechanism has physically settled.
        constexpr uint16_t RADAR_SERVO_SETTLE_MS = 30;
        constexpr uint8_t SERVO_TRAVEL_MS_PER_DEG = 1;
        constexpr uint8_t CLEAR_CONFIRM_VISITS = 2;
        // Only the three rays nearest the chassis centreline (85/90/95 deg)
        // define whether the forward driving corridor is blocked. Wider rays
        // frequently see the floor, sensor bracket, or a harmless side wall.
        constexpr uint8_t FORWARD_MIN_DEG = 85;
        constexpr uint8_t FORWARD_MAX_DEG = 95;
        constexpr uint16_t THREAT_SINGLE_RAY_CM = 55;
        constexpr uint16_t THREAT_CONFIRM_WINDOW_MS = 700;
        constexpr uint16_t THREAT_CONFIRMED_FRESH_MS = 1800;
        constexpr uint16_t SIDE_RAY_FRESH_MS = 1500;
        constexpr uint16_t SPATIAL_FRESH_MS = 12000;
        constexpr uint16_t TURNING_SPATIAL_FRESH_MS = 3500;
        constexpr int8_t TURNING_STEER_THRESHOLD_PERCENT = 15;
        constexpr uint16_t FRONT_FRESH_MS = 8000;
        constexpr uint8_t RADAR_MIN_GAP_POINTS = 3;
        constexpr uint8_t RADAR_CORRIDOR_POINTS = 5;
        // Escape turns need a wider run of clear rays than ordinary rolling
        // guidance. This rejects a deep return between furniture legs that is wide
        // enough for the sonar beam but not for the robot chassis.
        constexpr uint8_t ESCAPE_CORRIDOR_POINTS = 7;

        constexpr uint16_t EMERGENCY_CM = 18;
        // Begin actively creating side-wall clearance before the chassis gets close.
        constexpr uint16_t STEER_CM = 90;
        constexpr int8_t MAX_STEER_PERCENT = 75;
        constexpr uint16_t HARD_AVOID_CM = 45;
        constexpr uint16_t SIDE_AVOID_CM = 75;
        // With the forward corridor open, use a gentle correction rather than
        // ignoring a real side obstacle or entering aggressive avoidance.
        constexpr uint16_t SIDE_GUIDANCE_CM = 70;
        constexpr int8_t SIDE_GUIDANCE_MAX_STEER_PERCENT = 25;
        constexpr uint16_t SIDE_AVOID_HOLD_MS = 4500;
        constexpr uint16_t SIDE_STUCK_CM = 20;
        constexpr uint16_t SIDE_STUCK_RELEASE_CM = 35;
        constexpr uint16_t SIDE_AVOID_RELEASE_CM = 85;
        constexpr uint16_t SIDE_DIRECTION_COMMIT_MS = 900;
        constexpr int8_t SIDE_RESOLUTION_MAX_STEER_PERCENT = 60;
        constexpr uint16_t SIDE_EMERGENCY_CM = 10;
        constexpr uint16_t SIDE_EMERGENCY_CONFIRM_MS = 500;
        constexpr uint16_t SIDE_STUCK_GRACE_MS = 7000;
        constexpr uint16_t SIDE_PROGRESS_TIMEOUT_MS = 4000;
        constexpr uint8_t SIDE_PROGRESS_CM = 3;
        // Hysteresis must release above the distance that activates avoidance.
        // The previous 120/50 pair made readings from 50..120 cm repeatedly
        // enter and leave avoidance, producing continuous left/right hunting.
        constexpr uint16_t FRONT_AVOID_CM = 60;
        constexpr uint16_t FRONT_AVOID_RELEASE_CM = 85;

        constexpr uint16_t FRONT_AVOID_RELEASE_CONFIRM_MS = 600;
        constexpr uint16_t POST_AVOID_STRAIGHT_MS = 900;
        constexpr uint16_t POST_AVOID_SIDE_OVERRIDE_CM = 25;
        constexpr uint16_t AVOID_FAILSAFE_CM = 28;
        constexpr uint16_t AVOID_FAILSAFE_MS = 8000;
        constexpr uint16_t FRONT_PROGRESS_TIMEOUT_MS = 3500;
        constexpr uint8_t FRONT_PROGRESS_CM = 4;
        // Avoidance normally commits to one direction to prevent hunting, but
        // one early decision may be corrected after both sides have been
        // freshly observed and the opposite side is clearly safer.
        constexpr uint16_t AVOID_DIRECTION_RECHECK_MS = 700;
        constexpr uint16_t AVOID_DIRECTION_SWITCH_MARGIN_CM = 35;
        // While a strong turn is commanded, compare a five-angle radar fingerprint
        // over time. If the wheels report travel but the surrounding scene stays
        // nearly identical, the chassis is caught and the wheels are spinning free.
        constexpr uint16_t RADAR_STUCK_TIMEOUT_MS = 5000;
        constexpr float RADAR_STUCK_MIN_WHEEL_TRAVEL_MM = 80.0F;
        constexpr int32_t RADAR_STUCK_MIN_COMMAND_TPS =
            motor::rpmToTicksPerSecond(2.0F);
        constexpr int32_t RADAR_STUCK_MIN_REPORTED_TPS =
            motor::rpmToTicksPerSecond(1.0F);
        constexpr int8_t RADAR_STUCK_TURN_THRESHOLD_PERCENT = 15;
        constexpr uint8_t RADAR_STUCK_ANCHOR_COUNT = 5;
        constexpr uint8_t RADAR_STUCK_MIN_COMPARABLE_ANCHORS = 3;
        constexpr uint8_t RADAR_STUCK_STABLE_DELTA_CM = 6;
        constexpr uint16_t RADAR_STUCK_MAX_ANCHOR_CM = 240;
        constexpr uint16_t RADAR_STUCK_ANCHOR_FRESH_MS = 6000;
        constexpr uint8_t radarStuckAnchorAngle(
            const uint8_t index)
        {
            return static_cast<uint8_t>(30U + index * 30U);
        }
        constexpr uint16_t ESCAPE_EARLY_SCAN_MIN_MS = 1000;
        constexpr uint16_t ESCAPE_RECOVERY_GRACE_MS = 5000;
        constexpr uint8_t FAILED_ANGLE_SECTOR_COUNT = 2;
        constexpr uint8_t FAILED_ANGLE_INITIAL_RADIUS_DEG = 20;
        constexpr uint8_t FAILED_ANGLE_EXPAND_DEG = 10;
        constexpr uint8_t FAILED_ANGLE_MAX_RADIUS_DEG = 45;
        constexpr uint16_t FAILED_ANGLE_MEMORY_MS = 15000;
        // Failed headings are discouraged, not deleted. Keeping at least the open
        // threshold preserves a physically wide corridor even when its centre
        // overlaps a recently failed heading.
        constexpr uint8_t FAILED_ANGLE_CENTER_RETAIN_PERCENT = 55;
        constexpr uint8_t FAILED_ANGLE_EDGE_RETAIN_PERCENT = 85;
        constexpr uint16_t CORRIDOR_FRONT_CM = 70;
        constexpr uint8_t CORRIDOR_BALANCE_CM = 30;
        constexpr uint8_t CENTERING_DEADBAND_CM = 8;
        // Correct one corridor-side error until it has shrunk by 60%, then
        // coast straight briefly before reconsidering the opposite side.
        constexpr uint8_t CORRIDOR_PROGRESS_PERCENT = 60;
        constexpr uint16_t CORRIDOR_MIN_COMMIT_MS = 350;
        constexpr uint16_t CORRIDOR_MAX_COMMIT_MS = 1800;
        constexpr uint16_t CORRIDOR_SETTLE_MS = 450;
        constexpr uint16_t CORRIDOR_PAIR_MAX_SKEW_MS = 1000;
        constexpr int8_t CORRIDOR_MIN_STEER_PERCENT = 12;
        constexpr int8_t CORRIDOR_MAX_STEER_PERCENT = 30;
        constexpr int8_t PATH_MAX_STEER_PERCENT = 45;
        constexpr int8_t OPEN_PATH_MAX_STEER_PERCENT = 25;
        constexpr uint8_t OPEN_PATH_GUIDANCE_DEADBAND_DEG = 10;
        constexpr int8_t AVOID_MIN_STEER_PERCENT = 35;
        constexpr int8_t CRITICAL_MAX_STEER_PERCENT = 95;
        constexpr int8_t AVOID_INNER_BRAKE_BOOST_PERCENT = 30;
        // Keep both tyres rolling during avoidance. A stopped inner wheel
        // makes the outer tyre scrub sideways and can peel it off the rim.
        constexpr int8_t ROLLING_MAX_STEER_PERCENT = 75;
        constexpr uint8_t ROLLING_MIN_INNER_WHEEL_PERCENT = 25;
        constexpr int8_t STEER_DEADBAND_PERCENT = 5;
        constexpr int8_t STEER_SLEW_STEP_PERCENT = 6;
        constexpr uint16_t STEERING_UPDATE_MS = 40;
        constexpr uint8_t PATH_CONFIRMATION_WINS = 2;
        constexpr uint8_t PATH_TARGET_STEP_DEG = 10;
        constexpr uint8_t AVOID_PATH_CONFIRMATION_WINS = 1;
        constexpr uint8_t AVOID_PATH_TARGET_STEP_DEG = 20;
        constexpr int8_t AVOID_STEER_SLEW_STEP_PERCENT = 10;
        constexpr uint16_t CLEAR_CM = 50;
        constexpr uint16_t STOP_CM = 40;
        constexpr uint16_t SLOW_CM = 55;
        constexpr uint16_t OPEN_SPACE_CM = 35;

        constexpr uint32_t BRAKE_SETTLE_MS = 50;

        constexpr uint8_t SCAN_MIN_DEG = 30;
        constexpr uint8_t SCAN_MAX_DEG = 150;
        constexpr uint8_t SCAN_STEP_DEG = 40;
        constexpr uint8_t SCAN_POINT_COUNT =
            ((SCAN_MAX_DEG - SCAN_MIN_DEG) / SCAN_STEP_DEG) + 1;
        constexpr uint16_t SERVO_SETTLE_MS = 160;
        constexpr uint8_t SAMPLES_PER_ANGLE = 8;
        constexpr uint8_t MIN_SAMPLES_PER_ANGLE = 5;
        constexpr uint8_t STABILITY_VARIANCE_CM = 3;
        constexpr uint8_t MIN_GAP_POINTS = 1;
        constexpr uint16_t MAX_SCAN_DURATION_MS = 4000;
    } // namespace sonar

    namespace servo
    {
        constexpr uint8_t CENTER_DEG = 90;
        constexpr uint16_t MIN_PULSE_US = 600;
        constexpr uint16_t MAX_PULSE_US = 2400;
        // This mounting points left as the commanded servo angle increases.
        constexpr int8_t ANGLE_TO_TURN_SIGN = -1;
    } // namespace servo

    namespace pms
    {
        constexpr uint32_t BAUD = 9600;
        constexpr uint16_t WARMUP_MS = 5000;
        constexpr uint16_t STALE_MS = 10000;
        constexpr uint8_t MOVING_AVERAGE_SAMPLES = 12;
    } // namespace pms

    namespace fan
    {
        // Dedicated ESP32-S3 LEDC configuration. Channel 7 is reserved for
        // the fan; the motor PWM outputs and ESP32Servo use other channels.
        constexpr uint32_t PWM_FREQUENCY_HZ = 500UL;
        constexpr uint8_t PWM_RESOLUTION_BITS = 10;
        constexpr uint8_t PWM_CHANNEL = 7;
        constexpr uint16_t PWM_MAX_DUTY =
            (1U << PWM_RESOLUTION_BITS) - 1U;
        // Gentle linear ramp: 1% every 50 ms (about 20% per second). This
        // keeps dust-triggered level changes from feeling like a hard step.
        constexpr uint16_t PWM_UPDATE_MS = 50;
        // Airflow commands are human-readable percentages. They are converted
        // to the LEDC 10-bit duty scale only at the GPIO boundary.
        constexpr uint8_t LOW_PERCENT = 45;
        constexpr uint8_t MEDIUM_PERCENT = 75;
        constexpr uint8_t HIGH_PERCENT = 100;
        constexpr uint8_t FAILSAFE_PERCENT = HIGH_PERCENT;
        constexpr uint8_t SLEW_PERCENT_PER_UPDATE = 1;
        // At power-on, command 100% immediately and hold it long enough for
        // the fan's own built-in soft-start to complete. After this launch
        // burst, PMS warm-up uses LOW and valid readings select the normal
        // air-quality level.
        constexpr uint16_t STARTUP_FULL_THROTTLE_HOLD_MS = 5000;

        // Independent hysteresis thresholds in micrograms per cubic metre.
        // PM10 is normally numerically higher than PM2.5, so applying the
        // PM2.5 limits to it would leave the fan unnecessarily at HIGH.
        // Sensitive purifier profile: react early, but retain generous exit
        // hysteresis so ordinary one-frame variation cannot chatter the fan.
        constexpr uint16_t PM25_LOW_TO_MEDIUM = 20;
        constexpr uint16_t PM25_MEDIUM_TO_LOW = 12;
        constexpr uint16_t PM25_MEDIUM_TO_HIGH = 50;
        constexpr uint16_t PM25_HIGH_TO_MEDIUM = 35;
        constexpr uint16_t PM10_LOW_TO_MEDIUM = 40;
        constexpr uint16_t PM10_MEDIUM_TO_LOW = 25;
        constexpr uint16_t PM10_MEDIUM_TO_HIGH = 100;
        constexpr uint16_t PM10_HIGH_TO_MEDIUM = 70;
    } // namespace fan

    namespace battery
    {
        // Standard 0-25 V Arduino voltage-sensor module: 30 kΩ high side and
        // 7.5 kΩ low side (one-fifth output). With the ESP32-S3's 3.3 V ADC,
        // keep its input below 16.5 V; a 12 V SLA at 14.4 V is safe.
        // Voltage display and automatic low-battery protection are enabled for
        // the robot's nominal 12 V lead-acid battery.
        constexpr bool MONITOR_ENABLED = true;
        constexpr bool PROTECTION_ENABLED = true;
        constexpr float DIVIDER_R_TOP_OHM = 30000.0F;
        constexpr float DIVIDER_R_BOTTOM_OHM = 7500.0F;
        // Cross-calibrated against the INA219 VIN- bus, which is wired directly
        // to battery positive. With the adapter removed and the robot loaded,
        // the divider reported 13.91 V while INA219 measured about 12.78 V.
        // INA219 remains authoritative; this divider is the fallback sensor.
        constexpr float CALIBRATION_MULTIPLIER = 0.816F;
        constexpr float VALID_MIN_V = 9.0F;
        constexpr float VALID_MAX_V = 16.0F;
        constexpr uint16_t SAMPLE_INTERVAL_MS = 100;
        constexpr uint8_t FILTER_SAMPLES = 16;
        constexpr float NOMINAL_CAPACITY_MAH = 12000.0F;
        constexpr float CHARGE_COULOMB_EFFICIENCY = 0.85F;
        constexpr float SOC_MAX_DROP_PERCENT_PER_MIN = 1.0F;
        constexpr float SOC_MAX_RECOVERY_PERCENT_PER_MIN = 0.2F;
        constexpr float SOC_PERSIST_STEP_PERCENT = 5.0F;
        constexpr uint32_t SOC_PERSIST_MIN_INTERVAL_MS = 600000UL;

        // Require the cutoff voltage to remain low for three seconds so motor
        // start-up sag cannot stop the robot on a single ADC sample.
        constexpr float WARNING_V = 11.5F;
        constexpr float CUTOFF_V = 10.8F;
        constexpr uint16_t LOW_CONFIRM_MS = 3000;
    } // namespace battery

    namespace charge
    {
        // INA219 is on the dedicated 3.3 V Wire1 bus at GPIO8/GPIO9.
        // Default address is 0x40; alter this if the A0/A1 jumpers are bridged.
        constexpr uint8_t I2C_ADDRESS = 0x40;
        constexpr uint16_t SAMPLE_INTERVAL_MS = 250;
        // SLA 12 V / 12 Ah charge-state thresholds. The battery-side voltage
        // settles near 14.1 V because the charger feeds it through a series
        // protection diode, so FULL must be confirmed from sustained voltage
        // and low tail current rather than waiting for an unreachable 14.3 V.
        constexpr float RECHARGE_V = 12.5F;
        constexpr float RECENT_FULL_MIN_V = 12.8F;
        constexpr float ABSORPTION_V = 14.0F;
        constexpr float FULL_CURRENT_MA = 200.0F;
        // Reject a reversed/disconnected shunt reading when confirming FULL.
        constexpr float FULL_MIN_CURRENT_MA = -10.0F;
        constexpr float CHARGER_DETECT_CURRENT_MA = 40.0F;
        constexpr uint16_t DETECT_CONFIRM_MS = 2000;
        constexpr uint16_t QUICK_VERIFY_MS = 20000;
        // Require 14.0 V or higher and <= 0.20 A continuously for 30 minutes.
        // Any voltage drop or current rise resets the confirmation timer.
        constexpr uint32_t FULL_CONFIRM_MS = 1800000UL;
        // Integration gaps larger than this usually mean the MCU was blocked or
        // rebooted; do not turn one stale sample into a large charge estimate.
        constexpr uint16_t MAX_INTEGRATION_STEP_MS = 2000;
        constexpr bool CURRENT_SIGN_INVERTED = false;
        constexpr bool PRESENT_ACTIVE_HIGH = true;
    } // namespace charge

    namespace display
    {
        constexpr uint8_t COLUMNS = 20;
        constexpr uint8_t ROWS = 4;
        constexpr uint16_t STARTUP_REFRESH_MS = 250;
        constexpr uint16_t NORMAL_REFRESH_MS = 1000;
        constexpr uint16_t LOW_BATTERY_BLINK_MS = 500;
    } // namespace display

    namespace safety
    {
        // Keep false during wiring and encoder calibration. The robot will remain in
        // STARTUP while sensors and encoder counts continue to be reported.
        constexpr bool MOTION_ENABLED = true;
        constexpr uint8_t MAX_AVOID_EVENTS = 5;
        constexpr uint16_t AVOID_WINDOW_MS = 12000;
        constexpr uint16_t STARTUP_GRACE_MS = 500;
    } // namespace safety

    namespace logging
    {
        constexpr uint32_t SERIAL_BAUD = 115200;
    } // namespace logging

    static_assert(sonar::SCAN_POINT_COUNT <= 20, "Scan buffer is too large");
    static_assert(motor::PWM_LEFT_CHANNEL != motor::PWM_RIGHT_CHANNEL &&
                      motor::PWM_LEFT_CHANNEL != fan::PWM_CHANNEL &&
                      motor::PWM_RIGHT_CHANNEL != fan::PWM_CHANNEL,
                  "LEDC channels must be unique");
    static_assert((motor::PWM_LEFT_CHANNEL / 2U) ==
                      (motor::PWM_RIGHT_CHANNEL / 2U),
                  "Motor PWM channels must share one LEDC timer");
    static_assert((motor::PWM_LEFT_CHANNEL / 2U) !=
                      (fan::PWM_CHANNEL / 2U),
                  "Motor and fan PWM must use different LEDC timers");
    static_assert(sonar::RADAR_POINT_COUNT <= 32,
                  "Radar coverage mask is too small");
    static_assert(sonar::THREAT_SIDE_SAMPLES_PER_ANGLE <=
                          sonar::RADAR_SAMPLE_BUFFER_SIZE &&
                      sonar::STARTUP_SAMPLES_PER_ANGLE <=
                          sonar::RADAR_SAMPLE_BUFFER_SIZE &&
                      sonar::ANOMALY_FOCUS_SAMPLES <=
                          sonar::RADAR_SAMPLE_BUFFER_SIZE,
                  "Threat sample buffer is too small");
    static_assert(sonar::SCAN_MAX_DEG > sonar::SCAN_MIN_DEG,
                  "Invalid scanner angle range");
    static_assert(motor::ENCODER_TICKS_PER_WHEEL_REV > 0,
                  "Encoder resolution must be configured");
    static_assert(motor::MOTOR_SUPPLY_V >= motor::MOTOR_RATED_V,
                  "Motor supply voltage must be at least the rated voltage");
    static_assert(fan::LOW_PERCENT <= fan::MEDIUM_PERCENT &&
                      fan::MEDIUM_PERCENT <= fan::HIGH_PERCENT &&
                      fan::HIGH_PERCENT <= 100U,
                  "Fan percentages must be ordered and within 0-100");

} // namespace config
