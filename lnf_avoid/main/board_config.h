#pragma once

/*
 * Four-channel line sensor, viewed from the front of the car:
 *
 *       car left                         car right
 *       channel 4  channel 3  channel 2  channel 1
 *
 * Replace all four -1 values with the actual GPIO numbers before building
 * firmware for the car.
 */
#define IR_CHANNEL_4_GPIO  (9)
#define IR_CHANNEL_3_GPIO  (10)
#define IR_CHANNEL_2_GPIO  (11)
#define IR_CHANNEL_1_GPIO  (12)

/* LQ_R4CHVB outputs low while its sensor is over a black line. */
#define IR_BLACK_LEVEL              0
#define IR_SAMPLE_PERIOD_MS        10
#define IR_DEBOUNCE_SAMPLE_COUNT    2

/* D24A/TB6612FNG motor outputs verified by the wheel-test project. */
#define MOTOR_LEFT_IN1_GPIO        41
#define MOTOR_LEFT_IN2_GPIO        42
#define MOTOR_LEFT_PWM_GPIO         2

#define MOTOR_RIGHT_IN1_GPIO       7
#define MOTOR_RIGHT_IN2_GPIO       6
#define MOTOR_RIGHT_PWM_GPIO       5

#define MOTOR_REAR_IN1_GPIO        8
#define MOTOR_REAR_IN2_GPIO        18
#define MOTOR_REAR_PWM_GPIO        17

/* D24A/TB6612 standby/enable input. High enables the motor bridges. */
#define MOTOR_STBY_GPIO           (4)

/* Quadrature Hall encoder inputs copied from the verified wheel-test wiring. */
#define ENCODER_LEFT_A_GPIO       (40)  /* Motor D: E4A */
#define ENCODER_LEFT_B_GPIO       (39)  /* Motor D: E4B */
#define ENCODER_RIGHT_A_GPIO      (15)  /* Motor A: E1A */
#define ENCODER_RIGHT_B_GPIO      (16)  /* Motor A: E1B */
#define ENCODER_REAR_A_GPIO        (3)  /* Motor B: E2A */
#define ENCODER_REAR_B_GPIO       (46)  /* Motor B: E2B */

/* Positive counts correspond to the wheel-test definition of forward. */
#define ENCODER_LEFT_REVERSED       0
#define ENCODER_RIGHT_REVERSED      1
#define ENCODER_REAR_REVERSED       0
#define ENCODER_GLITCH_FILTER_NS 1000

/* Change a value to 1 if that individual motor runs in the wrong direction. */
#define MOTOR_LEFT_REVERSED         1
#define MOTOR_RIGHT_REVERSED        1
#define MOTOR_REAR_REVERSED         0

/*
 * Two-wheel differential drive: only the two front wheels are driven.
 * Equal speeds = forward; positive turn = right turn (left wheel faster
 * than right). The rear wheel is passive and never driven.
 */

/* Open-loop line-following parameters; all motor commands use -1000..1000. */
#define LINE_BASE_FORWARD          160
#define LINE_MIN_FORWARD           100
#define LINE_ERROR_SLOWDOWN         20
#define LINE_KP                     70
#define LINE_KD                     15                                          
#define LINE_TURN_LIMIT            250

#define LINE_CORNER_ARM_MS              30
#define LINE_CORNER_CONFIRM_WINDOW_MS  120
#define LINE_CORNER_APPROACH_FORWARD   120
#define LINE_CORNER_ROTATE_FORWARD       0
#define LINE_CORNER_ROTATE_TURN         200
#define LINE_CORNER_MIN_ROTATE_MS        60
#define LINE_CORNER_CENTERED_MS          30
#define LINE_CORNER_MAX_ROTATE_MS       600
#define LINE_CORNER_EXIT_MS              50

#define LINE_SEARCH_FORWARD          0
#define LINE_SEARCH_TURN           230
#define LINE_SEARCH_FINE_TURN      180
#define LINE_SEARCH_30_DEG_COUNTS  160
#define LINE_SEARCH_FIRST_COUNTS   (4 * LINE_SEARCH_30_DEG_COUNTS)
#define LINE_SEARCH_SECOND_COUNTS  (8 * LINE_SEARCH_30_DEG_COUNTS)
#define LINE_SEARCH_SETTLE_MS       30
#define LINE_SEARCH_MAX_LEGS         2
#define LINE_SEARCH_STALL_MS       400
#define LINE_SEARCH_STALL_COUNTS     2
#define LINE_LOST_STOP_MS         5000
#define LINE_ALL_BLACK_STOP_MS     100
#define LINE_INVALID_GRACE_MS      100
#define LINE_START_DELAY_MS       3000
#define LINE_LOG_PERIOD_MS         100

/* HC-SR04 ultrasonic ranger. ECHO is a 5 V signal: use a divider/level shifter. */
#define ULTRASONIC_TRIG_GPIO         14
#define ULTRASONIC_ECHO_GPIO         13
#define ULTRASONIC_SAMPLE_PERIOD_MS  60
#define ULTRASONIC_ECHO_TIMEOUT_US   30000

/*
 * One-shot obstacle avoidance. Positive lateral means car-left.
 * The lateral mixer uses [left, right, rear] = [-v/2, v/2, -v]. Encoder
 * targets are deliberately calibration constants: verify them at low speed
 * on the actual floor before increasing any speed.
 */
#define AVOID_TRIGGER_DISTANCE_MM      50
#define AVOID_TRIGGER_CONFIRM_SAMPLES   2
#define AVOID_SLOW_DISTANCE_MM         100
#define AVOID_SLOW_FORWARD              90
#define AVOID_CLEAR_DISTANCE_MM        120
#define AVOID_CLEAR_CONFIRM_SAMPLES      3
#define AVOID_SENSOR_STALE_MS           500

#define AVOID_BRAKE_MS                  100
#define AVOID_LATERAL_SPEED             120
#define AVOID_FORWARD_SPEED             140
#define AVOID_LEFT_MIN_COUNTS           240
#define AVOID_LEFT_CLEARANCE_MS          250
#define AVOID_LEFT_MAX_COUNTS          2600
#define AVOID_FORWARD_TARGET_COUNTS    1000
#define AVOID_RIGHT_EXTRA_COUNTS        600
#define AVOID_LINE_CENTERED_MS           30
#define AVOID_MOTION_TIMEOUT_MS        6000
#define AVOID_STALL_TIMEOUT_MS          500
#define AVOID_STALL_MIN_COUNTS            2

/* Per-wheel encoder PI loop used only while lateral motion is requested.
 *
 * Each strafe direction gets one high feed-forward pulse until the first
 * speed sample, then it may fall below the old 260 PWM floor.  The run log
 * showed that the wheels were still several times faster than their targets
 * at PWM 260, so left strafe runs down to 100 PWM and right strafe to 90.
 */
#define DRIVE_SPEED_CONTROL_PERIOD_MS       50
#define DRIVE_TARGET_CPS_PER_COMMAND_NUM     2
#define DRIVE_TARGET_CPS_PER_COMMAND_DEN     1
#define DRIVE_LATERAL_MIN_ACTIVE_PWM       260
#define DRIVE_LEFT_STRAFE_MIN_ACTIVE_PWM   100
#define DRIVE_RIGHT_STRAFE_MIN_ACTIVE_PWM   90
#define DRIVE_APPROACH_STARTUP_PWM         180
#define DRIVE_APPROACH_MIN_ACTIVE_PWM      100
#define DRIVE_FORWARD_MIN_ACTIVE_PWM       180
#define DRIVE_LATERAL_MAX_PWM              700
#define DRIVE_SPEED_KP_NUM                   1
#define DRIVE_SPEED_KP_DEN                   2
#define DRIVE_SPEED_KI_NUM                   1
#define DRIVE_SPEED_KI_DEN                   4
#define DRIVE_SPEED_INTEGRAL_LIMIT        1200
