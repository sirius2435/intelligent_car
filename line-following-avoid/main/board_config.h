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

/* HC-SR04-style ultrasonic sensor. Echo must be level-shifted to 3.3 V
 * if the module is powered from 5 V. */
#define ULTRASONIC_ECHO_GPIO        (13)
#define ULTRASONIC_TRIG_GPIO        (14)
#define ULTRASONIC_TIMEOUT_US       25000
#define ULTRASONIC_SAMPLE_PERIOD_MS 50
#define ULTRASONIC_OBSTACLE_CM      5

/* ============================================================
 * Obstacle avoidance parameters
 * ============================================================
 *
 * Normal line following:
 *   front wheels driven
 *   rear wheel = 0
 *
 * Obstacle avoidance:
 *   front wheels = 1/2 rear wheel speed
 *   rear wheel is enabled
 *
 * Sequence:
 *
 *   obstacle < 5 cm
 *        ↓
 *      STOP
 *        ↓
 *   SHIFT LEFT
 *        ↓
 *   FORWARD ~10 cm
 *        ↓
 *   SHIFT RIGHT
 *        ↓
 *   reacquire black line
 *        ↓
 *   LINE FOLLOWING
 * ============================================================ */

/* Rear wheel speed during lateral movement. */
#define OBSTACLE_SHIFT_SPEED        300

/* Front wheels are half the rear-wheel speed. */
#define OBSTACLE_SHIFT_FRONT_RATIO  0.50f

/* Duration of the initial left lateral movement. */
#define OBSTACLE_LEFT_SHIFT_MS      450

/* Forward movement during obstacle avoidance. */
#define OBSTACLE_FORWARD_SPEED      160

/*
 * Approximate forward travel of ~10 cm.
 *
 * This is open-loop timing. If the actual distance is too long/short,
 * this is the first value to tune.
 */
#define OBSTACLE_FORWARD_MS         650

/*
 * Maximum time allowed for the final right shift.
 * The right shift terminates earlier when the infrared sensors
 * reacquire the black line.
 */
#define OBSTACLE_RIGHT_SHIFT_TIMEOUT_MS 5000

/* Small settling pauses between movements. */
#define OBSTACLE_STOP_SETTLE_MS     100
#define OBSTACLE_SHIFT_SETTLE_MS    80

/*
 * The right-shift stage must first see white, then black.
 * This prevents the controller from immediately accepting a line
 * that was already underneath the sensors.
 */
#define OBSTACLE_REQUIRE_WHITE_MS   30

/* Prevent immediate retrigger after completing avoidance. */
#define OBSTACLE_REARM_MS           500

/*
 * Lateral direction calibration.
 *
 * +1 = normal direction
 * -1 = reverse lateral direction
 *
 * Keep +1 initially.
 */
#define OBSTACLE_RIGHT_REAR_SIGN    1

/* For the current 3-wheel omni layout, a lateral command uses
 * front-wheel magnitude = rear-wheel magnitude / 2, with the rear wheel
 * running in the opposite direction to both front wheels. If a first bench
 * test moves sideways in the opposite direction, change this from +1 to -1. */
#define OBSTACLE_RIGHT_REAR_SIGN    1

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