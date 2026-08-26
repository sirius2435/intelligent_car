#pragma once

/*
 * D24A four-channel DC motor driver board with two TB6612FNG chips.
 * This car uses Motor D, Motor A, and Motor B; Motor C is unused.
 */

/* Left wheel: D24A Motor D (DIN1, DIN2, PWMD). */
#define MOTOR_LEFT_IN1_GPIO     41
#define MOTOR_LEFT_IN2_GPIO     42
#define MOTOR_LEFT_PWM_GPIO      2

/* Right wheel: D24A Motor A (AIN1, AIN2, PWMA). */
#define MOTOR_RIGHT_IN1_GPIO     7
#define MOTOR_RIGHT_IN2_GPIO     6
#define MOTOR_RIGHT_PWM_GPIO     5

/* Rear wheel: D24A Motor B (BIN1, BIN2, PWMB). */
#define MOTOR_REAR_IN1_GPIO      8
#define MOTOR_REAR_IN2_GPIO     18
#define MOTOR_REAR_PWM_GPIO     17

/*
 * Hall encoder inputs. Replace all six -1 values with bare ESP32-S3 GPIO
 * numbers. Leaving all six at -1 keeps encoder support safely disabled.
 * Filling only some of the pins is treated as an invalid configuration.
 */
#define ENCODER_LEFT_A_GPIO     (40)  /* Motor D: E4A */
#define ENCODER_LEFT_B_GPIO     (39)  /* Motor D: E4B */
#define ENCODER_RIGHT_A_GPIO    (15)  /* Motor A: E1A */
#define ENCODER_RIGHT_B_GPIO    (16)  /* Motor A: E1B */
#define ENCODER_REAR_A_GPIO     (3)  /* Motor B: E2A */
#define ENCODER_REAR_B_GPIO     (46)  /* Motor B: E2B */

/* Set to 1 if a forward-running wheel produces a negative encoder count. */
#define ENCODER_LEFT_REVERSED   0
#define ENCODER_RIGHT_REVERSED  0
#define ENCODER_REAR_REVERSED   0

/* Reject encoder input pulses shorter than this duration. */
#define ENCODER_GLITCH_FILTER_NS 1000

/*
 * The D24A board's ON/OFF switch handles standby/enable for both TB6612FNG
 * chips. Keep this at -1 and move the switch to ON before running a test.
 */
#define MOTOR_STBY_GPIO        (4)

/* Set to 1 if a wheel runs backward when commanded forward. */
#define MOTOR_LEFT_REVERSED    0
#define MOTOR_RIGHT_REVERSED   1
#define MOTOR_REAR_REVERSED    0

/* Conservative bench-test settings. Speed is in permille (0..1000). */
#define MOTOR_TEST_SPEED       200
#define NEG_MOTOR_TEST_SPEED   -200
#define NULL_MOTOR_TEST_SPEED  0

#define MOTOR_TEST_RUN_MS      2000
#define MOTOR_TEST_PAUSE_MS    1000
#define MOTOR_TEST_START_MS    3000

/* Keep reverse disabled for the first powered test. */
#define MOTOR_RUN_REVERSE_TEST 0

/* Test all three simultaneously only after each wheel passes by itself. */
#define MOTOR_RUN_ALL_TEST     0
