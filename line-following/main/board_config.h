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
#define IR_CHANNEL_4_GPIO  (-1)
#define IR_CHANNEL_3_GPIO  (-1)
#define IR_CHANNEL_2_GPIO  (-1)
#define IR_CHANNEL_1_GPIO  (-1)

/* LQ_R4CHVB outputs low while its sensor is over a black line. */
#define IR_BLACK_LEVEL              0
#define IR_SAMPLE_PERIOD_MS        10
#define IR_DEBOUNCE_SAMPLE_COUNT    2

/* D24A/TB6612FNG motor outputs verified by the wheel-test project. */
#define MOTOR_LEFT_IN1_GPIO        42
#define MOTOR_LEFT_IN2_GPIO         2
#define MOTOR_LEFT_PWM_GPIO         1

#define MOTOR_RIGHT_IN1_GPIO       11
#define MOTOR_RIGHT_IN2_GPIO       10
#define MOTOR_RIGHT_PWM_GPIO        9

#define MOTOR_REAR_IN1_GPIO        17
#define MOTOR_REAR_IN2_GPIO        16
#define MOTOR_REAR_PWM_GPIO        15

/* The physical switch on the D24A board controls standby. */
#define MOTOR_STBY_GPIO           (-1)

/* Change a value to 1 if that individual motor runs in the wrong direction. */
#define MOTOR_LEFT_REVERSED         0
#define MOTOR_RIGHT_REVERSED        0
#define MOTOR_REAR_REVERSED         0

/* Open-loop line-following parameters; all motor commands use -1000..1000. */
#define LINE_BASE_FORWARD          220
#define LINE_MIN_FORWARD           140
#define LINE_ERROR_SLOWDOWN         25
#define LINE_KP                     70
#define LINE_KD                     30
#define LINE_TURN_LIMIT            250

#define LINE_SEARCH_FORWARD         80
#define LINE_SEARCH_TURN           180
#define LINE_LOST_STOP_MS         1500
#define LINE_ALL_BLACK_STOP_MS     100
#define LINE_INVALID_GRACE_MS      100
#define LINE_START_DELAY_MS       3000
#define LINE_LOG_PERIOD_MS         100
