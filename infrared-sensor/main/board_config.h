#pragma once

/*
 * LQ_R4CHVB sensor order when viewed from the car:
 *
 *        left                         right
 *     channel 4  channel 3  channel 2  channel 1
 *
 * Replace each -1 with a bare ESP32-S3 GPIO number after wiring is fixed.
 * The program deliberately refuses to sample while any value remains -1.
 */
#define IR_CHANNEL_4_GPIO  (9)
#define IR_CHANNEL_3_GPIO  (10)
#define IR_CHANNEL_2_GPIO  (11)
#define IR_CHANNEL_1_GPIO  (12)

/* The RGB status LED used by the existing blink-led project. */
#define STATUS_LED_GPIO    38

/* LQ_R4CHVB outputs low over black and high over white. */
#define IR_BLACK_LEVEL     0

/* Three equal readings at 10 ms intervals are required to accept a change. */
#define IR_SAMPLE_PERIOD_MS       10
#define IR_DEBOUNCE_SAMPLE_COUNT   3
