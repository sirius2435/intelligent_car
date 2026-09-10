#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IR_CHANNEL_4_MASK  (1U << 3)
#define IR_CHANNEL_3_MASK  (1U << 2)
#define IR_CHANNEL_2_MASK  (1U << 1)
#define IR_CHANNEL_1_MASK  (1U << 0)
#define IR_ALL_BLACK_MASK  0x0FU

typedef struct {
    uint8_t black_mask;
    /* Set only by camera pseudo-infrared when the wide END marker has been
     * confirmed. This separates normal mission completion from lost-line stop. */
    bool finish_detected;
} infrared_sensor_state_t;

#ifdef __cplusplus
}
#endif
