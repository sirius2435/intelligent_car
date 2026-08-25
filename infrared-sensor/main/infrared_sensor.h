#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Mask bits are printed/read from left to right as channel 4, 3, 2, 1. */
#define IR_CHANNEL_4_MASK  (1U << 3)
#define IR_CHANNEL_3_MASK  (1U << 2)
#define IR_CHANNEL_2_MASK  (1U << 1)
#define IR_CHANNEL_1_MASK  (1U << 0)

typedef struct {
    uint8_t black_mask;
    bool changed;
} infrared_sensor_state_t;

/* Configure all four inputs. Returns ESP_ERR_INVALID_ARG for placeholder pins. */
esp_err_t infrared_sensor_init(void);

/* Read and debounce the four digital outputs. Call every IR_SAMPLE_PERIOD_MS. */
esp_err_t infrared_sensor_sample(infrared_sensor_state_t *state);

/* Format the mask as a four-character channel 4..1 black/white string. */
void infrared_sensor_format(uint8_t black_mask, char output[5]);

#ifdef __cplusplus
}
#endif
