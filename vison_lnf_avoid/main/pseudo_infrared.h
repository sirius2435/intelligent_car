#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "infrared_sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Pseudo-infrared sensor: samples four fixed pixel blocks directly from the
 * decoded RGB888 camera frame and converts them into the same four-bit black
 * mask that the LQ_R4CHVB infrared board produces, so the original infrared
 * line_follow / obstacle_avoidance state machines run unchanged.
 *
 * Channel ordering is authoritative from the reference project (lnf_avoid),
 * viewed from the front of the car:
 *
 *       car left                         car right
 *       channel 4  channel 3  channel 2  channel 1
 *
 * bit 0 = channel 1 (car right), bit 3 = channel 4 (car left).
 *
 * Block geometry (block size, sample row, thresholds) lives in board_config.h
 * under the PSEUDO_IR_* macros. Sampling semantics: a new mask is published
 * once per decoded frame; between frames the previous
 * mask is held so the 10 ms control loop sees a stable value. A confirmed
 * finish marker (PSEUDO_IR_FINISH_CONFIRM_FRAMES consecutive frames with all
 * four blocks black) is latched and triggers the controller's all-black stop.
 */
esp_err_t pseudo_infrared_sample_rgb888(const uint8_t *rgb,
                                        unsigned width,
                                        unsigned height,
                                        size_t stride_bytes,
                                        infrared_sensor_state_t *state);

/* Clears the held mask and finish confirmation so a USB reconnect starts from
 * a clean all-white state. */
void pseudo_infrared_reset(void);

/* CH4 CH3 CH2 CH1 order (bit 3 = channel 4 = car left). */
void pseudo_infrared_format(uint8_t black_mask, char output[5]);

#ifdef __cplusplus
}
#endif
