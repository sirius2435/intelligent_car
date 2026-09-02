#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "infrared_sensor.h"
#include "vision_line.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Pseudo-infrared sensor: converts the camera line result (vision_result_t)
 * into the same four-bit black mask that the LQ_R4CHVB infrared board
 * produces, so the original infrared line_follow / obstacle_avoidance state
 * machines run unchanged.
 *
 * Channel ordering is authoritative from the reference project (lnf_avoid),
 * viewed from the front of the car:
 *
 *       car left                         car right
 *       channel 4  channel 3  channel 2  channel 1
 *
 * bit 0 = channel 1 (car right), bit 3 = channel 4 (car left).
 *
 * Semantics match infrared_sensor_sample(): a new mask is only published when
 * the vision frame sequence advances; between frames the previous mask is held
 * so the 10 ms control loop sees a stable value. A confirmed finish marker
 * (VISION_FINISH_CONFIRM_FRAMES consecutive frames) is reported as all-black,
 * which triggers the infrared controller's all-black stop.
 */
esp_err_t pseudo_infrared_sample(const vision_result_t *vision,
                                 infrared_sensor_state_t *state);

/* Same CH4 CH3 CH2 CH1 order as infrared_sensor_format(). */
void pseudo_infrared_format(uint8_t black_mask, char output[5]);

#ifdef __cplusplus
}
#endif
