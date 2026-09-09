#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Two-axis MG90S camera gimbal (pan + tilt).
 *
 * The pan/tilt GPIOs come from board_config.h (CAMERA_PAN_SERVO_GPIO and
 * CAMERA_TILT_SERVO_GPIO) and default to -1 (NULL). While both are -1,
 * camera_gimbal_init() logs a warning and stays disabled, so the camera
 * remains fixed. Once real GPIOs are set, init() drives a dedicated 50 Hz
 * LEDC timer, centres both axes, and every later command is clamped to the
 * per-axis MIN/MAX degrees so the servo horn never pushes into a mechanical
 * stop (the downward tilt is blocked by the chassis; straight up is the tilt
 * maximum).
 *
 * Angle arguments are servo-shaft degrees. A disabled axis (its GPIO is -1)
 * is a no-op that returns ESP_OK, so a pan-only or tilt-only mount works.
 */
esp_err_t camera_gimbal_init(void);
bool camera_gimbal_enabled(void);

esp_err_t camera_gimbal_set_pan(int angle_deg);
esp_err_t camera_gimbal_set_tilt(int angle_deg);
esp_err_t camera_gimbal_set(int pan_deg, int tilt_deg);
esp_err_t camera_gimbal_center(void);

/* Last clamped angles actually applied; either pointer may be NULL. */
void camera_gimbal_get_position(int *pan_deg, int *tilt_deg);

#ifdef __cplusplus
}
#endif
