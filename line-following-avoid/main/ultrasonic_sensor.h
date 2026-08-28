#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t ultrasonic_sensor_init(void);

/* Measure the current distance in centimeters.
 * Returns 0 when no valid echo is received before the timeout. */
esp_err_t ultrasonic_sensor_measure_cm(float *distance_cm);

#ifdef __cplusplus
}
#endif
