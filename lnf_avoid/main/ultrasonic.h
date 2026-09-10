#pragma once

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ULTRASONIC_READING_UNAVAILABLE = 0,
    ULTRASONIC_READING_VALID,
    ULTRASONIC_READING_NO_ECHO,
    ULTRASONIC_READING_ERROR,
} ultrasonic_reading_status_t;

typedef struct {
    ultrasonic_reading_status_t status;
    uint32_t distance_mm;
    uint32_t sequence;
} ultrasonic_reading_t;

esp_err_t ultrasonic_init(void);
esp_err_t ultrasonic_get_latest(ultrasonic_reading_t *reading);
const char *ultrasonic_status_name(ultrasonic_reading_status_t status);

#ifdef __cplusplus
}
#endif
