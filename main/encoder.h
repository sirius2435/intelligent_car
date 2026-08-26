#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ENCODER_WHEEL_LEFT = 0,
    ENCODER_WHEEL_RIGHT,
    ENCODER_WHEEL_REAR,
    ENCODER_WHEEL_COUNT,
} encoder_wheel_t;

esp_err_t encoder_init(void);
bool encoder_is_enabled(void);
esp_err_t encoder_get_count(encoder_wheel_t wheel, int *count);
esp_err_t encoder_get_all(int *left_count, int *right_count, int *rear_count);
esp_err_t encoder_clear(encoder_wheel_t wheel);
esp_err_t encoder_clear_all(void);

#ifdef __cplusplus
}
#endif
