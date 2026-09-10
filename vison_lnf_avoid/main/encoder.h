#pragma once

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
esp_err_t encoder_get_count(encoder_wheel_t wheel, int *count);
esp_err_t encoder_get_all(int *left_count, int *right_count, int *rear_count);

#ifdef __cplusplus
}
#endif
