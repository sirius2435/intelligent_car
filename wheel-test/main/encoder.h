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

/*
 * Initialize three quadrature counters. If all encoder GPIO values are -1,
 * this is a successful no-op so the open-loop motor test remains usable.
 */
esp_err_t encoder_init(void);

/* True after encoder_init() has successfully started all three counters. */
bool encoder_is_enabled(void);

/* Read a signed x4 quadrature count. Positive should correspond to forward. */
esp_err_t encoder_get_count(encoder_wheel_t wheel, int *count);
esp_err_t encoder_get_all(int *left_count, int *right_count, int *rear_count);

/* Clear both the hardware count and its software overflow accumulator. */
esp_err_t encoder_clear(encoder_wheel_t wheel);
esp_err_t encoder_clear_all(void);

#ifdef __cplusplus
}
#endif
