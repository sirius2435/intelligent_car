#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t status_led_init(void);
esp_err_t status_led_set_enabled(bool enabled);
esp_err_t status_led_show_sensor(uint8_t black_mask);

#ifdef __cplusplus
}
#endif
