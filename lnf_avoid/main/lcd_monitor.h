#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Start the LCD dashboard task: shows the three wheel speeds (rpm) and the
 * filtered HC-SR04 distance on the LQ_TFT18SPI panel. The car control loop
 * is untouched; if the panel is missing the car still drives normally. */
esp_err_t lcd_monitor_start(void);

#ifdef __cplusplus
}
#endif
