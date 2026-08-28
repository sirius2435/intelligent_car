#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t motor_init(void);
esp_err_t motor_set_all(int left_speed, int right_speed, int rear_speed);
esp_err_t motor_stop_all(void);

#ifdef __cplusplus
}
#endif
