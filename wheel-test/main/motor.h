#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Initialize the three configured motor channels. The driver starts stopped. */
esp_err_t motor_init(void);

/* Speed range: -1000..1000. Positive is forward, negative is reverse. */
esp_err_t motor_set_left(int speed);
esp_err_t motor_set_right(int speed);
esp_err_t motor_set_rear(int speed);
esp_err_t motor_set_all(int left_speed, int right_speed, int rear_speed);

/* Set all three channels to zero and put a configured STBY pin low. */
esp_err_t motor_stop_all(void);

#ifdef __cplusplus
}
#endif
