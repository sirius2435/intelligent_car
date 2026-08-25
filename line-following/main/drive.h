#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int left;
    int right;
    int rear;
} drive_wheel_command_t;

esp_err_t drive_init(void);
esp_err_t drive_set_motion(int forward, int turn, drive_wheel_command_t *applied);
esp_err_t drive_stop(void);

#ifdef __cplusplus
}
#endif
