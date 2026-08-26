#pragma once

#if defined(__has_include)
#  if __has_include("esp_err.h")
#    include "esp_err.h"
#  else
typedef int esp_err_t;
#  endif
#else
#  include "esp_err.h"
#endif

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
