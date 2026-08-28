#pragma once

#include <stdbool.h>
#include <stdint.h>

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

typedef struct {
    bool active;
    drive_wheel_command_t target_cps;
    drive_wheel_command_t measured_cps;
    drive_wheel_command_t pwm;
} drive_feedback_status_t;

esp_err_t drive_init(void);
void drive_mix_motion(int forward,
                      int lateral,
                      int turn,
                      drive_wheel_command_t *command);
/* Positive lateral moves the car left; positive turn rotates it right. */
esp_err_t drive_set_motion(int forward,
                           int lateral,
                           int turn,
                           drive_wheel_command_t *applied);
esp_err_t drive_set_motion_feedback(int forward,
                                    int lateral,
                                    int turn,
                                    uint32_t elapsed_ms,
                                    int left_count,
                                    int right_count,
                                    int rear_count,
                                    drive_wheel_command_t *applied);
/* Encoder speed control for the straight obstacle-passing stage only. */
esp_err_t drive_set_forward_feedback(int forward,
                                     uint32_t elapsed_ms,
                                     int left_count,
                                     int right_count,
                                     drive_wheel_command_t *applied);
/* Low-speed line-following approach with encoder anti-stall control. */
esp_err_t drive_set_approach_feedback(int forward,
                                      int turn,
                                      uint32_t elapsed_ms,
                                      int left_count,
                                      int right_count,
                                      drive_wheel_command_t *applied);
void drive_get_feedback_status(drive_feedback_status_t *status);
esp_err_t drive_stop(void);

#ifdef __cplusplus
}
#endif
