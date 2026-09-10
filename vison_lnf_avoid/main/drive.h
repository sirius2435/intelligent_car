#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Host tests compile this with -Itests/stubs, which supplies a minimal
 * esp_err.h - the same arrangement every other header here relies on. */
#include "esp_err.h"

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

/* The trailing `applied` out-parameter of every setter below is test-only
 * instrumentation: it reports the per-wheel command/PWM the mixer and the PI
 * loop actually produced, which is otherwise unobservable from outside
 * drive.c. main.c passes NULL. tests/drive_mix_test.c depends on it, so do not
 * delete it as "dead code" without deleting that test's assertions first. */
esp_err_t drive_init(void);
/* Also exported for tests/drive_mix_test.c; the firmware only calls it through
 * the setters below. */
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
/* Test-only instrumentation: the firmware never reads this. It exists so
 * tests/drive_mix_test.c can pin the wheel PI loop's internals (target vs.
 * measured cps and the resulting PWM, including the anti-windup clamp), which
 * are otherwise unobservable from outside drive.c. Cost is a few dozen int
 * stores per control tick. Do not delete it as "dead code" without first
 * deleting that test's feedback assertions. */
void drive_get_feedback_status(drive_feedback_status_t *status);
esp_err_t drive_stop(void);

#ifdef __cplusplus
}
#endif
