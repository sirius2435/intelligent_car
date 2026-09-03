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

/*
 * Drive the three omni wheels from a 3-DOF body command.
 *   forward > 0 : car moves forward.
 *   strafe > 0  : car moves left (bench-verified), negative moves right.
 *   turn > 0    : car turns right (matches the existing convention).
 */
esp_err_t drive_set_motion(int forward, int strafe, int turn,
                           drive_wheel_command_t *applied);
esp_err_t drive_stop(void);

#ifdef __cplusplus
}
#endif
