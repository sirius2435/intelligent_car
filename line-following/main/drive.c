#include "drive.h"

#include <stdlib.h>

#include "motor.h"

#define DRIVE_COMMAND_MAX  1000
#define SIN_60_PERMILLE     866

static int maximum_magnitude(int a, int b, int c)
{
    int maximum = abs(a);
    if (abs(b) > maximum) {
        maximum = abs(b);
    }
    if (abs(c) > maximum) {
        maximum = abs(c);
    }
    return maximum;
}

esp_err_t drive_init(void)
{
    return motor_init();
}

esp_err_t drive_set_motion(int forward, int turn, drive_wheel_command_t *applied)
{
    /*
     * Standard 120-degree omni layout: front-left, front-right, rear-center.
     * Positive turn is defined as a right turn at this interface.
     */
    int left = -(SIN_60_PERMILLE * forward) / 1000 + turn;
    int right = (SIN_60_PERMILLE * forward) / 1000 + turn;
    int rear = turn;

    const int maximum = maximum_magnitude(left, right, rear);
    if (maximum > DRIVE_COMMAND_MAX) {
        left = left * DRIVE_COMMAND_MAX / maximum;
        right = right * DRIVE_COMMAND_MAX / maximum;
        rear = rear * DRIVE_COMMAND_MAX / maximum;
    }

    if (applied != NULL) {
        applied->left = left;
        applied->right = right;
        applied->rear = rear;
    }
    return motor_set_all(left, right, rear);
}

esp_err_t drive_stop(void)
{
    return motor_stop_all();
}
