#include "drive.h"

#include <stdlib.h>

#include "motor.h"

#define DRIVE_COMMAND_MAX  1000

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
     * Classic two-wheel differential drive. Positive turn commands a right
     * turn: the left wheel speeds up and the right wheel slows down. The
     * rear wheel is passive and never driven.
     */
    int left = forward + turn;
    int right = forward - turn;
    int rear = 0;

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
