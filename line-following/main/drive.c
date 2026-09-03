#include "drive.h"

#include <stdlib.h>

#include "motor.h"

#define DRIVE_COMMAND_MAX  1000
#define SIN_60_PERMILLE     866
#define COS_60_PERMILLE     500

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

esp_err_t drive_set_motion(int forward, int strafe, int turn,
                           drive_wheel_command_t *applied)
{
    /*
     * Standard 120-degree omni layout: front-left, front-right, rear-center.
     * Wheel angles 120 deg (left), 240 deg (right), 0 deg (rear) give the
     * inverse kinematics (strafe > 0 moves the car left, bench-verified):
     *   left  = -0.866*forward + 0.5*strafe - turn
     *   right = +0.866*forward + 0.5*strafe - turn
     *   rear  =                  -1.0*strafe - turn
     * Positive turn is defined as a right turn at this interface. The rear
     * wheel's turn sign is still bench-unverified; pure-strafe commands
     * (turn == 0) are unaffected by it.
     */
    int left = -(SIN_60_PERMILLE * forward) / 1000
               + (COS_60_PERMILLE * strafe) / 1000 - turn;
    int right = (SIN_60_PERMILLE * forward) / 1000
                + (COS_60_PERMILLE * strafe) / 1000 - turn;
    int rear = -strafe - turn;

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
