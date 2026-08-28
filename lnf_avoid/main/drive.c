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

void drive_mix_motion(int forward,
                      int lateral,
                      int turn,
                      drive_wheel_command_t *command)
{
    /*
     * Keep the verified differential line-following behavior, then add the
     * calibrated three-omni-wheel lateral vector. Positive lateral is left:
     * [left, right, rear] = [lateral/2, lateral/2, -lateral/2].
     */
    int left = forward + turn + lateral / 2;
    int right = forward - turn + lateral / 2;
    int rear = -lateral / 2;

    const int maximum = maximum_magnitude(left, right, rear);
    if (maximum > DRIVE_COMMAND_MAX) {
        left = left * DRIVE_COMMAND_MAX / maximum;
        right = right * DRIVE_COMMAND_MAX / maximum;
        rear = rear * DRIVE_COMMAND_MAX / maximum;
    }

    if (command != NULL) {
        command->left = left;
        command->right = right;
        command->rear = rear;
    }
}

esp_err_t drive_set_motion(int forward,
                           int lateral,
                           int turn,
                           drive_wheel_command_t *applied)
{
    drive_wheel_command_t command = {0};
    drive_mix_motion(forward, lateral, turn, &command);
    if (applied != NULL) {
        *applied = command;
    }
    return motor_set_all(command.left, command.right, command.rear);
}

esp_err_t drive_stop(void)
{
    return motor_stop_all();
}
