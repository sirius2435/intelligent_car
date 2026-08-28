#include "drive.h"

#include <stdlib.h>

#include "motor.h"
#include "board_config.h"

#define DRIVE_COMMAND_MAX 1000

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


/*
 * ============================================================
 * Normal line-following motion
 * ============================================================
 *
 * This is the original two-front-wheel differential drive.
 *
 * rear = 0
 *
 * IMPORTANT:
 * Do not change this part when tuning obstacle avoidance.
 */
esp_err_t drive_set_motion(int forward,
                           int turn,
                           drive_wheel_command_t *applied)
{
    int left = forward + turn;
    int right = forward - turn;

    /* Rear wheel remains disabled during normal line following. */
    int rear = 0;

    const int maximum =
        maximum_magnitude(left, right, rear);

    if (maximum > DRIVE_COMMAND_MAX) {
        left = left * DRIVE_COMMAND_MAX / maximum;
        right = right * DRIVE_COMMAND_MAX / maximum;
        rear = 0;
    }

    if (applied != NULL) {
        applied->left = left;
        applied->right = right;
        applied->rear = rear;
    }

    return motor_set_all(left, right, rear);
}


/*
 * ============================================================
 * Lateral movement for obstacle avoidance
 * ============================================================
 *
 * direction:
 *
 *     -1 = move LEFT
 *     +1 = move RIGHT
 *
 * During lateral movement:
 *
 *     front wheels = 150
 *     rear wheel   = 300
 *
 * Therefore:
 *
 *     front : rear = 1 : 2
 *
 * The rear wheel rotates in the opposite direction to the
 * two front wheels.
 *
 * This function is ONLY used by obstacle avoidance.
 */
/*
 * ============================================================
 * Lateral movement for obstacle avoidance
 * ============================================================
 *
 * Three-wheel omni-directional movement.
 *
 * direction:
 *
 *     -1 = LEFT
 *     +1 = RIGHT
 *
 * Wheel relationship:
 *
 *     LEFT SHIFT:
 *
 *          front-left   <- 150
 *          front-right  <- 150
 *          rear          -> 300
 *
 *
 *     RIGHT SHIFT:
 *
 *          front-left   -> 150
 *          front-right  -> 150
 *          rear          <- 300
 *
 *
 * Front : Rear = 1 : 2
 *
 * This function is ONLY used by obstacle avoidance.
 */
esp_err_t drive_set_lateral(int direction,
                            int rear_speed,
                            drive_wheel_command_t *applied)
{
    if (direction == 0) {
        return drive_stop();
    }

    /*
     * Normalize direction.
     *
     * -1 = LEFT
     * +1 = RIGHT
     */
    direction = (direction > 0) ? 1 : -1;

    /*
     * Always use positive speed magnitude.
     */
    if (rear_speed < 0) {
        rear_speed = -rear_speed;
    }

    if (rear_speed > DRIVE_COMMAND_MAX) {
        rear_speed = DRIVE_COMMAND_MAX;
    }

    /*
     * Front wheels are exactly half the rear wheel.
     */
    const int front_speed =
        (int)(rear_speed *
              OBSTACLE_SHIFT_FRONT_RATIO +
              0.5f);

    /*
     * ========================================================
     * Lateral kinematics
     * ========================================================
     *
     * RIGHT:
     *
     *     left  = +front
     *     right = +front
     *     rear  = -rear
     *
     * LEFT:
     *
     *     left  = -front
     *     right = -front
     *     rear  = +rear
     *
     * Therefore:
     *
     *     left  = direction * front
     *     right = direction * front
     *     rear  = -direction * rear
     */
    const int left =
        direction * front_speed;

    const int right =
        direction * front_speed;

    const int rear =
        -direction * rear_speed;

    /*
     * Save the actual command for logging.
     */
    if (applied != NULL) {
        applied->left = left;
        applied->right = right;
        applied->rear = rear;
    }

    /*
     * Send command to all three wheels.
     */
    return motor_set_all(
        left,
        right,
        rear
    );
}


/*
 * Stop all three wheels.
 */
esp_err_t drive_stop(void)
{
    return motor_stop_all();
}