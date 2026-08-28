#include "drive.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "board_config.h"
#include "motor.h"

#define DRIVE_COMMAND_MAX  1000

typedef struct {
    bool initialized;
    bool has_measurement;
    int command_sign;
    int sample_start_count;
    uint32_t sample_ms;
    int filtered_cps;
    int integral_cps_s;
    int output_magnitude;
} wheel_feedback_controller_t;

typedef enum {
    FEEDBACK_MODE_NONE = 0,
    FEEDBACK_MODE_LATERAL,
    FEEDBACK_MODE_FORWARD,
    FEEDBACK_MODE_APPROACH,
} feedback_mode_t;

static wheel_feedback_controller_t s_feedback[3];
static drive_feedback_status_t s_feedback_status;
static feedback_mode_t s_feedback_mode;

static int clamp_int(int value, int minimum, int maximum)
{
    if (value < minimum) {
        return minimum;
    }
    return value > maximum ? maximum : value;
}

static int command_sign(int command)
{
    return command < 0 ? -1 : (command > 0 ? 1 : 0);
}

static int target_cps(int command)
{
    return abs(command) * DRIVE_TARGET_CPS_PER_COMMAND_NUM /
           DRIVE_TARGET_CPS_PER_COMMAND_DEN;
}

static int lateral_feedforward(int command, int minimum_pwm)
{
    const int magnitude = abs(command);
    return minimum_pwm +
           magnitude * (DRIVE_LATERAL_MAX_PWM - minimum_pwm) /
               DRIVE_COMMAND_MAX;
}

static void reset_feedback(void)
{
    memset(s_feedback, 0, sizeof(s_feedback));
    memset(&s_feedback_status, 0, sizeof(s_feedback_status));
    s_feedback_mode = FEEDBACK_MODE_NONE;
}

static int update_wheel_feedback(wheel_feedback_controller_t *controller,
                                 int target_command,
                                 int encoder_count,
                                 uint32_t elapsed_ms,
                                 int startup_minimum_pwm,
                                 int running_minimum_pwm,
                                 int *measured_cps)
{
    const int sign = command_sign(target_command);
    if (sign == 0) {
        *controller = (wheel_feedback_controller_t) {0};
        *measured_cps = 0;
        return 0;
    }

    if (!controller->initialized || controller->command_sign != sign) {
        *controller = (wheel_feedback_controller_t) {
            .initialized = true,
            .command_sign = sign,
            .sample_start_count = encoder_count,
            /* Use the proven 260-PWM level only as a short starting kick. */
            .output_magnitude = lateral_feedforward(
                target_command, startup_minimum_pwm),
        };
        *measured_cps = 0;
        return sign * controller->output_magnitude;
    }

    controller->sample_ms += elapsed_ms;
    if (controller->sample_ms >= DRIVE_SPEED_CONTROL_PERIOD_MS) {
        int64_t delta = (int64_t)encoder_count - controller->sample_start_count;
        if (delta < 0) {
            delta = -delta;
        }
        const int raw_cps = controller->sample_ms == 0 ? 0 :
            (int)(delta * 1000 / controller->sample_ms);
        if (controller->has_measurement) {
            controller->filtered_cps =
                (3 * controller->filtered_cps + raw_cps) / 4;
        } else {
            controller->filtered_cps = raw_cps;
            controller->has_measurement = true;
        }

        const int error_cps = target_cps(target_command) -
                              controller->filtered_cps;
        const int candidate_integral = clamp_int(
            controller->integral_cps_s +
                error_cps * (int)controller->sample_ms / 1000,
            -DRIVE_SPEED_INTEGRAL_LIMIT,
            DRIVE_SPEED_INTEGRAL_LIMIT);

        int correction =
            DRIVE_SPEED_KP_NUM * error_cps / DRIVE_SPEED_KP_DEN +
            DRIVE_SPEED_KI_NUM * candidate_integral /
            DRIVE_SPEED_KI_DEN;
        const int feedforward =
            lateral_feedforward(target_command, running_minimum_pwm);
        const int unconstrained_output = feedforward + correction;

        /* Do not wind the integrator farther into a saturated limit.  This
         * lets a wheel recover immediately if the lower PWM makes it stall. */
        const bool pushes_below_minimum =
            unconstrained_output < running_minimum_pwm && error_cps < 0;
        const bool pushes_above_maximum =
            unconstrained_output > DRIVE_LATERAL_MAX_PWM && error_cps > 0;
        if (!pushes_below_minimum && !pushes_above_maximum) {
            controller->integral_cps_s = candidate_integral;
        } else {
            correction =
                DRIVE_SPEED_KP_NUM * error_cps / DRIVE_SPEED_KP_DEN +
                DRIVE_SPEED_KI_NUM * controller->integral_cps_s /
                    DRIVE_SPEED_KI_DEN;
        }
        controller->output_magnitude =
            clamp_int(feedforward + correction,
                      running_minimum_pwm,
                      DRIVE_LATERAL_MAX_PWM);
        controller->sample_start_count = encoder_count;
        controller->sample_ms = 0;
    }

    *measured_cps = controller->filtered_cps;
    return sign * controller->output_magnitude;
}

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
    reset_feedback();
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
     * [left, right, rear] = [-lateral/2, lateral/2, -lateral].
     */
    int left = forward + turn - lateral / 2;
    int right = forward - turn + lateral / 2;
    int rear = -lateral;

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
    reset_feedback();
    drive_wheel_command_t command = {0};
    drive_mix_motion(forward, lateral, turn, &command);
    if (applied != NULL) {
        *applied = command;
    }
    return motor_set_all(command.left, command.right, command.rear);
}

esp_err_t drive_set_motion_feedback(int forward,
                                    int lateral,
                                    int turn,
                                    uint32_t elapsed_ms,
                                    int left_count,
                                    int right_count,
                                    int rear_count,
                                    drive_wheel_command_t *applied)
{
    if (lateral == 0) {
        return drive_set_motion(forward, lateral, turn, applied);
    }

    if (s_feedback_mode != FEEDBACK_MODE_LATERAL) {
        reset_feedback();
        s_feedback_mode = FEEDBACK_MODE_LATERAL;
    }

    drive_wheel_command_t target = {0};
    drive_mix_motion(forward, lateral, turn, &target);
    const int running_minimum_pwm = lateral > 0 ?
        DRIVE_LEFT_STRAFE_MIN_ACTIVE_PWM : DRIVE_RIGHT_STRAFE_MIN_ACTIVE_PWM;
    drive_wheel_command_t pwm = {0};
    drive_wheel_command_t measured = {0};
    pwm.left = update_wheel_feedback(&s_feedback[0], target.left,
                                     left_count, elapsed_ms,
                                     DRIVE_LATERAL_MIN_ACTIVE_PWM,
                                     running_minimum_pwm,
                                     &measured.left);
    pwm.right = update_wheel_feedback(&s_feedback[1], target.right,
                                      right_count, elapsed_ms,
                                      DRIVE_LATERAL_MIN_ACTIVE_PWM,
                                      running_minimum_pwm,
                                      &measured.right);
    pwm.rear = update_wheel_feedback(&s_feedback[2], target.rear,
                                     rear_count, elapsed_ms,
                                     DRIVE_LATERAL_MIN_ACTIVE_PWM,
                                     running_minimum_pwm,
                                     &measured.rear);

    s_feedback_status = (drive_feedback_status_t) {
        .active = true,
        .target_cps = {
            .left = command_sign(target.left) * target_cps(target.left),
            .right = command_sign(target.right) * target_cps(target.right),
            .rear = command_sign(target.rear) * target_cps(target.rear),
        },
        .measured_cps = {
            .left = command_sign(target.left) * measured.left,
            .right = command_sign(target.right) * measured.right,
            .rear = command_sign(target.rear) * measured.rear,
        },
        .pwm = pwm,
    };
    if (applied != NULL) {
        *applied = pwm;
    }
    return motor_set_all(pwm.left, pwm.right, pwm.rear);
}

esp_err_t drive_set_forward_feedback(int forward,
                                     uint32_t elapsed_ms,
                                     int left_count,
                                     int right_count,
                                     drive_wheel_command_t *applied)
{
    if (forward == 0) {
        return drive_set_motion(0, 0, 0, applied);
    }
    if (s_feedback_mode != FEEDBACK_MODE_FORWARD) {
        reset_feedback();
        s_feedback_mode = FEEDBACK_MODE_FORWARD;
    }

    drive_wheel_command_t pwm = {0};
    drive_wheel_command_t measured = {0};
    pwm.left = update_wheel_feedback(&s_feedback[0], forward,
                                     left_count, elapsed_ms,
                                     DRIVE_LATERAL_MIN_ACTIVE_PWM,
                                     DRIVE_FORWARD_MIN_ACTIVE_PWM,
                                     &measured.left);
    pwm.right = update_wheel_feedback(&s_feedback[1], forward,
                                      right_count, elapsed_ms,
                                      DRIVE_LATERAL_MIN_ACTIVE_PWM,
                                      DRIVE_FORWARD_MIN_ACTIVE_PWM,
                                      &measured.right);
    /* The rear wheel is not driven during straight motion. */
    s_feedback[2] = (wheel_feedback_controller_t) {0};

    s_feedback_status = (drive_feedback_status_t) {
        .active = true,
        .target_cps = {
            .left = target_cps(forward),
            .right = target_cps(forward),
            .rear = 0,
        },
        .measured_cps = {
            .left = measured.left,
            .right = measured.right,
            .rear = 0,
        },
        .pwm = pwm,
    };
    if (applied != NULL) {
        *applied = pwm;
    }
    return motor_set_all(pwm.left, pwm.right, 0);
}

esp_err_t drive_set_approach_feedback(int forward,
                                      int turn,
                                      uint32_t elapsed_ms,
                                      int left_count,
                                      int right_count,
                                      drive_wheel_command_t *applied)
{
    if (forward == 0) {
        return drive_set_motion(forward, 0, turn, applied);
    }
    if (s_feedback_mode != FEEDBACK_MODE_APPROACH) {
        reset_feedback();
        s_feedback_mode = FEEDBACK_MODE_APPROACH;
    }

    drive_wheel_command_t target = {0};
    drive_mix_motion(forward, 0, turn, &target);
    drive_wheel_command_t pwm = {0};
    drive_wheel_command_t measured = {0};
    pwm.left = update_wheel_feedback(&s_feedback[0], target.left,
                                     left_count, elapsed_ms,
                                     DRIVE_APPROACH_STARTUP_PWM,
                                     DRIVE_APPROACH_MIN_ACTIVE_PWM,
                                     &measured.left);
    pwm.right = update_wheel_feedback(&s_feedback[1], target.right,
                                      right_count, elapsed_ms,
                                      DRIVE_APPROACH_STARTUP_PWM,
                                      DRIVE_APPROACH_MIN_ACTIVE_PWM,
                                      &measured.right);
    s_feedback[2] = (wheel_feedback_controller_t) {0};

    s_feedback_status = (drive_feedback_status_t) {
        .active = true,
        .target_cps = {
            .left = command_sign(target.left) * target_cps(target.left),
            .right = command_sign(target.right) * target_cps(target.right),
            .rear = 0,
        },
        .measured_cps = {
            .left = command_sign(target.left) * measured.left,
            .right = command_sign(target.right) * measured.right,
            .rear = 0,
        },
        .pwm = pwm,
    };
    if (applied != NULL) {
        *applied = pwm;
    }
    return motor_set_all(pwm.left, pwm.right, 0);
}

void drive_get_feedback_status(drive_feedback_status_t *status)
{
    if (status != NULL) {
        *status = s_feedback_status;
    }
}

esp_err_t drive_stop(void)
{
    reset_feedback();
    return motor_stop_all();
}
