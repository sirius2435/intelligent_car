#include <stdio.h>
#include <stdlib.h>

#include "board_config.h"
#include "drive.h"
#include "motor.h"

#define CHECK(condition) do {                                                   \
    if (!(condition)) {                                                         \
        fprintf(stderr, "FAIL %s:%d: %s\n", __func__, __LINE__, #condition);   \
        exit(1);                                                                \
    }                                                                           \
} while (0)

static drive_wheel_command_t last_motor_command;

esp_err_t motor_init(void) { return 0; }
esp_err_t motor_set_all(int left, int right, int rear)
{
    last_motor_command = (drive_wheel_command_t) {
        .left = left,
        .right = right,
        .rear = rear,
    };
    return 0;
}
esp_err_t motor_stop_all(void) { return 0; }

int main(void)
{
    drive_wheel_command_t command;

    drive_mix_motion(160, 0, 0, &command);
    CHECK(command.left == 160 && command.right == 160 && command.rear == 0);

    drive_mix_motion(0, 300, 0, &command);
    CHECK(command.left == -150 && command.right == 150 && command.rear == -300);

    drive_mix_motion(0, -300, 0, &command);
    CHECK(command.left == 150 && command.right == -150 && command.rear == 300);

    drive_mix_motion(0, 0, 200, &command);
    CHECK(command.left == 200 && command.right == -200 && command.rear == 0);

    drive_mix_motion(900, 600, 500, &command);
    CHECK(abs(command.left) <= 1000);
    CHECK(abs(command.right) <= 1000);
    CHECK(abs(command.rear) <= 1000);
    CHECK(command.left == 1000);

    /* Ordinary straight motion remains open loop for line following. */
    drive_set_motion_feedback(140, 0, 0, 10, 0, 0, 0, &command);
    CHECK(command.left == 140 && command.right == 140 && command.rear == 0);

    /* Obstacle-pass forward motion uses a starting kick, then independently
       raises PWM for a stalled wheel. */
    drive_set_forward_feedback(140, 10, 0, 0, &command);
    CHECK(command.left == 321 && command.right == 321 && command.rear == 0);
    drive_set_forward_feedback(140, 50, -14, 0, &command);
    CHECK(command.left < command.right);
    CHECK(command.left >= DRIVE_FORWARD_MIN_ACTIVE_PWM);
    CHECK(command.right > DRIVE_FORWARD_MIN_ACTIVE_PWM);

    drive_feedback_status_t forward_feedback;
    drive_get_feedback_status(&forward_feedback);
    CHECK(forward_feedback.active);
    CHECK(forward_feedback.target_cps.left == 280);
    CHECK(forward_feedback.target_cps.right == 280);
    CHECK(forward_feedback.measured_cps.left == 280);
    CHECK(forward_feedback.measured_cps.right == 0);

    /* The 10 cm approach keeps its 90 command as a speed target, but boosts
       a stalled wheel instead of waiting for a manual push. */
    drive_set_motion(0, 0, 0, NULL);
    drive_set_approach_feedback(90, 0, 10, 0, 0, &command);
    CHECK(command.left > 90 && command.right > 90 && command.rear == 0);
    drive_set_approach_feedback(90, 0, 50, -9, 0, &command);
    CHECK(command.left < command.right);
    CHECK(command.right > DRIVE_APPROACH_MIN_ACTIVE_PWM);

    /* Entering lateral control applies static-friction feed-forward. */
    drive_set_motion(0, 0, 0, NULL);
    drive_set_motion_feedback(0, 300, 0, 10, 0, 0, 0, &command);
    CHECK(command.left == -326);
    CHECK(command.right == 326);
    CHECK(command.rear == -392);
    CHECK(last_motor_command.left == command.left);

    /* After the first 50 ms speed sample, left strafe drops from its starting
       kick to the lower running range.  A stalled wheel still receives more
       PWM than wheels already at their target rates. */
    drive_set_motion_feedback(0, 300, 0, 50, -15, 0, -30, &command);
    CHECK(abs(command.left) < 260);
    CHECK(command.right > abs(command.left));
    CHECK(abs(command.rear) < 392);

    /* The same measured overspeed seen in the real log must be able to drive
       left-strafe PWM below the old hard floor of 260. */
    drive_set_motion(0, 0, 0, NULL);
    drive_set_motion_feedback(0, 120, 0, 10, 0, 0, 0, &command);
    drive_set_motion_feedback(0, 120, 0, 50, 30, -30, 30, &command);
    CHECK(abs(command.left) < 260);
    CHECK(abs(command.right) < 260);
    CHECK(abs(command.rear) < 260);

    /* Right strafe keeps the same 260 startup kick, then drops to its current
       160 PWM running floor after the first speed sample. */
    drive_set_motion(0, 0, 0, NULL);
    drive_set_motion_feedback(0, -120, 0, 10, 0, 0, 0, &command);
    CHECK(command.left == 286);
    CHECK(command.right == -286);
    CHECK(command.rear == 312);
    drive_set_motion_feedback(0, -120, 0, 50, -30, 30, -30, &command);
    CHECK(abs(command.left) == DRIVE_RIGHT_STRAFE_MIN_ACTIVE_PWM);
    CHECK(abs(command.right) == DRIVE_RIGHT_STRAFE_MIN_ACTIVE_PWM);
    CHECK(abs(command.rear) == DRIVE_RIGHT_STRAFE_MIN_ACTIVE_PWM);

    drive_feedback_status_t feedback;
    drive_get_feedback_status(&feedback);
    CHECK(feedback.active);
    CHECK(feedback.target_cps.left == 120);
    CHECK(feedback.target_cps.right == -120);
    CHECK(feedback.target_cps.rear == 240);
    CHECK(feedback.measured_cps.left == 600);
    CHECK(feedback.measured_cps.right == -600);
    CHECK(feedback.measured_cps.rear == 600);

    puts("drive mix tests passed");
    return 0;
}
