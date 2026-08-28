#include <stdio.h>
#include <stdlib.h>

#include "drive.h"
#include "motor.h"

#define CHECK(condition) do {                                                   \
    if (!(condition)) {                                                         \
        fprintf(stderr, "FAIL %s:%d: %s\n", __func__, __LINE__, #condition);   \
        exit(1);                                                                \
    }                                                                           \
} while (0)

esp_err_t motor_init(void) { return 0; }
esp_err_t motor_set_all(int left, int right, int rear)
{
    (void)left;
    (void)right;
    (void)rear;
    return 0;
}
esp_err_t motor_stop_all(void) { return 0; }

int main(void)
{
    drive_wheel_command_t command;

    drive_mix_motion(160, 0, 0, &command);
    CHECK(command.left == 160 && command.right == 160 && command.rear == 0);

    drive_mix_motion(0, 300, 0, &command);
    CHECK(command.left == 150 && command.right == 150 && command.rear == -150);

    drive_mix_motion(0, -300, 0, &command);
    CHECK(command.left == -150 && command.right == -150 && command.rear == 150);

    drive_mix_motion(0, 0, 200, &command);
    CHECK(command.left == 200 && command.right == -200 && command.rear == 0);

    drive_mix_motion(900, 600, 300, &command);
    CHECK(abs(command.left) <= 1000);
    CHECK(abs(command.right) <= 1000);
    CHECK(abs(command.rear) <= 1000);
    CHECK(command.left == 1000);

    puts("drive mix tests passed");
    return 0;
}
