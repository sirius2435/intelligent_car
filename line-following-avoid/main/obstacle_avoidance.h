#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "drive.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    OBSTACLE_IDLE = 0,
    OBSTACLE_STOP,
    OBSTACLE_SHIFT_LEFT,
    OBSTACLE_SETTLE_AFTER_LEFT,
    OBSTACLE_FORWARD,
    OBSTACLE_SETTLE_AFTER_FORWARD,
    OBSTACLE_SHIFT_RIGHT,
} obstacle_state_t;

typedef struct {
    obstacle_state_t state;
    uint32_t state_ms;
    uint32_t cooldown_ms;
    bool saw_white;
} obstacle_controller_t;

void obstacle_avoidance_init(obstacle_controller_t *controller);

/* Returns true when obstacle avoidance owns the drive system. */
bool obstacle_avoidance_active(const obstacle_controller_t *controller);

/*
 * Update the obstacle state machine.
 * distance_cm <= 0 means "no valid ultrasonic echo" and never triggers avoidance.
 * black_mask is the debounced infrared black-line mask.
 */
esp_err_t obstacle_avoidance_update(obstacle_controller_t *controller,
                                    uint8_t black_mask,
                                    float distance_cm,
                                    uint32_t elapsed_ms,
                                    drive_wheel_command_t *applied,
                                    bool *finished);

const char *obstacle_avoidance_state_name(obstacle_state_t state);

#ifdef __cplusplus
}
#endif
