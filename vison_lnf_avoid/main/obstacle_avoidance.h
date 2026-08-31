#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "ultrasonic.h"
#include "vision_line.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    AVOIDANCE_ARMED = 0,
    AVOIDANCE_BRAKE,
    AVOIDANCE_REVERSE,
    AVOIDANCE_STRAFE_LEFT,
    AVOIDANCE_FORWARD_PASS,
    AVOIDANCE_STRAFE_RIGHT_FIND_LINE,
    AVOIDANCE_STRAFE_RIGHT_ALIGN_LINE,
    AVOIDANCE_COMPLETE,
    AVOIDANCE_FAULT_STOP,
} obstacle_avoidance_state_t;

typedef struct {
    obstacle_avoidance_state_t state;
    uint32_t stage_ms;
    uint32_t sensor_silence_ms;
    uint32_t last_ultrasonic_sequence;
    unsigned trigger_confirm_count;
    unsigned clear_confirm_count;
    bool left_edge_confirmed;
    uint32_t left_clearance_ms;
    int start_left_count;
    int start_right_count;
    int start_rear_count;
    int64_t progress_counts;
    int64_t last_motion_counts;
    int64_t outbound_lateral_counts;
    uint32_t stall_ms;
    uint32_t centered_ms;
    uint32_t last_vision_sequence;
    unsigned line_confirm_count;
    unsigned center_confirm_count;
    int align_lateral_direction;
} obstacle_avoidance_controller_t;

typedef struct {
    obstacle_avoidance_state_t state;
    int forward;
    int lateral;
    int turn;
    int tracking_forward_limit;
    bool active;
    bool slow_approach;
    bool just_completed;
    bool state_changed;
} obstacle_avoidance_result_t;

void obstacle_avoidance_init(obstacle_avoidance_controller_t *controller);
obstacle_avoidance_result_t obstacle_avoidance_update(
    obstacle_avoidance_controller_t *controller,
    const ultrasonic_reading_t *ultrasonic,
    const vision_result_t *vision,
    uint32_t elapsed_ms,
    int left_count,
    int right_count,
    int rear_count);
const char *obstacle_avoidance_state_name(obstacle_avoidance_state_t state);

#ifdef __cplusplus
}
#endif
