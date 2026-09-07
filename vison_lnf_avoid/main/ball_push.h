#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "ball_vision.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Fixed, encoder-delimited route used after a confirmed END marker. */
typedef enum {
    BALL_PUSH_SETTLE = 0,
    BALL_PUSH_ADVANCE,
    BALL_PUSH_ROTATE_RIGHT,
    BALL_PUSH_ALIGN_RED,
    BALL_PUSH_PUSH_RED,
    BALL_PUSH_RETREAT_RED,
    BALL_PUSH_RETURN_CENTER,
    BALL_PUSH_ALIGN_WHITE,
    BALL_PUSH_PUSH_WHITE,
    BALL_PUSH_COMPLETE,
    BALL_PUSH_FAULT_STOP,
} ball_push_state_t;

typedef struct {
    ball_push_state_t state;
    ball_push_state_t next_state;
    ball_color_t target_color;
    unsigned completed_balls;
    uint32_t state_ms;
    int start_left_count;
    int start_right_count;
    int start_rear_count;
    int64_t last_progress;
    uint32_t stall_ms;
} ball_push_controller_t;

typedef struct {
    ball_push_state_t state;
    ball_color_t target_color;
    int forward;
    int lateral;
    int turn;
    bool active;
    bool just_completed_ball;
    bool mission_complete;
    bool fault;
    bool state_changed;
} ball_push_result_t;

void ball_push_init(ball_push_controller_t *controller,
                    int left_count, int right_count, int rear_count);
ball_push_result_t ball_push_update(ball_push_controller_t *controller,
                                    uint32_t elapsed_ms,
                                    int left_count,
                                    int right_count,
                                    int rear_count);
const char *ball_push_state_name(ball_push_state_t state);

#ifdef __cplusplus
}
#endif
