#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "vision_line.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    VISION_FOLLOW_WAITING_CAMERA = 0,
    VISION_FOLLOW_TRACKING,
    VISION_FOLLOW_CORNER,
    VISION_FOLLOW_LOST,
    VISION_FOLLOW_FINISHED,
    VISION_FOLLOW_FAULT_STOP,
} vision_follow_state_t;

typedef struct {
    vision_follow_state_t state;
    int last_error;
    int last_turn_direction;
    uint32_t last_frame_sequence;
    uint32_t lost_ms;
    unsigned finish_confirm_frames;
} vision_follow_controller_t;

typedef struct {
    vision_follow_state_t state;
    int forward;
    int turn;
    int error;
    bool finished;
    bool state_changed;
} vision_follow_output_t;

void vision_follow_init(vision_follow_controller_t *controller);
vision_follow_output_t vision_follow_update(
    vision_follow_controller_t *controller,
    const vision_result_t *vision,
    bool finish_enabled,
    uint32_t elapsed_ms);
const char *vision_follow_state_name(vision_follow_state_t state);

#ifdef __cplusplus
}
#endif
