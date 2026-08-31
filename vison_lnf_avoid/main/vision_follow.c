#include "vision_follow.h"

#include <limits.h>
#include <stddef.h>
#include <stdlib.h>

#include "board_config.h"

static int clamp_int(int value, int minimum, int maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static uint32_t add_saturated(uint32_t value, uint32_t increment)
{
    return UINT32_MAX - value < increment ? UINT32_MAX : value + increment;
}

void vision_follow_init(vision_follow_controller_t *controller)
{
    if (controller == NULL) {
        return;
    }
    *controller = (vision_follow_controller_t) {
        .state = VISION_FOLLOW_WAITING_CAMERA,
        .last_turn_direction = 1,
    };
}

vision_follow_output_t vision_follow_update(
    vision_follow_controller_t *controller,
    const vision_result_t *vision,
    bool finish_enabled,
    uint32_t elapsed_ms)
{
    if (controller == NULL || vision == NULL) {
        return (vision_follow_output_t) {
            .state = VISION_FOLLOW_FAULT_STOP,
        };
    }

    const vision_follow_state_t old_state = controller->state;
    if (controller->state == VISION_FOLLOW_FINISHED ||
        controller->state == VISION_FOLLOW_FAULT_STOP) {
        return (vision_follow_output_t) {
            .state = controller->state,
            .finished = controller->state == VISION_FOLLOW_FINISHED,
        };
    }

    const bool new_frame = vision->sequence != 0U &&
                           vision->sequence != controller->last_frame_sequence;
    if (new_frame) {
        controller->last_frame_sequence = vision->sequence;
    }

    if (finish_enabled && new_frame && vision->finish_marker) {
        if (controller->finish_confirm_frames < UINT_MAX) {
            ++controller->finish_confirm_frames;
        }
    } else if (new_frame) {
        controller->finish_confirm_frames = 0;
    }
    if (controller->finish_confirm_frames >= VISION_FINISH_CONFIRM_FRAMES) {
        controller->state = VISION_FOLLOW_FINISHED;
        return (vision_follow_output_t) {
            .state = controller->state,
            .finished = true,
            .state_changed = old_state != controller->state,
        };
    }

    const bool usable_line = vision->frame_valid && vision->line_found &&
                             vision->confidence >= VISION_LINE_CONFIDENCE_MIN;
    if (!usable_line) {
        controller->lost_ms = add_saturated(controller->lost_ms, elapsed_ms);
        controller->state = VISION_FOLLOW_LOST;
        vision_follow_output_t output = {
            .state = controller->state,
            .state_changed = old_state != controller->state,
        };
        if (controller->lost_ms <= VISION_LOST_GRACE_MS) {
            output.forward = VISION_MIN_FORWARD;
            output.turn = controller->last_turn_direction *
                          (VISION_LOST_TURN / 2);
        } else if (controller->lost_ms <= VISION_LOST_SEARCH_MS) {
            output.turn = controller->last_turn_direction * VISION_LOST_TURN;
        } else {
            controller->state = VISION_FOLLOW_FAULT_STOP;
            output.state = controller->state;
            output.forward = 0;
            output.turn = 0;
            output.state_changed = true;
        }
        return output;
    }

    controller->lost_ms = 0;
    const int error = clamp_int(
        (vision->lateral_error * 3 + vision->heading_error) / 4,
        -1000, 1000);
    int derivative = 0;
    if (new_frame) {
        derivative = error - controller->last_error;
        controller->last_error = error;
    }
    int turn = error * VISION_KP_NUM / VISION_KP_DEN +
               derivative * VISION_KD_NUM / VISION_KD_DEN +
               vision->heading_error * VISION_HEADING_GAIN / 1000;
    turn = clamp_int(turn, -VISION_TURN_LIMIT, VISION_TURN_LIMIT);
    if (turn != 0) {
        controller->last_turn_direction = turn > 0 ? 1 : -1;
    }

    const bool corner = vision->corner_detected;
    controller->state = corner ? VISION_FOLLOW_CORNER : VISION_FOLLOW_TRACKING;
    int forward = corner ? VISION_CORNER_FORWARD :
        VISION_BASE_FORWARD - abs(error) * VISION_ERROR_SLOWDOWN / 1000;
    forward = clamp_int(forward, VISION_MIN_FORWARD, VISION_BASE_FORWARD);
    return (vision_follow_output_t) {
        .state = controller->state,
        .forward = forward,
        .turn = turn,
        .error = error,
        .state_changed = old_state != controller->state,
    };
}

const char *vision_follow_state_name(vision_follow_state_t state)
{
    switch (state) {
    case VISION_FOLLOW_WAITING_CAMERA:
        return "WAIT_CAMERA";
    case VISION_FOLLOW_TRACKING:
        return "TRACKING";
    case VISION_FOLLOW_CORNER:
        return "CORNER";
    case VISION_FOLLOW_LOST:
        return "LOST";
    case VISION_FOLLOW_FINISHED:
        return "FINISHED";
    case VISION_FOLLOW_FAULT_STOP:
        return "FAULT_STOP";
    default:
        return "UNKNOWN";
    }
}
