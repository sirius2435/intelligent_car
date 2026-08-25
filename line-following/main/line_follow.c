#include "line_follow.h"

#include <stdlib.h>

#include "board_config.h"
#include "infrared_sensor.h"

static int clamp_int(int value, int minimum, int maximum)
{
    if (value < minimum) {
        return minimum;
    }
    return value > maximum ? maximum : value;
}

static uint32_t add_saturated(uint32_t value, uint32_t increment)
{
    return UINT32_MAX - value < increment ? UINT32_MAX : value + increment;
}

static bool mask_is_contiguous(uint8_t mask)
{
    while ((mask & 1U) == 0U && mask != 0U) {
        mask >>= 1;
    }
    while ((mask & 1U) != 0U) {
        mask >>= 1;
    }
    return mask == 0U;
}

static int mask_to_error(uint8_t mask)
{
    static const int weights[4] = {3, 1, -1, -3};
    int sum = 0;
    int count = 0;
    for (unsigned bit = 0; bit < 4; ++bit) {
        if ((mask & (1U << bit)) != 0U) {
            sum += weights[bit];
            ++count;
        }
    }
    return count == 0 ? 0 : sum / count;
}

static line_follow_result_t stopped_result(bool changed)
{
    return (line_follow_result_t) {
        .state = LINE_FOLLOW_STOPPED,
        .error = 0,
        .forward = 0,
        .turn = 0,
        .state_changed = changed,
    };
}

static line_follow_result_t search_result(line_follow_controller_t *controller,
                                          uint32_t elapsed_ms)
{
    const line_follow_state_t old_state = controller->state;
    if (!controller->has_last_error) {
        controller->state = LINE_FOLLOW_WAITING_LINE;
        controller->lost_ms = 0;
        return (line_follow_result_t) {
            .state = controller->state,
            .error = 0,
            .forward = 0,
            .turn = 0,
            .state_changed = old_state != controller->state,
        };
    }
    controller->lost_ms = add_saturated(controller->lost_ms, elapsed_ms);
    if (controller->lost_ms >= LINE_LOST_STOP_MS) {
        controller->state = LINE_FOLLOW_STOPPED;
        return stopped_result(old_state != LINE_FOLLOW_STOPPED);
    }

    controller->state = LINE_FOLLOW_LOST_SEARCH;
    const int direction = controller->last_error < 0 ? -1 : 1;
    return (line_follow_result_t) {
        .state = controller->state,
        .error = controller->last_error,
        .forward = LINE_SEARCH_FORWARD,
        .turn = direction * LINE_SEARCH_TURN,
        .state_changed = old_state != controller->state,
    };
}

void line_follow_init(line_follow_controller_t *controller)
{
    if (controller == NULL) {
        return;
    }
    *controller = (line_follow_controller_t) {
        .state = LINE_FOLLOW_WAITING_LINE,
    };
}

line_follow_result_t line_follow_update(line_follow_controller_t *controller,
                                        uint8_t black_mask,
                                        uint32_t elapsed_ms)
{
    if (controller == NULL || controller->state == LINE_FOLLOW_STOPPED) {
        return stopped_result(false);
    }

    black_mask &= IR_ALL_BLACK_MASK;
    if (black_mask == IR_ALL_BLACK_MASK) {
        controller->all_black_ms = add_saturated(controller->all_black_ms, elapsed_ms);
        if (controller->all_black_ms >= LINE_ALL_BLACK_STOP_MS) {
            const bool changed = controller->state != LINE_FOLLOW_STOPPED;
            controller->state = LINE_FOLLOW_STOPPED;
            return stopped_result(changed);
        }
        /* Pause while confirming a possible stop marker/intersection. */
        return (line_follow_result_t) {
            .state = controller->state,
            .error = controller->has_last_error ? controller->last_error : 0,
            .forward = 0,
            .turn = 0,
            .state_changed = false,
        };
    }
    controller->all_black_ms = 0;

    if (black_mask == 0U) {
        controller->invalid_ms = 0;
        return search_result(controller, elapsed_ms);
    }

    if (!mask_is_contiguous(black_mask)) {
        controller->invalid_ms = add_saturated(controller->invalid_ms, elapsed_ms);
        if (controller->invalid_ms >= LINE_INVALID_GRACE_MS) {
            return search_result(controller, elapsed_ms);
        }
        const int held_error = controller->has_last_error ? controller->last_error : 0;
        return (line_follow_result_t) {
            .state = controller->state,
            .error = held_error,
            .forward = controller->has_last_error ? LINE_MIN_FORWARD : 0,
            .turn = clamp_int(LINE_KP * held_error, -LINE_TURN_LIMIT, LINE_TURN_LIMIT),
            .state_changed = false,
        };
    }

    const line_follow_state_t old_state = controller->state;
    const int error = mask_to_error(black_mask);
    const int derivative = controller->has_last_error ? error - controller->last_error : 0;
    const int turn = clamp_int(LINE_KP * error + LINE_KD * derivative,
                               -LINE_TURN_LIMIT, LINE_TURN_LIMIT);
    int forward = LINE_BASE_FORWARD - LINE_ERROR_SLOWDOWN * abs(error);
    forward = clamp_int(forward, LINE_MIN_FORWARD, LINE_BASE_FORWARD);

    controller->state = LINE_FOLLOW_TRACKING;
    controller->last_error = error;
    controller->has_last_error = true;
    controller->lost_ms = 0;
    controller->invalid_ms = 0;

    return (line_follow_result_t) {
        .state = controller->state,
        .error = error,
        .forward = forward,
        .turn = turn,
        .state_changed = old_state != controller->state,
    };
}

const char *line_follow_state_name(line_follow_state_t state)
{
    switch (state) {
    case LINE_FOLLOW_WAITING_LINE:
        return "WAITING_LINE";
    case LINE_FOLLOW_TRACKING:
        return "TRACKING";
    case LINE_FOLLOW_LOST_SEARCH:
        return "LOST_SEARCH";
    case LINE_FOLLOW_STOPPED:
        return "STOPPED";
    default:
        return "UNKNOWN";
    }
}
