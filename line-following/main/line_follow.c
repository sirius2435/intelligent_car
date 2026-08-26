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

static int error_direction(int error)
{
    return error < 0 ? -1 : (error > 0 ? 1 : 0);
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

static void reset_corner_detection(line_follow_controller_t *controller,
                                   bool require_recenter)
{
    controller->corner_arm_ms = 0;
    controller->corner_candidate_ms = 0;
    controller->corner_rotate_ms = 0;
    controller->corner_centered_ms = 0;
    controller->corner_exit_ms = 0;
    controller->corner_direction = 0;
    if (require_recenter) {
        controller->corner_rearm_ready = false;
    }
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

static line_follow_result_t controlled_result(line_follow_controller_t *controller,
                                              line_follow_state_t state,
                                              int error,
                                              int forward_limit,
                                              line_follow_state_t old_state)
{
    const int derivative = controller->last_cycle_tracking ?
                           error - controller->last_error : 0;
    const int turn = clamp_int(LINE_KP * error + LINE_KD * derivative,
                               -LINE_TURN_LIMIT, LINE_TURN_LIMIT);
    int forward = LINE_BASE_FORWARD - LINE_ERROR_SLOWDOWN * abs(error);
    forward = clamp_int(forward, LINE_MIN_FORWARD, LINE_BASE_FORWARD);
    if (forward > forward_limit) {
        forward = forward_limit;
    }

    controller->state = state;
    controller->last_error = error;
    controller->has_last_error = true;
    controller->lost_ms = 0;
    controller->invalid_ms = 0;
    controller->last_cycle_tracking = true;
    if (error != 0) {
        controller->last_direction = error_direction(error);
    }

    return (line_follow_result_t) {
        .state = state,
        .error = error,
        .forward = forward,
        .turn = turn,
        .state_changed = old_state != state,
    };
}

static line_follow_result_t search_result(line_follow_controller_t *controller,
                                          uint32_t elapsed_ms)
{
    const line_follow_state_t old_state = controller->state;
    controller->last_cycle_tracking = false;
    reset_corner_detection(controller, true);
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
    const int direction = controller->last_direction != 0 ? controller->last_direction : 1;
    return (line_follow_result_t) {
        .state = controller->state,
        .error = controller->last_error,
        .forward = LINE_SEARCH_FORWARD,
        .turn = direction * LINE_SEARCH_TURN,
        .state_changed = old_state != controller->state,
    };
}

static line_follow_result_t rotate_result(line_follow_controller_t *controller,
                                          line_follow_state_t old_state)
{
    return (line_follow_result_t) {
        .state = LINE_FOLLOW_CORNER_ROTATE,
        .error = controller->has_last_error ? controller->last_error : 0,
        .forward = LINE_CORNER_ROTATE_FORWARD,
        .turn = controller->corner_direction * LINE_CORNER_ROTATE_TURN,
        .state_changed = old_state != LINE_FOLLOW_CORNER_ROTATE,
    };
}

static line_follow_result_t update_corner_rotate(line_follow_controller_t *controller,
                                                 uint8_t black_mask,
                                                 uint32_t elapsed_ms)
{
    const line_follow_state_t old_state = controller->state;
    controller->corner_rotate_ms = add_saturated(controller->corner_rotate_ms, elapsed_ms);
    controller->lost_ms = add_saturated(controller->lost_ms, elapsed_ms);
    controller->last_cycle_tracking = false;

    if (black_mask != 0U && mask_is_contiguous(black_mask)) {
        const int error = mask_to_error(black_mask);
        if (abs(error) <= 1) {
            controller->corner_centered_ms =
                add_saturated(controller->corner_centered_ms, elapsed_ms);
        } else {
            controller->corner_centered_ms = 0;
        }

        if (controller->corner_rotate_ms >= LINE_CORNER_MIN_ROTATE_MS &&
            controller->corner_centered_ms >= LINE_CORNER_CENTERED_MS) {
            const int completed_direction = controller->corner_direction;
            reset_corner_detection(controller, false);
            controller->corner_rearm_ready = true;
            controller->last_direction = completed_direction;
            controller->last_cycle_tracking = false;
            return controlled_result(controller, LINE_FOLLOW_CORNER_EXIT, error,
                                     LINE_MIN_FORWARD, old_state);
        }
    } else {
        controller->corner_centered_ms = 0;
    }

    if (controller->corner_rotate_ms >= LINE_CORNER_MAX_ROTATE_MS) {
        controller->last_direction = controller->corner_direction;
        return search_result(controller, 0);
    }
    return rotate_result(controller, old_state);
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
        controller->last_cycle_tracking = false;
        return (line_follow_result_t) {
            .state = controller->state,
            .error = controller->has_last_error ? controller->last_error : 0,
            .forward = 0,
            .turn = 0,
            .state_changed = false,
        };
    }
    controller->all_black_ms = 0;

    if (controller->state == LINE_FOLLOW_CORNER_ROTATE) {
        return update_corner_rotate(controller, black_mask, elapsed_ms);
    }

    if (controller->state == LINE_FOLLOW_CORNER_CANDIDATE && black_mask == 0U) {
        const line_follow_state_t old_state = controller->state;
        if (add_saturated(controller->corner_candidate_ms, elapsed_ms) >
            LINE_CORNER_CONFIRM_WINDOW_MS) {
            controller->state = LINE_FOLLOW_TRACKING;
            reset_corner_detection(controller, true);
            return search_result(controller, elapsed_ms);
        }
        controller->state = LINE_FOLLOW_CORNER_ROTATE;
        controller->corner_rotate_ms = elapsed_ms;
        controller->corner_centered_ms = 0;
        controller->lost_ms = elapsed_ms;
        controller->invalid_ms = 0;
        controller->last_cycle_tracking = false;
        controller->corner_rearm_ready = false;
        return rotate_result(controller, old_state);
    }

    if (black_mask == 0U) {
        controller->invalid_ms = 0;
        return search_result(controller, elapsed_ms);
    }

    const line_follow_state_t old_state = controller->state;
    if (!mask_is_contiguous(black_mask)) {
        if (controller->state == LINE_FOLLOW_CORNER_CANDIDATE ||
            controller->state == LINE_FOLLOW_CORNER_EXIT) {
            controller->state = LINE_FOLLOW_TRACKING;
            reset_corner_detection(controller, true);
        } else {
            controller->corner_arm_ms = 0;
            controller->corner_direction = 0;
        }
        controller->invalid_ms = add_saturated(controller->invalid_ms, elapsed_ms);
        if (controller->invalid_ms >= LINE_INVALID_GRACE_MS) {
            return search_result(controller, elapsed_ms);
        }
        controller->last_cycle_tracking = false;
        const int held_error = controller->has_last_error ? controller->last_error : 0;
        return (line_follow_result_t) {
            .state = controller->state,
            .error = held_error,
            .forward = controller->has_last_error ? LINE_MIN_FORWARD : 0,
            .turn = clamp_int(LINE_KP * held_error, -LINE_TURN_LIMIT, LINE_TURN_LIMIT),
            .state_changed = old_state != controller->state,
        };
    }

    const int error = mask_to_error(black_mask);
    const int direction = error_direction(error);

    if (controller->state == LINE_FOLLOW_CORNER_CANDIDATE) {
        if (abs(error) <= 1 || direction != controller->corner_direction) {
            controller->state = LINE_FOLLOW_TRACKING;
            reset_corner_detection(controller, true);
        } else {
            controller->corner_candidate_ms =
                add_saturated(controller->corner_candidate_ms, elapsed_ms);
            if (controller->corner_candidate_ms <= LINE_CORNER_CONFIRM_WINDOW_MS) {
                return controlled_result(controller, LINE_FOLLOW_CORNER_CANDIDATE,
                                         error, LINE_CORNER_APPROACH_FORWARD, old_state);
            }
            controller->state = LINE_FOLLOW_TRACKING;
            reset_corner_detection(controller, true);
        }
    }

    if (controller->state == LINE_FOLLOW_CORNER_EXIT) {
        controller->corner_exit_ms = add_saturated(controller->corner_exit_ms, elapsed_ms);
        line_follow_state_t next_state = LINE_FOLLOW_CORNER_EXIT;
        if (controller->corner_exit_ms >= LINE_CORNER_EXIT_MS) {
            controller->corner_exit_ms = 0;
            next_state = LINE_FOLLOW_TRACKING;
        }
        return controlled_result(controller, next_state, error,
                                 LINE_MIN_FORWARD, old_state);
    }

    if (abs(error) <= 1) {
        controller->corner_arm_ms = 0;
        controller->corner_direction = 0;
        if (!controller->corner_rearm_ready) {
            controller->corner_centered_ms =
                add_saturated(controller->corner_centered_ms, elapsed_ms);
            if (controller->corner_centered_ms >= LINE_CORNER_CENTERED_MS) {
                controller->corner_rearm_ready = true;
                controller->corner_centered_ms = LINE_CORNER_CENTERED_MS;
            }
        }
    } else {
        controller->corner_centered_ms = 0;
        if (controller->corner_rearm_ready) {
            if (controller->corner_direction == direction) {
                controller->corner_arm_ms =
                    add_saturated(controller->corner_arm_ms, elapsed_ms);
            } else {
                controller->corner_direction = direction;
                controller->corner_arm_ms = elapsed_ms;
            }
            if (controller->corner_arm_ms >= LINE_CORNER_ARM_MS) {
                controller->corner_candidate_ms = 0;
                return controlled_result(controller, LINE_FOLLOW_CORNER_CANDIDATE,
                                         error, LINE_CORNER_APPROACH_FORWARD, old_state);
            }
        } else {
            controller->corner_arm_ms = 0;
            controller->corner_direction = 0;
        }
    }

    return controlled_result(controller, LINE_FOLLOW_TRACKING, error,
                             LINE_BASE_FORWARD, old_state);
}

const char *line_follow_state_name(line_follow_state_t state)
{
    switch (state) {
    case LINE_FOLLOW_WAITING_LINE:
        return "WAITING_LINE";
    case LINE_FOLLOW_TRACKING:
        return "TRACKING";
    case LINE_FOLLOW_CORNER_CANDIDATE:
        return "CORNER_CANDIDATE";
    case LINE_FOLLOW_CORNER_ROTATE:
        return "CORNER_ROTATE";
    case LINE_FOLLOW_CORNER_EXIT:
        return "CORNER_EXIT";
    case LINE_FOLLOW_LOST_SEARCH:
        return "LOST_SEARCH";
    case LINE_FOLLOW_STOPPED:
        return "STOPPED";
    default:
        return "UNKNOWN";
    }
}
