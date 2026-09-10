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

static int64_t abs_i64(int64_t value)
{
    return value < 0 ? -value : value;
}

static int64_t search_target_counts(unsigned leg)
{
    return leg == 0U ? LINE_SEARCH_FIRST_COUNTS : LINE_SEARCH_SECOND_COUNTS;
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

/* Tracking error uses ONLY the two inner channels (CH2 = +1 right of centre,
 * CH3 = -1 left of centre); both lit = 0 (centred). The two outer channels are
 * deliberately excluded so a wide line or gentle curve cannot yank the error to
 * +/-3 and steer hard early. Outer channels are used solely to arm a corner in
 * outer_direction() below. */
static int mask_to_error(uint8_t mask)
{
    const bool right = (mask & IR_CHANNEL_2_MASK) != 0U;
    const bool left = (mask & IR_CHANNEL_3_MASK) != 0U;
    if (right == left) {
        return 0;
    }
    return right ? 1 : -1;
}

/* Which outer channel sees black: +1 = CH1 (car right), -1 = CH4 (car left),
 * 0 = neither. For a contiguous, non-all-black mask at most one outer channel
 * is lit, so the corner direction is unambiguous. */
static int outer_direction(uint8_t mask)
{
    if ((mask & IR_CHANNEL_1_MASK) != 0U) {
        return 1;
    }
    if ((mask & IR_CHANNEL_4_MASK) != 0U) {
        return -1;
    }
    return 0;
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

static void reset_search(line_follow_controller_t *controller)
{
    controller->search_phase = LINE_SEARCH_IDLE;
    controller->search_leg = 0;
    controller->search_direction = 0;
    controller->search_start_left_count = 0;
    controller->search_start_right_count = 0;
    controller->search_progress_counts = 0;
    controller->search_target_counts = 0;
    controller->search_last_motion_counts = 0;
    controller->search_settle_ms = 0;
    controller->search_stall_ms = 0;
}

static line_follow_result_t stopped_result(void)
{
    return (line_follow_result_t) {
        .state = LINE_FOLLOW_STOPPED,
        .forward = 0,
        .turn = 0,
    };
}

static line_follow_result_t controlled_result(line_follow_controller_t *controller,
                                              line_follow_state_t state,
                                              int error,
                                              int forward_limit)
{
    reset_search(controller);
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
        .forward = forward,
        .turn = turn,
    };
}

static line_follow_result_t search_result(line_follow_controller_t *controller,
                                          uint32_t elapsed_ms,
                                          int left_count,
                                          int right_count)
{
    /* Captured before any mutation below: the LOST_SEARCH entry test needs to
     * know whether the controller was already searching on the previous tick,
     * which decides between (re)starting leg 0 and continuing the current leg. */
    const line_follow_state_t old_state = controller->state;
    controller->last_cycle_tracking = false;
    reset_corner_detection(controller, true);
    if (!controller->has_last_error) {
        controller->state = LINE_FOLLOW_WAITING_LINE;
        controller->lost_ms = 0;
        reset_search(controller);
        return (line_follow_result_t) {
            .state = controller->state,
            .forward = 0,
            .turn = 0,
        };
    }
    controller->lost_ms = add_saturated(controller->lost_ms, elapsed_ms);
    if (controller->lost_ms >= LINE_LOST_STOP_MS) {
        controller->state = LINE_FOLLOW_STOPPED;
        reset_search(controller);
        return stopped_result();
    }

    controller->state = LINE_FOLLOW_LOST_SEARCH;
    if (old_state != LINE_FOLLOW_LOST_SEARCH ||
        controller->search_phase == LINE_SEARCH_IDLE) {
        controller->search_phase = LINE_SEARCH_ROTATE;
        controller->search_leg = 0;
        /* Lost-line scanning always checks the RIGHT side first, independent of
           the last tracking error, then flips to the left for the second leg.
           Positive turn is a right turn, so the first leg uses +1. */
        controller->search_direction = 1;
        controller->search_start_left_count = left_count;
        controller->search_start_right_count = right_count;
        controller->search_progress_counts = 0;
        controller->search_target_counts = search_target_counts(0);
        controller->search_last_motion_counts = 0;
        controller->search_settle_ms = 0;
        controller->search_stall_ms = 0;
    } else if (controller->search_phase == LINE_SEARCH_SETTLE) {
        controller->search_settle_ms =
            add_saturated(controller->search_settle_ms, elapsed_ms);
        if (controller->search_settle_ms < LINE_SEARCH_SETTLE_MS) {
            return (line_follow_result_t) {
                .state = controller->state,
                .forward = 0,
                .turn = 0,
            };
        }
        if (controller->search_leg + 1U >= LINE_SEARCH_MAX_LEGS) {
            controller->state = LINE_FOLLOW_STOPPED;
            reset_search(controller);
            return stopped_result();
        }
        ++controller->search_leg;
        controller->search_direction = -controller->search_direction;
        controller->search_phase = LINE_SEARCH_ROTATE;
        controller->search_start_left_count = left_count;
        controller->search_start_right_count = right_count;
        controller->search_progress_counts = 0;
        controller->search_target_counts =
            search_target_counts(controller->search_leg);
        controller->search_last_motion_counts = 0;
        controller->search_settle_ms = 0;
        controller->search_stall_ms = 0;
    } else {
        const int64_t left_delta =
            (int64_t)left_count - controller->search_start_left_count;
        const int64_t right_delta =
            (int64_t)right_count - controller->search_start_right_count;
        controller->search_progress_counts =
            abs_i64(left_delta) + abs_i64(right_delta);

        if (controller->search_progress_counts >= controller->search_target_counts) {
            controller->search_phase = LINE_SEARCH_SETTLE;
            controller->search_settle_ms = 0;
            controller->search_stall_ms = 0;
            return (line_follow_result_t) {
                .state = controller->state,
                .forward = 0,
                .turn = 0,
            };
        }

        if (controller->search_progress_counts -
            controller->search_last_motion_counts >= LINE_SEARCH_STALL_COUNTS) {
            controller->search_last_motion_counts = controller->search_progress_counts;
            controller->search_stall_ms = 0;
        } else {
            controller->search_stall_ms =
                add_saturated(controller->search_stall_ms, elapsed_ms);
            if (controller->search_stall_ms >= LINE_SEARCH_STALL_MS) {
                controller->state = LINE_FOLLOW_STOPPED;
                reset_search(controller);
                return stopped_result();
            }
        }
    }

    const int64_t remaining =
        controller->search_target_counts - controller->search_progress_counts;
    const int turn_magnitude =
        remaining <= LINE_SEARCH_30_DEG_COUNTS / 4 ?
        LINE_SEARCH_FINE_TURN : LINE_SEARCH_TURN;
    return (line_follow_result_t) {
        .state = controller->state,
        .forward = LINE_SEARCH_FORWARD,
        .turn = controller->search_direction * turn_magnitude,
    };
}

static line_follow_result_t rotate_result(line_follow_controller_t *controller)
{
    return (line_follow_result_t) {
        .state = LINE_FOLLOW_CORNER_ROTATE,
        .forward = LINE_CORNER_ROTATE_FORWARD,
        .turn = controller->corner_direction * LINE_CORNER_ROTATE_TURN,
    };
}

static line_follow_result_t update_corner_rotate(line_follow_controller_t *controller,
                                                 uint8_t black_mask,
                                                 uint32_t elapsed_ms,
                                                 int left_count,
                                                 int right_count)
{
    controller->corner_rotate_ms = add_saturated(controller->corner_rotate_ms, elapsed_ms);
    controller->lost_ms = add_saturated(controller->lost_ms, elapsed_ms);
    controller->last_cycle_tracking = false;

    if (black_mask != 0U && mask_is_contiguous(black_mask)) {
        /* mask_to_error now ignores the outer channels, so require an inner
           channel to actually see the line before counting it as centred;
           otherwise an outer-only hit during the spin would read error 0 and
           finish the corner prematurely. */
        const uint8_t mid = black_mask & (IR_CHANNEL_2_MASK | IR_CHANNEL_3_MASK);
        const int error = mask_to_error(black_mask);
        if (mid != 0U && abs(error) <= 1) {
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
                                     LINE_MIN_FORWARD);
        }
    } else {
        controller->corner_centered_ms = 0;
    }

    if (controller->corner_rotate_ms >= LINE_CORNER_MAX_ROTATE_MS) {
        controller->last_direction = controller->corner_direction;
        return search_result(controller, 0, left_count, right_count);
    }
    return rotate_result(controller);
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
                                        uint32_t elapsed_ms,
                                        int left_count,
                                        int right_count)
{
    if (controller == NULL || controller->state == LINE_FOLLOW_STOPPED) {
        return stopped_result();
    }

    black_mask &= IR_ALL_BLACK_MASK;
    if (black_mask == IR_ALL_BLACK_MASK) {
        controller->all_black_ms = add_saturated(controller->all_black_ms, elapsed_ms);
        if (controller->all_black_ms >= LINE_ALL_BLACK_STOP_MS) {
            controller->state = LINE_FOLLOW_STOPPED;
            reset_search(controller);
            return stopped_result();
        }
        controller->last_cycle_tracking = false;
        return (line_follow_result_t) {
            .state = controller->state,
            .forward = 0,
            .turn = 0,
        };
    }
    controller->all_black_ms = 0;

    if (controller->state == LINE_FOLLOW_CORNER_ROTATE) {
        return update_corner_rotate(controller, black_mask, elapsed_ms,
                                    left_count, right_count);
    }

    if (controller->state == LINE_FOLLOW_CORNER_CANDIDATE && black_mask == 0U) {
        if (add_saturated(controller->corner_candidate_ms, elapsed_ms) >
            LINE_CORNER_CONFIRM_WINDOW_MS) {
            controller->state = LINE_FOLLOW_TRACKING;
            reset_corner_detection(controller, true);
            return search_result(controller, elapsed_ms, left_count, right_count);
        }
        controller->state = LINE_FOLLOW_CORNER_ROTATE;
        controller->corner_rotate_ms = elapsed_ms;
        controller->corner_centered_ms = 0;
        controller->lost_ms = elapsed_ms;
        controller->invalid_ms = 0;
        controller->last_cycle_tracking = false;
        controller->corner_rearm_ready = false;
        return rotate_result(controller);
    }

    if (black_mask == 0U) {
        controller->invalid_ms = 0;
        return search_result(controller, elapsed_ms, left_count, right_count);
    }

    if (!mask_is_contiguous(black_mask)) {
        if (controller->state == LINE_FOLLOW_LOST_SEARCH) {
            controller->invalid_ms =
                add_saturated(controller->invalid_ms, elapsed_ms);
            return search_result(controller, elapsed_ms, left_count, right_count);
        }
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
            return search_result(controller, elapsed_ms, left_count, right_count);
        }
        controller->last_cycle_tracking = false;
        const int held_error = controller->has_last_error ? controller->last_error : 0;
        return (line_follow_result_t) {
            .state = controller->state,
            .forward = controller->has_last_error ? LINE_MIN_FORWARD : 0,
            .turn = clamp_int(LINE_KP * held_error, -LINE_TURN_LIMIT, LINE_TURN_LIMIT),
        };
    }

    /* Error comes only from the two inner channels; the outer channels arm a
       corner instead of steering. outer_dir: +1 CH1 (right), -1 CH4 (left). */
    const int error = mask_to_error(black_mask);
    const int outer_dir = outer_direction(black_mask);

    if (controller->state == LINE_FOLLOW_CORNER_CANDIDATE) {
        /* Still a corner only while the armed outer channel stays black and the
           confirm window has not elapsed; otherwise it was a gentle curve. */
        if (outer_dir == controller->corner_direction) {
            controller->corner_candidate_ms =
                add_saturated(controller->corner_candidate_ms, elapsed_ms);
            if (controller->corner_candidate_ms <= LINE_CORNER_CONFIRM_WINDOW_MS) {
                return controlled_result(controller, LINE_FOLLOW_CORNER_CANDIDATE,
                                         error, LINE_CORNER_APPROACH_FORWARD);
            }
        }
        controller->state = LINE_FOLLOW_TRACKING;
        reset_corner_detection(controller, true);
    }

    if (controller->state == LINE_FOLLOW_CORNER_EXIT) {
        controller->corner_exit_ms = add_saturated(controller->corner_exit_ms, elapsed_ms);
        line_follow_state_t next_state = LINE_FOLLOW_CORNER_EXIT;
        if (controller->corner_exit_ms >= LINE_CORNER_EXIT_MS) {
            controller->corner_exit_ms = 0;
            next_state = LINE_FOLLOW_TRACKING;
        }
        return controlled_result(controller, next_state, error,
                                 LINE_MIN_FORWARD);
    }

    if (outer_dir != 0) {
        /* An outer channel sees black: arm a corner when we were recently
           centred; otherwise ignore it and keep tracking on the inner two. */
        controller->corner_centered_ms = 0;
        if (controller->corner_rearm_ready) {
            if (controller->corner_direction == outer_dir) {
                controller->corner_arm_ms =
                    add_saturated(controller->corner_arm_ms, elapsed_ms);
            } else {
                controller->corner_direction = outer_dir;
                controller->corner_arm_ms = elapsed_ms;
            }
            if (controller->corner_arm_ms >= LINE_CORNER_ARM_MS) {
                controller->corner_candidate_ms = 0;
                return controlled_result(controller, LINE_FOLLOW_CORNER_CANDIDATE,
                                         error, LINE_CORNER_APPROACH_FORWARD);
            }
        } else {
            controller->corner_arm_ms = 0;
            controller->corner_direction = 0;
        }
    } else {
        controller->corner_arm_ms = 0;
        controller->corner_direction = 0;
        if (error == 0 && !controller->corner_rearm_ready) {
            controller->corner_centered_ms =
                add_saturated(controller->corner_centered_ms, elapsed_ms);
            if (controller->corner_centered_ms >= LINE_CORNER_CENTERED_MS) {
                controller->corner_rearm_ready = true;
                controller->corner_centered_ms = LINE_CORNER_CENTERED_MS;
            }
        } else if (error != 0) {
            controller->corner_centered_ms = 0;
        }
    }

    return controlled_result(controller, LINE_FOLLOW_TRACKING, error,
                             LINE_BASE_FORWARD);
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
