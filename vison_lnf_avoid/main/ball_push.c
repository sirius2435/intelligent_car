#include "ball_push.h"

#include <limits.h>

#include "board_config.h"

static uint32_t add_saturated(uint32_t value, uint32_t increment)
{
    return UINT32_MAX - value < increment ? UINT32_MAX : value + increment;
}

static int64_t abs_delta(int current, int start)
{
    const int64_t delta = (int64_t)current - start;
    return delta < 0 ? -delta : delta;
}

static int64_t minimum_i64(int64_t a, int64_t b)
{
    return a < b ? a : b;
}

/* Straight and turn stages need both driven side wheels to arrive. Using the
 * slower wheel prevents one free-spinning wheel from ending a stage early. */
static int64_t side_wheel_progress(const ball_push_controller_t *c,
                                   int left, int right)
{
    return minimum_i64(abs_delta(left, c->start_left_count),
                       abs_delta(right, c->start_right_count));
}

/* In the lateral mixer the rear target is twice either side target. Normalize
 * the side counts before taking the slowest of all three wheels. */
static int64_t lateral_progress(const ball_push_controller_t *c,
                                int left, int right, int rear)
{
    const int64_t left_normalized =
        abs_delta(left, c->start_left_count) * 2;
    const int64_t right_normalized =
        abs_delta(right, c->start_right_count) * 2;
    const int64_t rear_delta = abs_delta(rear, c->start_rear_count);
    return minimum_i64(minimum_i64(left_normalized, right_normalized),
                       rear_delta);
}

static void begin_state(ball_push_controller_t *c, ball_push_state_t state,
                        int left, int right, int rear)
{
    c->state = state;
    c->state_ms = 0U;
    c->start_left_count = left;
    c->start_right_count = right;
    c->start_rear_count = rear;
    c->last_progress = 0;
    c->stall_ms = 0U;
}

static void begin_settle(ball_push_controller_t *c, ball_push_state_t next,
                         int left, int right, int rear)
{
    c->next_state = next;
    begin_state(c, BALL_PUSH_SETTLE, left, right, rear);
}

static bool movement_stalled(ball_push_controller_t *c, int64_t progress,
                             uint32_t elapsed_ms)
{
    if (progress - c->last_progress >= BALL_SCRIPT_STALL_MIN_COUNTS) {
        c->last_progress = progress;
        c->stall_ms = 0U;
    } else {
        c->stall_ms = add_saturated(c->stall_ms, elapsed_ms);
    }
    return c->stall_ms >= BALL_SCRIPT_STALL_TIMEOUT_MS;
}

static ball_push_result_t make_result(const ball_push_controller_t *c,
                                      ball_push_state_t old_state)
{
    return (ball_push_result_t) {
        .state = c->state,
        .target_color = c->target_color,
        .active = c->state != BALL_PUSH_COMPLETE &&
                  c->state != BALL_PUSH_FAULT_STOP,
        .mission_complete = c->state == BALL_PUSH_COMPLETE,
        .fault = c->state == BALL_PUSH_FAULT_STOP,
        .state_changed = c->state != old_state,
    };
}

static ball_push_result_t fault(ball_push_controller_t *c,
                                ball_push_state_t old_state)
{
    c->state = BALL_PUSH_FAULT_STOP;
    return make_result(c, old_state);
}

void ball_push_init(ball_push_controller_t *c, int left, int right, int rear)
{
    if (c == NULL) return;
    *c = (ball_push_controller_t) {
        .state = BALL_PUSH_SETTLE,
        .next_state = BALL_PUSH_ADVANCE,
        .target_color = BALL_COLOR_RED,
        .start_left_count = left,
        .start_right_count = right,
        .start_rear_count = rear,
    };
}

static int64_t target_for_state(ball_push_state_t state)
{
    switch (state) {
    case BALL_PUSH_ADVANCE: return BALL_SCRIPT_ADVANCE_COUNTS;
    case BALL_PUSH_ROTATE_RIGHT: return BALL_SCRIPT_ROTATE_RIGHT_COUNTS;
    case BALL_PUSH_ALIGN_RED: return BALL_SCRIPT_RED_ALIGN_COUNTS;
    case BALL_PUSH_PUSH_RED: return BALL_SCRIPT_RED_PUSH_COUNTS;
    case BALL_PUSH_RETREAT_RED: return BALL_SCRIPT_RED_PUSH_COUNTS;
    case BALL_PUSH_RETURN_CENTER: return BALL_SCRIPT_RED_ALIGN_COUNTS;
    case BALL_PUSH_ALIGN_WHITE: return BALL_SCRIPT_WHITE_ALIGN_COUNTS;
    case BALL_PUSH_PUSH_WHITE: return BALL_SCRIPT_WHITE_PUSH_COUNTS;
    default: return 0;
    }
}

static ball_push_state_t next_motion_state(ball_push_state_t state)
{
    switch (state) {
    case BALL_PUSH_ADVANCE: return BALL_PUSH_ROTATE_RIGHT;
    case BALL_PUSH_ROTATE_RIGHT: return BALL_PUSH_ALIGN_RED;
    case BALL_PUSH_ALIGN_RED: return BALL_PUSH_PUSH_RED;
    case BALL_PUSH_PUSH_RED: return BALL_PUSH_RETREAT_RED;
    case BALL_PUSH_RETREAT_RED: return BALL_PUSH_RETURN_CENTER;
    case BALL_PUSH_RETURN_CENTER: return BALL_PUSH_ALIGN_WHITE;
    case BALL_PUSH_ALIGN_WHITE: return BALL_PUSH_PUSH_WHITE;
    case BALL_PUSH_PUSH_WHITE: return BALL_PUSH_COMPLETE;
    default: return BALL_PUSH_FAULT_STOP;
    }
}

ball_push_result_t ball_push_update(ball_push_controller_t *c,
                                    uint32_t elapsed_ms, int left, int right,
                                    int rear)
{
    if (c == NULL) {
        return (ball_push_result_t) {
            .state = BALL_PUSH_FAULT_STOP,
            .fault = true,
        };
    }

    const ball_push_state_t old_state = c->state;
    if (c->state == BALL_PUSH_COMPLETE || c->state == BALL_PUSH_FAULT_STOP)
        return make_result(c, old_state);

    c->state_ms = add_saturated(c->state_ms, elapsed_ms);

    if (c->state == BALL_PUSH_SETTLE) {
        if (c->state_ms >= BALL_SCRIPT_SETTLE_MS) {
            begin_state(c, c->next_state, left, right, rear);
        }
        return make_result(c, old_state);
    }

    ball_push_result_t result = make_result(c, old_state);
    int64_t moved;
    if (c->state == BALL_PUSH_ALIGN_RED ||
        c->state == BALL_PUSH_RETURN_CENTER ||
        c->state == BALL_PUSH_ALIGN_WHITE) {
        moved = lateral_progress(c, left, right, rear);
    } else {
        moved = side_wheel_progress(c, left, right);
    }

    const int64_t target = target_for_state(c->state);
    if (target <= 0 || moved >= target) {
        const ball_push_state_t completed_state = c->state;
        const ball_push_state_t next = next_motion_state(completed_state);
        if (completed_state == BALL_PUSH_PUSH_RED) {
            c->completed_balls = 1U;
            result.just_completed_ball = true;
        } else if (completed_state == BALL_PUSH_RETURN_CENTER) {
            c->target_color = BALL_COLOR_WHITE;
        } else if (completed_state == BALL_PUSH_PUSH_WHITE) {
            c->completed_balls = 2U;
            c->state = BALL_PUSH_COMPLETE;
            result = make_result(c, old_state);
            result.just_completed_ball = true;
            return result;
        }
        begin_settle(c, next, left, right, rear);
        result = make_result(c, old_state);
        if (completed_state == BALL_PUSH_PUSH_RED)
            result.just_completed_ball = true;
        return result;
    }

    if (c->state_ms >= BALL_SCRIPT_STATE_TIMEOUT_MS ||
        movement_stalled(c, moved, elapsed_ms)) {
        return fault(c, old_state);
    }

    switch (c->state) {
    case BALL_PUSH_ADVANCE:
        result.forward = BALL_SCRIPT_ADVANCE_FORWARD;
        break;
    case BALL_PUSH_ROTATE_RIGHT:
        result.turn = BALL_SCRIPT_ROTATE_RIGHT_TURN;
        break;
    case BALL_PUSH_ALIGN_RED:
        result.lateral = BALL_SCRIPT_RED_ALIGN_LATERAL;
        break;
    case BALL_PUSH_PUSH_RED:
        result.forward = BALL_SCRIPT_PUSH_FORWARD;
        break;
    case BALL_PUSH_RETREAT_RED:
        result.forward = BALL_SCRIPT_RETREAT_FORWARD;
        break;
    case BALL_PUSH_RETURN_CENTER:
        result.lateral = -BALL_SCRIPT_RED_ALIGN_LATERAL;
        break;
    case BALL_PUSH_ALIGN_WHITE:
        result.lateral = BALL_SCRIPT_WHITE_ALIGN_LATERAL;
        break;
    case BALL_PUSH_PUSH_WHITE:
        result.forward = BALL_SCRIPT_PUSH_FORWARD;
        break;
    default:
        return fault(c, old_state);
    }
    return result;
}

const char *ball_push_state_name(ball_push_state_t state)
{
    switch (state) {
    case BALL_PUSH_SETTLE: return "SETTLE";
    case BALL_PUSH_ADVANCE: return "ADVANCE";
    case BALL_PUSH_ROTATE_RIGHT: return "ROTATE_RIGHT";
    case BALL_PUSH_ALIGN_RED: return "ALIGN_RED";
    case BALL_PUSH_PUSH_RED: return "PUSH_RED";
    case BALL_PUSH_RETREAT_RED: return "RETREAT_RED";
    case BALL_PUSH_RETURN_CENTER: return "RETURN_CENTER";
    case BALL_PUSH_ALIGN_WHITE: return "ALIGN_WHITE";
    case BALL_PUSH_PUSH_WHITE: return "PUSH_WHITE";
    case BALL_PUSH_COMPLETE: return "COMPLETE";
    case BALL_PUSH_FAULT_STOP: return "FAULT_STOP";
    default: return "UNKNOWN";
    }
}
