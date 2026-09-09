#include "ball_push.h"

#include <limits.h>

#include "board_config.h"

static uint32_t add_saturated(uint32_t value, uint32_t increment)
{
    return UINT32_MAX - value < increment ? UINT32_MAX : value + increment;
}

static int abs_int(int value)
{
    return value < 0 ? -value : value;
}

static int clamp_int(int value, int minimum, int maximum)
{
    if (value < minimum) {
        return minimum;
    }
    return value > maximum ? maximum : value;
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

/* Progress of the ALIGN_TURN micro-step currently in flight, as opposed to the
 * whole-stage progress measured from the stage baseline. */
static int64_t step_progress(const ball_push_controller_t *c,
                             int left, int right)
{
    return minimum_i64(abs_delta(left, c->step_left_count),
                       abs_delta(right, c->step_right_count));
}

/* Straight and turn stages need both driven side wheels to arrive. Using the
 * slower wheel prevents one free-spinning wheel from ending a stage early. */
static int64_t side_wheel_progress(const ball_push_controller_t *c,
                                   int left, int right)
{
    return minimum_i64(abs_delta(left, c->start_left_count),
                       abs_delta(right, c->start_right_count));
}

/* Accumulated travel of a servo stage: monotonic in both motion directions and
 * scaled to the same unit the encoder budgets use. Averaging the normalized
 * per-wheel deltas (in the lateral mixer the rear target is twice either side
 * target, so a side delta counts double) keeps one wheel's encoder
 * quantization inside a 10 ms tick from zeroing the whole measurement. The
 * numerator is accumulated and divided only on read, so no precision is lost.
 * `lateral` selects the three-wheel strafe scale over the two-wheel rotation
 * one; a stage never mixes the two because travel restarts on entry. */
static int64_t accumulate_travel(ball_push_controller_t *c, bool lateral,
                                 int left, int right, int rear)
{
    const int64_t side = abs_delta(left, c->travel_left_count) +
                         abs_delta(right, c->travel_right_count);
    c->travel += lateral ?
        side * 2 + abs_delta(rear, c->travel_rear_count) : side;
    c->travel_left_count = left;
    c->travel_right_count = right;
    c->travel_rear_count = rear;
    return lateral ? c->travel / 3 : c->travel / 2;
}

/* ------------------------------------------------------------------ */
/* Ball-frame interpretation                                           */
/* ------------------------------------------------------------------ */

typedef struct {
    bool ball_found;
    bool seen;         /* target ball and a pocket in the same frame */
    int ball_error;    /* px from the image centre line, +car right */
    int hole_error;
    int column_error;  /* ball_error - hole_error: driven by translation */
    int mean_error;    /* average of both: driven by rotation */
} ball_shot_t;

static const ball_vision_object_t *shot_ball(const ball_vision_result_t *vision,
                                             ball_color_t color)
{
    return color == BALL_COLOR_RED ? &vision->red_ball : &vision->white_ball;
}

/* The shot does not care which pocket it is: pick any shape-valid one, and
 * when both are visible take the one nearest the ball, which needs the least
 * translation to become collinear. A lone pocket carries no left/right
 * identity in the analyzer, and none is needed here. */
static const ball_vision_object_t *shot_hole(const ball_vision_result_t *vision,
                                             const ball_vision_object_t *ball)
{
    if (vision->left_hole.found && vision->right_hole.found) {
        const int to_left =
            abs_int((int)vision->left_hole.center_x - (int)ball->center_x);
        const int to_right =
            abs_int((int)vision->right_hole.center_x - (int)ball->center_x);
        return to_left <= to_right ? &vision->left_hole : &vision->right_hole;
    }
    if (vision->left_hole.found) return &vision->left_hole;
    if (vision->right_hole.found) return &vision->right_hole;
    return vision->lone_hole.found ? &vision->lone_hole : NULL;
}

static ball_shot_t evaluate_shot(const ball_vision_result_t *vision,
                                 ball_color_t color)
{
    ball_shot_t shot = {0};
    if (vision == NULL || vision->image_width == 0U) return shot;
    const ball_vision_object_t *ball = shot_ball(vision, color);
    if (!ball->found) return shot;
    shot.ball_found = true;
    const ball_vision_object_t *hole = shot_hole(vision, ball);
    if (hole == NULL) return shot;
    const int centre = (int)(vision->image_width / 2U);
    shot.seen = true;
    shot.ball_error = (int)ball->center_x - centre;
    shot.hole_error = (int)hole->center_x - centre;
    shot.column_error = shot.ball_error - shot.hole_error;
    shot.mean_error = (shot.ball_error + shot.hole_error) / 2;
    return shot;
}

static bool within_percent(int value, unsigned width, unsigned percent)
{
    return (unsigned)(abs_int(value) * 100) <= width * percent;
}

/* Consume each decoded frame exactly once. image_width stays zero until the
 * ball analyzer has produced a frame in this mode, so the result cleared by a
 * mode switch can never be mistaken for a fresh detection. */
static bool frame_is_new(ball_push_controller_t *c,
                         const ball_vision_result_t *vision)
{
    if (vision == NULL || vision->image_width == 0U) return false;
    if (c->have_frame && vision->frame_sequence == c->last_frame_sequence) {
        return false;
    }
    c->have_frame = true;
    c->last_frame_sequence = vision->frame_sequence;
    return true;
}

/* Proportional output with a floor: a small residual error must still produce
 * a command the wheels act on, otherwise the stage looks stalled while it is
 * in fact converged to just outside the tolerance. */
static int servo_command(int error, int kp_num, int kp_den,
                         int minimum, int maximum)
{
    if (error == 0) {
        return 0;
    }
    const int magnitude =
        clamp_int(abs_int(error) * kp_num / kp_den, minimum, maximum);
    return error < 0 ? -magnitude : magnitude;
}

/* ------------------------------------------------------------------ */
/* Stage machinery                                                     */
/* ------------------------------------------------------------------ */

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
    c->have_frame = false;
    c->seen_frames = 0U;
    c->aligned_frames = 0U;
    c->lost_frames = 0U;
    c->vision_lost_ms = 0U;
    c->lateral_command = 0;
    c->turn_command = 0;
    c->step_counts = 0;
    c->step_left_count = left;
    c->step_right_count = right;
    c->travel = 0;
    c->travel_left_count = left;
    c->travel_right_count = right;
    c->travel_rear_count = rear;
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

/* Watchdog for the two servo stages. A stage that is waiting for its target or
 * sitting inside the tolerance deadband makes no encoder progress on purpose,
 * so the stall bookkeeping is reset instead of counting down to a fault. */
static bool servo_watchdog(ball_push_controller_t *c, bool seen, bool moving,
                           int64_t progress, uint32_t elapsed_ms)
{
    if (!seen) {
        c->vision_lost_ms = add_saturated(c->vision_lost_ms, elapsed_ms);
        c->last_progress = progress;
        c->stall_ms = 0U;
        return c->vision_lost_ms < BALL_ALIGN_LOST_TOLERANCE_MS;
    }
    c->vision_lost_ms = 0U;
    if (!moving) {
        c->last_progress = progress;
        c->stall_ms = 0U;
    } else if (movement_stalled(c, progress, elapsed_ms)) {
        return false;
    }
    return c->state_ms < BALL_ALIGN_TIMEOUT_MS;
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
        .ball_error = c->ball_error,
        .hole_error = c->hole_error,
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

/* Encoder-delimited straight stage: END -> shooting position. */
static ball_push_result_t advance_stage(ball_push_controller_t *c,
                                        ball_push_state_t old_state,
                                        uint32_t elapsed_ms,
                                        int left, int right, int rear)
{
    const int64_t moved = side_wheel_progress(c, left, right);
    if (moved >= (int64_t)BALL_SCRIPT_ADVANCE_COUNTS) {
        begin_settle(c, BALL_PUSH_ACQUIRE, left, right, rear);
        return make_result(c, old_state);
    }
    if (c->state_ms >= BALL_SCRIPT_STATE_TIMEOUT_MS ||
        movement_stalled(c, moved, elapsed_ms)) {
        return fault(c, old_state);
    }
    ball_push_result_t result = make_result(c, old_state);
    result.forward = BALL_SCRIPT_ADVANCE_FORWARD;
    result.lateral = BALL_SCRIPT_ADVANCE_LATERAL;
    return result;
}

/* Stands still until the target ball and a pocket are in the same frame. */
static ball_push_result_t acquire_stage(ball_push_controller_t *c,
                                        ball_push_state_t old_state,
                                        const ball_vision_result_t *vision,
                                        int left, int right, int rear)
{
    const ball_shot_t shot = evaluate_shot(vision, c->target_color);
    if (shot.seen) {
        c->ball_error = shot.ball_error;
        c->hole_error = shot.hole_error;
    }
    if (frame_is_new(c, vision)) {
        c->seen_frames = shot.seen ? c->seen_frames + 1U : 0U;
        if (c->seen_frames >= BALL_ACQUIRE_CONFIRM_FRAMES) {
            c->align_rounds = 0U;
            begin_settle(c, BALL_PUSH_ALIGN_STRAFE, left, right, rear);
            return make_result(c, old_state);
        }
    }
    if (c->state_ms >= BALL_ACQUIRE_TIMEOUT_MS) {
        return fault(c, old_state);
    }
    return make_result(c, old_state);
}

/* Translation servo: makes car, ball and pocket collinear. Rotation shifts the
 * ball and the pocket by the same image amount, so it can never change their
 * column difference; only translation can, because the nearer ball parallaxes
 * more than the farther pocket. */
static ball_push_result_t strafe_stage(ball_push_controller_t *c,
                                       ball_push_state_t old_state,
                                       const ball_vision_result_t *vision,
                                       uint32_t elapsed_ms,
                                       int left, int right, int rear)
{
    const ball_shot_t shot = evaluate_shot(vision, c->target_color);
    if (shot.seen) {
        c->ball_error = shot.ball_error;
        c->hole_error = shot.hole_error;
    }
    const int64_t moved = accumulate_travel(c, true, left, right, rear);

    if (frame_is_new(c, vision)) {
        const bool collinear = shot.seen &&
            within_percent(shot.column_error, vision->image_width,
                           BALL_ALIGN_COL_TOL_PERCENT);
        c->aligned_frames = collinear ? c->aligned_frames + 1U : 0U;
        /* Positive lateral moves the car left, which shifts the image right and
         * shifts the ball more than the pocket: the column difference follows
         * the lateral sign, so the error is negated. */
        c->lateral_command = collinear ? 0 :
            servo_command(-shot.column_error, BALL_ALIGN_STRAFE_KP_NUM,
                          BALL_ALIGN_STRAFE_KP_DEN, BALL_ALIGN_STRAFE_MIN,
                          BALL_ALIGN_STRAFE_MAX);
        if (c->aligned_frames >= BALL_ALIGN_CONFIRM_FRAMES) {
            begin_settle(c, BALL_PUSH_ALIGN_TURN, left, right, rear);
            return make_result(c, old_state);
        }
    }

    if (!servo_watchdog(c, shot.seen, c->lateral_command != 0, moved,
                        elapsed_ms) ||
        moved >= (int64_t)BALL_ALIGN_MAX_STRAFE_COUNTS) {
        return fault(c, old_state);
    }
    ball_push_result_t result = make_result(c, old_state);
    result.lateral = shot.seen ? c->lateral_command : 0;
    return result;
}

/* Rotation servo: turns the heading onto the collinear shot line.
 *
 * The drive layer closes the per-wheel speed loop only while a lateral
 * component is requested, so a rotation runs at raw PWM: holding one turn
 * output for a whole ball-mode frame would sweep many degrees past the
 * tolerance. The stage therefore moves in encoder-delimited micro-steps -
 * turn, stop, then re-measure on the next frame - which quantizes the
 * correction by encoder counts instead of by the frame interval. */
static ball_push_result_t turn_stage(ball_push_controller_t *c,
                                     ball_push_state_t old_state,
                                     const ball_vision_result_t *vision,
                                     uint32_t elapsed_ms,
                                     int left, int right, int rear)
{
    const ball_shot_t shot = evaluate_shot(vision, c->target_color);
    if (shot.seen) {
        c->ball_error = shot.ball_error;
        c->hole_error = shot.hole_error;
    }
    const int64_t moved = accumulate_travel(c, false, left, right, rear);

    if (frame_is_new(c, vision)) {
        const bool collinear = shot.seen &&
            within_percent(shot.column_error, vision->image_width,
                           BALL_ALIGN_COL_TOL_PERCENT);
        const bool on_axis = shot.seen &&
            within_percent(shot.mean_error, vision->image_width,
                           BALL_ALIGN_CENTER_TOL_PERCENT);
        if (collinear && on_axis) {
            c->turn_command = 0;
            c->aligned_frames += 1U;
            if (c->aligned_frames >= BALL_ALIGN_CONFIRM_FRAMES) {
                begin_settle(c, BALL_PUSH_PUSH, left, right, rear);
                return make_result(c, old_state);
            }
        } else if (on_axis) {
            /* Heading is on the shot line but the car is off it: rotation can
             * never change the column difference, so hand the error back to
             * translation instead of spinning uselessly. */
            c->turn_command = 0;
            if (c->align_rounds + 1U >= BALL_ALIGN_MAX_ROUNDS) {
                return fault(c, old_state);
            }
            c->align_rounds += 1U;
            begin_settle(c, BALL_PUSH_ALIGN_STRAFE, left, right, rear);
            return make_result(c, old_state);
        } else if (shot.seen) {
            c->aligned_frames = 0U;
            /* Positive turn rotates the car right, which shifts near and far
             * objects left by the same image amount: it drives the mean. */
            c->turn_command = shot.mean_error < 0 ?
                -BALL_ALIGN_TURN_SPEED : BALL_ALIGN_TURN_SPEED;
            c->step_counts = clamp_int(
                abs_int(shot.mean_error) * BALL_ALIGN_TURN_COUNTS_PER_PX_NUM /
                    BALL_ALIGN_TURN_COUNTS_PER_PX_DEN,
                BALL_ALIGN_TURN_MIN_STEP, BALL_ALIGN_TURN_MAX_STEP);
            c->step_left_count = left;
            c->step_right_count = right;
        } else {
            c->aligned_frames = 0U;
            c->turn_command = 0;
        }
    }

    /* End the micro-step as soon as it is carried out and wait, standing still,
     * for the next frame to measure its effect. */
    if (c->turn_command != 0 &&
        step_progress(c, left, right) >= c->step_counts) {
        c->turn_command = 0;
    }

    if (!servo_watchdog(c, shot.seen, c->turn_command != 0, moved,
                        elapsed_ms)) {
        return fault(c, old_state);
    }
    if (moved >= (int64_t)BALL_ALIGN_MAX_TURN_COUNTS) {
        if (c->align_rounds + 1U < BALL_ALIGN_MAX_ROUNDS) {
            c->align_rounds += 1U;
            begin_settle(c, BALL_PUSH_ALIGN_STRAFE, left, right, rear);
            return make_result(c, old_state);
        }
        return fault(c, old_state);
    }
    ball_push_result_t result = make_result(c, old_state);
    result.turn = shot.seen ? c->turn_command : 0;
    return result;
}

/* Straight push along the aligned shot line. */
static ball_push_result_t push_stage(ball_push_controller_t *c,
                                     ball_push_state_t old_state,
                                     const ball_vision_result_t *vision,
                                     uint32_t elapsed_ms,
                                     int left, int right, int rear)
{
    const ball_shot_t shot = evaluate_shot(vision, c->target_color);
    if (shot.seen) {
        c->ball_error = shot.ball_error;
        c->hole_error = shot.hole_error;
    }
    const int64_t moved = side_wheel_progress(c, left, right);
    if (frame_is_new(c, vision)) {
        c->lost_frames = shot.ball_found ? 0U : c->lost_frames + 1U;
    }
    /* The ball leaving the frame is the goal signature, but only after the car
     * has really travelled: a detection dropout at the start is not a score. */
    const bool scored = c->lost_frames >= BALL_PUSH_SCORE_LOST_FRAMES &&
                        moved >= (int64_t)BALL_PUSH_MIN_SCORE_COUNTS;
    if (scored || moved >= (int64_t)BALL_PUSH_MAX_COUNTS) {
        c->completed_balls = c->target_color == BALL_COLOR_RED ? 1U : 2U;
        begin_settle(c, BALL_PUSH_RETREAT, left, right, rear);
        ball_push_result_t result = make_result(c, old_state);
        result.just_completed_ball = true;
        result.ball_scored = scored;
        return result;
    }
    if (c->state_ms >= BALL_SCRIPT_STATE_TIMEOUT_MS ||
        movement_stalled(c, moved, elapsed_ms)) {
        return fault(c, old_state);
    }
    ball_push_result_t result = make_result(c, old_state);
    result.forward = BALL_SCRIPT_PUSH_FORWARD;
    return result;
}

static ball_push_result_t retreat_stage(ball_push_controller_t *c,
                                        ball_push_state_t old_state,
                                        uint32_t elapsed_ms,
                                        int left, int right, int rear)
{
    const int64_t moved = side_wheel_progress(c, left, right);
    if (moved >= (int64_t)BALL_SCRIPT_RETREAT_COUNTS) {
        if (c->target_color == BALL_COLOR_RED) {
            c->target_color = BALL_COLOR_WHITE;
            begin_settle(c, BALL_PUSH_ACQUIRE, left, right, rear);
            return make_result(c, old_state);
        }
        c->state = BALL_PUSH_COMPLETE;
        return make_result(c, old_state);
    }
    if (c->state_ms >= BALL_SCRIPT_STATE_TIMEOUT_MS ||
        movement_stalled(c, moved, elapsed_ms)) {
        return fault(c, old_state);
    }
    ball_push_result_t result = make_result(c, old_state);
    result.forward = BALL_SCRIPT_RETREAT_FORWARD;
    return result;
}

ball_push_result_t ball_push_update(ball_push_controller_t *c,
                                    const ball_vision_result_t *vision,
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
    if (c->state == BALL_PUSH_COMPLETE || c->state == BALL_PUSH_FAULT_STOP) {
        return make_result(c, old_state);
    }

    c->state_ms = add_saturated(c->state_ms, elapsed_ms);

    if (c->state == BALL_PUSH_SETTLE) {
        if (c->state_ms >= BALL_SCRIPT_SETTLE_MS) {
            begin_state(c, c->next_state, left, right, rear);
        }
        return make_result(c, old_state);
    }

    switch (c->state) {
    case BALL_PUSH_ADVANCE:
        return advance_stage(c, old_state, elapsed_ms, left, right, rear);
    case BALL_PUSH_ACQUIRE:
        return acquire_stage(c, old_state, vision, left, right, rear);
    case BALL_PUSH_ALIGN_STRAFE:
        return strafe_stage(c, old_state, vision, elapsed_ms, left, right, rear);
    case BALL_PUSH_ALIGN_TURN:
        return turn_stage(c, old_state, vision, elapsed_ms, left, right, rear);
    case BALL_PUSH_PUSH:
        return push_stage(c, old_state, vision, elapsed_ms, left, right, rear);
    case BALL_PUSH_RETREAT:
        return retreat_stage(c, old_state, elapsed_ms, left, right, rear);
    default:
        return fault(c, old_state);
    }
}

const char *ball_push_state_name(ball_push_state_t state)
{
    switch (state) {
    case BALL_PUSH_SETTLE: return "SETTLE";
    case BALL_PUSH_ADVANCE: return "ADVANCE";
    case BALL_PUSH_ACQUIRE: return "ACQUIRE";
    case BALL_PUSH_ALIGN_STRAFE: return "ALIGN_STRAFE";
    case BALL_PUSH_ALIGN_TURN: return "ALIGN_TURN";
    case BALL_PUSH_PUSH: return "PUSH";
    case BALL_PUSH_RETREAT: return "RETREAT";
    case BALL_PUSH_COMPLETE: return "COMPLETE";
    case BALL_PUSH_FAULT_STOP: return "FAULT_STOP";
    default: return "UNKNOWN";
    }
}
