#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "ball_vision.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Shot sequence used after a confirmed END marker.
 *
 * ADVANCE is the only encoder-scripted stage left: it covers the calibrated
 * distance from the END bar to a pose where the balls and the pockets are in
 * view. Everything after it is a visual servo on the decoded ball frame:
 *
 *   ACQUIRE       stand still until the target ball and any pocket are seen
 *   ALIGN_STRAFE  translate until ball and pocket share an image column,
 *                 i.e. car, ball and pocket are collinear
 *   ALIGN_TURN    rotate until that column is the image centre line, i.e. the
 *                 heading points along the shot line; rotation has no speed
 *                 loop, so it runs in encoder-delimited micro-steps
 *                 (turn, stop, re-measure on the next frame)
 *   PUSH          drive straight through the ball until it drops out of view
 *                 (scored) or the encoder budget runs out
 *   RETREAT       back off the pocket, then run the cycle for the next ball
 *
 * Pocket identity is irrelevant: the servo shoots at whichever shape-valid
 * pocket is nearest the target ball, so no left/right hole is ever chosen. */
typedef enum {
    BALL_PUSH_SETTLE = 0,
    BALL_PUSH_ADVANCE,
    BALL_PUSH_ACQUIRE,
    BALL_PUSH_ALIGN_STRAFE,
    BALL_PUSH_ALIGN_TURN,
    BALL_PUSH_PUSH,
    BALL_PUSH_RETREAT,
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
    /* Visual servo bookkeeping. The camera publishes a few frames per second
     * while the control loop runs every CONTROL_PERIOD_MS, so each decoded
     * frame is consumed exactly once and its command is held in between. */
    uint32_t last_frame_sequence;
    bool have_frame;
    unsigned seen_frames;     /* consecutive frames with ball + pocket */
    unsigned aligned_frames;  /* consecutive frames inside both tolerances */
    unsigned lost_frames;     /* consecutive frames without the ball (push) */
    uint32_t vision_lost_ms;  /* how long this stage has lost its detection */
    unsigned align_rounds;    /* strafe rounds already used for this ball */
    int lateral_command;      /* held between frames */
    int turn_command;         /* held between frames */
    /* One ALIGN_TURN micro-step: turn this many side-wheel counts, measured
     * from these baselines, then hold still until the next frame. */
    int64_t step_counts;
    int step_left_count;
    int step_right_count;
    /* Accumulated travel of the servo stages, in the same units the old
     * net-displacement budgets used. The servos correct in both directions, so
     * a stage budget and a stall check must count motion, not displacement:
     * displacement shrinks whenever a correction reverses and would be read as
     * a stalled wheel. */
    int64_t travel;
    int travel_left_count;
    int travel_right_count;
    int travel_rear_count;
    int ball_error;           /* last measured image errors, px, +car right */
    int hole_error;
} ball_push_controller_t;

typedef struct {
    ball_push_state_t state;
    ball_color_t target_color;
    int forward;
    int lateral;
    int turn;
    bool active;
    bool just_completed_ball;
    bool ball_scored;   /* the push ended because the ball left the frame */
    bool mission_complete;
    bool fault;
    bool state_changed;
    int ball_error;     /* px offsets from the image centre line, +car right */
    int hole_error;
} ball_push_result_t;

void ball_push_init(ball_push_controller_t *controller,
                    int left_count, int right_count, int rear_count);
/* `vision` is the latest decoded ball-mode frame; NULL is treated as "nothing
 * detected", which the acquisition stage times out on. */
ball_push_result_t ball_push_update(ball_push_controller_t *controller,
                                    const ball_vision_result_t *vision,
                                    uint32_t elapsed_ms,
                                    int left_count,
                                    int right_count,
                                    int rear_count);
const char *ball_push_state_name(ball_push_state_t state);

#ifdef __cplusplus
}
#endif
