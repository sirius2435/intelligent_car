#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "ball_vision.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Pocket-push controller: pushes the balls found by ball_vision into their
 * pockets after the line-following finish (see main/ball_push.c header for
 * the control principle).
 *
 * Pure state machine like line_follow.c / obstacle_avoidance.c: one tick per
 * control period, saturated counters, encoder-count stage supervision, no
 * sleeps, no hardware calls. main.c maps the resulting motion onto the drive
 * API selected by drive_mode. Host-testable together with ball_vision.c
 * (which provides the ball/pocket lookup helpers used here).
 */

typedef enum {
    BALL_PUSH_START_FORWARD = 0,    /* fixed short straight run from the start line */
    BALL_PUSH_FIND_BALL,             /* acquire the required ball after the start run */
    BALL_PUSH_APPROACH_BALL,          /* strafe-to-centre first, then straight approach */
    BALL_PUSH_CREEP_OFF_FINISH,       /* legacy alias/state kept for compatibility */
    BALL_PUSH_SCAN,                   /* rotating sweep for the required ball */
    BALL_PUSH_ALIGN,                /* collinearity regulators (pocket + ball) */
    BALL_PUSH_BACKOFF,              /* ball too close: back away, then ALIGN */
    BALL_PUSH_FAR_SPRINT,           /* open-loop charge from far field (vision-free) */
    BALL_PUSH_PUSH,                 /* straight push with small turn corrections */
    BALL_PUSH_BACKOUT,              /* reverse after a miss, then retry */
    BALL_PUSH_EGRESS,               /* reverse after a pocketed ball, next ball */
    BALL_PUSH_POST_EGRESS_TURN,     /* turn 50 deg right after egress, before forward move */
    BALL_PUSH_POST_EGRESS_FORWARD,  /* move forward after egress turn, before FIND_BALL */
    BALL_PUSH_DONE,                 /* both balls pocketed (sticky) */
    BALL_PUSH_FAULT_STOP,           /* retries/timeout/stall (sticky) */
} ball_push_state_t;

/* Tells main.c which drive_* API to call this tick. */
typedef enum {
    BALL_DRIVE_OPEN = 0,     /* drive_set_motion (pure commands, supervised) */
    BALL_DRIVE_LATERAL,      /* drive_set_motion_feedback (strafe PI) */
    BALL_DRIVE_APPROACH,     /* drive_set_approach_feedback (slow anti-stall) */
} ball_push_drive_mode_t;

typedef struct {
    ball_push_state_t state;
    int forward;
    int lateral;
    int turn;
    bool active;           /* controller wants the car to move */
    bool state_changed;
    unsigned attempt;      /* 0 = first ball, 1 = second */
    unsigned retry;        /* pushes already failed within this attempt */
    ball_push_drive_mode_t drive_mode;
} ball_push_result_t;

typedef struct {
    ball_push_state_t state;
    unsigned attempt;
    unsigned retry;
    uint32_t stage_ms;         /* ms inside the current state */
    uint32_t task_ms;          /* whole-task timeout accumulator */
    uint32_t last_vision_seq;  /* frame_seq of the last consumed fresh frame */
    bool ball_present_last;    /* required ball seen on the last fresh frame */

    /* Rotational / translational stage odometry (creep, scan, backoff,
     * backout, egress, push). stage_progress = |dL| + |dR| since begin.
     * stage_started guards the very first tick (CREEP is entered by init,
     * where no encoder snapshot exists yet). */
    bool stage_started;
    int64_t stage_start_l;
    int64_t stage_start_r;
    int64_t stage_progress;
    int64_t stage_target;      /* counts; negative = no count target */
    int64_t stage_last_motion;
    uint32_t stage_stall_ms;
    unsigned scan_leg;
    int scan_direction;        /* +1 right turn, -1 left turn */
    unsigned backoff_cycles;   /* ALIGN <-> BACKOFF bounce guard */

    /* Alignment convergence window. */
    uint32_t align_ok_ms;
    unsigned approach_phase;        /* 0=strafe-centre, 1=mandatory straight approach */
    uint32_t approach_forward_ms;   /* forward phase dwell before ALIGN is allowed */
    uint32_t approach_strafe_ms;    /* phase 0 strafe timeout accumulator */

    /* Per-frame success/loss geometry (only advanced on NEW fresh frames so
     * 10 ms ticks cannot multi-count one slow camera frame). */
    unsigned pocket_hit_frames;
    unsigned ball_lost_frames;
    int last_ball_cx;
    int last_ball_cy;
    int last_ball_radius;
    bool last_in_pocket;

    /* Push turn-sign latch with hysteresis. */
    int push_turn_sign;
    bool push_turn_latched;

    /* Far-sprint bookkeeping: locked at trigger time so the post-sprint
     * check can tell whether the ball was actually reached (it disappears
     * from view once the car is on top of it). */
    int far_sprint_ball_cx;
    int far_sprint_ball_cy;

    /* Boundary-pocket completion tracking.  The real pocket may be a black
     * opening at the table edge, so the ball can disappear immediately
     * after crossing it instead of remaining visible inside a rectangle. */
    bool last_ball_near_pocket;
    bool last_ball_approaching_pocket;
    bool red_pocketed;
    bool blue_pocketed;
} ball_push_controller_t;

typedef struct {
    ball_push_state_t state;
    unsigned attempt;
    unsigned retry;
    bool target_ball_visible;
    bool target_pocket_visible;
    bool red_pocketed;
    bool blue_pocketed;
} ball_push_status_t;

void ball_push_get_status(ball_push_status_t *status);

void ball_push_init(ball_push_controller_t *controller);

/* vision_fresh is computed by the caller: camera connected + result valid +
 * frame age <= BALL_VISION_FRESH_MAX_MS. Not-fresh ticks freeze the per-frame
 * geometry counters but still accrue stage/time watchdogs, so the machine
 * never spins on stale geometry. */
ball_push_result_t ball_push_update(ball_push_controller_t *controller,
                                    const ball_vision_result_t *vision,
                                    bool vision_fresh,
                                    uint32_t elapsed_ms,
                                    int left_count,
                                    int right_count,
                                    int rear_count);

const char *ball_push_state_name(ball_push_state_t state);
const char *ball_push_drive_mode_name(ball_push_drive_mode_t mode);

#ifdef __cplusplus
}
#endif
