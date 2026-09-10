/* Host-side sequence test for main/ball_push.c.
 *
 * Mirrors the CURRENT state machine (see the ball_push.c header comment):
 *   START_FORWARD -> FIND_BALL -> APPROACH(phase 0 strafe / phase 1 far-field
 *   heading + optional FAR_SPRINT / phase 2 mandatory straight) -> ALIGN
 *   (strafe only) -> PUSH (straight, time-terminated) -> EGRESS ->
 *   POST_EGRESS_TURN -> POST_EGRESS_FORWARD -> FIND_BALL -> ... -> DONE
 *
 * Two firmware behaviours this file pins deliberately, because they are easy to
 * regress and are not obvious from the code:
 *  - a tick whose state_changed is set RE-RUNS the new state in the same call,
 *    so one ball_push_update() can traverse several states and the returned
 *    result already carries the newest state's motion command;
 *  - ALIGN never rotates and PUSH never steers, so both must report turn == 0.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ball_push.h"
#include "board_config.h"

#define CHECK(condition) do {                                                   \
    if (!(condition)) {                                                         \
        fprintf(stderr, "FAIL %s:%d: %s\n", __func__, __LINE__, #condition);    \
        exit(1);                                                                \
    }                                                                           \
} while (0)

#define TEST_WIDTH   120U
#define TEST_HEIGHT   80U
#define CENTER_X      60
#define TICK_MS       10U

/* ------------------------------------------------------------------ */
/* Vision-feed helpers: the controller only consumes geometry on NEW   */
/* fresh frames, so the test distinguishes feed_new (new frame_seq)    */
/* from feed_hold (same seq, still fresh) and feed_stale (not fresh).  */
/* ------------------------------------------------------------------ */

static unsigned s_seq;

static void blank_vision(ball_vision_result_t *v)
{
    memset(v, 0, sizeof(*v));
    v->valid = true;
    v->width = TEST_WIDTH;
    v->height = TEST_HEIGHT;
}

static void add_ball(ball_vision_result_t *v, ball_color_t color,
                     int cx, int cy, int radius)
{
    ball_blob_t *ball = &v->balls[color];
    ball->present = true;
    ball->cx = cx;
    ball->cy = cy;
    ball->radius = radius;
    if (!v->ball_count) {
        v->ball_count = 1;
    } else if (v->balls[BALL_COLOR_RED].present &&
               v->balls[BALL_COLOR_BLUE].present) {
        v->ball_count = 2;
    }
}

static void no_balls(ball_vision_result_t *v)
{
    v->balls[BALL_COLOR_RED].present = false;
    v->balls[BALL_COLOR_BLUE].present = false;
    v->ball_count = 0;
}

/* Puts the pocket of the given side in vision (0 = left, 1 = right) and hides
 * the other slot, matching what ball_vision.c publishes. */
static void add_pocket(ball_vision_result_t *v, unsigned side,
                       int cx, int cy, int x0, int y0, int x1, int y1)
{
    pocket_blob_t *slot = side == 0U ? &v->pockets[0] : &v->pockets[1];
    pocket_blob_t *other = side == 0U ? &v->pockets[1] : &v->pockets[0];
    other->visible = false;
    slot->visible = true;
    slot->cx = cx;
    slot->cy = cy;
    slot->area = (unsigned)((x1 - x0 + 1) * (y1 - y0 + 1));
    slot->x0 = x0;
    slot->y0 = y0;
    slot->x1 = x1;
    slot->y1 = y1;
    v->pocket_count = 1;
}

static ball_push_result_t feed_new(ball_push_controller_t *ctl,
                                   ball_vision_result_t *v,
                                   uint32_t ms, int l, int r)
{
    v->frame_seq = ++s_seq;
    return ball_push_update(ctl, v, true, ms, l, r, 0);
}

static ball_push_result_t feed_hold(ball_push_controller_t *ctl,
                                    ball_vision_result_t *v,
                                    uint32_t ms, int l, int r)
{
    return ball_push_update(ctl, v, true, ms, l, r, 0);
}

static ball_push_result_t feed_stale(ball_push_controller_t *ctl,
                                     ball_vision_result_t *v,
                                     uint32_t ms, int l, int r)
{
    return ball_push_update(ctl, v, false, ms, l, r, 0);
}

/* Advances (dl, dr) encoder counts per tick and keeps feeding the SAME frame
 * until the controller leaves `from`, or max_ticks is reached. Returns the
 * result of the transitioning tick; *ticks_used reports how many it took. */
static ball_push_result_t run_until_changed(ball_push_controller_t *ctl,
                                            ball_vision_result_t *v,
                                            ball_push_state_t from,
                                            int dl, int dr,
                                            int *l, int *r,
                                            unsigned max_ticks,
                                            unsigned *ticks_used)
{
    ball_push_result_t res;
    unsigned n = 0;
    memset(&res, 0, sizeof(res));
    while (ctl->state == from && n < max_ticks) {
        *l += dl;
        *r += dr;
        res = feed_hold(ctl, v, TICK_MS, *l, *r);
        ++n;
    }
    CHECK(ctl->state != from);   /* the cap was hit -> the machine is stuck */
    if (ticks_used != NULL) {
        *ticks_used = n;
    }
    return res;
}

/* Runs the fixed startup dwell except its last tick, with vision withheld.
 * The caller then supplies the transition tick itself, so it decides whether
 * FIND_BALL sees a ball (-> APPROACH_BALL) or not (-> SCAN). */
static void burn_startup(ball_push_controller_t *ctl, ball_vision_result_t *v,
                         int l, int r)
{
    const unsigned dwell = PUSH_START_FORWARD_MS / TICK_MS;
    CHECK(dwell > 1U);
    for (unsigned i = 0; i + 1U < dwell; ++i) {
        const ball_push_result_t res = feed_stale(ctl, v, TICK_MS, l, r);
        CHECK(res.state == BALL_PUSH_START_FORWARD);
        CHECK(res.forward == PUSH_START_FORWARD);
        CHECK(res.lateral == 0 && res.turn == 0);
        CHECK(res.drive_mode == BALL_DRIVE_APPROACH);
        CHECK(res.active);
        CHECK(res.attempt == 0);
    }
    CHECK(ctl->stage_ms == (dwell - 1U) * TICK_MS);
}

/* Fresh controller + left pocket centred + the given ball at (CENTRE_X, 50),
 * driven through the startup dwell and into APPROACH phase 2 (the mandatory
 * straight run). The ball is centred and not "far", so phase 0 and the
 * phase-1 heading/sprint branch are both passed over in the entry tick. */
static void enter_approach(ball_push_controller_t *ctl, ball_vision_result_t *v,
                           ball_color_t color, int radius, int *l, int *r)
{
    ball_push_init(ctl);
    blank_vision(v);
    add_pocket(v, 0, CENTER_X, 12, 50, 4, 70, 20);
    burn_startup(ctl, v, *l, *r);

    add_ball(v, color, CENTER_X, 50, radius);
    const ball_push_result_t res = feed_new(ctl, v, TICK_MS, *l, *r);
    CHECK(res.state == BALL_PUSH_APPROACH_BALL);
    CHECK(ctl->approach_phase == 2);
    CHECK(res.forward == PUSH_APPROACH_FORWARD_NEAR);   /* cy 50 >= 38 */
    CHECK(res.lateral == 0 && res.turn == 0);
    CHECK(res.drive_mode == BALL_DRIVE_APPROACH);
}

/* Runs the mandatory straight phase to completion and returns the state the
 * controller cascaded into on the transition tick: ALIGN normally, or BACKOFF
 * when the ball is already inside PUSH_ALIGN_SAFE_RADIUS_PX. */
static ball_push_state_t finish_approach(ball_push_controller_t *ctl,
                                         ball_vision_result_t *v,
                                         int *l, int *r)
{
    unsigned ticks = 0;
    run_until_changed(ctl, v, BALL_PUSH_APPROACH_BALL, 0, 0, l, r, 500, &ticks);
    CHECK(ticks == PUSH_APPROACH_FORWARD_MIN_MS / TICK_MS - 1U);
    return ctl->state;
}

/* ------------------------------------------------------------------ */

static void test_mapping_macros(void)
{
    CHECK(BALL_TASK_FIRST_COLOR == BALL_COLOR_RED);
    CHECK(BALL_TASK_FIRST_POCKET == 0);
    CHECK(BALL_TASK_SECOND_COLOR == BALL_COLOR_BLUE);
    CHECK(BALL_TASK_SECOND_POCKET == 1);
    CHECK(PUSH_FAR_SPRINT_ENABLE == 1);
}

static void test_startup_run_then_scan_without_a_ball(void)
{
    ball_push_controller_t ctl;
    ball_push_init(&ctl);
    CHECK(ctl.state == BALL_PUSH_START_FORWARD);
    CHECK(ctl.scan_direction == 1);

    ball_vision_result_t v;
    blank_vision(&v);
    int l = 0, r = 0;
    burn_startup(&ctl, &v, l, r);

    /* Last dwell tick with still no vision: FIND_BALL hands over to SCAN, and
     * because state_changed re-runs the new state in the same tick, this one
     * call already returns the first SCAN rotation command. */
    const ball_push_result_t res = feed_stale(&ctl, &v, TICK_MS, l, r);
    CHECK(res.state == BALL_PUSH_SCAN);
    CHECK(res.turn == PUSH_SCAN_TURN);          /* first leg sweeps RIGHT */
    CHECK(res.forward == 0 && res.lateral == 0);
    CHECK(res.drive_mode == BALL_DRIVE_OPEN);
    CHECK(res.active);
    CHECK(ctl.scan_leg == 0);
    CHECK(ctl.scan_direction == 1);
    CHECK(ctl.stage_target == PUSH_SCAN_FIRST_COUNTS);
}

static void test_scan_sweeps_all_legs_then_faults(void)
{
    ball_push_controller_t ctl;
    ball_push_init(&ctl);
    ball_vision_result_t v;
    blank_vision(&v);
    int l = 0, r = 0;
    burn_startup(&ctl, &v, l, r);
    feed_stale(&ctl, &v, TICK_MS, l, r);
    CHECK(ctl.state == BALL_PUSH_SCAN);

    /* Rotate 60 counts per tick and never see a ball: four legs, three
     * direction flips, then FAULT_STOP. */
    unsigned flips = 0;
    int last_direction = ctl.scan_direction;
    for (unsigned tick = 0; tick < 4000U && ctl.state == BALL_PUSH_SCAN; ++tick) {
        l += 30;
        r -= 30;
        feed_stale(&ctl, &v, TICK_MS, l, r);
        if (ctl.scan_direction != last_direction) {
            last_direction = ctl.scan_direction;
            ++flips;
        }
    }
    CHECK(ctl.state == BALL_PUSH_FAULT_STOP);
    CHECK(flips == PUSH_SCAN_MAX_LEGS - 1U);
    CHECK(ctl.scan_leg == PUSH_SCAN_MAX_LEGS);

    ball_push_status_t st;
    ball_push_get_status(&st);
    CHECK(st.state == BALL_PUSH_FAULT_STOP);    /* terminal states publish */
    CHECK(!st.red_pocketed && !st.blue_pocketed);
}

static void test_scan_stall_faults(void)
{
    ball_push_controller_t ctl;
    ball_push_init(&ctl);
    ball_vision_result_t v;
    blank_vision(&v);
    int l = 0, r = 0;
    burn_startup(&ctl, &v, l, r);
    feed_stale(&ctl, &v, TICK_MS, l, r);
    CHECK(ctl.state == BALL_PUSH_SCAN);

    /* Wheels frozen while SCAN wants rotation: the shared stall watchdog fires
     * after PUSH_SCAN_STALL_MS with less than PUSH_SCAN_STALL_COUNTS movement.
     * The entry tick already accrued one interval. */
    unsigned ticks = 0;
    while (ctl.state == BALL_PUSH_SCAN && ticks < 500U) {
        feed_stale(&ctl, &v, TICK_MS, l, r);
        ++ticks;
    }
    CHECK(ctl.state == BALL_PUSH_FAULT_STOP);
    CHECK(ticks == PUSH_SCAN_STALL_MS / TICK_MS - 1U);
}

static void test_approach_phase0_strafes_only(void)
{
    ball_push_controller_t ctl;
    ball_push_init(&ctl);
    ball_vision_result_t v;
    blank_vision(&v);
    add_pocket(&v, 0, CENTER_X, 12, 50, 4, 70, 20);
    int l = 0, r = 0;
    burn_startup(&ctl, &v, l, r);

    /* Ball 30 px right of centre: phase 0 must strafe and nothing else. */
    add_ball(&v, BALL_COLOR_RED, 90, 50, 6);
    const ball_push_result_t res = feed_new(&ctl, &v, TICK_MS, l, r);
    CHECK(res.state == BALL_PUSH_APPROACH_BALL);
    CHECK(ctl.approach_phase == 0);
    CHECK(res.lateral == -PUSH_APPROACH_LATERAL_SPEED);  /* ball right -> move right */
    CHECK(res.forward == 0);
    CHECK(res.turn == 0);
    CHECK(res.drive_mode == BALL_DRIVE_LATERAL);

    /* Ball left of centre flips the strafe sign. */
    add_ball(&v, BALL_COLOR_RED, 30, 50, 6);
    const ball_push_result_t res2 = feed_new(&ctl, &v, TICK_MS, l, r);
    CHECK(res2.lateral == PUSH_APPROACH_LATERAL_SPEED);
    CHECK(res2.drive_mode == BALL_DRIVE_LATERAL);
    CHECK(ctl.approach_phase == 0);
}

static void test_approach_strafe_timeout_advances_the_phase(void)
{
    ball_push_controller_t ctl;
    ball_push_init(&ctl);
    ball_vision_result_t v;
    blank_vision(&v);
    add_pocket(&v, 0, CENTER_X, 12, 50, 4, 70, 20);
    int l = 0, r = 0;
    burn_startup(&ctl, &v, l, r);

    /* A ball that can never be centred (detection flicker) must not wedge
     * phase 0 forever. */
    add_ball(&v, BALL_COLOR_RED, 100, 50, 6);
    feed_new(&ctl, &v, TICK_MS, l, r);
    CHECK(ctl.approach_phase == 0);

    const unsigned timeout_ticks = PUSH_APPROACH_STRAFE_TIMEOUT_MS / TICK_MS;
    /* Tick 1 was the feed_new above (strafe_ms = TICK_MS), so ticks 2 ..
     * timeout_ticks-1 are still inside the window. */
    for (unsigned i = 1; i + 1U < timeout_ticks; ++i) {
        const ball_push_result_t res = feed_hold(&ctl, &v, TICK_MS, l, r);
        CHECK(res.state == BALL_PUSH_APPROACH_BALL);
        CHECK(ctl.approach_phase == 0);
        CHECK(res.lateral == -PUSH_APPROACH_LATERAL_SPEED);
    }
    /* The tick that reaches the timeout still emits its strafe, then advances. */
    const ball_push_result_t flip = feed_hold(&ctl, &v, TICK_MS, l, r);
    CHECK(ctl.approach_phase == 1);
    CHECK(ctl.approach_strafe_ms == 0);
    CHECK(flip.state == BALL_PUSH_APPROACH_BALL);
    CHECK(flip.lateral == -PUSH_APPROACH_LATERAL_SPEED);
    CHECK(flip.forward == 0);

    /* Phase 1 re-centres before doing anything else, so an off-centre ball
     * still strafes - the timeout advances the phase, it does not force
     * forward motion. */
    const ball_push_result_t res = feed_hold(&ctl, &v, TICK_MS, l, r);
    CHECK(res.state == BALL_PUSH_APPROACH_BALL);
    CHECK(res.lateral == -PUSH_APPROACH_LATERAL_SPEED);
    CHECK(res.forward == 0);
    CHECK(ctl.approach_phase == 1);
}

static void test_approach_phase1_corrects_heading_for_a_far_ball(void)
{
    ball_push_controller_t ctl;
    ball_push_init(&ctl);
    ball_vision_result_t v;
    blank_vision(&v);
    int l = 0, r = 0;
    burn_startup(&ctl, &v, l, r);

    /* Far ball (radius <= 5, cy < 40) centred in the frame, but the pocket is
     * 30 px left of centre: phase 1 must rotate, not drive forward. */
    add_pocket(&v, 0, 30, 12, 20, 4, 40, 20);
    add_ball(&v, BALL_COLOR_RED, CENTER_X, 20, 4);
    const ball_push_result_t res = feed_new(&ctl, &v, TICK_MS, l, r);
    CHECK(res.state == BALL_PUSH_APPROACH_BALL);
    CHECK(ctl.approach_phase == 1);
    CHECK(res.forward == 0 && res.lateral == 0);
    CHECK(res.drive_mode == BALL_DRIVE_OPEN);
    /* err = pocket.cx - centre = 30 - 60 = -30 -> KP gives 2*err = -60, which
     * is inside +/-PUSH_APPROACH_HEADING_MAX but weaker than the minimum
     * magnitude, so it is bumped out to -PUSH_APPROACH_HEADING_TURN: a heading
     * correction must be strong enough to actually overcome static friction. */
    CHECK(res.turn == -PUSH_APPROACH_HEADING_TURN);
    CHECK(res.turn < 0 && res.turn >= -PUSH_APPROACH_HEADING_MAX);

    /* A pocket far enough off bearing saturates at the clamp instead. */
    add_pocket(&v, 0, 0, 12, 0, 4, 10, 20);
    const ball_push_result_t res2 = feed_new(&ctl, &v, TICK_MS, l, r);
    CHECK(res2.state == BALL_PUSH_APPROACH_BALL);
    CHECK(ctl.approach_phase == 1);
    CHECK(res2.turn == -PUSH_APPROACH_HEADING_MAX);
}

static void test_align_strafes_and_never_rotates(void)
{
    ball_push_controller_t ctl;
    ball_vision_result_t v;
    int l = 0, r = 0;
    enter_approach(&ctl, &v, BALL_COLOR_RED, 6, &l, &r);
    CHECK(finish_approach(&ctl, &v, &l, &r) == BALL_PUSH_ALIGN);

    /* One tick of the confirm window has already run on the entry tick. */
    CHECK(ctl.align_ok_ms == TICK_MS);

    /* Ball 16 px right of the pocket column: ALIGN strafes onto it. KP*err =
     * 2*16 = 32 is below the static-friction floor, so it is bumped up to
     * PUSH_ALIGN_STRAFE_MIN_PWM. Crucially turn stays 0. */
    add_ball(&v, BALL_COLOR_RED, 76, 50, 6);
    const ball_push_result_t res = feed_new(&ctl, &v, TICK_MS, l, r);
    CHECK(res.state == BALL_PUSH_ALIGN);
    CHECK(res.turn == 0);
    CHECK(res.forward == 0);
    CHECK(res.lateral == -PUSH_ALIGN_STRAFE_MIN_PWM);
    CHECK(res.drive_mode == BALL_DRIVE_LATERAL);
    CHECK(res.active);
    CHECK(ctl.align_ok_ms == TICK_MS);   /* strafing does not advance the window */

    /* Large error clamps to PUSH_ALIGN_STRAFE_MAX. */
    add_ball(&v, BALL_COLOR_RED, 100, 50, 6);
    const ball_push_result_t res2 = feed_new(&ctl, &v, TICK_MS, l, r);
    CHECK(res2.lateral == -PUSH_ALIGN_STRAFE_MAX);

    /* Back on the column: the window runs on wall time, so it converges even
     * with a frozen frame, and hands over to PUSH inside the same tick. */
    add_ball(&v, BALL_COLOR_RED, CENTER_X, 50, 6);
    unsigned ticks = 0;
    const ball_push_result_t res3 =
        run_until_changed(&ctl, &v, BALL_PUSH_ALIGN, 0, 0, &l, &r, 500, &ticks);
    CHECK(ctl.state == BALL_PUSH_PUSH);
    CHECK(ticks == PUSH_ALIGN_CONFIRM_MS / TICK_MS - 1U);
    CHECK(res3.forward == PUSH_HARD_FORWARD);
    CHECK(res3.lateral == 0 && res3.turn == 0);
    CHECK(res3.drive_mode == BALL_DRIVE_OPEN);
}

static void test_align_backs_off_a_bumper_range_ball(void)
{
    ball_push_controller_t ctl;
    ball_vision_result_t v;
    int l = 0, r = 0;
    /* radius 10 >= PUSH_ALIGN_SAFE_RADIUS_PX: too close to strafe. */
    enter_approach(&ctl, &v, BALL_COLOR_RED, 10, &l, &r);

    /* ALIGN rejects the ball on its entry tick and cascades into BACKOFF, so
     * the reverse command comes back from the very same call. */
    CHECK(finish_approach(&ctl, &v, &l, &r) == BALL_PUSH_BACKOFF);
    CHECK(ctl.stage_target == PUSH_BACKOFF_COUNTS);

    /* Back off far enough that the ball reads small again, otherwise ALIGN
     * would immediately bounce back into BACKOFF. */
    add_ball(&v, BALL_COLOR_RED, CENTER_X, 50, 6);
    unsigned ticks = 0;
    const ball_push_result_t res =
        run_until_changed(&ctl, &v, BALL_PUSH_BACKOFF, -40, -40, &l, &r,
                          500, &ticks);
    /* |dl|+|dr| = 80 per tick, so 3 ticks clear the 220-count budget. */
    CHECK(ticks == 3U);
    CHECK(res.state == BALL_PUSH_ALIGN);   /* backed off -> try to align again */
    CHECK(res.forward == 0 && res.lateral == 0 && res.turn == 0);

    /* ALIGN now converges normally. */
    unsigned align_ticks = 0;
    run_until_changed(&ctl, &v, BALL_PUSH_ALIGN, 0, 0, &l, &r, 500, &align_ticks);
    CHECK(ctl.state == BALL_PUSH_PUSH);
}

static void test_push_is_straight_and_time_terminated(void)
{
    ball_push_controller_t ctl;
    ball_vision_result_t v;
    int l = 0, r = 0;
    enter_approach(&ctl, &v, BALL_COLOR_RED, 6, &l, &r);
    CHECK(finish_approach(&ctl, &v, &l, &r) == BALL_PUSH_ALIGN);
    run_until_changed(&ctl, &v, BALL_PUSH_ALIGN, 0, 0, &l, &r, 500, NULL);
    CHECK(ctl.state == BALL_PUSH_PUSH);

    /* PUSH ignores vision entirely: no steering, no distance target, no stall
     * watchdog. It is purely a strong straight command for a fixed window.
     * Drop the ball to prove it - and note that this tick is itself one PUSH
     * interval, so it eats one tick off the remaining window. */
    no_balls(&v);
    feed_new(&ctl, &v, TICK_MS, l, r);
    CHECK(ctl.state == BALL_PUSH_PUSH);

    unsigned ticks = 0;
    const ball_push_result_t res =
        run_until_changed(&ctl, &v, BALL_PUSH_PUSH, 0, 0, &l, &r, 500, &ticks);
    CHECK(ticks == PUSH_HARD_PUSH_MS / TICK_MS - 2U);

    /* Reaching the end of the window counts the ball as pocketed and, for the
     * first attempt, cascades straight into EGRESS. */
    CHECK(ctl.red_pocketed);
    CHECK(!ctl.blue_pocketed);
    CHECK(ctl.attempt == 1);
    CHECK(ctl.state == BALL_PUSH_EGRESS);
    CHECK(res.forward == -PUSH_EGRESS_SPEED);
    CHECK(res.drive_mode == BALL_DRIVE_APPROACH);
    CHECK(ctl.stage_target == PUSH_EGRESS_REVERSE_COUNTS);
}

static void test_far_sprint_triggers_on_an_aligned_far_ball(void)
{
    ball_push_controller_t ctl;
    ball_push_init(&ctl);
    ball_vision_result_t v;
    blank_vision(&v);
    add_pocket(&v, 0, CENTER_X, 12, 50, 4, 70, 20);
    int l = 0, r = 0;
    burn_startup(&ctl, &v, l, r);

    /* Far (radius <= 5, cy <= 35), centred on the frame AND already on the
     * ball->pocket line: the open-loop charge replaces the whole near-field
     * phase in a single tick. */
    add_ball(&v, BALL_COLOR_RED, CENTER_X, 20, 4);
    const ball_push_result_t res = feed_new(&ctl, &v, TICK_MS, l, r);
    CHECK(res.state == BALL_PUSH_FAR_SPRINT);
    CHECK(res.forward == PUSH_FAR_SPRINT_FORWARD);
    CHECK(res.lateral == 0 && res.turn == 0);
    CHECK(res.drive_mode == BALL_DRIVE_APPROACH);
    CHECK(ctl.stage_target ==
          (int64_t)PUSH_FAR_SPRINT_BASE_COUNTS +
          (int64_t)(PUSH_FAR_SPRINT_CY_REF_PX - 20) * PUSH_FAR_SPRINT_CY_SCALE);
    CHECK(ctl.stage_target >= PUSH_FAR_SPRINT_MIN_COUNTS);
    CHECK(ctl.stage_target <= PUSH_FAR_SPRINT_MAX_COUNTS);

    /* The sprint is encoder-supervised and hands over to the ordinary PUSH. */
    unsigned ticks = 0;
    const ball_push_result_t res2 =
        run_until_changed(&ctl, &v, BALL_PUSH_FAR_SPRINT, 60, 60, &l, &r,
                          500, &ticks);
    CHECK(ctl.state == BALL_PUSH_PUSH);
    CHECK(res2.forward == PUSH_HARD_FORWARD);
}

static void test_far_sprint_stall_backs_out_to_find_ball(void)
{
    ball_push_controller_t ctl;
    ball_push_init(&ctl);
    ball_vision_result_t v;
    blank_vision(&v);
    add_pocket(&v, 0, CENTER_X, 12, 50, 4, 70, 20);
    int l = 0, r = 0;
    burn_startup(&ctl, &v, l, r);
    add_ball(&v, BALL_COLOR_RED, CENTER_X, 20, 4);
    feed_new(&ctl, &v, TICK_MS, l, r);
    CHECK(ctl.state == BALL_PUSH_FAR_SPRINT);

    /* Wedged on the ball or the table edge: BACKOUT is decision-only, so it
     * cascades through FIND_BALL in the same tick. With the ball gone the
     * machine ends up scanning again - and, this being the "not pocketed"
     * branch, it must NOT have advanced to EGRESS. */
    no_balls(&v);
    feed_new(&ctl, &v, TICK_MS, l, r);
    unsigned ticks = 0;
    run_until_changed(&ctl, &v, BALL_PUSH_FAR_SPRINT, 0, 0, &l, &r, 500, &ticks);
    /* The entry tick and the no_balls tick above each accrued one stall
     * interval, so only timeout/10 - 2 remain. */
    CHECK(ticks == PUSH_SCAN_STALL_MS / TICK_MS - 2U);
    CHECK(ctl.state == BALL_PUSH_SCAN);
    CHECK(!ctl.red_pocketed);
    CHECK(ctl.attempt == 0);
}

static void test_ball_lost_in_approach_returns_to_scan(void)
{
    ball_push_controller_t ctl;
    ball_push_init(&ctl);
    ball_vision_result_t v;
    blank_vision(&v);
    add_pocket(&v, 0, CENTER_X, 12, 50, 4, 70, 20);
    int l = 0, r = 0;
    burn_startup(&ctl, &v, l, r);
    add_ball(&v, BALL_COLOR_RED, CENTER_X, 50, 6);
    feed_new(&ctl, &v, TICK_MS, l, r);
    CHECK(ctl.state == BALL_PUSH_APPROACH_BALL);
    CHECK(ctl.ball_lost_frames == 0);

    /* Losing the ball holds the car still; only NEW frames count, so frozen
     * frames in between cannot multi-count one slow camera image. */
    for (unsigned i = 0; i + 1U < PUSH_APPROACH_LOST_MAX_FRAMES; ++i) {
        no_balls(&v);
        const ball_push_result_t res = feed_new(&ctl, &v, TICK_MS, l, r);
        CHECK(res.state == BALL_PUSH_APPROACH_BALL);
        CHECK(res.forward == 0 && res.lateral == 0 && res.turn == 0);
        CHECK(!res.active);
        CHECK(ctl.ball_lost_frames == i + 1U);
        feed_hold(&ctl, &v, TICK_MS, l, r);       /* stale repeats do not count */
        CHECK(ctl.ball_lost_frames == i + 1U);
    }
    no_balls(&v);
    const ball_push_result_t res = feed_new(&ctl, &v, TICK_MS, l, r);
    CHECK(res.state == BALL_PUSH_SCAN);           /* the last allowed frame */
    CHECK(ctl.ball_lost_frames == 0);             /* reset_stage cleared it */
}

/* Drives both balls home: red -> left pocket, then blue -> right pocket. */
static void run_to_done(ball_push_controller_t *ctl, ball_vision_result_t *v,
                        int *l, int *r)
{
    enter_approach(ctl, v, BALL_COLOR_RED, 6, l, r);
    CHECK(finish_approach(ctl, v, l, r) == BALL_PUSH_ALIGN);
    run_until_changed(ctl, v, BALL_PUSH_ALIGN, 0, 0, l, r, 500, NULL);
    CHECK(ctl->state == BALL_PUSH_PUSH);
    run_until_changed(ctl, v, BALL_PUSH_PUSH, 0, 0, l, r, 500, NULL);
    CHECK(ctl->state == BALL_PUSH_EGRESS);
    CHECK(ctl->attempt == 1);
    CHECK(ctl->red_pocketed);

    /* EGRESS reverses clear of the table edge. */
    ball_push_result_t res =
        run_until_changed(ctl, v, BALL_PUSH_EGRESS, 60, 60, l, r, 500, NULL);
    CHECK(ctl->state == BALL_PUSH_POST_EGRESS_TURN);
    CHECK(res.turn == PUSH_POST_EGRESS_TURN_SPEED);
    CHECK(res.forward == 0 && res.lateral == 0);
    CHECK(res.drive_mode == BALL_DRIVE_OPEN);

    /* Then turns right and drives forward to a better viewpoint. */
    res = run_until_changed(ctl, v, BALL_PUSH_POST_EGRESS_TURN, 30, -30,
                            l, r, 500, NULL);
    CHECK(ctl->state == BALL_PUSH_POST_EGRESS_FORWARD);
    CHECK(res.forward == PUSH_POST_EGRESS_FORWARD_SPEED);
    CHECK(res.drive_mode == BALL_DRIVE_APPROACH);

    /* The blue ball is already in view, so FIND_BALL cascades into APPROACH. */
    blank_vision(v);
    add_pocket(v, 1, CENTER_X, 12, 50, 4, 70, 20);
    add_ball(v, BALL_COLOR_BLUE, CENTER_X, 50, 6);
    res = run_until_changed(ctl, v, BALL_PUSH_POST_EGRESS_FORWARD, 60, 60,
                            l, r, 500, NULL);
    CHECK(ctl->state == BALL_PUSH_APPROACH_BALL);
    CHECK(ctl->attempt == 1);

    /* Same alignment path for the second ball, into the RIGHT pocket. */
    res = feed_new(ctl, v, TICK_MS, *l, *r);
    CHECK(res.state == BALL_PUSH_APPROACH_BALL);
    run_until_changed(ctl, v, BALL_PUSH_APPROACH_BALL, 0, 0, l, r, 500, NULL);
    CHECK(ctl->state == BALL_PUSH_ALIGN);
    run_until_changed(ctl, v, BALL_PUSH_ALIGN, 0, 0, l, r, 500, NULL);
    CHECK(ctl->state == BALL_PUSH_PUSH);
    res = run_until_changed(ctl, v, BALL_PUSH_PUSH, 0, 0, l, r, 500, NULL);

    /* Second attempt completing the window finishes the whole task. */
    CHECK(ctl->state == BALL_PUSH_DONE);
    CHECK(ctl->blue_pocketed);
    CHECK(ctl->attempt == 1);
    CHECK(res.state == BALL_PUSH_DONE);
    CHECK(res.state_changed);
    CHECK(!res.active);
    CHECK(res.forward == 0 && res.lateral == 0 && res.turn == 0);
}

static void test_full_two_ball_run_ends_done_and_sticks(void)
{
    ball_push_controller_t ctl;
    ball_vision_result_t v;
    int l = 0, r = 0;
    run_to_done(&ctl, &v, &l, &r);

    ball_push_status_t st;
    ball_push_get_status(&st);
    CHECK(st.state == BALL_PUSH_DONE);
    CHECK(st.attempt == 1);
    CHECK(st.red_pocketed && st.blue_pocketed);

    /* DONE is sticky and motionless, and keeps reporting itself. */
    for (int i = 0; i < 5; ++i) {
        l += 10;
        r += 10;
        const ball_push_result_t res = feed_new(&ctl, &v, TICK_MS, l, r);
        CHECK(res.state == BALL_PUSH_DONE);
        CHECK(!res.state_changed);
        CHECK(!res.active);
        CHECK(res.forward == 0 && res.lateral == 0 && res.turn == 0);
    }
    ball_push_get_status(&st);
    CHECK(st.state == BALL_PUSH_DONE);
}

static void test_task_timeout_faults_and_publishes(void)
{
    ball_push_controller_t ctl;
    ball_push_init(&ctl);
    ball_vision_result_t v;
    blank_vision(&v);

    /* One oversized tick straight through the whole-task watchdog. */
    const ball_push_result_t res =
        ball_push_update(&ctl, &v, false, BALL_TASK_TIMEOUT_MS + 1U, 0, 0, 0);
    CHECK(ctl.state == BALL_PUSH_FAULT_STOP);
    CHECK(res.state == BALL_PUSH_FAULT_STOP);
    CHECK(res.state_changed);
    CHECK(!res.active);

    ball_push_status_t st;
    ball_push_get_status(&st);
    CHECK(st.state == BALL_PUSH_FAULT_STOP);

    /* FAULT_STOP is sticky. */
    const ball_push_result_t res2 = feed_new(&ctl, &v, TICK_MS, 0, 0);
    CHECK(res2.state == BALL_PUSH_FAULT_STOP);
    CHECK(!res2.state_changed);
    CHECK(res2.forward == 0 && res2.lateral == 0 && res2.turn == 0);
}

static void test_null_controller_is_safe(void)
{
    const ball_push_result_t res =
        ball_push_update(NULL, NULL, false, TICK_MS, 0, 0, 0);
    CHECK(res.state == BALL_PUSH_FAULT_STOP);
    CHECK(res.state_changed);
    ball_push_init(NULL);          /* must not crash */
}

int main(void)
{
    CHECK(CAMERA_FLIP_HORIZONTAL == 1);
    CHECK(CAMERA_FLIP_VERTICAL == 1);

    test_mapping_macros();
    test_startup_run_then_scan_without_a_ball();
    test_scan_sweeps_all_legs_then_faults();
    test_scan_stall_faults();
    test_approach_phase0_strafes_only();
    test_approach_strafe_timeout_advances_the_phase();
    test_approach_phase1_corrects_heading_for_a_far_ball();
    test_align_strafes_and_never_rotates();
    test_align_backs_off_a_bumper_range_ball();
    test_push_is_straight_and_time_terminated();
    test_far_sprint_triggers_on_an_aligned_far_ball();
    test_far_sprint_stall_backs_out_to_find_ball();
    test_ball_lost_in_approach_returns_to_scan();
    test_full_two_ball_run_ends_done_and_sticks();
    test_task_timeout_faults_and_publishes();
    test_null_controller_is_safe();
    puts("ball_push sequence tests passed");
    return 0;
}
