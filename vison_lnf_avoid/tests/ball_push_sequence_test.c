#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ball_push.h"
#include "board_config.h"

#define CHECK(condition) do {                                                   \
    if (!(condition)) {                                                         \
        fprintf(stderr, "FAIL %s:%d: %s\n", __func__, __LINE__, #condition);   \
        exit(1);                                                                \
    }                                                                           \
} while (0)

#define TEST_WIDTH   120U
#define TEST_HEIGHT   80U
#define CENTER_X      60

/* ------------------------------------------------------------------ */
/* Vision-feed helpers: the controller only acts on NEW fresh frames,  */
/* so the test distinguishes feed_new (new seq) from feed_hold        */
/* (same seq, fresh) and feed_stale (not fresh).                       */
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
    ball->color = color;
    ball->cx = cx;
    ball->cy = cy;
    ball->radius = radius;
    ball->area = (unsigned)(314 * radius * radius / 100);
    ball->x0 = cx - radius;
    ball->x1 = cx + radius;
    ball->y0 = cy - radius;
    ball->y1 = cy + radius;
    if (v->ball_count == 0U) {
        v->ball_count = 1;
    } else {
        v->ball_count = 2;
    }
}

static void no_balls(ball_vision_result_t *v)
{
    v->balls[BALL_COLOR_RED].present = false;
    v->balls[BALL_COLOR_BLUE].present = false;
    v->ball_count = 0;
}

/* Puts the pocket of the given side in vision. Slots must stay cx-sorted;
 * a single pocket goes into slot 0. */
static void add_pocket(ball_vision_result_t *v, unsigned side,
                       int cx, int cy, int x0, int y0, int x1, int y1)
{
    pocket_blob_t *slot = side == 0U ? &v->pockets[0] : &v->pockets[1];
    if (side == 0U) {
        v->pockets[1].visible = false;
    } else {
        v->pockets[0].visible = false;
    }
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
                                   uint32_t ms,
                                   int left_count,
                                   int right_count)
{
    v->frame_seq = ++s_seq;
    return ball_push_update(ctl, v, true, ms, left_count, right_count, 0);
}

static ball_push_result_t feed_hold(ball_push_controller_t *ctl,
                                    ball_vision_result_t *v,
                                    uint32_t ms,
                                    int left_count,
                                    int right_count)
{
    return ball_push_update(ctl, v, true, ms, left_count, right_count, 0);
}

static ball_push_result_t feed_stale(ball_push_controller_t *ctl,
                                     ball_vision_result_t *v,
                                     uint32_t ms,
                                     int left_count,
                                     int right_count)
{
    return ball_push_update(ctl, v, false, ms, left_count, right_count, 0);
}

/* Holds the given geometry until ALIGN converges and PUSH begins (ALIGN
 * accumulates by wall time on identical frames). */
static void hold_align_to_push(ball_push_controller_t *ctl,
                               ball_vision_result_t *v,
                               int *left_count,
                               int *right_count)
{
    ball_push_result_t result = {0};
    const unsigned ticks = PUSH_ALIGN_CONFIRM_MS / 10U;
    for (unsigned i = 0; i < ticks; ++i) {
        result = feed_hold(ctl, v, 10, *left_count, *right_count);
    }
    CHECK(result.state == BALL_PUSH_PUSH);
}

/* ------------------------------------------------------------------ */

/* Runs a straight push to success for the CURRENT attempt, given a frame
 * where the ball is centered, and pushes the ball into the given pocket
 * bbox over successive new frames. The ball starts at (cx, cy). */
static void push_ball_into_pocket(ball_push_controller_t *ctl,
                                  ball_vision_result_t *v,
                                  ball_color_t color,
                                  int ball_cx, int ball_cy, int radius,
                                  int pocket_x0, int pocket_y0,
                                  int pocket_x1, int pocket_y1,
                                  int *left_count, int *right_count,
                                  bool expect_done)
{
    /* Phase 1: advance the ball toward the pocket mouth. */
    int cy = ball_cy;
    while (cy - 4 > pocket_y1) {
        cy -= 6;
        add_ball(v, color, ball_cx, cy, radius);
        *left_count += 40;
        *right_count += 40;
        CHECK(feed_new(ctl, v, 10, *left_count, *right_count).state ==
              BALL_PUSH_PUSH);
    }
    /* Phase 2: inside the (shrunk) pocket bbox for 3 distinct frames. */
    CHECK(cy > pocket_y0);
    int hits = 0;
    while (hits < (int)PUSH_POCKET_CONFIRM_FRAMES) {
        const bool inside =
            ball_cx >= pocket_x0 + PUSH_POCKET_HIT_MARGIN_PX &&
            ball_cx <= pocket_x1 - PUSH_POCKET_HIT_MARGIN_PX &&
            cy >= pocket_y0 + PUSH_POCKET_HIT_MARGIN_PX &&
            cy <= pocket_y1 - PUSH_POCKET_HIT_MARGIN_PX;
        *left_count += 40;
        *right_count += 40;
        const ball_push_result_t result =
            feed_new(ctl, v, 10, *left_count, *right_count);
        if (inside) {
            ++hits;
        } else if (result.state != BALL_PUSH_PUSH) {
            return; /* a miss/transition already happened */
        }
        cy -= 2;
        if (result.state != BALL_PUSH_PUSH) {
            CHECK(expect_done ? result.state == BALL_PUSH_DONE
                              : result.state == BALL_PUSH_EGRESS);
            return;
        }
        add_ball(v, color, ball_cx, cy, radius);
    }
}

static void test_two_balls_full_run(void)
{
    ball_push_controller_t ctl;
    ball_push_init(&ctl);
    ball_vision_result_t v;
    int left_count = 0;
    int right_count = 0;

    /* --- attempt 0: red into the left pocket ----------------------- */
    blank_vision(&v);
    add_pocket(&v, 0, 40, 12, 30, 4, 50, 20);
    add_ball(&v, BALL_COLOR_RED, CENTER_X, 30, 9);
    ball_push_result_t result = feed_new(&ctl, &v, 10, left_count, right_count);
    CHECK(result.state == BALL_PUSH_ALIGN);   /* creep exits immediately */
    CHECK(result.turn == -PUSH_ALIGN_TURN_MAX); /* pocket left of centre */
    CHECK(result.lateral == 0);

    /* Rotate: the pocket centre walks to the image centre. */
    const int pocket_steps[] = { 48, 56, 59, 60 };
    for (size_t i = 0; i < sizeof(pocket_steps) / sizeof(pocket_steps[0]); ++i) {
        const int p = pocket_steps[i];
        add_pocket(&v, 0, p, 12, p - 10, 4, p + 10, 20);
        result = feed_new(&ctl, &v, 10, left_count, right_count);
        CHECK(result.state == BALL_PUSH_ALIGN);
    }
    /* Ball 10 px right of centre -> strafe right (negative lateral, KP*err). */
    add_ball(&v, BALL_COLOR_RED, 70, 30, 9);
    result = feed_new(&ctl, &v, 10, left_count, right_count);
    CHECK(result.lateral == -PUSH_ALIGN_STRAFE_KP * 10);
    CHECK(result.drive_mode == BALL_DRIVE_LATERAL);

    /* Converged: ball and pocket centred. */
    add_ball(&v, BALL_COLOR_RED, CENTER_X, 30, 9);
    result = feed_new(&ctl, &v, 10, left_count, right_count);
    CHECK(result.state == BALL_PUSH_ALIGN);
    add_pocket(&v, 0, CENTER_X, 12, 50, 4, 70, 20);
    hold_align_to_push(&ctl, &v, &left_count, &right_count);
    CHECK(ctl.state == BALL_PUSH_PUSH);
    CHECK(result.forward == 0);

    /* Push into the pocket (bbox x 50..70, y 4..20). */
    result = feed_new(&ctl, &v, 10, left_count, right_count);
    CHECK(result.state == BALL_PUSH_PUSH);
    CHECK(result.forward == PUSH_PUSH_FORWARD);
    CHECK(result.drive_mode == BALL_DRIVE_APPROACH);

    add_ball(&v, BALL_COLOR_RED, CENTER_X, 30, 9);
    push_ball_into_pocket(&ctl, &v, BALL_COLOR_RED, CENTER_X, 30, 9,
                          50, 4, 70, 20, &left_count, &right_count,
                          false);
    CHECK(ctl.state == BALL_PUSH_EGRESS);
    CHECK(ctl.attempt == 1);
    CHECK(ctl.retry == 0);

    /* --- egress: reverse off the pocket ----------------------------- */
    while (ctl.state == BALL_PUSH_EGRESS) {
        left_count += 60;
        right_count += 60;
        result = feed_new(&ctl, &v, 10, left_count, right_count);
    }
    CHECK(result.state == BALL_PUSH_SCAN);
    CHECK(ctl.attempt == 1);

    /* --- attempt 1: blue into the right pocket ---------------------- */
    blank_vision(&v);
    add_pocket(&v, 1, 84, 12, 74, 4, 94, 20);
    add_ball(&v, BALL_COLOR_BLUE, 92, 28, 8);
    result = feed_new(&ctl, &v, 10, left_count, right_count);
    CHECK(result.state == BALL_PUSH_ALIGN);
    CHECK(result.turn == PUSH_ALIGN_TURN_MAX); /* pocket right of centre */
    CHECK(result.lateral == -PUSH_ALIGN_STRAFE_MAX); /* ball err 32 clamps */

    add_ball(&v, BALL_COLOR_BLUE, CENTER_X, 28, 8);
    add_pocket(&v, 1, CENTER_X, 12, 50, 4, 70, 20);
    result = feed_new(&ctl, &v, 10, left_count, right_count);
    CHECK(result.state == BALL_PUSH_ALIGN);
    hold_align_to_push(&ctl, &v, &left_count, &right_count);
    CHECK(ctl.state == BALL_PUSH_PUSH);

    add_ball(&v, BALL_COLOR_BLUE, CENTER_X, 28, 8);
    push_ball_into_pocket(&ctl, &v, BALL_COLOR_BLUE, CENTER_X, 28, 8,
                          50, 4, 70, 20, &left_count, &right_count,
                          true);
    CHECK(ctl.state == BALL_PUSH_DONE);
    CHECK(ctl.attempt == 1);

    /* DONE is sticky and motionless. */
    result = feed_new(&ctl, &v, 10, left_count + 1, right_count + 1);
    CHECK(result.state == BALL_PUSH_DONE);
    CHECK(!result.active);
    CHECK(result.forward == 0 && result.lateral == 0 && result.turn == 0);
}

static void test_creep_scan_when_no_ball_visible(void)
{
    ball_push_controller_t ctl;
    ball_push_init(&ctl);
    ball_vision_result_t v;
    blank_vision(&v);
    int left_count = 0;
    int right_count = 0;

    /* No fresh vision at first: creep by dead-reckon until the cap. */
    int steps = 0;
    while (ctl.state == BALL_PUSH_CREEP_OFF_FINISH && steps < 40) {
        left_count += 50;
        right_count += 50;
        const ball_push_result_t r =
            feed_stale(&ctl, &v, 10, left_count, right_count);
        if (r.state != BALL_PUSH_CREEP_OFF_FINISH) {
            break; /* transition tick reports the new state (SCAN) */
        }
        ++steps;
    }
    CHECK(ctl.state == BALL_PUSH_SCAN);

    /* Sweep all four legs without ever seeing the ball -> fault. */
    unsigned legs = 0;
    while (ctl.state == BALL_PUSH_SCAN) {
        const ball_push_result_t r =
            feed_stale(&ctl, &v, 10, left_count, right_count);
        if (r.state != BALL_PUSH_SCAN) {
            break; /* legs exhausted -> FAULT on this tick */
        }
        left_count += 300;
        right_count -= 300;
        ++legs;
        if (legs > 200U) {
            break; /* guard against a stuck machine */
        }
    }
    CHECK(ctl.state == BALL_PUSH_FAULT_STOP);
}

static void test_miss_backout_retry_then_fault(void)
{
    ball_push_controller_t ctl;
    ball_push_init(&ctl);
    ball_vision_result_t v;
    int left_count = 0;
    int right_count = 0;

    /* Reach SCAN by creeping without vision. */
    int steps = 0;
    blank_vision(&v);
    while (ctl.state == BALL_PUSH_CREEP_OFF_FINISH && steps < 40) {
        left_count += 50;
        right_count += 50;
        const ball_push_result_t r =
            feed_stale(&ctl, &v, 10, left_count, right_count);
        if (r.state != BALL_PUSH_CREEP_OFF_FINISH) {
            break;
        }
        ++steps;
    }
    CHECK(ctl.state == BALL_PUSH_SCAN);

    for (unsigned round = 0; round < BALL_TASK_RETRY_MAX; ++round) {
        /* Ball reappears centred -> align, hold, push. */
        blank_vision(&v);
        add_pocket(&v, 0, CENTER_X, 12, 50, 4, 70, 20);
        add_ball(&v, BALL_COLOR_RED, CENTER_X, 30, 9);
        ball_push_result_t result =
            feed_new(&ctl, &v, 10, left_count, right_count);
        CHECK(result.state == BALL_PUSH_ALIGN);
        hold_align_to_push(&ctl, &v, &left_count, &right_count);
        CHECK(ctl.state == BALL_PUSH_PUSH);

        /* Miss: the ball drifts far off the pocket line. */
        add_ball(&v, BALL_COLOR_RED, 20, 34, 9);
        result = feed_new(&ctl, &v, 10, left_count, right_count);
        CHECK(result.state == BALL_PUSH_BACKOUT);

        /* Back out fully: retry++ (or fault on the last round). */
        int out = 0;
        while (ctl.state == BALL_PUSH_BACKOUT && out < 30) {
            left_count += 60;
            right_count += 60;
            result = feed_new(&ctl, &v, 10, left_count, right_count);
            ++out;
        }
        /* After the full back-out the retry counter already advanced; with
         * the ball still visible the machine re-enters ALIGN immediately
         * (or SCAN if it is out of view again). */
        CHECK(ctl.state == BALL_PUSH_ALIGN || ctl.state == BALL_PUSH_SCAN ||
              ctl.state == BALL_PUSH_FAULT_STOP);
        CHECK(ctl.retry == round + 1U);
        if (ctl.state == BALL_PUSH_FAULT_STOP) {
            CHECK(result.state == BALL_PUSH_FAULT_STOP);
            CHECK(!result.active);
            return;
        }
    }
    CHECK(!"expected fault after exhausting retries");
}

static void test_ball_lost_inside_pocket_counts_success(void)
{
    ball_push_controller_t ctl;
    ball_push_init(&ctl);
    ball_vision_result_t v;
    int left_count = 0;
    int right_count = 0;

    blank_vision(&v);
    add_pocket(&v, 0, CENTER_X, 12, 50, 4, 70, 20);
    add_ball(&v, BALL_COLOR_RED, CENTER_X, 30, 9);
    CHECK(feed_new(&ctl, &v, 10, left_count, right_count).state ==
          BALL_PUSH_ALIGN);
    hold_align_to_push(&ctl, &v, &left_count, &right_count);
    CHECK(ctl.state == BALL_PUSH_PUSH);

    /* One frame with the ball inside the pocket (enlarged bbox), then it
     * disappears (rolled in / merged with the dark paint). */
    add_ball(&v, BALL_COLOR_RED, CENTER_X, 10, 4);
    left_count += 40;
    right_count += 40;
    CHECK(feed_new(&ctl, &v, 10, left_count, right_count).state ==
          BALL_PUSH_PUSH);
    no_balls(&v);
    int frames = 0;
    while (ctl.state == BALL_PUSH_PUSH && frames < 20) {
        left_count += 40;
        right_count += 40;
        feed_new(&ctl, &v, 10, left_count, right_count);
        ++frames;
    }
    CHECK(ctl.state == BALL_PUSH_EGRESS);
    CHECK(ctl.attempt == 1);
}

static void test_push_stall_is_a_miss(void)
{
    ball_push_controller_t ctl;
    ball_push_init(&ctl);
    ball_vision_result_t v;
    int left_count = 0;
    int right_count = 0;

    blank_vision(&v);
    add_pocket(&v, 0, CENTER_X, 12, 50, 4, 70, 20);
    add_ball(&v, BALL_COLOR_RED, CENTER_X, 30, 9);
    CHECK(feed_new(&ctl, &v, 10, left_count, right_count).state ==
          BALL_PUSH_ALIGN);
    hold_align_to_push(&ctl, &v, &left_count, &right_count);
    CHECK(ctl.state == BALL_PUSH_PUSH);

    /* Wheels frozen while PUSH wants motion: stall -> treated as a miss. */
    ball_push_result_t result = {0};
    unsigned ticks = (PUSH_PUSH_STALL_MS / 10U) + 3U;
    for (unsigned i = 0; i < ticks && ctl.state == BALL_PUSH_PUSH; ++i) {
        result = feed_hold(&ctl, &v, 10, left_count, right_count);
    }
    CHECK(ctl.state == BALL_PUSH_BACKOUT);
    CHECK(result.state == BALL_PUSH_BACKOUT);
}

static void test_task_timeout_faults(void)
{
    ball_push_controller_t ctl;
    ball_push_init(&ctl);
    ball_vision_result_t v;
    blank_vision(&v);
    const ball_push_result_t result =
        ball_push_update(&ctl, &v, false, BALL_TASK_TIMEOUT_MS + 1U, 0, 0, 0);
    CHECK(ctl.state == BALL_PUSH_FAULT_STOP);
    CHECK(result.state == BALL_PUSH_FAULT_STOP);
    CHECK(result.state_changed);
}

static void test_mapping_macros(void)
{
    CHECK(BALL_TASK_FIRST_COLOR == BALL_COLOR_RED);
    CHECK(BALL_TASK_FIRST_POCKET == 0);
    CHECK(BALL_TASK_SECOND_COLOR == BALL_COLOR_BLUE);
    CHECK(BALL_TASK_SECOND_POCKET == 1);
}

int main(void)
{
    test_mapping_macros();
    test_two_balls_full_run();
    test_creep_scan_when_no_ball_visible();
    test_miss_backout_retry_then_fault();
    test_ball_lost_inside_pocket_counts_success();
    test_push_stall_is_a_miss();
    test_task_timeout_faults();
    puts("ball_push sequence tests passed");
    return 0;
}
