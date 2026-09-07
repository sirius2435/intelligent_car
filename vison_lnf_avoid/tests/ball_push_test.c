#include <stdio.h>
#include <stdlib.h>

#include "ball_push.h"
#include "board_config.h"

#define CHECK(c) do { if (!(c)) { fprintf(stderr, "FAIL line %d: %s\n", \
    __LINE__, #c); exit(1); } } while (0)

static ball_push_result_t step(ball_push_controller_t *c, uint32_t elapsed_ms,
                               int left, int right, int rear)
{
    return ball_push_update(c, elapsed_ms, left, right, rear);
}

static void finish_settle(ball_push_controller_t *c,
                          int left, int right, int rear,
                          ball_push_state_t expected)
{
    const ball_push_result_t r =
        step(c, BALL_SCRIPT_SETTLE_MS, left, right, rear);
    CHECK(r.state == expected);
    CHECK(r.state_changed);
}

static void finish_side_stage(ball_push_controller_t *c, int target,
                              int *left, int *right, int rear,
                              ball_push_state_t next)
{
    *left += target;
    *right -= target;
    const ball_push_result_t r = step(c, 10, *left, *right, rear);
    CHECK(r.state == BALL_PUSH_SETTLE || r.state == BALL_PUSH_COMPLETE);
    if (r.state == BALL_PUSH_SETTLE) CHECK(c->next_state == next);
}

static void finish_lateral_stage(ball_push_controller_t *c, int target,
                                 int *left, int *right, int *rear,
                                 ball_push_state_t next)
{
    *left += (target + 1) / 2;
    *right -= (target + 1) / 2;
    *rear += target;
    const ball_push_result_t r = step(c, 10, *left, *right, *rear);
    CHECK(r.state == BALL_PUSH_SETTLE);
    CHECK(c->next_state == next);
}

static void test_fixed_route(void)
{
    int left = 100;
    int right = -50;
    int rear = 25;
    ball_push_controller_t c;
    ball_push_init(&c, left, right, rear);
    CHECK(c.state == BALL_PUSH_SETTLE);
    CHECK(c.next_state == BALL_PUSH_ADVANCE);
    CHECK(c.target_color == BALL_COLOR_RED);

    finish_settle(&c, left, right, rear, BALL_PUSH_ADVANCE);
    ball_push_result_t r = step(&c, 10, left, right, rear);
    CHECK(r.forward == BALL_SCRIPT_ADVANCE_FORWARD);
    finish_side_stage(&c, BALL_SCRIPT_ADVANCE_COUNTS,
                      &left, &right, rear, BALL_PUSH_ROTATE_RIGHT);

    finish_settle(&c, left, right, rear, BALL_PUSH_ROTATE_RIGHT);
    r = step(&c, 10, left, right, rear);
    CHECK(r.turn == BALL_SCRIPT_ROTATE_RIGHT_TURN);
    finish_side_stage(&c, BALL_SCRIPT_ROTATE_RIGHT_COUNTS,
                      &left, &right, rear, BALL_PUSH_ALIGN_RED);

    finish_settle(&c, left, right, rear, BALL_PUSH_ALIGN_RED);
    r = step(&c, 10, left, right, rear);
    CHECK(r.lateral == BALL_SCRIPT_RED_ALIGN_LATERAL);
    finish_lateral_stage(&c, BALL_SCRIPT_RED_ALIGN_COUNTS,
                         &left, &right, &rear, BALL_PUSH_PUSH_RED);

    finish_settle(&c, left, right, rear, BALL_PUSH_PUSH_RED);
    r = step(&c, 10, left, right, rear);
    CHECK(r.forward == BALL_SCRIPT_PUSH_FORWARD);
    finish_side_stage(&c, BALL_SCRIPT_RED_PUSH_COUNTS,
                      &left, &right, rear, BALL_PUSH_RETREAT_RED);
    CHECK(c.completed_balls == 1U);

    finish_settle(&c, left, right, rear, BALL_PUSH_RETREAT_RED);
    r = step(&c, 10, left, right, rear);
    CHECK(r.forward == BALL_SCRIPT_RETREAT_FORWARD);
    finish_side_stage(&c, BALL_SCRIPT_RED_PUSH_COUNTS,
                      &left, &right, rear, BALL_PUSH_RETURN_CENTER);

    finish_settle(&c, left, right, rear, BALL_PUSH_RETURN_CENTER);
    r = step(&c, 10, left, right, rear);
    CHECK(r.lateral == -BALL_SCRIPT_RED_ALIGN_LATERAL);
    finish_lateral_stage(&c, BALL_SCRIPT_RED_ALIGN_COUNTS,
                         &left, &right, &rear, BALL_PUSH_ALIGN_WHITE);
    CHECK(c.target_color == BALL_COLOR_WHITE);

    finish_settle(&c, left, right, rear, BALL_PUSH_ALIGN_WHITE);
    r = step(&c, 10, left, right, rear);
    CHECK(r.lateral == BALL_SCRIPT_WHITE_ALIGN_LATERAL);
    finish_lateral_stage(&c, BALL_SCRIPT_WHITE_ALIGN_COUNTS,
                         &left, &right, &rear, BALL_PUSH_PUSH_WHITE);

    finish_settle(&c, left, right, rear, BALL_PUSH_PUSH_WHITE);
    r = step(&c, 10, left, right, rear);
    CHECK(r.forward == BALL_SCRIPT_PUSH_FORWARD);
    left += BALL_SCRIPT_WHITE_PUSH_COUNTS;
    right -= BALL_SCRIPT_WHITE_PUSH_COUNTS;
    r = step(&c, 10, left, right, rear);
    CHECK(r.mission_complete);
    CHECK(r.just_completed_ball);
    CHECK(c.completed_balls == 2U);
}

static void test_slowest_wheel_controls_completion(void)
{
    ball_push_controller_t c;
    ball_push_init(&c, 0, 0, 0);
    finish_settle(&c, 0, 0, 0, BALL_PUSH_ADVANCE);
    ball_push_result_t r = step(&c, 10,
                                BALL_SCRIPT_ADVANCE_COUNTS, 0, 0);
    CHECK(r.state == BALL_PUSH_ADVANCE);
    CHECK(r.forward == BALL_SCRIPT_ADVANCE_FORWARD);

    c.state = BALL_PUSH_ALIGN_RED;
    c.state_ms = 0;
    c.start_left_count = c.start_right_count = c.start_rear_count = 0;
    r = step(&c, 10, BALL_SCRIPT_RED_ALIGN_COUNTS,
             BALL_SCRIPT_RED_ALIGN_COUNTS, 0);
    CHECK(r.state == BALL_PUSH_ALIGN_RED);
}

static void test_stall_fault(void)
{
    ball_push_controller_t c;
    ball_push_init(&c, 0, 0, 0);
    finish_settle(&c, 0, 0, 0, BALL_PUSH_ADVANCE);
    ball_push_result_t r = {0};
    for (uint32_t elapsed = 0; elapsed < BALL_SCRIPT_STALL_TIMEOUT_MS;
         elapsed += 10U) {
        r = step(&c, 10, 0, 0, 0);
    }
    CHECK(r.fault);
    CHECK(c.state == BALL_PUSH_FAULT_STOP);
}

int main(void)
{
    test_fixed_route();
    test_slowest_wheel_controls_completion();
    test_stall_fault();
    puts("fixed ball route tests passed");
    return 0;
}
