#include <stdio.h>
#include <stdlib.h>

#include "board_config.h"
#include "line_follow.h"

#define MASK_CENTER  0x06U
#define MASK_RIGHT   0x03U
#define MASK_LEFT    0x0CU
#define MASK_WHITE   0x00U
#define MASK_BLACK   0x0FU

#define CHECK(condition) do {                                                   \
    if (!(condition)) {                                                         \
        fprintf(stderr, "FAIL %s:%d: %s\n", __func__, __LINE__, #condition);   \
        exit(1);                                                                \
    }                                                                           \
} while (0)

static line_follow_result_t step(line_follow_controller_t *controller, uint8_t mask)
{
    return line_follow_update(controller, mask, IR_SAMPLE_PERIOD_MS);
}

static void warm_center(line_follow_controller_t *controller)
{
    line_follow_init(controller);
    for (int i = 0; i < 3; ++i) {
        CHECK(step(controller, MASK_CENTER).state == LINE_FOLLOW_TRACKING);
    }
    CHECK(controller->corner_rearm_ready);
}

static void enter_rotate(line_follow_controller_t *controller, int direction)
{
    const uint8_t edge = direction > 0 ? MASK_RIGHT : MASK_LEFT;
    for (int i = 0; i < 3; ++i) {
        step(controller, edge);
    }
    CHECK(controller->state == LINE_FOLLOW_CORNER_CANDIDATE);
    const line_follow_result_t result = step(controller, MASK_WHITE);
    CHECK(result.state == LINE_FOLLOW_CORNER_ROTATE);
    CHECK(result.forward == LINE_CORNER_ROTATE_FORWARD);
    CHECK(result.turn == direction * LINE_CORNER_ROTATE_TURN);
}

static void test_short_edge_returns_to_tracking(void)
{
    line_follow_controller_t controller;
    warm_center(&controller);
    CHECK(step(&controller, MASK_RIGHT).state == LINE_FOLLOW_TRACKING);
    CHECK(step(&controller, MASK_CENTER).state == LINE_FOLLOW_TRACKING);
}

static void test_twenty_ms_then_white_uses_normal_search(void)
{
    line_follow_controller_t controller;
    warm_center(&controller);
    step(&controller, MASK_RIGHT);
    step(&controller, MASK_RIGHT);
    CHECK(step(&controller, MASK_WHITE).state == LINE_FOLLOW_LOST_SEARCH);
}

static void test_thirty_ms_then_white_confirms_corner(void)
{
    line_follow_controller_t controller;
    warm_center(&controller);
    enter_rotate(&controller, 1);
}

static void test_candidate_direction_change_cancels(void)
{
    line_follow_controller_t controller;
    warm_center(&controller);
    for (int i = 0; i < 3; ++i) {
        step(&controller, MASK_RIGHT);
    }
    CHECK(step(&controller, MASK_LEFT).state == LINE_FOLLOW_TRACKING);
    CHECK(!controller.corner_rearm_ready);
}

static void test_candidate_timeout_requires_recenter(void)
{
    line_follow_controller_t controller;
    warm_center(&controller);
    for (int i = 0; i < 3; ++i) {
        step(&controller, MASK_RIGHT);
    }
    for (int i = 0; i < 12; ++i) {
        CHECK(step(&controller, MASK_RIGHT).state == LINE_FOLLOW_CORNER_CANDIDATE);
    }
    CHECK(step(&controller, MASK_RIGHT).state == LINE_FOLLOW_TRACKING);
    for (int i = 0; i < 5; ++i) {
        CHECK(step(&controller, MASK_RIGHT).state == LINE_FOLLOW_TRACKING);
    }
}

static void test_white_after_confirm_window_uses_normal_search(void)
{
    line_follow_controller_t controller;
    warm_center(&controller);
    for (int i = 0; i < 3; ++i) {
        step(&controller, MASK_RIGHT);
    }
    for (int i = 0; i < 12; ++i) {
        CHECK(step(&controller, MASK_RIGHT).state == LINE_FOLLOW_CORNER_CANDIDATE);
    }
    CHECK(step(&controller, MASK_WHITE).state == LINE_FOLLOW_LOST_SEARCH);
}

static void test_rotate_obeys_minimum_and_center_time(void)
{
    line_follow_controller_t controller;
    warm_center(&controller);
    enter_rotate(&controller, 1);
    for (int i = 0; i < 4; ++i) {
        CHECK(step(&controller, MASK_CENTER).state == LINE_FOLLOW_CORNER_ROTATE);
    }
    CHECK(step(&controller, MASK_CENTER).state == LINE_FOLLOW_CORNER_EXIT);
}

static void test_rotate_timeout_falls_back_to_search(void)
{
    line_follow_controller_t controller;
    warm_center(&controller);
    enter_rotate(&controller, 1);
    for (int i = 0; i < 58; ++i) {
        CHECK(step(&controller, MASK_WHITE).state == LINE_FOLLOW_CORNER_ROTATE);
    }
    CHECK(step(&controller, MASK_WHITE).state == LINE_FOLLOW_LOST_SEARCH);
}

static void test_all_black_still_stops(void)
{
    line_follow_controller_t controller;
    warm_center(&controller);
    enter_rotate(&controller, 1);
    for (int i = 0; i < 3; ++i) {
        CHECK(step(&controller, MASK_BLACK).state == LINE_FOLLOW_CORNER_ROTATE);
    }
    CHECK(step(&controller, MASK_BLACK).state == LINE_FOLLOW_STOPPED);
}

static void test_consecutive_opposite_corners(void)
{
    line_follow_controller_t controller;
    warm_center(&controller);
    enter_rotate(&controller, 1);
    for (int i = 0; i < 5; ++i) {
        step(&controller, MASK_CENTER);
    }
    CHECK(controller.state == LINE_FOLLOW_CORNER_EXIT);
    for (int i = 0; i < 5; ++i) {
        step(&controller, MASK_CENTER);
    }
    CHECK(controller.state == LINE_FOLLOW_TRACKING);
    enter_rotate(&controller, -1);
}

int main(void)
{
    test_short_edge_returns_to_tracking();
    test_twenty_ms_then_white_uses_normal_search();
    test_thirty_ms_then_white_confirms_corner();
    test_candidate_direction_change_cancels();
    test_candidate_timeout_requires_recenter();
    test_white_after_confirm_window_uses_normal_search();
    test_rotate_obeys_minimum_and_center_time();
    test_rotate_timeout_falls_back_to_search();
    test_all_black_still_stops();
    test_consecutive_opposite_corners();
    puts("line_follow sequence tests passed");
    return 0;
}
