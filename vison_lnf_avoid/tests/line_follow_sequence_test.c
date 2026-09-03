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
    return line_follow_update(controller, mask, IR_SAMPLE_PERIOD_MS, 0, 0);
}

static line_follow_result_t step_at(line_follow_controller_t *controller,
                                    uint8_t mask,
                                    int left_count,
                                    int right_count)
{
    return line_follow_update(controller, mask, IR_SAMPLE_PERIOD_MS,
                              left_count, right_count);
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
    /* Candidate holds while the armed outer channel stays black, until the
       confirm window elapses (window / sample-period steps). */
    const int hold = LINE_CORNER_CONFIRM_WINDOW_MS / IR_SAMPLE_PERIOD_MS;
    for (int i = 0; i < hold; ++i) {
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
    /* Expire the candidate (window elapsed + 1 step drops back to TRACKING),
       so a following all-white uses the normal lost search, not a corner. */
    const int expire = LINE_CORNER_CONFIRM_WINDOW_MS / IR_SAMPLE_PERIOD_MS + 1;
    for (int i = 0; i < expire; ++i) {
        step(&controller, MASK_RIGHT);
    }
    CHECK(controller.state == LINE_FOLLOW_TRACKING);
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
    const int stop_samples = LINE_ALL_BLACK_STOP_MS / IR_SAMPLE_PERIOD_MS;
    for (int i = 0; i < stop_samples - 1; ++i) {
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

static void enter_lost_search(line_follow_controller_t *controller)
{
    warm_center(controller);
    CHECK(step(controller, MASK_RIGHT).state == LINE_FOLLOW_TRACKING);
    const line_follow_result_t result = step_at(controller, MASK_WHITE, 0, 0);
    CHECK(result.state == LINE_FOLLOW_LOST_SEARCH);
    CHECK(result.forward == 0);
    /* Lost search sweeps RIGHT first (positive turn). */
    CHECK(result.turn == LINE_SEARCH_TURN);
    CHECK(controller->search_phase == LINE_SEARCH_ROTATE);
    CHECK(controller->search_leg == 0);
    CHECK(controller->search_target_counts == LINE_SEARCH_FIRST_COUNTS);
}

static void test_search_always_starts_right_after_left_error(void)
{
    line_follow_controller_t controller;
    warm_center(&controller);
    CHECK(step(&controller, MASK_LEFT).state == LINE_FOLLOW_TRACKING);
    CHECK(controller.last_direction == -1);

    const line_follow_result_t result =
        step_at(&controller, MASK_WHITE, 0, 0);
    CHECK(result.state == LINE_FOLLOW_LOST_SEARCH);
    CHECK(result.turn == LINE_SEARCH_TURN);
    CHECK(controller.search_direction == 1);
}

static void test_search_first_leg_and_settle(void)
{
    line_follow_controller_t controller;
    enter_lost_search(&controller);

    const int first_half = LINE_SEARCH_FIRST_COUNTS / 2;
    line_follow_result_t result =
        step_at(&controller, MASK_WHITE, first_half, -first_half);
    CHECK(result.state == LINE_FOLLOW_LOST_SEARCH);
    CHECK(result.forward == 0);
    CHECK(result.turn == 0);
    CHECK(controller.search_phase == LINE_SEARCH_SETTLE);
    CHECK(controller.search_progress_counts == LINE_SEARCH_FIRST_COUNTS);

    CHECK(step_at(&controller, MASK_WHITE, first_half, -first_half).turn == 0);
    CHECK(step_at(&controller, MASK_WHITE, first_half, -first_half).turn == 0);
    result = step_at(&controller, MASK_WHITE, first_half, -first_half);
    CHECK(result.turn == -LINE_SEARCH_TURN);
    CHECK(controller.search_phase == LINE_SEARCH_ROTATE);
    CHECK(controller.search_leg == 1);
    CHECK(controller.search_target_counts == LINE_SEARCH_SECOND_COUNTS);
}

static void test_search_uses_fine_turn_near_target(void)
{
    line_follow_controller_t controller;
    enter_lost_search(&controller);
    const int progress = LINE_SEARCH_FIRST_COUNTS -
                         LINE_SEARCH_30_DEG_COUNTS / 4 + 1;
    const line_follow_result_t result =
        step_at(&controller, MASK_WHITE, progress, 0);
    CHECK(result.turn == LINE_SEARCH_FINE_TURN);
    CHECK(controller.search_phase == LINE_SEARCH_ROTATE);
}

static void test_search_reacquires_while_rotating_and_settling(void)
{
    line_follow_controller_t controller;
    enter_lost_search(&controller);
    CHECK(step_at(&controller, MASK_CENTER, 20, -20).state == LINE_FOLLOW_TRACKING);
    CHECK(controller.search_phase == LINE_SEARCH_IDLE);

    enter_lost_search(&controller);
    const int first_half = LINE_SEARCH_FIRST_COUNTS / 2;
    CHECK(step_at(&controller, MASK_WHITE, first_half, -first_half).turn == 0);
    CHECK(controller.search_phase == LINE_SEARCH_SETTLE);
    CHECK(step_at(&controller, MASK_CENTER, first_half, -first_half).state ==
          LINE_FOLLOW_TRACKING);
    CHECK(controller.search_phase == LINE_SEARCH_IDLE);
}

static void test_search_accepts_negative_encoder_polarity(void)
{
    line_follow_controller_t controller;
    enter_lost_search(&controller);
    const int first_half = LINE_SEARCH_FIRST_COUNTS / 2;
    const line_follow_result_t result =
        step_at(&controller, MASK_WHITE, -first_half, first_half);
    CHECK(result.turn == 0);
    CHECK(controller.search_progress_counts == LINE_SEARCH_FIRST_COUNTS);
    CHECK(controller.search_phase == LINE_SEARCH_SETTLE);
}

static void test_search_exhausts_two_legs(void)
{
    line_follow_controller_t controller;
    enter_lost_search(&controller);
    int left_count = 0;
    int right_count = 0;

    for (unsigned leg = 0; leg < LINE_SEARCH_MAX_LEGS; ++leg) {
        const int target = leg == 0U ?
                           LINE_SEARCH_FIRST_COUNTS : LINE_SEARCH_SECOND_COUNTS;
        left_count += target / 2;
        right_count -= target - target / 2;
        CHECK(step_at(&controller, MASK_WHITE, left_count, right_count).turn == 0);
        CHECK(controller.search_phase == LINE_SEARCH_SETTLE);
        CHECK(step_at(&controller, MASK_WHITE, left_count, right_count).state ==
              LINE_FOLLOW_LOST_SEARCH);
        CHECK(step_at(&controller, MASK_WHITE, left_count, right_count).state ==
              LINE_FOLLOW_LOST_SEARCH);
        const line_follow_result_t result =
            step_at(&controller, MASK_WHITE, left_count, right_count);
        if (leg + 1U == LINE_SEARCH_MAX_LEGS) {
            CHECK(result.state == LINE_FOLLOW_STOPPED);
        } else {
            CHECK(result.state == LINE_FOLLOW_LOST_SEARCH);
            CHECK(controller.search_leg == leg + 1U);
            CHECK(controller.search_direction == ((leg % 2U) == 0U ? -1 : 1));
        }
    }
}

static void test_search_timeout_stops(void)
{
    line_follow_controller_t controller;
    enter_lost_search(&controller);
    const line_follow_result_t result =
        line_follow_update(&controller, MASK_WHITE, LINE_LOST_STOP_MS, 0, 0);
    CHECK(result.state == LINE_FOLLOW_STOPPED);
}

static void test_search_stall_stops(void)
{
    line_follow_controller_t controller;
    enter_lost_search(&controller);
    line_follow_result_t result = {0};
    for (unsigned elapsed = 0; elapsed < LINE_SEARCH_STALL_MS;
         elapsed += IR_SAMPLE_PERIOD_MS) {
        result = step_at(&controller, MASK_WHITE, 0, 0);
    }
    CHECK(result.state == LINE_FOLLOW_STOPPED);
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
    test_search_always_starts_right_after_left_error();
    test_search_first_leg_and_settle();
    test_search_uses_fine_turn_near_target();
    test_search_reacquires_while_rotating_and_settling();
    test_search_accepts_negative_encoder_polarity();
    test_search_exhausts_two_legs();
    test_search_timeout_stops();
    test_search_stall_stops();
    puts("line_follow sequence tests passed");
    return 0;
}
