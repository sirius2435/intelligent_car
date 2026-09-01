#include <stdio.h>
#include <stdlib.h>

#include "board_config.h"
#include "obstacle_avoidance.h"

#define CHECK(condition) do {                                                   \
    if (!(condition)) {                                                         \
        fprintf(stderr, "FAIL %s:%d: %s\n", __func__, __LINE__, #condition);   \
        exit(1);                                                                \
    }                                                                           \
} while (0)

static uint32_t s_vision_sequence;

static vision_result_t vision_for_mask(uint8_t mask)
{
    const bool line_found = mask != 0U;
    return (vision_result_t) {
        .frame_valid = true,
        .line_found = line_found,
        .confidence = line_found ? 1000U : 0U,
        .sequence = ++s_vision_sequence,
    };
}

static obstacle_avoidance_result_t step(
    obstacle_avoidance_controller_t *controller,
    ultrasonic_reading_t *range,
    uint8_t mask,
    uint32_t elapsed_ms,
    int left,
    int right,
    int rear)
{
    const vision_result_t vision = vision_for_mask(mask);
    return obstacle_avoidance_update(controller, range, &vision, elapsed_ms,
                                     left, right, rear);
}

static void trigger_obstacle(obstacle_avoidance_controller_t *controller,
                             ultrasonic_reading_t *range)
{
    s_vision_sequence = 0;
    obstacle_avoidance_init(controller);
    *range = (ultrasonic_reading_t) {
        .status = ULTRASONIC_READING_VALID,
        .distance_mm = 45,
        .sequence = 1,
    };
    CHECK(!step(controller, range, 0x06, 10, 0, 0, 0).active);
    range->sequence = 2;
    const obstacle_avoidance_result_t result =
        step(controller, range, 0x06, 10, 0, 0, 0);
    CHECK(result.active);
    CHECK(result.state == AVOIDANCE_BRAKE);
}

static void enter_left_strafe(obstacle_avoidance_controller_t *controller,
                              ultrasonic_reading_t *range)
{
    trigger_obstacle(controller, range);
    for (unsigned i = 0; i < AVOID_BRAKE_MS / 10; ++i) {
        step(controller, range, 0x06, 10, 0, 0, 0);
    }
    CHECK(controller->state == AVOIDANCE_REVERSE);
    const obstacle_avoidance_result_t reversing =
        step(controller, range, 0x06, 10,
             AVOID_REVERSE_COUNTS / 2, AVOID_REVERSE_COUNTS / 2, 0);
    CHECK(reversing.state == AVOIDANCE_STRAFE_LEFT);
    CHECK(controller->state == AVOIDANCE_STRAFE_LEFT);
}

static void test_slow_zone_and_confirmation(void)
{
    obstacle_avoidance_controller_t controller;
    obstacle_avoidance_init(&controller);
    ultrasonic_reading_t range = {
        .status = ULTRASONIC_READING_VALID,
        .distance_mm = 80,
        .sequence = 1,
    };
    obstacle_avoidance_result_t result =
        step(&controller, &range, 0x06, 10, 0, 0, 0);
    CHECK(!result.active);
    CHECK(result.tracking_forward_limit == AVOID_SLOW_FORWARD);
    CHECK(result.slow_approach);

    range.distance_mm = 45;
    range.sequence = 2;
    CHECK(!step(&controller, &range, 0x06, 10, 0, 0, 0).active);
    range.distance_mm = 150;
    range.sequence = 3;
    CHECK(!step(&controller, &range, 0x06, 10, 0, 0, 0).active);
    CHECK(controller.state == AVOIDANCE_ARMED);
}

static void test_complete_sequence(void)
{
    obstacle_avoidance_controller_t controller;
    ultrasonic_reading_t range;
    enter_left_strafe(&controller, &range);

    for (uint32_t sequence = 3; sequence <= 5; ++sequence) {
        range.status = ULTRASONIC_READING_NO_ECHO;
        range.sequence = sequence;
        step(&controller, &range, 0, 10, 250, 250, 100);
    }
    CHECK(controller.state == AVOIDANCE_STRAFE_LEFT);
    CHECK(controller.left_edge_confirmed);
    for (unsigned elapsed = 10; elapsed < AVOID_LEFT_CLEARANCE_MS;
         elapsed += 10) {
        step(&controller, &range, 0, 10, 250, 250, 100);
    }
    CHECK(controller.state == AVOIDANCE_FORWARD_PASS);
    CHECK(controller.outbound_lateral_counts == 300);

    obstacle_avoidance_result_t result =
        step(&controller, &range, 0, 10, 750, 500, 100);
    CHECK(result.state == AVOIDANCE_FORWARD_PASS);
    result = step(&controller, &range, 0, 10, 750, 750, 100);
    CHECK(result.state == AVOIDANCE_STRAFE_RIGHT_FIND_LINE);

    for (unsigned frame = 0; frame < VISION_REACQUIRE_FRAMES; ++frame) {
        result = step(&controller, &range, 0x06, 10, 800, 800, 150);
        CHECK(result.active);
    }
    CHECK(result.state == AVOIDANCE_STRAFE_RIGHT_ALIGN_LINE);

    for (unsigned frame = 0; frame < VISION_CENTERED_FRAMES; ++frame) {
        result = step(&controller, &range, 0x06, 10, 800, 800, 150);
    }
    CHECK(result.just_completed);
    CHECK(!result.active);
    CHECK(result.state == AVOIDANCE_COMPLETE);

    range.status = ULTRASONIC_READING_VALID;
    range.distance_mm = 20;
    range.sequence = 6;
    result = step(&controller, &range, 0x06, 10, 800, 800, 150);
    CHECK(result.state == AVOIDANCE_COMPLETE);
    CHECK(!result.active);
}

static void test_stale_sensor_faults_during_left_strafe(void)
{
    obstacle_avoidance_controller_t controller;
    ultrasonic_reading_t range;
    enter_left_strafe(&controller, &range);
    const obstacle_avoidance_result_t result =
        step(&controller, &range, 0, AVOID_SENSOR_STALE_MS, 250, 250, 100);
    CHECK(result.state == AVOIDANCE_FAULT_STOP);
    CHECK(result.active);
    CHECK(result.forward == 0 && result.lateral == 0 && result.turn == 0);
}

int main(void)
{
    test_slow_zone_and_confirmation();
    test_complete_sequence();
    test_stale_sensor_faults_during_left_strafe();
    puts("obstacle avoidance tests passed");
    return 0;
}
