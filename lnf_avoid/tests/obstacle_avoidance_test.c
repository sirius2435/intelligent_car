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

static obstacle_avoidance_result_t step(
    obstacle_avoidance_controller_t *controller,
    ultrasonic_reading_t *range,
    uint8_t mask,
    uint32_t elapsed_ms,
    int left,
    int right,
    int rear)
{
    return obstacle_avoidance_update(controller, range, mask, elapsed_ms,
                                     left, right, rear);
}

static void trigger_obstacle(obstacle_avoidance_controller_t *controller,
                             ultrasonic_reading_t *range)
{
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
        step(&controller, &range, 0, 10, 100, 100, 100);
    }
    CHECK(controller.state == AVOIDANCE_STRAFE_LEFT);
    CHECK(controller.left_edge_confirmed);
    for (unsigned elapsed = 10; elapsed < AVOID_LEFT_CLEARANCE_MS;
         elapsed += 10) {
        step(&controller, &range, 0, 10, 100, 100, 100);
    }
    CHECK(controller.state == AVOIDANCE_FORWARD_PASS);
    CHECK(controller.outbound_lateral_counts == 300);

    obstacle_avoidance_result_t result =
        step(&controller, &range, 0, 10, 900, 100, 100);
    CHECK(result.state == AVOIDANCE_FORWARD_PASS);
    result = step(&controller, &range, 0, 10, 900, 800, 100);
    CHECK(result.state == AVOIDANCE_STRAFE_RIGHT_FIND_LINE);

    result = step(&controller, &range, 0x06, 10, 1100, 1000, 200);
    CHECK(result.active);
    result = step(&controller, &range, 0x06, 10, 1100, 1000, 200);
    CHECK(result.active);
    result = step(&controller, &range, 0x06, 10, 1100, 1000, 200);
    CHECK(result.just_completed);
    CHECK(!result.active);
    CHECK(result.state == AVOIDANCE_COMPLETE);

    range.status = ULTRASONIC_READING_VALID;
    range.distance_mm = 20;
    range.sequence = 6;
    result = step(&controller, &range, 0x06, 10, 1100, 1000, 200);
    CHECK(result.state == AVOIDANCE_COMPLETE);
    CHECK(!result.active);
}

static void test_stale_sensor_faults_during_left_strafe(void)
{
    obstacle_avoidance_controller_t controller;
    ultrasonic_reading_t range;
    enter_left_strafe(&controller, &range);
    const obstacle_avoidance_result_t result =
        step(&controller, &range, 0, AVOID_SENSOR_STALE_MS, 100, 100, 100);
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
