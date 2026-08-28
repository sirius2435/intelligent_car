#include "obstacle_avoidance.h"

#include <limits.h>

#include "board_config.h"
#include "infrared_sensor.h"

static uint32_t add_saturated(uint32_t value, uint32_t increment)
{
    return UINT32_MAX - value < increment ? UINT32_MAX : value + increment;
}

static int64_t abs_delta(int current, int start)
{
    const int64_t delta = (int64_t)current - start;
    return delta < 0 ? -delta : delta;
}

static int64_t all_wheel_progress(const obstacle_avoidance_controller_t *controller,
                                  int left_count,
                                  int right_count,
                                  int rear_count)
{
    return abs_delta(left_count, controller->start_left_count) +
           abs_delta(right_count, controller->start_right_count) +
           abs_delta(rear_count, controller->start_rear_count);
}

static int64_t forward_progress(const obstacle_avoidance_controller_t *controller,
                                int left_count,
                                int right_count)
{
    return abs_delta(left_count, controller->start_left_count) +
           abs_delta(right_count, controller->start_right_count);
}

static bool line_is_centered(uint8_t mask)
{
    mask &= IR_ALL_BLACK_MASK;
    return mask == IR_CHANNEL_3_MASK ||
           mask == IR_CHANNEL_2_MASK ||
           mask == (IR_CHANNEL_3_MASK | IR_CHANNEL_2_MASK) ||
           mask == IR_ALL_BLACK_MASK;
}

static void begin_motion_stage(obstacle_avoidance_controller_t *controller,
                               obstacle_avoidance_state_t state,
                               int left_count,
                               int right_count,
                               int rear_count)
{
    controller->state = state;
    controller->stage_ms = 0;
    controller->start_left_count = left_count;
    controller->start_right_count = right_count;
    controller->start_rear_count = rear_count;
    controller->progress_counts = 0;
    controller->last_motion_counts = 0;
    controller->stall_ms = 0;
    controller->centered_ms = 0;
}

static bool motion_failed(obstacle_avoidance_controller_t *controller,
                          uint32_t elapsed_ms,
                          int64_t maximum_counts)
{
    if (controller->progress_counts - controller->last_motion_counts >=
        AVOID_STALL_MIN_COUNTS) {
        controller->last_motion_counts = controller->progress_counts;
        controller->stall_ms = 0;
    } else {
        controller->stall_ms = add_saturated(controller->stall_ms, elapsed_ms);
    }
    return controller->stage_ms >= AVOID_MOTION_TIMEOUT_MS ||
           controller->stall_ms >= AVOID_STALL_TIMEOUT_MS ||
           (maximum_counts > 0 && controller->progress_counts > maximum_counts);
}

static obstacle_avoidance_result_t result_for(
    const obstacle_avoidance_controller_t *controller,
    obstacle_avoidance_state_t old_state)
{
    obstacle_avoidance_result_t result = {
        .state = controller->state,
        .tracking_forward_limit = 1000,
        .active = controller->state != AVOIDANCE_ARMED &&
                  controller->state != AVOIDANCE_COMPLETE,
        .state_changed = old_state != controller->state,
    };
    switch (controller->state) {
    case AVOIDANCE_STRAFE_LEFT:
        result.lateral = AVOID_LATERAL_SPEED;
        break;
    case AVOIDANCE_FORWARD_PASS:
        result.forward = AVOID_FORWARD_SPEED;
        break;
    case AVOIDANCE_STRAFE_RIGHT_FIND_LINE:
        result.lateral = -AVOID_LATERAL_SPEED;
        break;
    default:
        break;
    }
    return result;
}

static obstacle_avoidance_result_t fault_result(
    obstacle_avoidance_controller_t *controller,
    obstacle_avoidance_state_t old_state)
{
    controller->state = AVOIDANCE_FAULT_STOP;
    obstacle_avoidance_result_t result = result_for(controller, old_state);
    result.active = true;
    result.forward = 0;
    result.lateral = 0;
    result.turn = 0;
    return result;
}

void obstacle_avoidance_init(obstacle_avoidance_controller_t *controller)
{
    if (controller == NULL) {
        return;
    }
    *controller = (obstacle_avoidance_controller_t) {
        .state = AVOIDANCE_ARMED,
    };
}

obstacle_avoidance_result_t obstacle_avoidance_update(
    obstacle_avoidance_controller_t *controller,
    const ultrasonic_reading_t *ultrasonic,
    uint8_t infrared_black_mask,
    uint32_t elapsed_ms,
    int left_count,
    int right_count,
    int rear_count)
{
    if (controller == NULL || ultrasonic == NULL) {
        obstacle_avoidance_result_t invalid = {
            .state = AVOIDANCE_FAULT_STOP,
            .active = true,
            .tracking_forward_limit = 1000,
        };
        return invalid;
    }

    const obstacle_avoidance_state_t old_state = controller->state;
    if (controller->state == AVOIDANCE_COMPLETE) {
        return result_for(controller, old_state);
    }
    if (controller->state == AVOIDANCE_FAULT_STOP) {
        return fault_result(controller, old_state);
    }

    const bool new_ultrasonic =
        ultrasonic->sequence != 0 &&
        ultrasonic->sequence != controller->last_ultrasonic_sequence;
    if (new_ultrasonic) {
        controller->last_ultrasonic_sequence = ultrasonic->sequence;
        controller->sensor_silence_ms = 0;
    } else {
        controller->sensor_silence_ms =
            add_saturated(controller->sensor_silence_ms, elapsed_ms);
    }

    if (controller->state == AVOIDANCE_ARMED) {
        obstacle_avoidance_result_t result = result_for(controller, old_state);
        if (ultrasonic->status == ULTRASONIC_READING_VALID &&
            ultrasonic->distance_mm <= AVOID_SLOW_DISTANCE_MM) {
            result.tracking_forward_limit = AVOID_SLOW_FORWARD;
            result.slow_approach = true;
        }
        if (new_ultrasonic) {
            if (ultrasonic->status == ULTRASONIC_READING_VALID &&
                ultrasonic->distance_mm <= AVOID_TRIGGER_DISTANCE_MM) {
                ++controller->trigger_confirm_count;
            } else {
                controller->trigger_confirm_count = 0;
            }
        }
        if (controller->trigger_confirm_count >= AVOID_TRIGGER_CONFIRM_SAMPLES) {
            controller->state = AVOIDANCE_BRAKE;
            controller->stage_ms = 0;
            result = result_for(controller, old_state);
        }
        return result;
    }

    controller->stage_ms = add_saturated(controller->stage_ms, elapsed_ms);
    if (controller->state == AVOIDANCE_BRAKE) {
        if (controller->stage_ms >= AVOID_BRAKE_MS) {
            begin_motion_stage(controller, AVOIDANCE_STRAFE_LEFT,
                               left_count, right_count, rear_count);
        }
        return result_for(controller, old_state);
    }

    if (controller->state == AVOIDANCE_STRAFE_LEFT) {
        controller->progress_counts =
            all_wheel_progress(controller, left_count, right_count, rear_count);
        if (!controller->left_edge_confirmed && new_ultrasonic &&
            controller->progress_counts >= AVOID_LEFT_MIN_COUNTS) {
            const bool clear =
                ultrasonic->status == ULTRASONIC_READING_NO_ECHO ||
                (ultrasonic->status == ULTRASONIC_READING_VALID &&
                 ultrasonic->distance_mm >= AVOID_CLEAR_DISTANCE_MM);
            controller->clear_confirm_count =
                clear ? controller->clear_confirm_count + 1U : 0U;
        }
        if (!controller->left_edge_confirmed &&
            controller->clear_confirm_count >= AVOID_CLEAR_CONFIRM_SAMPLES) {
            controller->left_edge_confirmed = true;
            controller->left_clearance_ms = 0;
        }
        if (controller->left_edge_confirmed) {
            controller->left_clearance_ms =
                add_saturated(controller->left_clearance_ms, elapsed_ms);
        }
        if (controller->left_edge_confirmed &&
            controller->left_clearance_ms >= AVOID_LEFT_CLEARANCE_MS) {
            controller->outbound_lateral_counts = controller->progress_counts;
            begin_motion_stage(controller, AVOIDANCE_FORWARD_PASS,
                               left_count, right_count, rear_count);
            return result_for(controller, old_state);
        }
        if (controller->sensor_silence_ms >= AVOID_SENSOR_STALE_MS ||
            motion_failed(controller, elapsed_ms, AVOID_LEFT_MAX_COUNTS)) {
            return fault_result(controller, old_state);
        }
        return result_for(controller, old_state);
    }

    if (controller->state == AVOIDANCE_FORWARD_PASS) {
        controller->progress_counts =
            forward_progress(controller, left_count, right_count);
        const int64_t left_progress =
            abs_delta(left_count, controller->start_left_count);
        const int64_t right_progress =
            abs_delta(right_count, controller->start_right_count);
        const int64_t target_per_wheel = AVOID_FORWARD_TARGET_COUNTS / 2;
        if (left_progress >= target_per_wheel &&
            right_progress >= target_per_wheel) {
            begin_motion_stage(controller, AVOIDANCE_STRAFE_RIGHT_FIND_LINE,
                               left_count, right_count, rear_count);
            return result_for(controller, old_state);
        }
        if (motion_failed(controller, elapsed_ms,
                          AVOID_FORWARD_TARGET_COUNTS + AVOID_RIGHT_EXTRA_COUNTS)) {
            return fault_result(controller, old_state);
        }
        return result_for(controller, old_state);
    }

    controller->progress_counts =
        all_wheel_progress(controller, left_count, right_count, rear_count);
    const int64_t minimum_return = controller->outbound_lateral_counts / 2;
    if (controller->progress_counts >= minimum_return &&
        line_is_centered(infrared_black_mask)) {
        controller->centered_ms =
            add_saturated(controller->centered_ms, elapsed_ms);
    } else {
        controller->centered_ms = 0;
    }
    if (controller->centered_ms >= AVOID_LINE_CENTERED_MS) {
        controller->state = AVOIDANCE_COMPLETE;
        obstacle_avoidance_result_t result = result_for(controller, old_state);
        result.just_completed = true;
        return result;
    }
    if (motion_failed(controller, elapsed_ms,
                      controller->outbound_lateral_counts +
                      AVOID_RIGHT_EXTRA_COUNTS)) {
        return fault_result(controller, old_state);
    }
    return result_for(controller, old_state);
}

const char *obstacle_avoidance_state_name(obstacle_avoidance_state_t state)
{
    switch (state) {
    case AVOIDANCE_ARMED:
        return "ARMED";
    case AVOIDANCE_BRAKE:
        return "BRAKE";
    case AVOIDANCE_STRAFE_LEFT:
        return "STRAFE_LEFT";
    case AVOIDANCE_FORWARD_PASS:
        return "FORWARD_PASS";
    case AVOIDANCE_STRAFE_RIGHT_FIND_LINE:
        return "STRAFE_RIGHT_FIND_LINE";
    case AVOIDANCE_COMPLETE:
        return "COMPLETE";
    case AVOIDANCE_FAULT_STOP:
        return "FAULT_STOP";
    default:
        return "UNKNOWN";
    }
}
