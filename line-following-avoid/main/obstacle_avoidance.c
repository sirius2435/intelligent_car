#include "obstacle_avoidance.h"

#include "board_config.h"
#include "esp_log.h"

static const char *TAG = "obstacle";


/*
 * Safe uint32_t addition.
 */
static uint32_t add_ms(uint32_t value, uint32_t increment)
{
    return UINT32_MAX - value < increment
               ? UINT32_MAX
               : value + increment;
}


/*
 * Enter a new obstacle-avoidance state.
 */
static void enter_state(obstacle_controller_t *controller,
                        obstacle_state_t state)
{
    controller->state = state;
    controller->state_ms = 0;
}


/*
 * Initialize obstacle controller.
 */
void obstacle_avoidance_init(obstacle_controller_t *controller)
{
    if (controller == NULL) {
        return;
    }

    controller->state = OBSTACLE_IDLE;
    controller->state_ms = 0;
    controller->cooldown_ms = 0;
    controller->saw_white = false;
}


/*
 * Return true when obstacle avoidance owns the motors.
 */
bool obstacle_avoidance_active(
    const obstacle_controller_t *controller)
{
    return controller != NULL &&
           controller->state != OBSTACLE_IDLE;
}


/*
 * ============================================================
 * Obstacle avoidance state machine
 * ============================================================
 *
 * IDLE
 *   ↓
 * distance < 5 cm
 *   ↓
 * STOP
 *   ↓
 * SHIFT_LEFT
 *   ↓
 * SETTLE_LEFT
 *   ↓
 * FORWARD
 *   ↓
 * SETTLE_FORWARD
 *   ↓
 * SHIFT_RIGHT
 *   ↓
 * black line reacquired
 *   ↓
 * IDLE
 *
 * ============================================================
 */
esp_err_t obstacle_avoidance_update(
    obstacle_controller_t *controller,
    uint8_t black_mask,
    float distance_cm,
    uint32_t elapsed_ms,
    drive_wheel_command_t *applied,
    bool *finished)
{
    if (controller == NULL ||
        applied == NULL ||
        finished == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * Default outputs.
     */
    *applied = (drive_wheel_command_t){0};
    *finished = false;


    /*
     * --------------------------------------------------------
     * Cooldown timer
     * --------------------------------------------------------
     */
    if (controller->cooldown_ms > 0) {

        if (controller->cooldown_ms > elapsed_ms) {
            controller->cooldown_ms -= elapsed_ms;
        } else {
            controller->cooldown_ms = 0;
        }
    }


    /*
     * --------------------------------------------------------
     * IDLE
     * --------------------------------------------------------
     *
     * Only a valid ultrasonic measurement can trigger
     * obstacle avoidance.
     */
    if (controller->state == OBSTACLE_IDLE) {

        if (controller->cooldown_ms == 0 &&
            distance_cm > 0.0f &&
            distance_cm < (float)ULTRASONIC_OBSTACLE_CM) {

            ESP_LOGW(
                TAG,
                "Obstacle detected: %.1f cm -> starting avoidance",
                (double)distance_cm
            );

            enter_state(
                controller,
                OBSTACLE_STOP
            );

        } else {

            return ESP_OK;
        }
    }


    /*
     * Increase time spent in current state.
     */
    controller->state_ms =
        add_ms(controller->state_ms, elapsed_ms);


    /*
     * ========================================================
     * State machine
     * ========================================================
     */
    switch (controller->state) {

    /*
     * ========================================================
     * STOP
     * ========================================================
     *
     * Give the car a short moment to completely stop before
     * beginning the lateral movement.
     */
    case OBSTACLE_STOP:

        if (controller->state_ms >=
            OBSTACLE_STOP_SETTLE_MS) {

            ESP_LOGI(
                TAG,
                "Obstacle avoidance: SHIFT_LEFT"
            );

            enter_state(
                controller,
                OBSTACLE_SHIFT_LEFT
            );
        }

        return drive_stop();


    /*
     * ========================================================
     * SHIFT LEFT
     * ========================================================
     *
     * Rear wheel is now enabled.
     *
     * Front wheels = 1/2 rear wheel.
     */
    case OBSTACLE_SHIFT_LEFT:

        if (controller->state_ms >=
            OBSTACLE_LEFT_SHIFT_MS) {

            ESP_LOGI(
                TAG,
                "Obstacle avoidance: LEFT SHIFT complete"
            );

            enter_state(
                controller,
                OBSTACLE_SETTLE_AFTER_LEFT
            );

            return drive_stop();
        }

        /*
         * -1 means LEFT.
         */
        return drive_set_lateral(
            -1,
            OBSTACLE_SHIFT_SPEED,
            applied
        );


    /*
     * ========================================================
     * SETTLE AFTER LEFT
     * ========================================================
     */
    case OBSTACLE_SETTLE_AFTER_LEFT:

        if (controller->state_ms >=
            OBSTACLE_SHIFT_SETTLE_MS) {

            ESP_LOGI(
                TAG,
                "Obstacle avoidance: FORWARD"
            );

            enter_state(
                controller,
                OBSTACLE_FORWARD
            );
        }

        return drive_stop();


    /*
     * ========================================================
     * FORWARD
     * ========================================================
     *
     * The car now travels approximately 10 cm forward.
     *
     * Normal two-front-wheel drive is intentionally used here.
     * The rear wheel becomes passive again.
     */
    case OBSTACLE_FORWARD:

        if (controller->state_ms >=
            OBSTACLE_FORWARD_MS) {

            ESP_LOGI(
                TAG,
                "Obstacle avoidance: FORWARD complete"
            );

            enter_state(
                controller,
                OBSTACLE_SETTLE_AFTER_FORWARD
            );

            return drive_stop();
        }

        return drive_set_motion(
            OBSTACLE_FORWARD_SPEED,
            0,
            applied
        );


    /*
     * ========================================================
     * SETTLE AFTER FORWARD
     * ========================================================
     */
    case OBSTACLE_SETTLE_AFTER_FORWARD:

        if (controller->state_ms >=
            OBSTACLE_SHIFT_SETTLE_MS) {

            /*
             * We deliberately require the sensor to see WHITE
             * before accepting a BLACK line during the final
             * right shift.
             */
            controller->saw_white =
                (black_mask == 0);

            ESP_LOGI(
                TAG,
                "Obstacle avoidance: SHIFT_RIGHT"
            );

            enter_state(
                controller,
                OBSTACLE_SHIFT_RIGHT
            );
        }

        return drive_stop();


    /*
     * ========================================================
     * SHIFT RIGHT
     * ========================================================
     *
     * Keep moving right until the infrared sensor finds the
     * black line again.
     */
    case OBSTACLE_SHIFT_RIGHT:

        /*
         * First detect a completely white sensor pattern.
         */
        if (black_mask == 0) {
            controller->saw_white = true;
        }


        /*
         * Then require a black-line detection.
         *
         * This prevents an already-detected line from causing
         * immediate termination.
         */
        if (controller->saw_white &&
            black_mask != 0) {

            ESP_LOGI(
                TAG,
                "Line reacquired -> returning to line following"
            );

            /*
             * Stop the lateral movement first.
             */
            drive_stop();

            enter_state(
                controller,
                OBSTACLE_IDLE
            );

            controller->cooldown_ms =
                OBSTACLE_REARM_MS;

            *finished = true;

            return ESP_OK;
        }


        /*
         * Safety timeout.
         *
         * If the car cannot find the line within the allowed
         * time, stop rather than driving indefinitely.
         */
        if (controller->state_ms >=
            OBSTACLE_RIGHT_SHIFT_TIMEOUT_MS) {

            ESP_LOGW(
                TAG,
                "Right shift timeout: line not reacquired"
            );

            drive_stop();

            enter_state(
                controller,
                OBSTACLE_IDLE
            );

            controller->cooldown_ms =
                OBSTACLE_REARM_MS;

            *finished = true;

            return ESP_OK;
        }


        /*
         * +1 means RIGHT.
         */
        return drive_set_lateral(
            +1,
            OBSTACLE_SHIFT_SPEED,
            applied
        );


    /*
     * ========================================================
     * IDLE / DEFAULT
     * ========================================================
     */
    case OBSTACLE_IDLE:
    default:

        return drive_stop();
    }
}


/*
 * Human-readable state names used by the serial log.
 */
const char *obstacle_avoidance_state_name(
    obstacle_state_t state)
{
    switch (state) {

    case OBSTACLE_IDLE:
        return "IDLE";

    case OBSTACLE_STOP:
        return "STOP";

    case OBSTACLE_SHIFT_LEFT:
        return "SHIFT_LEFT";

    case OBSTACLE_SETTLE_AFTER_LEFT:
        return "SETTLE_LEFT";

    case OBSTACLE_FORWARD:
        return "FORWARD";

    case OBSTACLE_SETTLE_AFTER_FORWARD:
        return "SETTLE_FORWARD";

    case OBSTACLE_SHIFT_RIGHT:
        return "SHIFT_RIGHT";

    default:
        return "UNKNOWN";
    }
}