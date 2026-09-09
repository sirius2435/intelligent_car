#include "board_config.h"
#include "ball_push.h"
#include "camera_gimbal.h"
#include "camera_stream.h"
#include "camera_vision.h"
#include "drive.h"
#include "encoder.h"
#include "lcd_monitor.h"
#include "line_follow.h"
#include "obstacle_avoidance.h"
#include "pseudo_infrared.h"
#include "ultrasonic.h"

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "vision_car";

/* Run phases: LINE is the original track task (line following + obstacle
 * avoidance, fed by the pseudo-infrared mask). When the line controller
 * stops on a confirmed FINISH bar the car hands over to BALL: the camera
 * pipeline switches to the push grid and ball_push drives everything.
 * Obstacle avoidance is structurally off in BALL (its update is never
 * called), because the ultrasonic would read the balls/table as obstacles. */
typedef enum {
    RUN_PHASE_LINE = 0,
    RUN_PHASE_BALL,
} run_phase_t;

static void lock_stop(const char *reason)
{
    ESP_LOGE(TAG, "locked stop: %s", reason);
    ESP_ERROR_CHECK_WITHOUT_ABORT(drive_stop());
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void require_ok(const char *operation, esp_err_t result)
{
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "%s failed: %s", operation, esp_err_to_name(result));
        lock_stop(operation);
    }
}

static void wait_for_first_camera_frame(void)
{
    const int64_t deadline = esp_timer_get_time() +
        (int64_t)CAMERA_CONNECT_TIMEOUT_MS * 1000LL;
    while (esp_timer_get_time() < deadline) {
        camera_vision_status_t camera = {0};
        if (camera_vision_get_status(&camera) == ESP_OK &&
            camera.connected && camera.image_width > 0U) {
            ESP_LOGI(TAG, "camera ready after %u frames",
                     (unsigned)camera.received_frames);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    lock_stop("camera did not provide a valid MJPEG frame");
}

static void enter_push_phase(run_phase_t *phase,
                             ball_push_controller_t *push,
                             int left_count,
                             int right_count,
                             int rear_count)
{
    ESP_LOGW(TAG, "finish bar confirmed; starting pocket-push task "
             "(enc=[%d,%d,%d])", left_count, right_count, rear_count);
    require_ok("camera mode push", camera_vision_set_mode(
                   CAMERA_VISION_MODE_PUSH));

    /* One-shot gimbal tilt so the balls and pockets sit in the frame at the
     * hand-over spot. The angle is a board_config.h calibration constant. */
    if (camera_gimbal_enabled()) {
        const esp_err_t tilt_result =
            camera_gimbal_set_tilt(BALL_GIMBAL_TILT_DEG);
        if (tilt_result != ESP_OK) {
            ESP_LOGW(TAG, "ball gimbal tilt failed: %s",
                     esp_err_to_name(tilt_result));
        } else {
            ESP_LOGI(TAG, "ball gimbal tilt -> %d deg, settling %d ms",
                     (int)BALL_GIMBAL_TILT_DEG, (int)BALL_GIMBAL_SETTLE_MS);
            vTaskDelay(pdMS_TO_TICKS(BALL_GIMBAL_SETTLE_MS));
        }
    }

    ball_push_init(push);
    *phase = RUN_PHASE_BALL;
}

void app_main(void)
{
    require_ok("drive_init", drive_init());
    require_ok("encoder_init", encoder_init());
    require_ok("ultrasonic_init", ultrasonic_init());

    const esp_err_t lcd_result = lcd_monitor_start();
    if (lcd_result != ESP_OK) {
        ESP_LOGW(TAG, "LCD disabled: %s", esp_err_to_name(lcd_result));
    }

    /* Wi-Fi viewer (PC / phone): started before the camera so /status still
     * reports why no frame arrives if the camera later fails. Non-fatal. */
    const esp_err_t stream_result = camera_stream_start();
    if (stream_result != ESP_OK) {
        ESP_LOGW(TAG, "camera stream disabled: %s",
                 esp_err_to_name(stream_result));
    }

    /* Camera gimbal (two MG90S pan/tilt). Disabled while the servo GPIOs are
     * -1; then init() only logs and the camera stays fixed. Non-fatal. */
    const esp_err_t gimbal_result = camera_gimbal_init();
    if (gimbal_result != ESP_OK) {
        ESP_LOGW(TAG, "camera gimbal unavailable: %s",
                 esp_err_to_name(gimbal_result));
    }

    require_ok("camera_vision_start", camera_vision_start());
    wait_for_first_camera_frame();

    ESP_LOGW(TAG, "line following starts in %d ms; infrared board is "
             "disconnected, vision is converted to a pseudo-infrared mask",
             LINE_START_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(LINE_START_DELAY_MS));

    line_follow_controller_t controller;
    line_follow_init(&controller);
    obstacle_avoidance_controller_t avoidance;
    obstacle_avoidance_init(&avoidance);

    ball_push_controller_t push;
    run_phase_t phase = RUN_PHASE_LINE;
    bool push_start_pending = false;
    bool done_logged = false;
    bool fault_logged = false;
    ball_push_state_t last_push_state = BALL_PUSH_START_FORWARD;

    TickType_t wake_time = xTaskGetTickCount();
    TickType_t previous_ticks = wake_time;
    uint32_t log_elapsed_ms = 0;
    bool locked = false;

    while (1) {
        const TickType_t now_ticks = xTaskGetTickCount();
        uint32_t elapsed_ms =
            (uint32_t)((now_ticks - previous_ticks) * portTICK_PERIOD_MS);
        previous_ticks = now_ticks;
        if (elapsed_ms == 0U) {
            elapsed_ms = CONTROL_PERIOD_MS;
        }

        int left_count = 0;
        int right_count = 0;
        int rear_count = 0;
        require_ok("encoder_get_all",
                   encoder_get_all(&left_count, &right_count, &rear_count));

        camera_vision_status_t camera = {0};
        require_ok("camera_vision_get_status",
                   camera_vision_get_status(&camera));
        const int64_t frame_age_ms = camera.last_frame_us == 0 ? INT64_MAX :
            (esp_timer_get_time() - camera.last_frame_us) / 1000LL;
        if (!camera.connected || frame_age_ms > CAMERA_FRAME_STALE_MS) {
            if (!locked) {
                ESP_LOGE(TAG, "camera unavailable: connected=%d age=%lldms",
                         camera.connected, (long long)frame_age_ms);
            }
            locked = true;
        }

        drive_wheel_command_t wheels = {0};
        esp_err_t drive_result = ESP_OK;

        if (!locked && phase == RUN_PHASE_LINE) {
            ultrasonic_reading_t ultrasonic = {0};
            require_ok("ultrasonic_get_latest",
                       ultrasonic_get_latest(&ultrasonic));

            infrared_sensor_state_t sensor = camera.infrared;

            obstacle_avoidance_result_t avoid = obstacle_avoidance_update(
                &avoidance, &ultrasonic, sensor.black_mask, elapsed_ms,
                left_count, right_count, rear_count);
            if (avoid.just_completed) {
                line_follow_init(&controller);
            }

            int forward = 0;
            int lateral = 0;
            int turn = 0;
            line_follow_result_t control = {
                .state = controller.state,
            };
            if (avoid.active) {
                forward = avoid.forward;
                lateral = avoid.lateral;
                turn = avoid.turn;
            } else {
                control = line_follow_update(&controller, sensor.black_mask,
                                             elapsed_ms, left_count,
                                             right_count);
                forward = control.forward;
                turn = control.turn;
                if (forward > avoid.tracking_forward_limit) {
                    forward = avoid.tracking_forward_limit;
                }
                if (control.state == LINE_FOLLOW_STOPPED) {
                    /* The shot starts from a fixed advance off the END bar, so
                     * it still needs the repeatable pose that only a confirmed
                     * END marker provides. */
                    if (!sensor.finish_detected) {
                        ESP_LOGE(TAG, "line stopped without END confirmation; ball shot inhibited");
                        locked = true;
                    } else {
                        ESP_LOGI(TAG, "END confirmed; starting vision-aligned ball shot");
                        push_start_pending = true;
                    }
                    forward = 0;
                    turn = 0;
                }
            }
            if (avoid.state == AVOIDANCE_FAULT_STOP) {
                locked = true;
                forward = 0;
                lateral = 0;
                turn = 0;
            }

            if (locked) {
                drive_result = drive_stop();
            } else if (push_start_pending) {
                /* Zero command this tick, then switch phase below. */
                drive_result = drive_set_motion(0, 0, 0, &wheels);
            } else if (avoid.active &&
                       avoid.state == AVOIDANCE_FORWARD_PASS) {
                drive_result = drive_set_forward_feedback(
                    forward, elapsed_ms, left_count, right_count, &wheels);
            } else if (!avoid.active && avoid.slow_approach &&
                       forward > 0) {
                drive_result = drive_set_approach_feedback(
                    forward, turn, elapsed_ms, left_count, right_count,
                    &wheels);
            } else {
                drive_result = drive_set_motion_feedback(
                    forward, lateral, turn, elapsed_ms,
                    left_count, right_count, rear_count, &wheels);
            }
            require_ok("drive command", drive_result);

            if (push_start_pending) {
                push_start_pending = false;
                enter_push_phase(&phase, &push, left_count, right_count,
                                 rear_count);
                last_push_state = push.state;
                /* enter_push_phase() blocks for BALL_GIMBAL_SETTLE_MS while
                 * the motors are stopped. Re-anchor the control-loop clock so
                 * the START_FORWARD timer does not count that stopped settling
                 * time as motion time; otherwise the 650 ms startup drive is
                 * consumed before the wheels ever turn. */
                previous_ticks = xTaskGetTickCount();
                wake_time = xTaskGetTickCount();
                log_elapsed_ms = 0;
            }

            log_elapsed_ms += elapsed_ms;
            if (log_elapsed_ms >= LINE_LOG_PERIOD_MS) {
                char display[5];
                pseudo_infrared_format(sensor.black_mask, display);
                ESP_LOGI(TAG,
                         "follow=%s avoid=%s lock=%d sensor=%s motion=[%d,%d,%d] "
                         "cam=[%ux%u age=%lldms frames=%u drop=%u dec=%u] "
                         "range=[%s,%umm] enc=[%d,%d,%d]",
                         line_follow_state_name(control.state),
                         obstacle_avoidance_state_name(avoid.state), locked,
                         display, forward, lateral, turn,
                         (unsigned)camera.image_width,
                         (unsigned)camera.image_height,
                         (long long)frame_age_ms,
                         (unsigned)camera.received_frames,
                         (unsigned)camera.dropped_frames,
                         (unsigned)camera.decode_failures,
                         ultrasonic_status_name(ultrasonic.status),
                         (unsigned)ultrasonic.distance_mm,
                         left_count, right_count, rear_count);
                log_elapsed_ms = 0;
            }
        } else if (!locked && phase == RUN_PHASE_BALL) {
            ball_vision_result_t ball_vision = {0};
            (void)camera_vision_get_ball_result(&ball_vision);
            const bool vision_fresh =
                ball_vision.valid &&
                frame_age_ms <= BALL_VISION_FRESH_MAX_MS;

            const ball_push_result_t push_result = ball_push_update(
                &push, &ball_vision, vision_fresh, elapsed_ms,
                left_count, right_count, rear_count);

            if (push_result.state_changed) {
                ESP_LOGI(TAG, "push %s -> %s (attempt=%u retry=%u)",
                         ball_push_state_name(last_push_state),
                         ball_push_state_name(push_result.state),
                         push_result.attempt, push_result.retry);
                last_push_state = push_result.state;
            }
            if (push_result.state == BALL_PUSH_DONE && !done_logged) {
                done_logged = true;
                ESP_LOGW(TAG, "pocket-push task COMPLETE: both balls pocketed");
            } else if (push_result.state == BALL_PUSH_FAULT_STOP &&
                       !fault_logged) {
                fault_logged = true;
                ESP_LOGE(TAG, "pocket-push task FAULT (attempt=%u retry=%u)",
                         push_result.attempt, push_result.retry);
            }
            if (push_result.state == BALL_PUSH_DONE ||
                push_result.state == BALL_PUSH_FAULT_STOP) {
                locked = true;
            }

            if (locked) {
                drive_result = drive_stop();
            } else {
                switch (push_result.drive_mode) {
                case BALL_DRIVE_OPEN:
                    drive_result = drive_set_motion(
                        push_result.forward, push_result.lateral,
                        push_result.turn, &wheels);
                    break;
                case BALL_DRIVE_LATERAL:
                    /* Same proven lateral path used by obstacle avoidance. */
                    drive_result = drive_set_motion_feedback(
                        push_result.forward, push_result.lateral,
                        push_result.turn, elapsed_ms,
                        left_count, right_count, rear_count, &wheels);
                    break;
                case BALL_DRIVE_APPROACH:
                    drive_result = drive_set_approach_feedback(
                        push_result.forward, push_result.turn, elapsed_ms,
                        left_count, right_count, &wheels);
                    break;
                default:
                    drive_result = drive_stop();
                    break;
                }
            }
            require_ok("drive command", drive_result);

            log_elapsed_ms += elapsed_ms;
            if (log_elapsed_ms >= LINE_LOG_PERIOD_MS) {
                ESP_LOGI(TAG,
                         "push=%s attempt=%u retry=%u motion=[%d,%d,%d] "
                         "drive=%s ball=[red=%d blue=%d] px=[r=%u b=%u] pockets=%u "
                         "cam=[%ux%u age=%lldms frames=%u drop=%u dec=%u] "
                         "enc=[%d,%d,%d]",
                         ball_push_state_name(push_result.state),
                         push_result.attempt, push_result.retry,
                         push_result.forward, push_result.lateral,
                         push_result.turn,
                         ball_push_drive_mode_name(push_result.drive_mode),
                         ball_vision.balls[BALL_COLOR_RED].present ? 1 : 0,
                         ball_vision.balls[BALL_COLOR_BLUE].present ? 1 : 0,
                         ball_vision.red_px,
                         ball_vision.blue_px,
                         ball_vision.pocket_count,
                         (unsigned)camera.image_width,
                         (unsigned)camera.image_height,
                         (long long)frame_age_ms,
                         (unsigned)camera.received_frames,
                         (unsigned)camera.dropped_frames,
                         (unsigned)camera.decode_failures,
                         left_count, right_count, rear_count);
                log_elapsed_ms = 0;
            }
        } else {
            /* Locked (either phase): hold the car, keep the dashboard and
             * Wi-Fi viewer alive so the fault is visible. */
            drive_result = drive_stop();
            require_ok("drive command", drive_result);
        }

        vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}
