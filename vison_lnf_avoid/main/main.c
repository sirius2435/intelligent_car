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

typedef enum {
    MISSION_LINE = 0,
    MISSION_BALL,
    MISSION_COMPLETE,
} mission_state_t;

static const char *mission_state_name(mission_state_t state)
{
    switch (state) {
    case MISSION_LINE: return "LINE";
    case MISSION_BALL: return "BALL";
    case MISSION_COMPLETE: return "COMPLETE";
    default: return "UNKNOWN";
    }
}

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
    ball_push_controller_t ball_controller;
    ball_push_init(&ball_controller, 0, 0, 0);
    mission_state_t mission = MISSION_LINE;

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

        ultrasonic_reading_t ultrasonic = {0};
        require_ok("ultrasonic_get_latest",
                   ultrasonic_get_latest(&ultrasonic));

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

        int forward = 0;
        int lateral = 0;
        int turn = 0;
        infrared_sensor_state_t sensor = camera.infrared;
        obstacle_avoidance_result_t avoid = {
            .state = avoidance.state,
            .tracking_forward_limit = 1000,
        };
        line_follow_result_t control = {
            .state = controller.state,
        };
        ball_push_result_t ball_control = {
            .state = ball_controller.state,
            .target_color = ball_controller.target_color,
        };

        if (mission == MISSION_LINE && !locked) {
            avoid = obstacle_avoidance_update(
                &avoidance, &ultrasonic, sensor.black_mask, elapsed_ms,
                left_count, right_count, rear_count);
            if (avoid.just_completed) {
                line_follow_init(&controller);
            }
            if (avoid.active) {
                forward = avoid.forward;
                lateral = avoid.lateral;
                turn = avoid.turn;
            } else {
                control = line_follow_update(&controller, sensor.black_mask,
                                             elapsed_ms, left_count, right_count);
                forward = control.forward;
                turn = control.turn;
                if (forward > avoid.tracking_forward_limit) {
                    forward = avoid.tracking_forward_limit;
                }
            }
            if (avoid.state == AVOIDANCE_FAULT_STOP) {
                locked = true;
            }
            if (control.state == LINE_FOLLOW_STOPPED) {
                forward = lateral = turn = 0;
                if (!sensor.finish_detected) {
                    /* A scripted route has no visual correction and therefore
                     * requires the repeatable pose supplied by the END bar. */
                    ESP_LOGE(TAG, "line stopped without END confirmation; fixed ball route inhibited");
                    locked = true;
                } else {
                    ESP_LOGI(TAG, "END confirmed; starting fixed ball route");
                    ball_push_init(&ball_controller, left_count, right_count,
                                   rear_count);
                    require_ok("camera ball mode",
                               camera_vision_set_mode(CAMERA_VISION_BALL));
                    mission = MISSION_BALL;
                    /* Keep ball-mode overlays available for route calibration,
                     * although the fixed controller does not depend on them. */
                    if (camera_gimbal_enabled()) {
                        const esp_err_t tilt_result =
                            camera_gimbal_set_tilt(BALL_GIMBAL_TILT_DEG);
                        if (tilt_result != ESP_OK) {
                            ESP_LOGW(TAG, "ball gimbal tilt failed: %s",
                                     esp_err_to_name(tilt_result));
                        } else {
                            ESP_LOGI(TAG, "ball gimbal tilt -> %d deg, settling %d ms",
                                     (int)BALL_GIMBAL_TILT_DEG,
                                     (int)BALL_GIMBAL_SETTLE_MS);
                            vTaskDelay(pdMS_TO_TICKS(BALL_GIMBAL_SETTLE_MS));
                            previous_ticks = xTaskGetTickCount();
                        }
                    }
                }
            }
        } else if (mission == MISSION_BALL && !locked) {
            ball_control = ball_push_update(
                &ball_controller, elapsed_ms, left_count, right_count,
                rear_count);
            forward = ball_control.forward;
            lateral = ball_control.lateral;
            turn = ball_control.turn;
            if (ball_control.fault) {
                ESP_LOGE(TAG, "ball task failed in %s",
                         ball_push_state_name(ball_control.state));
                locked = true;
            } else if (ball_control.mission_complete) {
                ESP_LOGI(TAG, "both balls completed");
                mission = MISSION_COMPLETE;
                forward = lateral = turn = 0;
            }
        }

        drive_wheel_command_t wheels = {0};
        esp_err_t drive_result;
        if (locked || mission == MISSION_COMPLETE) {
            drive_result = drive_stop();
        } else if (mission == MISSION_LINE && avoid.active &&
                   avoid.state == AVOIDANCE_FORWARD_PASS) {
            drive_result = drive_set_forward_feedback(
                forward, elapsed_ms, left_count, right_count, &wheels);
        } else if (mission == MISSION_LINE && !avoid.active &&
                   avoid.slow_approach && forward > 0) {
            drive_result = drive_set_approach_feedback(
                forward, turn, elapsed_ms, left_count, right_count, &wheels);
        } else if (mission == MISSION_BALL && forward != 0 && lateral == 0) {
            drive_result = drive_set_approach_feedback(
                forward, turn, elapsed_ms, left_count, right_count, &wheels);
        } else {
            drive_result = drive_set_motion_feedback(
                forward, lateral, turn, elapsed_ms,
                left_count, right_count, rear_count, &wheels);
        }
        require_ok("drive command", drive_result);

        log_elapsed_ms += elapsed_ms;
        if (log_elapsed_ms >= LINE_LOG_PERIOD_MS) {
            char display[5];
            pseudo_infrared_format(sensor.black_mask, display);
            ESP_LOGI(TAG,
                     "mission=%s follow=%s avoid=%s ball=%s/%s lock=%d "
                     "sensor=%s end=%d motion=[%d,%d,%d] "
                     "cam=[%ux%u age=%lldms frames=%u drop=%u dec=%u] "
                     "det=[R:%d W:%d HL:%d HR:%d] "
                     "range=[%s,%umm] enc=[%d,%d,%d]",
                     mission_state_name(mission),
                     line_follow_state_name(control.state),
                     obstacle_avoidance_state_name(avoid.state),
                     ball_push_state_name(ball_controller.state),
                     ball_controller.target_color == BALL_COLOR_RED ? "RED" : "WHITE",
                     locked,
                     display, sensor.finish_detected,
                     forward, lateral, turn,
                     (unsigned)camera.image_width,
                     (unsigned)camera.image_height,
                     (long long)frame_age_ms,
                     (unsigned)camera.received_frames,
                     (unsigned)camera.dropped_frames,
                     (unsigned)camera.decode_failures,
                     camera.ball.red_ball.found,
                     camera.ball.white_ball.found,
                     camera.ball.left_hole.found,
                     camera.ball.right_hole.found,
                     ultrasonic_status_name(ultrasonic.status),
                     (unsigned)ultrasonic.distance_mm,
                     left_count, right_count, rear_count);
            log_elapsed_ms = 0;
        }

        vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}
