#include "board_config.h"
#include "camera_stream.h"
#include "camera_vision.h"
#include "drive.h"
#include "encoder.h"
#include "lcd_monitor.h"
#include "obstacle_avoidance.h"
#include "ultrasonic.h"
#include "vision_follow.h"

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "vision_car";

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
            camera.connected && camera.vision.frame_valid) {
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

    require_ok("camera_vision_start", camera_vision_start());
    wait_for_first_camera_frame();

    ESP_LOGW(TAG, "task 2 starts in %d ms; infrared power must be disconnected",
             LINE_START_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(LINE_START_DELAY_MS));

    vision_follow_controller_t follower;
    vision_follow_init(&follower);
    obstacle_avoidance_controller_t avoidance;
    obstacle_avoidance_init(&avoidance);

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

        obstacle_avoidance_result_t avoid = obstacle_avoidance_update(
            &avoidance, &ultrasonic, &camera.vision, elapsed_ms,
            left_count, right_count, rear_count);
        if (avoid.just_completed) {
            vision_follow_init(&follower);
        }

        vision_follow_output_t follow = {
            .state = follower.state,
        };
        int forward = 0;
        int lateral = 0;
        int turn = 0;
        if (!locked && avoid.active) {
            forward = avoid.forward;
            lateral = avoid.lateral;
            turn = avoid.turn;
        } else if (!locked) {
            const bool finish_enabled =
                avoidance.state == AVOIDANCE_COMPLETE;
            follow = vision_follow_update(&follower, &camera.vision,
                                          finish_enabled, elapsed_ms);
            forward = follow.forward;
            turn = follow.turn;
            if (forward > avoid.tracking_forward_limit) {
                forward = avoid.tracking_forward_limit;
            }
            if (follow.finished || follow.state == VISION_FOLLOW_FAULT_STOP) {
                locked = true;
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

        drive_wheel_command_t wheels = {0};
        esp_err_t drive_result;
        if (locked) {
            drive_result = drive_stop();
        } else if (avoid.active &&
                   avoid.state == AVOIDANCE_FORWARD_PASS) {
            drive_result = drive_set_forward_feedback(
                forward, elapsed_ms, left_count, right_count, &wheels);
        } else if (lateral != 0) {
            drive_result = drive_set_motion_feedback(
                forward, lateral, turn, elapsed_ms,
                left_count, right_count, rear_count, &wheels);
        } else if (!avoid.active && avoid.slow_approach && forward > 0) {
            drive_result = drive_set_approach_feedback(
                forward, turn, elapsed_ms, left_count, right_count, &wheels);
        } else {
            drive_result = drive_set_motion(forward, lateral, turn, &wheels);
        }
        require_ok("drive command", drive_result);

        log_elapsed_ms += elapsed_ms;
        if (log_elapsed_ms >= LINE_LOG_PERIOD_MS) {
            ESP_LOGI(TAG,
                     "follow=%s avoid=%s lock=%d motion=[%d,%d,%d] "
                     "vision=[line=%d finish=%d conf=%u err=%d head=%d rows=%u "
                     "seq=%u age=%lldms drop=%u] range=[%s,%umm] "
                     "enc=[%d,%d,%d] progress=%lld",
                     vision_follow_state_name(follow.state),
                     obstacle_avoidance_state_name(avoid.state), locked,
                     forward, lateral, turn,
                     camera.vision.line_found, camera.vision.finish_marker,
                     camera.vision.confidence, camera.vision.lateral_error,
                     camera.vision.heading_error, camera.vision.valid_rows,
                     (unsigned)camera.vision.sequence, (long long)frame_age_ms,
                     (unsigned)camera.dropped_frames,
                     ultrasonic_status_name(ultrasonic.status),
                     (unsigned)ultrasonic.distance_mm,
                     left_count, right_count, rear_count,
                     (long long)avoidance.progress_counts);
            log_elapsed_ms = 0;
        }

        vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}
