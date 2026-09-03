#include "board_config.h"
#include "drive.h"
#include "encoder.h"
#include "infrared_sensor.h"
#include "line_follow.h"

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "line_following";

static void stop_after_runtime_error(const char *operation, esp_err_t result)
{
    ESP_LOGE(TAG, "%s failed: %s", operation, esp_err_to_name(result));
    const esp_err_t stop_result = drive_stop();
    if (stop_result != ESP_OK) {
        ESP_LOGE(TAG, "Emergency motor stop failed: %s", esp_err_to_name(stop_result));
    }
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

#if STRAFE_DEMO_ENABLED
static void run_strafe_demo(void)
{
    ESP_LOGW(TAG, "Strafe demo: left translation at %d/1000 for %d ms",
             STRAFE_DEMO_SPEED, STRAFE_DEMO_RUN_MS);
    vTaskDelay(pdMS_TO_TICKS(STRAFE_DEMO_START_MS));

    drive_wheel_command_t wheels = {0};
    esp_err_t result = drive_set_motion(0, STRAFE_DEMO_SPEED, 0, &wheels);
    if (result != ESP_OK) {
        stop_after_runtime_error("drive_set_motion", result);
    }
    ESP_LOGI(TAG, "Strafe wheels=[%d,%d,%d]", wheels.left, wheels.right, wheels.rear);

    vTaskDelay(pdMS_TO_TICKS(STRAFE_DEMO_RUN_MS));

    result = drive_stop();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Strafe stop failed: %s", esp_err_to_name(result));
    }
    ESP_LOGI(TAG, "Strafe demo complete; motors stopped");
}
#endif

void app_main(void)
{
    esp_err_t result = drive_init();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Drive initialization failed: %s", esp_err_to_name(result));
        return;
    }

    result = infrared_sensor_init();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Sensor initialization failed: %s", esp_err_to_name(result));
        ESP_LOGE(TAG, "Fill all IR_CHANNEL_*_GPIO values in main/board_config.h");
        ESP_ERROR_CHECK_WITHOUT_ABORT(drive_stop());
        return;
    }

    result = encoder_init();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Encoder initialization failed: %s", esp_err_to_name(result));
        ESP_ERROR_CHECK_WITHOUT_ABORT(drive_stop());
        return;
    }

#if STRAFE_DEMO_ENABLED
    run_strafe_demo();
    return;
#endif

    ESP_LOGW(TAG, "Line following starts in %d ms; keep the car safely positioned",
             LINE_START_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(LINE_START_DELAY_MS));

    line_follow_controller_t controller;
    line_follow_init(&controller);
    infrared_sensor_state_t sensor = {0};
    TickType_t wake_time = xTaskGetTickCount();
    uint32_t log_elapsed_ms = LINE_LOG_PERIOD_MS;
    int previous_left_count = 0;
    int previous_right_count = 0;
    int previous_rear_count = 0;

    while (1) {
        result = infrared_sensor_sample(&sensor);
        if (result != ESP_OK) {
            stop_after_runtime_error("infrared_sensor_sample", result);
        }

        const line_follow_result_t control =
            line_follow_update(&controller, sensor.black_mask, IR_SAMPLE_PERIOD_MS);
        drive_wheel_command_t wheels = {0};
        result = drive_set_motion(control.forward, 0, control.turn, &wheels);
        if (result != ESP_OK) {
            stop_after_runtime_error("drive_set_motion", result);
        }

        log_elapsed_ms += IR_SAMPLE_PERIOD_MS;
        if (control.state_changed || sensor.changed || log_elapsed_ms >= LINE_LOG_PERIOD_MS) {
            int left_count = 0;
            int right_count = 0;
            int rear_count = 0;
            result = encoder_get_all(&left_count, &right_count, &rear_count);
            if (result != ESP_OK) {
                stop_after_runtime_error("encoder_get_all", result);
            }
            char display[5];
            infrared_sensor_format(sensor.black_mask, display);
            ESP_LOGI(TAG,
                     "state=%s sensor=%s error=%d forward=%d turn=%d "
                     "wheels=[%d,%d,%d] enc=[%d,%d,%d] delta=[%d,%d,%d] dt=%ums",
                     line_follow_state_name(control.state), display, control.error,
                     control.forward, control.turn, wheels.left, wheels.right, wheels.rear,
                     left_count, right_count, rear_count,
                     left_count - previous_left_count,
                     right_count - previous_right_count,
                     rear_count - previous_rear_count,
                     (unsigned)log_elapsed_ms);
            previous_left_count = left_count;
            previous_right_count = right_count;
            previous_rear_count = rear_count;
            log_elapsed_ms = 0;
        }

        vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(IR_SAMPLE_PERIOD_MS));
    }
}
