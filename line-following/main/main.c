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

    ESP_LOGW(TAG, "Line following starts in %d ms; keep the car safely positioned",
             LINE_START_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(LINE_START_DELAY_MS));

    line_follow_controller_t controller;
    line_follow_init(&controller);
    infrared_sensor_state_t sensor = {0};
    TickType_t wake_time = xTaskGetTickCount();
    TickType_t previous_ticks = wake_time;
    uint32_t log_elapsed_ms = 0;
    uint32_t max_loop_ms = 0;
    uint32_t loop_overrun_count = 0;
    int previous_left_count = 0;
    int previous_right_count = 0;
    int previous_rear_count = 0;

    while (1) {
        /* Real elapsed time since the previous iteration: the nominal
           period is a lower bound, and a slow log write would otherwise
           stretch every ms-based timer in the controller. */
        const TickType_t now = xTaskGetTickCount();
        const uint32_t elapsed_ms =
            (uint32_t)((now - previous_ticks) * portTICK_PERIOD_MS);
        previous_ticks = now;
        if (elapsed_ms > max_loop_ms) {
            max_loop_ms = elapsed_ms;
        }
        if (elapsed_ms > IR_SAMPLE_PERIOD_MS) {
            ++loop_overrun_count;
        }

        result = infrared_sensor_sample(&sensor);
        if (result != ESP_OK) {
            stop_after_runtime_error("infrared_sensor_sample", result);
        }

        const line_follow_result_t control =
            line_follow_update(&controller, sensor.black_mask, elapsed_ms);
        drive_wheel_command_t wheels = {0};
        result = drive_set_motion(control.forward, control.turn, &wheels);
        if (result != ESP_OK) {
            stop_after_runtime_error("drive_set_motion", result);
        }

        log_elapsed_ms += elapsed_ms;
        /* Keep UART output strictly periodic. Sensor transitions can occur every
           control cycle on dense bends and must not trigger extra blocking logs. */
        if (log_elapsed_ms >= LINE_LOG_PERIOD_MS) {
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
                     "wheels=[%d,%d,%d] enc=[%d,%d,%d] delta=[%d,%d,%d] "
                     "dt=%ums loop_max=%ums overruns=%u",
                     line_follow_state_name(control.state), display, control.error,
                     control.forward, control.turn, wheels.left, wheels.right, wheels.rear,
                     left_count, right_count, rear_count,
                     left_count - previous_left_count,
                     right_count - previous_right_count,
                     rear_count - previous_rear_count,
                     (unsigned)log_elapsed_ms, (unsigned)max_loop_ms,
                     (unsigned)loop_overrun_count);
            previous_left_count = left_count;
            previous_right_count = right_count;
            previous_rear_count = rear_count;
            log_elapsed_ms = 0;
            max_loop_ms = 0;
            loop_overrun_count = 0;
        }

        vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(IR_SAMPLE_PERIOD_MS));
    }
}
