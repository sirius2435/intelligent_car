#include "board_pins.h"
#include "encoder.h"
#include "motor.h"

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "wheel_test";

static void clear_encoder_if_enabled(encoder_wheel_t wheel)
{
    if (encoder_is_enabled()) {
        ESP_ERROR_CHECK(encoder_clear(wheel));
    }
}

static void log_encoder_if_enabled(const char *wheel_name, encoder_wheel_t wheel)
{
    if (!encoder_is_enabled()) {
        return;
    }

    int count = 0;
    ESP_ERROR_CHECK(encoder_get_count(wheel, &count));
    ESP_LOGI(TAG, "%s encoder count: %d", wheel_name, count);
}

static void pause_with_motors_stopped(void)
{
    ESP_ERROR_CHECK(motor_stop_all());
    vTaskDelay(pdMS_TO_TICKS(MOTOR_TEST_PAUSE_MS));
}

void app_main(void)
{
    ESP_LOGW(TAG, "Lift the car so all drive wheels are off the ground");
    ESP_LOGW(TAG, "Motor test begins in %d ms", MOTOR_TEST_START_MS);

    esp_err_t result = motor_init();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Motor initialization blocked: %s", esp_err_to_name(result));
        ESP_LOGE(TAG, "Fill in the GPIO values in main/board_pins.h, then rebuild");
        return;
    }
    ESP_ERROR_CHECK(encoder_init());

    vTaskDelay(pdMS_TO_TICKS(MOTOR_TEST_START_MS));

    ESP_LOGI(TAG, "Test 1/3: left wheel (motor D) forward at %d/1000", MOTOR_TEST_SPEED);
    clear_encoder_if_enabled(ENCODER_WHEEL_LEFT);
    ESP_ERROR_CHECK(motor_set_left(MOTOR_TEST_SPEED));
    vTaskDelay(pdMS_TO_TICKS(MOTOR_TEST_RUN_MS));
    pause_with_motors_stopped();
    log_encoder_if_enabled("Left/Motor D", ENCODER_WHEEL_LEFT);

    ESP_LOGI(TAG, "Test 2/3: right wheel (motor A) forward at %d/1000", MOTOR_TEST_SPEED);
    clear_encoder_if_enabled(ENCODER_WHEEL_RIGHT);
    ESP_ERROR_CHECK(motor_set_right(MOTOR_TEST_SPEED));
    vTaskDelay(pdMS_TO_TICKS(MOTOR_TEST_RUN_MS));
    pause_with_motors_stopped();
    log_encoder_if_enabled("Right/Motor A", ENCODER_WHEEL_RIGHT);

    ESP_LOGI(TAG, "Test 3/3: rear wheel (motor B) forward at %d/1000", MOTOR_TEST_SPEED);
    clear_encoder_if_enabled(ENCODER_WHEEL_REAR);
    ESP_ERROR_CHECK(motor_set_rear(MOTOR_TEST_SPEED));
    vTaskDelay(pdMS_TO_TICKS(MOTOR_TEST_RUN_MS));
    pause_with_motors_stopped();
    log_encoder_if_enabled("Rear/Motor B", ENCODER_WHEEL_REAR);

#if MOTOR_RUN_REVERSE_TEST
    ESP_LOGI(TAG, "Optional test: all three wheels reverse at %d/1000", MOTOR_TEST_SPEED);
    ESP_ERROR_CHECK(motor_set_all(-MOTOR_TEST_SPEED, -MOTOR_TEST_SPEED, -MOTOR_TEST_SPEED));
    vTaskDelay(pdMS_TO_TICKS(MOTOR_TEST_RUN_MS));
    pause_with_motors_stopped();
#endif

#if MOTOR_RUN_ALL_TEST
    ESP_LOGI(TAG, "Optional test: all three wheels forward at %d/1000", MOTOR_TEST_SPEED);
    ESP_ERROR_CHECK(motor_set_all(MOTOR_TEST_SPEED, MOTOR_TEST_SPEED, MOTOR_TEST_SPEED));
    vTaskDelay(pdMS_TO_TICKS(MOTOR_TEST_RUN_MS));
    pause_with_motors_stopped();
#endif

    ESP_LOGI(TAG, "Motor test complete; motors remain stopped");
}
