#include "board_config.h"
#include "infrared_sensor.h"
#include "status_led.h"

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "infrared_demo";

static void show_configuration_error(void)
{
    ESP_LOGE(TAG, "Set IR_CHANNEL_4_GPIO through IR_CHANNEL_1_GPIO in main/board_config.h");
    ESP_LOGE(TAG, "Expected physical order from left to right: channel 4, 3, 2, 1");

    while (1) {
        ESP_ERROR_CHECK(status_led_show_sensor(0));
        vTaskDelay(pdMS_TO_TICKS(400));
        ESP_ERROR_CHECK(status_led_set_enabled(false));
        vTaskDelay(pdMS_TO_TICKS(400));
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(status_led_init());

    const esp_err_t sensor_result = infrared_sensor_init();
    if (sensor_result != ESP_OK) {
        show_configuration_error();
        return;
    }

    infrared_sensor_state_t state = {0};
    ESP_ERROR_CHECK(infrared_sensor_sample(&state));
    ESP_ERROR_CHECK(status_led_show_sensor(state.black_mask));

    char display[5];
    infrared_sensor_format(state.black_mask, display);
    ESP_LOGI(TAG, "CH4..CH1=%s, black_mask=0x%X (B=black, W=white)",
             display, state.black_mask);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(IR_SAMPLE_PERIOD_MS));
        ESP_ERROR_CHECK(infrared_sensor_sample(&state));
        if (state.changed) {
            infrared_sensor_format(state.black_mask, display);
            ESP_LOGI(TAG, "CH4..CH1=%s, black_mask=0x%X",
                     display, state.black_mask);
            ESP_ERROR_CHECK(status_led_show_sensor(state.black_mask));
        }
    }
}
