#include "ultrasonic_sensor.h"

#include <stdbool.h>

#include "board_config.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

static const char *TAG = "ultrasonic";
static bool s_initialized;

esp_err_t ultrasonic_sensor_init(void)
{
    if (!GPIO_IS_VALID_OUTPUT_GPIO(ULTRASONIC_TRIG_GPIO) ||
        !GPIO_IS_VALID_GPIO(ULTRASONIC_ECHO_GPIO) ||
        ULTRASONIC_TRIG_GPIO == ULTRASONIC_ECHO_GPIO) {
        ESP_LOGE(TAG, "Invalid ultrasonic GPIO configuration: trig=%d echo=%d",
                 ULTRASONIC_TRIG_GPIO, ULTRASONIC_ECHO_GPIO);
        return ESP_ERR_INVALID_ARG;
    }

    const gpio_config_t trig_config = {
        .pin_bit_mask = 1ULL << ULTRASONIC_TRIG_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&trig_config), TAG,
                        "failed to configure ultrasonic trigger");

    const gpio_config_t echo_config = {
        .pin_bit_mask = 1ULL << ULTRASONIC_ECHO_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&echo_config), TAG,
                        "failed to configure ultrasonic echo");

    gpio_set_level((gpio_num_t)ULTRASONIC_TRIG_GPIO, 0);
    s_initialized = true;
    ESP_LOGI(TAG, "Ultrasonic initialized: TRIG=%d ECHO=%d",
             ULTRASONIC_TRIG_GPIO, ULTRASONIC_ECHO_GPIO);
    return ESP_OK;
}

esp_err_t ultrasonic_sensor_measure_cm(float *distance_cm)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (distance_cm == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *distance_cm = 0.0f;

    /* HC-SR04 trigger pulse: at least 10 us high. */
    gpio_set_level((gpio_num_t)ULTRASONIC_TRIG_GPIO, 0);
    esp_rom_delay_us(2);
    gpio_set_level((gpio_num_t)ULTRASONIC_TRIG_GPIO, 1);
    esp_rom_delay_us(10);
    gpio_set_level((gpio_num_t)ULTRASONIC_TRIG_GPIO, 0);

    const int64_t wait_start = esp_timer_get_time();
    while (gpio_get_level((gpio_num_t)ULTRASONIC_ECHO_GPIO) == 0) {
        if (esp_timer_get_time() - wait_start >= ULTRASONIC_TIMEOUT_US) {
            return ESP_OK;
        }
    }

    const int64_t echo_start = esp_timer_get_time();
    while (gpio_get_level((gpio_num_t)ULTRASONIC_ECHO_GPIO) != 0) {
        if (esp_timer_get_time() - echo_start >= ULTRASONIC_TIMEOUT_US) {
            return ESP_OK;
        }
    }
    const int64_t echo_us = esp_timer_get_time() - echo_start;

    /* Speed of sound ~= 0.0343 cm/us; divide by two for the round trip. */
    *distance_cm = ((float)echo_us * 0.0343f) / 2.0f;
    return ESP_OK;
}
