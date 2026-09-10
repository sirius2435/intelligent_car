#include "ultrasonic.h"

#include <stdbool.h>
#include <stddef.h>

#include "board_config.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static const char *TAG = "ultrasonic";

typedef struct {
    int level;
    int64_t timestamp_us;
} echo_edge_t;

static QueueHandle_t s_edge_queue;
static portMUX_TYPE s_reading_lock = portMUX_INITIALIZER_UNLOCKED;
static ultrasonic_reading_t s_latest;
static bool s_initialized;

static esp_err_t validate_pins(void)
{
    const int occupied[] = {
        MOTOR_LEFT_IN1_GPIO, MOTOR_LEFT_IN2_GPIO, MOTOR_LEFT_PWM_GPIO,
        MOTOR_RIGHT_IN1_GPIO, MOTOR_RIGHT_IN2_GPIO, MOTOR_RIGHT_PWM_GPIO,
        MOTOR_REAR_IN1_GPIO, MOTOR_REAR_IN2_GPIO, MOTOR_REAR_PWM_GPIO,
        ENCODER_LEFT_A_GPIO, ENCODER_LEFT_B_GPIO,
        ENCODER_RIGHT_A_GPIO, ENCODER_RIGHT_B_GPIO,
        ENCODER_REAR_A_GPIO, ENCODER_REAR_B_GPIO,
        IR_CHANNEL_1_GPIO, IR_CHANNEL_2_GPIO,
        IR_CHANNEL_3_GPIO, IR_CHANNEL_4_GPIO,
    };

    if (!GPIO_IS_VALID_OUTPUT_GPIO(ULTRASONIC_TRIG_GPIO) ||
        !GPIO_IS_VALID_GPIO(ULTRASONIC_ECHO_GPIO) ||
        ULTRASONIC_TRIG_GPIO == ULTRASONIC_ECHO_GPIO) {
        ESP_LOGE(TAG, "Invalid HC-SR04 TRIG/ECHO GPIO configuration");
        return ESP_ERR_INVALID_ARG;
    }
    for (size_t i = 0; i < sizeof(occupied) / sizeof(occupied[0]); ++i) {
        if (ULTRASONIC_TRIG_GPIO == occupied[i] ||
            ULTRASONIC_ECHO_GPIO == occupied[i]) {
            ESP_LOGE(TAG, "HC-SR04 GPIO conflicts with GPIO %d", occupied[i]);
            return ESP_ERR_INVALID_ARG;
        }
    }
#if MOTOR_STBY_GPIO >= 0
    if (ULTRASONIC_TRIG_GPIO == MOTOR_STBY_GPIO ||
        ULTRASONIC_ECHO_GPIO == MOTOR_STBY_GPIO) {
        ESP_LOGE(TAG, "HC-SR04 GPIO conflicts with motor STBY");
        return ESP_ERR_INVALID_ARG;
    }
#endif
    return ESP_OK;
}

static void IRAM_ATTR echo_isr(void *argument)
{
    (void)argument;
    const echo_edge_t edge = {
        .level = gpio_get_level((gpio_num_t)ULTRASONIC_ECHO_GPIO),
        .timestamp_us = esp_timer_get_time(),
    };
    BaseType_t higher_priority_woken = pdFALSE;
    xQueueSendFromISR(s_edge_queue, &edge, &higher_priority_woken);
    if (higher_priority_woken) {
        portYIELD_FROM_ISR();
    }
}

static void publish_reading(ultrasonic_reading_status_t status,
                            uint32_t distance_mm)
{
    taskENTER_CRITICAL(&s_reading_lock);
    s_latest.status = status;
    s_latest.distance_mm = distance_mm;
    ++s_latest.sequence;
    taskEXIT_CRITICAL(&s_reading_lock);
}

static bool wait_for_level(int expected_level, echo_edge_t *edge)
{
    const TickType_t timeout_ticks =
        pdMS_TO_TICKS((ULTRASONIC_ECHO_TIMEOUT_US + 999U) / 1000U) + 1U;
    const TickType_t start = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start) < timeout_ticks) {
        const TickType_t remaining = timeout_ticks - (xTaskGetTickCount() - start);
        if (xQueueReceive(s_edge_queue, edge, remaining) != pdTRUE) {
            return false;
        }
        if (edge->level == expected_level) {
            return true;
        }
    }
    return false;
}

static void ultrasonic_task(void *argument)
{
    (void)argument;
    while (1) {
        xQueueReset(s_edge_queue);
        gpio_set_level((gpio_num_t)ULTRASONIC_TRIG_GPIO, 0);
        esp_rom_delay_us(2);
        gpio_set_level((gpio_num_t)ULTRASONIC_TRIG_GPIO, 1);
        esp_rom_delay_us(10);
        gpio_set_level((gpio_num_t)ULTRASONIC_TRIG_GPIO, 0);

        echo_edge_t rising = {0};
        echo_edge_t falling = {0};
        if (!wait_for_level(1, &rising)) {
            publish_reading(ULTRASONIC_READING_NO_ECHO, 0);
        } else if (!wait_for_level(0, &falling) ||
                   falling.timestamp_us <= rising.timestamp_us) {
            publish_reading(ULTRASONIC_READING_ERROR, 0);
        } else {
            const int64_t pulse_us = falling.timestamp_us - rising.timestamp_us;
            const uint32_t distance_mm = (uint32_t)(pulse_us * 10 / 58);
            publish_reading(ULTRASONIC_READING_VALID, distance_mm);
        }
        vTaskDelay(pdMS_TO_TICKS(ULTRASONIC_SAMPLE_PERIOD_MS));
    }
}

esp_err_t ultrasonic_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(validate_pins(), TAG, "unsafe HC-SR04 pin configuration");

    const gpio_config_t trigger_config = {
        .pin_bit_mask = 1ULL << ULTRASONIC_TRIG_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&trigger_config), TAG,
                        "failed to configure TRIG");
    gpio_set_level((gpio_num_t)ULTRASONIC_TRIG_GPIO, 0);

    const gpio_config_t echo_config = {
        .pin_bit_mask = 1ULL << ULTRASONIC_ECHO_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&echo_config), TAG,
                        "failed to configure ECHO");

    s_edge_queue = xQueueCreate(8, sizeof(echo_edge_t));
    if (s_edge_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }
    esp_err_t result = gpio_install_isr_service(0);
    if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "failed to install GPIO ISR service: %s",
                 esp_err_to_name(result));
        return result;
    }
    ESP_RETURN_ON_ERROR(
        gpio_isr_handler_add((gpio_num_t)ULTRASONIC_ECHO_GPIO, echo_isr, NULL),
        TAG, "failed to attach ECHO ISR");

    s_latest = (ultrasonic_reading_t) {
        .status = ULTRASONIC_READING_UNAVAILABLE,
    };
    if (xTaskCreate(ultrasonic_task, "hc_sr04", 3072, NULL, 4, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    s_initialized = true;
    ESP_LOGI(TAG, "HC-SR04 initialized: TRIG=%d ECHO=%d",
             ULTRASONIC_TRIG_GPIO, ULTRASONIC_ECHO_GPIO);
    return ESP_OK;
}

esp_err_t ultrasonic_get_latest(ultrasonic_reading_t *reading)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (reading == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    taskENTER_CRITICAL(&s_reading_lock);
    *reading = s_latest;
    taskEXIT_CRITICAL(&s_reading_lock);
    return ESP_OK;
}

const char *ultrasonic_status_name(ultrasonic_reading_status_t status)
{
    switch (status) {
    case ULTRASONIC_READING_UNAVAILABLE:
        return "UNAVAILABLE";
    case ULTRASONIC_READING_VALID:
        return "VALID";
    case ULTRASONIC_READING_NO_ECHO:
        return "NO_ECHO";
    case ULTRASONIC_READING_ERROR:
        return "ERROR";
    default:
        return "UNKNOWN";
    }
}
