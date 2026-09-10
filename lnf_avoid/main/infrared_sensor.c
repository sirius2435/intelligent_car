#include "infrared_sensor.h"

#include <stddef.h>

#include "board_config.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "infrared_sensor";

static const int s_gpio_by_bit[4] = {
    IR_CHANNEL_1_GPIO,
    IR_CHANNEL_2_GPIO,
    IR_CHANNEL_3_GPIO,
    IR_CHANNEL_4_GPIO,
};

static bool s_initialized;
static uint8_t s_stable_mask;
static uint8_t s_candidate_mask;
static uint8_t s_candidate_count;

static esp_err_t validate_pins(void)
{
    const int motor_pins[] = {
        MOTOR_LEFT_IN1_GPIO, MOTOR_LEFT_IN2_GPIO, MOTOR_LEFT_PWM_GPIO,
        MOTOR_RIGHT_IN1_GPIO, MOTOR_RIGHT_IN2_GPIO, MOTOR_RIGHT_PWM_GPIO,
        MOTOR_REAR_IN1_GPIO, MOTOR_REAR_IN2_GPIO, MOTOR_REAR_PWM_GPIO,
    };
    const int encoder_pins[] = {
        ENCODER_LEFT_A_GPIO, ENCODER_LEFT_B_GPIO,
        ENCODER_RIGHT_A_GPIO, ENCODER_RIGHT_B_GPIO,
        ENCODER_REAR_A_GPIO, ENCODER_REAR_B_GPIO,
    };

    for (size_t i = 0; i < 4; ++i) {
        const int gpio = s_gpio_by_bit[i];
        if (!GPIO_IS_VALID_GPIO(gpio)) {
            ESP_LOGE(TAG, "Channel %u GPIO is invalid or still -1; edit board_config.h",
                     (unsigned)i + 1U);
            return ESP_ERR_INVALID_ARG;
        }
        for (size_t j = i + 1; j < 4; ++j) {
            if (gpio == s_gpio_by_bit[j]) {
                ESP_LOGE(TAG, "GPIO %d is assigned to multiple infrared channels", gpio);
                return ESP_ERR_INVALID_ARG;
            }
        }
        for (size_t j = 0; j < sizeof(motor_pins) / sizeof(motor_pins[0]); ++j) {
            if (gpio == motor_pins[j]) {
                ESP_LOGE(TAG, "Infrared GPIO %d conflicts with a motor signal", gpio);
                return ESP_ERR_INVALID_ARG;
            }
        }
#if MOTOR_STBY_GPIO >= 0
        if (gpio == MOTOR_STBY_GPIO) {
            ESP_LOGE(TAG, "Infrared GPIO %d conflicts with motor STBY", gpio);
            return ESP_ERR_INVALID_ARG;
        }
#endif
        for (size_t j = 0; j < sizeof(encoder_pins) / sizeof(encoder_pins[0]); ++j) {
            if (gpio == encoder_pins[j]) {
                ESP_LOGE(TAG, "Infrared GPIO %d conflicts with an encoder signal", gpio);
                return ESP_ERR_INVALID_ARG;
            }
        }
    }
    return ESP_OK;
}

static uint8_t read_black_mask(void)
{
    uint8_t mask = 0;
    for (size_t bit = 0; bit < 4; ++bit) {
        if (gpio_get_level((gpio_num_t)s_gpio_by_bit[bit]) == IR_BLACK_LEVEL) {
            mask |= (uint8_t)(1U << bit);
        }
    }
    return mask;
}

esp_err_t infrared_sensor_init(void)
{
    ESP_RETURN_ON_ERROR(validate_pins(), TAG, "unsafe infrared pin configuration");

    uint64_t pin_mask = 0;
    for (size_t i = 0; i < 4; ++i) {
        pin_mask |= 1ULL << (unsigned)s_gpio_by_bit[i];
    }

    const gpio_config_t config = {
        .pin_bit_mask = pin_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&config), TAG, "failed to configure sensor inputs");

    s_stable_mask = read_black_mask();
    s_candidate_mask = s_stable_mask;
    s_candidate_count = 0;
    s_initialized = true;
    return ESP_OK;
}

esp_err_t infrared_sensor_sample(infrared_sensor_state_t *state)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t instant_mask = read_black_mask();
    if (instant_mask != s_candidate_mask) {
        s_candidate_mask = instant_mask;
        s_candidate_count = 1;
    } else if (s_candidate_count < IR_DEBOUNCE_SAMPLE_COUNT) {
        ++s_candidate_count;
    }

    if (s_candidate_count >= IR_DEBOUNCE_SAMPLE_COUNT &&
        s_stable_mask != s_candidate_mask) {
        s_stable_mask = s_candidate_mask;
    }
    state->black_mask = s_stable_mask;
    return ESP_OK;
}

void infrared_sensor_format(uint8_t black_mask, char output[5])
{
    if (output == NULL) {
        return;
    }
    output[0] = (black_mask & IR_CHANNEL_4_MASK) ? 'B' : 'W';
    output[1] = (black_mask & IR_CHANNEL_3_MASK) ? 'B' : 'W';
    output[2] = (black_mask & IR_CHANNEL_2_MASK) ? 'B' : 'W';
    output[3] = (black_mask & IR_CHANNEL_1_MASK) ? 'B' : 'W';
    output[4] = '\0';
}
