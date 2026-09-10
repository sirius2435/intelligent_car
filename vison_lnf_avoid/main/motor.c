#include "motor.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "board_config.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_check.h"
#include "esp_log.h"

#define MOTOR_PWM_FREQUENCY_HZ  20000
#define MOTOR_PWM_MAX_DUTY       1023
#define MOTOR_SPEED_MAX          1000

static const char *TAG = "motor";
static bool s_initialized;

typedef struct {
    gpio_num_t in1;
    gpio_num_t in2;
    int pwm_gpio;
    ledc_channel_t pwm_channel;
    bool reversed;
    int last_sign;
} motor_channel_t;

static motor_channel_t s_motors[3] = {
    {
        .in1 = MOTOR_LEFT_IN1_GPIO,
        .in2 = MOTOR_LEFT_IN2_GPIO,
        .pwm_gpio = MOTOR_LEFT_PWM_GPIO,
        .pwm_channel = LEDC_CHANNEL_0,
        .reversed = MOTOR_LEFT_REVERSED,
    },
    {
        .in1 = MOTOR_RIGHT_IN1_GPIO,
        .in2 = MOTOR_RIGHT_IN2_GPIO,
        .pwm_gpio = MOTOR_RIGHT_PWM_GPIO,
        .pwm_channel = LEDC_CHANNEL_1,
        .reversed = MOTOR_RIGHT_REVERSED,
    },
    {
        .in1 = MOTOR_REAR_IN1_GPIO,
        .in2 = MOTOR_REAR_IN2_GPIO,
        .pwm_gpio = MOTOR_REAR_PWM_GPIO,
        .pwm_channel = LEDC_CHANNEL_2,
        .reversed = MOTOR_REAR_REVERSED,
    },
};

static esp_err_t validate_pins(void)
{
    const int pins[] = {
        MOTOR_LEFT_IN1_GPIO, MOTOR_LEFT_IN2_GPIO, MOTOR_LEFT_PWM_GPIO,
        MOTOR_RIGHT_IN1_GPIO, MOTOR_RIGHT_IN2_GPIO, MOTOR_RIGHT_PWM_GPIO,
        MOTOR_REAR_IN1_GPIO, MOTOR_REAR_IN2_GPIO, MOTOR_REAR_PWM_GPIO,
    };
    for (size_t i = 0; i < sizeof(pins) / sizeof(pins[0]); ++i) {
        if (!GPIO_IS_VALID_OUTPUT_GPIO(pins[i])) {
            ESP_LOGE(TAG, "Motor GPIO %d is not a valid output", pins[i]);
            return ESP_ERR_INVALID_ARG;
        }
        for (size_t j = i + 1; j < sizeof(pins) / sizeof(pins[0]); ++j) {
            if (pins[i] == pins[j]) {
                ESP_LOGE(TAG, "Motor GPIO %d is assigned more than once", pins[i]);
                return ESP_ERR_INVALID_ARG;
            }
        }
    }

#if MOTOR_STBY_GPIO >= 0
    if (!GPIO_IS_VALID_OUTPUT_GPIO(MOTOR_STBY_GPIO)) {
        ESP_LOGE(TAG, "STBY GPIO %d is not a valid output", MOTOR_STBY_GPIO);
        return ESP_ERR_INVALID_ARG;
    }
    for (size_t i = 0; i < sizeof(pins) / sizeof(pins[0]); ++i) {
        if (MOTOR_STBY_GPIO == pins[i]) {
            ESP_LOGE(TAG, "STBY GPIO %d duplicates a motor signal", MOTOR_STBY_GPIO);
            return ESP_ERR_INVALID_ARG;
        }
    }
#endif
    return ESP_OK;
}

static esp_err_t set_pwm(ledc_channel_t channel, uint32_t duty)
{
    ESP_RETURN_ON_ERROR(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty),
                        TAG, "failed to set PWM duty");
    return ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

static esp_err_t set_one(motor_channel_t *motor, int speed)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (speed > MOTOR_SPEED_MAX) {
        speed = MOTOR_SPEED_MAX;
    } else if (speed < -MOTOR_SPEED_MAX) {
        speed = -MOTOR_SPEED_MAX;
    }

    const int requested_sign = speed > 0 ? 1 : (speed < 0 ? -1 : 0);
    if (requested_sign != motor->last_sign) {
        ESP_RETURN_ON_ERROR(set_pwm(motor->pwm_channel, 0), TAG,
                            "failed to stop PWM before direction update");
    }
    if (speed == 0) {
        gpio_set_level(motor->in1, 0);
        gpio_set_level(motor->in2, 0);
        motor->last_sign = 0;
        return ESP_OK;
    }

#if MOTOR_STBY_GPIO >= 0
    gpio_set_level((gpio_num_t)MOTOR_STBY_GPIO, 1);
#endif

    if (requested_sign != motor->last_sign) {
        bool forward = speed > 0;
        if (motor->reversed) {
            forward = !forward;
        }
        gpio_set_level(motor->in1, forward ? 1 : 0);
        gpio_set_level(motor->in2, forward ? 0 : 1);
        motor->last_sign = requested_sign;
    }

    const uint32_t duty = (uint32_t)abs(speed) * MOTOR_PWM_MAX_DUTY / MOTOR_SPEED_MAX;
    return set_pwm(motor->pwm_channel, duty);
}

esp_err_t motor_init(void)
{
    ESP_RETURN_ON_ERROR(validate_pins(), TAG, "unsafe motor pin configuration");

    uint64_t direction_mask = 0;
    for (size_t i = 0; i < 3; ++i) {
        direction_mask |= 1ULL << (unsigned)s_motors[i].in1;
        direction_mask |= 1ULL << (unsigned)s_motors[i].in2;
    }
    const gpio_config_t direction_config = {
        .pin_bit_mask = direction_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&direction_config), TAG,
                        "failed to configure motor direction pins");

    for (size_t i = 0; i < 3; ++i) {
        gpio_set_level(s_motors[i].in1, 0);
        gpio_set_level(s_motors[i].in2, 0);
    }

#if MOTOR_STBY_GPIO >= 0
    const gpio_config_t standby_config = {
        .pin_bit_mask = 1ULL << (unsigned)MOTOR_STBY_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&standby_config), TAG,
                        "failed to configure motor STBY pin");
    gpio_set_level((gpio_num_t)MOTOR_STBY_GPIO, 0);
#endif

    const ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = MOTOR_PWM_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_config), TAG,
                        "failed to configure motor PWM timer");

    for (size_t i = 0; i < 3; ++i) {
        const ledc_channel_config_t channel_config = {
            .gpio_num = s_motors[i].pwm_gpio,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = s_motors[i].pwm_channel,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
            .flags.output_invert = 0,
        };
        ESP_RETURN_ON_ERROR(ledc_channel_config(&channel_config), TAG,
                            "failed to configure motor PWM channel");
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Three motor channels initialized at %d Hz", MOTOR_PWM_FREQUENCY_HZ);
    return motor_stop_all();
}

esp_err_t motor_set_all(int left_speed, int right_speed, int rear_speed)
{
    ESP_RETURN_ON_ERROR(set_one(&s_motors[0], left_speed), TAG, "left motor update failed");
    ESP_RETURN_ON_ERROR(set_one(&s_motors[1], right_speed), TAG, "right motor update failed");
    return set_one(&s_motors[2], rear_speed);
}

esp_err_t motor_stop_all(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t first_error = ESP_OK;
    for (size_t i = 0; i < 3; ++i) {
        const esp_err_t result = set_one(&s_motors[i], 0);
        if (first_error == ESP_OK && result != ESP_OK) {
            first_error = result;
        }
    }
#if MOTOR_STBY_GPIO >= 0
    gpio_set_level((gpio_num_t)MOTOR_STBY_GPIO, 0);
#endif
    return first_error;
}
