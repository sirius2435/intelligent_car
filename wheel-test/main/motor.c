#include "motor.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "board_pins.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_check.h"
#include "esp_log.h"

#define MOTOR_PWM_FREQUENCY_HZ  20000
#define MOTOR_PWM_MAX_DUTY      1023
#define MOTOR_SPEED_MAX         1000

static const char *TAG = "motor";
static bool s_initialized;

typedef struct {
    gpio_num_t in1;
    gpio_num_t in2;
    ledc_channel_t pwm_channel;
    bool reversed;
} motor_channel_t;

static const motor_channel_t s_left_motor = {
    .in1 = (gpio_num_t)MOTOR_LEFT_IN1_GPIO,
    .in2 = (gpio_num_t)MOTOR_LEFT_IN2_GPIO,
    .pwm_channel = LEDC_CHANNEL_0,
    .reversed = MOTOR_LEFT_REVERSED,
};

static const motor_channel_t s_right_motor = {
    .in1 = (gpio_num_t)MOTOR_RIGHT_IN1_GPIO,
    .in2 = (gpio_num_t)MOTOR_RIGHT_IN2_GPIO,
    .pwm_channel = LEDC_CHANNEL_1,
    .reversed = MOTOR_RIGHT_REVERSED,
};

static const motor_channel_t s_rear_motor = {
    .in1 = (gpio_num_t)MOTOR_REAR_IN1_GPIO,
    .in2 = (gpio_num_t)MOTOR_REAR_IN2_GPIO,
    .pwm_channel = LEDC_CHANNEL_2,
    .reversed = MOTOR_REAR_REVERSED,
};

static esp_err_t validate_pin_configuration(void)
{
    const int pins[] = {
        MOTOR_LEFT_IN1_GPIO,
        MOTOR_LEFT_IN2_GPIO,
        MOTOR_LEFT_PWM_GPIO,
        MOTOR_RIGHT_IN1_GPIO,
        MOTOR_RIGHT_IN2_GPIO,
        MOTOR_RIGHT_PWM_GPIO,
        MOTOR_REAR_IN1_GPIO,
        MOTOR_REAR_IN2_GPIO,
        MOTOR_REAR_PWM_GPIO,
    };

    for (size_t i = 0; i < sizeof(pins) / sizeof(pins[0]); ++i) {
        if (!GPIO_IS_VALID_OUTPUT_GPIO(pins[i])) {
            ESP_LOGE(TAG, "Required motor GPIO is invalid or still -1; edit main/board_pins.h");
            return ESP_ERR_INVALID_ARG;
        }
        for (size_t j = i + 1; j < sizeof(pins) / sizeof(pins[0]); ++j) {
            if (pins[i] == pins[j]) {
                ESP_LOGE(TAG, "GPIO %d is assigned to more than one motor signal", pins[i]);
                return ESP_ERR_INVALID_ARG;
            }
        }
    }

#if MOTOR_STBY_GPIO >= 0
    {
        if (!GPIO_IS_VALID_OUTPUT_GPIO(MOTOR_STBY_GPIO)) {
            ESP_LOGE(TAG, "Configured STBY GPIO %d is not a valid output", MOTOR_STBY_GPIO);
            return ESP_ERR_INVALID_ARG;
        }
        for (size_t i = 0; i < sizeof(pins) / sizeof(pins[0]); ++i) {
            if (MOTOR_STBY_GPIO == pins[i]) {
                ESP_LOGE(TAG, "STBY GPIO %d duplicates another motor signal", MOTOR_STBY_GPIO);
                return ESP_ERR_INVALID_ARG;
            }
        }
    }
#endif

    return ESP_OK;
}

static esp_err_t set_pwm_duty(ledc_channel_t channel, uint32_t duty)
{
    ESP_RETURN_ON_ERROR(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty), TAG,
                        "failed to set PWM duty");
    return ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

static esp_err_t configure_pwm_channel(ledc_channel_t channel, int gpio)
{
    const ledc_channel_config_t channel_config = {
        .gpio_num = gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
        .flags.output_invert = 0,
    };
    return ledc_channel_config(&channel_config);
}

static esp_err_t set_one_motor(const motor_channel_t *motor, int speed)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (speed > MOTOR_SPEED_MAX) {
        speed = MOTOR_SPEED_MAX;
    } else if (speed < -MOTOR_SPEED_MAX) {
        speed = -MOTOR_SPEED_MAX;
    }

    ESP_RETURN_ON_ERROR(set_pwm_duty(motor->pwm_channel, 0), TAG,
                        "failed to stop PWM before direction change");

    if (speed == 0) {
        gpio_set_level(motor->in1, 0);
        gpio_set_level(motor->in2, 0);
        return ESP_OK;
    }

    bool forward = speed > 0;
    if (motor->reversed) {
        forward = !forward;
    }

    gpio_set_level(motor->in1, forward ? 1 : 0);
    gpio_set_level(motor->in2, forward ? 0 : 1);

#if MOTOR_STBY_GPIO >= 0
    {
        gpio_set_level((gpio_num_t)MOTOR_STBY_GPIO, 1);
    }
#endif

    const uint32_t duty = (uint32_t)abs(speed) * MOTOR_PWM_MAX_DUTY / MOTOR_SPEED_MAX;
    return set_pwm_duty(motor->pwm_channel, duty);
}

esp_err_t motor_init(void)
{
    ESP_RETURN_ON_ERROR(validate_pin_configuration(), TAG, "unsafe motor pin configuration");

    const uint64_t direction_mask =
        (1ULL << (unsigned)s_left_motor.in1) |
        (1ULL << (unsigned)s_left_motor.in2) |
        (1ULL << (unsigned)s_right_motor.in1) |
        (1ULL << (unsigned)s_right_motor.in2) |
        (1ULL << (unsigned)s_rear_motor.in1) |
        (1ULL << (unsigned)s_rear_motor.in2);

    gpio_config_t direction_config = {
        .pin_bit_mask = direction_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&direction_config), TAG, "failed to configure direction GPIOs");

    gpio_set_level((gpio_num_t)MOTOR_LEFT_IN1_GPIO, 0);
    gpio_set_level((gpio_num_t)MOTOR_LEFT_IN2_GPIO, 0);
    gpio_set_level((gpio_num_t)MOTOR_RIGHT_IN1_GPIO, 0);
    gpio_set_level((gpio_num_t)MOTOR_RIGHT_IN2_GPIO, 0);
    gpio_set_level((gpio_num_t)MOTOR_REAR_IN1_GPIO, 0);
    gpio_set_level((gpio_num_t)MOTOR_REAR_IN2_GPIO, 0);

#if MOTOR_STBY_GPIO >= 0
    {
        const gpio_num_t standby_pin = (gpio_num_t)MOTOR_STBY_GPIO;
        gpio_config_t standby_config = {
            .pin_bit_mask = 1ULL << (unsigned)standby_pin,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_RETURN_ON_ERROR(gpio_config(&standby_config), TAG, "failed to configure STBY GPIO");
        gpio_set_level((gpio_num_t)MOTOR_STBY_GPIO, 0);
    }
#endif

    const ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = MOTOR_PWM_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_config), TAG, "failed to configure PWM timer");
    ESP_RETURN_ON_ERROR(configure_pwm_channel(LEDC_CHANNEL_0, MOTOR_LEFT_PWM_GPIO), TAG,
                        "failed to configure left PWM");
    ESP_RETURN_ON_ERROR(configure_pwm_channel(LEDC_CHANNEL_1, MOTOR_RIGHT_PWM_GPIO), TAG,
                        "failed to configure right PWM");
    ESP_RETURN_ON_ERROR(configure_pwm_channel(LEDC_CHANNEL_2, MOTOR_REAR_PWM_GPIO), TAG,
                        "failed to configure rear PWM");

    s_initialized = true;

#if MOTOR_STBY_GPIO >= 0
    {
        gpio_set_level((gpio_num_t)MOTOR_STBY_GPIO, 1);
    }
#endif

    ESP_LOGI(TAG, "Motor driver initialized; PWM=%d Hz", MOTOR_PWM_FREQUENCY_HZ);
    return motor_stop_all();
}

esp_err_t motor_set_left(int speed)
{
    return set_one_motor(&s_left_motor, speed);
}

esp_err_t motor_set_right(int speed)
{
    return set_one_motor(&s_right_motor, speed);
}

esp_err_t motor_set_rear(int speed)
{
    return set_one_motor(&s_rear_motor, speed);
}

esp_err_t motor_set_all(int left_speed, int right_speed, int rear_speed)
{
    ESP_RETURN_ON_ERROR(motor_set_left(left_speed), TAG, "failed to set left motor");
    ESP_RETURN_ON_ERROR(motor_set_right(right_speed), TAG, "failed to set right motor");
    return motor_set_rear(rear_speed);
}

esp_err_t motor_stop_all(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t left_result = set_one_motor(&s_left_motor, 0);
    esp_err_t right_result = set_one_motor(&s_right_motor, 0);
    esp_err_t rear_result = set_one_motor(&s_rear_motor, 0);

#if MOTOR_STBY_GPIO >= 0
    {
        gpio_set_level((gpio_num_t)MOTOR_STBY_GPIO, 0);
    }
#endif

    if (left_result != ESP_OK) {
        return left_result;
    }
    return right_result != ESP_OK ? right_result : rear_result;
}
