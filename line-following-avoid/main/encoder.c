#include "encoder.h"

#include <stddef.h>

#include "board_config.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_check.h"
#include "esp_log.h"

#define ENCODER_PCNT_HIGH_LIMIT  32767
#define ENCODER_PCNT_LOW_LIMIT  (-32768)

static const char *TAG = "encoder";
static bool s_enabled;

typedef struct {
    int gpio_a;
    int gpio_b;
    bool reversed;
    const char *name;
    pcnt_unit_handle_t unit;
} encoder_channel_t;

static encoder_channel_t s_encoders[ENCODER_WHEEL_COUNT] = {
    [ENCODER_WHEEL_LEFT] = {
        .gpio_a = ENCODER_LEFT_A_GPIO,
        .gpio_b = ENCODER_LEFT_B_GPIO,
        .reversed = ENCODER_LEFT_REVERSED,
        .name = "left/Motor D",
    },
    [ENCODER_WHEEL_RIGHT] = {
        .gpio_a = ENCODER_RIGHT_A_GPIO,
        .gpio_b = ENCODER_RIGHT_B_GPIO,
        .reversed = ENCODER_RIGHT_REVERSED,
        .name = "right/Motor A",
    },
    [ENCODER_WHEEL_REAR] = {
        .gpio_a = ENCODER_REAR_A_GPIO,
        .gpio_b = ENCODER_REAR_B_GPIO,
        .reversed = ENCODER_REAR_REVERSED,
        .name = "rear/Motor B",
    },
};

static bool wheel_is_valid(encoder_wheel_t wheel)
{
    return wheel >= ENCODER_WHEEL_LEFT && wheel < ENCODER_WHEEL_COUNT;
}

static esp_err_t validate_pin_configuration(void)
{
    const int encoder_pins[] = {
        ENCODER_LEFT_A_GPIO, ENCODER_LEFT_B_GPIO,
        ENCODER_RIGHT_A_GPIO, ENCODER_RIGHT_B_GPIO,
        ENCODER_REAR_A_GPIO, ENCODER_REAR_B_GPIO,
    };
    const int reserved_pins[] = {
        MOTOR_LEFT_IN1_GPIO, MOTOR_LEFT_IN2_GPIO, MOTOR_LEFT_PWM_GPIO,
        MOTOR_RIGHT_IN1_GPIO, MOTOR_RIGHT_IN2_GPIO, MOTOR_RIGHT_PWM_GPIO,
        MOTOR_REAR_IN1_GPIO, MOTOR_REAR_IN2_GPIO, MOTOR_REAR_PWM_GPIO,
        IR_CHANNEL_1_GPIO, IR_CHANNEL_2_GPIO,
        IR_CHANNEL_3_GPIO, IR_CHANNEL_4_GPIO,
    };

    for (size_t i = 0; i < sizeof(encoder_pins) / sizeof(encoder_pins[0]); ++i) {
        if (!GPIO_IS_VALID_GPIO(encoder_pins[i])) {
            ESP_LOGE(TAG, "Encoder GPIO %d is not a valid input", encoder_pins[i]);
            return ESP_ERR_INVALID_ARG;
        }
        for (size_t j = i + 1; j < sizeof(encoder_pins) / sizeof(encoder_pins[0]); ++j) {
            if (encoder_pins[i] == encoder_pins[j]) {
                ESP_LOGE(TAG, "Encoder GPIO %d is assigned more than once", encoder_pins[i]);
                return ESP_ERR_INVALID_ARG;
            }
        }
        for (size_t j = 0; j < sizeof(reserved_pins) / sizeof(reserved_pins[0]); ++j) {
            if (encoder_pins[i] == reserved_pins[j]) {
                ESP_LOGE(TAG, "Encoder GPIO %d conflicts with another signal", encoder_pins[i]);
                return ESP_ERR_INVALID_ARG;
            }
        }
#if MOTOR_STBY_GPIO >= 0
        if (encoder_pins[i] == MOTOR_STBY_GPIO) {
            ESP_LOGE(TAG, "Encoder GPIO %d conflicts with motor STBY", encoder_pins[i]);
            return ESP_ERR_INVALID_ARG;
        }
#endif
    }
    return ESP_OK;
}

static esp_err_t configure_one_encoder(encoder_channel_t *encoder)
{
    const pcnt_unit_config_t unit_config = {
        .high_limit = ENCODER_PCNT_HIGH_LIMIT,
        .low_limit = ENCODER_PCNT_LOW_LIMIT,
        .flags.accum_count = 1,
    };
    ESP_RETURN_ON_ERROR(pcnt_new_unit(&unit_config, &encoder->unit), TAG,
                        "failed to allocate %s PCNT unit", encoder->name);

    const pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = ENCODER_GLITCH_FILTER_NS,
    };
    ESP_RETURN_ON_ERROR(pcnt_unit_set_glitch_filter(encoder->unit, &filter_config), TAG,
                        "failed to configure %s glitch filter", encoder->name);

    const pcnt_chan_config_t channel_a_config = {
        .edge_gpio_num = encoder->gpio_a,
        .level_gpio_num = encoder->gpio_b,
    };
    const pcnt_chan_config_t channel_b_config = {
        .edge_gpio_num = encoder->gpio_b,
        .level_gpio_num = encoder->gpio_a,
    };
    pcnt_channel_handle_t channel_a = NULL;
    pcnt_channel_handle_t channel_b = NULL;
    ESP_RETURN_ON_ERROR(pcnt_new_channel(encoder->unit, &channel_a_config, &channel_a), TAG,
                        "failed to allocate %s channel A", encoder->name);
    ESP_RETURN_ON_ERROR(pcnt_new_channel(encoder->unit, &channel_b_config, &channel_b), TAG,
                        "failed to allocate %s channel B", encoder->name);

    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_edge_action(channel_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                     PCNT_CHANNEL_EDGE_ACTION_INCREASE),
        TAG, "failed to configure %s A edges", encoder->name);
    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_level_action(channel_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                      PCNT_CHANNEL_LEVEL_ACTION_INVERSE),
        TAG, "failed to configure %s A level", encoder->name);
    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_edge_action(channel_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                     PCNT_CHANNEL_EDGE_ACTION_DECREASE),
        TAG, "failed to configure %s B edges", encoder->name);
    ESP_RETURN_ON_ERROR(
        pcnt_channel_set_level_action(channel_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                      PCNT_CHANNEL_LEVEL_ACTION_INVERSE),
        TAG, "failed to configure %s B level", encoder->name);

    ESP_RETURN_ON_ERROR(gpio_set_pull_mode((gpio_num_t)encoder->gpio_a, GPIO_PULLUP_ONLY), TAG,
                        "failed to enable %s A pull-up", encoder->name);
    ESP_RETURN_ON_ERROR(gpio_set_pull_mode((gpio_num_t)encoder->gpio_b, GPIO_PULLUP_ONLY), TAG,
                        "failed to enable %s B pull-up", encoder->name);
    ESP_RETURN_ON_ERROR(pcnt_unit_add_watch_point(encoder->unit, ENCODER_PCNT_HIGH_LIMIT), TAG,
                        "failed to add %s high limit", encoder->name);
    ESP_RETURN_ON_ERROR(pcnt_unit_add_watch_point(encoder->unit, ENCODER_PCNT_LOW_LIMIT), TAG,
                        "failed to add %s low limit", encoder->name);
    ESP_RETURN_ON_ERROR(pcnt_unit_enable(encoder->unit), TAG,
                        "failed to enable %s counter", encoder->name);
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(encoder->unit), TAG,
                        "failed to clear %s counter", encoder->name);
    return pcnt_unit_start(encoder->unit);
}

esp_err_t encoder_init(void)
{
    if (s_enabled) {
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(validate_pin_configuration(), TAG,
                        "unsafe encoder pin configuration");
    for (size_t i = 0; i < ENCODER_WHEEL_COUNT; ++i) {
        ESP_RETURN_ON_ERROR(configure_one_encoder(&s_encoders[i]), TAG,
                            "failed to initialize %s", s_encoders[i].name);
    }
    s_enabled = true;
    ESP_LOGI(TAG, "Three x4 quadrature encoders initialized");
    return ESP_OK;
}

bool encoder_is_enabled(void)
{
    return s_enabled;
}

esp_err_t encoder_get_count(encoder_wheel_t wheel, int *count)
{
    if (!s_enabled) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!wheel_is_valid(wheel) || count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_RETURN_ON_ERROR(pcnt_unit_get_count(s_encoders[wheel].unit, count), TAG,
                        "failed to read %s", s_encoders[wheel].name);
    if (s_encoders[wheel].reversed) {
        *count = -*count;
    }
    return ESP_OK;
}

esp_err_t encoder_get_all(int *left_count, int *right_count, int *rear_count)
{
    if (left_count == NULL || right_count == NULL || rear_count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_RETURN_ON_ERROR(encoder_get_count(ENCODER_WHEEL_LEFT, left_count), TAG,
                        "failed to read left encoder");
    ESP_RETURN_ON_ERROR(encoder_get_count(ENCODER_WHEEL_RIGHT, right_count), TAG,
                        "failed to read right encoder");
    return encoder_get_count(ENCODER_WHEEL_REAR, rear_count);
}

esp_err_t encoder_clear(encoder_wheel_t wheel)
{
    if (!s_enabled) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!wheel_is_valid(wheel)) {
        return ESP_ERR_INVALID_ARG;
    }
    return pcnt_unit_clear_count(s_encoders[wheel].unit);
}

esp_err_t encoder_clear_all(void)
{
    if (!s_enabled) {
        return ESP_ERR_INVALID_STATE;
    }
    for (size_t i = 0; i < ENCODER_WHEEL_COUNT; ++i) {
        ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(s_encoders[i].unit), TAG,
                            "failed to clear encoder counter");
    }
    return ESP_OK;
}