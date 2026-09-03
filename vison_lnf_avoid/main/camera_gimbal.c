#include "camera_gimbal.h"

#include <stddef.h>

#include "board_config.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "camera_gimbal";

/* Dedicated LEDC resources: motor.c owns TIMER_0 / CHANNEL_0..2, so the
 * gimbal uses TIMER_1 and the next two channels. 50 Hz with a 14-bit duty
 * resolution gives ~1.2 us per step, far finer than an MG90S needs. */
#define GIMBAL_LEDC_TIMER    LEDC_TIMER_1
#define GIMBAL_PAN_CHANNEL   LEDC_CHANNEL_3
#define GIMBAL_TILT_CHANNEL  LEDC_CHANNEL_4
#define GIMBAL_DUTY_RES      LEDC_TIMER_14_BIT
#define GIMBAL_DUTY_FULL     (1U << 14)

#define GIMBAL_PAN_ENABLED   (CAMERA_PAN_SERVO_GPIO >= 0)
#define GIMBAL_TILT_ENABLED  (CAMERA_TILT_SERVO_GPIO >= 0)

static bool s_initialized;
static int s_pan_deg = CAMERA_PAN_SERVO_CENTER_DEG;
static int s_tilt_deg = CAMERA_TILT_SERVO_CENTER_DEG;

static int clamp_angle(int angle, int minimum, int maximum)
{
    if (angle < minimum) {
        return minimum;
    }
    return angle > maximum ? maximum : angle;
}

/* Shaft degrees (0..180) -> pulse width in microseconds. */
static uint32_t angle_to_pulse_us(int angle_deg)
{
    const int shaft = clamp_angle(angle_deg, 0, 180);
    return SERVO_PULSE_MIN_US + (uint32_t)shaft *
           (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US) / 180U;
}

static uint32_t pulse_to_duty(uint32_t pulse_us)
{
    const uint32_t period_us = 1000000U / SERVO_PWM_FREQ_HZ;
    return pulse_us * GIMBAL_DUTY_FULL / period_us;
}

static bool pin_is_free(int gpio)
{
    static const int occupied[] = {
        MOTOR_LEFT_IN1_GPIO, MOTOR_LEFT_IN2_GPIO, MOTOR_LEFT_PWM_GPIO,
        MOTOR_RIGHT_IN1_GPIO, MOTOR_RIGHT_IN2_GPIO, MOTOR_RIGHT_PWM_GPIO,
        MOTOR_REAR_IN1_GPIO, MOTOR_REAR_IN2_GPIO, MOTOR_REAR_PWM_GPIO,
        MOTOR_STBY_GPIO,
        ENCODER_LEFT_A_GPIO, ENCODER_LEFT_B_GPIO,
        ENCODER_RIGHT_A_GPIO, ENCODER_RIGHT_B_GPIO,
        ENCODER_REAR_A_GPIO, ENCODER_REAR_B_GPIO,
        ULTRASONIC_TRIG_GPIO, ULTRASONIC_ECHO_GPIO,
        IR_CHANNEL_1_GPIO, IR_CHANNEL_2_GPIO,
        IR_CHANNEL_3_GPIO, IR_CHANNEL_4_GPIO,
        LCD_CS_GPIO, LCD_SCK_GPIO, LCD_SDI_GPIO,
        LCD_DC_GPIO, LCD_RST_GPIO, LCD_BLK_GPIO,
        19, 20,  /* USB D-/D+, fixed internally for the UVC camera */
    };
    for (size_t i = 0; i < sizeof(occupied) / sizeof(occupied[0]); ++i) {
        if (occupied[i] >= 0 && gpio == occupied[i]) {
            return false;
        }
    }
    return true;
}

static esp_err_t validate_servo_pin(const char *axis, int gpio)
{
    if (!GPIO_IS_VALID_OUTPUT_GPIO(gpio)) {
        ESP_LOGE(TAG, "%s servo GPIO %d is not a valid output", axis, gpio);
        return ESP_ERR_INVALID_ARG;
    }
    if (!pin_is_free(gpio)) {
        ESP_LOGE(TAG, "%s servo GPIO %d conflicts with another signal",
                 axis, gpio);
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

static esp_err_t configure_channel(int gpio, ledc_channel_t channel)
{
    const ledc_channel_config_t config = {
        .gpio_num = gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = GIMBAL_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
        .flags.output_invert = 0,
    };
    return ledc_channel_config(&config);
}

static esp_err_t apply_axis(ledc_channel_t channel,
                            int angle_deg,
                            int minimum,
                            int maximum,
                            int *stored)
{
    const int clamped = clamp_angle(angle_deg, minimum, maximum);
    const uint32_t duty = pulse_to_duty(angle_to_pulse_us(clamped));
    ESP_RETURN_ON_ERROR(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty),
                        TAG, "failed to set servo duty");
    ESP_RETURN_ON_ERROR(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel),
                        TAG, "failed to apply servo duty");
    *stored = clamped;
    return ESP_OK;
}

esp_err_t camera_gimbal_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }
    if (!GIMBAL_PAN_ENABLED && !GIMBAL_TILT_ENABLED) {
        ESP_LOGW(TAG, "disabled: CAMERA_PAN_SERVO_GPIO and "
                 "CAMERA_TILT_SERVO_GPIO are -1; camera stays fixed");
        return ESP_OK;
    }

    if (GIMBAL_PAN_ENABLED) {
        ESP_RETURN_ON_ERROR(validate_servo_pin("pan", CAMERA_PAN_SERVO_GPIO),
                            TAG, "unsafe pan servo pin");
    }
    if (GIMBAL_TILT_ENABLED) {
        ESP_RETURN_ON_ERROR(validate_servo_pin("tilt", CAMERA_TILT_SERVO_GPIO),
                            TAG, "unsafe tilt servo pin");
    }
    if (GIMBAL_PAN_ENABLED && GIMBAL_TILT_ENABLED &&
        CAMERA_PAN_SERVO_GPIO == CAMERA_TILT_SERVO_GPIO) {
        ESP_LOGE(TAG, "pan and tilt share GPIO %d", CAMERA_PAN_SERVO_GPIO);
        return ESP_ERR_INVALID_ARG;
    }

    const ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = GIMBAL_DUTY_RES,
        .timer_num = GIMBAL_LEDC_TIMER,
        .freq_hz = SERVO_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_config), TAG,
                        "failed to configure servo PWM timer");

    if (GIMBAL_PAN_ENABLED) {
        ESP_RETURN_ON_ERROR(
            configure_channel(CAMERA_PAN_SERVO_GPIO, GIMBAL_PAN_CHANNEL),
            TAG, "failed to configure pan channel");
    }
    if (GIMBAL_TILT_ENABLED) {
        ESP_RETURN_ON_ERROR(
            configure_channel(CAMERA_TILT_SERVO_GPIO, GIMBAL_TILT_CHANNEL),
            TAG, "failed to configure tilt channel");
    }

    s_initialized = true;
    ESP_LOGI(TAG, "MG90S gimbal ready: pan=GPIO%d tilt=GPIO%d (%d Hz, %d-%d us)",
             CAMERA_PAN_SERVO_GPIO, CAMERA_TILT_SERVO_GPIO,
             SERVO_PWM_FREQ_HZ, SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);
    return camera_gimbal_center();
}

bool camera_gimbal_enabled(void)
{
    return s_initialized;
}

esp_err_t camera_gimbal_set_pan(int angle_deg)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!GIMBAL_PAN_ENABLED) {
        return ESP_OK;
    }
    return apply_axis(GIMBAL_PAN_CHANNEL, angle_deg,
                      CAMERA_PAN_SERVO_MIN_DEG, CAMERA_PAN_SERVO_MAX_DEG,
                      &s_pan_deg);
}

esp_err_t camera_gimbal_set_tilt(int angle_deg)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!GIMBAL_TILT_ENABLED) {
        return ESP_OK;
    }
    return apply_axis(GIMBAL_TILT_CHANNEL, angle_deg,
                      CAMERA_TILT_SERVO_MIN_DEG, CAMERA_TILT_SERVO_MAX_DEG,
                      &s_tilt_deg);
}

esp_err_t camera_gimbal_set(int pan_deg, int tilt_deg)
{
    const esp_err_t pan_result = camera_gimbal_set_pan(pan_deg);
    const esp_err_t tilt_result = camera_gimbal_set_tilt(tilt_deg);
    return pan_result != ESP_OK ? pan_result : tilt_result;
}

esp_err_t camera_gimbal_center(void)
{
    return camera_gimbal_set(CAMERA_PAN_SERVO_CENTER_DEG,
                             CAMERA_TILT_SERVO_CENTER_DEG);
}

void camera_gimbal_get_position(int *pan_deg, int *tilt_deg)
{
    if (pan_deg != NULL) {
        *pan_deg = s_pan_deg;
    }
    if (tilt_deg != NULL) {
        *tilt_deg = s_tilt_deg;
    }
}
