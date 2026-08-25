#include "status_led.h"

#include "board_config.h"
#include "infrared_sensor.h"
#include "led_strip.h"

static led_strip_handle_t s_led_strip;
static uint8_t s_red;
static uint8_t s_green;
static uint8_t s_blue;

esp_err_t status_led_init(void)
{
    const led_strip_config_t strip_config = {
        .strip_gpio_num = STATUS_LED_GPIO,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags.invert_out = false,
    };
    const led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 64,
        .flags.with_dma = false,
    };
    return led_strip_new_rmt_device(&strip_config, &rmt_config, &s_led_strip);
}

esp_err_t status_led_set_enabled(bool enabled)
{
    if (s_led_strip == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!enabled) {
        return led_strip_clear(s_led_strip);
    }
    esp_err_t result = led_strip_set_pixel(s_led_strip, 0, s_red, s_green, s_blue);
    return result == ESP_OK ? led_strip_refresh(s_led_strip) : result;
}

esp_err_t status_led_show_sensor(uint8_t black_mask)
{
    if (black_mask == 0) {
        /* No black line is visible. */
        s_red = 40;
        s_green = 0;
        s_blue = 0;
    } else if (black_mask == 0x0F) {
        /* All channels are black: intersection or stop marker. */
        s_red = 25;
        s_green = 0;
        s_blue = 25;
    } else {
        static const int weights[4] = {3, 1, -1, -3}; /* bits 1..4, right to left */
        int weighted_sum = 0;
        int count = 0;
        for (unsigned bit = 0; bit < 4; ++bit) {
            if ((black_mask & (1U << bit)) != 0) {
                weighted_sum += weights[bit];
                ++count;
            }
        }
        const int error = weighted_sum / count;
        if (error <= -2) {
            s_red = 0;  s_green = 0;  s_blue = 40; /* line is far left */
        } else if (error < 0) {
            s_red = 0;  s_green = 25; s_blue = 25; /* line is slightly left */
        } else if (error == 0) {
            s_red = 0;  s_green = 40; s_blue = 0;  /* centered */
        } else if (error < 2) {
            s_red = 30; s_green = 25; s_blue = 0;  /* line is slightly right */
        } else {
            s_red = 35; s_green = 0;  s_blue = 15; /* line is far right */
        }
    }
    return status_led_set_enabled(true);
}
