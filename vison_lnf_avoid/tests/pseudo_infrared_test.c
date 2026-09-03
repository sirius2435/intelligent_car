#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "board_config.h"
#include "pseudo_infrared.h"

#define CHECK(condition) do {                                                   \
    if (!(condition)) {                                                         \
        fprintf(stderr, "FAIL %s:%d: %s\n", __func__, __LINE__, #condition);   \
        exit(1);                                                                \
    }                                                                           \
} while (0)

#define TEST_WIDTH   120U
#define TEST_HEIGHT  213U

static uint8_t s_rgb[TEST_WIDTH * TEST_HEIGHT * 3U];

static void clear_white(void)
{
    memset(s_rgb, 255, sizeof(s_rgb));
}

/* Draw one logical pixel, applying the same flips as pseudo_infrared.c. */
static void set_black(unsigned logical_x, unsigned logical_y)
{
#if CAMERA_FLIP_HORIZONTAL
    const unsigned source_x = TEST_WIDTH - 1U - logical_x;
#else
    const unsigned source_x = logical_x;
#endif
#if CAMERA_FLIP_VERTICAL
    const unsigned source_y = TEST_HEIGHT - 1U - logical_y;
#else
    const unsigned source_y = logical_y;
#endif
    uint8_t *pixel = s_rgb + (source_y * TEST_WIDTH + source_x) * 3U;
    pixel[0] = 0;
    pixel[1] = 0;
    pixel[2] = 0;
}

/* Fill logical x in [x0, x1] over the full height with black. */
static void fill_band(unsigned x0, unsigned x1)
{
    for (unsigned y = 0; y < TEST_HEIGHT; ++y) {
        for (unsigned x = x0; x <= x1; ++x) {
            set_black(x, y);
        }
    }
}

static uint8_t sample_mask(void)
{
    infrared_sensor_state_t state = {0};
    CHECK(pseudo_infrared_sample_rgb888(s_rgb, TEST_WIDTH, TEST_HEIGHT,
                                        TEST_WIDTH * 3U, &state) == ESP_OK);
    return state.black_mask;
}

static void test_four_fixed_channels(void)
{
    /* Channel centers: 7/8, 5/8, 3/8, 1/8 of width = 105, 75, 45, 15. */
    clear_white();
    fill_band(102, 108);                 /* car right -> bit 0 */
    CHECK(sample_mask() == IR_CHANNEL_1_MASK);

    clear_white();
    fill_band(72, 78);                   /* center right -> bit 1 */
    CHECK(sample_mask() == IR_CHANNEL_2_MASK);

    clear_white();
    fill_band(42, 48);                   /* center left -> bit 2 */
    CHECK(sample_mask() == IR_CHANNEL_3_MASK);

    clear_white();
    fill_band(12, 18);                   /* car left -> bit 3 */
    CHECK(sample_mask() == IR_CHANNEL_4_MASK);
}

static void test_line_on_boundary_lights_two_channels(void)
{
    /* A wide line straddling channels 2 and 3 lights both -> 0x06. */
    clear_white();
    fill_band(40, 80);
    CHECK(sample_mask() == (IR_CHANNEL_2_MASK | IR_CHANNEL_3_MASK));
}

static void test_all_white_is_no_line(void)
{
    clear_white();
    CHECK(sample_mask() == 0x00U);
}

static void test_finish_needs_confirm_then_all_black(void)
{
    /* Wide bar over channels 4..2 (0..90) leaves channel 1 (105) white, so the
     * raw mask is 0x0E. After PSEUDO_IR_FINISH_CONFIRM_FRAMES frames the
     * finish confirmation upgrades it to all-black 0x0F. */
    pseudo_infrared_reset();
    clear_white();
    fill_band(0, 90);

    CHECK(sample_mask() == 0x0EU);
    CHECK(sample_mask() == 0x0EU);
    CHECK(sample_mask() == IR_ALL_BLACK_MASK);
}

static void test_finish_confirmation_resets(void)
{
    pseudo_infrared_reset();
    clear_white();
    fill_band(0, 90);
    CHECK(sample_mask() == 0x0EU);   /* first candidate frame: not yet all black */

    clear_white();                   /* finish bar gone -> confirmation resets */
    fill_band(72, 78);
    CHECK(sample_mask() == IR_CHANNEL_2_MASK);
}

static void test_format_order_is_ch4_ch3_ch2_ch1(void)
{
    char output[5];
    pseudo_infrared_format(0x09U, output);  /* CH4 + CH1 black */
    CHECK(strcmp(output, "BWWB") == 0);
    pseudo_infrared_format(0x06U, output);  /* CH3 + CH2 black */
    CHECK(strcmp(output, "WBBW") == 0);
    pseudo_infrared_format(0x00U, output);
    CHECK(strcmp(output, "WWWW") == 0);
    pseudo_infrared_format(0x0FU, output);
    CHECK(strcmp(output, "BBBB") == 0);
}

int main(void)
{
    CHECK(CAMERA_FLIP_HORIZONTAL == 1);
    CHECK(CAMERA_FLIP_VERTICAL == 1);
    test_four_fixed_channels();
    test_line_on_boundary_lights_two_channels();
    test_all_white_is_no_line();
    test_finish_needs_confirm_then_all_black();
    test_finish_confirmation_resets();
    test_format_order_is_ch4_ch3_ch2_ch1();
    puts("pseudo infrared tests passed");
    return 0;
}
