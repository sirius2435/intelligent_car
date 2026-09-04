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

/* The real decoded image is 60 x 40 px (CAMERA_FRAME 480x320 / SCALE 8). The
 * percent centres land on different pixels there than on the 2x-wide test
 * image, so the block geometry is also checked once at the real size. */
#define REAL_WIDTH    60U
#define REAL_HEIGHT   40U

static uint8_t s_rgb[TEST_WIDTH * TEST_HEIGHT * 3U];
static unsigned s_width = TEST_WIDTH;
static unsigned s_height = TEST_HEIGHT;

static void use_real_size(void)
{
    s_width = REAL_WIDTH;
    s_height = REAL_HEIGHT;
}

static void use_test_size(void)
{
    s_width = TEST_WIDTH;
    s_height = TEST_HEIGHT;
}

static void clear_white(void)
{
    memset(s_rgb, 255, (size_t)s_width * s_height * 3U);
}

/* Draw one logical pixel, applying the same flips as pseudo_infrared.c. */
static void set_black(unsigned logical_x, unsigned logical_y)
{
#if CAMERA_FLIP_HORIZONTAL
    const unsigned source_x = s_width - 1U - logical_x;
#else
    const unsigned source_x = logical_x;
#endif
#if CAMERA_FLIP_VERTICAL
    const unsigned source_y = s_height - 1U - logical_y;
#else
    const unsigned source_y = logical_y;
#endif
    uint8_t *pixel = s_rgb + (source_y * s_width + source_x) * 3U;
    pixel[0] = 0;
    pixel[1] = 0;
    pixel[2] = 0;
}

/* Fill logical x in [x0, x1] over the full height with black. */
static void fill_band(unsigned x0, unsigned x1)
{
    for (unsigned y = 0; y < s_height; ++y) {
        for (unsigned x = x0; x <= x1; ++x) {
            set_black(x, y);
        }
    }
}

static uint8_t sample_mask(void)
{
    infrared_sensor_state_t state = {0};
    CHECK(pseudo_infrared_sample_rgb888(s_rgb, s_width, s_height,
                                        (size_t)s_width * 3U, &state) == ESP_OK);
    return state.black_mask;
}

static void test_four_fixed_channels(void)
{
    /* Channel centers: 65%, 54%, 47%, 35% of width 120 = 78, 64, 56, 42. */
    clear_white();
    fill_band(77, 79);                   /* car right -> bit 0 */
    CHECK(sample_mask() == IR_CHANNEL_1_MASK);

    clear_white();
    fill_band(63, 65);                   /* center right -> bit 1 */
    CHECK(sample_mask() == IR_CHANNEL_2_MASK);

    clear_white();
    fill_band(55, 57);                   /* center left -> bit 2 */
    CHECK(sample_mask() == IR_CHANNEL_3_MASK);

    clear_white();
    fill_band(41, 43);                   /* car left -> bit 3 */
    CHECK(sample_mask() == IR_CHANNEL_4_MASK);
}

static void test_line_on_boundary_lights_two_channels(void)
{
    /* A wide line straddling the two inner channels lights both -> 0x06.
     * On the real 60 px wide image the inner blocks share the exact centre,
     * so a dead-centre line reads 0x06 and mask_to_error returns 0. */
    clear_white();
    fill_band(52, 68);
    CHECK(sample_mask() == (IR_CHANNEL_2_MASK | IR_CHANNEL_3_MASK));
}

static void test_all_white_is_no_line(void)
{
    clear_white();
    CHECK(sample_mask() == 0x00U);
}

static void test_finish_bar_lights_every_channel(void)
{
    /* CH1/CH4 moved inward to 65%/35% (blocks 76-80 and 40-44 here), so the
     * four blocks span 40..80 of the 120 px row. A dark run wide enough to be
     * a finish bar (>= PSEUDO_IR_FINISH_WIDTH_PERCENT -> 84 px) can no longer
     * slip past an outer block, so the raw mask is already 0x0F on the first
     * frame and PSEUDO_IR_FINISH_CONFIRM_FRAMES is only a backstop. */
    pseudo_infrared_reset();
    clear_white();
    fill_band(0, 84);

    CHECK(sample_mask() == IR_ALL_BLACK_MASK);
    CHECK(sample_mask() == IR_ALL_BLACK_MASK);
}

static void test_finish_confirmation_resets(void)
{
    pseudo_infrared_reset();
    clear_white();
    fill_band(0, 84);
    CHECK(sample_mask() == IR_ALL_BLACK_MASK);

    clear_white();                   /* finish bar gone -> confirmation resets */
    fill_band(63, 65);
    CHECK(sample_mask() == IR_CHANNEL_2_MASK);
}

static void test_real_image_block_geometry(void)
{
    /* On the real 60 px wide decoded image the centres are 39, 32, 28, 21 and
     * the 5 px blocks are 37-41, 30-34, 26-30, 19-23: a 22 px aperture, still
     * symmetric about x = 30, with only the 2 px gaps 35-36 and 24-25 between
     * an outer block and its inner neighbour. */
    use_real_size();
    pseudo_infrared_reset();

    clear_white();
    fill_band(29, 30);                   /* dead centre -> both inner blocks */
    CHECK(sample_mask() == (IR_CHANNEL_2_MASK | IR_CHANNEL_3_MASK));

    clear_white();
    fill_band(39, 39);                   /* car right outer -> CH1 only */
    CHECK(sample_mask() == IR_CHANNEL_1_MASK);

    clear_white();
    fill_band(21, 21);                   /* car left outer -> CH4 only */
    CHECK(sample_mask() == IR_CHANNEL_4_MASK);

    clear_white();
    fill_band(35, 36);                   /* inside the CH1/CH2 gap -> no line */
    CHECK(sample_mask() == 0x00U);

    clear_white();
    fill_band(34, 37);                   /* wide enough to span the gap */
    CHECK(sample_mask() == (IR_CHANNEL_1_MASK | IR_CHANNEL_2_MASK));

    use_test_size();
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
    test_finish_bar_lights_every_channel();
    test_finish_confirmation_resets();
    test_real_image_block_geometry();
    test_format_order_is_ch4_ch3_ch2_ch1();
    puts("pseudo infrared tests passed");
    return 0;
}
