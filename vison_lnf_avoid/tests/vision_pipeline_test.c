#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "board_config.h"
#include "vision_follow.h"
#include "vision_line.h"

#define TEST_WIDTH  120U
#define TEST_HEIGHT 213U
#define LINE_HALF_WIDTH 4

#define CHECK(condition) do {                                                   \
    if (!(condition)) {                                                         \
        fprintf(stderr, "FAIL %s:%d: %s\n", __func__, __LINE__, #condition);   \
        exit(1);                                                                \
    }                                                                           \
} while (0)

static uint8_t s_rgb[TEST_WIDTH * TEST_HEIGHT * 3U];

static void clear_white(void)
{
    memset(s_rgb, 255, sizeof(s_rgb));
}

static void set_logical_black(unsigned logical_x, unsigned logical_y)
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

static void draw_line(int near_center, int far_shift)
{
    clear_white();
    for (unsigned y = 0; y < TEST_HEIGHT; ++y) {
        const int shift = far_shift * (int)(TEST_HEIGHT - 1U - y) /
                          (int)(TEST_HEIGHT - 1U);
        const int center = near_center + shift;
        for (int offset = -LINE_HALF_WIDTH;
             offset <= LINE_HALF_WIDTH; ++offset) {
            const int x = center + offset;
            if (x >= 0 && x < (int)TEST_WIDTH) {
                set_logical_black((unsigned)x, y);
            }
        }
    }
}

static vision_result_t analyze(uint32_t sequence)
{
    vision_result_t result;
    vision_line_analyze_rgb888(s_rgb, TEST_WIDTH, TEST_HEIGHT,
                               TEST_WIDTH * 3U, &result);
    result.sequence = sequence;
    return result;
}

static void test_center_and_horizontal_direction(void)
{
    draw_line(60, 0);
    vision_result_t result = analyze(1);
    CHECK(result.frame_valid);
    CHECK(result.line_found);
    CHECK(result.confidence == 1000U);
    CHECK(abs(result.lateral_error) <= 20);
    CHECK(result.image_width == TEST_WIDTH);
    CHECK(result.image_height == TEST_HEIGHT);
    CHECK(result.scan_valid_mask == 0x3FU);
    for (unsigned row = 0; row < VISION_RESULT_SCAN_ROWS; ++row) {
        CHECK(result.scan_center_x[row] == 60U);
        if (row > 0U) {
            CHECK(result.scan_y[row] < result.scan_y[row - 1U]);
        }
    }

    draw_line(88, 0);
    result = analyze(2);
    CHECK(result.lateral_error > 300);

    vision_follow_controller_t follower;
    vision_follow_init(&follower);
    vision_follow_output_t output =
        vision_follow_update(&follower, &result, false, 10);
    CHECK(output.turn > 0);

    draw_line(32, 0);
    result = analyze(3);
    CHECK(result.lateral_error < -300);
    vision_follow_init(&follower);
    output = vision_follow_update(&follower, &result, false, 10);
    CHECK(output.turn < 0);
}

static void test_vertical_direction_preserves_heading(void)
{
    /* The path is near the center at the car and bends right in the distance.
       A vertical inversion bug changes the sign of heading_error. */
    draw_line(54, 120);
    const vision_result_t result = analyze(4);
    CHECK(result.line_found);
    CHECK(result.heading_error > 150);
}

static void test_missing_line_marks_all_scan_rows_invalid(void)
{
    clear_white();
    const vision_result_t result = analyze(5);
    CHECK(result.frame_valid);
    CHECK(!result.line_found);
    CHECK(result.valid_rows == 0U);
    CHECK(result.scan_valid_mask == 0U);
}

int main(void)
{
    CHECK(CAMERA_FLIP_HORIZONTAL == 1);
    CHECK(CAMERA_FLIP_VERTICAL == 1);
    test_center_and_horizontal_direction();
    test_vertical_direction_preserves_heading();
    test_missing_line_marks_all_scan_rows_invalid();
    puts("vision pipeline tests passed");
    return 0;
}
