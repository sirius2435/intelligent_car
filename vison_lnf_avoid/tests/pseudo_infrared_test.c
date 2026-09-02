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

#define W 120

static vision_result_t make_vision(uint32_t sequence,
                                   bool line_found,
                                   int center_x,
                                   int lateral_error)
{
    vision_result_t result = (vision_result_t) {
        .frame_valid = true,
        .line_found = line_found,
        .confidence = line_found ? 1000U : 0U,
        .image_width = W,
        .image_height = 213,
        .lateral_error = lateral_error,
        .sequence = sequence,
    };
    if (line_found) {
        result.scan_center_x[0] = (uint16_t)center_x;
        result.scan_valid_mask = 0x01U;
    }
    return result;
}

static uint8_t sample_mask(uint32_t sequence,
                           bool line_found,
                           int center_x,
                           int lateral_error)
{
    const vision_result_t vision =
        make_vision(sequence, line_found, center_x, lateral_error);
    infrared_sensor_state_t state = {0};
    CHECK(pseudo_infrared_sample(&vision, &state) == ESP_OK);
    return state.black_mask;
}

static void test_center_maps_to_middle_channels(void)
{
    /* Centers: bit0=105 bit1=75 bit2=45 bit3=15; band=22. x=60 hits bits 1,2. */
    CHECK(sample_mask(1, true, 60, 0) == 0x06U);
}

static void test_right_and_left_extremes(void)
{
    CHECK(sample_mask(2, true, 105, 0) == IR_CHANNEL_1_MASK);   /* car right */
    CHECK(sample_mask(3, true, 15, 0) == IR_CHANNEL_4_MASK);    /* car left  */
    CHECK(sample_mask(4, true, 90, 0) ==
          (IR_CHANNEL_1_MASK | IR_CHANNEL_2_MASK));
    CHECK(sample_mask(5, true, 30, 0) ==
          (IR_CHANNEL_4_MASK | IR_CHANNEL_3_MASK));
}

static void test_missing_or_unreliable_line_is_white(void)
{
    CHECK(sample_mask(6, false, 0, 0) == 0x00U);

    vision_result_t vision =
        make_vision(7, true, 60, 0);
    vision.confidence = VISION_LINE_CONFIDENCE_MIN - 1U;
    infrared_sensor_state_t state = {0};
    CHECK(pseudo_infrared_sample(&vision, &state) == ESP_OK);
    CHECK(state.black_mask == 0x00U);
}

static void test_lateral_error_fallback_without_scan_rows(void)
{
    vision_result_t vision = make_vision(8, true, 0, 750);
    vision.scan_valid_mask = 0U;
    infrared_sensor_state_t state = {0};
    CHECK(pseudo_infrared_sample(&vision, &state) == ESP_OK);
    /* lateral_error +750 -> x = 60 + 750*60/1000 = 105 -> channel 1 (right). */
    CHECK(state.black_mask == IR_CHANNEL_1_MASK);

    vision = make_vision(9, true, 0, -750);
    vision.scan_valid_mask = 0U;
    CHECK(pseudo_infrared_sample(&vision, &state) == ESP_OK);
    CHECK(state.black_mask == IR_CHANNEL_4_MASK);
}

static void test_finish_marker_needs_confirm_then_all_black(void)
{
    vision_result_t vision = make_vision(10, true, 60, 0);
    vision.finish_marker = true;
    infrared_sensor_state_t state = {0};

    /* First two finish frames keep the normal centered mask. */
    CHECK(pseudo_infrared_sample(&vision, &state) == ESP_OK);
    CHECK(state.black_mask == 0x06U);
    vision.sequence = 11;
    CHECK(pseudo_infrared_sample(&vision, &state) == ESP_OK);
    CHECK(state.black_mask == 0x06U);

    /* Third consecutive finish frame confirms -> all black. */
    vision.sequence = 12;
    CHECK(pseudo_infrared_sample(&vision, &state) == ESP_OK);
    CHECK(state.black_mask == IR_ALL_BLACK_MASK);

    /* A later non-finish frame clears the confirmation. */
    vision.finish_marker = false;
    vision.sequence = 13;
    CHECK(pseudo_infrared_sample(&vision, &state) == ESP_OK);
    CHECK(state.black_mask == 0x06U);
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
    test_center_maps_to_middle_channels();
    test_right_and_left_extremes();
    test_missing_or_unreliable_line_is_white();
    test_lateral_error_fallback_without_scan_rows();
    test_finish_marker_needs_confirm_then_all_black();
    test_format_order_is_ch4_ch3_ch2_ch1();
    puts("pseudo infrared tests passed");
    return 0;
}
