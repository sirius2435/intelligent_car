#include "vision_line.h"

#include <limits.h>
#include <stdlib.h>

#include "board_config.h"

typedef struct {
    bool valid;
    int center;
    unsigned width;
    unsigned dark_pixels;
    unsigned longest_run;
} row_measurement_t;

static int clamp_int(int value, int minimum, int maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static uint8_t pixel_luma(const uint8_t *pixel)
{
    return (uint8_t)(((unsigned)pixel[0] * 77U +
                      (unsigned)pixel[1] * 150U +
                      (unsigned)pixel[2] * 29U) >> 8U);
}

static row_measurement_t measure_row(const uint8_t *row,
                                     unsigned width,
                                     int expected_center)
{
    uint32_t luma_sum = 0;
    for (unsigned x = 0; x < width; ++x) {
        luma_sum += pixel_luma(row + x * 3U);
    }
    const int mean = width == 0 ? 0 : (int)(luma_sum / width);
    const int threshold = clamp_int(mean - VISION_BLACK_MARGIN, 12, 220);
    const unsigned minimum_width =
        (width * VISION_MIN_LINE_WIDTH_PERCENT + 99U) / 100U;
    const unsigned maximum_width =
        (width * VISION_MAX_LINE_WIDTH_PERCENT) / 100U;

    row_measurement_t measurement = {0};
    int best_distance = INT_MAX;
    unsigned run_start = 0;
    bool in_run = false;
    for (unsigned x = 0; x <= width; ++x) {
        const bool dark = x < width && pixel_luma(row + x * 3U) <= threshold;
        if (dark) {
            ++measurement.dark_pixels;
            if (!in_run) {
                run_start = x;
                in_run = true;
            }
        }
        if ((!dark || x == width) && in_run) {
            const unsigned run_width = x - run_start;
            if (run_width > measurement.longest_run) {
                measurement.longest_run = run_width;
            }
            if (run_width >= minimum_width && run_width <= maximum_width) {
                const int center = (int)(run_start + run_width / 2U);
                const int distance = abs(center - expected_center);
                if (!measurement.valid || distance < best_distance ||
                    (distance == best_distance && run_width > measurement.width)) {
                    measurement.valid = true;
                    measurement.center = center;
                    measurement.width = run_width;
                    best_distance = distance;
                }
            }
            in_run = false;
        }
    }
    return measurement;
}

void vision_line_analyze_rgb888(const uint8_t *rgb,
                                unsigned width,
                                unsigned height,
                                size_t stride_bytes,
                                vision_result_t *result)
{
    if (result == NULL) {
        return;
    }
    *result = (vision_result_t) {0};
    if (rgb == NULL || width < 32U || height < 24U ||
        stride_bytes < width * 3U) {
        return;
    }

    const unsigned roi_top = height * VISION_ROI_TOP_PERCENT / 100U;
    const unsigned roi_bottom = height * VISION_ROI_BOTTOM_PERCENT / 100U;
    if (roi_bottom <= roi_top || VISION_SCAN_ROW_COUNT < 2) {
        return;
    }

    row_measurement_t rows[VISION_SCAN_ROW_COUNT];
    int expected_center = (int)width / 2;
    unsigned width_sum = 0;
    unsigned finish_rows = 0;
    unsigned valid_rows = 0;
    int center_sum = 0;
    int near_center = expected_center;
    int far_center = expected_center;
    bool have_near = false;
    bool have_far = false;

    /* Measure from the near field upwards so a corner branch is associated
     * with the line already under the car rather than a remote dark object. */
    for (unsigned index = 0; index < VISION_SCAN_ROW_COUNT; ++index) {
        const unsigned span = roi_bottom - roi_top;
        const unsigned y = roi_bottom - 1U -
            index * (span - 1U) / (VISION_SCAN_ROW_COUNT - 1U);
        rows[index] = measure_row(rgb + y * stride_bytes, width, expected_center);
        if (rows[index].longest_run * 100U >=
            width * VISION_FINISH_WIDTH_PERCENT) {
            ++finish_rows;
        }
        if (!rows[index].valid) {
            continue;
        }
        expected_center = rows[index].center;
        center_sum += rows[index].center;
        width_sum += rows[index].width;
        ++valid_rows;
        if (!have_near) {
            near_center = rows[index].center;
            have_near = true;
        }
        far_center = rows[index].center;
        have_far = true;
    }

    result->frame_valid = true;
    result->valid_rows = valid_rows;
    result->finish_marker = finish_rows >= 2U;
    if (valid_rows < 3U) {
        return;
    }

    const int average_center = center_sum / (int)valid_rows;
    const int half_width = (int)width / 2;
    result->line_found = true;
    result->lateral_error = clamp_int(
        (average_center - half_width) * 1000 / half_width, -1000, 1000);
    if (have_near && have_far) {
        result->heading_error = clamp_int(
            (far_center - near_center) * 1000 / half_width, -1000, 1000);
    }
    result->line_width_pixels = width_sum / valid_rows;

    unsigned confidence = valid_rows * 1000U / VISION_SCAN_ROW_COUNT;
    const unsigned average_width = result->line_width_pixels;
    unsigned width_deviation = 0;
    for (unsigned index = 0; index < VISION_SCAN_ROW_COUNT; ++index) {
        if (rows[index].valid) {
            width_deviation += rows[index].width > average_width ?
                rows[index].width - average_width : average_width - rows[index].width;
        }
    }
    if (valid_rows > 0U && average_width > 0U) {
        const unsigned relative_deviation =
            width_deviation * 100U / (valid_rows * average_width);
        if (relative_deviation > 50U) {
            confidence = confidence * 3U / 4U;
        }
    }
    result->confidence = confidence > 1000U ? 1000U : confidence;
    result->corner_detected =
        abs(result->heading_error) >= VISION_CORNER_HEADING_THRESHOLD;
}
