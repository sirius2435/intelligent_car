#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VISION_RESULT_SCAN_ROWS 6U

/* lateral_error and heading_error are normalized to -1000..1000.
 * Positive means the visible path is to the car's right. scan_y and
 * scan_center_x use the corrected logical image coordinates shown by the
 * Wi-Fi viewer; scan_valid_mask bit N marks row N as having a selected run. */
typedef struct {
    bool frame_valid;
    bool line_found;
    bool corner_detected;
    bool finish_marker;
    int lateral_error;
    int heading_error;
    unsigned confidence;
    unsigned valid_rows;
    unsigned line_width_pixels;
    uint16_t image_width;
    uint16_t image_height;
    uint16_t scan_y[VISION_RESULT_SCAN_ROWS];
    uint16_t scan_center_x[VISION_RESULT_SCAN_ROWS];
    uint8_t scan_valid_mask;
    uint32_t sequence;
    int64_t timestamp_us;
} vision_result_t;

void vision_line_analyze_rgb888(const uint8_t *rgb,
                                unsigned width,
                                unsigned height,
                                size_t stride_bytes,
                                vision_result_t *result);

#ifdef __cplusplus
}
#endif
