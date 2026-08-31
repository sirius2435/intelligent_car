#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* lateral_error and heading_error are normalized to -1000..1000.
 * Positive means the visible path is to the car's right. */
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
