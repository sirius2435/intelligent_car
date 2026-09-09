#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BALL_COLOR_RED = 0,
    BALL_COLOR_WHITE,
} ball_color_t;

typedef struct {
    bool found;
    uint16_t center_x;
    uint16_t center_y;
    uint16_t width;
    uint16_t height;
    uint32_t area;
    uint8_t confidence;
} ball_vision_object_t;

typedef struct {
    uint32_t frame_sequence;
    uint16_t image_width;
    uint16_t image_height;
    ball_vision_object_t red_ball;
    ball_vision_object_t white_ball;
    ball_vision_object_t left_hole;
    ball_vision_object_t right_hole;
    /* Set when exactly one shape-valid pocket is visible: left/right identity
     * cannot be decided from a single frame, so left_hole/right_hole stay
     * unset and the caller may adopt this candidate by nearest-neighbour
     * continuity with the previous frame. */
    ball_vision_object_t lone_hole;
} ball_vision_result_t;

/* Workspace is three bytes per pixel and may live in PSRAM. */
size_t ball_vision_workspace_size(unsigned width, unsigned height);
esp_err_t ball_vision_analyze_rgb888(const uint8_t *rgb,
                                     unsigned width,
                                     unsigned height,
                                     size_t stride_bytes,
                                     uint32_t frame_sequence,
                                     void *workspace,
                                     size_t workspace_bytes,
                                     ball_vision_result_t *result);

#ifdef __cplusplus
}
#endif
