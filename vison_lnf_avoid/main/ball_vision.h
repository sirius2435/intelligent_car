#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Colored-ball / dark-pocket detector for the pocket-push task.
 *
 * Pure image analysis: no FreeRTOS, no hardware. It consumes one decoded
 * RGB888 frame (the vision task feeds it the push-mode image, 120x80 at
 * PUSH_CAMERA_DECODE_SCALE = 4) and returns up to one red ball, one blue
 * ball (chroma-dominant disk, see board_config.h BALL_BLUE_*) and up to two
 * dark pocket blobs in LOGICAL coordinates.
 *
 * Logical coordinates match the Wi-Fi viewer and pseudo_infrared.c: with the
 * current mounting (CAMERA_FLIP_HORIZONTAL/VERTICAL both 1) the raw decoder
 * buffer is a 180-degree rotation of what the viewer shows. Everything here
 * is expressed the way the viewer sees it: logical x grows to the viewer's
 * right, logical y=0 is the top (the far table edge / pockets), "left pocket"
 * is the one with the smaller logical x.
 *
 * Threading: ball_vision keeps static scratch buffers, so analyze() must be
 * called from a single task (the vision task). Calibration thresholds live in
 * board_config.h (BALL_*, POCKET_*, PUSH_CAMERA_DECODE_SCALE).
 */

typedef enum {
    BALL_COLOR_RED = 0,
    BALL_COLOR_BLUE,
    BALL_COLOR_COUNT,
} ball_color_t;

/* Only the four fields below are ever consumed: ball_push.c aims on cx/cy and
 * uses radius as its distance proxy, and the Wi-Fi viewer publishes cx/cy/
 * radius. The blob's area and bounding box were dropped because nothing read
 * them - the shape gates work on ball_vision_component_t inside ball_vision.c,
 * which is where the bbox actually lives. pocket_blob_t keeps its bbox because
 * the viewer draws it. */
typedef struct {
    bool present;
    int cx;       /* logical centroid, px */
    int cy;
    int radius;   /* sqrt(area / pi), px: DISTANCE PROXY (bigger = closer) */
} ball_blob_t;

typedef struct {
    bool visible;
    int cx;
    /* cy / area are not read by the push controller (it aims on cx alone); they
     * are asserted by tests/ball_vision_test.c to prove a detected pocket sits
     * in the far band and is big enough to be real. Free by-products of the
     * same accumulation pass - keep them. */
    int cy;
    unsigned area;
    int x0, y0, x1, y1;
} pocket_blob_t;

typedef struct {
    bool valid;        /* analyze ran on a usable frame */
    unsigned width;    /* logical dims == decoded dims */
    unsigned height;
    uint32_t frame_seq;/* stamped by the consumer (camera_vision) */
    ball_blob_t balls[BALL_COLOR_COUNT]; /* [RED] and [BLUE] slots */
    unsigned ball_count;   /* present-slot count; asserted by the host tests,
                            * the controller indexes balls[color] directly */
    pocket_blob_t pockets[2];             /* [0] left (smaller cx), [1] right */
    unsigned pocket_count;
    /* Per-frame classification counters. Cheap by-products of the single
     * classification pass; red_px / blue_px are reported by the main control
     * loop log, pocket_px is asserted by tests/ball_vision_test.c. */
    unsigned red_px;     /* pixels classified red */
    unsigned blue_px;    /* pixels classified blue */
    unsigned pocket_px;  /* pixels classified as pocket (dark, band only) */
} ball_vision_result_t;

esp_err_t ball_vision_analyze(const uint8_t *rgb,
                              unsigned width,
                              unsigned height,
                              size_t stride_bytes,
                              ball_vision_result_t *out);

/* Convenience lookup used by the push controller. Pockets are read straight
 * out of result->pockets[side] (0 = left / smaller logical x, 1 = right). */
const ball_blob_t *ball_vision_find_ball(const ball_vision_result_t *result,
                                         ball_color_t color);

#ifdef __cplusplus
}
#endif
