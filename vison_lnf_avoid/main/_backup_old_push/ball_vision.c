#include "ball_vision.h"

#include <limits.h>
#include <string.h>

#include "board_config.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "ball_vision";

typedef struct {
    uint32_t area;
    uint32_t sum_x;
    uint32_t sum_y;
    unsigned min_x;
    unsigned max_x;
    unsigned min_y;
    unsigned max_y;
} component_t;

/* Largest dark component rejected by the hole shape gate in the latest scan;
 * reported once per second so floor calibration can read which criterion
 * (size, aspect, fill) is killing the real pocket. */
static component_t s_shape_rejected;
static int64_t s_diag_us;

static uint8_t luma(const uint8_t *p)
{
    return (uint8_t)(((unsigned)p[0] * 77U + (unsigned)p[1] * 150U +
                      (unsigned)p[2] * 29U) >> 8U);
}

static const uint8_t *logical_pixel(const uint8_t *rgb, unsigned width,
                                    unsigned height, size_t stride,
                                    unsigned x, unsigned y)
{
#if CAMERA_FLIP_HORIZONTAL
    x = width - 1U - x;
#endif
#if CAMERA_FLIP_VERTICAL
    y = height - 1U - y;
#endif
    return rgb + (size_t)y * stride + (size_t)x * 3U;
}

size_t ball_vision_workspace_size(unsigned width, unsigned height)
{
    const size_t pixels = (size_t)width * height;
    if (width == 0U || height == 0U || pixels > UINT16_MAX) {
        return 0U;
    }
    const size_t mask_bytes = pixels + (pixels & 1U);
    return mask_bytes + pixels * sizeof(uint16_t);
}

static void object_from_component(ball_vision_object_t *out,
                                  const component_t *c,
                                  unsigned image_area)
{
    const unsigned w = c->max_x - c->min_x + 1U;
    const unsigned h = c->max_y - c->min_y + 1U;
    unsigned confidence = image_area == 0U ? 0U :
        (unsigned)(c->area * 400U / image_area);
    if (confidence > 100U) {
        confidence = 100U;
    }
    *out = (ball_vision_object_t) {
        .found = true,
        .center_x = (uint16_t)(c->sum_x / c->area),
        .center_y = (uint16_t)(c->sum_y / c->area),
        .width = (uint16_t)w,
        .height = (uint16_t)h,
        .area = c->area,
        .confidence = (uint8_t)confidence,
    };
}

static bool component_shape_ok(const component_t *c, unsigned min_area,
                               unsigned max_area, unsigned min_fill,
                               unsigned aspect_num, unsigned aspect_den)
{
    if (c->area < min_area || c->area > max_area) {
        return false;
    }
    const unsigned w = c->max_x - c->min_x + 1U;
    const unsigned h = c->max_y - c->min_y + 1U;
    if (w * aspect_den > h * aspect_num ||
        h * aspect_den > w * aspect_num) {
        return false;
    }
    return c->area * 100U >= w * h * min_fill;
}

static bool next_component(uint8_t *mask, uint16_t *queue,
                           unsigned width, unsigned height, size_t start,
                           component_t *out, size_t *next_start)
{
    const size_t pixels = (size_t)width * height;
    while (start < pixels && mask[start] != 1U) {
        ++start;
    }
    if (start == pixels) {
        *next_start = pixels;
        return false;
    }
    size_t head = 0U;
    size_t tail = 0U;
    queue[tail++] = (uint16_t)start;
    mask[start] = 2U;
    component_t c = {
        .min_x = width, .min_y = height,
    };
    while (head < tail) {
        const size_t index = queue[head++];
        const unsigned x = (unsigned)(index % width);
        const unsigned y = (unsigned)(index / width);
        ++c.area;
        c.sum_x += x;
        c.sum_y += y;
        if (x < c.min_x) c.min_x = x;
        if (x > c.max_x) c.max_x = x;
        if (y < c.min_y) c.min_y = y;
        if (y > c.max_y) c.max_y = y;
        const size_t neighbours[4] = {
            x > 0U ? index - 1U : pixels,
            x + 1U < width ? index + 1U : pixels,
            y > 0U ? index - width : pixels,
            y + 1U < height ? index + width : pixels,
        };
        for (unsigned i = 0; i < 4U; ++i) {
            const size_t n = neighbours[i];
            if (n < pixels && mask[n] == 1U) {
                mask[n] = 2U;
                queue[tail++] = (uint16_t)n;
            }
        }
    }
    *out = c;
    *next_start = start + 1U;
    return true;
}

static void find_largest(uint8_t *mask, uint16_t *queue,
                         unsigned width, unsigned height,
                         unsigned min_area, unsigned max_area,
                         unsigned min_fill, unsigned aspect_num,
                         unsigned aspect_den, ball_vision_object_t *out)
{
    component_t best = {0};
    size_t start = 0U;
    component_t c;
    while (next_component(mask, queue, width, height, start, &c, &start)) {
        if (component_shape_ok(&c, min_area, max_area, min_fill,
                               aspect_num, aspect_den) && c.area > best.area) {
            best = c;
        }
    }
    if (best.area > 0U) {
        object_from_component(out, &best, width * height);
    }
}

static void build_red_mask(const uint8_t *rgb, unsigned width, unsigned height,
                           size_t stride, uint8_t *mask)
{
    for (unsigned y = 0; y < height; ++y) {
        for (unsigned x = 0; x < width; ++x) {
            const uint8_t *p = logical_pixel(rgb, width, height, stride, x, y);
            mask[(size_t)y * width + x] =
                p[0] >= BALL_RED_R_MIN &&
                (int)p[0] - (int)p[1] >= BALL_RED_DOMINANCE &&
                (int)p[0] - (int)p[2] >= BALL_RED_DOMINANCE;
        }
    }
}

static void build_white_mask(const uint8_t *rgb, unsigned width, unsigned height,
                             size_t stride, uint8_t *mask)
{
    memset(mask, 0, (size_t)width * height);
    const unsigned r = BALL_WHITE_RING_RADIUS;
    if (width <= 2U * r || height <= 2U * r) return;
    static const int8_t dx[8] = {-1, 0, 1, 1, 1, 0, -1, -1};
    static const int8_t dy[8] = {-1, -1, -1, 0, 1, 1, 1, 0};
    for (unsigned y = r; y + r < height; ++y) {
        for (unsigned x = r; x + r < width; ++x) {
            const uint8_t *p = logical_pixel(rgb, width, height, stride, x, y);
            const unsigned maximum = p[0] > p[1] ?
                (p[0] > p[2] ? p[0] : p[2]) : (p[1] > p[2] ? p[1] : p[2]);
            const unsigned minimum = p[0] < p[1] ?
                (p[0] < p[2] ? p[0] : p[2]) : (p[1] < p[2] ? p[1] : p[2]);
            const unsigned center = luma(p);
            if (center < BALL_WHITE_CENTER_LUMA ||
                maximum - minimum > BALL_WHITE_MAX_CHROMA) continue;
            unsigned ring = 0U;
            bool ring_touches_black = false;
            for (unsigned i = 0; i < 8U; ++i) {
                const uint8_t sample = luma(logical_pixel(rgb, width, height,
                                                          stride,
                    (unsigned)((int)x + dx[i] * (int)r),
                    (unsigned)((int)y + dy[i] * (int)r)));
                ring += sample;
                if (sample <= BALL_HOLE_MAX_LUMA) {
                    ring_touches_black = true;
                }
            }
            /* A bright pixel whose contrast ring reaches a pocket is just the
             * white collar around a hole, not a ball: its "dark ring" is the
             * pocket itself, so veto it (a real ball never sits on black). */
            mask[(size_t)y * width + x] =
                !ring_touches_black &&
                center >= ring / 8U + BALL_WHITE_LOCAL_CONTRAST;
        }
    }
}

/* Absolute dark threshold plus a 3x3 local-density pass. The density pass
 * removes one/two-pixel track lines before connected components are formed
 * and separates a black pocket from a thin line that physically touches its
 * edge. `raw` aliases the queue storage, which is only scratch afterwards. */
static void build_hole_mask(const uint8_t *rgb, unsigned width,
                            unsigned height, size_t stride,
                            uint8_t *raw, uint8_t *mask)
{
    for (unsigned y = 0; y < height; ++y) {
        for (unsigned x = 0; x < width; ++x) {
            raw[(size_t)y * width + x] =
                luma(logical_pixel(rgb, width, height, stride, x, y)) <=
                BALL_HOLE_MAX_LUMA;
        }
    }
    memset(mask, 0, (size_t)width * height);
    for (unsigned y = 1U; y + 1U < height; ++y) {
        for (unsigned x = 1U; x + 1U < width; ++x) {
            unsigned dark = 0U;
            for (unsigned yy = y - 1U; yy <= y + 1U; ++yy)
                for (unsigned xx = x - 1U; xx <= x + 1U; ++xx)
                    dark += raw[(size_t)yy * width + xx] != 0U;
            mask[(size_t)y * width + x] = dark >= 7U;
        }
    }
}

/* Keeps the two largest shape-valid dark components whose centroid lies
 * above max_y (exclusive). */
static void collect_holes(uint8_t *mask, uint16_t *queue,
                          unsigned width, unsigned height, unsigned max_y,
                          component_t best[2])
{
    size_t start = 0U;
    component_t c;
    while (next_component(mask, queue, width, height, start, &c, &start)) {
        const bool shape_ok = component_shape_ok(&c, BALL_HOLE_MIN_AREA,
                                                 BALL_HOLE_MAX_AREA,
                                                 BALL_HOLE_MIN_FILL_PERCENT,
                                                 BALL_HOLE_MAX_ASPECT_NUM,
                                                 BALL_HOLE_MAX_ASPECT_DEN);
        if (!shape_ok) {
            if (c.area >= BALL_HOLE_MIN_AREA && c.area > s_shape_rejected.area) {
                s_shape_rejected = c;
            }
            continue;
        }
        if (c.sum_y / c.area >= max_y) {
            continue;
        }
        if (c.area > best[0].area) {
            best[1] = best[0];
            best[0] = c;
        } else if (c.area > best[1].area) {
            best[1] = c;
        }
    }
}

static void find_holes(const uint8_t *rgb, unsigned width, unsigned height,
                       size_t stride, uint8_t *mask, uint16_t *queue,
                       ball_vision_object_t *left, ball_vision_object_t *right,
                       ball_vision_object_t *lone)
{
    /* The pockets lie beyond the ball, i.e. in the upper part of the frame,
     * while the chassis, its shadow and floor cables live at the bottom:
     * first accept only centroids above BALL_HOLE_SEARCH_MAX_Y_PERCENT. The
     * pocket image sinks as the car approaches, so a scan that finds fewer
     * than two holes is repeated over the full frame. */
    uint8_t *raw = (uint8_t *)queue;
    component_t best[2] = {{0}, {0}};
    s_shape_rejected = (component_t) {0};
    build_hole_mask(rgb, width, height, stride, raw, mask);
    collect_holes(mask, queue, width, height,
                  height * BALL_HOLE_SEARCH_MAX_Y_PERCENT / 100U, best);
    if (best[0].area == 0U || best[1].area == 0U) {
        build_hole_mask(rgb, width, height, stride, raw, mask);
        best[0] = (component_t) {0};
        best[1] = (component_t) {0};
        collect_holes(mask, queue, width, height, height, best);
    }
    const int64_t now = esp_timer_get_time();
    if (s_shape_rejected.area != 0U && now - s_diag_us >= 1000000LL) {
        s_diag_us = now;
        const unsigned w = s_shape_rejected.max_x - s_shape_rejected.min_x + 1U;
        const unsigned h = s_shape_rejected.max_y - s_shape_rejected.min_y + 1U;
        ESP_LOGI(TAG,
                 "hole shape-rejected: area=%u wxh=%ux%u fill=%u%% cy=%u%%",
                 (unsigned)s_shape_rejected.area, w, h,
                 (unsigned)(s_shape_rejected.area * 100U / (w * h)),
                 (unsigned)((s_shape_rejected.sum_y / s_shape_rejected.area) *
                            100U / height));
    }
    if (best[0].area && best[1].area) {
        const component_t *l = best[0].sum_x * best[1].area <=
                               best[1].sum_x * best[0].area ? &best[0] : &best[1];
        const component_t *r = l == &best[0] ? &best[1] : &best[0];
        object_from_component(left, l, width * height);
        object_from_component(right, r, width * height);
    } else if (best[0].area != 0U) {
        /* One shape-valid pocket alone carries no left/right identity; hand it
         * to the caller as a lone candidate so an identity established while
         * both pockets were visible can be continued by continuity. */
        object_from_component(lone, &best[0], width * height);
    }
}

esp_err_t ball_vision_analyze_rgb888(const uint8_t *rgb, unsigned width,
                                     unsigned height, size_t stride,
                                     uint32_t sequence, void *workspace,
                                     size_t workspace_bytes,
                                     ball_vision_result_t *result)
{
    if (rgb == NULL || result == NULL || workspace == NULL || width == 0U ||
        height == 0U || stride < (size_t)width * 3U) return ESP_ERR_INVALID_ARG;
    const size_t needed = ball_vision_workspace_size(width, height);
    if (needed == 0U || workspace_bytes < needed) return ESP_ERR_NO_MEM;
    *result = (ball_vision_result_t) {
        .frame_sequence = sequence,
        .image_width = (uint16_t)width,
        .image_height = (uint16_t)height,
    };
    const size_t pixels = (size_t)width * height;
    uint8_t *mask = (uint8_t *)workspace;
    uint16_t *queue = (uint16_t *)(mask + pixels + (pixels & 1U));

    build_red_mask(rgb, width, height, stride, mask);
    find_largest(mask, queue, width, height, BALL_RED_MIN_AREA,
                 BALL_RED_MAX_AREA, 30U, 3U, 1U, &result->red_ball);
    build_white_mask(rgb, width, height, stride, mask);
    find_largest(mask, queue, width, height, BALL_WHITE_MIN_AREA,
                 BALL_WHITE_MAX_AREA, 10U, 4U, 1U, &result->white_ball);
    find_holes(rgb, width, height, stride, mask, queue,
               &result->left_hole, &result->right_hole, &result->lone_hole);
    return ESP_OK;
}
