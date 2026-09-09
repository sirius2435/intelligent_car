#include "ball_vision.h"

#include <limits.h>
#include <math.h>
#include <string.h>

#include "board_config.h"

/* Work grid: one classification byte per pixel plus a span stack for the
 * flood fills. Sized for the push-mode decode grid; analyze() rejects frames
 * larger than this so the static buffers never overflow. */
#define BALL_VISION_MAX_W (CAMERA_FRAME_WIDTH / PUSH_CAMERA_DECODE_SCALE)
#define BALL_VISION_MAX_H (CAMERA_FRAME_HEIGHT / PUSH_CAMERA_DECODE_SCALE)

#define BALL_VISION_CLASS_BG        0u
#define BALL_VISION_CLASS_RED       1u
#define BALL_VISION_CLASS_BLUE      2u
#define BALL_VISION_CLASS_BLACK     3u   /* dark pocket regions */

/* Component bookkeeping (one entry per blob found by a flood fill). Floods
 * erase their class as they go, so the component count per class is what the
 * frame actually contains; 32 covers the target scenes (2 balls + glare). */
#define BALL_VISION_MAX_COMPONENTS 32

typedef struct {
    int area;
    int sum_x;
    int sum_y;
    int64_t sum_xx;   /* second moments about the origin, for the */
    int64_t sum_yy;   /* principal-axis ratio of the blob */
    int64_t sum_xy;
    int x0, y0, x1, y1;
    int luma_min;
    int luma_max;
} ball_vision_component_t;

/* Span stack for scanline flood fill. Depth is bounded by the number of
 * horizontal frontier segments a shape exposes. */
#define BALL_VISION_SPAN_STACK 1024

typedef struct {
    int x0, x1, y;
} ball_vision_span_t;

static uint8_t s_class_map[BALL_VISION_MAX_W * BALL_VISION_MAX_H];
static ball_vision_span_t s_span_stack[BALL_VISION_SPAN_STACK];

typedef struct {
    const uint8_t *rgb;
    unsigned width;
    unsigned height;
    size_t stride;
} ball_vision_frame_t;

static uint32_t luma_of(const uint8_t *pixel)
{
    return ((uint32_t)pixel[0] * 77U + (uint32_t)pixel[1] * 150U +
            (uint32_t)pixel[2] * 29U) >> 8U;
}

static int max3(int a, int b, int c)
{
    int m = a;
    if (b > m) {
        m = b;
    }
    if (c > m) {
        m = c;
    }
    return m;
}

static int min3(int a, int b, int c)
{
    int m = a;
    if (b < m) {
        m = b;
    }
    if (c < m) {
        m = c;
    }
    return m;
}

/* Maps a logical (viewer-consistent) coordinate to the raw decoded buffer
 * coordinate, applying the camera mounting flips like pseudo_infrared.c. */
static const uint8_t *pixel_at(const ball_vision_frame_t *frame,
                               unsigned logical_x,
                               unsigned logical_y)
{
    unsigned sx = logical_x;
    unsigned sy = logical_y;
#if CAMERA_FLIP_HORIZONTAL
    sx = frame->width - 1U - sx;
#endif
#if CAMERA_FLIP_VERTICAL
    sy = frame->height - 1U - sy;
#endif
    return frame->rgb + (size_t)sy * frame->stride + (size_t)sx * 3U;
}

/* Flood-fill one component of class c starting at (start_x, y). Pixels are
 * consumed (set to BG) as they are visited; component stats accumulate on
 * the way (including luma min/max for the blue sphere-shading gate). Stack
 * overflow abandons the rest of the component. */
static void flood_component(const ball_vision_frame_t *frame,
                            int start_x,
                            int y,
                            uint8_t class_id,
                            ball_vision_component_t *component)
{
    *component = (ball_vision_component_t) {
        .x0 = INT_MAX,
        .y0 = INT_MAX,
        .x1 = -1,
        .y1 = -1,
        .luma_min = INT_MAX,
        .luma_max = -1,
    };
    const int width = (int)frame->width;
    const int height = (int)frame->height;
    const int row_pitch = (int)frame->width;
    int stack_top = 0;

    s_span_stack[stack_top++] = (ball_vision_span_t) {
        .x0 = start_x, .x1 = start_x, .y = y,
    };
    s_class_map[(size_t)y * row_pitch + start_x] = BALL_VISION_CLASS_BG;

    while (stack_top > 0) {
        const ball_vision_span_t span = s_span_stack[--stack_top];

        /* Expand the span left and right over class pixels. */
        int x0 = span.x0;
        int x1 = span.x1;
        uint8_t *map = s_class_map + (size_t)span.y * row_pitch;
        while (x0 > 0 && map[x0 - 1] == class_id) {
            --x0;
        }
        while (x1 < width - 1 && map[x1 + 1] == class_id) {
            ++x1;
        }

        component->area += x1 - x0 + 1;
        component->sum_x += (x0 + x1) * (x1 - x0 + 1) / 2;
        component->sum_y += (x1 - x0 + 1) * span.y;
        if (x0 < component->x0) {
            component->x0 = x0;
        }
        if (x1 > component->x1) {
            component->x1 = x1;
        }
        if (span.y < component->y0) {
            component->y0 = span.y;
        }
        if (span.y > component->y1) {
            component->y1 = span.y;
        }
        for (int x = x0; x <= x1; ++x) {
            const int64_t px = x;
            const int64_t py = span.y;
            const int luma = (int)luma_of(pixel_at(frame,
                                                   (unsigned)x,
                                                   (unsigned)span.y));
            if (luma < component->luma_min) {
                component->luma_min = luma;
            }
            if (luma > component->luma_max) {
                component->luma_max = luma;
            }
            component->sum_xx += px * px;
            component->sum_yy += py * py;
            component->sum_xy += px * py;
            map[x] = BALL_VISION_CLASS_BG;
        }

        /* Scan the rows above and below for connected class runs. */
        if (stack_top > BALL_VISION_SPAN_STACK - 2 * (width + 3)) {
            return; /* stack guard: abandon the rest of this component */
        }
        for (int dy = -1; dy <= 1; dy += 2) {
            const int ny = span.y + dy;
            if (ny < 0 || ny >= height) {
                continue;
            }
            uint8_t *nmap = s_class_map + (size_t)ny * row_pitch;
            bool inside = false;
            for (int x = x0 - 1; x <= x1 + 1; ++x) {
                if (x >= 0 && x < width && nmap[x] == class_id) {
                    if (!inside) {
                        inside = true;
                        s_span_stack[stack_top++] = (ball_vision_span_t) {
                            .x0 = x, .x1 = x, .y = ny,
                        };
                        nmap[x] = BALL_VISION_CLASS_BG;
                    }
                } else {
                    inside = false;
                }
            }
        }
    }
}

/* Walks the class map and flood-fills every connected run of `class_id`,
 * returning up to max_components components. */
static unsigned collect_components(const ball_vision_frame_t *frame,
                                   uint8_t class_id,
                                   ball_vision_component_t *out,
                                   unsigned max_components)
{
    const int width = (int)frame->width;
    const int height = (int)frame->height;
    unsigned count = 0;
    for (int y = 0; y < height; ++y) {
        uint8_t *map = s_class_map + (size_t)y * width;
        for (int x = 0; x < width; ++x) {
            if (map[x] != class_id) {
                continue;
            }
            if (count >= max_components) {
                return count; /* abandon the rest of the frame */
            }
            flood_component(frame, x, y, class_id, &out[count]);
            if (out[count].x1 < out[count].x0) {
                continue; /* flood was aborted by the stack guard */
            }
            ++count;
        }
    }
    return count;
}

static void zero_result(ball_vision_result_t *out)
{
    memset(out, 0, sizeof(*out));
}

static unsigned isqrt_u32(unsigned value)
{
    unsigned root = 0;
    unsigned bit = 1u << 30;
    while (bit > value) {
        bit >>= 2;
    }
    while (bit != 0U) {
        if (value >= root + bit) {
            value -= root + bit;
            root = (root >> 1) + bit;
        } else {
            root >>= 1;
        }
        bit >>= 2;
    }
    return root;
}

static ball_blob_t blob_from_component(uint8_t class_id,
                                       const ball_vision_component_t *c)
{
    return (ball_blob_t) {
        .present = true,
        .color = class_id == BALL_VISION_CLASS_RED ? BALL_COLOR_RED
                                                   : BALL_COLOR_BLUE,
        .cx = c->sum_x / c->area,
        .cy = c->sum_y / c->area,
        .radius = (int)isqrt_u32((unsigned)c->area * 100U / 314U),
        .area = (unsigned)c->area,
        .x0 = c->x0, .y0 = c->y0, .x1 = c->x1, .y1 = c->y1,
    };
}

/* Bounding-box fill (percent): 100 * area / (w * h). A perfect disk in a
 * square bbox fills pi/4 ~= 79%; glare streaks, tails and fragmented noise
 * score far lower. */
static unsigned bbox_fill_percent(const ball_vision_component_t *c)
{
    const int w = c->x1 - c->x0 + 1;
    const int h = c->y1 - c->y0 + 1;
    if (w <= 0 || h <= 0) {
        return 0;
    }
    return (unsigned)((int64_t)c->area * 10000 / ((int64_t)w * h)) / 100U;
}

/* Principal-axis ratio of the blob's second moments (minor/major, per
 * mille). A disk scores ~1000 even at an oblique viewing angle; crescents
 * and arc-shaped glare (e.g. light bouncing off a track bend) score far
 * lower because their mass is spread along one axis. */
static int axis_ratio_mille(const ball_vision_component_t *c)
{
    if (c->area <= 1) {
        return 1000;
    }
    const int64_t area = c->area;
    const double cx = (double)c->sum_x / area;
    const double cy = (double)c->sum_y / area;
    const double m20 = (double)c->sum_xx / area - cx * cx;
    const double m02 = (double)c->sum_yy / area - cy * cy;
    const double m11 = (double)c->sum_xy / area - cx * cy;
    const double trace = m20 + m02;
    double disc = trace * trace - 4.0 * (m20 * m02 - m11 * m11);
    if (disc < 0.0) {
        disc = 0.0;
    }
    const double sq = disc > 0.0 ? sqrt(disc) : 0.0;
    const double lambda_max = (trace + sq) / 2.0;
    const double lambda_min = (trace - sq) / 2.0;
    if (lambda_max <= 0.0) {
        return 1000;
    }
    return (int)(lambda_min * 1000.0 / lambda_max + 0.5);
}

static bool shape_ok(const ball_vision_component_t *c, uint8_t class_id)
{
    const unsigned area = (unsigned)c->area;

    /* Red balls are deliberately allowed to be smaller / slightly less
     * circular than blue balls.  At the far end of the table the 120x80
     * push grid can represent the red ball with only ~8-20 pixels after
     * JPEG decoding.  The red-specific compactness + axis gates still
     * reject long LED / chassis artifacts. */
    const unsigned min_area = class_id == BALL_VISION_CLASS_RED
        ? (unsigned)BALL_RED_MIN_AREA_PX : (unsigned)BALL_MIN_AREA_PX;
    const unsigned compactness_min = class_id == BALL_VISION_CLASS_RED
        ? (unsigned)BALL_RED_COMPACTNESS_MIN
        : (unsigned)BALL_COMPACTNESS_MIN;
    const int max_aspect = class_id == BALL_VISION_CLASS_RED
        ? (int)BALL_RED_MAX_ASPECT : (int)BALL_MAX_ASPECT;
    const int axis_min = class_id == BALL_VISION_CLASS_RED
        ? (int)BALL_RED_AXIS_RATIO_MIN_MILLE
        : (int)BALL_AXIS_RATIO_MIN_MILLE;

    if (area < min_area || area > BALL_MAX_AREA_PX) {
        return false;
    }
    if (bbox_fill_percent(c) < compactness_min) {
        return false;
    }
    const int w = c->x1 - c->x0 + 1;
    const int h = c->y1 - c->y0 + 1;
    const int long_side = w > h ? w : h;
    const int short_side = w > h ? h : w;
    if (short_side > 0 && long_side * 100 / short_side > max_aspect) {
        return false;
    }
    if (axis_ratio_mille(c) < axis_min) {
        return false;
    }
    return true;
}

/* Best (largest, valid-shaped) blob of a class, if any. The blue ball
 * additionally requires a luminance spread across the blob: a shaded sphere
 * spans highlight to rim, a flat patch of blue paint does not. */
static bool pick_best_ball(const ball_vision_frame_t *frame,
                           uint8_t class_id,
                           ball_blob_t *out)
{
    ball_vision_component_t components[BALL_VISION_MAX_COMPONENTS];
    const unsigned count = collect_components(frame, class_id, components,
                                              BALL_VISION_MAX_COMPONENTS);
    int best = -1;
    unsigned best_area = 0;
    for (unsigned i = 0; i < count; ++i) {
        const ball_vision_component_t *c = &components[i];
        if (!shape_ok(c, class_id)) {
            continue;
        }
        if (class_id == BALL_VISION_CLASS_BLUE) {
            /* The lower part of the logical image contains the car chassis,
             * wiring and metal hardware. Those surfaces can carry a blue
             * cast (anodised aluminium, blue loom) that passes the same
             * chroma test as the real ball. The ball is expected above this
             * cutoff during the approach; once it reaches this depth it is
             * already too close and ALIGN should not be entered anyway. */
            const int cy = c->sum_y / c->area;
            if (cy > BALL_BLUE_MAX_CY_PX) {
                continue;
            }
            /* Sphere shading. Drop BALL_BLUE_GRADIENT_MIN to 0 if the ball
             * reads flat on site (matte paint / heavy JPEG smoothing). */
            if (c->luma_max - c->luma_min < BALL_BLUE_GRADIENT_MIN) {
                continue;
            }
        }
        if ((unsigned)c->area > best_area) {
            best_area = (unsigned)c->area;
            best = (int)i;
        }
    }
    if (best < 0) {
        out->present = false;
        return false;
    }
    *out = blob_from_component(class_id, &components[best]);
    return true;
}

static bool pocket_pixel(const ball_vision_frame_t *frame,
                         unsigned x, unsigned y)
{
    const uint8_t *p = pixel_at(frame, x, y);
    const int luma = (int)luma_of(p);
    const int sat = max3(p[0], p[1], p[2]) - min3(p[0], p[1], p[2]);
    return luma <= POCKET_BLACK_MAX_LUMA && sat <= POCKET_BLACK_MAX_SAT;
}

static void add_edge_zone_pocket(const ball_vision_frame_t *frame,
                                 pocket_blob_t *slots,
                                 unsigned *count,
                                 unsigned side)
{
    if (*count >= 2U) {
        return;
    }
    const unsigned width = frame->width;
    const unsigned height = frame->height;
    const unsigned band = height * POCKET_REGION_MAX_Y_PERCENT / 100U;
    const unsigned top_rows = height * POCKET_EDGE_TOP_ROWS_PERCENT / 100U;
    const unsigned edge_rows = top_rows < 2U ? 2U : top_rows;
    const unsigned x0 = side == 0U ? 0U : width / 2U;
    const unsigned x1 = side == 0U ? width / 2U : width;

    unsigned area = 0;
    unsigned top_area = 0;
    int sum_x = 0;
    int sum_y = 0;
    int min_x = (int)width, min_y = (int)height;
    int max_x = -1, max_y = -1;

    /* This is deliberately an EDGE test rather than a rectangle test.  The
     * real target can be one continuous black strip along the table boundary;
     * connected-component extraction then returns one giant blob.  Split that
     * edge-connected black mass into left/right target zones for the controller. */
    for (unsigned y = 0; y < band; ++y) {
        for (unsigned x = x0; x < x1; ++x) {
            if (!pocket_pixel(frame, x, y)) {
                continue;
            }
            ++area;
            if (y < edge_rows) {
                ++top_area;
            }
            sum_x += (int)x;
            sum_y += (int)y;
            if ((int)x < min_x) min_x = (int)x;
            if ((int)x > max_x) max_x = (int)x;
            if ((int)y < min_y) min_y = (int)y;
            if ((int)y > max_y) max_y = (int)y;
        }
    }
    if (area < POCKET_EDGE_MIN_AREA_PX ||
        top_area < POCKET_EDGE_MIN_TOP_PIXELS ||
        max_x < min_x || max_y < min_y) {
        return;
    }

    pocket_blob_t candidate = {
        .visible = true,
        .cx = sum_x / (int)area,
        .cy = sum_y / (int)area,
        .area = area,
        .x0 = min_x, .y0 = min_y, .x1 = max_x, .y1 = max_y,
    };

    /* Replace an existing slot for the same side only when the edge-zone
     * fallback is stronger.  Slots are always kept sorted left -> right. */
    unsigned pos = *count;
    if (pos < 2U) {
        while (pos > 0U && slots[pos - 1U].cx > candidate.cx) {
            slots[pos] = slots[pos - 1U];
            --pos;
        }
        slots[pos] = candidate;
        ++*count;
    }
}

static void pick_pockets(const ball_vision_frame_t *frame,
                         pocket_blob_t *slots,
                         unsigned *count)
{
    ball_vision_component_t components[BALL_VISION_MAX_COMPONENTS];
    const unsigned found = collect_components(frame, BALL_VISION_CLASS_BLACK,
                                              components,
                                              BALL_VISION_MAX_COMPONENTS);
    /* Keep the two largest valid pockets, then sort them left -> right. */
    int picks[2] = { -1, -1 };
    for (unsigned i = 0; i < found; ++i) {
        if ((unsigned)components[i].area < (unsigned)POCKET_MIN_AREA_PX) {
            continue;
        }
        if (picks[0] < 0 || components[i].area > components[picks[0]].area) {
            picks[1] = picks[0];
            picks[0] = (int)i;
        } else if (picks[1] < 0 ||
                   components[i].area > components[picks[1]].area) {
            picks[1] = (int)i;
        }
    }
    for (unsigned k = 0; k < 2; ++k) {
        if (picks[k] < 0) {
            continue;
        }
        const ball_vision_component_t *c = &components[picks[k]];
        /* A real target is the black boundary band at the far image edge.
         * Reject every interior black component here; otherwise floor marks,
         * shadows and loose black objects become fake pockets. The separate
         * edge-zone fallback below handles a single continuous black strip. */
        if (c->y0 > (int)POCKET_EDGE_TOUCH_ROWS_PIXELS) {
            continue;
        }
        pocket_blob_t candidate = {
            .visible = true,
            .cx = c->sum_x / c->area,
            .cy = c->sum_y / c->area,
            .area = (unsigned)c->area,
            .x0 = c->x0, .y0 = c->y0, .x1 = c->x1, .y1 = c->y1,
        };
        if (*count >= 2) {
            break;
        }
        unsigned pos = *count;
        while (pos > 0 && slots[pos - 1].cx > candidate.cx) {
            slots[pos] = slots[pos - 1];
            --pos;
        }
        slots[pos] = candidate;
        ++*count;
    }

    /* If the physical boundary is a single continuous black area, the
     * component list may contain only one blob.  Recover the left/right
     * targets directly from edge-connected dark pixels. */
    if (*count < 2U) {
        pocket_blob_t edge_slots[2] = {0};
        unsigned edge_count = 0;
        add_edge_zone_pocket(frame, edge_slots, &edge_count, 0U);
        add_edge_zone_pocket(frame, edge_slots, &edge_count, 1U);
        if (edge_count > *count) {
            for (unsigned i = 0; i < edge_count; ++i) {
                slots[i] = edge_slots[i];
            }
            *count = edge_count;
        }
    }
}

esp_err_t ball_vision_analyze(const uint8_t *rgb,
                              unsigned width,
                              unsigned height,
                              size_t stride_bytes,
                              ball_vision_result_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    zero_result(out);
    if (rgb == NULL || width == 0U || height == 0U ||
        width > BALL_VISION_MAX_W || height > BALL_VISION_MAX_H ||
        stride_bytes < (size_t)width * 3U) {
        return ESP_ERR_INVALID_ARG;
    }

    const ball_vision_frame_t frame = {
        .rgb = rgb,
        .width = width,
        .height = height,
        .stride = stride_bytes,
    };

    /* Pass 1: TABLETOP luminance = the dominant peak of the luminance
     * histogram, not the frame mean. The mean is dragged down whenever the
     * black track or its bends sweep through the view. Both ball tests are
     * chromatic now, so this value is a DIAGNOSTIC for the on-site tuning
     * page (/status "t") rather than a gate; the histogram peak still says
     * "this is the board surface" whatever else is in frame. */
    uint16_t hist[256];
    memset(hist, 0, sizeof(hist));
    for (unsigned y = 0; y < height; ++y) {
        const uint8_t *row = rgb + (size_t)y * stride_bytes;
        for (unsigned x = 0; x < width; ++x) {
            ++hist[luma_of(row + (size_t)x * 3U)];
        }
    }
    unsigned peak_bin = 0;
    for (unsigned i = 1; i < 256; ++i) {
        if (hist[i] > hist[peak_bin]) {
            peak_bin = i;
        }
    }
    uint64_t weight_sum = 0;
    uint64_t luma_weighted = 0;
    const int lo = (int)peak_bin > 3 ? (int)peak_bin - 3 : 0;
    const int hi = (int)peak_bin < 252 ? (int)peak_bin + 3 : 255;
    for (int i = lo; i <= hi; ++i) {
        weight_sum += hist[i];
        luma_weighted += (uint64_t)hist[i] * (uint64_t)i;
    }
    const int table_luma = weight_sum == 0 ? 128 :
        (int)(luma_weighted / weight_sum);

    /* Pass 2: per-pixel classification in LOGICAL order. The dark (pocket)
     * test only runs on the far-table band (logical y < 55% height), which
     * keeps the black track / finish bar in the lower half out of the test.
     * Red and blue are tested with priority so a far ball - including the
     * shaded, dark side of the blue one - cannot be swallowed by the pocket
     * test. */
    const unsigned pocket_band = height * POCKET_REGION_MAX_Y_PERCENT / 100U;
    unsigned red_px = 0;
    unsigned blue_px = 0;
    unsigned pocket_px = 0;
    for (unsigned ly = 0; ly < height; ++ly) {
        for (unsigned lx = 0; lx < width; ++lx) {
            const uint8_t *p = pixel_at(&frame, lx, ly);
            const int r = p[0];
            const int g = p[1];
            const int b = p[2];
            const int luma = (int)luma_of(p);
            const int sat = max3(r, g, b) - min3(r, g, b);
            uint8_t class_id = BALL_VISION_CLASS_BG;
            if (r - g >= BALL_RED_MIN_RG_DOM &&
                r - b >= BALL_RED_MIN_RB_DOM) {
                /* Pure R-G / R-B dominance (see board_config.h): survives
                 * underexposure, rejects neutral board and its chroma noise. */
                class_id = BALL_VISION_CLASS_RED;
                ++red_px;
            } else if (b - r >= BALL_BLUE_MIN_BR_DOM &&
                       b - g >= BALL_BLUE_MIN_BG_DOM &&
                       b >= BALL_BLUE_MIN_B &&
                       sat >= BALL_BLUE_MIN_SAT) {
                /* Pure B-R / B-G dominance, mirroring the red test. The two
                 * absolute floors only exist to keep dark regions (track,
                 * pocket interior) with their larger relative chroma noise
                 * out of the ball class. */
                class_id = BALL_VISION_CLASS_BLUE;
                ++blue_px;
            } else if (ly < pocket_band &&
                       luma <= POCKET_BLACK_MAX_LUMA &&
                       sat <= POCKET_BLACK_MAX_SAT) {
                /* Dark pocket region (far band only). Require a genuinely
                 * dark, low-saturation region so ordinary grey shadows are
                 * less likely to qualify. */
                class_id = BALL_VISION_CLASS_BLACK;
                ++pocket_px;
            }
            s_class_map[(size_t)ly * width + lx] = class_id;
        }
    }
    out->table_luma = (unsigned)table_luma;
    out->red_px = red_px;
    out->blue_px = blue_px;
    out->pocket_px = pocket_px;
    for (unsigned i = 0; i < 16; ++i) {
        unsigned bin = 0;
        for (unsigned k = 0; k < 16; ++k) {
            bin += hist[i * 16 + k];
        }
        out->luma_hist[i] = bin;
    }

    out->width = width;
    out->height = height;

    if (pick_best_ball(&frame, BALL_VISION_CLASS_RED,
                       &out->balls[BALL_COLOR_RED])) {
        out->ball_count = 1;
    }
    if (pick_best_ball(&frame, BALL_VISION_CLASS_BLUE,
                       &out->balls[BALL_COLOR_BLUE])) {
        ++out->ball_count;
    }

    pick_pockets(&frame, out->pockets, &out->pocket_count);

    out->valid = true;
    return ESP_OK;
}

const ball_blob_t *ball_vision_find_ball(const ball_vision_result_t *result,
                                         ball_color_t color)
{
    if (result == NULL || color >= BALL_COLOR_COUNT) {
        return NULL;
    }
    return result->balls[color].present ? &result->balls[color] : NULL;
}

const pocket_blob_t *ball_vision_find_pocket(const ball_vision_result_t *result,
                                             unsigned side)
{
    if (result == NULL || side >= 2) {
        return NULL;
    }
    return result->pockets[side].visible ? &result->pockets[side] : NULL;
}
