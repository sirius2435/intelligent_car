#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ball_vision.h"
#include "board_config.h"

#define CHECK(condition) do {                                                   \
    if (!(condition)) {                                                         \
        fprintf(stderr, "FAIL %s:%d: %s\n", __func__, __LINE__, #condition);   \
        exit(1);                                                                \
    }                                                                           \
} while (0)

#define TEST_WIDTH   120U
#define TEST_HEIGHT   80U

/* Scene palette matching the real setup:
 * - white background board (neutral, ~238).
 * - red ball: strong R dominance.
 * - blue ball: strong B dominance with sphere shading (dark rim -> bright
 *   core), so the B-R / B-G gap holds at every level of the ball.
 * - pockets: DARK, near-neutral regions at the far table band (they read
 *   black in the camera, see POCKET_BLACK_*).
 * - shadows on the board: neutral grey gradients, never chromatic. */
#define BOARD_R 238
#define BOARD_G 238
#define BOARD_B 238
#define RED_R 235
#define RED_G 45
#define RED_B 50
#define POCKET_R 26
#define POCKET_G 26
#define POCKET_B 30
/* Blue-ball hue: (r,g,b) = level * (30%,45%,100%), i.e. `level` is the blue
 * channel and r/g stay a fixed fraction of it. */
#define BLUE_R_PERCENT 30
#define BLUE_G_PERCENT 45

static uint8_t s_rgb[TEST_WIDTH * TEST_HEIGHT * 3U];

static void set_pixel(unsigned logical_x, unsigned logical_y,
                      uint8_t r, uint8_t g, uint8_t b)
{
    unsigned source_x = logical_x;
    unsigned source_y = logical_y;
#if CAMERA_FLIP_HORIZONTAL
    source_x = TEST_WIDTH - 1U - source_x;
#endif
#if CAMERA_FLIP_VERTICAL
    source_y = TEST_HEIGHT - 1U - source_y;
#endif
    uint8_t *pixel = s_rgb + (source_y * TEST_WIDTH + source_x) * 3U;
    pixel[0] = r;
    pixel[1] = g;
    pixel[2] = b;
}

static void fill_board(void)
{
    for (unsigned y = 0; y < TEST_HEIGHT; ++y) {
        for (unsigned x = 0; x < TEST_WIDTH; ++x) {
            set_pixel(x, y, BOARD_R, BOARD_G, BOARD_B);
        }
    }
}

static void draw_disk(int cx, int cy, int radius,
                      uint8_t r, uint8_t g, uint8_t b)
{
    const int r2 = radius * radius;
    for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
            const int x = cx + dx;
            const int y = cy + dy;
            if (x < 0 || y < 0 || x >= (int)TEST_WIDTH ||
                y >= (int)TEST_HEIGHT || dx * dx + dy * dy > r2) {
                continue;
            }
            set_pixel((unsigned)x, (unsigned)y, r, g, b);
        }
    }
}

/* Neutral sphere shading from rim_luma at the edge to core_luma at centre -
 * used for board shadows: a neutral grey disk with a gradient, which no
 * chroma test may ever accept as a ball. */
static void draw_neutral_ball(int cx, int cy, int radius,
                              int rim_luma, int core_luma)
{
    const int r2 = radius * radius;
    const int spread = core_luma - rim_luma;
    for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
            const int x = cx + dx;
            const int y = cy + dy;
            if (x < 0 || y < 0 || x >= (int)TEST_WIDTH ||
                y >= (int)TEST_HEIGHT || dx * dx + dy * dy > r2) {
                continue;
            }
            const int fall = r2 - (dx * dx + dy * dy);
            const int level = rim_luma + fall * spread / r2;
            const uint8_t gray = (uint8_t)level;
            set_pixel((unsigned)x, (unsigned)y, gray, gray, gray);
        }
    }
}

/* Shaded blue sphere: `level` is the blue channel, running from rim_level at
 * the edge to core_level at the centre, with r/g following as a fixed
 * fraction so every pixel keeps the dominance of a real blue ball. */
static void draw_blue_ball(int cx, int cy, int radius,
                           int rim_level, int core_level)
{
    const int r2 = radius * radius;
    const int spread = core_level - rim_level;
    for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
            const int x = cx + dx;
            const int y = cy + dy;
            if (x < 0 || y < 0 || x >= (int)TEST_WIDTH ||
                y >= (int)TEST_HEIGHT || dx * dx + dy * dy > r2) {
                continue;
            }
            const int fall = r2 - (dx * dx + dy * dy);
            const int level = rim_level + fall * spread / r2;
            set_pixel((unsigned)x, (unsigned)y,
                      (uint8_t)(level * BLUE_R_PERCENT / 100),
                      (uint8_t)(level * BLUE_G_PERCENT / 100),
                      (uint8_t)level);
        }
    }
}

/* Dark pocket trapezoid: half-width grows from top_half to bot_half over
 * `rows` rows, top at logical top_y. */
static void draw_pocket_trapezoid(int cx, int top_y, int rows,
                                  int top_half, int bot_half)
{
    for (int y = top_y; y < top_y + rows; ++y) {
        const int t = (y - top_y) * 100 / rows;
        const int hw = top_half + (bot_half - top_half) * t / 100;
        for (int x = cx - hw; x <= cx + hw; ++x) {
            if (x >= 0 && y >= 0 && x < (int)TEST_WIDTH &&
                y < (int)TEST_HEIGHT) {
                set_pixel((unsigned)x, (unsigned)y,
                          POCKET_R, POCKET_G, POCKET_B);
            }
        }
    }
}

static ball_vision_result_t analyze(void)
{
    ball_vision_result_t result;
    memset(&result, 0, sizeof(result));
    CHECK(ball_vision_analyze(s_rgb, TEST_WIDTH, TEST_HEIGHT,
                              TEST_WIDTH * 3U, &result) == ESP_OK);
    return result;
}

static void test_red_ball_on_white_board(void)
{
    fill_board();
    draw_disk(60, 45, 9, RED_R, RED_G, RED_B);
    const ball_vision_result_t result = analyze();
    CHECK(result.valid);
    CHECK(result.ball_count == 1);
    CHECK(result.balls[BALL_COLOR_RED].present);
    CHECK(!result.balls[BALL_COLOR_BLUE].present);
    CHECK(result.balls[BALL_COLOR_RED].cx >= 58 &&
          result.balls[BALL_COLOR_RED].cx <= 62);
    CHECK(result.balls[BALL_COLOR_RED].radius >= 7 &&
          result.balls[BALL_COLOR_RED].radius <= 12);
}

static void test_underexposed_red_ball_detected(void)
{
    /* Underexposure scales channels down together: the red ball reads much
     * darker (110,45,40) yet its dominance survives - no absolute R/luma
     * floor may kill it. */
    fill_board();
    draw_disk(60, 45, 9, 110, 45, 40);
    const ball_vision_result_t result = analyze();
    CHECK(result.valid);
    CHECK(result.balls[BALL_COLOR_RED].present);
    CHECK(!result.balls[BALL_COLOR_BLUE].present);
}

static void test_red_and_blue_balls_on_board(void)
{
    fill_board();
    draw_disk(28, 52, 8, RED_R, RED_G, RED_B);
    /* Blue ball: shaded disk whose blue channel runs 90 (rim) -> 190 (core),
     * i.e. B-R / B-G dominant at every level. */
    draw_blue_ball(92, 44, 9, 90, 190);
    const ball_vision_result_t result = analyze();
    CHECK(result.ball_count == 2);
    CHECK(result.balls[BALL_COLOR_RED].present);
    CHECK(result.balls[BALL_COLOR_RED].cx >= 26 &&
          result.balls[BALL_COLOR_RED].cx <= 31);
    CHECK(result.balls[BALL_COLOR_BLUE].present);
    CHECK(result.balls[BALL_COLOR_BLUE].cx >= 90 &&
          result.balls[BALL_COLOR_BLUE].cx <= 95);
    CHECK(result.balls[BALL_COLOR_BLUE].radius >= 7 &&
          result.balls[BALL_COLOR_BLUE].radius <= 12);
}

static void test_underexposed_blue_ball_in_pocket_band(void)
{
    /* Underexposure scales the channels down together: the ball reads dark
     * (blue 75..120, luma ~34..56) yet its dominance survives - no luma
     * window may kill it. Those pixels are ALSO dark and low-saturation
     * enough to satisfy the pocket test, so this pins the classification
     * priority: inside the far band a blue ball must not be swallowed by the
     * dark pocket class. */
    fill_board();
    draw_blue_ball(60, 20, 7, 75, 120);
    const ball_vision_result_t result = analyze();
    CHECK(result.valid);
    CHECK(result.balls[BALL_COLOR_BLUE].present);
    CHECK(result.pocket_count == 0);
    CHECK(result.pocket_px == 0);
}

static void test_shadow_disk_rejected(void)
{
    /* A round grey shadow with a strong gradient: neutral, so it can satisfy
     * neither the blue nor the red chroma test. */
    fill_board();
    draw_neutral_ball(60, 50, 9, 150, 215);
    const ball_vision_result_t result = analyze();
    CHECK(result.valid);
    CHECK(!result.balls[BALL_COLOR_BLUE].present);
    CHECK(result.ball_count == 0);
}

static void test_flat_patch_rejected(void)
{
    /* Uniform neutral patch: no sphere shading (gradient 0). */
    fill_board();
    draw_disk(60, 45, 9, 170, 170, 170);
    const ball_vision_result_t result = analyze();
    CHECK(result.valid);
    CHECK(!result.balls[BALL_COLOR_BLUE].present);
    CHECK(!result.balls[BALL_COLOR_RED].present);
}

static void test_kidney_arc_rejected(void)
{
    /* Two overlapping shaded blue disks: kidney / arc shape (glare off a
     * track bend) - the principal-axis gate must reject it. */
    fill_board();
    draw_blue_ball(40, 45, 9, 90, 190);
    draw_blue_ball(50, 45, 9, 90, 190);
    const ball_vision_result_t result = analyze();
    CHECK(result.valid);
    CHECK(!result.balls[BALL_COLOR_BLUE].present);
}

static void test_dark_pockets_found_and_shadow_ignored(void)
{
    fill_board();
    /* Black track bar in the LOWER half: never a pocket (band-limited). */
    for (unsigned y = 62; y < TEST_HEIGHT; ++y) {
        for (unsigned x = 0; x < TEST_WIDTH; ++x) {
            set_pixel(x, y, 8, 8, 8);
        }
    }
    draw_pocket_trapezoid(20, 4, 26, 6, 14);    /* left pocket */
    draw_pocket_trapezoid(100, 4, 26, 6, 14);   /* right pocket */
    /* Big grey shadow inside the far band: neutral -> cannot be a pocket. */
    draw_neutral_ball(60, 26, 9, 150, 215);

    const ball_vision_result_t result = analyze();
    CHECK(result.valid);
    CHECK(result.pocket_count == 2);
    CHECK(result.pockets[0].cx < result.pockets[1].cx);
    CHECK(result.pockets[0].cx < 45);
    CHECK(result.pockets[1].cx > 75);
    CHECK(result.pockets[0].cy < 44 && result.pockets[1].cy < 44);
    CHECK(result.pockets[0].area > 150 && result.pockets[1].area > 150);
    CHECK(!result.balls[BALL_COLOR_BLUE].present); /* the shadow, rejected */
    CHECK(!result.balls[BALL_COLOR_RED].present);
}

static void test_small_flat_blue_speck_ignored(void)
{
    /* A tiny FLAT blue patch (no sphere shading): not a pocket (it does not
     * touch the far edge) and not a ball either (gradient gate). */
    fill_board();
    for (int y = 8; y < 14; ++y) {
        for (int x = 30; x < 36; ++x) {
            set_pixel((unsigned)x, (unsigned)y, 40, 70, 200);
        }
    }
    const ball_vision_result_t result = analyze();
    CHECK(result.pocket_count == 0);
    CHECK(!result.balls[BALL_COLOR_BLUE].present);
}

int main(void)
{
    CHECK(CAMERA_FLIP_HORIZONTAL == 1);
    CHECK(CAMERA_FLIP_VERTICAL == 1);
    CHECK(BALL_COLOR_RED == 0);
    CHECK(BALL_COLOR_BLUE == 1);
    test_red_ball_on_white_board();
    test_underexposed_red_ball_detected();
    test_red_and_blue_balls_on_board();
    test_underexposed_blue_ball_in_pocket_band();
    test_shadow_disk_rejected();
    test_flat_patch_rejected();
    test_kidney_arc_rejected();
    test_dark_pockets_found_and_shadow_ignored();
    test_small_flat_blue_speck_ignored();
    puts("ball vision tests passed");
    return 0;
}
