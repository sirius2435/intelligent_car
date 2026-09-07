#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ball_vision.h"
#include "board_config.h"

#define WIDTH 120U
#define HEIGHT 80U
#define CHECK(c) do { if (!(c)) { fprintf(stderr, "FAIL line %d: %s\n", \
    __LINE__, #c); exit(1); } } while (0)

static unsigned char image[WIDTH * HEIGHT * 3U];

static void set_logical(unsigned x, unsigned y,
                        unsigned char r, unsigned char g, unsigned char b)
{
#if CAMERA_FLIP_HORIZONTAL
    x = WIDTH - 1U - x;
#endif
#if CAMERA_FLIP_VERTICAL
    y = HEIGHT - 1U - y;
#endif
    unsigned char *p = &image[(y * WIDTH + x) * 3U];
    p[0] = r; p[1] = g; p[2] = b;
}

static void rect(unsigned x0, unsigned y0, unsigned w, unsigned h,
                 unsigned char r, unsigned char g, unsigned char b)
{
    for (unsigned y = y0; y < y0 + h; ++y)
        for (unsigned x = x0; x < x0 + w; ++x)
            set_logical(x, y, r, g, b);
}

int main(void)
{
    memset(image, 190, sizeof(image));
    /* Thin black line must not become a hole. */
    rect(10, 40, 100, 1, 15, 15, 15);
    rect(8, 10, 10, 10, 10, 10, 10);
    rect(100, 12, 10, 10, 10, 10, 10);
    rect(35, 45, 6, 6, 220, 30, 25);
    /* White ball: bright core and a darker local surround/shadow. */
    rect(75, 45, 9, 9, 145, 145, 145);
    rect(77, 47, 5, 5, 245, 245, 245);

    const size_t work_size = ball_vision_workspace_size(WIDTH, HEIGHT);
    void *work = malloc(work_size);
    CHECK(work != NULL);
    ball_vision_result_t result;
    CHECK(ball_vision_analyze_rgb888(image, WIDTH, HEIGHT, WIDTH * 3U,
                                     7, work, work_size, &result) == ESP_OK);
    CHECK(result.frame_sequence == 7);
    CHECK(result.red_ball.found);
    CHECK(result.red_ball.center_x >= 35 && result.red_ball.center_x <= 41);
    CHECK(result.white_ball.found);
    CHECK(result.left_hole.found && result.right_hole.found);
    CHECK(result.left_hole.center_x < result.right_hole.center_x);
    CHECK(result.left_hole.center_x < 20);
    CHECK(result.right_hole.center_x > 95);
    free(work);
    puts("ball vision tests passed");
    return 0;
}
