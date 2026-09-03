#include "pseudo_infrared.h"

#include "board_config.h"

static uint8_t s_stable_mask;
static unsigned s_finish_confirm;

static uint8_t pixel_luma(const uint8_t *pixel)
{
    return (uint8_t)(((unsigned)pixel[0] * 77U +
                      (unsigned)pixel[1] * 150U +
                      (unsigned)pixel[2] * 29U) >> 8U);
}

/* Maps logical image coordinates (as shown by the Wi-Fi viewer) to the raw
 * decoded buffer, applying the camera mounting flips so the four blocks land
 * on the same physical spots regardless of camera orientation. */
static const uint8_t *pixel_at(const uint8_t *rgb,
                               unsigned width,
                               unsigned height,
                               size_t stride_bytes,
                               unsigned logical_x,
                               unsigned logical_y)
{
#if CAMERA_FLIP_HORIZONTAL
    const unsigned source_x = width - 1U - logical_x;
#else
    const unsigned source_x = logical_x;
#endif
#if CAMERA_FLIP_VERTICAL
    const unsigned source_y = height - 1U - logical_y;
#else
    const unsigned source_y = logical_y;
#endif
    return rgb + (size_t)source_y * stride_bytes + (size_t)source_x * 3U;
}

esp_err_t pseudo_infrared_sample_rgb888(const uint8_t *rgb,
                                        unsigned width,
                                        unsigned height,
                                        size_t stride_bytes,
                                        infrared_sensor_state_t *state)
{
    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    state->changed = false;
    if (rgb == NULL || width < 4U || height < 4U ||
        stride_bytes < (size_t)width * 3U) {
        state->black_mask = s_stable_mask;  /* hold the last known mask */
        return ESP_OK;
    }

    const unsigned sample_y = height * PSEUDO_IR_SAMPLE_ROW_PERCENT / 100U;
    const unsigned sample_y_clamped = sample_y >= height ? height - 1U
                                                          : sample_y;

    /* Adaptive threshold from the sample row's mean luminance. */
    uint32_t luma_sum = 0U;
    for (unsigned x = 0U; x < width; ++x) {
        luma_sum += pixel_luma(pixel_at(rgb, width, height, stride_bytes,
                                        x, sample_y_clamped));
    }
    int threshold = (int)(luma_sum / width) - PSEUDO_IR_BLACK_MARGIN;
    if (threshold < 12) {
        threshold = 12;
    } else if (threshold > 220) {
        threshold = 220;
    }

    /*
     * Four fixed blocks, one per channel. bit 0 = channel 1 = car right
     * (largest logical x), bit 3 = channel 4 = car left. Channel centers are
     * 7/8, 5/8, 3/8, 1/8 of the image width.
     */
    static const unsigned center_numerators[4] = { 7U, 5U, 3U, 1U };
    const unsigned half = PSEUDO_IR_BLOCK_SIZE / 2U;
    uint8_t mask = 0U;
    for (unsigned i = 0U; i < 4U; ++i) {
        const unsigned center_x = center_numerators[i] * width / 8U;
        const unsigned x0 = center_x > half ? center_x - half : 0U;
        const unsigned x1 = center_x + half < width ? center_x + half
                                                    : width - 1U;
        const unsigned y0 = sample_y_clamped > half ? sample_y_clamped - half
                                                    : 0U;
        const unsigned y1 = sample_y_clamped + half < height
                                ? sample_y_clamped + half : height - 1U;

        unsigned dark = 0U;
        for (unsigned yy = y0; yy <= y1; ++yy) {
            for (unsigned xx = x0; xx <= x1; ++xx) {
                if (pixel_luma(pixel_at(rgb, width, height, stride_bytes,
                                        xx, yy)) <= (uint8_t)threshold) {
                    ++dark;
                }
            }
        }
        if (dark >= PSEUDO_IR_BLOCK_DARK_MIN) {
            mask |= (uint8_t)(1U << i);
        }
    }

    /* Finish marker: a wide dark run across the sample row. */
    unsigned longest_run = 0U;
    unsigned run = 0U;
    for (unsigned x = 0U; x < width; ++x) {
        const bool dark =
            pixel_luma(pixel_at(rgb, width, height, stride_bytes,
                                x, sample_y_clamped)) <= (uint8_t)threshold;
        if (dark) {
            ++run;
            if (run > longest_run) {
                longest_run = run;
            }
        } else {
            run = 0U;
        }
    }
    const bool finish_candidate =
        longest_run * 100U >= width * PSEUDO_IR_FINISH_WIDTH_PERCENT;
    if (finish_candidate) {
        if (s_finish_confirm < PSEUDO_IR_FINISH_CONFIRM_FRAMES) {
            ++s_finish_confirm;
        }
    } else {
        s_finish_confirm = 0U;
    }
    if (s_finish_confirm >= PSEUDO_IR_FINISH_CONFIRM_FRAMES) {
        mask = IR_ALL_BLACK_MASK;
    }

    if (mask != s_stable_mask) {
        state->changed = true;
        s_stable_mask = mask;
    }
    state->black_mask = s_stable_mask;
    return ESP_OK;
}

void pseudo_infrared_reset(void)
{
    s_stable_mask = 0U;
    s_finish_confirm = 0U;
}

void pseudo_infrared_format(uint8_t black_mask, char output[5])
{
    if (output == NULL) {
        return;
    }
    output[0] = (black_mask & IR_CHANNEL_4_MASK) ? 'B' : 'W';
    output[1] = (black_mask & IR_CHANNEL_3_MASK) ? 'B' : 'W';
    output[2] = (black_mask & IR_CHANNEL_2_MASK) ? 'B' : 'W';
    output[3] = (black_mask & IR_CHANNEL_1_MASK) ? 'B' : 'W';
    output[4] = '\0';
}
