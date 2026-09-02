#include "pseudo_infrared.h"

#include <stdlib.h>

#include "board_config.h"

static uint32_t s_last_sequence;
static uint8_t s_stable_mask;
static unsigned s_finish_confirm;

static int clamp_int(int value, int minimum, int maximum)
{
    if (value < minimum) {
        return minimum;
    }
    return value > maximum ? maximum : value;
}

static uint8_t vision_to_mask(const vision_result_t *vision, bool finish)
{
    if (vision == NULL) {
        return 0U;
    }
    if (finish) {
        return IR_ALL_BLACK_MASK;
    }
    if (!vision->frame_valid || !vision->line_found ||
        vision->confidence < VISION_LINE_CONFIDENCE_MIN) {
        return 0U;                      /* no usable line -> all white */
    }

    const int width = (int)vision->image_width;
    if (width < 4) {
        return 0U;
    }
    const int half = width / 2;

    /* Prefer the nearest scan row; fall back to the normalized lateral error. */
    int x = half + (vision->lateral_error * half) / 1000;
    if ((vision->scan_valid_mask & 0x01U) != 0U) {
        x = (int)vision->scan_center_x[0];
    }
    x = clamp_int(x, 0, width - 1);

    /*
     * Four virtual channels across the image. bit 0 = channel 1 = car right
     * (largest logical x), bit 3 = channel 4 = car left. The band is wider
     * than half the channel spacing so a line on a boundary lights two
     * adjacent channels, matching the real sensor array.
     */
    const int centers[4] = { 7 * width / 8, 5 * width / 8,
                             3 * width / 8, width / 8 };
    const int band = (3 * width) / 16;

    uint8_t mask = 0;
    int best = 0;
    int best_distance = width;
    for (unsigned i = 0; i < 4U; ++i) {
        const int distance = abs(x - centers[i]);
        if (distance < best_distance) {
            best_distance = distance;
            best = (int)i;
        }
        if (distance <= band) {
            mask |= (uint8_t)(1U << i);
        }
    }
    if (mask == 0U) {
        mask = (uint8_t)(1U << best);
    }
    return mask;
}

esp_err_t pseudo_infrared_sample(const vision_result_t *vision,
                                 infrared_sensor_state_t *state)
{
    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    state->changed = false;
    if (vision != NULL && vision->sequence != 0U &&
        vision->sequence != s_last_sequence) {
        s_last_sequence = vision->sequence;

        if (vision->finish_marker) {
            if (s_finish_confirm < VISION_FINISH_CONFIRM_FRAMES) {
                ++s_finish_confirm;
            }
        } else {
            s_finish_confirm = 0;
        }
        const bool finish = s_finish_confirm >= VISION_FINISH_CONFIRM_FRAMES;
        const uint8_t mask = vision_to_mask(vision, finish);
        if (mask != s_stable_mask) {
            state->changed = true;
            s_stable_mask = mask;
        }
    }

    state->black_mask = s_stable_mask;
    return ESP_OK;
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
