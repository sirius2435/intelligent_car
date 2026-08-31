#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "vision_line.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool started;
    bool connected;
    uint32_t received_frames;
    uint32_t decode_failures;
    uint32_t dropped_frames;
    int64_t last_frame_us;
    vision_result_t vision;
} camera_vision_status_t;

esp_err_t camera_vision_start(void);
esp_err_t camera_vision_get_status(camera_vision_status_t *status);

#ifdef __cplusplus
}
#endif
