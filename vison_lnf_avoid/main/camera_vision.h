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

/* Copies the most recent raw MJPEG frame (untouched camera payload) for
 * network streaming. Returns ESP_ERR_INVALID_STATE when no frame has been
 * received yet, ESP_ERR_NO_MEM when dst_capacity is too small. */
esp_err_t camera_vision_get_jpeg(uint8_t *dst, size_t dst_capacity,
                                 size_t *out_bytes, uint32_t *out_seq);

#ifdef __cplusplus
}
#endif
