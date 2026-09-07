#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "ball_vision.h"
#include "infrared_sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CAMERA_VISION_LINE = 0,
    CAMERA_VISION_BALL,
} camera_vision_mode_t;

typedef struct {
    bool started;
    bool connected;
    uint32_t received_frames;
    uint32_t decode_failures;
    uint32_t dropped_frames;
    int64_t last_frame_us;
    uint16_t image_width;
    uint16_t image_height;
    camera_vision_mode_t mode;
    infrared_sensor_state_t infrared;
    ball_vision_result_t ball;
} camera_vision_status_t;

esp_err_t camera_vision_start(void);
esp_err_t camera_vision_get_status(camera_vision_status_t *status);
/* Mode changes only the software decode scale/analyzer. UVC capture settings
 * and the camera gimbal remain unchanged. */
esp_err_t camera_vision_set_mode(camera_vision_mode_t mode);

/* Copies the most recent raw MJPEG frame (untouched camera payload) for
 * network streaming. Returns ESP_ERR_INVALID_STATE when no frame has been
 * received yet, ESP_ERR_NO_MEM when dst_capacity is too small. */
esp_err_t camera_vision_get_jpeg(uint8_t *dst, size_t dst_capacity,
                                 size_t *out_bytes, uint32_t *out_seq);

#ifdef __cplusplus
}
#endif
