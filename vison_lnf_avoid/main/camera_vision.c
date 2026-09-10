#include "camera_vision.h"

#include <stddef.h>
#include <string.h>

#include "board_config.h"
#include "ball_vision.h"
#include "pseudo_infrared.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_psram.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "jpeg_decoder.h"
#include "usb_stream.h"

/* esp_jpeg_decode() is stateless and its out_scale is a per-call option, so
 * the decode grid can follow the active mode (LINE: CAMERA_DECODE_SCALE,
 * PUSH: PUSH_CAMERA_DECODE_SCALE) without reconfiguring anything. */
static esp_jpeg_image_scale_t jpeg_scale_for(unsigned scale)
{
    switch (scale) {
    case 2:
        return JPEG_IMAGE_SCALE_1_2;
    case 4:
        return JPEG_IMAGE_SCALE_1_4;
    case 8:
        return JPEG_IMAGE_SCALE_1_8;
    default:
        return JPEG_IMAGE_SCALE_0; /* no downscale */
    }
}

static unsigned active_decode_scale(camera_vision_mode_t mode)
{
    return mode == CAMERA_VISION_MODE_PUSH ? PUSH_CAMERA_DECODE_SCALE
                                           : CAMERA_DECODE_SCALE;
}

/* The RGB output buffer must fit the COARSER of the two grids. */
#if CAMERA_DECODE_SCALE < PUSH_CAMERA_DECODE_SCALE
#define CAMERA_WORK_SCALE CAMERA_DECODE_SCALE
#else
#define CAMERA_WORK_SCALE PUSH_CAMERA_DECODE_SCALE
#endif

/* JPEG decoding is CPU-heavy (ROM jd_decomp, software only). Doing it inside
 * usb_stream's "sample_proc" callback starves IDLE0 and trips the task WDT.
 * The callback therefore only copies the MJPEG payload and wakes a dedicated
 * vision task; while that task decodes, new frames are dropped. */
#define VISION_TASK_STACK_SIZE 8192
#define VISION_TASK_PRIORITY   1
/* Pin the decode to core 1: the control loop and Wi-Fi both run on core 0
 * (see the boot log), so a dedicated core stops the CPU-heavy software JPEG
 * decode from being preempted, which was inflating the per-frame latency. */
#define VISION_TASK_CORE       1

static const char *TAG = "camera_vision";

static uint8_t *s_xfer_a;
static uint8_t *s_xfer_b;
static uint8_t *s_frame_buffer;
static uint8_t *s_rgb_buffer;
static size_t s_rgb_buffer_size;
static uint8_t *s_jpeg_buffer;
static size_t s_jpeg_buffer_size;
static bool s_started;
static bool s_connected;
static uint32_t s_received_frames;
static uint32_t s_decode_failures;
static uint32_t s_dropped_frames;
static int64_t s_last_frame_us;
static uint16_t s_image_width;
static uint16_t s_image_height;
static infrared_sensor_state_t s_sensor;
static camera_vision_mode_t s_mode = CAMERA_VISION_MODE_LINE;
static ball_vision_result_t s_ball_result;
static portMUX_TYPE s_status_lock = portMUX_INITIALIZER_UNLOCKED;

static volatile bool s_frame_pending;
static volatile uint32_t s_pending_bytes;
static volatile uint32_t s_pending_seq;
static SemaphoreHandle_t s_frame_sem;

/* Snapshot of the latest raw MJPEG payload for HTTP streaming. The vision
 * task publishes frames into s_stream_buffer; the stream server copies them
 * out under s_stream_lock. Optional: allocation failure only disables the
 * viewer, never the vision pipeline. */
static uint8_t *s_stream_buffer;
static SemaphoreHandle_t s_stream_lock;
static size_t s_stream_bytes;
static uint32_t s_stream_seq;

static void publish_connection(bool connected)
{
    taskENTER_CRITICAL(&s_status_lock);
    s_connected = connected;
    if (!connected) {
        s_sensor.black_mask = 0U;
        s_image_width = 0U;
        s_image_height = 0U;
        s_ball_result.valid = false;
        pseudo_infrared_reset();
    }
    taskEXIT_CRITICAL(&s_status_lock);
}

static void stream_state_callback(usb_stream_state_t state, void *user_ptr)
{
    (void)user_ptr;
    const bool connected = state == STREAM_CONNECTED;
    publish_connection(connected);
    ESP_LOGI(TAG, "USB camera %s", connected ? "connected" : "disconnected");
}

static void camera_frame_callback(uvc_frame_t *frame, void *user_ptr)
{
    (void)user_ptr;
    if (frame == NULL || frame->data == NULL || frame->data_bytes == 0U ||
        frame->frame_format != UVC_FRAME_FORMAT_MJPEG ||
        frame->data_bytes > s_jpeg_buffer_size) {
        taskENTER_CRITICAL(&s_status_lock);
        ++s_decode_failures;
        taskEXIT_CRITICAL(&s_status_lock);
        return;
    }

    taskENTER_CRITICAL(&s_status_lock);
    ++s_received_frames;
    if (s_frame_pending) {
        ++s_dropped_frames;
        taskEXIT_CRITICAL(&s_status_lock);
        return;
    }
    const uint32_t seq = s_received_frames;
    taskEXIT_CRITICAL(&s_status_lock);

    /* The vision task never reads s_jpeg_buffer until s_frame_pending is set
     * below, so copying outside the critical section is safe. */
    memcpy(s_jpeg_buffer, frame->data, frame->data_bytes);

    taskENTER_CRITICAL(&s_status_lock);
    s_pending_bytes = (uint32_t)frame->data_bytes;
    s_pending_seq = seq;
    s_frame_pending = true;
    taskEXIT_CRITICAL(&s_status_lock);

    xSemaphoreGive(s_frame_sem);
}

static void vision_task(void *arg)
{
    (void)arg;
    while (1) {
        if (xSemaphoreTake(s_frame_sem, pdMS_TO_TICKS(1000)) != pdTRUE) {
            continue;
        }

        taskENTER_CRITICAL(&s_status_lock);
        const uint32_t bytes = s_pending_bytes;
        const uint32_t seq = s_pending_seq;
        const camera_vision_mode_t mode = s_mode;
        taskEXIT_CRITICAL(&s_status_lock);

        esp_jpeg_image_cfg_t decode = {
            .indata = s_jpeg_buffer,
            .indata_size = bytes,
            .outbuf = s_rgb_buffer,
            .outbuf_size = (uint32_t)s_rgb_buffer_size,
            .out_format = JPEG_IMAGE_FORMAT_RGB888,
            .out_scale = jpeg_scale_for(active_decode_scale(mode)),
        };
        esp_jpeg_image_output_t output = {0};
        infrared_sensor_state_t sensor = {0};
        ball_vision_result_t ball_result = {0};
        const esp_err_t decoded = esp_jpeg_decode(&decode, &output);
        const int64_t now_us = esp_timer_get_time();
        if (decoded == ESP_OK && output.width > 0U && output.height > 0U) {
            if (mode == CAMERA_VISION_MODE_PUSH) {
                /* Push task: pseudo-infrared sampling is skipped; run the
                 * ball/pocket detector on the coarse grid instead. */
                (void)ball_vision_analyze(s_rgb_buffer, output.width,
                                          output.height,
                                          (size_t)output.width * 3U,
                                          &ball_result);
            } else {
                pseudo_infrared_sample_rgb888(s_rgb_buffer, output.width,
                                              output.height,
                                              (size_t)output.width * 3U,
                                              &sensor);
            }
        }

        /* Publish only after analyzing this JPEG so the browser image and
         * scan-row diagnostics refer to the same frame instead of the overlay
         * lagging one full software-decode interval behind. */
        if (s_stream_lock != NULL &&
            xSemaphoreTake(s_stream_lock, pdMS_TO_TICKS(20)) == pdTRUE) {
            memcpy(s_stream_buffer, s_jpeg_buffer, bytes);
            s_stream_bytes = bytes;
            s_stream_seq = seq;
            xSemaphoreGive(s_stream_lock);
        }

        taskENTER_CRITICAL(&s_status_lock);
        s_frame_pending = false;
        s_last_frame_us = now_us;
        if (decoded != ESP_OK || output.width == 0U || output.height == 0U) {
            ++s_decode_failures;
            s_sensor.black_mask = 0U;   /* no usable frame -> all white */
            if (mode == CAMERA_VISION_MODE_PUSH) {
                s_ball_result.valid = false;
            }
        } else {
            s_image_width = (uint16_t)output.width;
            s_image_height = (uint16_t)output.height;
            if (mode == CAMERA_VISION_MODE_PUSH) {
                s_ball_result = ball_result;
                s_ball_result.frame_seq = seq;
            } else {
                s_sensor = sensor;
            }
        }
        taskEXIT_CRITICAL(&s_status_lock);
    }
}

static void *allocate_psram(size_t size)
{
    return heap_caps_aligned_alloc(16, size,
                                   MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}

esp_err_t camera_vision_start(void)
{
    if (s_started) {
        return ESP_OK;
    }
    if (!esp_psram_is_initialized()) {
        ESP_LOGE(TAG, "PSRAM is required for UVC frame buffers");
        return ESP_ERR_INVALID_STATE;
    }

    s_xfer_a = allocate_psram(CAMERA_UVC_BUFFER_SIZE);
    s_xfer_b = allocate_psram(CAMERA_UVC_BUFFER_SIZE);
    s_frame_buffer = allocate_psram(CAMERA_UVC_BUFFER_SIZE);
    s_jpeg_buffer_size = CAMERA_UVC_BUFFER_SIZE;
    s_jpeg_buffer = allocate_psram(s_jpeg_buffer_size);
    /* Sized for the coarser (smaller scale) of LINE / PUSH grids; decode
     * outbuf_size is a capacity, so one buffer serves both modes. */
    s_rgb_buffer_size =
        (CAMERA_FRAME_WIDTH / CAMERA_WORK_SCALE) *
        (CAMERA_FRAME_HEIGHT / CAMERA_WORK_SCALE) * 3U;
    s_rgb_buffer = allocate_psram(s_rgb_buffer_size);
    if (s_xfer_a == NULL || s_xfer_b == NULL || s_frame_buffer == NULL ||
        s_jpeg_buffer == NULL || s_rgb_buffer == NULL) {
        ESP_LOGE(TAG, "failed to allocate UVC buffers in PSRAM");
        return ESP_ERR_NO_MEM;
    }

    s_frame_sem = xSemaphoreCreateBinary();
    if (s_frame_sem == NULL) {
        ESP_LOGE(TAG, "failed to create frame semaphore");
        return ESP_ERR_NO_MEM;
    }

    /* Stream snapshot is best-effort; running out of memory only turns the
     * network viewer off. */
    s_stream_buffer = allocate_psram(CAMERA_UVC_BUFFER_SIZE);
    s_stream_lock = xSemaphoreCreateMutex();
    if (s_stream_buffer == NULL || s_stream_lock == NULL) {
        ESP_LOGW(TAG, "stream snapshot unavailable; HTTP viewer disabled");
        if (s_stream_lock != NULL) {
            vSemaphoreDelete(s_stream_lock);
            s_stream_lock = NULL;
        }
        if (s_stream_buffer != NULL) {
            heap_caps_free(s_stream_buffer);
            s_stream_buffer = NULL;
        }
    }

    if (xTaskCreatePinnedToCore(vision_task, "vision_proc",
                                VISION_TASK_STACK_SIZE, NULL,
                                VISION_TASK_PRIORITY, NULL,
                                VISION_TASK_CORE) != pdPASS) {
        ESP_LOGE(TAG, "failed to create vision task");
        return ESP_ERR_NO_MEM;
    }

    const uvc_config_t config = {
        .frame_width = CAMERA_FRAME_WIDTH,
        .frame_height = CAMERA_FRAME_HEIGHT,
        .frame_interval = FPS2INTERVAL(CAMERA_FRAME_FPS),
        .xfer_buffer_size = CAMERA_UVC_BUFFER_SIZE,
        .xfer_buffer_a = s_xfer_a,
        .xfer_buffer_b = s_xfer_b,
        .frame_buffer_size = CAMERA_UVC_BUFFER_SIZE,
        .frame_buffer = s_frame_buffer,
        .frame_cb = camera_frame_callback,
        .frame_cb_arg = NULL,
        .format = UVC_FORMAT_MJPEG,
        .xfer_type = UVC_XFER_UNKNOWN,
    };
    esp_err_t result = uvc_streaming_config(&config);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "uvc_streaming_config failed: %s",
                 esp_err_to_name(result));
        return result;
    }
    result = usb_streaming_state_register(stream_state_callback, NULL);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "state callback registration failed: %s",
                 esp_err_to_name(result));
        return result;
    }
    result = usb_streaming_start();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "usb_streaming_start failed: %s",
                 esp_err_to_name(result));
        return result;
    }
    taskENTER_CRITICAL(&s_status_lock);
    s_started = true;
    taskEXIT_CRITICAL(&s_status_lock);
    ESP_LOGI(TAG, "UVC started: MJPEG %dx%d @ %d fps",
             CAMERA_FRAME_WIDTH, CAMERA_FRAME_HEIGHT, CAMERA_FRAME_FPS);
    return ESP_OK;
}

esp_err_t camera_vision_get_status(camera_vision_status_t *status)
{
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    taskENTER_CRITICAL(&s_status_lock);
    *status = (camera_vision_status_t) {
        .started = s_started,
        .connected = s_connected,
        .received_frames = s_received_frames,
        .decode_failures = s_decode_failures,
        .dropped_frames = s_dropped_frames,
        .last_frame_us = s_last_frame_us,
        .image_width = s_image_width,
        .image_height = s_image_height,
        .infrared = s_sensor,
    };
    taskEXIT_CRITICAL(&s_status_lock);
    return s_started ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_err_t camera_vision_set_mode(camera_vision_mode_t mode)
{
    if (mode != CAMERA_VISION_MODE_LINE && mode != CAMERA_VISION_MODE_PUSH) {
        return ESP_ERR_INVALID_ARG;
    }
    taskENTER_CRITICAL(&s_status_lock);
    s_mode = mode;
    taskEXIT_CRITICAL(&s_status_lock);
    ESP_LOGI(TAG, "mode -> %s (decode scale %u)",
             mode == CAMERA_VISION_MODE_PUSH ? "PUSH" : "LINE",
             active_decode_scale(mode));
    return ESP_OK;
}

camera_vision_mode_t camera_vision_get_mode(void)
{
    camera_vision_mode_t mode;
    taskENTER_CRITICAL(&s_status_lock);
    mode = s_mode;
    taskEXIT_CRITICAL(&s_status_lock);
    return mode;
}

esp_err_t camera_vision_get_ball_result(ball_vision_result_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    taskENTER_CRITICAL(&s_status_lock);
    if (!s_started) {
        taskEXIT_CRITICAL(&s_status_lock);
        return ESP_ERR_INVALID_STATE;
    }
    *out = s_ball_result;
    taskEXIT_CRITICAL(&s_status_lock);
    return ESP_OK;
}

esp_err_t camera_vision_get_jpeg(uint8_t *dst, size_t dst_capacity,
                                 size_t *out_bytes, uint32_t *out_seq)
{
    if (dst == NULL || out_bytes == NULL || out_seq == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_stream_buffer == NULL || s_stream_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_stream_lock, pdMS_TO_TICKS(50)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t result;
    if (s_stream_seq == 0U) {
        result = ESP_ERR_INVALID_STATE;
    } else if (s_stream_bytes == 0U || s_stream_bytes > dst_capacity) {
        result = ESP_ERR_NO_MEM;
    } else {
        memcpy(dst, s_stream_buffer, s_stream_bytes);
        *out_bytes = s_stream_bytes;
        *out_seq = s_stream_seq;
        result = ESP_OK;
    }
    xSemaphoreGive(s_stream_lock);
    return result;
}
