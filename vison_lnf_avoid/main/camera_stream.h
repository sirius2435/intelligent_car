#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Wi-Fi soft-AP + HTTP viewer for the USB camera (PC / phone, no app).
 *
 *   http://192.168.4.1/          viewer page: MJPEG picture + live vision overlay
 *   http://192.168.4.1/status    camera + vision result JSON
 *   http://192.168.4.1:81/stream raw MJPEG stream (one client at a time)
 *
 * Non-fatal by design: any failure is logged and returned, the car logic
 * keeps running with the viewer disabled.
 */
esp_err_t camera_stream_start(void);

#ifdef __cplusplus
}
#endif
