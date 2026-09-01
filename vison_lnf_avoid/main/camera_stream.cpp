/*
 * Wi-Fi soft-AP + HTTP viewer for the USB camera (PC / phone, no app).
 *
 * The camera already delivers complete MJPEG frames (see camera_vision.c),
 * so "streaming" is a straight copy of the raw JPEG payloads into the HTTP
 * response -- no re-encoding, near-zero CPU.
 *
 *   http://192.168.4.1/          viewer page: MJPEG picture + live overlay
 *   http://192.168.4.1/status    camera + vision result JSON
 *   http://192.168.4.1:81/stream MJPEG stream (one client at a time)
 *
 * A streaming handler blocks its esp_http_server task until the viewer
 * disconnects, so the stream runs on a SECOND server instance on port 81;
 * page/status on port 80 always stay responsive.
 */
#include "camera_stream.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "board_config.h"
#include "camera_vision.h"

#include "esp_err.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#define STREAM_TAG "camera_stream"

#if CAMERA_FLIP_HORIZONTAL && CAMERA_FLIP_VERTICAL
#define STREAM_IMAGE_TRANSFORM "transform:scale(-1,-1);"
#elif CAMERA_FLIP_HORIZONTAL
#define STREAM_IMAGE_TRANSFORM "transform:scaleX(-1);"
#elif CAMERA_FLIP_VERTICAL
#define STREAM_IMAGE_TRANSFORM "transform:scaleY(-1);"
#else
#define STREAM_IMAGE_TRANSFORM ""
#endif

static const char VIEWER_PAGE_HTML[] = R"html(<!doctype html>
<html lang="zh">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>智能车视觉监视</title>
<style>
body{margin:0;background:#101418;color:#e8eef4;font:14px/1.5 system-ui,"Segoe UI",sans-serif}
.wrap{max-width:560px;margin:0 auto;padding:12px}
.camera{position:relative;width:100%}
.camera img{width:100%;background:#000;border-radius:8px;display:block;min-height:120px;)html"
STREAM_IMAGE_TRANSFORM
R"html(}
.camera canvas{position:absolute;inset:0;width:100%;height:100%;pointer-events:none;border-radius:8px}
h3{margin:4px 0 8px}
.badges{display:flex;flex-wrap:wrap;gap:6px;margin:10px 0}
.b{padding:3px 10px;border-radius:14px;background:#232b33;font-size:13px}
.ok{background:#1d5c37}.bad{background:#6b2222}
.kv{display:flex;justify-content:space-between;border-bottom:1px solid #232b33;padding:3px 0}
.bar{position:relative;height:14px;background:#232b33;border-radius:7px;margin:6px 0 14px}
.bar i{position:absolute;top:0;bottom:0;width:6px;background:#4da3ff;border-radius:3px;transform:translateX(-50%)}
.tip{color:#8aa;font-size:12px}
</style>
</head>
<body><div class="wrap">
<h3>摄像头画面 <small style="color:#8aa" id="host"></small></h3>
<div class="camera"><img id="cam" alt="等待画面（需要摄像头出帧）…"><canvas id="overlay"></canvas></div>
<div class="badges" id="badges"></div>
<div class="kv"><span>横向误差 lateral_error（正=线在车右侧）</span><span id="lerr">-</span></div>
<div class="bar"><i id="lmark" style="left:50%"></i></div>
<div class="kv"><span>航向误差 heading_error</span><span id="herr">-</span></div>
<div class="kv"><span>置信度 confidence</span><span id="conf">-</span></div>
<div class="kv"><span>有效扫描行 valid_rows</span><span id="rows">-</span></div>
<div class="kv"><span>线宽 px</span><span id="wpx">-</span></div>
<div class="kv"><span>帧序号 / 累计丢帧 / 解码失败</span><span id="seq">-</span></div>
<div class="kv"><span>画面年龄</span><span id="age">-</span></div>
<p class="tip">黄/红线是算法的6条实际扫描行；绿点是该行最终选中的黑线中心，红线表示该行没有合格黑线。青色虚线是图像几何中心。端口81的裸 MJPEG 保持相机原始方向。</p>
</div>
<script>
document.getElementById('host').textContent = location.hostname;
const cam = document.getElementById('cam');
const overlay = document.getElementById('overlay');
let latestStatus = null;
cam.src = 'http://' + location.hostname + ':81/stream';
const B = (t, cls) => '<span class="b ' + (cls||'') + '">' + t + '</span>';
function drawOverlay(s){
  const w = cam.clientWidth, h = cam.clientHeight;
  if(!w || !h || !s || !s.img_w || !s.img_h) return;
  const dpr = window.devicePixelRatio || 1;
  const pw = Math.max(1, Math.round(w*dpr));
  const ph = Math.max(1, Math.round(h*dpr));
  if(overlay.width !== pw || overlay.height !== ph){
    overlay.width = pw; overlay.height = ph;
  }
  const c = overlay.getContext('2d');
  c.setTransform(dpr,0,0,dpr,0,0);
  c.clearRect(0,0,w,h);
  c.save();
  c.strokeStyle = 'rgba(40,220,255,.8)';
  c.setLineDash([6,5]);
  c.beginPath(); c.moveTo(w/2,0); c.lineTo(w/2,h); c.stroke();
  c.restore();
  const ys = s.scan_y || [], xs = s.scan_x || [];
  for(let i=0;i<ys.length;i++){
    const valid = ((s.scan_mask >>> i) & 1) !== 0;
    const y = (ys[i]+0.5)*h/s.img_h;
    c.strokeStyle = valid ? 'rgba(255,205,40,.82)' : 'rgba(255,65,65,.9)';
    c.lineWidth = 1.5;
    c.beginPath(); c.moveTo(0,y); c.lineTo(w,y); c.stroke();
    c.fillStyle = valid ? '#ffe066' : '#ff5555';
    c.font = 'bold 12px system-ui,sans-serif';
    c.fillText('R'+i+(valid?'':' ×'),5,Math.max(12,y-3));
    if(valid){
      const x = (xs[i]+0.5)*w/s.img_w;
      c.fillStyle = '#31e981';
      c.beginPath(); c.arc(x,y,5,0,Math.PI*2); c.fill();
      c.strokeStyle = '#062b17'; c.lineWidth = 2; c.stroke();
    }
  }
}
async function poll(){
  try{
    const r = await fetch('/status', {cache:'no-store'});
    const s = await r.json();
    document.getElementById('badges').innerHTML =
      B('USB ' + (s.connected ? '已连接' : '未连接'), s.connected ? 'ok' : 'bad') +
      B('引导线 ' + (s.line ? '找到' : '丢失'), s.line ? 'ok' : 'bad') +
      (s.finish ? B('终点标记','ok') : '') +
      (s.corner ? B('拐角','ok') : '') +
      B('解码失败 ' + s.dec_fail, s.dec_fail ? 'bad' : '');
    document.getElementById('lerr').textContent = s.lerr;
    document.getElementById('lmark').style.left = ((s.lerr + 1000) / 20) + '%';
    document.getElementById('herr').textContent = s.herr;
    document.getElementById('conf').textContent = s.conf;
    document.getElementById('rows').textContent = s.rows + '/6，ROI ' + s.roi_top + '–' + s.roi_bottom + '%';
    document.getElementById('wpx').textContent = s.width_px;
    document.getElementById('seq').textContent = s.seq + ' / ' + s.drop + ' / ' + s.dec_fail;
    document.getElementById('age').textContent = s.age_ms < 0 ? '从未' : (s.age_ms + ' ms 前');
    latestStatus = s;
    requestAnimationFrame(() => drawOverlay(s));
  }catch(e){
    document.getElementById('badges').innerHTML = B('状态获取失败', 'bad');
  }
  setTimeout(poll, 300);
}
cam.addEventListener('load', () => drawOverlay(latestStatus));
window.addEventListener('resize', () => drawOverlay(latestStatus));
poll();
</script>
</body></html>)html";

static const char STREAM_PART[] =
    "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static volatile int s_stream_clients;

/* ------------------------------------------------------------------ */
/* Port 80: viewer page + status JSON                                   */
/* ------------------------------------------------------------------ */

static esp_err_t page_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, VIEWER_PAGE_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t status_handler(httpd_req_t *req)
{
    camera_vision_status_t camera = {0};
    (void)camera_vision_get_status(&camera);
    const int64_t age_ms = camera.last_frame_us == 0 ? -1 :
        (esp_timer_get_time() - camera.last_frame_us) / 1000LL;

    char json[640];
    const int len = snprintf(json, sizeof(json),
        "{\"started\":%d,\"connected\":%d,\"frames\":%u,\"drop\":%u,"
        "\"dec_fail\":%u,\"seq\":%u,\"age_ms\":%lld,\"line\":%d,"
        "\"finish\":%d,\"corner\":%d,\"conf\":%u,\"lerr\":%d,\"herr\":%d,"
        "\"rows\":%u,\"width_px\":%u,\"img_w\":%u,\"img_h\":%u,"
        "\"roi_top\":%u,\"roi_bottom\":%u,\"scan_mask\":%u,"
        "\"scan_y\":[%u,%u,%u,%u,%u,%u],"
        "\"scan_x\":[%u,%u,%u,%u,%u,%u]}",
        camera.started ? 1 : 0,
        camera.connected ? 1 : 0,
        (unsigned)camera.received_frames,
        (unsigned)camera.dropped_frames,
        (unsigned)camera.decode_failures,
        (unsigned)camera.vision.sequence,
        (long long)age_ms,
        camera.vision.line_found ? 1 : 0,
        camera.vision.finish_marker ? 1 : 0,
        camera.vision.corner_detected ? 1 : 0,
        (unsigned)camera.vision.confidence,
        camera.vision.lateral_error,
        camera.vision.heading_error,
        (unsigned)camera.vision.valid_rows,
        (unsigned)camera.vision.line_width_pixels,
        (unsigned)camera.vision.image_width,
        (unsigned)camera.vision.image_height,
        (unsigned)VISION_ROI_TOP_PERCENT,
        (unsigned)VISION_ROI_BOTTOM_PERCENT,
        (unsigned)camera.vision.scan_valid_mask,
        (unsigned)camera.vision.scan_y[0],
        (unsigned)camera.vision.scan_y[1],
        (unsigned)camera.vision.scan_y[2],
        (unsigned)camera.vision.scan_y[3],
        (unsigned)camera.vision.scan_y[4],
        (unsigned)camera.vision.scan_y[5],
        (unsigned)camera.vision.scan_center_x[0],
        (unsigned)camera.vision.scan_center_x[1],
        (unsigned)camera.vision.scan_center_x[2],
        (unsigned)camera.vision.scan_center_x[3],
        (unsigned)camera.vision.scan_center_x[4],
        (unsigned)camera.vision.scan_center_x[5]);
    if (len < 0 || (size_t)len >= sizeof(json)) {
        return ESP_ERR_NO_MEM;
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, json, (ssize_t)len);
}

/* ------------------------------------------------------------------ */
/* Port 81: MJPEG stream                                                */
/* ------------------------------------------------------------------ */

static esp_err_t stream_handler(httpd_req_t *req)
{
    if (__atomic_fetch_add(&s_stream_clients, 1, __ATOMIC_SEQ_CST) >=
        STREAM_MAX_CLIENTS) {
        (void)__atomic_sub_fetch(&s_stream_clients, 1, __ATOMIC_SEQ_CST);
        httpd_resp_set_status(req, "503 Busy");
        httpd_resp_set_type(req, "text/plain; charset=utf-8");
        (void)httpd_resp_sendstr(req,
            "已有客户端在看流；请先关闭它的页面再刷新。\r\n");
        return ESP_OK;
    }

    esp_err_t result = ESP_OK;
    uint8_t *frame = static_cast<uint8_t *>(heap_caps_aligned_alloc(
        16, CAMERA_UVC_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (frame == NULL) {
        (void)__atomic_sub_fetch(&s_stream_clients, 1, __ATOMIC_SEQ_CST);
        httpd_resp_set_status(req, "503 No Memory");
        httpd_resp_set_type(req, "text/plain; charset=utf-8");
        (void)httpd_resp_sendstr(req, "no memory for stream buffer\r\n");
        return ESP_OK;
    }

    result = httpd_resp_set_type(
        req, "multipart/x-mixed-replace; boundary=frame");
    char part[72];
    uint32_t last_seq = 0;
    bool first = true;
    while (result == ESP_OK) {
        size_t bytes = 0;
        uint32_t seq = 0;
        const esp_err_t got = camera_vision_get_jpeg(
            frame, CAMERA_UVC_BUFFER_SIZE, &bytes, &seq);
        if (got == ESP_OK && (first || seq != last_seq)) {
            const int part_len =
                snprintf(part, sizeof(part), STREAM_PART, (unsigned)bytes);
            if (part_len <= 0 || (size_t)part_len >= sizeof(part) ||
                httpd_resp_send_chunk(req, part, part_len) != ESP_OK ||
                httpd_resp_send_chunk(req, (const char *)frame,
                                      (ssize_t)bytes) != ESP_OK ||
                httpd_resp_send_chunk(req, "\r\n", 2) != ESP_OK) {
                break; /* viewer gone (or TCP backpressure) */
            }
            last_seq = seq;
            first = false;
        } else {
            vTaskDelay(pdMS_TO_TICKS(STREAM_POLL_MS));
        }
    }

    heap_caps_free(frame);
    (void)__atomic_sub_fetch(&s_stream_clients, 1, __ATOMIC_SEQ_CST);
    return ESP_OK;
}

/* ------------------------------------------------------------------ */

static esp_err_t wifi_ap_start(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(STREAM_TAG, "nvs_flash_init failed: %s",
                 esp_err_to_name(err));
        return err;
    }

    err = esp_netif_init();
    if (err != ESP_OK) {
        ESP_LOGE(STREAM_TAG, "esp_netif_init failed: %s",
                 esp_err_to_name(err));
        return err;
    }
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(STREAM_TAG, "event loop failed: %s", esp_err_to_name(err));
        return err;
    }
    if (esp_netif_create_default_wifi_ap() == NULL) {
        ESP_LOGE(STREAM_TAG, "esp_netif_create_default_wifi_ap failed");
        return ESP_FAIL;
    }

    const wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(STREAM_TAG, "esp_wifi_init failed: %s",
                 esp_err_to_name(err));
        return err;
    }
    err = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (err != ESP_OK) {
        ESP_LOGE(STREAM_TAG, "esp_wifi_set_storage failed: %s",
                 esp_err_to_name(err));
        return err;
    }
    err = esp_wifi_set_mode(WIFI_MODE_AP);
    if (err != ESP_OK) {
        ESP_LOGE(STREAM_TAG, "esp_wifi_set_mode failed: %s",
                 esp_err_to_name(err));
        return err;
    }

    wifi_config_t ap_config = { 0 };
    (void)strlcpy((char *)ap_config.ap.ssid, STREAM_AP_SSID,
                  sizeof(ap_config.ap.ssid));
    ap_config.ap.ssid_len = (uint8_t)strlen(STREAM_AP_SSID);
    const bool open_network = STREAM_AP_PASSWORD[0] == '\0';
    (void)strlcpy((char *)ap_config.ap.password, STREAM_AP_PASSWORD,
                  sizeof(ap_config.ap.password));
    ap_config.ap.channel = STREAM_AP_CHANNEL;
    ap_config.ap.max_connection = STREAM_AP_MAX_CONNECTIONS;
    ap_config.ap.authmode = open_network ? WIFI_AUTH_OPEN
                                         : WIFI_AUTH_WPA2_PSK;
    ap_config.ap.pmf_cfg.capable = true;
    ap_config.ap.pmf_cfg.required = false;
    err = esp_wifi_set_config(WIFI_IF_AP, &ap_config);
    if (err != ESP_OK) {
        ESP_LOGE(STREAM_TAG, "esp_wifi_set_config failed: %s",
                 esp_err_to_name(err));
        return err;
    }

    err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(STREAM_TAG, "esp_wifi_start failed: %s",
                 esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(STREAM_TAG, "soft-AP \"%s\" channel %d up; open http://192.168.4.1/",
             STREAM_AP_SSID, STREAM_AP_CHANNEL);
    return ESP_OK;
}

esp_err_t camera_stream_start(void)
{
    static bool started;
    static esp_err_t start_result = ESP_ERR_INVALID_STATE;
    if (started) {
        return start_result;
    }

    const esp_err_t wifi_result = wifi_ap_start();
    if (wifi_result != ESP_OK) {
        return wifi_result;
    }
    started = true;
    esp_err_t first_error = ESP_OK;

    /* Page + status server. */
    httpd_config_t page_cfg = HTTPD_DEFAULT_CONFIG();
    page_cfg.server_port = STREAM_HTTP_PORT;
    page_cfg.max_open_sockets = 4;
    page_cfg.stack_size = 6144;
    page_cfg.lru_purge_enable = true;
    httpd_handle_t page_server = NULL;
    esp_err_t server_result = httpd_start(&page_server, &page_cfg);
    if (server_result == ESP_OK) {
        const httpd_uri_t page_uri = {
            .uri = "/", .method = HTTP_GET, .handler = page_handler,
        };
        const httpd_uri_t status_uri = {
            .uri = "/status", .method = HTTP_GET, .handler = status_handler,
        };
        server_result = httpd_register_uri_handler(page_server, &page_uri);
        if (server_result == ESP_OK) {
            server_result = httpd_register_uri_handler(page_server, &status_uri);
        }
        if (server_result != ESP_OK) {
            ESP_LOGE(STREAM_TAG, "page URI registration failed: %s",
                     esp_err_to_name(server_result));
            first_error = server_result;
        }
    } else {
        ESP_LOGE(STREAM_TAG, "page httpd on port %d failed",
                 STREAM_HTTP_PORT);
        first_error = server_result;
    }

    /* Dedicated stream server: its handler runs until the viewer leaves. */
    httpd_config_t stream_cfg = HTTPD_DEFAULT_CONFIG();
    stream_cfg.server_port = STREAM_MJPEG_PORT;
    stream_cfg.ctrl_port = page_cfg.ctrl_port + 1;
    stream_cfg.max_open_sockets = STREAM_MAX_CLIENTS + 1;
    stream_cfg.stack_size = 6144;
    stream_cfg.lru_purge_enable = true;
    httpd_handle_t stream_server = NULL;
    server_result = httpd_start(&stream_server, &stream_cfg);
    if (server_result == ESP_OK) {
        const httpd_uri_t stream_uri = {
            .uri = "/stream", .method = HTTP_GET, .handler = stream_handler,
        };
        server_result = httpd_register_uri_handler(stream_server, &stream_uri);
        if (server_result == ESP_OK) {
            ESP_LOGI(STREAM_TAG,
                     "MJPEG stream ready: http://192.168.4.1:%d/stream",
                     STREAM_MJPEG_PORT);
        } else {
            ESP_LOGE(STREAM_TAG, "stream URI registration failed: %s",
                     esp_err_to_name(server_result));
            if (first_error == ESP_OK) {
                first_error = server_result;
            }
        }
    } else {
        ESP_LOGE(STREAM_TAG, "stream httpd on port %d failed",
                 STREAM_MJPEG_PORT);
        if (first_error == ESP_OK) {
            first_error = server_result;
        }
    }
    start_result = first_error;
    return start_result;
}
