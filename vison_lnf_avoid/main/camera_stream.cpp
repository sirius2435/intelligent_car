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
#include "ball_push.h"

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
.pushStatus{margin:10px 0;padding:10px 12px;border-radius:10px;background:#1b232b;border:1px solid #2b3741}
.pushStatus .title{font-size:20px;font-weight:700}
.pushStatus .sub{margin-top:3px;color:#9fb0bf;font-size:12px}
.done{background:#1d5c37!important;border-color:#2b8b56!important}
</style>
</head>
<body><div class="wrap">
<h3>摄像头画面 <small style="color:#8aa" id="host"></small></h3>
<div class="camera"><img id="cam" alt="等待画面（需要摄像头出帧）…"><canvas id="overlay"></canvas></div>
<div class="pushStatus" id="pushStatus" style="display:none"><div class="title" id="pushTitle">-</div><div class="sub" id="pushSub">-</div></div>
<div class="badges" id="badges"></div>
<div class="kv"><span id="kvlabel">伪红外通道（左→右）</span><span id="mask">-</span></div>
<div class="kv"><span>帧序号 / 累计丢帧 / 解码失败</span><span id="seq">-</span></div>
<div class="kv"><span>画面年龄</span><span id="age">-</span></div>
<p class="tip">循迹阶段：绿色方块=4个采样块，亮起表示压到黑线（左→右：通道4→通道1），全黑=终点标记。推球阶段：半透明圆+十字=红/蓝球识别（坐标=120×80工作图像素），青色虚线框=识别到的暗色底袋区域，橙色虚线以下阴影=蓝球丢弃区（BALL_BLUE_MAX_CY_PX，底部车体蓝色干扰不识别）。端口81的裸 MJPEG 保持相机原始方向。</p>
</div>
<script>
document.getElementById('host').textContent = location.hostname;
const cam = document.getElementById('cam');
const overlay = document.getElementById('overlay');
let latestStatus = null;
cam.src = 'http://' + location.hostname + ':81/stream';
const B = (t, cls) => '<span class="b ' + (cls||'') + '">' + t + '</span>';
function fitCanvas(){
  const w = cam.clientWidth, h = cam.clientHeight;
  if(!w || !h) return null;
  const dpr = window.devicePixelRatio || 1;
  const pw = Math.max(1, Math.round(w*dpr));
  const ph = Math.max(1, Math.round(h*dpr));
  if(overlay.width !== pw || overlay.height !== ph){
    overlay.width = pw; overlay.height = ph;
  }
  const c = overlay.getContext('2d');
  c.setTransform(dpr,0,0,dpr,0,0);
  c.clearRect(0,0,w,h);
  return {c: c, w: w, h: h};
}

/* Line-following overlay: centre line, sample row, four pseudo-IR blocks. */
function drawLineOverlay(s){
  const f = fitCanvas(); if(!f) return;
  const c = f.c, w = f.w, h = f.h;
  c.save();
  c.strokeStyle = 'rgba(40,220,255,.8)';
  c.setLineDash([6,5]);
  c.beginPath(); c.moveTo(w/2,0); c.lineTo(w/2,h); c.stroke();
  c.restore();
  const y = (s.row_pct/100)*h;
  c.strokeStyle = 'rgba(255,205,40,.5)';
  c.setLineDash([3,4]);
  c.beginPath(); c.moveTo(0,y); c.lineTo(w,y); c.stroke();
  c.setLineDash([]);
  const bw = s.block*w/s.img_w, bh = s.block*h/s.img_h;
  const cp = s.ch_pct;
  if(!cp || cp.length < 4) return;  /* /status always sends it; never guess a geometry */
  for(let i=0;i<4;i++){
    const on = ((s.mask >>> i) & 1) !== 0;
    const x = cp[i]/100*w;
    c.fillStyle = on ? 'rgba(49,233,129,.55)' : 'rgba(255,255,255,.05)';
    c.strokeStyle = on ? '#31e981' : 'rgba(255,255,255,.6)';
    c.lineWidth = 1.5;
    c.fillRect(x-bw/2, y-bh/2, bw, bh);
    c.strokeRect(x-bw/2, y-bh/2, bw, bh);
    c.fillStyle = on ? '#062b17' : '#8aa';
    c.font = 'bold 11px system-ui,sans-serif';
    c.fillText('CH'+(i+1), x-bw/2+2, y-bh/2+12);
  }
}

/* Pocket-push overlay: red/blue ball blobs with crosshair + detected pocket
 * regions (dark pockets at the far table edge) framed in cyan, plus the
 * blue-ball rejection band below BALL_BLUE_MAX_CY_PX shaded in orange.
 * Coordinates arrive in LOGICAL working-grid px (img_w x img_h) and are
 * scaled to the displayed image. */
function drawPushOverlay(s){
  const f = fitCanvas(); if(!f || !s.img_w || !s.img_h) return;
  const c = f.c, w = f.w, h = f.h;
  const X = x => x*w/s.img_w, Y = y => y*h/s.img_h;
  const R = r => Math.max(2, r*w/s.img_w);
  /* Blue-ball cutoff zone: blobs whose centroid lands in the shaded band are
   * discarded by the firmware (chassis / wiring / blue hardware down there). */
  if(s.blue_max_cy > 0 && s.blue_max_cy < s.img_h){
    const yc = Y(s.blue_max_cy);
    c.fillStyle = 'rgba(255,150,40,.14)';
    c.fillRect(0, yc, w, h-yc);
    c.strokeStyle = 'rgba(255,150,40,.8)'; c.lineWidth = 1.5;
    c.setLineDash([6,4]);
    c.beginPath(); c.moveTo(0,yc); c.lineTo(w,yc); c.stroke();
    c.setLineDash([]);
    c.fillStyle = 'rgba(255,170,70,.95)'; c.font = 'bold 10px system-ui,sans-serif';
    c.fillText('蓝球丢弃区 cy>' + s.blue_max_cy, 4, Math.min(h-4, yc+12));
  }
  const balls = [['红', s.r, 'rgba(255,70,70,.9)'], ['蓝', s.b, 'rgba(70,130,255,.95)']];
  for(const [label, bb, col] of balls){
    if(!bb || !bb[0]) continue;
    const cx = X(bb[1]), cy = Y(bb[2]), rad = R(bb[3]);
    c.beginPath(); c.arc(cx,cy,rad,0,Math.PI*2);
    c.fillStyle = col; c.globalAlpha = 0.35; c.fill(); c.globalAlpha = 1;
    c.strokeStyle = '#ffffff'; c.lineWidth = 1.5; c.stroke();
    c.strokeStyle = 'rgba(255,255,255,.85)'; c.lineWidth = 1;
    c.beginPath();
    c.moveTo(cx-rad-3,cy); c.lineTo(cx+rad+3,cy);
    c.moveTo(cx,cy-rad-3); c.lineTo(cx,cy+rad+3);
    c.stroke();
    c.fillStyle = '#000'; c.font = 'bold 11px system-ui,sans-serif';
    c.fillText(label, cx-rad, cy-rad-5);
  }
  const pockets = [['L', s.p1], ['R', s.p2]];
  for(const [side, p] of pockets){
    if(!p || !p[0]) continue;
    const x0 = X(p[1]), y0 = Y(p[2]), x1 = X(p[3]), y1 = Y(p[4]);
    c.strokeStyle = 'rgba(0,230,255,.9)'; c.lineWidth = 1.5;
    c.setLineDash([5,4]);
    c.strokeRect(x0, y0, Math.max(1,x1-x0), Math.max(1,y1-y0));
    c.setLineDash([]);
    c.fillStyle = 'rgba(0,230,255,.9)'; c.font = 'bold 10px system-ui,sans-serif';
    c.fillText('袋'+side, x0+2, y0-4);
  }
}

function drawOverlay(s){
  if(!cam.clientWidth || !cam.clientHeight || !s || !s.img_w || !s.img_h) return;
  if(s.mode === 1){ drawPushOverlay(s); return; }
  drawLineOverlay(s);
}
function pushStatusText(s){
  const left = s.push_attempt === 0 ? '左边袋' : '右边袋';
  const ball = s.push_attempt === 0 ? '红球' : '蓝球';
  /* case numbers must match ball_push_state_t in main/ball_push.h exactly. */
  switch(s.push_state){
    case 0: return '启动直行';                 /* START_FORWARD */
    case 1: return '找' + ball;                /* FIND_BALL */
    case 2: return '追' + ball;                /* APPROACH_BALL */
    case 3: return '扫描找' + ball;             /* SCAN */
    case 4: return s.push_pocket_visible ? '横移对准' + left
                                         : '横移靠近' + ball + '，寻找' + left; /* ALIGN */
    case 5: return '退后重新对准';              /* BACKOFF */
    case 6: return '远场开环冲刺';              /* FAR_SPRINT */
    case 7: return '推' + ball + '入洞';        /* PUSH */
    case 8: return '冲刺受阻，重新找球';         /* BACKOUT */
    case 9: return '退出边界';                 /* EGRESS */
    case 10: return '右转调整位置';             /* POST_EGRESS_TURN */
    case 11: return '前进寻找蓝球';             /* POST_EGRESS_FORWARD */
    case 12: return '两球已完成';               /* DONE */
    case 13: return '故障停止';                 /* FAULT_STOP */
    default: return '未知状态 ' + s.push_state;
  }
}
function updatePushStatus(s){
  const box = document.getElementById('pushStatus');
  const title = document.getElementById('pushTitle');
  const sub = document.getElementById('pushSub');
  if(s.mode !== 1){ box.style.display='none'; return; }
  box.style.display='block';
  box.className = 'pushStatus ' + (s.red_pocketed && s.blue_pocketed ? 'done' : '');
  title.textContent = pushStatusText(s);
  const target = s.push_attempt === 0 ? '红球 → 左边袋' : '蓝球 → 右边袋';
  sub.textContent = target;
}
async function poll(){
  try{
    const r = await fetch('/status', {cache:'no-store'});
    const s = await r.json();
    const pushMode = s.mode === 1;
    let badges = B('USB ' + (s.connected ? '已连接' : '未连接'), s.connected ? 'ok' : 'bad');
    if(pushMode){
      badges += B('状态：' + pushStatusText(s), '');
      badges += B('红球 ' + (s.red_pocketed ? '已推入' : (s.r && s.r[0] ? '识别' : '未识别')), s.red_pocketed ? 'ok' : ((s.r && s.r[0]) ? 'ok' : 'bad'));
      badges += B('蓝球 ' + (s.blue_pocketed ? '已推入' : (s.b && s.b[0] ? '识别' : '未识别')), s.blue_pocketed ? 'ok' : ((s.b && s.b[0]) ? 'ok' : 'bad'));
      badges += B('底袋 ' + ((s.p1 && s.p1[0] ? 1 : 0) + (s.p2 && s.p2[0] ? 1 : 0)) + '/2', '');
    } else if(((s.mask & 0x0F) === 0x0F)){
      badges += B('终点标记','ok');
    }
    badges += B('解码失败 ' + s.dec_fail, s.dec_fail ? 'bad' : '');
    document.getElementById('badges').innerHTML = badges;
    updatePushStatus(s);
    document.getElementById('kvlabel').textContent =
      pushMode ? '球 / 底袋识别（工作图 px）' : '伪红外通道（左→右）';
    let m = '';
    if(pushMode){
      const fmt = a => (a && a[0]) ? '(' + a[1] + ',' + a[2] + ') r' + a[3] : '未识别';
      m = '红球 ' + fmt(s.r) + '　蓝球 ' + fmt(s.b) + '　袋区 x:';
      m += (s.p1 && s.p1[0]) ? '[' + s.p1[1] + '..' + s.p1[3] + ']' : '-';
      m += ' ' + ((s.p2 && s.p2[0]) ? '[' + s.p2[1] + '..' + s.p2[3] + ']' : '-');
    } else {
      for(let i=3;i>=0;i--){ m += ((s.mask >>> i) & 1) ? 'B' : 'W'; }
      m += ' (0x' + s.mask.toString(16).toUpperCase() + ')';
    }
    document.getElementById('mask').textContent = m;
    document.getElementById('seq').textContent = s.frames + ' / ' + s.drop + ' / ' + s.dec_fail;
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
    camera_vision_status_t camera{};   /* C++ value-init: {0} would warn here */
    (void)camera_vision_get_status(&camera);
    const int64_t age_ms = camera.last_frame_us == 0 ? -1 :
        (esp_timer_get_time() - camera.last_frame_us) / 1000LL;

    /* Push-mode vision result: r/b = [present, cx, cy, radius], pockets
     * p1/p2 = [present, x0, y0, x1, y1], all in LOGICAL working-grid px
     * (120x80 in push mode) - the same flip-corrected frame the page shows. */
    const camera_vision_mode_t mode = camera_vision_get_mode();
    ball_vision_result_t ball{};
    (void)camera_vision_get_ball_result(&ball);
    const ball_blob_t *red = ball.valid ?
        ball_vision_find_ball(&ball, BALL_COLOR_RED) : NULL;
    const ball_blob_t *blue = ball.valid ?
        ball_vision_find_ball(&ball, BALL_COLOR_BLUE) : NULL;
    ball_push_status_t push_status{};
    ball_push_get_status(&push_status);

    char json[1024];
    const int len = snprintf(json, sizeof(json),
        "{\"started\":%d,\"connected\":%d,\"frames\":%u,\"drop\":%u,"
        "\"dec_fail\":%u,\"age_ms\":%lld,\"mask\":%u,"
        "\"img_w\":%u,\"img_h\":%u,\"block\":%u,\"row_pct\":%u,"
        "\"blue_max_cy\":%u,"
        "\"ch_pct\":[%u,%u,%u,%u],\"mode\":%d,"
        "\"r\":[%d,%d,%d,%d],\"b\":[%d,%d,%d,%d],"
        "\"p1\":[%d,%d,%d,%d,%d],\"p2\":[%d,%d,%d,%d,%d],"
        "\"push_state\":%d,"
        "\"push_attempt\":%u,"
        "\"push_pocket_visible\":%d,"
        "\"red_pocketed\":%d,"
        "\"blue_pocketed\":%d,"
        "\"rp\":%u,\"blp\":%u,\"pp\":%u}",
        camera.started ? 1 : 0,
        camera.connected ? 1 : 0,
        (unsigned)camera.received_frames,
        (unsigned)camera.dropped_frames,
        (unsigned)camera.decode_failures,
        (long long)age_ms,
        (unsigned)camera.infrared.black_mask,
        (unsigned)camera.image_width,
        (unsigned)camera.image_height,
        (unsigned)PSEUDO_IR_BLOCK_SIZE,
        (unsigned)PSEUDO_IR_SAMPLE_ROW_PERCENT,
        (unsigned)BALL_BLUE_MAX_CY_PX,
        (unsigned)PSEUDO_IR_CH1_CENTER_PERCENT,
        (unsigned)PSEUDO_IR_CH2_CENTER_PERCENT,
        (unsigned)PSEUDO_IR_CH3_CENTER_PERCENT,
        (unsigned)PSEUDO_IR_CH4_CENTER_PERCENT,
        (int)mode,
        red != NULL ? 1 : 0,
        red != NULL ? red->cx : 0,
        red != NULL ? red->cy : 0,
        red != NULL ? red->radius : 0,
        blue != NULL ? 1 : 0,
        blue != NULL ? blue->cx : 0,
        blue != NULL ? blue->cy : 0,
        blue != NULL ? blue->radius : 0,
        ball.pockets[0].visible ? 1 : 0,
        ball.pockets[0].visible ? ball.pockets[0].x0 : 0,
        ball.pockets[0].visible ? ball.pockets[0].y0 : 0,
        ball.pockets[0].visible ? ball.pockets[0].x1 : 0,
        ball.pockets[0].visible ? ball.pockets[0].y1 : 0,
        ball.pockets[1].visible ? 1 : 0,
        ball.pockets[1].visible ? ball.pockets[1].x0 : 0,
        ball.pockets[1].visible ? ball.pockets[1].y0 : 0,
        ball.pockets[1].visible ? ball.pockets[1].x1 : 0,
        ball.pockets[1].visible ? ball.pockets[1].y1 : 0,
        (int)push_status.state,
        push_status.attempt,
        push_status.target_pocket_visible ? 1 : 0,
        push_status.red_pocketed ? 1 : 0,
        push_status.blue_pocketed ? 1 : 0,
        (unsigned)ball.red_px,
        (unsigned)ball.blue_px,
        (unsigned)ball.pocket_px);
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

    uint8_t *frame = static_cast<uint8_t *>(heap_caps_aligned_alloc(
        16, CAMERA_UVC_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (frame == NULL) {
        (void)__atomic_sub_fetch(&s_stream_clients, 1, __ATOMIC_SEQ_CST);
        httpd_resp_set_status(req, "503 No Memory");
        httpd_resp_set_type(req, "text/plain; charset=utf-8");
        (void)httpd_resp_sendstr(req, "no memory for stream buffer\r\n");
        return ESP_OK;
    }

    const esp_err_t result = httpd_resp_set_type(
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

    wifi_config_t ap_config{};   /* C++ value-init: { 0 } would warn here */
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
