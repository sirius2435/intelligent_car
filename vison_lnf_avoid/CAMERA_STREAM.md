# 摄像头画面远程查看（WiFi 推流）

不接串口、不装任何 App：车自己发一个 WiFi 热点，手机或电脑连上后用浏览器看方向校正后的摄像头画面 + 算法实时判定叠加。

## 使用步骤

1. 正常烧录运行固件（本功能随主程序自动启动）。
2. 手机/电脑连接热点：**SSID `vison_car`，密码 `12345678`**（连接后设备会提示"无互联网"，忽略即可）。
3. 浏览器打开 **http://192.168.4.1/**

页面内容：

| 区域 | 说明 |
|---|---|
| 大图 | 按 `CAMERA_FLIP_HORIZONTAL/VERTICAL` 校正方向的 MJPEG 画面，并叠加算法实际使用的6条扫描行 |
| 徽标行 | USB 相机连接状态 / 引导线是否找到 / 终点 / 拐角 / 解码失败计数 |
| 横向误差条 | `lateral_error`（-1000..1000，正=线在车右侧），蓝色游标位置即误差 |
| 其余行 | `heading_error`、`confidence`、`valid_rows`、线宽、帧序号/丢帧/解码失败、画面年龄 |

叠加图中黄线表示该扫描行找到合格黑线，绿点是最终选中的黑线中心；红线和
`×` 表示该行没有合格目标，青色虚线是图像几何中心。叠加坐标与算法使用的
镜像校正后坐标完全一致。画面里线明明在中间但绿点不在线上，说明算法选中了
阴影、地砖缝或其他黑色区域；扫描行没有绿点则检查 ROI、线宽和亮度阈值。

## URL 一览

| URL | 用途 |
|---|---|
| `http://192.168.4.1/` | 观看页（画面 + 状态叠加） |
| `http://192.168.4.1/status` | 纯 JSON，包含 `scan_y`、`scan_x`、`scan_mask` 和图像尺寸 |
| `http://192.168.4.1:81/stream` | 摄像头原始方向的裸 MJPEG 流（VLC 打开“网络串流”也能看） |

## 实现要点（改了什么）

- `main/camera_stream.cpp`：WiFi soft-AP + 两个 esp_http_server 实例。流在 81 端口独立实例——流式 handler 会一直占用其服务器任务，独立开端口保证页面/状态保持响应。
- `main/camera_vision.c`：vision 任务在完成当前帧解码和分析后，再把对应的 MJPEG 原码流发布到独立 PSRAM 缓冲，避免网页叠加结果系统性落后一帧。
- `main/vision_line.c`：按相机安装配置同时校正算法的水平和垂直坐标；当前两轴均镜像，等效旋转 180°。
- `main/main.c`：在相机启动**之前**启动推流——若相机出问题锁车，`/status` 仍能报告原因，方便排查。
- 推流不做重新编码（摄像头本身输出 MJPEG），CPU 开销近乎为零。

## 参数（`main/board_config.h` 末尾）

```c
#define STREAM_AP_SSID       "vison_car"   // 热点名
#define STREAM_AP_PASSWORD   "12345678"    // 置 "" 为开放网络；WPA2 需 >= 8 字符
#define STREAM_AP_CHANNEL      6
#define STREAM_MJPEG_PORT     81
#define STREAM_MAX_CLIENTS     1
#define STREAM_POLL_MS        40
```

## 已知限制 / 排障

- **MJPEG 流同一时间只支持 1 个客户端**（第二个人打开会得到 503）。页面/状态不受限。
- 画面 ~5 fps 是正常的：帧率由 JPEG 软解码速度决定（`CAMERA_DECODE_SCALE=4`），推流本身不拖慢它。
- 看不到热点 → 看串口日志 `camera_stream` 标签：WiFi 初始化失败会打印原因；推流失败只告警，不影响小车运行。
- 流一直转圈不出图 → 摄像头还没出第一帧（`/status` 看 `started/connected/age_ms`）。
- 若 WiFi 推流时控制环出现抖动（日志 `motion` 明显不稳），先降低看流频率或暂时关推流再跑正赛；平时调试无碍。
