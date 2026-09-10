# ESP32-S3 三轮全向视觉小车（巡线避障 + 推球入洞）

基于 **ESP32-S3** 的三轮全向小车，用**单个 USB UVC 摄像头**（物理红外板已断开）完成两阶段任务：

1. **视觉巡线 + 超声波避障**：沿黑线行驶，遇障一次性横移绕行，识别终点横条后停车。
2. **推球入洞**：终点确认后切到视觉伺服，把**红球推入左洞、蓝球推入右洞**。

辅助功能：MG90S 双轴云台（固定俯角）、ST7735 LCD 仪表盘、Wi-Fi MJPEG 远程查看器。

> 所有可调参数集中在 [`main/board_config.h`](main/board_config.h)；本文档中的数值均以该文件为准。

---

## 一、系统架构

分层为 **感知 → 决策 → 运动 → 执行**。所有决策逻辑（巡线/避障/推球）都是"每 tick 输入传感器、输出运动指令"的**纯状态机**：不调用硬件、不 sleep，因此能在 PC 上用 gcc 单独编译测试（见 `tests/`）。`main.c` 用一个 **10 ms 控制循环**串起两阶段调度。

```
        USB UVC 摄像头 (MJPEG 480x320 @25fps)
                    │  usb_stream 回调(仅拷贝+唤醒)
                    ▼
   camera_vision.c  专用视觉任务(绑核 core1) · 软件 JPEG 解码 → RGB888
        │ LINE 模式(scale8, 60x40)        │ PUSH 模式(scale4, 120x80)
        ▼                                 ▼
   pseudo_infrared.c                  ball_vision.c
   4 块采样 → 4bit 黑线掩码             红球/蓝球/暗洞口检测
        │                                 │
        ▼                                 ▼
   line_follow.c(巡线)                ball_push.c(推球 14 态)
   obstacle_avoidance.c(避障)             │
        │ forward/lateral/turn            │ + drive_mode
        └──────────────┬──────────────────┘
                       ▼
        main.c  10ms 循环 + 两阶段调度(RUN_PHASE_LINE → RUN_PHASE_BALL)
                       ▼
        drive.c  三轮运动混合 + 轮级 PI 反馈  →  motor.c  TB6612 三路 PWM
                       ▲
   encoder.c(三轮计数) / ultrasonic.c(测距) ── 反馈 ──┘

 旁路(不阻塞控制环)：lcd_monitor.c(仪表盘) · camera_stream.cpp(WiFi 查看器) · camera_gimbal.c(云台)
```

### 模块职责

| 文件 | 职责 |
|------|------|
| `board_config.h` | 全局唯一配置源：引脚、控制参数、视觉阈值 |
| `infrared_sensor.h` | 共享类型 `infrared_sensor_state_t` 与 `IR_*_MASK` 掩码（物理红外板实现已移除） |
| `camera_vision.c/h` | 视觉管线：UVC 取流 → 软解 RGB888；LINE/PUSH 双模式；加锁快照 |
| `pseudo_infrared.c/h` | 从解码帧采样 4 个像素块生成 4bit 黑线掩码 + 终点全黑确认 |
| `ball_vision.c/h` | 纯图像分析：红球/蓝球/暗洞口检测（逻辑坐标） |
| `line_follow.c/h` | 巡线状态机（PD 误差 + 急弯 + 丢线搜索 + 终点停车） |
| `obstacle_avoidance.c/h` | 一次性横移避障状态机 |
| `ball_push.c/h` | 推球入洞视觉伺服状态机（14 态） |
| `drive.c/h` | 三轮全向运动混合 + 轮级编码器 PI 反馈 |
| `motor.c/h` | TB6612FNG 三路 PWM 驱动 |
| `encoder.c/h` | 三轮正交霍尔编码器（PCNT x4） |
| `ultrasonic.c/h` | HC-SR04 测距（中断 + 队列） |
| `lcd.c/h` · `lcd_monitor.c/h` | ST7735 SPI 屏驱动 + 仪表盘任务 |
| `camera_gimbal.c/h` | MG90S 双轴云台（pan/tilt，软限位钳位） |
| `camera_stream.cpp/h` | Wi-Fi soft-AP + HTTP MJPEG 查看器（唯一 C++ 文件） |
| `main.c` | `app_main` + 10 ms 控制循环 + 两阶段调度 |

---

## 二、硬件与接线

所有引脚定义见 `board_config.h`。

| 外设 | 引脚 |
|------|------|
| 左轮电机 | IN1=41, IN2=42, PWM=2（`MOTOR_LEFT_REVERSED=1`） |
| 右轮电机 | IN1=7, IN2=6, PWM=5（`MOTOR_RIGHT_REVERSED=1`） |
| 后轮电机 | IN1=8, IN2=18, PWM=17（`MOTOR_REAR_REVERSED=0`） |
| 电机使能 STBY | GPIO4（运行前拉高，停车拉低） |
| 左编码器 | A=40, B=39 |
| 右编码器 | A=15, B=16（`ENCODER_RIGHT_REVERSED=1`） |
| 后编码器 | A=3, B=46 |
| 超声波 HC-SR04 | TRIG=14, ECHO=13（ECHO 是 5 V，必须分压/电平转换） |
| 云台舵机 | PAN=45, TILT=21（50 Hz） |
| LCD（ST7735S） | CS=9, SCK=10, SDI=11, DC=12, RST=38, BLK=-1（常亮） |
| USB 摄像头 | GPIO19/20（ESP32-S3 USB D-/D+ 内部固定） |

> **GPIO19/20 已独占给 USB UVC 摄像头**，因此烧录/串口监视请使用板子的 **UART 下载口**（CH340/CP2102 桥芯片那个）；程序运行后原生 USB 口（Type-C）会断开，属正常现象。

### 驱动接口约定

驱动层用 `forward / lateral / turn` 三个分量（命令范围 −1000..1000）：

- 正 `forward` 直行；正 `lateral` **向左**横移；正 `turn` **向右**转。
- 三轮全向混合（`drive_mix_motion`）：横移时三轮都参与，比例 `[左, 右, 后] = [-lateral/2, +lateral/2, -lateral]`；纯巡线为 `[forward+turn, forward-turn, 0]`。
- **首次运行必须架空车轮**，分别确认正 `forward` 直行、正 `lateral` 左移、正 `turn` 右转。单个轮子方向错误时改对应的 `MOTOR_*_REVERSED`，不要在状态机里改符号。

横移/低速接近/越障直行阶段会自动启用**三个独立的编码器 PI 速度环**（50 ms 窗口测速 + 最低启动 PWM 补偿静摩擦）；巡线和拐角仍为开环输出。

---

## 三、视觉系统

### 摄像头与解码

- UVC 摄像头请求 **480×320 MJPEG @25 fps**。
- 软件 JPEG 解码很重，放在**专用视觉任务并绑到 core 1**（控制环与 Wi-Fi 在 core 0），解码期间新帧直接丢弃，避免抢占导致延迟膨胀。
- 双解码尺度：巡线 `CAMERA_DECODE_SCALE=8`（→ 60×40 工作图）；推球 `PUSH_CAMERA_DECODE_SCALE=4`（→ 120×80，用于识别小球/洞口）。确认 END 后才切换，不影响前半程帧率。
- 当前摄像头倒装，`CAMERA_FLIP_HORIZONTAL=1` + `CAMERA_FLIP_VERTICAL=1`（等效旋转 180°）。视觉算法与 Wi-Fi 主页共用此校正；端口 81 的裸 MJPEG 流保持摄像头原始方向。

### 伪红外巡线采样（`pseudo_infrared.c`）

在画面 **70% 高度**（`PSEUDO_IR_SAMPLE_ROW_PERCENT`）处固定采样 **4 个像素块**，等效替代物理红外板的 4 通道：

- 块边长 `PSEUDO_IR_BLOCK_SIZE=4`（60 px 宽图上约 5 px）。
- 4 个通道的横向中心为图宽百分比：**CH1=65%、CH2=54%、CH3=47%、CH4=35%**（CH1=车右，CH4=车左）。内侧 CH2/CH3 跨越正中，使居中黑线同时点亮两块（误差 0），不会落入中缝读成全白丢线。
- 每块用**行自适应阈值**（该行亮度均值 − `PSEUDO_IR_BLACK_MARGIN=40`）判暗像素，暗像素数 ≥ `PSEUDO_IR_BLOCK_DARK_MIN=2` 即视为压线。
- 输出 4bit 掩码，位序与红外板一致：**bit0=CH1=车右，bit3=CH4=车左**。
- **终点**：横穿 4 块的黑线使掩码全黑 `0x0F`，连续 `PSEUDO_IR_FINISH_CONFIRM_FRAMES=3` 帧确认后锁存 `finish_detected` 并触发停车。

> 调参后需在低速实地验证三条：①居中线点亮内侧两块（WBBW，误差 0）；②偏到某通道只点亮该块；③终点横条稳定确认全黑。

### 球与洞口检测（`ball_vision.c`）

在 PUSH 模式的 120×80 图上检测（逻辑坐标与 Wi-Fi 查看器一致，y=0 为远端桌边/洞口）：

- **红球**：纯色度优势判据（R−G、R−B ≥ 25），无绝对亮度门限，抗欠曝。
- **蓝球**：B−R、B−G 色度优势 + 饱和度/亮度下限，比旧的"白球"判据更抗中性背景与阴影。
- **洞口**：桌面远端（y < 55% 高度）的暗、低饱和区域，且需触及画面远边。
- 输出球心/半径（半径作距离代理）、洞口位置，以及红/蓝/暗区像素计数（串口日志的 `px=[r=.. b=..]` 与 `/status` 的 `rp`/`blp`/`pp` 字段，用于现场判断色度阈值偏紧还是偏松）。

---

## 四、任务一：巡线 + 避障（`RUN_PHASE_LINE`）

### 巡线状态机（`line_follow.c`）

`WAITING_LINE`（等线）→ `TRACKING`（PD 误差巡线）→ `CORNER_CANDIDATE/ROTATE/EXIT`（急弯武装与旋转通过）→ `LOST_SEARCH`（丢线低速旋转搜索）→ `STOPPED`（终点全黑或超时停车）。

主要参数：`LINE_BASE_FORWARD=170`、`LINE_KP=70`、`LINE_KD=15`、`LINE_TURN_LIMIT=250`；急弯确认窗口 `LINE_CORNER_CONFIRM_WINDOW_MS=450`（须大于一帧间隔）；上电后 `LINE_START_DELAY_MS=3000` 才开始运动。

### 避障状态机（`obstacle_avoidance.c`）

持续读前向距离，进入 `AVOID_SLOW_DISTANCE_MM=150` 减速，连续 `AVOID_TRIGGER_CONFIRM_SAMPLES=2` 次 ≤ `AVOID_TRIGGER_DISTANCE_MM=50` 触发一次性避障：

```
ARMED → BRAKE → STRAFE_LEFT → FORWARD_PASS → STRAFE_RIGHT_FIND_LINE → COMPLETE
                                                                  └→ FAULT_STOP
```

- **无后退阶段**：制动后直接左横移绕行（旧文档描述的"短距离后退"已不存在）。
- `STRAFE_LEFT`：左横移直到前方连续确认无障碍（`AVOID_CLEAR_DISTANCE_MM=120`）；确认边缘消失后仍保持横移 `AVOID_LEFT_CLEARANCE_MS=250` 留板边余量。
- `FORWARD_PASS`：编码器定距前进越过障碍（`AVOID_FORWARD_TARGET_COUNTS`），对左右轮分别闭环防低 PWM 堵转。
- `STRAFE_RIGHT_FIND_LINE`：右移直到视觉连续识别到黑线并对中，然后恢复巡线（`just_completed` 会重置巡线控制器）。
- **避障只执行一次**。测距超时、任一阶段超时、编码器堵转或横移超安全上限都会进入 `FAULT_STOP` **锁定停车**（需复位开发板）。推球阶段避障结构性关闭（超声波会把球/桌沿当障碍）。

---

## 五、任务二：推球入洞（`RUN_PHASE_BALL`）

**只有确认 END 横条后**才启动（普通丢线会锁定停车）。切换时把摄像头解码切到 scale 4，并做一次云台俯角调整（`BALL_GIMBAL_TILT_DEG=110`，等待 `BALL_GIMBAL_SETTLE_MS=600`）。注意该角度当前与 `CAMERA_TILT_SERVO_CENTER_DEG` 相同，而 `camera_gimbal_init()` 开机已把俯角回中，所以这次调整目前是空操作、只白等 600 ms；若推球确实需要与巡线不同的俯角，把两个宏改成不同值即可。

任务映射（`board_config.h`）：**红球（FIRST_COLOR=0）→ 左洞（FIRST_POCKET=0）**，**蓝球（SECOND_COLOR=1）→ 右洞（SECOND_POCKET=1）**。

控制原理（`ball_push.c`）：`START_FORWARD → FIND_BALL/SCAN → APPROACH（相位 0 横移居中 → 相位 1 远场转向对准洞 + 可选 FAR_SPRINT → 相位 2 强制直行接近）→ ALIGN（只横移，把球挪到洞所在的列）→ HARD PUSH（定时纯直行）→ EGRESS 后退 → POST_EGRESS 转向 + 前进 → 下一球`。

**两条关键规则：ALIGN 阶段既不前进、也不旋转。** 朝向在球还很远时（APPROACH 相位 1）就已修正完毕；贴着球旋转正是把蓝球滚丢、进而丢失目标的原因。

14 态状态机（`ball_push.h`）：

| 状态 | 作用 |
|------|------|
| `START_FORWARD` | 从起跑线固定短直行（`PUSH_START_FORWARD_MS`） |
| `FIND_BALL` / `SCAN` | 获取目标球；找不到则旋转扫描（`PUSH_SCAN_*`） |
| `APPROACH_BALL` | 先横移居中（phase 0），再强制直行接近（phase 1，`PUSH_APPROACH_*`） |
| `ALIGN` | **只横移**：把球挪到洞所在的列，再保持居中一段确认窗口（`PUSH_ALIGN_STRAFE_*`、`PUSH_ALIGN_DEADBAND_PX`、`PUSH_ALIGN_CONFIRM_MS`）。球已进入保险杠距离（`PUSH_ALIGN_SAFE_RADIUS_PX`）则转 `BACKOFF` |
| `BACKOFF` | 球太近：定距后退（`PUSH_BACKOFF_SPEED/COUNTS`）再回 `ALIGN`。**没有循环次数上限**，来回弹跳最终由整任务超时兜底 |
| `FAR_SPRINT` | 远场已对准时开环冲刺（`PUSH_FAR_SPRINT_ENABLE=1`，绕过近场视觉盲区）；行程由球的 `cy` 估算，编码器定距、堵转则转 `BACKOUT` |
| `PUSH` | 纯直行强推，**定时结束**（`PUSH_HARD_FORWARD`、`PUSH_HARD_PUSH_MS`）：无转向修正、无定距目标、无堵转监督、也无入袋像素确认——时间一到即记为已入袋 |
| `BACKOUT` | **纯决策态，自身不产生任何运动**：`FAR_SPRINT` 堵转后在同一 tick 内决定回 `FIND_BALL`（本球未入袋）还是 `EGRESS` |
| `EGRESS` | 进洞后后退离场（`PUSH_EGRESS_*`） |
| `POST_EGRESS_TURN/FORWARD` | 离场后转向 + 前进到利于找下一球的位置 |
| `DONE` / `FAULT_STOP` | 两球入洞（粘滞）/ 超时·堵转·扫描腿耗尽（粘滞锁定） |

- **没有"每球重试上限"**：`BACKOFF`/`BACKOUT` 的回环只受整任务超时 `BALL_TASK_TIMEOUT_MS=180000` 约束，超时即 `FAULT_STOP`。
- 定距阶段（`SCAN`/`BACKOFF`/`FAR_SPRINT`/`EGRESS`/`POST_EGRESS_*`）的行程统一统计为 **`|Δ左| + |Δ右|` 的累计值**（不是位移，也不是较慢轮），因此对位阶段来回修正、方向反转都不会误判堵转；同一套 `PUSH_SCAN_STALL_MS` / `PUSH_SCAN_STALL_COUNTS` 窗口兼做堵转看门狗。`PUSH` 和 `START_FORWARD`/`ALIGN` 则是纯时间窗口，不看编码器。
- 视觉帧率仅几帧/秒而控制环 10 ms，故每帧只消费一次，帧间保持上次对位指令。
- `FAULT_STOP` 为锁定态，需复位开发板；两球完成后也保持停车。

---

## 六、Wi-Fi 摄像头查看器（`camera_stream.cpp`）

不接串口、不装 App：小车自建 Wi-Fi 热点，浏览器即可看方向校正后的画面 + 算法实时判定叠加。随主程序自动启动（在相机启动**之前**，即使相机故障 `/status` 也能报因）。

**连接**：SSID `vison_car` / 密码 `12345678`（提示"无互联网"忽略），浏览器打开 `http://192.168.4.1/`。

| URL | 用途 |
|-----|------|
| `http://192.168.4.1/` | 观看页：MJPEG 画面 + 伪红外采样块/通道掩码/状态叠加 |
| `http://192.168.4.1/status` | 纯 JSON：`mask`、`img_w/h`、`block`、`row_pct`、球/洞结果与帧统计 |
| `http://192.168.4.1:81/stream` | 摄像头原始方向的裸 MJPEG 流（VLC "网络串流"亦可） |

- 主页 4 个方框即伪红外采样块（采样行 70% 高度，黄色虚线标出），亮绿=压线、暗=未压。叠加坐标与算法使用的镜像校正坐标一致。线在通道位置但方块没亮 → 块内暗像素未达 `PSEUDO_IR_BLOCK_DARK_MIN`，需校准采样行/块大小/阈值。
- 参数在 `board_config.h` 末尾：`STREAM_AP_SSID/PASSWORD/CHANNEL=6`、`STREAM_HTTP_PORT=80`、`STREAM_MJPEG_PORT=81`、`STREAM_MAX_CLIENTS=1`、`STREAM_POLL_MS=40`。
- 推流直接转发摄像头 MJPEG 原码流、**不重编码**，CPU 开销近乎为零。

**限制/排障**：MJPEG 流同一时间只支持 1 个客户端（第二个得 503），页面/状态不受限；看不到热点 → 查串口 `camera_stream` 标签日志（推流失败只告警、不影响跑车）；流一直转圈 → 相机还没出首帧（看 `/status` 的 `started/connected/age_ms`）。

---

## 七、LCD 实时仪表盘（`lcd.c` + `lcd_monitor.c`）

LQ_TFT18SPI V3.3 1.8″ SPI 屏（ST7735S，128×160，IPS）。独立任务每 `LCD_MONITOR_PERIOD_MS=200` 刷新，只重绘变化行、无闪烁；面板缺失时小车照常运行（`main.c` 中失败仅告警）。

```
SMART CAR              （青色标题）
LEFT   123 RPM         （黄色，带符号，二倍字号）
RIGHT  123 RPM
REAR   123 RPM
DIST   12.3 CM         （绿色，最近 5 次有效测距的中值）
US:OK / US:NO ECHO     （底部状态行，无回波变红）
```

- 转速单位 rpm（负值=倒转）；距离单位 cm、保留 1 位小数，取最近 `LCD_MONITOR_DISTANCE_MEDIAN=5` 次有效读数的**中值**滤除毛刺。
- **转速标定**：`WHEEL_COUNTS_PER_REV=1560`（默认 13 线霍尔 ×4 倍频 ×30 减速比）。若 rpm 整体偏一个比例：架空车、串口看 `enc=`、手转驱动轮整一圈记下计数差，填入该宏。
- 面板调优（`board_config.h`）：画面偏移调 `LCD_X_OFFSET/Y_OFFSET`；颜色负片改 `LCD_INVERT_COLORS`；红蓝互换改 `LCD_SWAP_RB`；花屏降 `LCD_CLOCK_HZ`（当前 10 MHz）。

---

## 八、构建与烧录

在 ESP-IDF 5.4.x 的 PowerShell 环境中：

```powershell
cd D:\idf_intelligent_car_txgayay\vison_lnf_avoid
idf.py set-target esp32s3
idf.py build
idf.py -p COM端口 flash monitor
```

- 依赖组件（`main/idf_component.yml`）：`usb_stream`（UVC 取流）、`esp_jpeg`（软解码）。`cmake_utilities` 是 `usb_stream` 带入的传递依赖，只出现在 `dependencies.lock` 中。
- 分区表 `partitions.csv`：nvs + phy_init + factory app（8 MB）。
- 若烧录报 `Could not open COMx, the port is busy or doesn't exist`：检查设备是否插好、COM 口是否变化、串口监视器是否占用端口，或让设备进入下载模式后重试。
- **务必先架空车轮确认三轮方向**，再在安全低速场地测试。

---

## 九、宿主机单元测试（`tests/`）

6 个纯逻辑单元测试用 PC 端 gcc 编译运行（无需硬件），覆盖最易错、迭代最慢的状态机与运动学，**当前全部通过**：

| 测试 | 覆盖 |
|------|------|
| `drive_mix_test.c` | 三轮运动混合 + 3 种反馈模式（**精确 PWM 数值断言**） |
| `pseudo_infrared_test.c` | 4 通道掩码生成（含真实 60×40 尺寸几何、终点确认） |
| `line_follow_sequence_test.c` | 巡线状态机时序 |
| `obstacle_avoidance_test.c` | 避障状态机各阶段 |
| `ball_push_sequence_test.c` | 推球 14 态转移、各阶段运动分量与驱动模式、终态发布 |
| `ball_vision_test.c` | 红/蓝球 + 洞口检测（合成图像） |

编译运行（任意安装了 gcc 的环境，在工作区根目录）：

```sh
gcc -std=gnu17 -Wall -Wextra -Imain -Itests/stubs -o /tmp/t_dm  tests/drive_mix_test.c            main/drive.c
gcc -std=gnu17 -Wall -Wextra -Imain -Itests/stubs -o /tmp/t_pir tests/pseudo_infrared_test.c      main/pseudo_infrared.c
gcc -std=gnu17 -Wall -Wextra -Imain -Itests/stubs -o /tmp/t_lf  tests/line_follow_sequence_test.c main/line_follow.c
gcc -std=gnu17 -Wall -Wextra -Imain -Itests/stubs -o /tmp/t_oa  tests/obstacle_avoidance_test.c   main/obstacle_avoidance.c
gcc -std=gnu17 -Wall -Wextra -Imain -Itests/stubs -o /tmp/t_bv  tests/ball_vision_test.c          main/ball_vision.c -lm
gcc -std=gnu17 -Wall -Wextra -Imain -Itests/stubs -o /tmp/t_bp  tests/ball_push_sequence_test.c   main/ball_push.c main/ball_vision.c -lm
```

`tests/stubs/esp_err.h` 提供脱离 ESP-IDF 的最小桩；部分测试内联硬件桩（如 `motor_set_all`）。这些测试**不被 `idf.py build` 自动运行**，也不被 `idf_component.yml` 引用，需手动编译。

> **改过 `board_config.h` 里的 `DRIVE_*` PWM 标定后，`drive_mix_test.c` 会失败——这是故意的。** 它把静摩擦前馈、P+I 修正与抗积分饱和的输出值写死成具体数字（每个断言旁都给出了推导公式），目的是在标定漂移时强制你确认新值是有意的。请重新推导后更新期望值，**不要**直接把实际输出拷进去。

---

## 十、故障排查

| 现象 | 处理 |
|------|------|
| 小车锁定不动、日志 `locked stop` | `FAULT_STOP`：相机断连/帧超时、避障堵转或超时、推球重试超限。需**复位开发板** |
| 巡线大幅摆动 | 帧率延迟 + 4 通道量化 + KP 偏高；可降 `LINE_KP`、确认采样几何，见 `PSEUDO_IR_*` |
| 线在通道位但方块不亮 | 块内暗像素未达 `PSEUDO_IR_BLOCK_DARK_MIN`，校准采样行/块大小/`BLACK_MARGIN` |
| 终点不停车/误停 | 检查 `PSEUDO_IR_FINISH_CONFIRM_FRAMES` 与终点横条宽度；宽暗目标可能使自适应阈值塌缩 |
| 某轮不转/方向反 | 架空车确认 `MOTOR_*_REVERSED` 与 `ENCODER_*_REVERSED`；看串口 `vision_car` 标签每 100 ms 打的 `enc=[左,右,后]` 与 `motion=[forward,lateral,turn]`，命令为正而计数不动即该轮堵转或接线错 |
| 烧录打不开 COM 口 | 见"构建与烧录"：设备/端口/占用/下载模式 |
| LCD 白屏/负片/花屏 | 见"LCD 仪表盘"面板调优宏 |
| 看不到 Wi-Fi 热点 | 查串口 `camera_stream` 日志；推流失败不影响跑车 |
