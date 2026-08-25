# 四路红外循迹传感器测试工程

本工程面向 ESP32-S3 和 LQ_R4CHVB 四路红外检测模块，结构与
`D:\esp-projects\blink-led` 相同，使用 ESP-IDF 5.4.x。

## 安装方向和逻辑

从车体视角由左至右：

```text
车体左侧                                      车体右侧
通道 4          通道 3          通道 2          通道 1
bit 3           bit 2           bit 1           bit 0
```

模块使用 3.3V 供电并与 ESP32-S3 共地。根据产品资料，检测到黑色时输出低
电平，检测到白色时输出高电平。串口中的 `B` 表示黑色，`W` 表示白色。

## 填写 GPIO

打开 `main/board_config.h`，把以下四个 `-1` 替换为实际 GPIO：

```c
#define IR_CHANNEL_4_GPIO  (-1)
#define IR_CHANNEL_3_GPIO  (-1)
#define IR_CHANNEL_2_GPIO  (-1)
#define IR_CHANNEL_1_GPIO  (-1)
```

没有填写完整时，程序不会读取传感器，板载 RGB 灯会红色闪烁。程序也会
检查 GPIO 是否有效、是否重复以及是否与 GPIO38 指示灯冲突。

## 指示灯含义

板载 GPIO38 WS2812 用低亮度颜色表示循迹状态：

| 颜色 | 状态 |
|---|---|
| 红色 | 四路均未检测到黑线 |
| 蓝色 | 黑线偏左较多 |
| 青色 | 黑线略偏左 |
| 绿色 | 黑线居中 |
| 黄色 | 黑线略偏右 |
| 粉色 | 黑线偏右较多 |
| 紫色 | 四路全黑，可能是交叉线或停止标志 |

串口日志会输出准确的四路状态，例如：

```text
CH4..CH1=WBBW, black_mask=0x6
```

## 构建与烧录

在已配置 ESP-IDF 5.4.x 的终端执行：

```powershell
cd D:\esp-projects\infrared-sensor
idf.py set-target esp32s3
idf.py build
idf.py -p COM8 flash monitor
```

采样周期为 10 ms，连续三次相同读数后才接受状态变化，以减少阈值附近的
抖动。模块上的四个电位器仍需根据安装高度分别调节。
