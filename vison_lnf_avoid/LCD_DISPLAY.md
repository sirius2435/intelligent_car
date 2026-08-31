# LCD 实时显示：3 路电机转速 + 超声波距离

本工程在原有"巡线 + 避障"逻辑完全不变的基础上，新增了 LQ_TFT18SPI V3.3
1.8 寸 SPI 液晶屏（ST7735S，128×160，IPS）实时仪表盘。

## 屏幕内容（每 200 ms 刷新一次，只重绘变化的行，无闪烁）

```
SMART CAR                 （青色标题）
LEFT                      （白色标签）
 123 RPM                  （黄色，带符号，二倍字号）
RIGHT
 123 RPM
REAR
 123 RPM
DIST
 12.3 CM                  （绿色，中值滤波后）
US:OK / US:NO ECHO ...    （底部状态行）
```

- 转速单位 rpm（转/分钟），负值表示倒转。
- 距离显示最近 5 次有效测距的**中值**，单位 cm、保留 1 位小数。
  无回波时数值行保持最后有效值，状态行变红提示 `US:NO ECHO`。

## 新增文件

| 文件 | 作用 |
| --- | --- |
| `main/lcd.h` / `main/lcd.c` | ST7735S SPI 驱动（4 线 SPI + D/C 预回调）、8×8 点阵字库、矩形/文本绘制 |
| `main/lcd_monitor.h` / `main/lcd_monitor.c` | 仪表盘任务：200 ms 采样 3 路编码器算转速、读取超声波、中值滤波、刷屏 |

修改的文件：`main/board_config.h`（新增 LCD 与标定配置）、`main/CMakeLists.txt`
（新增源文件与 `esp_driver_spi` 依赖）、`main/main.c`（启动仪表盘，失败仅告警不影响跑车）。

## 接线（屏 → ESP32-S3）

| 屏引脚 | GPIO | 说明 |
| --- | --- | --- |
| CS | 36 | SPI 片选 |
| SCK (SCL) | 35 | SPI 时钟 |
| SDI (SDA/MOSI) | 45 | SPI 数据（只写） |
| D/C (DC) | 21 | 0=命令 1=数据 |
| RST (RES) | 20 | 复位，见下方重要提示 |
| VCC / GND | 3.3V / GND | |
| BLK (LED) | 悬空或接 3.3V | 常亮背光；若接 GPIO，在 `LCD_BLK_GPIO` 填引脚号 |

### ⚠ RST 用了 GPIO20（USB-Serial-JTAG 副控制台引脚）

sdkconfig 启用了 `CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG`，该外设占用
GPIO19/20。驱动初始化时会调用 `gpio_reset_pin(20)` 把它收回为普通 GPIO：

- 日志不受影响：主控制台是 UART0（GPIO43/44）。
- **烧录/监视请使用板子的 UART 下载口（CH340/CP2102 桥芯片那个口）**。
  程序运行后，原生 USB 口（走 GPIO19/20 的那个 Type-C）会断开，属正常现象。
- 若你只能用原生 USB 口烧录，请把屏的 RST 改接其它空闲脚（如 47 或 48），
  并同步修改 `board_config.h` 里的 `LCD_RST_GPIO`。

## 编译烧录

```bat
:: 你的常规流程即可，例如：
idf.py build
idf.py -p COMx flash monitor
```

## 转速标定（重要）

`board_config.h` 中 `WHEEL_COUNTS_PER_REV` 默认按 **13 线霍尔 × 4 倍频 ×
减速比 30 = 1560** 计。若与实际电机不符，rpm 显示会整体偏一个固定比例：

1. 把车架起，串口日志看 `enc=` 计数；
2. 手转驱动轮整一圈，记下计数差；
3. 把该值填入 `WHEEL_COUNTS_PER_REV`。

## 测距误差为什么 < 5 cm

- 回波计时用 `esp_timer`（微秒级），声速 343 m/s 下 1 cm 对应约 58 µs，
  硬件计时分辨率贡献的误差 ≪ 1 mm；
- HC-SR04 模块标称精度 ±3 mm 左右，15° 波束角内对平整墙面稳定；
- 显示值取最近 5 次读数的**中值**，剔除偶发毛刺（毛刺通常是一个 58 µs
  整数倍的大跳变，中值滤波可完全滤除）；
- 温度每变化 1 °C 引入约 0.17% 声速误差，1 m 距离约 1.7 mm，可忽略；
  若在强风/极端温差环境可用温度补偿进一步提高。
- 注意：超声波打不到的目标（吸音材料、小倾角斜面、>4 m）会报 NO_ECHO。

## 故障排查

| 现象 | 处理（都在 `board_config.h` 改一个宏） |
| --- | --- |
| 白屏 | 确认型号是 ST7735S；调 `LCD_X_OFFSET`/`LCD_Y_OFFSET`（试 2/3 与 1/3） |
| 颜色发负片 | `LCD_INVERT_COLORS` 改 0 |
| 红蓝互换 | `LCD_SWAP_RB` 改 1 |
| 花屏/偶尔错位 | `LCD_CLOCK_HZ` 降到 `20000000` |
| 背光不亮 | BLK 接 3.3V，或填 `LCD_BLK_GPIO` 并接 GPIO |
| 屏幕完全不亮且无日志 | 检查 VCC/GND 与杜邦线；确认没用被其它外设占用的引脚 |
