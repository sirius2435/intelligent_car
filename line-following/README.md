# ESP32-S3 三轮全向车巡线工程

该工程与 `infrared-sensor` 测试工程并列，整合四路红外传感器、D24A
三电机驱动、双轮差速驱动和开环 PD 巡线控制。现有测试工程不会被修改。

## 使用前配置

在 `main/board_config.h` 中填写四路红外传感器 GPIO。安装顺序必须是车体
左侧到右侧依次为 CH4、CH3、CH2、CH1。传感器检测到黑线时输出低电平。

电机 GPIO、GPIO4 STBY 和三组 EA/EB 沿用 `wheel-test` 的实际配置：
左轮 Motor D、右轮 Motor A、后轮 Motor B。GPIO4 会在电机运行前拉高，
停车后拉低。

驱动方案：双轮差速——只有两个前轮被驱动，同速前进，正 `turn` 右转
（左轮快、右轮慢），后轮完全不驱动、被动随动。首次运行必须架空车轮：
正 `forward` 应直行，正 `turn` 应右转。若正 forward 倒车或横移、正 turn
左转，记录实测现象再改配置；单个轮子方向相反时修改对应的
`MOTOR_*_REVERSED`，不要在巡线算法中修改符号。

## 控制行为

- 正常黑线：按四路加权位置误差进行 PD 转向，弯道自动降低前进速度。
- 四路全白：按最后一次偏差方向低速搜索，1.5 秒未恢复则锁定停车。
- 四路全黑：立即暂停，持续 100 ms 后锁定停车。
- 不连续图案：短时保持最后方向并降速，持续 100 ms 后进入丢线搜索。
- 上电初始化成功后等待 3 秒才开始运动。
- 首次尚未看到黑线时保持 `WAITING_LINE`，不会在 1.5 秒后锁死；检测到
  黑线后才进入正常巡线。

编码器使用 ESP32-S3 PCNT 做三路 x4 正交计数。串口日志中的 `enc` 是累计
计数，`delta` 是两次日志间的增量，`dt` 是对应时间。当前编码器用于接线、
方向和轮速观测，尚未参与 PWM 闭环。

停车状态为锁定状态，需要复位开发板才能再次启动。

## 构建

在 ESP-IDF 5.4.x PowerShell 环境中执行：

```powershell
cd D:\esp-projects\line-following
idf.py set-target esp32s3
idf.py build
idf.py -p COM端口 flash monitor
```

先使用默认的低速参数测试。调参时先把 `LINE_KD` 设为 0，逐步增加
`LINE_KP`，再增加 `LINE_KD` 抑制摆动，最后才提高 `LINE_BASE_FORWARD`。
