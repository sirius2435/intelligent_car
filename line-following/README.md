# ESP32-S3 三轮全向车巡线工程

该工程与 `infrared-sensor` 测试工程并列，整合四路红外传感器、D24A
三电机驱动、120° 三轮全向混控和开环 PD 巡线控制。现有测试工程不会被修改。

## 使用前配置

在 `main/board_config.h` 中填写四路红外传感器 GPIO。安装顺序必须是车体
左侧到右侧依次为 CH4、CH3、CH2、CH1。传感器检测到黑线时输出低电平。

电机 GPIO 沿用已经验证的映射：左轮 Motor D、右轮 Motor A、后轮 Motor B。
首次运行必须架空车轮，确认逻辑前进和左右转向方向。如果单个轮子方向相反，
修改对应的 `MOTOR_*_REVERSED`，不要在巡线算法中修改符号。

## 控制行为

- 正常黑线：按四路加权位置误差进行 PD 转向，弯道自动降低前进速度。
- 四路全白：按最后一次偏差方向低速搜索，1.5 秒未恢复则锁定停车。
- 四路全黑：立即暂停，持续 100 ms 后锁定停车。
- 不连续图案：短时保持最后方向并降速，持续 100 ms 后进入丢线搜索。
- 上电初始化成功后等待 3 秒才开始运动。

停车状态为锁定状态，需要复位开发板才能再次启动。

## 构建

在 ESP-IDF 5.4.x PowerShell 环境中执行：

```powershell
cd D:\33984\桌面\智能车\line-following
idf.py set-target esp32s3
idf.py build
idf.py -p COM端口 flash monitor
```

先使用默认的低速参数测试。调参时先把 `LINE_KD` 设为 0，逐步增加
`LINE_KP`，再增加 `LINE_KD` 抑制摆动，最后才提高 `LINE_BASE_FORWARD`。
