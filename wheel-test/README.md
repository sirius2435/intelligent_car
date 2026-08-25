# ESP32-S3 wheel test

This is a safe, minimal ESP-IDF 5.4 motor test for the D24A four-channel DC
motor driver board. The board contains two TB6612FNG dual H-bridge chips. This
car uses Motor D for the left wheel, Motor A for the right wheel, and Motor B for
the rear wheel; Motor C is intentionally unused.

## Verified motor mapping

| Wheel | D24A output | Direction 1 | Direction 2 | PWM |
|---|---|---|---|---|
| Left | Motor D | DIN1 = GPIO42 | DIN2 = GPIO2 | PWMD = GPIO1 |
| Right | Motor A | AIN1 = GPIO11 | AIN2 = GPIO10 | PWMA = GPIO9 |
| Rear | Motor B | BIN1 = GPIO17 | BIN2 = GPIO16 | PWMB = GPIO15 |

Each wheel also provides two Hall encoder signals. Fill all six placeholders in
`main/board_pins.h` when the ESP32-S3 wiring is known:

| Wheel | D24A encoder labels | ESP32-S3 GPIO |
|---|---|---|
| Left / Motor D | E4A, E4B | `ENCODER_LEFT_A_GPIO`, `ENCODER_LEFT_B_GPIO` |
| Right / Motor A | E1A, E1B | `ENCODER_RIGHT_A_GPIO`, `ENCODER_RIGHT_B_GPIO` |
| Rear / Motor B | E2A, E2B | `ENCODER_REAR_A_GPIO`, `ENCODER_REAR_B_GPIO` |

Use bare GPIO numbers, for example `4`. With all six values left at `-1`, the
firmware logs a warning and continues with the open-loop motor test. A partially
filled encoder mapping is rejected to prevent an unsafe or misleading test.

The D24A board's ON/OFF switch controls standby/enable for both TB6612FNG
chips, so `MOTOR_STBY_GPIO` remains `-1`. Move the switch to `ON` before a test.
The unused Motor C output and its control inputs are not configured by this
firmware.

## Power and first-test checklist

- Lift the car so all three drive wheels are off the ground.
- Never connect a motor directly to an ESP32 GPIO.
- Use a motor supply suitable for the motor and driver; do not power the motors
  from the ESP32 3.3 V pin.
- Connect ESP32 ground, driver logic ground, and motor-supply ground together.
- Confirm the driver's logic input voltage is compatible with ESP32-S3 3.3 V.
- Move the D24A ON/OFF switch to `ON` before starting the test.
- Keep `MOTOR_TEST_SPEED` at 250 for the first test.
- Leave `MOTOR_RUN_REVERSE_TEST` at 0 until forward and stop are verified.

## Build, flash, and monitor

From an ESP-IDF terminal:

```powershell
cd D:\esp-projects\wheel-test
idf.py set-target esp32s3
idf.py build
idf.py -p COM9 flash monitor
```

Replace `COM9` if the board is on another serial port. Exit the monitor with
`Ctrl+]`.

Expected sequence: three-second warning, left wheel (Motor D) for two seconds,
stop, right wheel (Motor A) for two seconds, stop, rear wheel (Motor B) for two
seconds, then permanent stop. Motor C must remain idle throughout the test. Once
the six encoder GPIOs are configured, the monitor also prints a signed x4
quadrature count after each wheel test. If a forward-running wheel reports a
negative count, set its corresponding `ENCODER_*_REVERSED` value to 1.

The motor specification's `512 AB phase` value is not assumed to be a final
wheel-revolution conversion factor. Verify whether it is specified per motor
shaft or gearbox output shaft, then measure the observed x4 count over one wheel
revolution before calculating speed or distance.

If a wheel turns backward during a forward test, change its corresponding
`MOTOR_*_REVERSED` value from 0 to 1. Do not swap wires while power is connected.
