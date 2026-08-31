#pragma once

/* Task 2 disconnects the infrared board. Keep placeholders so the old
 * infrared-only sources can still be used by a separate build if required. */
#define IR_CHANNEL_4_GPIO  (-1)
#define IR_CHANNEL_3_GPIO  (-1)
#define IR_CHANNEL_2_GPIO  (-1)
#define IR_CHANNEL_1_GPIO  (-1)

/* LQ_R4CHVB outputs low while its sensor is over a black line. */
#define IR_BLACK_LEVEL              0
#define IR_SAMPLE_PERIOD_MS        10
#define IR_DEBOUNCE_SAMPLE_COUNT    2

/* D24A/TB6612FNG motor outputs verified by the wheel-test project. */
#define MOTOR_LEFT_IN1_GPIO        41
#define MOTOR_LEFT_IN2_GPIO        42
#define MOTOR_LEFT_PWM_GPIO         2

#define MOTOR_RIGHT_IN1_GPIO       7
#define MOTOR_RIGHT_IN2_GPIO       6
#define MOTOR_RIGHT_PWM_GPIO       5

#define MOTOR_REAR_IN1_GPIO        8
#define MOTOR_REAR_IN2_GPIO        18
#define MOTOR_REAR_PWM_GPIO        17

/* D24A/TB6612 standby/enable input. High enables the motor bridges. */
#define MOTOR_STBY_GPIO           (4)

/* Quadrature Hall encoder inputs copied from the verified wheel-test wiring. */
#define ENCODER_LEFT_A_GPIO       (40)  /* Motor D: E4A */
#define ENCODER_LEFT_B_GPIO       (39)  /* Motor D: E4B */
#define ENCODER_RIGHT_A_GPIO      (15)  /* Motor A: E1A */
#define ENCODER_RIGHT_B_GPIO      (16)  /* Motor A: E1B */
#define ENCODER_REAR_A_GPIO        (3)  /* Motor B: E2A */
#define ENCODER_REAR_B_GPIO       (46)  /* Motor B: E2B */

/* Positive counts correspond to the wheel-test definition of forward. */
#define ENCODER_LEFT_REVERSED       0
#define ENCODER_RIGHT_REVERSED      1
#define ENCODER_REAR_REVERSED       0
#define ENCODER_GLITCH_FILTER_NS 1000

/* Change a value to 1 if that individual motor runs in the wrong direction. */
#define MOTOR_LEFT_REVERSED         1
#define MOTOR_RIGHT_REVERSED        1
#define MOTOR_REAR_REVERSED         0

/*
 * Two-wheel differential drive: only the two front wheels are driven.
 * Equal speeds = forward; positive turn = right turn (left wheel faster
 * than right). The rear wheel is passive and never driven.
 */

/* Open-loop line-following parameters; all motor commands use -1000..1000. */
#define LINE_BASE_FORWARD          160
#define LINE_MIN_FORWARD           100
#define LINE_ERROR_SLOWDOWN         20
#define LINE_KP                     70
#define LINE_KD                     15                                          
#define LINE_TURN_LIMIT            250

#define LINE_CORNER_ARM_MS              30
#define LINE_CORNER_CONFIRM_WINDOW_MS  120
#define LINE_CORNER_APPROACH_FORWARD   120
#define LINE_CORNER_ROTATE_FORWARD       0
#define LINE_CORNER_ROTATE_TURN         200
#define LINE_CORNER_MIN_ROTATE_MS        60
#define LINE_CORNER_CENTERED_MS          30
#define LINE_CORNER_MAX_ROTATE_MS       600
#define LINE_CORNER_EXIT_MS              50

#define LINE_SEARCH_FORWARD          0
#define LINE_SEARCH_TURN           230
#define LINE_SEARCH_FINE_TURN      180
#define LINE_SEARCH_30_DEG_COUNTS  160
#define LINE_SEARCH_FIRST_COUNTS   (4 * LINE_SEARCH_30_DEG_COUNTS)
#define LINE_SEARCH_SECOND_COUNTS  (8 * LINE_SEARCH_30_DEG_COUNTS)
#define LINE_SEARCH_SETTLE_MS       30
#define LINE_SEARCH_MAX_LEGS         2
#define LINE_SEARCH_STALL_MS       400
#define LINE_SEARCH_STALL_COUNTS     2
#define LINE_LOST_STOP_MS         5000
#define LINE_ALL_BLACK_STOP_MS     100
#define LINE_INVALID_GRACE_MS      100
#define LINE_START_DELAY_MS       3000
#define LINE_LOG_PERIOD_MS         100
#define CONTROL_PERIOD_MS           10

/* USB UVC camera. ESP32-S3 USB D-/D+ are fixed internally to GPIO19/20.
 * The current camera (idVendor 0x349c / idProduct 0x3307, UVC+UAC, BULK)
 * advertises only these MJPEG frame sizes:
 *   1280x720, 800x480, 640x480, 480x320, 480x854.
 * The camera is mounted in portrait orientation (wide edge vertical), so
 * request the portrait 480x854 size (frame index 5). */
#define CAMERA_FRAME_WIDTH                 480
#define CAMERA_FRAME_HEIGHT                854
/* 25 fps -> FPS2INTERVAL(25)=400000, which is 480x854's FrameInterval[0].
 * Fall back to 20 fps (500000) if frames are dropped on the BULK pipe. */
#define CAMERA_FRAME_FPS                    25
#define CAMERA_UVC_BUFFER_SIZE      (256 * 1024)
#define CAMERA_CONNECT_TIMEOUT_MS         15000
#define CAMERA_FRAME_STALE_MS              1500
/* RGB888 decode downscale denominator: 4 yields a 120x213 working image.
 * 480x854 decodes ~550 ms/frame at scale 2 (~1.7 fps); scale 4 trades some
 * line resolution for a usable frame rate. */
#define CAMERA_DECODE_SCALE                   4

/* MG90S is intentionally stationary in task 2. Fill this only if software
 * centering is added later; -1 means the servo is not driven by firmware. */
#define CAMERA_PAN_SERVO_GPIO               (-1)

/* Vision segmentation and line geometry (processed image is 120x213). */
#define VISION_SCAN_ROW_COUNT                  6
#define VISION_ROI_TOP_PERCENT                35
#define VISION_ROI_BOTTOM_PERCENT             92
#define VISION_BLACK_MARGIN                   24
#define VISION_MIN_LINE_WIDTH_PERCENT          1
#define VISION_MAX_LINE_WIDTH_PERCENT         38
#define VISION_FINISH_WIDTH_PERCENT           70
#define VISION_LINE_CONFIDENCE_MIN            450
#define VISION_REACQUIRE_CONFIDENCE_MIN       600
#define VISION_REACQUIRE_ERROR_MAX            100
#define VISION_REACQUIRE_FRAMES                 3
#define VISION_CENTERED_FRAMES                  5

/* Camera line-following output; commands remain in -1000..1000. */
#define VISION_BASE_FORWARD                  120
#define VISION_MIN_FORWARD                    70
#define VISION_ERROR_SLOWDOWN                 55
#define VISION_KP_NUM                        340
#define VISION_KP_DEN                       1000
#define VISION_KD_NUM                         55
#define VISION_KD_DEN                       1000
#define VISION_HEADING_GAIN                  120
#define VISION_TURN_LIMIT                    260
#define VISION_CORNER_HEADING_THRESHOLD      420
#define VISION_CORNER_FORWARD                 65
#define VISION_LOST_GRACE_MS                 450
#define VISION_LOST_SEARCH_MS               1200
#define VISION_LOST_TURN                     120
#define VISION_FINISH_CONFIRM_FRAMES           3

/* HC-SR04 ultrasonic ranger. ECHO is a 5 V signal: use a divider/level shifter. */
#define ULTRASONIC_TRIG_GPIO         14
#define ULTRASONIC_ECHO_GPIO         13
#define ULTRASONIC_SAMPLE_PERIOD_MS  60
#define ULTRASONIC_ECHO_TIMEOUT_US   30000

/*
 * One-shot obstacle avoidance. Positive lateral means car-left.
 * The lateral mixer uses [left, right, rear] = [-v/2, v/2, -v]. Encoder
 * targets are deliberately calibration constants: verify them at low speed
 * on the actual floor before increasing any speed.
 */
#define AVOID_TRIGGER_DISTANCE_MM      50
#define AVOID_TRIGGER_CONFIRM_SAMPLES   2
#define AVOID_SLOW_DISTANCE_MM         100
#define AVOID_SLOW_FORWARD              90
#define AVOID_CLEAR_DISTANCE_MM        120
#define AVOID_CLEAR_CONFIRM_SAMPLES      3
#define AVOID_SENSOR_STALE_MS           500

#define AVOID_BRAKE_MS                  100
#define AVOID_REVERSE_SPEED             110
#define AVOID_REVERSE_COUNTS            300
#define AVOID_LATERAL_SPEED             120
#define AVOID_ALIGN_LATERAL_SPEED        65
#define AVOID_ALIGN_CORRECTION_SPEED     50
#define AVOID_FORWARD_SPEED             140
#define AVOID_LEFT_MIN_COUNTS           240
#define AVOID_LEFT_CLEARANCE_MS          250
#define AVOID_LEFT_MAX_COUNTS          2600
#define AVOID_FORWARD_TARGET_COUNTS    1000
#define AVOID_RIGHT_EXTRA_COUNTS        600
#define AVOID_RIGHT_MAX_COUNTS         3600
#define AVOID_LINE_CENTERED_MS           30
#define AVOID_MOTION_TIMEOUT_MS        6000
#define AVOID_STALL_TIMEOUT_MS         1000
#define AVOID_STALL_MIN_COUNTS            2

/* Per-wheel encoder PI loop used only while lateral motion is requested.
 *
 * Each strafe direction gets one high feed-forward pulse until the first
 * speed sample, then it may fall below the old 260 PWM floor.  The run log
 * showed that the wheels were still several times faster than their targets
 * at PWM 260, so left strafe runs down to 100 PWM and right strafe to 90.
 */
#define DRIVE_SPEED_CONTROL_PERIOD_MS       50
#define DRIVE_TARGET_CPS_PER_COMMAND_NUM     2
#define DRIVE_TARGET_CPS_PER_COMMAND_DEN     1
#define DRIVE_LATERAL_MIN_ACTIVE_PWM       260
#define DRIVE_LEFT_STRAFE_MIN_ACTIVE_PWM   180
#define DRIVE_RIGHT_STRAFE_MIN_ACTIVE_PWM   160
#define DRIVE_APPROACH_STARTUP_PWM         180
#define DRIVE_APPROACH_MIN_ACTIVE_PWM      100
#define DRIVE_FORWARD_MIN_ACTIVE_PWM       180
#define DRIVE_LATERAL_MAX_PWM              700
#define DRIVE_SPEED_KP_NUM                   1
#define DRIVE_SPEED_KP_DEN                   2
#define DRIVE_SPEED_KI_NUM                   1
#define DRIVE_SPEED_KI_DEN                   4
#define DRIVE_SPEED_INTEGRAL_LIMIT        1200

/* ------------------------------------------------------------------ */
/* LQ_TFT18SPI V3.3 1.8" SPI TFT dashboard (ST7735S, 128x160, IPS).    */
/* Task-2 wiring leaves GPIO19/20 exclusively available for USB UVC.   */
#define LCD_CS_GPIO                  9
#define LCD_SCK_GPIO                10
#define LCD_SDI_GPIO                11  /* SPI MOSI, panel SDI pin */
#define LCD_DC_GPIO                 12
#define LCD_RST_GPIO                38
#define LCD_BLK_GPIO                (-1) /* backlight pin, -1 = hardwired on */

#define LCD_SPI_HOST                SPI2_HOST /* GPIO matrix routes any pins */
#define LCD_CLOCK_HZ                (10 * 1000 * 1000) /* jumper wiring: keep <= 40 MHz */
#define LCD_PANEL_WIDTH             128
#define LCD_PANEL_HEIGHT            160

/* Panel tuning, per-module differences (change only if the picture is wrong):
 * - shifted picture: try LCD_X_OFFSET 2..3, LCD_Y_OFFSET 1..3
 * - colors look negative: LCD_INVERT_COLORS 0
 * - red and blue swapped: LCD_SWAP_RB 1
 * - LCD_ROTATION: 0 = native portrait (dashboard layout assumes this). */
#define LCD_X_OFFSET                0
#define LCD_Y_OFFSET                0
#define LCD_ROTATION                0
#define LCD_INVERT_COLORS           1 /* IPS panel */
#define LCD_SWAP_RB                 0

/* Speed display calibration: encoder counts per output-shaft revolution.
 * Default assumes 13-line Hall encoder x 4 quadrature x 30:1 gearbox.
 * Calibrate: lift the car, rotate one wheel exactly one full turn by hand
 * and read the encoder delta in the serial log; put that number here. */
#define WHEEL_COUNTS_PER_REV        1560

/* Dashboard refresh period; speed is averaged over this window and the
 * distance value is the median of the last N valid HC-SR04 readings. */
#define LCD_MONITOR_PERIOD_MS       200
#define LCD_MONITOR_DISTANCE_MEDIAN 5

/* ------------------------------------------------------------------ */
/* Camera stream: Wi-Fi soft-AP + MJPEG HTTP viewer for a PC / phone.  */
/* Connect to the AP below, then open http://192.168.4.1/ in a browser.*/
/* The MJPEG stream is on port 81 (one viewer at a time).              */
#define STREAM_AP_SSID              "vison_car"
#define STREAM_AP_PASSWORD          "12345678"  /* "" = open AP; WPA2 needs >= 8 chars */
#define STREAM_AP_CHANNEL             6
#define STREAM_AP_MAX_CONNECTIONS     4
#define STREAM_HTTP_PORT             80
#define STREAM_MJPEG_PORT            81
#define STREAM_MAX_CLIENTS            1
#define STREAM_POLL_MS               40   /* wait between "new frame?" polls */
