#pragma once

/*
 * Pseudo-infrared build: the four-channel infrared board is disconnected, so
 * no GPIOs are read. main/pseudo_infrared.c converts the camera line result
 * into the same four-bit black mask instead.
 *
 * Channel ordering is authoritative from the reference infrared project
 * (lnf_avoid), viewed from the front of the car:
 *
 *       car left                         car right
 *       channel 4  channel 3  channel 2  channel 1
 *
 * Bit 0 = channel 1 (car right), bit 3 = channel 4 (car left).
 */
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
/* Pseudo-infrared masks update once per camera frame (~100-200 ms at scale 8),
 * so the corner-confirm window must exceed one frame interval. The original
 * 120 ms (10 ms infrared sampling) would cancel the corner before the next
 * frame's all-white arrives. */
#define LINE_CORNER_CONFIRM_WINDOW_MS  450
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
 * Recognition fps is limited by the software JPEG decode, whose cost tracks
 * the pixel count. Request 480x320 (frame index 4) instead of the portrait
 * 480x854: horizontal stays 480 px so the four-channel lateral clarity and
 * geometry are unchanged (still 60 px wide at scale 8); only the vertical
 * drops 854 -> 320 (~2.6x fewer pixels to decode). The vertical FOV is
 * shorter, so re-verify PSEUDO_IR_SAMPLE_ROW_PERCENT via the browser overlay. */
#define CAMERA_FRAME_WIDTH                 480
#define CAMERA_FRAME_HEIGHT                320
/* 25 fps -> FPS2INTERVAL(25)=400000, which is 480x320's FrameInterval[0].
 * Keep 25 fps: after the resolution cut the decode nearly matches the input,
 * so a lower fps would only cap the recognition rate. */
#define CAMERA_FRAME_FPS                    25
#define CAMERA_UVC_BUFFER_SIZE      (256 * 1024)
#define CAMERA_CONNECT_TIMEOUT_MS         15000
#define CAMERA_FRAME_STALE_MS              1500
/* RGB888 decode downscale denominator: 8 yields a 60x40 working image from
 * 480x320. Scale 8 keeps the IDCT/output minimal; the pseudo-infrared
 * four-zone mapping only needs the coarse 60 px lateral resolution, which is
 * unchanged from the old 480x854 setup. */
#define CAMERA_DECODE_SCALE                   8

/* Camera mounting correction. The current module is mounted upside down,
 * so both axes are mirrored (equivalent to a 180-degree rotation). These
 * flags are shared by the vision analyzer and the Wi-Fi viewer. */
#define CAMERA_FLIP_HORIZONTAL                1
#define CAMERA_FLIP_VERTICAL                  1

/* ------------------------------------------------------------------ */
/* Camera gimbal: two MG90S servos pan/tilt the USB camera.            */
/* GPIOs default to -1 (NULL): camera_gimbal_init() then logs a warning */
/* and stays disabled, so the camera remains fixed. Set both to real,   */
/* non-conflicting output GPIOs to enable firmware centering. MG90S is  */
/* driven at 50 Hz; ~1000-2000 us maps to 0-180 deg of shaft travel.    */
/* Every command is clamped to the per-axis MIN/MAX degrees below so the */
/* horn never pushes into a mechanical stop.                            */
#define CAMERA_PAN_SERVO_GPIO              (45)
#define CAMERA_TILT_SERVO_GPIO             (21)

#define SERVO_PWM_FREQ_HZ                    50
#define SERVO_PULSE_MIN_US                 1000  /* shaft   0 deg */
#define SERVO_PULSE_MAX_US                 2000  /* shaft 180 deg */

/* Pan (horizontal): full ~180 deg sweep, no mechanical stop reported. */
#define CAMERA_PAN_SERVO_MIN_DEG              0
#define CAMERA_PAN_SERVO_MAX_DEG            180
#define CAMERA_PAN_SERVO_CENTER_DEG          80

/* Tilt (vertical): ~120 deg usable. MIN is the downward mechanical stop
 * (the chassis blocks any further down-tilt); MAX is straight up, the
 * reported maximum. CENTER looks forward. Confirm the up/down direction
 * on the bench and tune these three values to the real hard stops. */
#define CAMERA_TILT_SERVO_MIN_DEG            30
#define CAMERA_TILT_SERVO_MAX_DEG           150
#define CAMERA_TILT_SERVO_CENTER_DEG         110

/* Pseudo-infrared: fixed pixel-block sampling of the decoded RGB image.
 *
 * Four blocks stand in for the four channels of the LQ_R4CHVB infrared board.
 * They are sampled on a single row nearest the car (PSEUDO_IR_SAMPLE_ROW_PERCENT
 * of the frame height) at four lateral centers given as a percent of the image
 * width (PSEUDO_IR_CH*_CENTER_PERCENT). The two inner channels straddle the
 * exact image centre so a dead-centre line lights BOTH of them (error 0) rather
 * than falling into a centre gap and reading all-white / WAITING_LINE.
 * Bit 0 = channel 1 = car right, bit 3 = channel 4 = car left (authoritative
 * from the reference project lnf_avoid). A block counts as "black" when at
 * least PSEUDO_IR_BLOCK_DARK_MIN pixels fall below the row's adaptive
 * threshold; a confirmed wide dark run across the sample row (finish bar) is
 * reported as all-black so the infrared state machine stops on it.
 *
 * Calibration knobs (verify on the actual floor at low speed):
 * - PSEUDO_IR_SAMPLE_ROW_PERCENT: which image row maps to the distance ahead
 *   of the car where the physical infrared board sits. Raise it to sample a
 *   row closer to the car (wider in pixels).
 * - PSEUDO_IR_BLOCK_SIZE: sampled block side in pixels. On the 60 px wide
 *   decoded image at 4 the block spans ~5 px. Keep the block <= the spacing to
 *   the neighbouring channel so channels stay separable; larger blocks make
 *   narrow lines easier to catch but blur channel separation and can make two
 *   adjacent channels light together. Verify on the floor at low speed:
 *   (1) a dead-centre line lights the two inner blocks (sensor WBBW, error 0),
 *   (2) a line offset onto one channel lights exactly that block, (3) the
 *   finish bar still confirms as all-black 0x0F.
 * - PSEUDO_IR_CH*_CENTER_PERCENT: lateral centre of each block as a percent of
 *   the image width. CH2/CH3 must straddle 50 so a centred line reads error 0;
 *   keep CH1/CH4 symmetric with them. After any change re-run the floor check
 *   above (centred -> WBBW, off-centre -> single channel).
 * - PSEUDO_IR_BLOCK_DARK_MIN: how many dark pixels a block needs to count as
 *   "on the line". Keep at 2 after shrinking the block; raise to 3 only if
 *   two adjacent blocks still light together. */
#define PSEUDO_IR_BLOCK_SIZE                   4
/* Lateral block centres as a percent of the decoded image width. CH1 = car
 * right ... CH4 = car left. The two outer channels were pulled inward (80/20
 * -> 74/27) so their blocks no longer sit at the extreme frame edges, where at
 * start-up they picked up dark pixels OUTSIDE the lane (the adjacent loop
 * straight / table background) and gave a false left/right reading. The block
 * side was shrunk 6 -> 4 in proportion to the narrower channel pitch, so the
 * block-to-gap ratio (~5 px block, ~7 px gap) stays about the same.
 * The inner pair (54/47) still straddles the exact centre (50) so a
 * dead-centre line lights BOTH inner blocks -> mask_to_error gives
 * (+1 + -1)/2 = 0 (perfectly centred) instead of dropping into a centre gap
 * and reading all-white / WAITING_LINE. On the 60 px wide image these map to
 * pixel centres 44, 32, 28, 16 (blocks 42-46, 30-34, 26-30, 14-18); keep
 * CH2/CH3 symmetric about 50 and CH1/CH4 symmetric about them. */
#define PSEUDO_IR_CH1_CENTER_PERCENT          74
#define PSEUDO_IR_CH2_CENTER_PERCENT          54
#define PSEUDO_IR_CH3_CENTER_PERCENT          47
#define PSEUDO_IR_CH4_CENTER_PERCENT          27
#define PSEUDO_IR_SAMPLE_ROW_PERCENT          70
#define PSEUDO_IR_BLACK_MARGIN                40
#define PSEUDO_IR_BLOCK_DARK_MIN               2
#define PSEUDO_IR_FINISH_WIDTH_PERCENT        70
#define PSEUDO_IR_FINISH_CONFIRM_FRAMES        3

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
#define AVOID_STALL_TIMEOUT_MS          500
#define AVOID_STALL_MIN_COUNTS            2

/* Per-wheel encoder PI loop used only while lateral motion is requested.
 *
 * Each strafe direction gets one high feed-forward pulse until the first
 * speed sample, then it may fall below the old 260 PWM floor.  The run log
 * showed that the wheels were still several times faster than their targets
 * at PWM 260. The current floor calibration is 100 for left strafe and 90
 * for right strafe.
 */
#define DRIVE_SPEED_CONTROL_PERIOD_MS       50
#define DRIVE_TARGET_CPS_PER_COMMAND_NUM     2
#define DRIVE_TARGET_CPS_PER_COMMAND_DEN     1
#define DRIVE_LATERAL_MIN_ACTIVE_PWM       260
#define DRIVE_LEFT_STRAFE_MIN_ACTIVE_PWM   100
#define DRIVE_RIGHT_STRAFE_MIN_ACTIVE_PWM    90
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

/* ------------------------------------------------------------------ */
/* Pocket-push task (after the line-following finish).                  */
/* ------------------------------------------------------------------ */

/* During the push phase the vision pipeline decodes each MJPEG frame at
 * PUSH_CAMERA_DECODE_SCALE instead of CAMERA_DECODE_SCALE, trading software
 * JPEG-decode throughput (scale 4 -> about 4x the pixels -> <= ~8 fps) for a
 * 120x80 working image on which small balls and pockets are detectable.
 * The line-following phase keeps scale 8. Values must be one of 2/4/8. */
#define PUSH_CAMERA_DECODE_SCALE      4

/* Ball-to-pocket task mapping, fixed per run.
 * color: 0 = BALL_COLOR_RED, 1 = BALL_COLOR_BLUE (ball_vision.h).
 * pocket: 0 = left  (smaller logical x), 1 = right.
 * Default: red ball into the LEFT pocket first, then blue into the RIGHT. */
#define BALL_TASK_FIRST_COLOR         0
#define BALL_TASK_FIRST_POCKET        0
#define BALL_TASK_SECOND_COLOR        1
#define BALL_TASK_SECOND_POCKET       1

/* Red-ball pixel test (decoded RGB888). PURE DOMINANCE - no absolute R or
 * luma floor: underexposure scales every channel down together, so the
 * R-G / R-B gap survives while an absolute floor kills the whole ball
 * (measured on-site: white board reads only 145-182 under the lamp). JPEG
 * chroma noise on the neutral board stays below ~20, so 25 gives the
 * small far-away red ball enough margin after JPEG/downscale. */
#define BALL_RED_MIN_RG_DOM           25
#define BALL_RED_MIN_RB_DOM           25

/* The red ball is small at the start of the push task.  Keep the global
 * shape gates strict for blue-ball detection, but use a slightly more
 * tolerant gate for small red blobs.  The compactness check still rejects
 * the elongated red LEDs / car hardware visible at the bottom of the frame. */
#define BALL_RED_MIN_AREA_PX            8
#define BALL_RED_COMPACTNESS_MIN       40
#define BALL_RED_MAX_ASPECT           220
#define BALL_RED_AXIS_RATIO_MIN_MILLE 600

/* Blue-ball pixel test (decoded RGB888) for a glossy BLUE ball on a WHITE
 * background board under a table lamp.
 *
 * Same philosophy as the red test: PURE CHROMA DOMINANCE (B-R / B-G), no
 * brightness window. Underexposure scales all three channels down together,
 * so the dominance gap survives while a luma window kills the ball.
 *
 * This is strictly more robust than the white-ball test it replaces: the
 * board and its shadows are NEUTRAL, so they can never fake a blue-dominant
 * pixel. No glare-core / dark-vs-board reasoning is needed any more.
 *
 * BALL_BLUE_MIN_B and BALL_BLUE_MIN_SAT are noise floors for the DARK
 * regions of the scene (black track, pocket interior), where JPEG chroma
 * noise is relatively large compared with the signal. A genuinely black
 * pocket reads near-neutral and fails both.
 *
 * CAVEAT: a pocket painted in saturated blue would satisfy the same pixel
 * test, and only the geometry gates (BALL_COMPACTNESS_MIN / MAX_ASPECT /
 * AXIS_RATIO_MIN_MILLE) would separate the two. The pockets on this table
 * read DARK and NEUTRAL in the camera (see POCKET_BLACK_*), so the pixel
 * test is safe here - if the paint ever reads blue on site, restore a
 * blue-chroma pocket class and give it priority over the ball. */
#define BALL_BLUE_MIN_BR_DOM            25   /* B - R dominance */
#define BALL_BLUE_MIN_BG_DOM            25   /* B - G dominance */
#define BALL_BLUE_MIN_B                 50   /* below this: dark track / shadow noise */
#define BALL_BLUE_MIN_SAT               45   /* a real blue ball is strongly saturated */
/* A SMALL ball (radius 2-4 px on the push grid) survives JPEG soft-decoding
 * with only a few grey levels of shading across it, so the required sphere
 * spread must not be too strict - flat glare patches still read spread < 4. */
#define BALL_BLUE_GRADIENT_MIN           3   /* sphere shading spread */
#define BALL_BLUE_MAX_CY_PX             66   /* reject chassis/hardware in the bottom of the 120x80 view */

/* Ball blob size/shape gating. Defaults target the 120x80 push grid; rescale
 * proportionally if PUSH_CAMERA_DECODE_SCALE changes. radius is used by the
 * controller as a DISTANCE PROXY (bigger ball = closer to the camera).
 *
 * MEASURED ON THE REAL TABLE: at push-start distances the ball is SMALL in
 * the 120x80 grid - roughly the size of one pseudo-IR block plus a bit
 * (diameter ~6 px, radius ~3 px, area ~25 px). So BALL_MIN_AREA_PX must stay
 * low enough for a radius-2..3 px disk to pass (a radius-2 disk is ~12 px);
 * anything below that is one or two pixels and cannot be tracked anyway.
 * The upper end must cover a ball AT THE BUMPER (~radius 30 px), where the
 * controller's CLOSE_RADIUS back-off logic takes over.
 * BALL_COMPACTNESS_MIN is a bounding-box fill percentage (100*area/(w*h));
 * a perfect disk fills ~79%, glare streaks and tails score far lower.
 * BALL_MAX_ASPECT bounds the bbox aspect (longer*100/shorter), which rejects
 * elongated glare bars a disk-like fill test alone cannot.
 * BALL_AXIS_RATIO_MIN_MILLE is the SECOND-MOMENT axis ratio (minor/major,
 * per mille): it rejects crescent / arc-shaped glare (track bends read as
 * half-moons of light, whose minor axis is far shorter than the major one)
 * while still accepting the slightly elliptical projection of a ball seen
 * at an angle. A disk scores ~1000, a half-moon ~400-500. */
#define BALL_MIN_AREA_PX              10   /* radius ~= 1.8 px (far ball floor) */
#define BALL_MAX_AREA_PX            3200   /* radius ~= 32 px (ball at bumper) */
#define BALL_CLOSE_RADIUS_PX          14   /* above this: too close to push */
#define BALL_COMPACTNESS_MIN          50   /* bbox fill %; disk ~= 79; glare reject */
#define BALL_MAX_ASPECT              160   /* bbox long side *100 / short side */
#define BALL_AXIS_RATIO_MIN_MILLE    750   /* minor/major of blob moments */

/* Pocket detection: the side pockets are represented as BLACK regions on
 * the white background board, at the far edge of the table. Only the far
 * band is scanned - logical y above POCKET_REGION_MAX_Y_PERCENT of the
 * frame height. This keeps most of the black track/finish bar in the lower
 * half out of the pocket test.
 *
 * A pocket pixel is simply dark + low saturation. This intentionally does
 * not depend on blue paint; the physical pocket is treated as a black area.
 * Keep the threshold conservative enough to reject ordinary grey shadows. */
#define POCKET_REGION_MAX_Y_PERCENT   55
#define POCKET_BLACK_MAX_LUMA          85   /* black-region brightness ceiling */
#define POCKET_BLACK_MAX_SAT           85   /* reject strongly colored objects */
#define POCKET_MIN_AREA_PX             25   /* valid pocket must also touch the far image edge */
#define POCKET_EDGE_MIN_AREA_PX        12   /* edge-zone fallback minimum */
#define POCKET_EDGE_MIN_TOP_PIXELS      3   /* must actually touch the far edge */
#define POCKET_EDGE_TOP_ROWS_PERCENT   12   /* top boundary strip used by fallback */
#define POCKET_EDGE_TOUCH_ROWS_PIXELS   6   /* ordinary black components must touch this edge zone */
/* Virtual pocket x (percent of logical width) used for aiming when the real
 * pocket is out of view: the car first rotates toward this bearing, then
 * continues with visual feedback. */
#define POCKET_FALLBACK_LEFT_X_PERCENT   18
#define POCKET_FALLBACK_RIGHT_X_PERCENT  82

/* Push controller speeds/timings. PWM commands are -1000..1000 unless noted.
 * Rotation counts reuse the LINE_SEARCH chassis calibration (a car rotation of
 * ~30 deg moves |dL|+|dR| by PUSH_SCAN_30_DEG_COUNTS encoder counts). */
#define PUSH_APPROACH_FORWARD           55   /* base forward command for visual tracking */
#define PUSH_START_FORWARD             150  /* slower startup straight run */
#define PUSH_START_FORWARD_MS          650  /* clearly visible but gentler initial straight run */
#define PUSH_CREEP_FORWARD              55   /* legacy */
#define PUSH_CREEP_COUNTS              900   /* legacy */
#define PUSH_APPROACH_FORWARD_NEAR      40   /* very slow near-ball approach */
#define PUSH_APPROACH_FORWARD_MID       55
#define PUSH_APPROACH_FORWARD_FAR       75
#define PUSH_APPROACH_LATERAL_SPEED      80   /* slow strafe to avoid brushing the ball */
#define PUSH_APPROACH_X_HARD_PX         10   /* outside this: strafe only, no forward */
#define PUSH_APPROACH_DEADBAND_PX       10   /* visual X deadband used by ball_push.c */
#define PUSH_APPROACH_NEAR_Y_PX         46   /* close enough to hand over to ALIGN, but still before bumper range */
#define PUSH_APPROACH_NEAR_RADIUS_PX     8
#define PUSH_APPROACH_LOST_HOLD_FRAMES   3   /* briefly stop/hold before recovery */
#define PUSH_APPROACH_LOST_MAX_FRAMES    6
#define PUSH_APPROACH_FORWARD_MIN_MS   650   /* mandatory straight phase before ALIGN */
#define PUSH_APPROACH_STRAFE_TIMEOUT_MS 2000  /* phase 0 strafe timeout to prevent deadlock */

#define PUSH_SCAN_30_DEG_COUNTS        160
#define PUSH_SCAN_FIRST_COUNTS    (4 * PUSH_SCAN_30_DEG_COUNTS)
#define PUSH_SCAN_SECOND_COUNTS   (8 * PUSH_SCAN_30_DEG_COUNTS)
#define PUSH_SCAN_MAX_LEGS              4
#define PUSH_SCAN_TURN                 230   /* rotation PWM, matches LINE_SEARCH_TURN calibration */
#define PUSH_SCAN_FINE_TURN            180   /* rotation PWM, matches LINE_SEARCH_FINE_TURN calibration */
#define PUSH_SCAN_STALL_MS             400
#define PUSH_SCAN_STALL_COUNTS          2

/* Alignment: rotate so the pocket (or its fallback bearing) is centered, and
 * strafe so the ball is centered; small concurrent gains, deadband + confirm
 * window. Gains are per logical pixel of error on the 120x80 grid. */
#define PUSH_APPROACH_HEADING_TURN      70   /* far-away heading correction only */
#define PUSH_APPROACH_HEADING_DEADBAND  10
#define PUSH_APPROACH_HEADING_MAX       80
#define PUSH_ALIGN_TURN_KP              3   /* legacy; ALIGN no longer rotates near ball */
#define PUSH_ALIGN_TURN_MAX            80
#define PUSH_ALIGN_TURN_MIN_PWM         60
#define PUSH_ALIGN_STRAFE_KP            2   /* direct PWM per px of ball.x err */
#define PUSH_ALIGN_STRAFE_MAX           45
#define PUSH_ALIGN_STRAFE_MIN_PWM       35
#define PUSH_ALIGN_DEADBAND_PX          8
#define PUSH_ALIGN_SAFE_RADIUS_PX        8   /* above this, never strafe: back off first */
#define PUSH_ALIGN_CONFIRM_MS          300

#define PUSH_BACKOFF_SPEED             130   /* reverse away from a close ball */
#define PUSH_BACKOFF_COUNTS            220
#define PUSH_BACKOFF_MAX_CYCLES          4   /* ALIGN<->BACKOFF cycles -> fault */

#define PUSH_PUSH_FORWARD              140
#define PUSH_HARD_FORWARD              300   /* final straight impact remains strong */
#define PUSH_HARD_PUSH_MS              650   /* shorter strong impact: enough to eject, less overshoot */
#define PUSH_HARD_PUSH_COUNTS          1400  /* documentation / tuning reference */
#define PUSH_EGRESS_SPEED              120   /* slower retreat to preserve the search area */
#define PUSH_PUSH_TURN_KP               6   /* in-push heading-hold gain */
#define PUSH_PUSH_TURN_MAX              90
#define PUSH_PUSH_DEADBAND_PX           3
#define PUSH_PUSH_HYSTERESIS_PX         1   /* turn-sign latch until |err| < this */
#define PUSH_PUSH_MAX_COUNTS          3000   /* over-push guard (car->far edge) */
#define PUSH_PUSH_STALL_MS             300   /* wedged ball = a MISS, not a fault */

#define PUSH_POCKET_HIT_MARGIN_PX        4   /* bbox shrink/enlarge for success */
#define PUSH_BALL_NEAR_POCKET_X_PX     12   /* boundary-entry corridor */
#define PUSH_BALL_NEAR_POCKET_Y_PX     10   /* boundary-entry corridor */
#define PUSH_POCKET_CONFIRM_FRAMES       3   /* distinct frames inside pocket */
#define PUSH_BALL_DRIFT_MAX_PX          16   /* |ball.x - pocket.x| above = missed */
#define PUSH_BALL_LOST_FRAMES            6   /* ball lost outside pocket = missed */

#define PUSH_BACKOUT_SPEED             120   /* reverse after a missed push */
#define PUSH_BACKOUT_COUNTS            260
#define PUSH_EGRESS_REVERSE_COUNTS     1000   /* clear the table edge after a success - increased for more visible retreat */

/* Post-egress motion after first ball (red) is pocketed: turn right then move forward
 * to get into a better position to find the second ball (blue). */
#define PUSH_POST_EGRESS_TURN_DEG      120    /* turn angle in degrees after egress */
#define PUSH_POST_EGRESS_TURN_SPEED   200    /* rotation PWM, matches LINE_CORNER_ROTATE_TURN */
#define PUSH_POST_EGRESS_FORWARD_COUNTS 1000 /* forward distance after egress turn (encoder counts) */
#define PUSH_POST_EGRESS_FORWARD_SPEED 80    /* forward speed after egress turn */

#define BALL_TASK_RETRY_MAX              3   /* pushes per ball before fault */
#define BALL_TASK_TIMEOUT_MS         180000   /* whole task, both balls */
#define BALL_VISION_FRESH_MAX_MS       500   /* vision age still "current" in main */
