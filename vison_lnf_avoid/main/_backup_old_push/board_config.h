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
#define LINE_BASE_FORWARD          170
#define LINE_MIN_FORWARD           120
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
#define LINE_ALL_BLACK_STOP_MS     300
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
/* Keep the proven 1/8 line-following path untouched.  Only after END is
 * confirmed does the decoder switch to 1/4 for the much smaller ball/hole
 * features.  The UVC request and input frame rate do not change. */
#define CAMERA_BALL_DECODE_SCALE              4

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
 * threshold; all four blocks black for PSEUDO_IR_FINISH_CONFIRM_FRAMES frames
 * confirms the narrow transverse END line and latches finish_detected.
 *
 * Calibration knobs (verify on the actual floor at low speed):
 * - PSEUDO_IR_SAMPLE_ROW_PERCENT: which image row maps to the distance ahead
 *   of the car where the physical infrared board sits. Raise it to sample a
 *   row closer to the car (wider in pixels).
 * - PSEUDO_IR_BLOCK_SIZE: sampled block side in pixels. On the 60 px wide
 *   decoded image the block spans ~5 px. Keep the block <= the spacing to
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
 * right ... CH4 = car left. The two outer channels have been pulled inward in
 * two steps: 80/20 -> 74/27 kept the blocks off the extreme frame edges,
 * where at start-up they picked up dark pixels OUTSIDE the lane (the adjacent
 * loop straight / table background) and gave a false left/right reading; the
 * second step 74/27 -> 65/35 shifts each outer block toward the centre by
 * exactly one block length (5 px on the 60 px wide image), tightening the
 * whole aperture around the lane. The block side was shrunk 6 -> 4 with the
 * first step, in proportion to the narrower channel pitch.
 * The inner pair (54/47) still straddles the exact centre (50) so a
 * dead-centre line lights BOTH inner blocks -> mask_to_error gives
 * (+1 + -1)/2 = 0 (perfectly centred) instead of dropping into a centre gap
 * and reading all-white / WAITING_LINE. On the 60 px wide image these map to
 * pixel centres 39, 32, 28, 21 (blocks 37-41, 30-34, 26-30, 19-23); keep
 * CH2/CH3 symmetric about 50 and CH1/CH4 symmetric about them.
 * Side effect of the second step: the outer-to-inner pitch is now 7 px, so
 * only ~2 px of white separates CH1/CH2 and CH3/CH4 (block 5 px > gap 2 px).
 * A moderately wide line can therefore light an outer block together with its
 * inner neighbour, which arms a corner (see outer_direction() in line_follow.c)
 * at a smaller lateral offset than before. If gentle curves start arming
 * corners, raise PSEUDO_IR_BLOCK_DARK_MIN to 3 or move CH1/CH4 back out. */
#define PSEUDO_IR_CH1_CENTER_PERCENT          65
#define PSEUDO_IR_CH2_CENTER_PERCENT          54
#define PSEUDO_IR_CH3_CENTER_PERCENT          47
#define PSEUDO_IR_CH4_CENTER_PERCENT          35
#define PSEUDO_IR_SAMPLE_ROW_PERCENT          70
#define PSEUDO_IR_BLACK_MARGIN                40
#define PSEUDO_IR_BLOCK_DARK_MIN               2
#define PSEUDO_IR_FINISH_CONFIRM_FRAMES        3

/* ------------------------------------------------------------------ */
/* Ball-mode camera overlays retained as route-calibration diagnostics. */
/* Image positions are percentages and are not used to steer the car.  */

/* One-shot gimbal tilt (servo shaft degrees, clamped to
 * CAMERA_TILT_SERVO_MIN_DEG..MAX_DEG) commanded once at the LINE->BALL
 * hand-over. Defaults to the line-following centre so nothing moves until
 * you calibrate: nudge it a few degrees at a time until both balls and both
 * holes sit inside the frame at the hand-over spot (confirm the up/down
 * direction on the bench first). BALL_GIMBAL_SETTLE_MS waits for the MG90S to
 * arrive; the car is stationary at the hand-over, so it only delays the first
 * ball control tick. The gimbal is never moved again during the ball phase. */
#define BALL_GIMBAL_TILT_DEG                    85
#define BALL_GIMBAL_SETTLE_MS                  600

/* RGB888 segmentation. Red uses channel dominance. White uses a bright
 * low-chroma centre surrounded by a darker local ring. Black holes are
 * compact dark connected components; long thin black track lines are rejected. */
#define BALL_RED_R_MIN                       115
#define BALL_RED_DOMINANCE                    35
#define BALL_RED_MIN_AREA                      5
#define BALL_RED_MAX_AREA                   1800
#define BALL_WHITE_CENTER_LUMA               175
#define BALL_WHITE_MAX_CHROMA                 45
#define BALL_WHITE_RING_RADIUS                 4
#define BALL_WHITE_LOCAL_CONTRAST              9
#define BALL_WHITE_MIN_AREA                    2
#define BALL_WHITE_MAX_AREA                  500
#define BALL_HOLE_MAX_LUMA                     90
#define BALL_HOLE_MIN_AREA                    18
/* The real pockets are 15x10 cm: at the 1/4 ball decode a close pocket can
 * span several thousand pixels, so the old 2200 px ceiling rejected it during
 * the final approach. */
#define BALL_HOLE_MAX_AREA                  4800
#define BALL_HOLE_MIN_FILL_PERCENT            42
/* 15x10 cm pockets foreshorten into ~4-5:1 flat strips at the far table edge;
 * the 3x3 density pass (kills 1-2 px lines) plus the fill gate still reject
 * cables and tape edges, and once both pockets pass the upper-band scan the
 * full-frame fallback (which could pick up floor clutter) never runs. */
#define BALL_HOLE_MAX_ASPECT_NUM               5
#define BALL_HOLE_MAX_ASPECT_DEN               1
/* Hole search prefers the upper frame: pockets lie beyond the ball while the
 * chassis, its shadow and floor cables sit at the bottom. Components whose
 * centroid is at or below this percent of the image height are ignored in the
 * first pass; if fewer than two holes survive, the scan is repeated over the
 * full frame (the pocket image sinks as the car approaches). Raise toward 100
 * if the pocket disappears during the final approach. */
#define BALL_HOLE_SEARCH_MAX_Y_PERCENT        70
/* When only one pocket is in frame, it is adopted into the left/right identity
 * established earlier by nearest-neighbour continuity, provided it moved less
 * than this percent of the image width since the previous frame. Larger jumps
 * (fast SEARCH spins) keep the safe both-visible rule instead. */
#define BALL_HOLE_TRACK_MAX_MOVE_PERCENT      25

/* Shot sequence after END. Commands use the existing -1000..1000 convention;
 * positive lateral is car-left and positive turn is right. Counts are safe
 * starting values, not dimensions: calibrate on the actual surface.
 *
 * Route: settle -> advance -> acquire -> strafe align -> turn align -> push ->
 * retreat, then the same cycle for the second ball. The old fixed 90-degree
 * right turn and the two taped lanes are gone: the car reaches its shot line by
 * translating and rotating under visual feedback, and it shoots at whichever
 * pocket it lines up with (no left/right pocket identity).
 *
 * Collinearity in image terms: the camera looks along the driving direction,
 * so "car-ball-pocket on one line and the car heading along it" is exactly
 * "the ball AND the chosen pocket both sit on the image centre line". Rotation
 * shifts both image errors by the same amount (it controls their mean) while a
 * lateral translation shifts the nearer ball more than the farther pocket (it
 * controls their difference), so the two axes are driven one at a time: first
 * strafe until ball and pocket share an image column, then rotate until that
 * column is the centre line. Rotation cannot undo the column difference, so a
 * stage never fights the other one. */
#define BALL_SCRIPT_SETTLE_MS                 300
/* ADVANCE is the only open-loop stage left: END bar to a spot where the balls
 * and the pockets are in frame. Verify it with the Wi-Fi overlay before
 * trusting the servo stages that follow. */
#define BALL_SCRIPT_ADVANCE_FORWARD           100
/* Empirically calibrated car-right bias during ADVANCE. With forward=100 and
 * lateral=30 the nominal wheel mix is [left, right, rear] = [85, 115, -30]. */
#define BALL_SCRIPT_ADVANCE_LATERAL             30
#define BALL_SCRIPT_ADVANCE_COUNTS            700

/* ACQUIRE stands still until the target ball and at least one shape-valid
 * pocket appear in the same frame. Nothing is searched for blindly: a missing
 * detection here means the advance distance or the gimbal tilt is off, so the
 * stage times out into FAULT_STOP instead of driving away from the known pose. */
#define BALL_ACQUIRE_TIMEOUT_MS              5000
#define BALL_ACQUIRE_CONFIRM_FRAMES             2

/* Alignment tolerances as a percent of the decoded image width (120 px in ball
 * mode, so 8% is ~10 px). COL is the ball-vs-pocket column difference, CENTER
 * is the mean offset of the two from the image centre line; both must hold for
 * BALL_ALIGN_CONFIRM_FRAMES consecutive frames before the push starts. Widen
 * them only if the servo never confirms, never to hide a mis-set gain. */
#define BALL_ALIGN_COL_TOL_PERCENT              8
#define BALL_ALIGN_CENTER_TOL_PERCENT           6
#define BALL_ALIGN_CONFIRM_FRAMES               2
/* Proportional gains on the pixel error, clamped to MIN..MAX. MIN keeps the
 * command above the drive's static-friction floor (see
 * DRIVE_LEFT/RIGHT_STRAFE_MIN_ACTIVE_PWM), otherwise a small residual error
 * commands a magnitude the wheels ignore and the stall guard trips. The strafe
 * runs through the three-wheel speed PI, so a continuous command is safe. */
#define BALL_ALIGN_STRAFE_KP_NUM                3
#define BALL_ALIGN_STRAFE_KP_DEN                1
#define BALL_ALIGN_STRAFE_MIN                  70
#define BALL_ALIGN_STRAFE_MAX                 160
/* Rotation instead runs at raw PWM (the drive layer closes the speed loop only
 * while a lateral component is requested), and holding a turn output for one
 * ball-mode frame would sweep many degrees past the tolerance. ALIGN_TURN
 * therefore moves in encoder-delimited micro-steps: turn, stop, re-measure on
 * the next frame. SPEED is the fixed output of one step and reuses the proven
 * line-search fine-turn value, which reliably overcomes static friction; lower
 * it only if a step still overshoots. COUNTS_PER_PX scales the pixel error into
 * encoder counts (LINE_SEARCH_30_DEG_COUNTS 160 is the sum of both side wheels
 * for 30 degrees, i.e. 80 counts per wheel per 30 degrees on this chassis). */
#define BALL_ALIGN_TURN_SPEED           LINE_SEARCH_FINE_TURN
#define BALL_ALIGN_TURN_COUNTS_PER_PX_NUM       3
#define BALL_ALIGN_TURN_COUNTS_PER_PX_DEN       2
#define BALL_ALIGN_TURN_MIN_STEP                4
#define BALL_ALIGN_TURN_MAX_STEP              240
/* Motion budgets per alignment stage. Strafe counts are normalized to the rear
 * wheel; turn counts are per side wheel (LINE_SEARCH_30_DEG_COUNTS 160 is the
 * sum of both wheels for 30 degrees, so 1500 per wheel is several turns).
 *
 * The two stages are iterated, not run once: an in-place rotation does not turn
 * about the camera but about the driven axle, so a large rotation also
 * translates the camera sideways and puts a little column error back. When that
 * happens ALIGN_TURN hands the error back to ALIGN_STRAFE, at most
 * BALL_ALIGN_MAX_ROUNDS strafe rounds in total, before the shot is declared
 * unalignable. Raise the round count (or widen COL_TOL) if the log shows the
 * two stages bouncing without ever confirming.
 *
 * TIMEOUT must stay above the time the whole strafe budget takes at the minimum
 * strafe speed, otherwise the watchdog fires before the budget is spent. */
#define BALL_ALIGN_MAX_ROUNDS                   4
#define BALL_ALIGN_MAX_STRAFE_COUNTS         1500
#define BALL_ALIGN_MAX_TURN_COUNTS           1500
#define BALL_ALIGN_TIMEOUT_MS               15000
/* How long an alignment stage tolerates a dropped ball/pocket detection before
 * giving up. The car holds still during this window. */
#define BALL_ALIGN_LOST_TOLERANCE_MS         1500

/* PUSH drives straight along the aligned shot line. The ball normally drops
 * into the pocket and disappears, which ends the push early; the encoder budget
 * is the fallback for a shot where the detection never drops out. MIN_SCORE
 * counts prevent a detection flicker right after the start from being read as a
 * goal. */
#define BALL_SCRIPT_PUSH_FORWARD               85
#define BALL_PUSH_MAX_COUNTS                  900
#define BALL_PUSH_MIN_SCORE_COUNTS            150
#define BALL_PUSH_SCORE_LOST_FRAMES             2
/* Back off the pocket far enough to see the next ball, then re-acquire it. */
#define BALL_SCRIPT_RETREAT_FORWARD         (-100)
#define BALL_SCRIPT_RETREAT_COUNTS            400

/* Every moving state stops on timeout or if its slowest required wheel does
 * not advance. A zero distance is allowed and skips that calibrated stage. */
#define BALL_SCRIPT_STATE_TIMEOUT_MS         12000
#define BALL_SCRIPT_STALL_TIMEOUT_MS           600
#define BALL_SCRIPT_STALL_MIN_COUNTS             2

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
#define AVOID_SLOW_DISTANCE_MM         150
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
#define DRIVE_LATERAL_MAX_PWM              500
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
