#ifndef CONFIG_H
#define CONFIG_H
#include <Arduino.h>

// ============================================================================
//  Defender_Full / config.h  -  defender knobs + the Line PCB contract.
//
//  Sensor roles: the BACK ULTRASONIC is the continuous box standoff (depth PID);
//  the LINE ring (Serial8, 9-byte A5/5A mask frame) marks the box EDGES - the
//  FRONT board flags the up-field box line (up-field limit) and the RIGHT/LEFT
//  boards flag the side lines (corner limits). Ultrasonics also centre laterally
//  and home in RECOVER.
//
//  >>> BENCH-VERIFY the sign fix-points: IR_DIR_INVERT, the US enum (in the
//      .ino), and the Line board->direction mapping (LN_*_BIT + linePoll()).
// ============================================================================

// ---- feature flags (all default OFF; the sketch runs with none of them) ----
#define USE_CAMERA 0          // OpenMV goal cam on Serial1 (11-byte AA 55 frame)
#define USE_COMMS  0          // GO/STOP referee inputs on A12/A13
#define USE_OLED   0          // SSD1306 status mirror on Wire (pins 18/19)

// ---- IR ball bearing -> lateral ----
#define IR_DIR_INVERT 0       // set 1 if a ball on the RIGHT reads NEGATIVE

// ---- Lateral PID: input = ball bearing (deg, front=0 right=+ left=-), sp 0, out vx ----
#define KP_LAT 3.0f
#define KI_LAT 0.0f
#define KD_LAT 0.15f
#define LATERAL_MAX_VX 255

// ---- Depth PID: output vy. Input = back-ultrasonic standoff (the continuous datum) ----
#define KP_DEPTH 1.20f
#define KI_DEPTH 0.0f
#define KD_DEPTH 0.10f
#define DEPTH_MAX_VY 200
#define BACK_STANDOFF_MM 220  // ultrasonic fallback/cross-check standoff (US_BACK)

// ---- GUARD/RECOVER lateral centring (still ultrasonic: line only marks edges) ----
#define KP_CENTER 0.50f

// ---- Ultrasonic lateral limits (cross-check of the line edges) ----
#define SIDE_LIMIT_MM 140

// ---- Heading-hold behaviour (fixes idle drift) ----
// The BNO055 in IMUPLUS mode has no magnetometer, so its yaw DRIFTS. Holding
// heading continuously while parked makes the keeper slowly spin to chase that
// drift. So: ignore tiny errors, and relax (stop) when nothing is commanded.
#define HEADING_DEADBAND_DEG 3.0f  // don't correct heading errors smaller than this
#define IDLE_HOLD_HEADING    0     // 0 = relax heading when parked (no idle creep, like
                                   //     the attacker); re-locks the instant it moves.
                                   // 1 = keep actively holding even when stationary (only
                                   //     if your heading reference does NOT drift).

// ---- INTERCEPT (folded into TRACK) ----
#define INTERCEPT_RATE_DPS 120.0f
#define INTERCEPT_BOOST    1.6f

// ---- CLEAR ----
#define CLEAR_PUSH_VY  200
#define CLEAR_PUSH_MS  450

// ---- RECOVER ----
#define RECOVER_TOL_MM     60
#define RECOVER_TIMEOUT_MS 1200

// ============================================================================
//  LINE PCB CONTRACT  (Serial8)  -  9-byte A5/5A mask frame, byte-identical to
//  the one the attacker decodes (one shared line link across both robots):
//
//    [0]=0xA5 [1]=0x5A [2]=mask [3..6]=count[0..3] [7]=seq [8]=crc8 over [2..7]
//      mask  : bit0 RIGHT, bit1 FRONT, bit2 LEFT, bit3 BACK (board "sees line").
//      count : per-board white-channel count (diagnostics only).
//      seq   : uint8 packet counter (gaps => dropped packets).
//      crc8  : poly 0x07, init 0x00, over the 6 payload bytes [2..7] - the SAME
//              algorithm as the ultrasonic link, so us_crc8() is reused as-is.
//
//  The keeper uses the FRONT flag (up-field box line) and RIGHT/LEFT flags (side
//  lines) as hard edge limits; the back ultrasonic gives the continuous standoff.
//  The bit order lives in the .ino (LN_*_BIT) and MUST match the Line encoder.
// ============================================================================
#define LINE_BAUD           115200
#define LINE_STALE_MS       200   // no good frame this long -> distrust the line
// Safety cross-check (the line marks the FRONT/side box lines, the back US measures
// the OWN-goal wall - different datums, so we don't compare them directly). Instead:
// regardless of what the line says, never drive toward the own goal (vy < 0) when the
// back ultrasonic shows the own-goal wall closer than this. Stops a bad reading from
// reversing the keeper into its own net.
#define BACK_WALL_SAFE_MM   90

// ---- camera (optional, Serial1, default OFF) ----
#if USE_CAMERA
  #define CAM_PORT      Serial1
  #define CAM_BAUD      115200
  #define K_CAM_TRIM    0.02f
  #define CAM_TRIM_MAX  20.0f
  #define CAM_STALE_MS  300
  #define CAM_AIM_GAIN  0.30f
#endif

// ---- comms GO/STOP (optional, default OFF) ----
#if USE_COMMS
  #define COMMS_GO_PIN     A12
  #define COMMS_STOP_PIN   A13
  #define COMMS_ACTIVE_LOW 1
#endif

#endif // CONFIG_H
