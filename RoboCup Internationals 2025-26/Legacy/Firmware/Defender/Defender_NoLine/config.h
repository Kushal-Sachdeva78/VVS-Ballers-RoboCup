#ifndef CONFIG_H
#define CONFIG_H
#include <Arduino.h>

// ============================================================================
//  Defender_NoLine / config.h  -  minimal keeper + gyro heading hold + wall avoid.
//
//  Behaviour each loop, in strict priority order:
//    1) WALL ESCAPE  : a SIDE ultrasonic <= SIDE_AVOID_MM -> slide away from it.
//    2) BE IN RANGE  : back ultrasonic ABOVE the band -> go BACK;
//                      BELOW the band -> go FORWARD.
//    3) ONCE IN RANGE: slide RIGHT / LEFT by the SIGN of the IR ball bearing.
//    + ALWAYS        : a BNO055 PID holds the boot ("forward") heading on top of
//                      whatever it's doing (and while holding station).
//
//  >>> BENCH-VERIFY: IR_DIR_INVERT, the US enum in the .ino (which index is the
//      BACK / LEFT / RIGHT sensor), and the BNO heading sign (K_SIGN in the .ino).
// ============================================================================

// ---- Standoff range to the OWN-goal wall (back ultrasonic), millimetres ----
#define BACK_STANDOFF_MIN_MM 300   // below this (too close to wall)  -> go FORWARD
#define BACK_STANDOFF_MAX_MM 400   // above this (too far up-field)   -> go BACK

// ---- Side-wall avoidance ----
#define SIDE_AVOID_MM 200          // right/left sensor <= this (20 cm) -> move away

// ---- Fixed drive speeds (bang-bang, 0..255 PWM units) ----
#define DEPTH_SPEED   200          // forward/back speed while getting into range
#define LATERAL_SPEED 220          // left/right speed (ball tracking AND wall escape)

// ---- IR ball bearing -> lateral mapping ----
// On THIS keeper the IR ring's 0 deg faces the OWN goal, so a ball straight
// ahead (up-field) reads near +/-180, NOT 0. So the in-range rule is:
//   |dir| >= IR_FRONT_DEG  -> ball is in FRONT (lined up)   -> HOLD (no sideways)
//   dir < 0                -> ball to the RIGHT             -> go RIGHT
//   dir > 0                -> ball to the LEFT              -> go LEFT
#define IR_FRONT_DEG  170          // |bearing| at/above this = "in front" -> hold
#define IR_DIR_INVERT 0            // set 1 only if RIGHT/LEFT come out swapped on the bench

// ---- Gyro heading hold ----
#define HEADING_DEADBAND_DEG 2.0f  // ignore heading errors smaller than this (no jitter)
// PID gains (Kp/Ki/Kd), K_SIGN and MAX_CORRECTION are the proven attacker values,
// kept in the .ino. Flip K_SIGN there if holding heading SPINS the robot up.

#endif // CONFIG_H
