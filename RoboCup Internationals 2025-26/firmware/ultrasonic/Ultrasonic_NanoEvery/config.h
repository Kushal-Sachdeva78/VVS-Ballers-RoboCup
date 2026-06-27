#ifndef CONFIG_H
#define CONFIG_H
#include <Arduino.h>

// ============================================================================
//  config.h  -  Ultrasonic wall / obstacle ranging board.  ALL tuning knobs
//  live here.  See README.md "Tuning knobs" for what each one trades off.
//
//  Board   : Arduino Nano Every (ABX00028, ATmega4809 @ 16 MHz, 5 V logic)
//  Sensors : 4 x HC-SR04, one per side, fired as OPPOSITE PAIRS (anti-crosstalk)
//  Link    : framed + CRC binary packet to the main PCB on Serial1 (D1=TX,D0=RX)
//  Field   : 182 cm x 243 cm  ->  max useful range is bounded below
// ============================================================================

// ===== Sensor pin map (single source of truth, from Ultrasonic_PCB.kicad_pcb)
// Index order is CLOCKWISE: 0=FRONT, 1=RIGHT, 2=BACK, 3=LEFT.
//
//   Logical side | PCB sensor | board edge | TRIG | ECHO
//   -------------+------------+------------+------+------
//   FRONT (0)    | U4         | top        | D8   | D9
//   RIGHT (1)    | U3         | right      | D4   | D5
//   BACK  (2)    | U2         | bottom     | D6   | D7
//   LEFT  (3)    | U1*        | left       | D2   | D3
//   (*PCB ref "U1" is a sensor; the Arduino module's ref is "Arduino Nano".)
//
// The four sensors sit at the top/right/bottom/left edges of the board, around
// the Nano. "FRONT" here = the edge carrying sensor U4. If your robot's nose
// points at a different edge (or the board is mounted components-down, which
// swaps LEFT/RIGHT), just re-order the four entries below -- keep each TRIG/ECHO
// pair together -- so index 0 is whatever sensor faces forward. Then bench-check
// with the USB monitor (wave a hand at the front; F should drop).
#define NUM_SENSORS 4
static const uint8_t TRIG_PIN[NUM_SENSORS] = { 8, 4, 6, 2 };   // F, R, B, L
static const uint8_t ECHO_PIN[NUM_SENSORS] = { 9, 5, 7, 3 };   // F, R, B, L

// Opposite-facing sensors are pinged together so ADJACENT sensors never fire at
// the same instant -> no direct board/air cross-talk. Phase A = {FRONT,BACK},
// phase B = {RIGHT,LEFT}. (Each entry is a pair of indices into the arrays
// above; the two members MUST be on opposite sides.)
static const uint8_t PING_PHASE[2][2] = { { 0, 2 }, { 1, 3 } };

// ===== Range / field bounds ================================================
#define MIN_RANGE_MM   20      // HC-SR04 floor ~2 cm; readings below clamp to this
#define MAX_RANGE_MM   2600    // covers the 243 cm field length + margin. Also
                               // bounds the per-ping wait (timeout). Raise only if
                               // you truly need to see further than 2.6 m.

// ===== Acoustics ===========================================================
#define AMBIENT_TEMP_C 20.0f   // speed of sound c = 331.4 + 0.6*T  (m/s)
                               // distance_mm = echo_us * c/1000 / 2

// ===== Timing / rate =======================================================
#define MEASUREMENT_PERIOD_MS 20  // minimum full-cycle period -> <= 50 Hz for all
                                  // 4 sensors. A floor that bounds the re-ping rate
                                  // (reverberation control). Lower = faster but
                                  // more self-echo risk in a hard-walled arena.
#define INTER_PHASE_GAP_MS    2   // quiet settle between the two ping phases

// ===== Filtering ===========================================================
#define MEDIAN_WINDOW  3       // 1 = off; 3 (default) kills single-sample spikes
                               // at 1-sample lag; 5 = heavier. Odd, <= 5.
#define EWMA_ALPHA     1.0f    // 1.0 = no smoothing (snappiest -> best for a fast
                               // robot). Lower (e.g. 0.5) = smoother but laggier.
#define HOLD_CYCLES    2       // on a dropout, keep reporting the last good value
                               // for this many cycles before declaring out-of-range
                               // (stops a momentarily-missed echo from blinking the
                               // wall away under the main board).

// ===== UART link to main PCB (Serial1 = D0/D1, via connector J2) ===========
#define UART_BAUD      115200  // matches the IR board / main-board ecosystem
#define PACKET_SYNC0   0xAA
#define PACKET_SYNC1   0x55
// Per-sensor status, 2 bits each, packed into one byte (F=bits1:0 ... L=bits7:6):
#define ST_OK          0       // fresh, in-range measurement
#define ST_OUT_OF_RANGE 1      // no echo within MAX_RANGE (value sent = MAX_RANGE)
#define ST_HOLD        2       // dropout; value is last-good (coasting)

// ===== Reliability =========================================================
#define ENABLE_WATCHDOG 1      // 1 = reset the MCU if loop() ever stalls > ~1 s.
                               // Auto-disables if the core lacks the WDT symbols.

// ===== Debug ===============================================================
#define DEBUG_USB              // human-readable stream on USB Serial. Comment out
                               // to silence. Does NOT touch the Serial1 data link.
#define DEBUG_USB_BAUD 115200

// ---- compile-time sanity ---------------------------------------------------
#if (MEDIAN_WINDOW != 1) && (MEDIAN_WINDOW != 3) && (MEDIAN_WINDOW != 5)
#error "MEDIAN_WINDOW must be 1, 3, or 5"
#endif

#endif // CONFIG_H
