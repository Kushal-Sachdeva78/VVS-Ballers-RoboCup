#include <Arduino.h>
#include "IRSensor.h"

// ============================================================================
//  Code2_Calibrated - copy of Code2_Refined with this board's calibration baked
//  in (2026-06-27). See IRSensor.cpp for the values:
//    * IR_baseline[] : measured no-ball idle per channel (SOLID).
//    * IR_gain[]     : best-effort per-channel equalisation (~1.0, ring is
//                      well-matched; gain is not the limiting factor here).
//    * IR_distLUT[]  : IR peak intensity -> approximate cm (interpolated). The main
//                      board tracks by angle + detected(); distance is supplementary.
// ============================================================================
//  Code #2 - Refined IR ball detection for Teensy 4.1 + the 2026 ir-golf-ball
//  Lineage: Aegis_2025 "defense_IRvector w/" (vector-sum concept), but
//  re-architected for this board and the fast game:
//    * full-ring intensity-weighted centroid (atan2 over ALL lit sensors) -
//      smoother through the 0/360 wrap and more robust than strongest +/- 2.
//    * intensity = per-sensor baseline - raw  (correct polarity, ambient reject).
//    * calibrated distance via interpolated LUT (replaces Aegis's odd
//      inverted-magnitude metric).
//    * detection hysteresis + a real bool ballDetected (no shared 500 sentinel).
//    * EWMA averaging of the smoothed vector components.
//    * non-blocking update(); reports achieved update rate.
//  Path (i) analog-as-built is implemented; path (ii) RC rework is documented
//  in IRSensor.cpp / README. The ~72 Hz on-board RC sets the tracking ceiling.
// ============================================================================

IRSensor ir;

// ---- public API (read these from a future merged main loop) ---------------
float ballAngle    = 0.0f;     // degrees, robot frame (0 = front, +cw)
float ballDistance = 999.0f;   // cm (calibrated); large when no ball
bool  ballDetected = false;

static uint32_t _lastPrint = 0;

void setup()
{
  ir.begin();
  Serial.begin(115200);                  // USB debug
#ifdef SERIAL2_LEGACY_OUTPUT
  Serial2.begin(SERIAL2_BAUD);           // ball-data UART to main board (pins 7/8)
#endif

  // Optional: with the ball removed at boot, capture a live no-ball baseline.
  // Comment in if you want auto-baseline on every power-up (ball must be absent).
  // ir.captureBaseline();
}

void loop()
{
  ir.update();                           // non-blocking ring read + solve

  ballDetected = ir.detected();
  ballAngle    = ir.angle();
  ballDistance = ir.distance();

#ifdef SERIAL2_LEGACY_OUTPUT
  // Same framing as Code #1 / Aegis "main w": "<dir>a\t\r\n<dist>b\t\r\n".
  // On no-ball we emit the 500/500 sentinel so existing main-board logic
  // (if (IR_dir == 500) ...) keeps working unchanged. NOTE: the distance here is
  // calibrated cm, NOT Aegis's sqrt()/100 metric - retune any main-board
  // distance thresholds (e.g. Aegis used IR_dis > 11). See README.
  float outDir = ballDetected ? ballAngle    : 500.0f;
  float outDis = ballDetected ? ballDistance : 500.0f;
  Serial2.print(String(outDir)); Serial2.print('a'); Serial2.println("\t");
  Serial2.print(String(outDis)); Serial2.print('b'); Serial2.println("\t");
  Serial2.flush();
#endif

#ifdef DEBUG_USB_PLOTTER
  if (millis() - _lastPrint >= 20) {     // ~50 Hz console refresh
    _lastPrint = millis();
    Serial.print("det:");   Serial.print(ballDetected);
    Serial.print("\tang:"); Serial.print(ballAngle, 1);
    Serial.print("\tdist:");Serial.print(ballDistance, 1);
    Serial.print("\tpkCh:");Serial.print(ir.peakChannel());
    Serial.print("\tpk:");  Serial.print(ir.peakIntensity());
    Serial.print("\tHz:");  Serial.print(ir.updateRateHz(), 0);
    Serial.println();
  }
#endif
}
