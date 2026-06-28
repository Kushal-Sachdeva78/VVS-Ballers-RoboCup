#include "IRSensor.h"

// ============================================================================
//  IRSensor.cpp  (Code #2)
//
//  Path (i) - analog hardware AS-BUILT (default, implemented here):
//    Read all 16 RC-smoothed channels, convert each to an intensity
//    (intensity = baseline - raw, since LOWER raw = STRONGER IR), then take a
//    full-ring intensity-weighted centroid for angle and a calibrated LUT for
//    distance. The acquisition speed ceiling is the on-board RC low-pass:
//    tau = 10k * 0.22uF = 2.2 ms -> fc ~= 72 Hz. That ~72 Hz is the REAL limit
//    on how fast this version can track a moving ball; sampling faster just
//    oversamples the same smoothed waveform.
//
//  OPTIONAL faster ring read (dual ADC): the readRing() analogRead loop can be
//  replaced with synchronized ADC0/ADC1 conversions to read two pins at once:
//      #ifdef USE_ADC_LIB
//        #include <ADC.h>
//        ADC adc;  // adc.adc0->...; adc.adc1->...; adc.analogSynchronizedRead(pinA,pinB)
//      #endif
//  It is OFF by default so the firmware builds with zero external deps; enable
//  via platformio.ini (lib_deps = pedvide/ADC, build_flags = -D USE_ADC_LIB).
//
//  Path (ii) - MAX responsiveness (small RC rework): see README. Shrinking each
//  channel's filter (C 0.22uF -> ~1-4.7 nF, or R 10k -> ~330R-1k) preserves the
//  TSSP pulse edges so the same A0..A15 pins can be read DIGITALLY and the LOW
//  pulse width integrated over >=1 ball period (~833 us) - the most responsive
//  method for the fast game. Not assumed here (board is fabricated as-built).
// ============================================================================

// ============================================================================
//  CALIBRATION (2026-06-27, from the Calibration sketch on this board)
//  These are now compile-time CONSTANTS - begin() no longer overwrites them.
// ============================================================================

// ---- no-ball baseline -----------------------------------------------------
// Idle (ball-absent) ADC level per channel. Recovered from the calibration
// capture (raw + intensity); uniform ~1006-1007, just under the 1023 rail.
// SOLID - this part of the calibration is trustworthy.
uint16_t IR_baseline[NUM_SENSORS] = {
  1006, 1006, 1007, 1007, 1006, 1006, 1006, 1006,
  1006, 1007, 1007, 1007, 1006, 1006, 1007, 1007
};

// ---- per-channel gain (best-effort) ---------------------------------------
// gain[i] = (mean intensity over all channels) / (channel i's mean intensity,
// averaged over all 16 ball directions @ 20 cm). Centred on 1.0, so chronically
// hot channels are nudged DOWN and cold ones UP to de-bias which sensor "peaks".
// NOTE: the ring was measured as well-matched (per-channel sensitivity spread
// was only ~6%), so every gain lands within +/-4% of 1.0 - gain is NOT what
// limits this board. The peak-tracking problem is CONTRAST/saturation (see the
// distance-LUT warning below), which no multiplicative gain can fix.
float IR_gain[NUM_SENSORS] = {
  1.020f, 0.985f, 0.998f, 0.981f, 0.985f, 0.994f, 1.008f, 1.000f,
  0.990f, 1.017f, 0.976f, 0.993f, 0.988f, 1.019f, 1.012f, 1.036f
};

// ============================================================================
//  !!! DISTANCE LUT NOT CALIBRATED - DO NOT TRUST ballDistance() ON THIS RIG !!!
//  The calibration sweep showed the ring SATURATES: peak intensity stayed flat
//  at ~700 from 1 cm all the way to 70 cm (no fall-off), so there is no usable
//  intensity->distance mapping to fit. With the placeholder table below, a ~700
//  peak interpolates to a constant ~8 cm regardless of true range.
//  Likely cause: the ball was NOT confirmed to be in pulsed MODE-A (a constant
//  peak vs distance is exactly what a continuous emitter looks like) - masks are
//  fitted, so collimation is not the bottleneck. Fix before relying on distance:
//    1. confirm the ball is in the stepped/pulsed MODE-A envelope (README sec 7);
//    2. if still flat, do the path-(ii) pulse-width rework (README sec 8) - the
//       only method that yields a real distance cue from these TSSP parts;
//    3. re-run the Calibration sketch, log peak vs 5..50 cm, replace this table.
//  Until then, gate the main board on ANGLE + detected() only, not distance.
//  (Placeholder values kept so the firmware still builds and runs.)
// ============================================================================
const DistPoint IR_distLUT[] = {
  { 900.0f,  5.0f },
  { 600.0f, 10.0f },
  { 380.0f, 15.0f },
  { 240.0f, 20.0f },
  { 150.0f, 25.0f },
  {  95.0f, 30.0f },
  {  60.0f, 40.0f },
  {  35.0f, 50.0f },
};
const uint8_t IR_distLUT_len = sizeof(IR_distLUT) / sizeof(IR_distLUT[0]);

// ---------------------------------------------------------------------------
void IRSensor::begin()
{
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    pinMode(IR_PIN[i], INPUT);   // analog input, NO pull-up (would fight the RC)
    // NOTE: IR_baseline[]/IR_gain[] are now CALIBRATED constants initialised at
    // file scope above - deliberately NOT reset here (that would wipe them).
  }
  analogReadResolution(ADC_BITS);
  analogReadAveraging(ADC_AVERAGING);   // Teensy core hardware oversampling
  _lastMicros = micros();
}

// ---- read 16 channels -> raw[] and intensity[] -----------------------------
void IRSensor::readRing()
{
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    _raw[i] = (uint16_t)analogRead(IR_PIN[i]);
    int v = (int)IR_baseline[i] - (int)_raw[i];   // inverted polarity -> intensity
    v = (int)(v * IR_gain[i]);                     // per-sensor normalisation
    if (v <= NOISE_FLOOR) v = 0;                   // ambient / noise-floor rejection
    _intensity[i] = (v < 0) ? 0 : (uint16_t)v;
  }
}

// ---- per-channel board-frame angle (deg), wrapped to (-180, 180] -----------
float IRSensor::angleOfChannel(uint8_t ch) const
{
  float a = (float)((int)ch - FRONT_CHANNEL) * DIRECTION_SIGN * SENSOR_SPACING_DEG;
  while (a >   180.0f) a -= 360.0f;
  while (a <= -180.0f) a += 360.0f;
  return a;
}

// ---- peak intensity -> distance (cm) via interpolated LUT ------------------
float IRSensor::distanceFromIntensity(uint16_t peak) const
{
  if (peak >= IR_distLUT[0].intensity)               return IR_distLUT[0].cm;
  if (peak <= IR_distLUT[IR_distLUT_len - 1].intensity) return IR_distLUT[IR_distLUT_len - 1].cm;
  for (uint8_t i = 0; i < IR_distLUT_len - 1; i++) {
    float hi = IR_distLUT[i].intensity;
    float lo = IR_distLUT[i + 1].intensity;
    if (peak <= hi && peak >= lo) {
      float t = (hi - peak) / (hi - lo);             // 0..1 from hi->lo
      return IR_distLUT[i].cm + t * (IR_distLUT[i + 1].cm - IR_distLUT[i].cm);
    }
  }
  return IR_distLUT[IR_distLUT_len - 1].cm;
}

// ---------------------------------------------------------------------------
void IRSensor::update()
{
  readRing();

  // ---- single-spurious-channel guard --------------------------------------
  // The ball always lights >=2 adjacent sensors. A lone hot channel whose BOTH
  // neighbours are dark is almost certainly a reflection/noise spike, so damp it.
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    uint8_t l = (uint8_t)((i + NUM_SENSORS - 1) % NUM_SENSORS);
    uint8_t r = (uint8_t)((i + 1) % NUM_SENSORS);
    if (_intensity[i] > 0 && _intensity[l] == 0 && _intensity[r] == 0)
      _intensity[i] = (uint16_t)(_intensity[i] * SPURIOUS_DAMP);
  }

  // ---- peak + intensity-weighted vector sum over ALL lit sensors ----------
  _peak = 0; _peakCh = 0;
  float sumSin = 0.0f, sumCos = 0.0f;
  uint32_t total = 0;
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    uint16_t w = _intensity[i];
    if (w == 0) continue;
    if (w > _peak) { _peak = w; _peakCh = i; }
    float a = radians(angleOfChannel(i));
    sumSin += (float)w * sinf(a);
    sumCos += (float)w * cosf(a);
    total  += w;
  }

  // ---- detection with hysteresis ------------------------------------------
  // Uses a real boolean instead of a shared "500" angle sentinel.
  if (!_detected && _peak >= DETECT_ON_INTENSITY)       _detected = true;
  else if (_detected && _peak < DETECT_OFF_INTENSITY)   _detected = false;

  if (_detected && total > 0) {
    // EWMA-smooth the vector COMPONENTS (naturally continuous across 0/360 wrap).
    if (!_haveVec) { _sSin = sumSin; _sCos = sumCos; _haveVec = true; }
    else {
      _sSin += VECTOR_EWMA_ALPHA * (sumSin - _sSin);
      _sCos += VECTOR_EWMA_ALPHA * (sumCos - _sCos);
    }
    _angle = atan2f(_sSin, _sCos) * 180.0f / PI;

    float d = distanceFromIntensity(_peak);
    if (_distance > 900.0f) _distance = d;            // first lock - snap, don't drift
    else _distance += DIST_EWMA_ALPHA * (d - _distance);
  } else {
    _haveVec  = false;
    _distance = 999.0f;
    // keep last _angle; consumers should gate on detected().
  }

  // ---- achieved update-rate measurement -----------------------------------
  uint32_t now = micros();
  uint32_t dt  = now - _lastMicros;
  _lastMicros  = now;
  if (dt > 0) {
    float inst = 1e6f / (float)dt;
    _rateHz += 0.1f * (inst - _rateHz);               // light smoothing of readout
  }
}

// ---- capture a fresh no-ball baseline into IR_baseline[] -------------------
void IRSensor::captureBaseline(uint16_t samples)
{
  uint32_t acc[NUM_SENSORS] = {0};
  for (uint16_t s = 0; s < samples; s++)
    for (uint8_t i = 0; i < NUM_SENSORS; i++)
      acc[i] += analogRead(IR_PIN[i]);
  for (uint8_t i = 0; i < NUM_SENSORS; i++)
    IR_baseline[i] = (uint16_t)(acc[i] / samples);
}
