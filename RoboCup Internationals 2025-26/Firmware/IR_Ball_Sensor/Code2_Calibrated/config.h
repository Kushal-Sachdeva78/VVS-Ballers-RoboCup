#ifndef CONFIG_H
#define CONFIG_H
#include <Arduino.h>

// ============================================================================
//  config.h  (Code #2 - refined)  -  ALL tuning knobs live here.
//  See README "Tuning knobs" for what each one trades off.
// ============================================================================

// ===== Board geometry (Section 3) ==========================================
// Firmware channel i == IRout<i> == analog pin A<i>. Front = channel 15 (A15).
static const uint8_t IR_PIN[16] = {
    A0,  A1,  A2,  A3,  A4,  A5,  A6,  A7,
    A8,  A9,  A10, A11, A12, A13, A14, A15
};
#define NUM_SENSORS        16
#define FRONT_CHANNEL      15
#define DIRECTION_SIGN     (+1)      // +1 clockwise-positive; flip to -1 if a
                                     // ball-to-the-right test reads negative, or
                                     // the board is mounted components-down.
#define SENSOR_SPACING_DEG 22.5f     // 360 / 16

// ===== ADC =================================================================
// Keep 10-bit so thresholds share Aegis's scale (0..1023). If you switch to 12
// bit, set ADC_BITS to 12 and scale every threshold below by x4.
#define ADC_BITS           10
#define ADC_MAX            ((1 << ADC_BITS) - 1)   // 1023
#define ADC_AVERAGING      4         // hardware oversample per analogRead (1/4/8/16/32)

// ===== Detection (intensity = baseline - raw; HIGHER intensity = stronger) ==
#define NOISE_FLOOR          12      // intensity at/below this is treated as 0 (ambient)
#define DETECT_ON_INTENSITY  60      // peak intensity to ASSERT ballDetected (hysteresis hi)
#define DETECT_OFF_INTENSITY 35      // peak intensity to RELEASE ballDetected (hysteresis lo)
// (Aegis equivalent: "no ball" when strongest raw > 950, i.e. intensity < ~73.)

// ===== Smoothing ===========================================================
#define VECTOR_EWMA_ALPHA  0.45f     // angle vector smoothing 0..1 (higher = snappier)
#define DIST_EWMA_ALPHA    0.30f     // distance smoothing 0..1
#define SPURIOUS_DAMP      0.25f     // weight kept for a lone hot channel (see update())

// ===== Output ==============================================================
// Legacy drop-in: emit "<dir>a\t\r\n<dist>b\t\r\n" on Serial2, byte-compatible
// with Aegis "main w". Comment out to use the C++ API only.
#define SERIAL2_LEGACY_OUTPUT
#define SERIAL2_BAUD       115200
#define DEBUG_USB_PLOTTER            // human-readable status stream on USB Serial

// ===== Per-sensor calibration (fill from the Calibration sketch) ============
//   IR_baseline[i] : no-ball idle reading of channel i (~ADC_MAX). Default ADC_MAX.
//   IR_gain[i]     : multiplier that equalises channel sensitivity. Default 1.0.
extern uint16_t IR_baseline[NUM_SENSORS];
extern float    IR_gain[NUM_SENSORS];

// ===== Distance calibration LUT (peak intensity -> distance, cm) ============
// MUST be recalibrated on your hardware (README step 5). Sorted by DESCENDING
// intensity (closest first); interpolated; clamped outside the range.
struct DistPoint { float intensity; float cm; };
extern const DistPoint IR_distLUT[];
extern const uint8_t   IR_distLUT_len;

#endif // CONFIG_H
