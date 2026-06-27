#include <Arduino.h>

// ============================================================================
//  IR Calibration / Test sketch  (Teensy 4.1, Section 3 board)
//
//  Live, Serial-Plotter-friendly readout of all 16 IR channels plus a quick
//  full-ring centroid angle and peak channel. Use it for every calibration step
//  in the README. Open the Arduino/PlatformIO Serial Plotter at 115200.
//
//  Serial commands (type one char, no newline needed):
//    b : capture a no-ball BASELINE (remove the ball first)  -> prints values
//    p : toggle PLOTTER mode (16 raw values) <-> VERBOSE mode (raw+intensity+angle)
//    g : print a copy-paste IR_baseline[]/IR_gain[] block for config.h
//    h : help
// ============================================================================

static const uint8_t IR_PIN[16] = {
    A0,  A1,  A2,  A3,  A4,  A5,  A6,  A7,
    A8,  A9,  A10, A11, A12, A13, A14, A15
};
#define FRONT_CHANNEL   15
#define DIRECTION_SIGN  (+1)
#define SPACING_DEG     22.5f
#define ADC_BITS        10
#define ADC_MAX         ((1 << ADC_BITS) - 1)

uint16_t baseline[16];
float    gain[16];
bool     plotterMode = true;

static float chAngle(int ch) {
  float a = (ch - FRONT_CHANNEL) * DIRECTION_SIGN * SPACING_DEG;
  while (a >   180.0f) a -= 360.0f;
  while (a <= -180.0f) a += 360.0f;
  return a;
}

static void captureBaseline() {
  uint32_t acc[16] = {0};
  for (int s = 0; s < 64; s++)
    for (int i = 0; i < 16; i++) acc[i] += analogRead(IR_PIN[i]);
  for (int i = 0; i < 16; i++) baseline[i] = (uint16_t)(acc[i] / 64);
  Serial.println("# baseline captured (no-ball idle level per channel):");
  for (int i = 0; i < 16; i++) {
    Serial.print("#  ch"); Serial.print(i);
    Serial.print(" (A");   Serial.print(i); Serial.print(") = ");
    Serial.println(baseline[i]);
  }
}

// Print a block you can paste straight into Code #2 config.h / IRSensor.cpp.
static void printConfigBlock() {
  Serial.print("uint16_t IR_baseline[NUM_SENSORS] = {");
  for (int i = 0; i < 16; i++) { Serial.print(baseline[i]); if (i < 15) Serial.print(", "); }
  Serial.println("};");
  Serial.print("float IR_gain[NUM_SENSORS] = {");
  for (int i = 0; i < 16; i++) { Serial.print(gain[i], 3); if (i < 15) Serial.print(", "); }
  Serial.println("};");
}

void setup() {
  for (int i = 0; i < 16; i++) { pinMode(IR_PIN[i], INPUT); baseline[i] = ADC_MAX; gain[i] = 1.0f; }
  analogReadResolution(ADC_BITS);
  analogReadAveraging(4);
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 2000) { /* wait briefly for USB */ }
  Serial.println("# IR calibration/test sketch - Teensy 4.1");
  Serial.println("# cmds: b=baseline  p=plotter/verbose  g=print config block  h=help");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if      (c == 'b') captureBaseline();
    else if (c == 'p') { plotterMode = !plotterMode; Serial.print("# mode = ");
                         Serial.println(plotterMode ? "PLOTTER" : "VERBOSE"); }
    else if (c == 'g') printConfigBlock();
    else if (c == 'h') Serial.println("# b=baseline p=toggle-mode g=config-block h=help");
  }

  uint16_t raw[16];
  int      inten[16];
  int      peak = 0, peakCh = 0;
  float    sumSin = 0, sumCos = 0;
  long     total = 0;

  for (int i = 0; i < 16; i++) {
    raw[i] = analogRead(IR_PIN[i]);
    int v  = (int)baseline[i] - (int)raw[i];     // intensity (higher = stronger)
    if (v < 0) v = 0;
    inten[i] = v;
    if (v > peak) { peak = v; peakCh = i; }
    float a = radians(chAngle(i));
    sumSin += v * sinf(a);
    sumCos += v * cosf(a);
    total  += v;
  }
  float angle = (total > 0) ? atan2f(sumSin, sumCos) * 180.0f / PI : 0.0f;

  if (plotterMode) {
    // 16 RAW channels, tab-separated -> Arduino Serial Plotter shows 16 traces.
    for (int i = 0; i < 16; i++) { Serial.print(raw[i]); Serial.print('\t'); }
    Serial.println();
  } else {
    Serial.print("raw:");
    for (int i = 0; i < 16; i++) { Serial.print(' '); Serial.print(raw[i]); }
    Serial.print("  | inten:");
    for (int i = 0; i < 16; i++) { Serial.print(' '); Serial.print(inten[i]); }
    Serial.print("  | peakCh="); Serial.print(peakCh);
    Serial.print(" peak=");      Serial.print(peak);
    Serial.print(" angle=");     Serial.print(angle, 1);
    Serial.println();
  }

  delay(plotterMode ? 20 : 100);
}
