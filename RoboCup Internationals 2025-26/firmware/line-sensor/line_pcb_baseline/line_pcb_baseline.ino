/*
  ============================================================================
  LINE PCB  -  WHITE-LINE DETECTION via 1 s GREEN-BASELINE + DROP  (Teensy 4.1)
  RoboCup Junior Lightweight soccer  |  Line sensing board
  ----------------------------------------------------------------------------
  Different detection method from line_pcb_detect.ino (which uses fixed per-sensor
  thresholds). Here there is NOTHING to hand-tune per sensor:

    1. For the first BASELINE_MS (default 1000 ms = 1 s) after power-on, each
       sensor's GREEN level is AVERAGED into a per-sensor baseline. During this
       window the Line packet is still sent every loop with flags = 0 (no line),
       so the Main link stays live and the robot behaves normally while learning.
       >>> KEEP ALL SENSORS OVER GREEN during that 1 s -- that window DEFINES
           "green". (Power-cycle, or send 'r' over USB, to relearn.)

    2. After 1 s, detection goes live: a channel reads WHITE when its raw value
       has DROPPED by at least WHITE_DROP (default 5) below ITS OWN learned green
       baseline. Per-sensor, so every channel uses its own green reference.

    3. Per QTR board, "this board sees the line" = >= LINE_MIN_SENSORS channels
       WHITE (default 2; QTR4 has only 3 channels -> still 2).

  This assumes WHITE reads LOWER than green (measured on your surface: white < ~180).

  ----------------------------------------------------------------------------
  WIRING (identical to the raw reader / line_pcb_detect):
    Emitter CTRL_ODD (shared) ............ Teensy D6  (driven HIGH = emitters ON)
    QTR1 OUT0..OUT4 ...................... A0  A1  A2  A3  A4    (RIGHT)
    QTR2 OUT0..OUT4 ...................... A5  A6  A7  A8  A9    (FRONT)
    QTR3 OUT0..OUT4 ...................... A10 A11 A12 A13 A14   (LEFT)
    QTR4 OUT1 OUT2 OUT3 (OUT0 & OUT4 NC) . A15 A16 A17          (BACK)
    UART to Main ......................... Serial7 (TX7 = pin 29 -> Main RX8/34)
    Sensors on 3.3V, Teensy on 5V (VIN), common ground.

  PACKET: 9-byte A5/5A frame, identical to line_pcb_detect.ino, so the main board
  decodes it with no change:
    [0]0xA5 [1]0x5A [2]mask [3..6]count[0..3] [7]seq [8]crc8(poly07 over [2..7]).
    mask bit0=QTR1/RIGHT bit1=QTR2/FRONT bit2=QTR3/LEFT bit3=QTR4/BACK; set iff
    that board's white count >= LINE_MIN_SENSORS.

  TIMING: deterministic 1 kHz loop (elapsedMicros), one packet/tick = 10x the
  Main's 100 Hz. 18 reads @averaging-16 ~ 250-350 us inside each 1 ms tick.

  Arduino IDE target: Tools -> Board: Teensy 4.1 ;  USB Type: Serial.
  ============================================================================
*/

// ============================ TUNE ME ========================================
const uint32_t BASELINE_MS = 1000;   // learn the per-sensor GREEN level this long at boot
const int      WHITE_DROP  = 5;      // WHITE when raw <= baseline - WHITE_DROP  ("drops by 5")
// Optional anti-flicker release: a white channel clears back to GREEN when its raw
// climbs to within RELEASE_DROP of baseline. RELEASE_DROP == WHITE_DROP => no
// hysteresis (a single hard cutoff at baseline-5, like your HYST=0 choice). Set
// RELEASE_DROP < WHITE_DROP (e.g. 3) to add a dead-band if an edge sensor chatters.
const int      RELEASE_DROP = WHITE_DROP;

// A board "sees the line" at >= this many WHITE channels (QTR4 has 3 -> 2 works).
// Set to 1 if you want ANY single white sensor on a board to flag that direction.
const int      LINE_MIN_SENSORS = 2;
// =============================================================================

// ----------------------------- Hardware ------------------------------------
const int EMITTER_PIN = 6;         // CTRL_ODD shared emitter line (HIGH = ON)
const int NUM_SENSORS = 18;
const int ADC_BITS    = 12;        // 12-bit -> 0..4095
const int ADC_MAX     = (1 << ADC_BITS) - 1;

// Analog pins in wiring order (board 1 -> board 4) -- IDENTICAL to the raw reader.
const int sensorPins[NUM_SENSORS] = {
  A0,  A1,  A2,  A3,  A4,     // QTR1 OUT0..4   (RIGHT)
  A5,  A6,  A7,  A8,  A9,     // QTR2 OUT0..4   (FRONT)
  A10, A11, A12, A13, A14,    // QTR3 OUT0..4   (LEFT)
  A15, A16, A17               // QTR4 OUT1,2,3  (BACK)   OUT0 & OUT4 = NC
};

// Per-board index blocks + labels (raw reader order).
const int     boardStart[4] = { 0, 5, 10, 15 };
const int     boardLen[4]   = { 5, 5, 5,  3  };
const char*   boardDir[4]   = { "RIGHT", "FRONT", "LEFT", "BACK" };

// ----------------------------- Link / packet -------------------------------
#define   LINE_PORT   Serial7
#define   LINE_BAUD   115200
#define   LINE_SYNC0  0xA5      // A5/5A sync, distinct from the US link's AA/55
#define   LINE_SYNC1  0x5A

// Loop / send pacing (deterministic). 1000 us = 1 kHz.
const unsigned long SEND_PERIOD_US = 1000;

// Bit positions in lineMask (board order). MUST match the Main decoder.
enum { LN_RIGHT_BIT = 0, LN_FRONT_BIT = 1, LN_LEFT_BIT = 2, LN_BACK_BIT = 3 };

// ----------------------------- USB debug -----------------------------------
#define   DEBUG_ENABLE        1
const unsigned long DEBUG_PERIOD_MS = 200;

// ----------------------------- State ---------------------------------------
int      values[NUM_SENSORS];                 // last raw ADC reads
bool     whiteState[NUM_SENSORS] = { false };  // latched per-channel WHITE flag
uint16_t baseline[NUM_SENSORS]   = { 0 };      // learned per-sensor GREEN level

// baseline-learning accumulators
uint32_t accum[NUM_SENSORS] = { 0 };
uint32_t accN     = 0;
bool     learning = true;

uint8_t       txSeq = 0;
elapsedMicros loopTimer;                       // paces the read->decide->send loop
elapsedMillis learnTimer;                      // measures the BASELINE_MS learn window
elapsedMillis debugTimer;

// ============================================================================
//  CRC-8  -  byte-IDENTICAL to the Main board's us_crc8() (poly 0x07, init 0x00).
// ============================================================================
static uint8_t crc8(const uint8_t *d, uint8_t n) {
  uint8_t c = 0x00;
  while (n--) {
    c ^= *d++;
    for (uint8_t b = 0; b < 8; b++)
      c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
  }
  return c;
}

// ---------------------------------------------------------------------------
// Per-channel WHITE decision from the learned baseline (white reads LOW -> a
// DROP below baseline = white). int math avoids unsigned underflow on a small
// baseline. RELEASE_DROP >= ... gives optional hysteresis (see TUNE ME).
bool decideWhite(int raw, bool prevWhite, int base) {
  const int onLevel  = base - WHITE_DROP;     // raw <= onLevel  -> WHITE
  const int offLevel = base - RELEASE_DROP;   // raw >= offLevel -> GREEN (offLevel >= onLevel)
  return prevWhite ? (raw <  offLevel)        // stay white until it climbs back near baseline
                   : (raw <= onLevel);        // turn white once dropped past the edge
}

// ---------------------------------------------------------------------------
void startLearning() {
  accN = 0;
  for (int i = 0; i < NUM_SENSORS; i++) { accum[i] = 0; whiteState[i] = false; }
  learning  = true;
  learnTimer = 0;
#if DEBUG_ENABLE
  Serial.print("Learning GREEN baseline for ");
  Serial.print(BASELINE_MS); Serial.println(" ms -- keep all sensors on GREEN...");
#endif
}

void finalizeBaseline() {
  if (accN == 0) accN = 1;                     // guard (shouldn't happen)
  for (int i = 0; i < NUM_SENSORS; i++) baseline[i] = (uint16_t)(accum[i] / accN);
  learning = false;
#if DEBUG_ENABLE
  Serial.print("GREEN baseline learned over "); Serial.print(accN); Serial.println(" samples:");
  for (int b = 0; b < 4; b++) {
    Serial.print("    "); Serial.print(boardDir[b]); Serial.print(":");
    for (int j = 0; j < boardLen[b]; j++) { Serial.print("  "); Serial.print(baseline[boardStart[b] + j]); }
    Serial.println();
  }
  Serial.print("RUN: WHITE when a sensor drops >= "); Serial.print(WHITE_DROP);
  Serial.println(" below its baseline. (send 'r' to relearn)");
#endif
}

// USB 'r'/'R' -> relearn the baseline (bench convenience; power-cycle does the same).
void checkRelearn() {
#if DEBUG_ENABLE
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 'r' || c == 'R') startLearning();
  }
#endif
}

// ---------------------------------------------------------------------------
void setup() {
#if DEBUG_ENABLE
  Serial.begin(115200);
  while (!Serial && millis() < 1500) { /* brief wait for the USB monitor */ }
#endif

  LINE_PORT.begin(LINE_BAUD);          // binary link to Main (Serial7, TX7 = pin 29)

  analogReadResolution(ADC_BITS);      // 12-bit, like the raw reader
  analogReadAveraging(16);             // clean reads -- the 5-count WHITE_DROP is tiny

  pinMode(EMITTER_PIN, OUTPUT);
  digitalWrite(EMITTER_PIN, HIGH);     // emitters ON

  loopTimer  = 0;
  debugTimer = 0;

#if DEBUG_ENABLE
  Serial.println("line_pcb_baseline ready (1 s green learn, then drop-by-5 = white).");
#endif
  startLearning();                     // begin the green-learn window
}

void loop() {
  checkRelearn();                      // USB 'r' restarts the learn window

  // ---- deterministic pacing: one read->decide->send per SEND_PERIOD_US ----
  if (loopTimer < SEND_PERIOD_US) return;
  loopTimer -= SEND_PERIOD_US;         // subtract (not =0) to hold the average rate

  // ---- read every channel ----
  for (int i = 0; i < NUM_SENSORS; i++) values[i] = analogRead(sensorPins[i]);

  uint8_t count[4] = { 0, 0, 0, 0 };
  uint8_t mask = 0;

  if (learning) {
    // accumulate the green baseline; report "no line" so the Main behaves normally
    for (int i = 0; i < NUM_SENSORS; i++) accum[i] += (uint32_t)values[i];
    accN++;
    if (learnTimer >= BASELINE_MS) finalizeBaseline();
    // mask + counts stay 0 during learn
  } else {
    // live detection: per-channel drop below its own learned baseline
    for (int i = 0; i < NUM_SENSORS; i++)
      whiteState[i] = decideWhite(values[i], whiteState[i], baseline[i]);
    for (int b = 0; b < 4; b++) {
      uint8_t cnt = 0;
      for (int j = 0; j < boardLen[b]; j++)
        if (whiteState[boardStart[b] + j]) cnt++;
      count[b] = cnt;
      if (cnt >= LINE_MIN_SENSORS) mask |= (uint8_t)(1u << b);   // b: 0=RIGHT 1=FRONT 2=LEFT 3=BACK
    }
  }

  // ---- build + send the 9-byte packet (A5/5A contract) ----
  uint8_t pkt[9];
  pkt[0] = LINE_SYNC0;
  pkt[1] = LINE_SYNC1;
  pkt[2] = mask;
  pkt[3] = count[0];
  pkt[4] = count[1];
  pkt[5] = count[2];
  pkt[6] = count[3];
  pkt[7] = txSeq++;
  pkt[8] = crc8(&pkt[2], 6);           // CRC over [2..7] = mask + 4 counts + seq
  LINE_PORT.write(pkt, sizeof(pkt));   // buffered, non-blocking

  // ---- throttled USB debug (separate port -> never touches Serial7) ----
#if DEBUG_ENABLE
  if (debugTimer >= DEBUG_PERIOD_MS) {
    debugTimer = 0;
    if (learning) {
      Serial.print("[LEARN "); Serial.print((uint32_t)learnTimer);
      Serial.print("/"); Serial.print(BASELINE_MS); Serial.print("ms] raw");
      for (int i = 0; i < NUM_SENSORS; i++) { Serial.print(' '); Serial.print(values[i]); }
      Serial.println();
    } else {
      Serial.print("[RUN] ");
      for (int b = 0; b < 4; b++) {
        Serial.print(boardDir[b]); Serial.print((mask >> b) & 1 ? "*" : ".");
        Serial.print("("); Serial.print(count[b]); Serial.print(") ");
      }
      Serial.print(" seq="); Serial.print(pkt[7]);
      Serial.print(" | raw/base");
      for (int i = 0; i < NUM_SENSORS; i++) {
        Serial.print(' '); Serial.print(values[i]);
        Serial.print('/');  Serial.print(baseline[i]);
      }
      Serial.println();
    }
  }
#endif
}
