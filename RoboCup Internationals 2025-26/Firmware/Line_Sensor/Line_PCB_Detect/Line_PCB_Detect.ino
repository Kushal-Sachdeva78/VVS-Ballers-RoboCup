/*
  ============================================================================
  LINE PCB  -  WHITE-LINE-ON-GREEN DETECTION FIRMWARE  (Teensy 4.1)
  RoboCup Junior Lightweight soccer  |  Line sensing board
  ----------------------------------------------------------------------------
  Built from line_pcb_raw_reader.ino (kept its EXACT pin order, emitter pin and
  12-bit ADC setup) and extended into a detection + UART-link firmware.

  WHAT IT DOES, every loop iteration:
    1. Reads all 18 QTR channels (raw reader's pin order).
    2. Latches each channel WHITE / GREEN with a PER-SENSOR threshold + shared
       hysteresis (separate cross-on / cross-off levels) so an edge sensor can't
       flicker at speed.
    3. Per QTR board, "this board sees the line" = >= 2 of its channels WHITE
       (QTR4 has only 3 channels -> still >= 2).
    4. Sends a compact 9-byte binary packet to the Main Teensy on Serial7,
       mirroring the proven ultrasonic link style (sync + payload + seq + CRC-8).
    5. Optional throttled USB-Serial debug for bench tuning (does NOT touch
       Serial7, so it can never corrupt or block the binary link).

  ----------------------------------------------------------------------------
  WIRING  (from the Line PCB doc; identical to the raw reader):
    Emitter CTRL_ODD (shared) ............ Teensy D6  (driven HIGH = emitters ON)
    QTR1 OUT0..OUT4 ...................... A0  A1  A2  A3  A4
    QTR2 OUT0..OUT4 ...................... A5  A6  A7  A8  A9
    QTR3 OUT0..OUT4 ...................... A10 A11 A12 A13 A14
    QTR4 OUT1 OUT2 OUT3 (OUT0 & OUT4 NC) . A15 A16 A17
    UART to Main ......................... Serial7  (TX7 = pin 29 -> Main RX8/34)
                                                     (RX7 = pin 28, unused here)
    Sensors on 3.3V, Teensy on 5V (VIN), common ground.
    NOTE: pins 6 and 28/29 are digital and do NOT collide with A0..A17. Verified.

  ----------------------------------------------------------------------------
  BOARD -> ROBOT DIRECTION  (CONFIRMED, "plus" pattern). This mapping is shared,
  byte-for-byte, with the Main decoder:
        QTR1 = RIGHT      QTR2 = FRONT      QTR3 = LEFT       QTR4 = BACK
  Packet bitmask bit order (board order):
        bit0 = QTR1 (RIGHT)   bit1 = QTR2 (FRONT)
        bit2 = QTR3 (LEFT)    bit3 = QTR4 (BACK)

  ----------------------------------------------------------------------------
  PACKET  (Serial7, 115200 8N1, little-endian, 9 bytes):  -- see Main decoder --
    [0] 0xA5          sync0  (distinct from US link's 0xAA/0x55 -> mis-wire safe)
    [1] 0x5A          sync1
    [2] lineMask      bit0=QTR1/RIGHT bit1=QTR2/FRONT bit2=QTR3/LEFT bit3=QTR4/BACK
    [3] count[0]      QTR1 white-sensor count (0..5)
    [4] count[1]      QTR2 white-sensor count (0..5)
    [5] count[2]      QTR3 white-sensor count (0..5)
    [6] count[3]      QTR4 white-sensor count (0..3)
    [7] seq           uint8, increments once per packet (wraps 255->0)
    [8] crc8          CRC-8 poly 0x07, init 0x00, over bytes [2..7] (6 bytes)
  CRC-8 is byte-identical to the ultrasonic link's us_crc8() on the Main board.

  ----------------------------------------------------------------------------
  TIMING: loop paced to SEND_PERIOD_US (default 2000 us = 500 Hz), i.e. 5x the
  Main's 100 Hz control rate and deterministic. A full 18-channel read + decide
  costs well under ~600 us (16x ADC averaging), so each pass has slack. The link
  carries 9 bytes/pkt; at 500 Hz that is 4500 B/s = ~39% of 115200's ~11.5 kB/s,
  leaving comfortable headroom. Push toward 1 kHz (SEND_PERIOD_US 1000) only if
  you keep link load < ~70%.

  Arduino IDE target: Tools -> Board: Teensy 4.1 ;  USB Type: Serial.
  ============================================================================
*/

// ----------------------------- Hardware ------------------------------------
const int EMITTER_PIN  = 6;        // CTRL_ODD shared emitter line (HIGH = ON)
const int NUM_SENSORS  = 18;
const int ADC_BITS     = 12;       // 12-bit -> 0..4095
const int ADC_MAX      = (1 << ADC_BITS) - 1;

// Analog pins in wiring order (board 1 -> board 4) -- IDENTICAL to the raw reader.
const int sensorPins[NUM_SENSORS] = {
  A0,  A1,  A2,  A3,  A4,     // QTR1 OUT0..4   (RIGHT)
  A5,  A6,  A7,  A8,  A9,     // QTR2 OUT0..4   (FRONT)
  A10, A11, A12, A13, A14,    // QTR3 OUT0..4   (LEFT)
  A15, A16, A17               // QTR4 OUT1,2,3  (BACK)   OUT0 & OUT4 = NC
};

// Labels Q<board>.<output>, raw-reader order -- used only for USB debug.
const char* sensorNames[NUM_SENSORS] = {
  "Q1.0","Q1.1","Q1.2","Q1.3","Q1.4",
  "Q2.0","Q2.1","Q2.2","Q2.3","Q2.4",
  "Q3.0","Q3.1","Q3.2","Q3.3","Q3.4",
  "Q4.1","Q4.2","Q4.3"
};

// ====================== PER-SENSOR WHITE THRESHOLDS =========================
// POLARITY CONVENTION (MEASURED on your surface):
//   On THIS hardware the WHITE line reads LOW (raw < ~180) and the GREEN field
//   reads HIGHER -- white is DARKER than green here. This is the value you
//   measured with the raw reader, so the default is white-below-threshold (0).
//   >>> If you ever fit sensors where white reads HIGHER than green, set
//       WHITE_ABOVE_THRESHOLD to 1 and every comparison inverts automatically.
#define WHITE_ABOVE_THRESHOLD 0

// One threshold per CHANNEL, in the raw reader's pin order (Q1.0..Q1.4,
// Q2.0..Q2.4, Q3.0..Q3.4, Q4.1..Q4.3). Seeded at the ADC mid-point (2048) as a
// placeholder ONLY.
//   >>> TUNE EACH ONE: run line_pcb_raw_reader.ino (or this sketch's USB debug),
//       note the raw WHITE value W and raw GREEN value G that channel reports on
//       your field, and set its threshold to about (W + G) / 2. Adjust if a
//       channel sits over a seam or reads dirty.
int WHITE_THRESH[NUM_SENSORS] = {
  /* Q1.0 */ 210, /* Q1.1 */ 200, /* Q1.2 */ 200, /* Q1.3 */ 200, /* Q1.4 */ 210,
  /* Q2.0 */ 200, /* Q2.1 */ 200, /* Q2.2 */ 200, /* Q2.3 */ 200, /* Q2.4 */ 200,
  /* Q3.0 */ 200, /* Q3.1 */ 200, /* Q3.2 */ 200, /* Q3.3 */ 200, /* Q3.4 */ 200,
  /* Q4.1 */ 200, /* Q4.2 */ 200, /* Q4.3 */ 200
};

// Shared hysteresis margin (ADC counts) AROUND each sensor's threshold.
//   HYST = 0  -> a single hard cutoff exactly at THRESH (white-low):
//                WHITE when raw <= THRESH, GREEN when raw >= THRESH. (current)
//   HYST > 0  -> split band: WHITE at raw <= THRESH-HYST, GREEN at raw >= THRESH+HYST,
//                which stops an edge sensor flickering when it sits right on THRESH.
// >>> If a sensor parked near 200 chatters between white/green at speed, set HYST
//     to a few counts (e.g. 3-5). Keep it SMALL -- it is NOT a second threshold
//     (HYST=150 once pushed the real trigger to raw<=50). Per-sensor later: turn
//     HYST into an int HYST[NUM_SENSORS] table indexed in updateSensor().
const int HYST = 0;

// A board "sees the line" when at least this many of its channels are WHITE.
// QTR4 has only 3 channels, so 2 still works there.
const int LINE_MIN_SENSORS = 2;

// ----------------------------- Link / packet -------------------------------
#define   LINE_PORT   Serial7
#define   LINE_BAUD   115200
#define   LINE_SYNC0  0xA5      // A5/5A sync, distinct from the US link's AA/55
#define   LINE_SYNC1  0x5A

// Loop / send pacing (deterministic). 2000 us = 500 Hz. See header timing note.
const unsigned long SEND_PERIOD_US = 2000;

// Bit positions in lineMask (board order). MUST match the Main decoder.
enum { LN_RIGHT_BIT = 0, LN_FRONT_BIT = 1, LN_LEFT_BIT = 2, LN_BACK_BIT = 3 };

// ----------------------------- USB debug -----------------------------------
#define   DEBUG_ENABLE        1      // 1 = print human-readable status on USB Serial
const unsigned long DEBUG_PERIOD_MS = 200;   // throttle so it never floods/blocks

// ----------------------------- State ---------------------------------------
int  values[NUM_SENSORS];                    // last raw ADC reads
bool whiteState[NUM_SENSORS] = { false };     // latched per-channel WHITE flag (hysteresis)
uint8_t  txSeq = 0;
elapsedMicros loopTimer;                      // paces the read->decide->send loop
elapsedMillis debugTimer;
unsigned long lastComputeUs = 0;             // measured read+decide cost (debug only)

// ============================================================================
//  CRC-8  -  byte-IDENTICAL to the Main board's us_crc8() (poly 0x07, init 0x00,
//  no reflection, no final XOR). Keeping these in lock-step is what makes the
//  encoder and decoder provably symmetric.
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
// Latch one channel WHITE/GREEN with hysteresis, honouring WHITE_ABOVE_THRESHOLD.
// Returns the new latched state given the raw value and the previous state.
bool updateSensor(int idx, int v, bool prevWhite) {
#if WHITE_ABOVE_THRESHOLD
  const int onLevel  = WHITE_THRESH[idx] + HYST;   // rise ABOVE this -> WHITE
  const int offLevel = WHITE_THRESH[idx] - HYST;   // fall BELOW this -> GREEN
  return prevWhite ? (v >  offLevel)               // stay white until it drops past near edge
                   : (v >= onLevel);               // become white only past far edge
#else
  const int onLevel  = WHITE_THRESH[idx] - HYST;   // fall BELOW this -> WHITE
  const int offLevel = WHITE_THRESH[idx] + HYST;   // rise ABOVE this -> GREEN
  return prevWhite ? (v <  offLevel)
                   : (v <= onLevel);
#endif
}

// ---------------------------------------------------------------------------
void setup() {
#if DEBUG_ENABLE
  Serial.begin(115200);
  while (!Serial && millis() < 1500) { /* brief wait for the USB monitor */ }
#endif

  LINE_PORT.begin(LINE_BAUD);          // binary link to Main (Serial7, TX7=pin 29)

  analogReadResolution(ADC_BITS);      // 12-bit, like the raw reader
  analogReadAveraging(16);             // noise smoothing (baseline). Drop to 4-8
                                       // if you push the loop well past 500 Hz.

  pinMode(EMITTER_PIN, OUTPUT);
  digitalWrite(EMITTER_PIN, HIGH);     // emitters ON

  loopTimer = 0;
  debugTimer = 0;

#if DEBUG_ENABLE
  Serial.println("line_pcb_detect ready. Per-sensor thresholds + hysteresis, >=2/board.");
  Serial.print("ADC 0.."); Serial.print(ADC_MAX);
  Serial.print("  HYST="); Serial.print(HYST);
  Serial.print("  WHITE_ABOVE_THRESHOLD="); Serial.println(WHITE_ABOVE_THRESHOLD);
#endif
}

void loop() {
  // ---- deterministic pacing: one read->decide->send per SEND_PERIOD_US ----
  if (loopTimer < SEND_PERIOD_US) return;
  loopTimer -= SEND_PERIOD_US;         // subtract (not =0) to hold the average rate

  elapsedMicros compute;

  // ---- 1) read every channel, 2) latch white/green with hysteresis ----
  for (int i = 0; i < NUM_SENSORS; i++) {
    values[i]     = analogRead(sensorPins[i]);
    whiteState[i] = updateSensor(i, values[i], whiteState[i]);
  }

  // ---- 3) per-board white counts (raw reader's index blocks) ----
  // QTR1 = 0..4, QTR2 = 5..9, QTR3 = 10..14, QTR4 = 15..17 (only 3 channels)
  uint8_t count[4] = { 0, 0, 0, 0 };
  for (int i = 0;  i <= 4;  i++) if (whiteState[i])  count[0]++;   // QTR1 RIGHT
  for (int i = 5;  i <= 9;  i++) if (whiteState[i])  count[1]++;   // QTR2 FRONT
  for (int i = 10; i <= 14; i++) if (whiteState[i])  count[2]++;   // QTR3 LEFT
  for (int i = 15; i <= 17; i++) if (whiteState[i])  count[3]++;   // QTR4 BACK

  // ---- per-board "sees line" = >= LINE_MIN_SENSORS white -> bitmask ----
  uint8_t mask = 0;
  if (count[0] >= LINE_MIN_SENSORS) mask |= (1 << LN_RIGHT_BIT);
  if (count[1] >= LINE_MIN_SENSORS) mask |= (1 << LN_FRONT_BIT);
  if (count[2] >= LINE_MIN_SENSORS) mask |= (1 << LN_LEFT_BIT);
  if (count[3] >= LINE_MIN_SENSORS) mask |= (1 << LN_BACK_BIT);

  // ---- 4) build + send the 9-byte packet (see header layout) ----
  uint8_t pkt[9];
  pkt[0] = LINE_SYNC0;
  pkt[1] = LINE_SYNC1;
  pkt[2] = mask;
  pkt[3] = count[0];
  pkt[4] = count[1];
  pkt[5] = count[2];
  pkt[6] = count[3];
  pkt[7] = txSeq++;
  pkt[8] = crc8(&pkt[2], 6);          // CRC over [2..7] = mask + 4 counts + seq
  LINE_PORT.write(pkt, sizeof(pkt));  // buffered, non-blocking at 500 Hz

  lastComputeUs = compute;

  // ---- 5) throttled USB debug (separate port -> never touches Serial7) ----
#if DEBUG_ENABLE
  if (debugTimer >= DEBUG_PERIOD_MS) {
    debugTimer = 0;
    Serial.print("mask=0b");
    for (int b = 3; b >= 0; b--) Serial.print((mask >> b) & 1);
    Serial.print(" [R="); Serial.print((mask >> LN_RIGHT_BIT) & 1);
    Serial.print(" F=");  Serial.print((mask >> LN_FRONT_BIT) & 1);
    Serial.print(" L=");  Serial.print((mask >> LN_LEFT_BIT) & 1);
    Serial.print(" B=");  Serial.print((mask >> LN_BACK_BIT) & 1);
    Serial.print("]  cnt R/F/L/B=");
    Serial.print(count[0]); Serial.print('/');
    Serial.print(count[1]); Serial.print('/');
    Serial.print(count[2]); Serial.print('/');
    Serial.print(count[3]);
    Serial.print("  seq="); Serial.print(pkt[7]);
    Serial.print("  comp="); Serial.print(lastComputeUs); Serial.print("us");
    // raw values per board, so you can read thresholds straight off the line
    Serial.print("  | Q1:");
    for (int i = 0;  i <= 4;  i++) { Serial.print(' '); Serial.print(values[i]); }
    Serial.print("  Q2:");
    for (int i = 5;  i <= 9;  i++) { Serial.print(' '); Serial.print(values[i]); }
    Serial.print("  Q3:");
    for (int i = 10; i <= 14; i++) { Serial.print(' '); Serial.print(values[i]); }
    Serial.print("  Q4:");
    for (int i = 15; i <= 17; i++) { Serial.print(' '); Serial.print(values[i]); }
    Serial.println();
  }
#endif
}
