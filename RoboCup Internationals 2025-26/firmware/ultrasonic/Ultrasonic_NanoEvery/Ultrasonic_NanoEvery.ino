/*
 * Ultrasonic_NanoEvery.ino
 * RoboCup Junior Soccer - 4x HC-SR04 wall/obstacle ranging board.
 * Streams four side distances to the main PCB over a framed UART packet.
 *
 * Board : Arduino Nano Every (ATmega4809).  Pin map + every tuning knob are in
 *         config.h.  Wire protocol + a ready-to-paste main-board parser are in
 *         README.md (and in ../Ultrasonic_Receiver_MainBoard/).
 *
 * Each cycle (paced to <= 50 Hz):
 *   1. Ping FRONT+BACK together, then RIGHT+LEFT together. Pinging OPPOSITE
 *      pairs means two adjacent sensors never fire at the same instant, so there
 *      is no direct cross-talk; the two echoes of a pair are timed CONCURRENTLY
 *      in one tight polling loop (deterministic, no interrupt jitter).
 *   2. echo width -> mm with a temperature-compensated speed of sound; clamp to
 *      [MIN_RANGE, MAX_RANGE].
 *   3. Per-sensor median filter (+ optional EWMA); coast over brief dropouts.
 *   4. Emit one 13-byte, CRC-8 checked binary packet on Serial1.
 *
 * Why HC-SR04 here is "accurate + fast + correct":
 *   - accurate : temp-comp distance, ~us timing resolution, median spike rejection.
 *   - fast     : opposite-pair concurrency halves the cycle vs. one-at-a-time;
 *                no per-ping 60 ms blocking; ~50 Hz on all four.
 *   - correct  : 2-byte sync + sequence counter + CRC-8 so the main board can
 *                reject any corrupted/partial frame and detect dropped packets.
 */
#include "config.h"

// ---- derived constants -----------------------------------------------------
static const float MM_PER_US = (331.4f + 0.6f * AMBIENT_TEMP_C) / 1000.0f; // mm/us
// Worst-case wait for one ping = round-trip time at MAX_RANGE + a start margin.
static const uint32_t ECHO_TIMEOUT_US =
    (uint32_t)((float)MAX_RANGE_MM * 2.0f / MM_PER_US) + 800UL;

// ---- watchdog availability (4809 WDT; guarded so it never breaks the build) -
#if ENABLE_WATCHDOG && defined(WDT_PERIOD_1KCLK_gc)
  #define WDT_ACTIVE 1
#endif

// ---- per-sensor state ------------------------------------------------------
static uint16_t g_dist[NUM_SENSORS];      // last reported distance, mm
static uint8_t  g_status[NUM_SENSORS];    // ST_OK / ST_OUT_OF_RANGE / ST_HOLD
static uint16_t g_lastGood[NUM_SENSORS];  // last in-range value (for coasting)
static uint8_t  g_holdCnt[NUM_SENSORS];
static bool     g_haveGood[NUM_SENSORS];

static uint16_t g_hist[NUM_SENSORS][MEDIAN_WINDOW];  // median ring buffer
static uint8_t  g_histIdx[NUM_SENSORS];
static uint8_t  g_histCnt[NUM_SENSORS];

static float    g_ewma[NUM_SENSORS];
static bool     g_ewmaInit[NUM_SENSORS];

static uint8_t  g_seq = 0;

// =============================== CRC-8 / 0x07 ===============================
static uint8_t crc8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0x00;
  while (len--) {
    crc ^= *data++;
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
  }
  return crc;
}

// =============================== watchdog ===================================
static inline void wdtPet() {
#ifdef WDT_ACTIVE
  __asm__ __volatile__("wdr");
#endif
}
static void wdtBegin() {
#ifdef WDT_ACTIVE
  while (WDT.STATUS & WDT_SYNCBUSY_bm) { }
  _PROTECTED_WRITE(WDT.CTRLA, WDT_PERIOD_1KCLK_gc);   // ~1.0 s timeout
#endif
}

// =============================== ranging ====================================
// Ping a pair of OPPOSITE sensors at once and time both echoes concurrently.
// Returns echo widths in microseconds via rawA/rawB (0 = no echo / timed out).
static void pingPair(uint8_t a, uint8_t b, uint32_t &rawA, uint32_t &rawB) {
  const uint8_t ea = ECHO_PIN[a], eb = ECHO_PIN[b];

  digitalWrite(TRIG_PIN[a], LOW);
  digitalWrite(TRIG_PIN[b], LOW);
  delayMicroseconds(3);
  digitalWrite(TRIG_PIN[a], HIGH);            // 10 us trigger, both ~simultaneously
  digitalWrite(TRIG_PIN[b], HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN[a], LOW);
  digitalWrite(TRIG_PIN[b], LOW);

  bool roseA = false, roseB = false, doneA = false, doneB = false;
  uint32_t startA = 0, startB = 0;
  rawA = 0; rawB = 0;

  const uint32_t t0 = micros();
  while ((!doneA || !doneB) && (uint32_t)(micros() - t0) < ECHO_TIMEOUT_US) {
    const uint32_t now = micros();
    if (!doneA) {
      if (digitalRead(ea)) { if (!roseA) { roseA = true; startA = now; } }
      else if (roseA)      { rawA = now - startA; doneA = true; }
    }
    if (!doneB) {
      if (digitalRead(eb)) { if (!roseB) { roseB = true; startB = now; } }
      else if (roseB)      { rawB = now - startB; doneB = true; }
    }
  }
}

// echo width (us) -> clamped distance (mm); sets *status.
static uint16_t rawToMm(uint32_t rawUs, uint8_t *status) {
  if (rawUs == 0) { *status = ST_OUT_OF_RANGE; return MAX_RANGE_MM; }
  float mm = (float)rawUs * MM_PER_US * 0.5f;
  if (mm < (float)MIN_RANGE_MM) { *status = ST_OK; return MIN_RANGE_MM; }
  if (mm > (float)MAX_RANGE_MM) { *status = ST_OUT_OF_RANGE; return MAX_RANGE_MM; }
  *status = ST_OK;
  return (uint16_t)(mm + 0.5f);
}

static uint16_t medianOf(const uint16_t *buf, uint8_t n) {
  uint16_t t[MEDIAN_WINDOW];
  for (uint8_t i = 0; i < n; i++) t[i] = buf[i];
  for (uint8_t i = 1; i < n; i++) {              // insertion sort (n <= 5)
    uint16_t k = t[i]; int8_t j = i - 1;
    while (j >= 0 && t[j] > k) { t[j + 1] = t[j]; j--; }
    t[j + 1] = k;
  }
  return t[n / 2];
}

// Fold one raw echo into sensor i's filtered, coasted output (g_dist/g_status).
static void updateSensor(uint8_t i, uint32_t rawUs) {
  uint8_t st;
  uint16_t meas = rawToMm(rawUs, &st);

  if (st == ST_OK) {
    g_hist[i][g_histIdx[i]] = meas;
    g_histIdx[i] = (g_histIdx[i] + 1) % MEDIAN_WINDOW;
    if (g_histCnt[i] < MEDIAN_WINDOW) g_histCnt[i]++;
    uint16_t med = (MEDIAN_WINDOW <= 1) ? meas : medianOf(g_hist[i], g_histCnt[i]);

    uint16_t value;
    if (EWMA_ALPHA >= 0.999f) {
      value = med;
    } else {
      if (!g_ewmaInit[i]) { g_ewma[i] = med; g_ewmaInit[i] = true; }
      else g_ewma[i] = EWMA_ALPHA * med + (1.0f - EWMA_ALPHA) * g_ewma[i];
      value = (uint16_t)(g_ewma[i] + 0.5f);
    }

    g_lastGood[i] = value;
    g_haveGood[i] = true;
    g_holdCnt[i]  = 0;
    g_status[i]   = ST_OK;
    g_dist[i]     = value;
  } else {
    // no echo: coast on the last good value for a few cycles, then report far.
    if (g_haveGood[i] && g_holdCnt[i] < HOLD_CYCLES) {
      g_holdCnt[i]++;
      g_status[i] = ST_HOLD;
      g_dist[i]   = g_lastGood[i];
    } else {
      g_status[i] = ST_OUT_OF_RANGE;
      g_dist[i]   = MAX_RANGE_MM;
    }
  }
}

// =============================== UART out ===================================
// 13-byte frame: AA 55 | F.lo F.hi R.lo R.hi B.lo B.hi L.lo L.hi | status seq | crc
// (distances uint16 little-endian mm; crc8 over the 10 payload bytes [2..11]).
static void sendPacket() {
  uint8_t pkt[13];
  pkt[0] = PACKET_SYNC0;
  pkt[1] = PACKET_SYNC1;
  uint8_t status = 0;
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    pkt[2 + 2 * i] = (uint8_t)(g_dist[i] & 0xFF);
    pkt[3 + 2 * i] = (uint8_t)(g_dist[i] >> 8);
    status |= (uint8_t)((g_status[i] & 0x03) << (2 * i));
  }
  pkt[10] = status;
  pkt[11] = g_seq++;
  pkt[12] = crc8(&pkt[2], 10);
  Serial1.write(pkt, sizeof(pkt));
}

// =============================== debug ======================================
#ifdef DEBUG_USB
static void debugPrint() {
  static uint32_t lastT = 0;
  static uint16_t frames = 0, hz = 0;
  frames++;
  uint32_t now = millis();
  if (now - lastT >= 1000) { hz = frames; frames = 0; lastT = now; }

  static const char *nm[NUM_SENSORS] = { "F", "R", "B", "L" };
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    Serial.print(nm[i]); Serial.print('=');
    Serial.print(g_dist[i]);
    Serial.print(g_status[i] == ST_OK ? ' ' : (g_status[i] == ST_HOLD ? '~' : '*'));
    Serial.print(' ');
  }
  Serial.print("mm  ~"); Serial.print(hz); Serial.println(" Hz   (* out-of-range, ~ held)");
}
#endif

// =============================== Arduino ====================================
void setup() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    pinMode(TRIG_PIN[i], OUTPUT);
    digitalWrite(TRIG_PIN[i], LOW);
    pinMode(ECHO_PIN[i], INPUT);
    g_dist[i]     = MAX_RANGE_MM;
    g_status[i]   = ST_OUT_OF_RANGE;
    g_lastGood[i] = MAX_RANGE_MM;
  }

  Serial1.begin(UART_BAUD);          // -> main PCB (D1=TX, D0=RX, connector J2)
#ifdef DEBUG_USB
  Serial.begin(DEBUG_USB_BAUD);      // -> USB, debug only
#endif

  delay(250);                        // let the HC-SR04 supplies settle
  wdtBegin();
}

void loop() {
  const uint32_t t = millis();
  uint32_t raw[NUM_SENSORS] = { 0 };

  // --- phase A: FRONT + BACK ---
  uint32_t ra, rb;
  pingPair(PING_PHASE[0][0], PING_PHASE[0][1], ra, rb);
  raw[PING_PHASE[0][0]] = ra;
  raw[PING_PHASE[0][1]] = rb;
  wdtPet();
  delay(INTER_PHASE_GAP_MS);

  // --- phase B: RIGHT + LEFT ---
  pingPair(PING_PHASE[1][0], PING_PHASE[1][1], ra, rb);
  raw[PING_PHASE[1][0]] = ra;
  raw[PING_PHASE[1][1]] = rb;
  wdtPet();

  for (uint8_t i = 0; i < NUM_SENSORS; i++) updateSensor(i, raw[i]);

  sendPacket();
#ifdef DEBUG_USB
  debugPrint();
#endif

  const uint32_t elapsed = millis() - t;     // pace the loop to the period floor
  if (elapsed < MEASUREMENT_PERIOD_MS) delay(MEASUREMENT_PERIOD_MS - elapsed);
}
