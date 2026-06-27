/*
  ============================================================================
  IR BALL CHASE + ULTRASONIC OBSTACLE AVOIDANCE  -  Main Teensy 4.1
  ----------------------------------------------------------------------------
  Drives a 4-wheel omni (diamond) robot toward an IR-emitting ball, but lets
  the ultrasonic ring OVERRIDE the chase to flee anything that gets too close.

  PRIORITY (highest first):
    1. OBSTACLE AVOIDANCE - if any ultrasonic side reads < US_AVOID_MM, drive
       AWAY from the closest one (right too close -> go left, front -> go back,
       etc.). This overrides the ball chase completely.
    2. BALL CHASE - otherwise translate toward the ball by its sector.
    3. IDLE - no ball, no obstacle -> stop.

  INPUTS
  ------
  - Ball DIRECTION arrives over UART from the IR-PCB Teensy on Serial4.
      Frame emitted by the IR sketch:  "<dir>a\t\r\n<dis>b\t\r\n"
      The board's NATIVE bearing is front = 0 deg, but THIS sketch uses it REVERSED:
      a ball AHEAD reads near +/-180 (wraps), BEHIND near 0, right = negative,
      left = positive. classify() below encodes exactly that (forward = |dir|>=150).
      IR no-ball sentinel: dir = 500.   (The distance field is parsed for frame
      sync only and then DISCARDED - this sketch no longer uses ball distance.)

  - Ultrasonic distances arrive over UART from the 4x HC-SR04 board (Nano Every)
    as the 13-byte CRC-8 framed packet. The parser below (struct + ultrasonicPoll
    + ultrasonicStale + crc8) is copied UNCHANGED from
    Firmware Ultrasonic/Ultrasonic_Receiver_MainBoard - only US_PORT is set to
    match this board's wiring. Distances are in MILLIMETRES.

  Wiring (from the Main PCB doc):
    SLEEP -> 6 ;  M1: 2,3 ;  M2: 4,5 ;  M3: 9,10 ;  M4: 11,12
    BNO055 on Wire2: SDA=25, SCL=24, VIN=3.3V
    IR UART       : Serial4  (RX4 = pin 16  <- IR board   TX_IR)
    Ultrasonic PCB: Serial3  (RX3 = pin 15  <- US board   TX_U,  TX3 = pin 14)
    BT telemetry  : Serial2  (TX2 = pin 8   -> HC-05 RXD,  RX2 = pin 7 <- HC-05 TXD)
                              (HC-05 VCC = 5V from VIN; data pins are 3.3V logic)

  OUTPUTS
  -------
  - Everything the robot RECEIVES (IR ball angle, BNO055 Euler angles, and the
    four ultrasonic distances) is streamed back out as one CSV line per debug
    tick over an HC-05 Bluetooth-Classic (SPP) module on Serial2 (pins 7/8), so a
    paired phone or laptop serial terminal can watch the live sensor feed
    wirelessly. The USB Serial Monitor debug print is unchanged and still works
    when tethered.
  ============================================================================
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// ----------------------------- Motor pins -----------------------------------
const int SLEEP_PIN = 6;
const int M1_EN = 2,  M1_DIR = 3;
const int M2_EN = 4,  M2_DIR = 5;
const int M3_EN = 10, M3_DIR = 9;
const int M4_EN = 12, M4_DIR = 11;
  
// Rotation mix (spin in place) - identical to the working forward/hold sketch.
const int turnSign[4] = { -1, +1, -1, +1 };

// ------------------------ Movement sign patterns ----------------------------
// Motor command sign for each [sector][motor 1..4], multiplied by drive speed.
//   FORWARD : all negative  (matches the proven forward sketch: FORWARD<0 = ahead)
//   BACK    : opposite of forward
//   LEFT    : ( - + + - )    RIGHT : opposite of LEFT
//
//  If left/right spin instead of strafing on the bench, swap to the STRAFE_FIX
//  block below (motors 3 & 4 flipped). Both ball-chase and avoidance reuse these
//  same patterns, so fixing it here fixes both.
enum Sector { S_FWD = 0, S_RIGHT = 1, S_LEFT = 2, S_BACK = 3, S_NONE = 4 };

const int moveSign[4][4] = {
  /* S_FWD   */ { -1, -1, -1, -1 },
  /* S_RIGHT */ { +1, -1, -1, +1 },   // opposite of LEFT
  /* S_LEFT  */ { -1, +1, +1, -1 },
  /* S_BACK  */ { +1, +1, +1, +1 },
};

// // ---- STRAFE_FIX (use if left/right spin instead of sliding sideways) ----
// const int moveSign[4][4] = {
//   /* S_FWD   */ { -1, -1, -1, -1 },
//   /* S_RIGHT */ { -1, +1, +1, -1 },
//   /* S_LEFT  */ { +1, -1, -1, +1 },
//   /* S_BACK  */ { +1, +1, +1, +1 },
// };

// ------------------------------- Tuning -------------------------------------
const int   DRIVE_SPEED   = 250;   // base wheel speed while chasing the ball
const int   AVOID_SPEED   = 250;   // wheel speed while fleeing an obstacle
#define     HOLD_HEADING  1        // 1 = stay facing forward, 0 = off
#define     IR_DIR_INVERT 0        // set 1 if a ball on the RIGHT reads NEGATIVE

#define     US_AVOID_MM   200      // flee any side closer than this (millimetres!)

float       Kp = 6.0, Ki = 0.0, Kd = 0.5;
const float MAX_CORRECTION = 200;
float       K_SIGN = -1.0;

const unsigned long LOOP_MS       = 10;    // 100 Hz control update
const unsigned long IR_TIMEOUT_MS = 250;   // no IR frame this long -> no ball

// ----------------- Capture sensor + solenoid kicker -------------------------
// A single HC-SR04 on the main PCB looks into the dribbler/capture mouth. When
// the ball sits in the mouth its face reads < CAPTURE_BALL_MM away. After
// CONFIRM_SAMPLES consecutive in-range pings confirm it (this rejects single
// glitches), the kicker solenoid fires through the relay for KICK_MS, then rests
// for KICK_COOLDOWN_MS before it can fire again. A hard KICK_MAX_ON_MS guard
// force-releases the coil if it is EVER energised longer than that, whatever the
// state machine does. Pins from the Main PCB doc:
//   HC-SR04   : TRIG = pin 20, ECHO = pin 21   (sensor at 3.3 V, so ECHO is 3.3 V
//               -> safe wired straight to the Teensy, no divider)
//   Relay IN1 : pin 22  -> switches the kicker solenoid
const int CAPTURE_TRIG = 20;
const int CAPTURE_ECHO = 21;
const int SOLENOID_PIN = 22;

#define   CAPTURE_BALL_MM   45       // ball "captured" when closer than this (4.5 cm)
#define   KICK_MS           500      // solenoid energised time per kick (0.5 s pulse) - matches the sketch
#define   KICK_COOLDOWN_MS  1500     // forced gap AFTER a kick before the next (1.5 s)
#define   CONFIRM_SAMPLES   2        // consecutive in-range pings needed before firing (rejects single glitches)
#define   KICK_MAX_ON_MS    600      // HARD safety: coil may NEVER be energised longer than this

const unsigned long CAPTURE_PING_MS         = 60;   // re-ping capture sensor at ~17 Hz (HC-SR04 wants >= 60 ms)
const unsigned int  CAPTURE_ECHO_TIMEOUT_US = 1800; // ~30 cm ceiling -> bounds pulseIn blocking

// Relay trigger polarity. Most blue opto-isolated relay boards are ACTIVE-LOW
// (IN = LOW energises the relay); if yours kicks inverted, set this to 0.
// >>> SAFETY: confirm the relay stays OFF at power-up (LED off) BEFORE wiring the
//     solenoid, so a wrong guess can never hold the coil energised.
#define   RELAY_ACTIVE_LOW  0
#if RELAY_ACTIVE_LOW
  #define RELAY_ON_LEVEL  LOW
  #define RELAY_OFF_LEVEL HIGH
#else
  #define RELAY_ON_LEVEL  HIGH
  #define RELAY_OFF_LEVEL LOW
#endif

// ------------------------------- Gyro ---------------------------------------
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire2);

// ============================================================================
//  ULTRASONIC RECEIVER  -  parser (struct + crc + poll) copied UNCHANGED from
//  Ultrasonic_Receiver_MainBoard. Two config-only edits for this board:
//    * US_PORT = Serial3 (the US PCB wires to pins 14/15 here)
//    * the side-index enum is set to the robot's REAL orientation (see below) -
//      config.h lists indices clockwise as F,R,B,L, but on this robot the board
//      reads B,L,F,R (the NanoEvery's own USB debug labels agree), a 180 deg
//      rotation. The parsing/framing logic is identical.
// ============================================================================
#define US_SYNC0 0xAA
#define US_SYNC1 0x55
#define US_PORT  Serial3          // <-- the UART wired to the ultrasonic board
#define US_BAUD  115200
#define US_STALE_MS 200           // flag data as stale if no good frame this long

// Physical side order on THIS robot is BACK, LEFT, FRONT, RIGHT for packet
// indices 0..3 (matches the NanoEvery's USB debug labels {B,L,F,R}); the ranging
// board sits rotated 180 deg vs config.h's clockwise "F,R,B,L" listing. Mapping
// the names here means "FRONT" really is the robot's nose everywhere below.
// >>> If you ever physically re-seat the board, this enum is the ONE place to fix.
enum { US_BACK = 0, US_LEFT = 1, US_FRONT = 2, US_RIGHT = 3 };
enum { US_ST_OK = 0, US_ST_OUT_OF_RANGE, US_ST_HOLD };

struct UltrasonicData {
  uint16_t mm[4];        // distance per side, millimetres (index = US_FRONT ...)
  uint8_t  status[4];    // US_ST_OK / US_ST_OUT_OF_RANGE / US_ST_HOLD
  uint8_t  seq;          // sender's packet counter (gaps => dropped packets)
  uint32_t lastUpdateMs; // millis() of the last CRC-valid frame
};
UltrasonicData us = {{2600,2600,2600,2600},{1,1,1,1},0,0};

static uint8_t us_crc8(const uint8_t *d, uint8_t n) {
  uint8_t c = 0x00;
  while (n--) {
    c ^= *d++;
    for (uint8_t b = 0; b < 8; b++)
      c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
  }
  return c;
}

// Non-blocking: drains the port, returns true when a fresh valid frame arrived.
bool ultrasonicPoll(Stream &port) {
  static uint8_t buf[13];
  static uint8_t idx = 0;
  bool got = false;

  while (port.available()) {
    uint8_t b = (uint8_t)port.read();
    if (idx == 0)      { if (b == US_SYNC0) buf[idx++] = b; }
    else if (idx == 1) { if (b == US_SYNC1) buf[idx++] = b; else idx = 0; }
    else {
      buf[idx++] = b;
      if (idx >= 13) {
        idx = 0;
        if (us_crc8(&buf[2], 10) == buf[12]) {
          for (uint8_t i = 0; i < 4; i++)
            us.mm[i] = (uint16_t)buf[2 + 2 * i] | ((uint16_t)buf[3 + 2 * i] << 8);
          for (uint8_t i = 0; i < 4; i++)
            us.status[i] = (buf[10] >> (2 * i)) & 0x03;
          us.seq = buf[11];
          us.lastUpdateMs = millis();
          got = true;
        }
        // CRC fail -> silently drop; header search restarts automatically.
      }
    }
  }
  return got;
}

bool ultrasonicStale() { return (millis() - us.lastUpdateMs) > US_STALE_MS; }
// ============================ end ultrasonic =================================

// ------------------------------ IR UART -------------------------------------
#define IR_SERIAL Serial4
#define IR_BAUD   115200

String        irBuf = "";
float         g_ballDir = 500.0;
bool          g_ballSeen = false;
unsigned long g_lastIR   = 0;

// ------------------------ Bluetooth telemetry (HC-05) -----------------------
// HC-05 Bluetooth-Classic (SPP) module on Serial2: RX2 = pin 7, TX2 = pin 8.
// It is a TRANSPARENT serial<->Bluetooth pipe, so anything printed to BT_SERIAL
// is relayed straight to whatever PAIRED phone/laptop is connected (default
// pairing PIN 1234 or 0000; default device name "HC-05").
//
// POWER + LEVELS (different from a BLE HM-10!):
//   * Feed the HC-05 backboard VCC ~5 V (it has its own on-board 3.3 V regulator,
//     so 3.3 V is NOT enough) - the Teensy 4.1 VIN/5 V pin is fine.
//   * Its DATA pins are 3.3 V logic. HC-05 TX (3.3 V) wires straight into Teensy
//     RX2 (the Teensy is 3.3 V and NOT 5 V tolerant, so this is ideal), and the
//     Teensy 3.3 V TX2 drives HC-05 RXD directly - NO divider needed here (the
//     usual HC-05 RX divider is only for 5 V Arduinos, not a 3.3 V Teensy).
//
// BAUD: HC-05 data mode defaults to 9600 8N1 (matches BT_BAUD). To change it,
// enter AT mode (hold EN/KEY high at power-up, AT runs at 38400) and use
// AT+UART=<baud>,<stop>,<parity>; then set BT_BAUD to match.
//   Teensy TX2 (pin 8) -> HC-05 RXD       Teensy RX2 (pin 7) <- HC-05 TXD
#define BT_SERIAL Serial2
#define BT_BAUD   9600
// Enlarged Serial2 TX ring. The built-in default is only 64 B, but one full
// telemetry line is ~75 B, so without extra room write() would busy-wait and
// stall the 100 Hz control loop. This buffer must outlive setup(), hence file
// scope; it is handed to the UART with addMemoryForWrite() in setup().
static uint8_t btTxBuf[160];

// ------------------------------ PID state -----------------------------------
float setpoint = 0, integral = 0, lastError = 0;
unsigned long lastLoop = 0;

// ---------------------- Capture / kicker state ------------------------------
uint16_t      g_captureMm       = 9999;   // last capture-sensor distance, mm (big = clear)
unsigned long g_lastCapturePing = 0;
uint8_t       g_captureHits     = 0;      // consecutive in-range pings (fire-debounce counter)

enum KickState { K_IDLE = 0, K_KICKING, K_COOLDOWN };
KickState     g_kick   = K_IDLE;
unsigned long g_kickTs = 0;               // timestamp the current kick phase began

// Hard-safety tracking: whether the coil is energised right now and when it was
// switched on, so updateKicker() can force-release it past KICK_MAX_ON_MS no
// matter what state the machine is in.
bool          g_solenoidOn   = false;
unsigned long g_solenoidOnTs = 0;

// ---------------------------------------------------------------------------
void setMotor(int enPin, int dirPin, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) { digitalWrite(dirPin, HIGH); analogWrite(enPin, speed); }
  else            { digitalWrite(dirPin, LOW);  analogWrite(enPin, -speed); }
}

void stopMotors() {
  setMotor(M1_EN, M1_DIR, 0); setMotor(M2_EN, M2_DIR, 0);
  setMotor(M3_EN, M3_DIR, 0); setMotor(M4_EN, M4_DIR, 0);
}

// Drive one sector pattern at `speed`, with the heading-hold correction layered
// on top (turnSign mix). At zero heading error the correction is ~0, so each
// wheel runs the pure sector translation.
void driveSector(Sector s, int speed, float correction) {
  setMotor(M1_EN, M1_DIR, moveSign[s][0]*speed + (int)(turnSign[0]*correction));
  setMotor(M2_EN, M2_DIR, moveSign[s][1]*speed + (int)(turnSign[1]*correction));
  setMotor(M3_EN, M3_DIR, moveSign[s][2]*speed + (int)(turnSign[2]*correction));
  setMotor(M4_EN, M4_DIR, moveSign[s][3]*speed + (int)(turnSign[3]*correction));
}

float wrap180(float a){ while (a > 180) a -= 360; while (a < -180) a += 360; return a; }
float readHeading(){ return bno.getVector(Adafruit_BNO055::VECTOR_EULER).x(); }

// Drain every available byte from the IR link and parse complete frames.
// Numeric chars accumulate; 'a' ends the direction field; 'b' ends the frame.
// The distance field is consumed (to keep frame sync) but no longer stored.
void pollIR() {
  while (IR_SERIAL.available()) {
    char c = (char)IR_SERIAL.read();
    if (c == 'a') {                       // direction field complete
      g_ballDir = irBuf.toFloat();
      irBuf = "";
    } else if (c == 'b') {                // distance field complete -> frame done
      irBuf = "";                         // distance value discarded
      g_lastIR = millis();
      g_ballSeen = (g_ballDir > -181.0 && g_ballDir < 181.0);  // 500 => no ball
    } else if ((c >= '0' && c <= '9') || c == '-' || c == '+' || c == '.') {
      irBuf += c;
      if (irBuf.length() > 12) irBuf = "";   // resync guard against a bad stream
    }
    // '\t' '\r' '\n' and anything else: ignore
  }
}

// Ball sector from its bearing. "Forward" is the -150..150 arc that WRAPS THROUGH
// +/-180 (i.e. -150,-160,-170,+170,+160,+150), so it is the rear ~60deg wedge =
// |dir| >= 150. The side wedges sit just inside it; everything around 0 is "back".
//   forward : |dir| >= 150      right : -150..-80      left : 80..150
//   back    : -80..80 (the remaining region around 0)
// NOTE: this maps NEGATIVE bearings to RIGHT. If a ball on the RIGHT makes the
// robot strafe LEFT, swap the RIGHT/LEFT cases below (or set IR_DIR_INVERT).
Sector classify(float dir) {
  if (dir >= 150 || dir <= -150)  return S_FWD;    // -150..(+/-180)..150 : ahead (wraps)
  if (dir <= -80)                 return S_RIGHT;  // -150 to -80 : ball to the right
  if (dir >=  80)                 return S_LEFT;   // 80 to 150   : ball to the left
  return S_BACK;                                   // -80..80 (around 0) : ball behind
}

// If any ultrasonic side is within US_AVOID_MM, return the direction to DRIVE
// to flee the CLOSEST one; otherwise S_NONE. Stale data -> no avoidance (we
// must not act on ranges we can no longer trust). Out-of-range reads report
// far (~2600 mm) so they never trip the threshold.
Sector avoidanceSector() {
  if (ultrasonicStale()) return S_NONE;

  int      worstSide = -1;
  uint16_t worstMm   = US_AVOID_MM;        // only sides closer than this qualify
  for (int i = 0; i < 4; i++) {
    if (us.mm[i] < worstMm) { worstMm = us.mm[i]; worstSide = i; }
  }

  switch (worstSide) {
    case US_FRONT: return S_BACK;   // obstacle ahead  -> back off
    case US_BACK:  return S_FWD;    // obstacle behind -> move forward
    case US_RIGHT: return S_LEFT;   // obstacle right  -> move left
    case US_LEFT:  return S_RIGHT;  // obstacle left   -> move right
    default:       return S_NONE;   // all clear
  }
}

// ----------------------- capture sensor + kicker ----------------------------
// solenoidOn() stamps the energise time only on the OFF->ON edge, so re-asserting
// it while already energised can never push the KICK_MAX_ON_MS safety deadline back.
void solenoidOn()  { digitalWrite(SOLENOID_PIN, RELAY_ON_LEVEL);
                     if (!g_solenoidOn) { g_solenoidOn = true; g_solenoidOnTs = millis(); } }
void solenoidOff() { digitalWrite(SOLENOID_PIN, RELAY_OFF_LEVEL); g_solenoidOn = false; }

// Fire one HC-SR04 ping into the capture mouth (self-throttled to ~17 Hz).
// Blocks at most CAPTURE_ECHO_TIMEOUT_US (~1.8 ms) waiting for the echo. Stores
// the distance in g_captureMm; a missing echo reports "far" (9999) = no ball.
void pollCapture() {
  unsigned long now = millis();
  if (now - g_lastCapturePing < CAPTURE_PING_MS) return;
  g_lastCapturePing = now;

  digitalWrite(CAPTURE_TRIG, LOW);
  delayMicroseconds(3);
  digitalWrite(CAPTURE_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(CAPTURE_TRIG, LOW);

  unsigned long echo = pulseIn(CAPTURE_ECHO, HIGH, CAPTURE_ECHO_TIMEOUT_US);
  g_captureMm = (echo == 0) ? 9999 : (uint16_t)(echo * 0.1715f);  // mm = us*(343/2)/1000

  // Debounce: count consecutive in-range pings; any clear/glitch read resets it.
  // updateKicker() fires only after CONFIRM_SAMPLES hits in a row.
  if (ballInCaptureZone()) { if (g_captureHits < 255) g_captureHits++; }
  else                     { g_captureHits = 0; }
}

// True only when a real, close echo says the ball is sitting in the mouth.
bool ballInCaptureZone() {
  return (g_captureMm > 0 && g_captureMm < CAPTURE_BALL_MM);
}

// Non-blocking kick sequencer: once CONFIRM_SAMPLES consecutive in-range pings
// confirm the ball (g_captureHits, maintained in pollCapture), energise the
// solenoid for KICK_MS, then hold off for KICK_COOLDOWN_MS before another kick
// can start. The cooldown guarantees the gap between kicks even if the ball
// stays put. A hard guard ABOVE the state machine force-releases the coil if it
// is ever energised longer than KICK_MAX_ON_MS, no matter what.
void updateKicker() {
  unsigned long now = millis();

  // --- HARD SAFETY: the coil may NEVER stay energised past KICK_MAX_ON_MS. If
  //     it somehow does, force it off and drop straight into cooldown. ---
  if (g_solenoidOn && (now - g_solenoidOnTs) >= KICK_MAX_ON_MS) {
    solenoidOff();
    g_kickTs = now;
    g_kick   = K_COOLDOWN;
    Serial.println("!! kicker MAX_ON cutoff");
  }

  switch (g_kick) {
    case K_IDLE:
      solenoidOff();                                 // continuously re-assert OFF
      if (g_captureHits >= CONFIRM_SAMPLES) {        // ball confirmed -> kick now
        solenoidOn();
        g_kickTs = now;
        g_kick   = K_KICKING;
      }
      break;
    case K_KICKING:
      if (now - g_kickTs >= KICK_MS) {               // pulse complete
        solenoidOff();
        g_kickTs = now;
        g_kick   = K_COOLDOWN;
      }
      break;
    case K_COOLDOWN:
      solenoidOff();                                 // continuously re-assert OFF
      if (now - g_kickTs >= KICK_COOLDOWN_MS) {
        g_captureHits = 0;                           // require a fresh detection before the next kick
        g_kick = K_IDLE;
      }
      break;
  }
}

// --------------------------- Bluetooth telemetry ----------------------------
// Push everything the robot RECEIVES out over the HC-05 link as ONE compact CSV
// line, formatted once and handed to the UART as a single non-blocking write.
// Sources mirrored:  the IR ball angle (+ ball-seen flag),  the BNO055 Euler
// angles (the SAME sample the PID used this pass, passed in - no second I2C read),
// and the four ultrasonic ring distances (+ stale flag).  The capture-mouth range
// and the chosen drive sector tag along so the line doubles as a live state view.
// Called ONLY from the throttled (~6.7 Hz) debug block, so the link never floods.
//   IR,<dir>,<seen>,BNO,<yaw>,<roll>,<pitch>,US,<F>,<R>,<B>,<L>,<stale>,CAP,<mm>,DRV,<sector>
void sendBluetoothTelemetry(float dir, Sector chosen, const imu::Vector<3> &euler) {
  static const char *const name[] = { "FWD", "RIGHT", "LEFT", "BACK", "NONE" };

  char line[160];
  int n = snprintf(line, sizeof(line),
    "IR,%.1f,%d,BNO,%.1f,%.1f,%.1f,US,%u,%u,%u,%u,%d,CAP,%u,DRV,%s\r\n",
    (double)dir, g_ballSeen ? 1 : 0,
    (double)euler.x(), (double)euler.y(), (double)euler.z(),
    (unsigned)us.mm[US_FRONT], (unsigned)us.mm[US_RIGHT],
    (unsigned)us.mm[US_BACK],  (unsigned)us.mm[US_LEFT],
    ultrasonicStale() ? 1 : 0, (unsigned)g_captureMm, name[chosen]);

  // DIAGNOSTIC: the availableForWrite() >= n gate was dropped ON PURPOSE. If
  // addMemoryForWrite() didn't actually enlarge the ring on this core build, that
  // gate is false every tick (64 >= ~78 is false) and silently swallows EVERY line.
  // Now we always write; only the snprintf-truncation guard (n < sizeof) remains.
  // CAVEAT: if the ring really isn't enlarged, write() busy-waits ~80 ms per line
  // at 9600 baud and will disturb the 100 Hz loop - acceptable for bench bring-up.
  if (n > 0 && n < (int)sizeof(line))
    BT_SERIAL.write((const uint8_t *)line, n);
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  IR_SERIAL.begin(IR_BAUD);
  US_PORT.begin(US_BAUD);          // ultrasonic board on Serial3 (pins 14/15)
  BT_SERIAL.begin(BT_BAUD);        // HC-05 SPP telemetry on Serial2 (pins 7/8)
  BT_SERIAL.addMemoryForWrite(btTxBuf, sizeof(btTxBuf));  // big TX ring -> never block

  // DIAGNOSTIC boot banner: plain text, short enough to fit even the default 64 B
  // TX ring, so it exercises the whole Teensy -> HC-05 -> terminal path INDEPENDENT
  // of the telemetry send-gate. Repeated so it isn't missed if the BT terminal
  // connects a moment after boot. Clean text => baud OK; garbage => wrong baud;
  // nothing => hardware (check the common ground first).
  for (int i = 0; i < 5; i++) { BT_SERIAL.println("HC-05 telemetry online"); delay(300); }

  // Kicker solenoid OFF before anything else can run (boot-safe).

  pinMode(SOLENOID_PIN, OUTPUT);
  solenoidOff();

  // Capture-mouth HC-SR04 (TRIG out / ECHO in).
  pinMode(CAPTURE_TRIG, OUTPUT);
  digitalWrite(CAPTURE_TRIG, LOW);
  pinMode(CAPTURE_ECHO, INPUT);

  pinMode(SLEEP_PIN, OUTPUT);
  pinMode(M1_EN, OUTPUT); pinMode(M1_DIR, OUTPUT);
  pinMode(M2_EN, OUTPUT); pinMode(M2_DIR, OUTPUT);
  pinMode(M3_EN, OUTPUT); pinMode(M3_DIR, OUTPUT);
  pinMode(M4_EN, OUTPUT); pinMode(M4_DIR, OUTPUT);
  digitalWrite(SLEEP_PIN, HIGH);   // wake all drivers
  stopMotors();

  // IMU fusion: relative heading from gyro+accel, magnetometer ignored.
  if (!bno.begin(OPERATION_MODE_IMUPLUS)) {
    Serial.println("BNO055 not found - check Wire2 wiring or try address 0x29.");
    while (true) { stopMotors(); delay(100); }
  }
  bno.setExtCrystalUse(true);

  delay(1000);                     // let fusion settle
  float sum = 0;
  for (int i = 0; i < 20; i++) { sum += readHeading(); delay(10); }
  setpoint = sum / 20.0;           // "forward" heading the robot will hold
  lastError = 0; integral = 0; lastLoop = millis();

  Serial.print("Center heading = "); Serial.println(setpoint, 1);
}

void loop() {
  pollIR();                                  // keep draining both UARTs every pass
  ultrasonicPoll(US_PORT);
  pollCapture();                             // capture-mouth ranging (self-throttled)
  updateKicker();                            // non-blocking 0.5 s kick + 1.5 s cooldown

  unsigned long now = millis();
  if (now - lastLoop < LOOP_MS) return;      // 100 Hz control gate
  float dt = (now - lastLoop) / 1000.0;
  lastLoop = now;

  // Stale IR link -> assume the ball is gone.
  if (now - g_lastIR > IR_TIMEOUT_MS) g_ballSeen = false;

  float dir = g_ballDir;
  if (IR_DIR_INVERT) dir = -dir;

  // ONE BNO055 sample per control pass: the heading-hold PID and the Bluetooth telemetry
  // both use THIS vector, so the heading they act on / report can never skew apart
  // and Wire2 is touched only once per loop (was twice before).
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float heading = euler.x();

  // ---- heading-hold PID (same maths as the forward sketch) ----
  float correction = 0;
  if (HOLD_HEADING) {
    float error   = wrap180(heading - setpoint);
    integral += error * dt;
    integral  = constrain(integral, -200, 200);     // anti-windup
    float deriv = (error - lastError) / dt;
    lastError = error;
    correction = K_SIGN * (Kp * error + Ki * integral + Kd * deriv);
    correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);
  }

  // ---- decide what to do: AVOID overrides CHASE overrides IDLE ----
  Sector avoid = avoidanceSector();
  Sector s;
  int    speed;
  if (avoid != S_NONE)      { s = avoid;          speed = AVOID_SPEED; }
  else if (g_ballSeen)      { s = classify(dir);  speed = DRIVE_SPEED; }
  else                      { s = S_NONE;         speed = 0; }

  if (s == S_NONE) {
    stopMotors();
    integral = 0; lastError = 0;             // don't wind up while parked
  } else {
    driveSector(s, speed, correction);
  }

  // ---- debug (Serial Monitor) ----
  static unsigned long lastPrint = 0;
  if (now - lastPrint > 150) {
    lastPrint = now;
    const char* name[] = { "FWD", "RIGHT", "LEFT", "BACK", "NONE" };
    Serial.print(avoid != S_NONE ? "[AVOID] " : "[chase] ");
    Serial.print("dir=");  Serial.print(dir, 1);
    Serial.print("  US F="); Serial.print(us.mm[US_FRONT]);
    Serial.print(" R=");     Serial.print(us.mm[US_RIGHT]);
    Serial.print(" B=");     Serial.print(us.mm[US_BACK]);
    Serial.print(" L=");     Serial.print(us.mm[US_LEFT]);
    Serial.print("  -> ");   Serial.print(name[s]);
    const char* ks[] = { "IDLE", "KICKING", "COOLDOWN" };
    Serial.print("  cap=");  Serial.print(g_captureMm);
    Serial.print("mm/");     Serial.print(ks[g_kick]);
    Serial.print("  corr="); Serial.println(correction, 1);

    sendBluetoothTelemetry(dir, s, euler);   // mirror the same snapshot out over the HC-05 link
  }
}
