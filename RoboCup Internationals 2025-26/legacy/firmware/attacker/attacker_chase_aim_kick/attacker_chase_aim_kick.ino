/*
  ============================================================================
  ATTACKER MAIN  -  IR CHASE + ULTRASONIC AVOID + CAMERA AIM-CORNER + KICK
  Teensy 4.1 main board  |  RoboCup Junior Lightweight attacker
  ----------------------------------------------------------------------------
  REBUILT FROM the working ir_ball_chase_avoid_angle.ino with the SMALLEST
  changes needed for the four asks - everything else is byte-for-byte the old
  sketch (pin map, motor matrices, heading PID, IR parser, ultrasonic parser,
  single-threshold avoidance, kicker state machine):

   (2) HC-05 BLUETOOTH REMOVED. All telemetry/BT code is gone; Serial2 (pins
       7/8) is now free for the camera. USB Serial debug stays (and is extended).

   (3) CAMERA + "THE TURN THING" KEPT. The OpenMV H7 goal camera feeds Serial2.
       When the ball is captured the chassis turns IN PLACE on the BNO055 PID to
       put the open goal-corner dead ahead, then kicks when aligned.
         Camera wiring (OpenMV H7 -> Teensy 4.1):
           OpenMV P4 (TX) ---> Teensy pin 7  (RX2)   <-- the data line, REQUIRED
           OpenMV P5 (RX) <--- Teensy pin 8  (TX2)   optional, unused
           OpenMV GND     ---- Teensy GND            REQUIRED (common ground)
           OpenMV VIN     <--- Teensy 5V / VIN rail  (H7 takes 3.6-5 V; 3.3 V
                                                      logic, no level shifter)
       Frame = goal_cam.py's 9 bytes @115200 (parsed in camPoll below).

   (1) ULTRASONIC "RANDOMLY GOING OFF" - main-board guard added: avoidance now
       has to see an offending side for AVOID_CONFIRM consecutive control passes
       before it flees, so a single spurious short frame can no longer make the
       robot dart. (Kept the stale gate, so frozen/dead data is never acted on.)
       NOTE: if instead the NANO ITSELF is resetting (its USB debug restarts when
       the motors/solenoid fire), that is electrical, not firmware - see the
       hardware checklist in the hand-off notes (bulk cap on the HC-SR04 5 V,
       common star ground, keep US wiring away from motor leads).

   (4) KICKER WORKS STANDALONE BUT NOT INTEGRATED - the capture-ping is now
       byte-identical to the proven Kicker_Ultrasonic.ino: CAPTURE_ECHO_TIMEOUT_US
       is 12000 (was 1800 here, the lone deviation from the working test), same
       trigger sequence, and the relay is forced OFF *before* its pin becomes an
       output (boot-safe, as in the test). The debug line now prints cap distance,
       the in-range hit count, the live relay level and the kick state every tick,
       so the bench test shows directly whether it is a US miss (cap never < 45)
       or a relay/power fault (relay goes ON but no throw). To reproduce the
       standalone test exactly (kick on capture, no camera needed) set
       REQUIRE_GOAL_TO_KICK to 0.

  PRIORITY each control pass (highest first):
    1. HAVE BALL -> AIM open corner -> KICK   (overrides avoidance AND chase)
    2. AVOIDANCE (ball not held, debounced)   (overrides chase)
    3. CHASE the ball
    4. IDLE

  Wiring (from the Main PCB doc):
    SLEEP -> 6 ;  M1: 2,3 ;  M2: 4,5 ;  M3: 9,10 ;  M4: 11,12
    BNO055 on Wire2: SDA=25, SCL=24, VIN=3.3V, addr 0x28, IMUPLUS
    IR UART       : Serial4 (RX4 = pin 16 <- IR board TX)
    Ultrasonic PCB: Serial3 (RX3 = pin 15 <- US board TX, TX3 = pin 14)
    Goal camera   : Serial2 (RX2 = pin 7  <- OpenMV P4/TX ; common GND REQUIRED)
    Capture HC-SR04: TRIG = pin 20, ECHO = pin 21
    Kicker relay  : IN1 = pin 22 (ACTIVE-HIGH on this board)

  Arduino IDE target: Tools -> Board: Teensy 4.1 ;  USB Type: Serial.
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
//   FORWARD : all negative   BACK : opposite   LEFT : (-+ + -)   RIGHT : opp LEFT
//  If left/right spin instead of strafing, swap to the STRAFE_FIX block below.
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

// Top-level behaviour selected each control pass (debug + dispatch).
enum TopState { ST_IDLE = 0, ST_CHASE = 1, ST_AVOID = 2, ST_AIM = 3 };

// ============================================================================
//  NEW TUNABLES added on top of the old sketch (the camera "turn thing" + the
//  two bug-fix knobs). Everything below this block is the original tuning.
// ============================================================================
// ---- Goal camera (OpenMV H7 on Serial2) ----
#define   CAM_BAUD      115200
const unsigned long CAM_STALE_MS = 200;     // no valid cam frame this long -> "goal not seen"
const int KICK_AIM_TOL_DEG = 5;             // fire only when |openCornerBear| <= this (deg)

// ---- Aim behaviour ----
// AIM_TURN_SIGN: BNO055 yaw direction vs the camera's "+ = right". CANNOT be
// assumed. We command heading = (heading + AIM_TURN_SIGN*openCornerBear). If a
// goal waved to the robot's RIGHT makes it turn LEFT (spins away), flip to -1.0.
// >>> BENCH-VERIFY: wave the goal to the robot's right; it must turn RIGHT.
const float AIM_TURN_SIGN = +1.0;
// Small forward bias while aiming, to keep the ball pinned. 0 = pure spin in
// place (DEFAULT, and the only value at which suppressing avoidance during aim
// is guaranteed wall-safe). Raise (e.g. 40) only with a spotter.
const int AIM_FORWARD_BIAS = 0;
// 1 = match mode: kick ONLY when the goal is seen AND the open corner is aligned
//     (this is "the turn thing").
// 0 = bench/kicker-test mode: kick on ball capture alone, exactly like the
//     standalone Kicker_Ultrasonic.ino (no camera needed). Use this to isolate
//     bug #4 (does the integrated kicker fire at all?).
#define   REQUIRE_GOAL_TO_KICK  1

// ---- Avoidance debounce (fix #1: reject single spurious ultrasonic frames) ---
const int AVOID_CONFIRM = 3;                // offending side must persist this many
                                            // control passes before the robot flees

// ------------------------------- Tuning (original) --------------------------
const int   DRIVE_SPEED   = 250;   // base wheel speed while chasing the ball
const int   AVOID_SPEED   = 250;   // wheel speed while fleeing an obstacle
#define     HOLD_HEADING  1        // 1 = stay facing the setpoint, 0 = off
#define     IR_DIR_INVERT 0        // set 1 if a ball on the RIGHT reads NEGATIVE

#define     US_AVOID_MM   200      // flee any side closer than this (millimetres!)

float       Kp = 6.0, Ki = 0.0, Kd = 0.5;
const float MAX_CORRECTION = 200;
float       K_SIGN = -1.0;

const unsigned long LOOP_MS       = 10;    // 100 Hz control update
const unsigned long IR_TIMEOUT_MS = 250;   // no IR frame this long -> no ball

// ----------------- Capture sensor + solenoid kicker -------------------------
// HC-SR04 in the capture mouth + relay-driven solenoid. After CONFIRM_SAMPLES
// consecutive in-range pings the ball is CAPTURED; the kicker then fires for
// KICK_MS (gated by the aim, see REQUIRE_GOAL_TO_KICK) and rests KICK_COOLDOWN_MS.
// A hard KICK_MAX_ON_MS guard force-releases the coil no matter what.
const int CAPTURE_TRIG = 20;
const int CAPTURE_ECHO = 21;
const int SOLENOID_PIN = 22;

#define   CAPTURE_BALL_MM   45       // ball "captured" when closer than this (4.5 cm)
#define   KICK_MS           500      // solenoid energised time per kick
#define   KICK_COOLDOWN_MS  1500     // forced gap AFTER a kick before the next
#define   CONFIRM_SAMPLES   2        // consecutive in-range pings before firing
#define   KICK_MAX_ON_MS    600      // HARD safety: coil may NEVER be on longer than this

const unsigned long CAPTURE_PING_MS         = 60;    // re-ping at ~17 Hz (HC-SR04 wants >= 60 ms)
// FIX #4: was 1800 us here (~30 cm). The proven Kicker_Ultrasonic.ino uses 12000
// us (~2 m). The short cap was the only firmware difference from the working
// test, so it is restored to the known-good value.
const unsigned int  CAPTURE_ECHO_TIMEOUT_US = 12000; // matches the working standalone test

// Relay polarity. ACTIVE-HIGH on this board (confirmed: active-LOW left the coil
// energised from power-up). Keep 0 unless a relay swap behaves inverted.
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
//  ULTRASONIC RECEIVER  -  parser copied UNCHANGED from the old sketch. US_PORT
//  = Serial3; side index enum is this robot's real orientation B,L,F,R.
// ============================================================================
#define US_SYNC0 0xAA
#define US_SYNC1 0x55
#define US_PORT  Serial3
#define US_BAUD  115200
#define US_STALE_MS 200

enum { US_BACK = 0, US_LEFT = 1, US_FRONT = 2, US_RIGHT = 3 };
enum { US_ST_OK = 0, US_ST_OUT_OF_RANGE, US_ST_HOLD };

struct UltrasonicData {
  uint16_t mm[4];
  uint8_t  status[4];
  uint8_t  seq;
  uint32_t lastUpdateMs;
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
      }
    }
  }
  return got;
}

bool ultrasonicStale() { return (millis() - us.lastUpdateMs) > US_STALE_MS; }
// ============================ end ultrasonic =================================

// ============================================================================
//  GOAL CAMERA RECEIVER  -  OpenMV H7 on Serial2 (goal_cam.py is the authority).
//  9-byte frame, parsed non-blocking exactly like ultrasonicPoll:
//    [0]=0xAA [1]=0x55 [2]=flags [3]=attackBearing(i8) [4]=attackDist(u8)
//    [5]=openCornerBear(i8) [6]=keeperBearing(i8) [7]=ownGoalBearing(i8)
//    [8]=checksum = (sum of bytes 2..7) & 0xFF
//  flags: bit0 attackGoalSeen | bit1 ownGoalSeen | bit2 keeperSeen.
//  The bearing fields are SIGNED int8 (+ = right of forward). Verified
//  byte-for-byte against goal_cam.py send_frame().
// ============================================================================
#define CAM_SYNC0 0xAA
#define CAM_SYNC1 0x55
#define CAM_PORT  Serial2

struct CameraData {
  bool     attackGoalSeen;
  bool     ownGoalSeen;
  bool     keeperSeen;
  int8_t   attackBearing;
  uint8_t  attackDist;
  int8_t   openCornerBear;   // THE aim angle
  int8_t   keeperBearing;
  int8_t   ownGoalBearing;
  uint32_t lastUpdateMs;
};
CameraData cam = {false, false, false, 0, 255, 0, 0, 0, 0};

bool camPoll(Stream &port) {
  static uint8_t buf[9];
  static uint8_t idx = 0;
  bool got = false;

  while (port.available()) {
    uint8_t b = (uint8_t)port.read();
    if (idx == 0)      { if (b == CAM_SYNC0) buf[idx++] = b; }
    else if (idx == 1) { if (b == CAM_SYNC1) buf[idx++] = b; else idx = 0; }
    else {
      buf[idx++] = b;
      if (idx >= 9) {
        idx = 0;
        uint8_t sum = (uint8_t)(buf[2] + buf[3] + buf[4] + buf[5] + buf[6] + buf[7]);
        if (sum == buf[8]) {
          cam.attackGoalSeen = (buf[2] & 0x01) != 0;
          cam.ownGoalSeen    = (buf[2] & 0x02) != 0;
          cam.keeperSeen     = (buf[2] & 0x04) != 0;
          cam.attackBearing  = (int8_t)buf[3];
          cam.attackDist     = buf[4];
          cam.openCornerBear = (int8_t)buf[5];
          cam.keeperBearing  = (int8_t)buf[6];
          cam.ownGoalBearing = (int8_t)buf[7];
          cam.lastUpdateMs   = millis();
          got = true;
        }
      }
    }
  }
  return got;
}

bool camStale() { return (millis() - cam.lastUpdateMs) > CAM_STALE_MS; }
// ============================ end goal camera ================================

// ------------------------------ IR UART -------------------------------------
#define IR_SERIAL Serial4
#define IR_BAUD   115200

String        irBuf = "";
float         g_ballDir = 500.0;
bool          g_ballSeen = false;
unsigned long g_lastIR   = 0;

// ------------------------------ PID state -----------------------------------
float setpoint = 0, integral = 0, lastError = 0;
float forwardHeading = 0;            // boot "forward" heading held in AVOID/CHASE/IDLE
float aimHeading     = 0;            // setpoint while aiming the open corner (state 1)
bool  g_hadBall      = false;        // rising-edge detector for entering possession
bool  g_camNewFrame  = false;        // a fresh cam frame arrived since the last control pass
int   g_avoidStreak  = 0;            // consecutive offending passes (fix #1 debounce)
unsigned long lastLoop = 0;

// ---------------------- Capture / kicker state ------------------------------
uint16_t      g_captureMm       = 9999;
unsigned long g_lastCapturePing = 0;
uint8_t       g_captureHits     = 0;

enum KickState { K_IDLE = 0, K_KICKING, K_COOLDOWN };
KickState     g_kick   = K_IDLE;
unsigned long g_kickTs = 0;

bool          g_solenoidOn   = false;
unsigned long g_solenoidOnTs = 0;

// ---- forward declarations --------------------------------------------------
bool    ballInCaptureZone();
bool    goalAimReady();
Sector  classify(float dir);
Sector  avoidanceSector();

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

// Drive one sector pattern at `speed`, heading-hold correction layered on top.
// With speed = 0 only the turnSign*correction term remains -> a pure spin in
// place (this is how aiming rotates the chassis toward the open corner).
void driveSector(Sector s, int speed, float correction) {
  setMotor(M1_EN, M1_DIR, moveSign[s][0]*speed + (int)(turnSign[0]*correction));
  setMotor(M2_EN, M2_DIR, moveSign[s][1]*speed + (int)(turnSign[1]*correction));
  setMotor(M3_EN, M3_DIR, moveSign[s][2]*speed + (int)(turnSign[2]*correction));
  setMotor(M4_EN, M4_DIR, moveSign[s][3]*speed + (int)(turnSign[3]*correction));
}

float wrap180(float a){ while (a > 180) a -= 360; while (a < -180) a += 360; return a; }
float readHeading(){ return bno.getVector(Adafruit_BNO055::VECTOR_EULER).x(); }

// IR parser - UNCHANGED. 'a' ends direction, 'b' ends frame; 500 = no ball.
void pollIR() {
  while (IR_SERIAL.available()) {
    char c = (char)IR_SERIAL.read();
    if (c == 'a') {
      g_ballDir = irBuf.toFloat();
      irBuf = "";
    } else if (c == 'b') {
      irBuf = "";
      g_lastIR = millis();
      g_ballSeen = (g_ballDir > -181.0 && g_ballDir < 181.0);
    } else if ((c >= '0' && c <= '9') || c == '-' || c == '+' || c == '.') {
      irBuf += c;
      if (irBuf.length() > 12) irBuf = "";
    }
  }
}

// Ball sector from its (reversed) bearing - UNCHANGED from the old sketch.
//   forward : |dir| >= 150     right : -150..-80     left : 80..150
//   back    : -80..80
Sector classify(float dir) {
  if (dir >= 150 || dir <= -150)  return S_FWD;
  if (dir <= -80)                 return S_RIGHT;
  if (dir >=  80)                 return S_LEFT;
  return S_BACK;
}

// Single-threshold avoidance - UNCHANGED from the old sketch (the per-side
// thresholds from the earlier rewrite are dropped). Flee the closest side under
// US_AVOID_MM. The CALLER debounces this over AVOID_CONFIRM passes (fix #1).
Sector avoidanceSector() {
  if (ultrasonicStale()) return S_NONE;

  int      worstSide = -1;
  uint16_t worstMm   = US_AVOID_MM;
  for (int i = 0; i < 4; i++) {
    if (us.mm[i] < worstMm) { worstMm = us.mm[i]; worstSide = i; }
  }

  switch (worstSide) {
    case US_FRONT: return S_BACK;
    case US_BACK:  return S_FWD;
    case US_RIGHT: return S_LEFT;
    case US_LEFT:  return S_RIGHT;
    default:       return S_NONE;
  }
}

// ----------------------- capture sensor + kicker ----------------------------
void solenoidOn()  { digitalWrite(SOLENOID_PIN, RELAY_ON_LEVEL);
                     if (!g_solenoidOn) { g_solenoidOn = true; g_solenoidOnTs = millis(); } }
void solenoidOff() { digitalWrite(SOLENOID_PIN, RELAY_OFF_LEVEL); g_solenoidOn = false; }

bool ballInCaptureZone() {
  return (g_captureMm > 0 && g_captureMm < CAPTURE_BALL_MM);
}

// Capture-mouth ping - now byte-identical to the proven Kicker_Ultrasonic.ino
// (fix #4): same 2 us / 10 us trigger, 12 ms echo timeout, self-throttled.
void pollCapture() {
  unsigned long now = millis();
  if (now - g_lastCapturePing < CAPTURE_PING_MS) return;
  g_lastCapturePing = now;

  digitalWrite(CAPTURE_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(CAPTURE_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(CAPTURE_TRIG, LOW);

  unsigned long echo = pulseIn(CAPTURE_ECHO, HIGH, CAPTURE_ECHO_TIMEOUT_US);
  g_captureMm = (echo == 0) ? 9999 : (uint16_t)(echo * 0.1715f);  // mm = us*(343/2)/1000

  if (ballInCaptureZone()) { if (g_captureHits < 255) g_captureHits++; }
  else                     { g_captureHits = 0; }
}

// Kick gate. REQUIRE_GOAL_TO_KICK 0 -> kick on capture alone (standalone-test
// behaviour). 1 -> also require the goal seen (camera fresh) AND the open corner
// within KICK_AIM_TOL_DEG. Stale camera counts as "goal not seen".
bool goalAimReady() {
#if REQUIRE_GOAL_TO_KICK
  if (camStale() || !cam.attackGoalSeen) return false;
  return (abs((int)cam.openCornerBear) <= KICK_AIM_TOL_DEG);
#else
  return true;
#endif
}

// Non-blocking kicker - UNCHANGED except the goalAimReady() gate on the K_IDLE
// transition. KICK_MAX_ON_MS hard cutoff, pulse, cooldown all as before.
void updateKicker() {
  unsigned long now = millis();

  if (g_solenoidOn && (now - g_solenoidOnTs) >= KICK_MAX_ON_MS) {
    solenoidOff();
    g_kickTs = now;
    g_kick   = K_COOLDOWN;
    Serial.println("!! kicker MAX_ON cutoff");
  }

  switch (g_kick) {
    case K_IDLE:
      solenoidOff();
      if (g_captureHits >= CONFIRM_SAMPLES && goalAimReady()) {
        solenoidOn();
        g_kickTs = now;
        g_kick   = K_KICKING;
      }
      break;
    case K_KICKING:
      if (now - g_kickTs >= KICK_MS) {
        solenoidOff();
        g_kickTs = now;
        g_kick   = K_COOLDOWN;
      }
      break;
    case K_COOLDOWN:
      solenoidOff();
      if (now - g_kickTs >= KICK_COOLDOWN_MS) {
        g_captureHits = 0;
        g_kick = K_IDLE;
      }
      break;
  }
}

// ---------------------------------------------------------------------------
void setup() {
  // SAFETY FIRST: force the relay to RELEASE level BEFORE the pin is an output,
  // then make it an output and re-assert OFF (boot-safe, as in the standalone).
  digitalWrite(SOLENOID_PIN, RELAY_OFF_LEVEL);
  pinMode(SOLENOID_PIN, OUTPUT);
  solenoidOff();

  Serial.begin(115200);
  IR_SERIAL.begin(IR_BAUD);
  US_PORT.begin(US_BAUD);          // ultrasonic board on Serial3
  CAM_PORT.begin(CAM_BAUD);        // OpenMV goal camera on Serial2 (RX2 = pin 7)

  pinMode(CAPTURE_TRIG, OUTPUT);
  digitalWrite(CAPTURE_TRIG, LOW);
  pinMode(CAPTURE_ECHO, INPUT);

  pinMode(SLEEP_PIN, OUTPUT);
  pinMode(M1_EN, OUTPUT); pinMode(M1_DIR, OUTPUT);
  pinMode(M2_EN, OUTPUT); pinMode(M2_DIR, OUTPUT);
  pinMode(M3_EN, OUTPUT); pinMode(M3_DIR, OUTPUT);
  pinMode(M4_EN, OUTPUT); pinMode(M4_DIR, OUTPUT);
  digitalWrite(SLEEP_PIN, HIGH);
  stopMotors();

  if (!bno.begin(OPERATION_MODE_IMUPLUS)) {
    Serial.println("BNO055 not found - check Wire2 wiring or try address 0x29.");
    while (true) { stopMotors(); delay(100); }
  }
  bno.setExtCrystalUse(true);

  delay(1000);
  float sum = 0;
  for (int i = 0; i < 20; i++) { sum += readHeading(); delay(10); }
  forwardHeading = sum / 20.0;
  setpoint   = forwardHeading;
  aimHeading = forwardHeading;
  lastError = 0; integral = 0; lastLoop = millis();

  Serial.print("Center (forward) heading = "); Serial.println(forwardHeading, 1);
}

void loop() {
  pollIR();
  ultrasonicPoll(US_PORT);
  if (camPoll(CAM_PORT)) g_camNewFrame = true;   // latch fresh cam frame for the aim outer loop
  pollCapture();
  updateKicker();                                // fires when captured (+ aligned, if REQUIRE_GOAL_TO_KICK)

  unsigned long now = millis();
  if (now - lastLoop < LOOP_MS) return;          // 100 Hz control gate
  float dt = (now - lastLoop) / 1000.0;
  lastLoop = now;

  if (now - g_lastIR > IR_TIMEOUT_MS) g_ballSeen = false;

  float dir = g_ballDir;
  if (IR_DIR_INVERT) dir = -dir;

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float heading = euler.x();

  // ---- avoidance debounce (fix #1): require AVOID_CONFIRM consecutive offending passes ----
  Sector avoidRaw = avoidanceSector();
  if (avoidRaw != S_NONE) { if (g_avoidStreak < 1000) g_avoidStreak++; }
  else                    { g_avoidStreak = 0; }
  bool   avoidActive = (g_avoidStreak >= AVOID_CONFIRM);

  // ---- high-level state: AIM > AVOID > CHASE > IDLE ----
  bool haveBall = (g_captureHits >= CONFIRM_SAMPLES);

  TopState top;
  if (haveBall)             top = ST_AIM;
  else if (avoidActive)     top = ST_AVOID;
  else if (g_ballSeen)      top = ST_CHASE;
  else                      top = ST_IDLE;

  // ---- setpoint selection: only AIM overrides the forward heading ----
  if (top == ST_AIM) {
    if (!g_hadBall) {                            // rising edge into possession
      aimHeading = heading;                      // hold here until the goal is seen
      integral = 0; lastError = 0;               // drop chase windup; setpoint just jumped
    }
    if (g_camNewFrame && !camStale() && cam.attackGoalSeen) {
      // outer loop: aim the open corner dead ahead. SIGN = AIM_TURN_SIGN (bench-verify).
      aimHeading = wrap180(heading + AIM_TURN_SIGN * (float)cam.openCornerBear);
    }
    setpoint = aimHeading;                        // goal lost -> aimHeading untouched -> holds last good heading
  } else {
    setpoint = forwardHeading;
  }
  g_camNewFrame = false;
  g_hadBall = haveBall;

  // ---- heading-hold PID (UNCHANGED maths) ----
  float correction = 0;
  if (HOLD_HEADING) {
    float error   = wrap180(heading - setpoint);
    integral += error * dt;
    integral  = constrain(integral, -200, 200);
    float deriv = (error - lastError) / dt;
    lastError = error;
    correction = K_SIGN * (Kp * error + Ki * integral + Kd * deriv);
    correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);
  }

  // ---- act ----
  switch (top) {
    case ST_AIM:
      driveSector(S_FWD, AIM_FORWARD_BIAS, correction);   // spin in place to face the open corner
      break;
    case ST_AVOID:
      driveSector(avoidRaw, AVOID_SPEED, correction);
      break;
    case ST_CHASE:
      driveSector(classify(dir), DRIVE_SPEED, correction);
      break;
    case ST_IDLE:
    default:
      stopMotors();
      integral = 0; lastError = 0;
      break;
  }

  // ---- debug (USB Serial Monitor) ----
  static unsigned long lastPrint = 0;
  if (now - lastPrint > 150) {
    lastPrint = now;
    const char* topNm[] = { "IDLE", "CHASE", "AVOID", "AIM" };
    const char* ks[]    = { "IDLE", "KICKING", "COOLDOWN" };

    Serial.print("[");   Serial.print(topNm[top]); Serial.print("] ");
    Serial.print("dir="); Serial.print(dir, 1);
    Serial.print(" US F="); Serial.print(us.mm[US_FRONT]);
    Serial.print(" R=");    Serial.print(us.mm[US_RIGHT]);
    Serial.print(" B=");    Serial.print(us.mm[US_BACK]);
    Serial.print(" L=");    Serial.print(us.mm[US_LEFT]);
    Serial.print(ultrasonicStale() ? "(STALE)" : "");
    Serial.print(" avS=");  Serial.print(g_avoidStreak);
    // ---- kicker diagnostics (fix #4: relay vs US at a glance) ----
    Serial.print(" | cap="); Serial.print(g_captureMm);
    Serial.print("mm hits="); Serial.print(g_captureHits);
    Serial.print(" relay=");  Serial.print(g_solenoidOn ? "ON" : "off");
    Serial.print("/");        Serial.print(ks[g_kick]);
    // ---- camera + aim ----
    Serial.print(" | CAM g="); Serial.print(cam.attackGoalSeen ? 1 : 0);
    Serial.print(" open=");    Serial.print(cam.openCornerBear);
    Serial.print(camStale() ? "(STALE)" : "");
    Serial.print(" KICKGATE="); Serial.print(goalAimReady() ? 1 : 0);
    Serial.print(" aim=");      Serial.print(setpoint, 1);
    Serial.print(" hdg=");      Serial.print(heading, 1);
    Serial.print(" corr=");     Serial.println(correction, 1);
  }
}
