// ============================================================================
//  AttackerRole.cpp  -  GENERATED for the combined build (Firmware/RobotMain).
//  CANONICAL SOURCE: Firmware/Attacker/Attacker_Chase_Aim_Kick_Line (unchanged).
//  The body is wrapped in an anonymous namespace so it links alongside the
//  defender in one firmware; it exposes attackerSetup()/Loop()/SafeStop().
//  Edit the canonical sketch and re-generate; do not hand-edit this copy.
// ============================================================================
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Roles.h"

namespace {
/*
  ============================================================================
  ATTACKER MAIN  -  IR CHASE + FUSED LINE/US BOUNDARY ESCAPE + CAMERA AIM + KICK
  Teensy 4.1 main board  |  RoboCup Junior Lightweight attacker
  ----------------------------------------------------------------------------
  Sensor links and behaviour:
   - Line PCB receiver on Serial8 (RX8 = pin 34 <- Line PCB TX7/pin 29). Parses the
       Line Teensy's 9-byte packet with the SAME non-blocking poll + CRC-8 + seq
       + stale-timeout pattern as the ultrasonic link. Decodes 4 per-board line
       flags (QTR1=RIGHT, QTR2=FRONT, QTR3=LEFT, QTR4=BACK).

   - Fused field-boundary escape. A side is a boundary only when BOTH agree:
            front : US_FRONT < 300 mm AND QTR2 sees line  -> flee BACK
            right : US_RIGHT < 300 mm AND QTR1 sees line  -> flee LEFT
            left  : US_LEFT  < 300 mm AND QTR3 sees line  -> flee RIGHT
            back  : US_BACK  < 300 mm AND QTR4 sees line  -> flee FRONT
       If more than one qualifies, flee the CLOSEST (smallest US distance). No
       escape fires if EITHER the US link OR the Line link is stale.
       Optional: #define KEEP_OBSTACLE_FALLBACK 1 to restore plain US obstacle
       dodging at LOWER priority than the boundary escape (see that block).

   - Camera aim: on ball capture the chassis turns in place on the BNO055 PID to
       put the open goal-corner dead ahead, then kicks.
         Camera wiring (OpenMV H7 -> Teensy 4.1):
           OpenMV P4 (TX) ---> Teensy pin 7  (RX2)   <-- data line, REQUIRED
           OpenMV P5 (RX) <--- Teensy pin 8  (TX2)   optional, unused
           OpenMV GND     ---- Teensy GND            REQUIRED (common ground)
           OpenMV VIN     <--- Teensy 5V / VIN rail
   - Avoidance debounce: an offending side must persist AVOID_CONFIRM control
       passes before the robot flees.

  PRIORITY each control pass (highest first):
    1. HAVE BALL -> AIM open corner -> KICK
    2. BOUNDARY ESCAPE (line + US agree, debounced)   [+ optional US-only dodge]
    3. CHASE the ball
    4. IDLE

  Wiring (from the Main PCB doc):
    SLEEP -> 6 ;  M1: 2,3 ;  M2: 4,5 ;  M3: 9,10 ;  M4: 11,12
    BNO055 on Wire2: SDA=25, SCL=24, VIN=3.3V, addr 0x28, IMUPLUS
    IR UART       : Serial4 (RX4 = pin 16 <- IR board TX)
    Ultrasonic PCB: Serial3 (RX3 = pin 15 <- US board TX, TX3 = pin 14)
    Line PCB      : Serial8 (RX8 = pin 34 <- Line PCB TX7/pin 29)
    Goal camera   : Serial2 (RX2 = pin 7  <- OpenMV P4/TX ; common GND REQUIRED)
    Capture HC-SR04: TRIG = pin 20, ECHO = pin 21
    Kicker relay  : IN1 = pin 22 (ACTIVE-HIGH on this board)

  Arduino IDE target: Tools -> Board: Teensy 4.1 ;  USB Type: Serial.
  ============================================================================
*/


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
//  Camera aim + kicker tunables.
// ============================================================================
// ---- Goal camera (OpenMV H7 on Serial2) ----
#define   CAM_BAUD      115200
const unsigned long CAM_STALE_MS = 200;     // no valid cam frame this long -> "goal not seen"
const int KICK_AIM_TOL_DEG = 5;             // fire only when |openCornerBear| <= this (deg)

// ---- Camera + IR ball fusion (the IR ring stays PRIMARY) ----
// The camera also reports the orange ball (flags bit3 + ballBearing + ballDist). The
// IR ring is the primary ball sensor; the camera ball is a SHORT-RANGE cross-check and
// a fallback when the IR ring drops the ball. The camera bearing is "+right, 0=front";
// the IR `dir` uses this robot's reversed convention (a ball AHEAD reads near +/-180),
// so the camera ball is mapped into that convention before it is compared/used.
#define   USE_CAM_BALL_FUSION    1          // 0 = ignore the camera ball (IR-only chase)
const uint8_t CAM_BALL_NEAR_CM      = 25;   // camera ball "short range" gate (cm)
const int     CAM_BALL_DISAGREE_DEG = 35;   // |camBall - IR| over this at short range -> trust camera

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
// 0 = bench/kicker-test mode: kick on ball capture alone (no camera needed),
//     to test the kicker in isolation.
#define   REQUIRE_GOAL_TO_KICK  1

// ---- Avoidance debounce: reject single spurious frames ----
const int AVOID_CONFIRM = 3;                // offending side must persist this many
                                            // control passes before the robot flees

// ------------------------------- Tuning (original) --------------------------
const int   DRIVE_SPEED   = 250;   // base wheel speed while chasing the ball
const int   AVOID_SPEED   = 250;   // wheel speed while fleeing a boundary/obstacle

#define     HOLD_HEADING  1        // 1 = stay facing the setpoint, 0 = off
#define     IR_DIR_INVERT 0        // set 1 if a ball on the RIGHT reads NEGATIVE

// ---- FUSED BOUNDARY ESCAPE distance (replaces the old US_AVOID_MM = 200) ----
// A side counts as the FIELD BOUNDARY only when its ultrasonic is under this AND
// that board's QTR sees the white line (see boundaryEscapeSector below).
const uint16_t BOUNDARY_MM = 300;

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
//  ULTRASONIC RECEIVER  -  US_PORT = Serial3; side index enum is this robot's
//  real orientation B,L,F,R.
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
//  LINE PCB RECEIVER  -  Line Teensy on Serial8 (RX8 = pin 34 <- Line TX7/pin 29).
//  Parsed non-blocking like ultrasonicPoll. CRC-8 reuses us_crc8() (poly 0x07,
//  init 0x00) so the Line encoder and this decoder stay symmetric. Frame layout:
//    [0]=0xA5 [1]=0x5A [2]=mask [3..6]=count[0..3] [7]=seq [8]=crc8 over [2..7]
//  mask bits (board order, MUST match the encoder):
//    bit0 = QTR1/RIGHT  bit1 = QTR2/FRONT  bit2 = QTR3/LEFT  bit3 = QTR4/BACK
//  Sync 0xA5/0x5A is deliberately distinct from the ultrasonic link's 0xAA/0x55
//  so a mis-wire can't false-sync one stream onto the other.
// ============================================================================
#define LINE_SYNC0 0xA5
#define LINE_SYNC1 0x5A
#define LINE_PORT  Serial8
#define LINE_BAUD  115200
#define LINE_STALE_MS 200

// QTR board -> robot direction (CONFIRMED). Bit positions MUST match the encoder.
enum { LN_RIGHT_BIT = 0, LN_FRONT_BIT = 1, LN_LEFT_BIT = 2, LN_BACK_BIT = 3 };

struct LineData {
  uint8_t  mask;        // bitmask of boards currently seeing the line
  uint8_t  count[4];    // white-sensor count per board (R,F,L,B) -- diagnostics only
  uint8_t  seq;
  uint32_t lastUpdateMs;
};
LineData line = {0, {0, 0, 0, 0}, 0, 0};

bool linePoll(Stream &port) {
  static uint8_t buf[9];
  static uint8_t idx = 0;
  bool got = false;

  while (port.available()) {
    uint8_t b = (uint8_t)port.read();
    if (idx == 0)      { if (b == LINE_SYNC0) buf[idx++] = b; }
    else if (idx == 1) { if (b == LINE_SYNC1) buf[idx++] = b; else idx = 0; }
    else {
      buf[idx++] = b;
      if (idx >= 9) {
        idx = 0;
        if (us_crc8(&buf[2], 6) == buf[8]) {       // SAME crc8 as the ultrasonic link
          line.mask     = buf[2];
          line.count[0] = buf[3];
          line.count[1] = buf[4];
          line.count[2] = buf[5];
          line.count[3] = buf[6];
          line.seq      = buf[7];
          line.lastUpdateMs = millis();
          got = true;
        }
      }
    }
  }
  return got;
}

bool lineStale() { return (millis() - line.lastUpdateMs) > LINE_STALE_MS; }
bool lineRight() { return (line.mask & (1 << LN_RIGHT_BIT)) != 0; }
bool lineFront() { return (line.mask & (1 << LN_FRONT_BIT)) != 0; }
bool lineLeft()  { return (line.mask & (1 << LN_LEFT_BIT))  != 0; }
bool lineBack()  { return (line.mask & (1 << LN_BACK_BIT))  != 0; }
// ============================ end line receiver ==============================

// ============================================================================
//  GOAL CAMERA RECEIVER  -  OpenMV H7 on Serial2 (Goal_Cam.py is the authority).
//  11-byte frame, parsed non-blocking exactly like ultrasonicPoll:
//    [0]=0xAA [1]=0x55 [2]=flags [3]=attackBearing(i8) [4]=attackDist(u8)
//    [5]=openCornerBear(i8) [6]=keeperBearing(i8) [7]=ownGoalBearing(i8)
//    [8]=ballBearing(i8) [9]=ballDist(u8) [10]=checksum = (sum of bytes 2..9) & 0xFF
//  flags: bit0 attackGoalSeen | bit1 ownGoalSeen | bit2 keeperSeen | bit3 ballSeen.
//  The bearing fields are SIGNED int8 (+ = right of forward), matching
//  Goal_Cam.py send_frame(). The ball fields feed the camera+IR fusion (see loop()).
//  >>> Flash the camera and this sketch together: the frame length changed (9 -> 11).
// ============================================================================
#define CAM_SYNC0 0xAA
#define CAM_SYNC1 0x55
#define CAM_PORT  Serial2

struct CameraData {
  bool     attackGoalSeen;
  bool     ownGoalSeen;
  bool     keeperSeen;
  bool     ballSeen;         // camera sees the orange ball (fusion cross-check)
  int8_t   attackBearing;
  uint8_t  attackDist;
  int8_t   openCornerBear;   // THE aim angle
  int8_t   keeperBearing;
  int8_t   ownGoalBearing;
  int8_t   ballBearing;      // + = ball to the robot's right (camera convention)
  uint8_t  ballDist;         // coarse cm, 255 = far/unknown
  uint32_t lastUpdateMs;
};
CameraData cam = {false, false, false, false, 0, 255, 0, 0, 0, 0, 255, 0};

bool camPoll(Stream &port) {
  static uint8_t buf[11];
  static uint8_t idx = 0;
  bool got = false;

  while (port.available()) {
    uint8_t b = (uint8_t)port.read();
    if (idx == 0)      { if (b == CAM_SYNC0) buf[idx++] = b; }
    else if (idx == 1) { if (b == CAM_SYNC1) buf[idx++] = b; else idx = 0; }
    else {
      buf[idx++] = b;
      if (idx >= 11) {
        idx = 0;
        uint8_t sum = (uint8_t)(buf[2] + buf[3] + buf[4] + buf[5] + buf[6]
                              + buf[7] + buf[8] + buf[9]);
        if (sum == buf[10]) {
          cam.attackGoalSeen = (buf[2] & 0x01) != 0;
          cam.ownGoalSeen    = (buf[2] & 0x02) != 0;
          cam.keeperSeen     = (buf[2] & 0x04) != 0;
          cam.ballSeen       = (buf[2] & 0x08) != 0;
          cam.attackBearing  = (int8_t)buf[3];
          cam.attackDist     = buf[4];
          cam.openCornerBear = (int8_t)buf[5];
          cam.keeperBearing  = (int8_t)buf[6];
          cam.ownGoalBearing = (int8_t)buf[7];
          cam.ballBearing    = (int8_t)buf[8];
          cam.ballDist       = buf[9];
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
int   g_avoidStreak  = 0;            // consecutive offending passes (boundary debounce)
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
Sector  boundaryEscapeSector();

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

// Ball sector from its (reversed) bearing.
//   forward : |dir| >= 150     right : -150..-80     left : 80..150
//   back    : -80..80
Sector classify(float dir) {
  if (dir >= 150 || dir <= -150)  return S_FWD;
  if (dir <= -80)                 return S_RIGHT;
  if (dir >=  80)                 return S_LEFT;
  return S_BACK;
}

// ============================================================================
//  FUSED FIELD-BOUNDARY ESCAPE  (replaces the old US-only avoidanceSector).
//  A side is a BOUNDARY only when BOTH agree on it: that side's ultrasonic is
//  under BOUNDARY_MM AND that board's QTR sees the white line. Flee AWAY from it.
//  If more than one side qualifies, flee the CLOSEST (smallest US distance),
//  reusing the old "worst side" idea but only among line-confirmed sides.
//  Returns S_NONE if EITHER link is stale -- never act on data we cannot trust.
//  The CALLER debounces this over AVOID_CONFIRM passes.
//  Side mapping: US enum is B,L,F,R ; line flags are R,F,L,B.
// ============================================================================
Sector boundaryEscapeSector() {
  if (ultrasonicStale() || lineStale()) return S_NONE;   // both links must be fresh

  int      fleeFrom = -1;             // US side index we are escaping (closest boundary)
  uint16_t bestMm   = BOUNDARY_MM;    // also the "< BOUNDARY_MM" gate (strictly less)

  if (us.mm[US_FRONT] < bestMm && lineFront()) { bestMm = us.mm[US_FRONT]; fleeFrom = US_FRONT; }
  if (us.mm[US_RIGHT] < bestMm && lineRight()) { bestMm = us.mm[US_RIGHT]; fleeFrom = US_RIGHT; }
  if (us.mm[US_LEFT]  < bestMm && lineLeft())  { bestMm = us.mm[US_LEFT];  fleeFrom = US_LEFT;  }
  if (us.mm[US_BACK]  < bestMm && lineBack())  { bestMm = us.mm[US_BACK];  fleeFrom = US_BACK;  }

  switch (fleeFrom) {
    case US_FRONT: return S_BACK;     // boundary ahead  -> drive back
    case US_BACK:  return S_FWD;      // boundary behind -> drive forward
    case US_RIGHT: return S_LEFT;     // boundary right  -> drive left
    case US_LEFT:  return S_RIGHT;    // boundary left   -> drive right
    default:       return S_NONE;     // no side has BOTH US-near AND line
  }
}

// ---- OPTIONAL robot/obstacle dodge fallback (DEFAULT OFF) -------------------
// DEFAULT build (flag 0) has no path that flees on ultrasonic alone. Set this to
// 1 to restore plain US obstacle-dodging at LOWER priority than the boundary
// escape (boundary > dodge > chase): it fires only when no boundary side
// qualifies and the US link is fresh.
#define KEEP_OBSTACLE_FALLBACK 0
#if KEEP_OBSTACLE_FALLBACK
const uint16_t OBSTACLE_MM = 180;         // US-only dodge distance (< BOUNDARY_MM, no line needed)
Sector obstacleDodgeSector() {
  if (ultrasonicStale()) return S_NONE;
  int      worstSide = -1;
  uint16_t worstMm   = OBSTACLE_MM;
  for (int i = 0; i < 4; i++)
    if (us.mm[i] < worstMm) { worstMm = us.mm[i]; worstSide = i; }
  switch (worstSide) {
    case US_FRONT: return S_BACK;
    case US_BACK:  return S_FWD;
    case US_RIGHT: return S_LEFT;
    case US_LEFT:  return S_RIGHT;
    default:       return S_NONE;
  }
}
#endif

// ----------------------- capture sensor + kicker ----------------------------
void solenoidOn()  { digitalWrite(SOLENOID_PIN, RELAY_ON_LEVEL);
                     if (!g_solenoidOn) { g_solenoidOn = true; g_solenoidOnTs = millis(); } }
void solenoidOff() { digitalWrite(SOLENOID_PIN, RELAY_OFF_LEVEL); g_solenoidOn = false; }

bool ballInCaptureZone() {
  return (g_captureMm > 0 && g_captureMm < CAPTURE_BALL_MM);
}

// Capture-mouth ping (self-throttled HC-SR04).
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
void roleSetup() {
  // SAFETY FIRST: force the relay to RELEASE level BEFORE the pin is an output,
  // then make it an output and re-assert OFF (boot-safe, as in the standalone).
  digitalWrite(SOLENOID_PIN, RELAY_OFF_LEVEL);
  pinMode(SOLENOID_PIN, OUTPUT);
  solenoidOff();

  Serial.begin(115200);
  IR_SERIAL.begin(IR_BAUD);
  US_PORT.begin(US_BAUD);          // ultrasonic board on Serial3
  LINE_PORT.begin(LINE_BAUD);      // line board on Serial8 (RX8 = pin 34)
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

void roleLoop() {
  pollIR();
  ultrasonicPoll(US_PORT);
  linePoll(LINE_PORT);                           // line board on Serial8
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

  // ---- camera + IR ball fusion (IR PRIMARY; camera = short-range check / fallback) ----
  bool ballSeen = g_ballSeen;
#if USE_CAM_BALL_FUSION
  if (cam.ballSeen && !camStale()) {
    // map the camera ball ("+right, 0=front") into the IR `dir` convention (ahead ~ +/-180)
    float camDir  = wrap180(180.0f - (float)cam.ballBearing);
    bool  camNear = (cam.ballDist <= CAM_BALL_NEAR_CM);     // 255 = far/unknown -> not near
    if (!g_ballSeen) {
      dir = camDir; ballSeen = true;                        // IR blind -> chase the camera ball
    } else if (camNear && fabs(wrap180(camDir - dir)) > (float)CAM_BALL_DISAGREE_DEG) {
      dir = camDir;                                         // close range + IR disagrees -> trust camera
    }
  }
#endif

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float heading = euler.x();

  // ---- fused boundary escape (+ optional US-only dodge), then debounce ----
  Sector avoidRaw = boundaryEscapeSector();      // line + US must agree on a side
#if KEEP_OBSTACLE_FALLBACK
  if (avoidRaw == S_NONE) avoidRaw = obstacleDodgeSector();   // lower-priority robot dodge
#endif
  if (avoidRaw != S_NONE) { if (g_avoidStreak < 1000) g_avoidStreak++; }
  else                    { g_avoidStreak = 0; }
  bool   avoidActive = (g_avoidStreak >= AVOID_CONFIRM);

  // ---- high-level state: AIM > AVOID > CHASE > IDLE ----
  bool haveBall = (g_captureHits >= CONFIRM_SAMPLES);

  TopState top;
  if (haveBall)             top = ST_AIM;
  else if (avoidActive)     top = ST_AVOID;
  else if (ballSeen)        top = ST_CHASE;   // ballSeen = IR ring OR camera fallback (fusion)
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
    // ---- line PCB flags (fused escape needs line + US to agree) ----
    Serial.print(" | LN[");
    Serial.print(lineRight() ? "R" : "-");
    Serial.print(lineFront() ? "F" : "-");
    Serial.print(lineLeft()  ? "L" : "-");
    Serial.print(lineBack()  ? "B" : "-");
    Serial.print("]");
    Serial.print(lineStale() ? "(STALE)" : "");
    Serial.print(" lseq="); Serial.print(line.seq);
    Serial.print(" avS=");  Serial.print(g_avoidStreak);
    // ---- kicker diagnostics: relay vs capture at a glance ----
    Serial.print(" | cap="); Serial.print(g_captureMm);
    Serial.print("mm hits="); Serial.print(g_captureHits);
    Serial.print(" relay=");  Serial.print(g_solenoidOn ? "ON" : "off");
    Serial.print("/");        Serial.print(ks[g_kick]);
    // ---- camera + aim ----
    Serial.print(" | CAM g="); Serial.print(cam.attackGoalSeen ? 1 : 0);
    Serial.print(" open=");    Serial.print(cam.openCornerBear);
    Serial.print(" ball=");    Serial.print(cam.ballSeen ? 1 : 0);
    Serial.print("@");         Serial.print(cam.ballBearing);
    Serial.print("/");         Serial.print(cam.ballDist);
    Serial.print(camStale() ? "(STALE)" : "");
    Serial.print(" KICKGATE="); Serial.print(goalAimReady() ? 1 : 0);
    Serial.print(" aim=");      Serial.print(setpoint, 1);
    Serial.print(" hdg=");      Serial.print(heading, 1);
    Serial.print(" corr=");     Serial.println(correction, 1);
  }
}

} // anonymous namespace (attacker)

void attackerSetup()    { roleSetup(); }
void attackerLoop()     { roleLoop(); }
void attackerSafeStop() { stopMotors(); solenoidOff(); }   // motors off + relay released