/*
  ============================================================================
  DEFENDER (goalkeeper) - FULL (with Line PCB)  -  Main Teensy 4.1
  ----------------------------------------------------------------------------
  Goalkeeper with the LINE PCB (Serial8) as the PRIMARY box-keeper:
     line-depth -> standoff (depth PID input)
     line-side  -> lateral / corner limits
  The ultrasonics handle lateral centring (the line only marks edges, not centre),
  a back-wall safety cross-check, and RECOVER homing when the line is lost.

  The Line receiver is a single non-blocking linePoll() (8-byte AA 55 CRC frame;
  see config.h LINE_* and the parser below).

  BUILD: Arduino IDE, Tools > Board > Teensy 4.1, USB Type "Serial". Libraries:
         Adafruit BNO055 + Adafruit Unified Sensor (and SSD1306+GFX iff USE_OLED).

  Wiring adds, over the NoLine build:
    Line PCB : Serial8 (RX8 = pin 34 <- Line board TX). 115200, 8-byte CRC frame.
               (If the Line Teensy TX is 3.3V, no shifter is needed into pin 34.)
  ============================================================================
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "config.h"

#if USE_OLED
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
#endif

// ----------------------------------------------------------------------------
// Drive base: motor pins, sign tables, heading PID, IR + ultrasonic parsers.
// ----------------------------------------------------------------------------

// ----------------------------- Motor pins -----------------------------------
const int SLEEP_PIN = 6;
const int M1_EN = 2,  M1_DIR = 3;
const int M2_EN = 4,  M2_DIR = 5;
const int M3_EN = 10, M3_DIR = 9;
const int M4_EN = 12, M4_DIR = 11;

const int turnSign[4] = { -1, +1, -1, +1 };

enum Sector { S_FWD = 0, S_RIGHT = 1, S_LEFT = 2, S_BACK = 3, S_NONE = 4 };

// Goalkeeper state-machine states. Declared up HERE (not beside the state
// machine below) so the type precedes the Arduino auto-generated prototypes,
// which the IDE injects just before the first function definition (us_crc8).
// Otherwise stateName(DefState)'s generated prototype names DefState before it
// exists and the sketch fails to compile.
enum DefState { ST_GUARD = 0, ST_TRACK, ST_CLEAR, ST_RECOVER, ST_STOPPED };

const int moveSign[4][4] = {
  /* S_FWD   */ { -1, -1, -1, -1 },
  /* S_RIGHT */ { +1, -1, -1, +1 },
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

#define     HOLD_HEADING  1
// IR_DIR_INVERT moved to config.h.

float       Kp = 6.0, Ki = 0.0, Kd = 0.5;
const float MAX_CORRECTION = 200;
float       K_SIGN = -1.0;

const unsigned long LOOP_MS       = 10;
const unsigned long IR_TIMEOUT_MS = 250;

const int CAPTURE_TRIG = 20;
const int CAPTURE_ECHO = 21;
const int SOLENOID_PIN = 22;

#define   CAPTURE_BALL_MM   50
#define   KICK_DELAY_MS     250
#define   KICK_MS           100
#define   KICK_COOLDOWN_MS  2000

const unsigned long CAPTURE_PING_MS         = 25;
const unsigned int  CAPTURE_ECHO_TIMEOUT_US = 1800;

#define   RELAY_ACTIVE_LOW  0
#if RELAY_ACTIVE_LOW
  #define RELAY_ON_LEVEL  LOW
  #define RELAY_OFF_LEVEL HIGH
#else
  #define RELAY_ON_LEVEL  HIGH
  #define RELAY_OFF_LEVEL LOW
#endif

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire2);

// ============================ ULTRASONIC ====================================
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

// ------------------------------ IR UART -------------------------------------
#define IR_SERIAL Serial4
#define IR_BAUD   115200

String        irBuf = "";
float         g_ballDir = 500.0;
bool          g_ballSeen = false;
unsigned long g_lastIR   = 0;

float setpoint = 0, integral = 0, lastError = 0;
unsigned long lastLoop = 0;

uint16_t      g_captureMm       = 9999;
unsigned long g_lastCapturePing = 0;

enum KickState { K_READY = 0, K_ARMING, K_FIRING, K_COOLDOWN };
KickState     g_kick   = K_READY;
unsigned long g_kickTs = 0;

void setMotor(int enPin, int dirPin, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) { digitalWrite(dirPin, HIGH); analogWrite(enPin, speed); }
  else            { digitalWrite(dirPin, LOW);  analogWrite(enPin, -speed); }
}

void stopMotors() {
  setMotor(M1_EN, M1_DIR, 0); setMotor(M2_EN, M2_DIR, 0);
  setMotor(M3_EN, M3_DIR, 0); setMotor(M4_EN, M4_DIR, 0);
}

void driveSector(Sector s, int speed, float correction) {
  setMotor(M1_EN, M1_DIR, moveSign[s][0]*speed + (int)(turnSign[0]*correction));
  setMotor(M2_EN, M2_DIR, moveSign[s][1]*speed + (int)(turnSign[1]*correction));
  setMotor(M3_EN, M3_DIR, moveSign[s][2]*speed + (int)(turnSign[2]*correction));
  setMotor(M4_EN, M4_DIR, moveSign[s][3]*speed + (int)(turnSign[3]*correction));
}

float wrap180(float a){ while (a > 180) a -= 360; while (a < -180) a += 360; return a; }
float readHeading(){ return bno.getVector(Adafruit_BNO055::VECTOR_EULER).x(); }

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

void solenoidOn()  { digitalWrite(SOLENOID_PIN, RELAY_ON_LEVEL); }
void solenoidOff() { digitalWrite(SOLENOID_PIN, RELAY_OFF_LEVEL); }

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
  g_captureMm = (echo == 0) ? 9999 : (uint16_t)(echo * 0.1715f);
}

bool ballInCaptureZone() {
  return (g_captureMm > 0 && g_captureMm < CAPTURE_BALL_MM);
}

void updateKicker() {
  unsigned long now = millis();
  switch (g_kick) {
    case K_READY:
      solenoidOff();
      if (ballInCaptureZone()) { g_kickTs = now; g_kick = K_ARMING; }
      break;
    case K_ARMING:
      solenoidOff();
      if (now - g_kickTs >= KICK_DELAY_MS) { solenoidOn(); g_kickTs = now; g_kick = K_FIRING; }
      break;
    case K_FIRING:
      if (now - g_kickTs >= KICK_MS) { solenoidOff(); g_kickTs = now; g_kick = K_COOLDOWN; }
      else                            solenoidOn();
      break;
    case K_COOLDOWN:
      solenoidOff();
      if (now - g_kickTs >= KICK_COOLDOWN_MS) g_kick = K_READY;
      break;
  }
}

// ----------------------------------------------------------------------------
// Goalkeeper-specific code below.
// ----------------------------------------------------------------------------

// ============================================================================
//  LINE PCB RECEIVER  (Serial8).
//  NOTE: this expects an 8-byte depth/side frame. The current Line PCB firmware
//  emits a different 9-byte mask/counts frame (see firmware/line-sensor), so this
//  depth/side contract is not yet produced. linePoll() + the LINE_* constants in
//  config.h are the single place to edit if the frame changes.
//
//  8-byte frame, little-endian, CRC-protected (reuses us_crc8):
//    [0]=0xAA [1]=0x55 | depthLo depthHi | side | flags | seq | crc8
//      depth = uint16 mm to the up-field box line (0xFFFF = LINE_NO_LINE)
//      side  = int8 deg, bearing of the nearest line (+ = line is to the RIGHT)
//      flags = bit0 F | bit1 R | bit2 B | bit3 L arm on the line
//      crc8  = poly 0x07 over payload bytes [2..6]
// ============================================================================
#define LINE_PORT  Serial8
#define LINE_SYNC0 0xAA
#define LINE_SYNC1 0x55

struct LineData {
  uint16_t depthMm;       // dist to up-field line, mm; LINE_NO_LINE = none
  int16_t  sideDeg;       // bearing of nearest line, deg (+right)
  uint8_t  arms;          // bit0 F, bit1 R, bit2 B, bit3 L
  uint8_t  seq;
  uint32_t lastUpdateMs;
};
LineData line = { LINE_NO_LINE, 0, 0, 0, 0 };

bool linePoll(Stream &port) {
  static uint8_t buf[8];
  static uint8_t idx = 0;
  bool got = false;
  while (port.available()) {
    uint8_t b = (uint8_t)port.read();
    if (idx == 0)      { if (b == LINE_SYNC0) buf[idx++] = b; }
    else if (idx == 1) { if (b == LINE_SYNC1) buf[idx++] = b; else idx = 0; }
    else {
      buf[idx++] = b;
      if (idx >= 8) {
        idx = 0;
        if (us_crc8(&buf[2], 5) == buf[7]) {   // CRC over payload [2..6]
          line.depthMm = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
          line.sideDeg = (int8_t)buf[4];
          line.arms    = buf[5];
          line.seq     = buf[6];
          line.lastUpdateMs = millis();
          got = true;
        }
        // CRC fail -> silently drop; header search restarts automatically.
      }
    }
  }
  return got;
}
bool lineStale()    { return (millis() - line.lastUpdateMs) > LINE_STALE_MS; }
bool lineSeesLine() { return !lineStale() && line.depthMm != LINE_NO_LINE; }
// ============================ end Line PCB ===================================

// ---------------------------------------------------------------------------
//  Holonomic mixer (identical to Defender_NoLine - same sign tables).
// ---------------------------------------------------------------------------
void driveXY(int vx, int vy, float corr) {
  long w[4];
  for (int i = 0; i < 4; i++)
    w[i] = (long)vy * moveSign[S_FWD][i]
         + (long)vx * moveSign[S_RIGHT][i]
         + (long)(turnSign[i] * corr);

  long m = 0;
  for (int i = 0; i < 4; i++) { long a = (w[i] < 0) ? -w[i] : w[i]; if (a > m) m = a; }
  if (m > 255) for (int i = 0; i < 4; i++) w[i] = (w[i] * 255) / m;

  setMotor(M1_EN, M1_DIR, (int)w[0]);
  setMotor(M2_EN, M2_DIR, (int)w[1]);
  setMotor(M3_EN, M3_DIR, (int)w[2]);
  setMotor(M4_EN, M4_DIR, (int)w[3]);
}

// ------------------------ lateral / depth PIDs ------------------------------
float lat_integral = 0, lat_last = 0;
float dep_integral = 0, dep_last = 0;
void  resetLat()   { lat_integral = 0; lat_last = 0; }
void  resetDepth() { dep_integral = 0; dep_last = 0; }

int lateralPID(float bearing, float dt, float gainScale) {
  // Reject the IR no-ball sentinel (g_ballDir = +/-500): never steer on it, and
  // clear the PID state so a stale bearing can't kick the derivative term. This
  // guards every caller (TRACK and CLEAR) if the ball leaves view mid-action.
  if (fabs(bearing) > 180.0f) { lat_integral = 0; lat_last = 0; return 0; }
  float err = bearing;
  lat_integral += err * dt;  lat_integral = constrain(lat_integral, -300, 300);
  float deriv = (dt > 0) ? (err - lat_last) / dt : 0;  lat_last = err;
  float out = gainScale * KP_LAT * err + KI_LAT * lat_integral + gainScale * KD_LAT * deriv;
  float lim = (float)LATERAL_MAX_VX * gainScale;
  int vx = (int)constrain(out, -lim, lim);
  return constrain(vx, -255, 255);
}

int depthPID(float err, float dt) {
  dep_integral += err * dt;  dep_integral = constrain(dep_integral, -400, 400);
  float deriv = (dt > 0) ? (err - dep_last) / dt : 0;  dep_last = err;
  float out = KP_DEPTH * err + KI_DEPTH * dep_integral + KD_DEPTH * deriv;
  return (int)constrain(out, -(float)DEPTH_MAX_VY, (float)DEPTH_MAX_VY);
}

// ------------------------- ultrasonic helpers -------------------------------
bool usValid(uint8_t i) { return !ultrasonicStale() && us.status[i] == US_ST_OK; }
int  backStandoffErr()  { return BACK_STANDOFF_MM - (int)us.mm[US_BACK]; }

// ---- DEPTH SOURCE (Full): LINE primary, back-ultrasonic fallback -----------
//  Sign convention (vy < 0 == toward own goal):
//   line  : err = depthMm - LINE_DEPTH_SETPOINT. depth small (too far up-field,
//           close to the line) -> err<0 -> vy<0 (back). depth large -> vy>0.
//   us    : err = BACK_STANDOFF_MM - us.mm[US_BACK]  (own-goal-wall datum).
//  *src is set to 'L'/'U'/'-' for the debug stream.
int depthErr(char* src, float dt) {
  if (lineSeesLine())   { *src = 'L'; return (int)line.depthMm - LINE_DEPTH_SETPOINT; }
  if (usValid(US_BACK)) { *src = 'U'; return backStandoffErr(); }
  *src = '-'; return 0;
}

// ---- LATERAL LIMITS (Full): LINE corners primary, ultrasonic cross-check ----
int clampLateralByLine(int vx) {
  if (lineStale()) return vx;
  bool rightEdge = (line.arms & 0x02) ||
                   (lineSeesLine() && line.sideDeg >  LINE_SIDE_DEADBAND);
  bool leftEdge  = (line.arms & 0x08) ||
                   (lineSeesLine() && line.sideDeg < -LINE_SIDE_DEADBAND);
  if (vx > 0 && rightEdge) vx = 0;   // line on the right edge -> don't slide right
  if (vx < 0 && leftEdge)  vx = 0;
  return vx;
}
int clampLateralByUS(int vx) {
  if (vx > 0 && usValid(US_RIGHT) && us.mm[US_RIGHT] < SIDE_LIMIT_MM) vx = 0;
  if (vx < 0 && usValid(US_LEFT)  && us.mm[US_LEFT]  < SIDE_LIMIT_MM) vx = 0;
  return vx;
}
int clampLateral(int vx) { return clampLateralByUS(clampLateralByLine(vx)); }

// Back-wall safety cross-check: never reverse into the own goal on a bad reading.
int backWallGuard(int vy) {
  if (vy < 0 && usValid(US_BACK) && us.mm[US_BACK] < BACK_WALL_SAFE_MM) vy = 0;
  return vy;
}

// GUARD/RECOVER centring (still ultrasonic: the line marks edges, not centre).
int centerVx() {
  if (!usValid(US_RIGHT) || !usValid(US_LEFT)) return 0;
  int diff = (int)us.mm[US_RIGHT] - (int)us.mm[US_LEFT];
  int vx = (int)(KP_CENTER * diff);
  vx = constrain(vx, -LATERAL_MAX_VX, LATERAL_MAX_VX);
  return clampLateral(vx);
}

// ------------------------- bearing-rate (INTERCEPT) -------------------------
float trk_prevBearing = 0; unsigned long trk_prevMs = 0; bool trk_have = false;
void  resetBearingRate() { trk_have = false; }
float bearingRate(float bearing, unsigned long now) {
  if (!trk_have) { trk_prevBearing = bearing; trk_prevMs = now; trk_have = true; return 0; }
  float dt = (now - trk_prevMs) / 1000.0f;
  float r = (dt > 0) ? wrap180(bearing - trk_prevBearing) / dt : 0;
  trk_prevBearing = bearing; trk_prevMs = now;
  return r;
}

// --------------------------------- camera -----------------------------------
#if USE_CAMERA
struct CamData {
  bool    attackSeen, ownSeen, keeperSeen;
  int8_t  attackBearing;
  uint8_t attackDist;
  int8_t  openCornerBear;
  int8_t  keeperBearing;
  int8_t  ownGoalBearing;
  uint32_t lastMs;
};
CamData cam = {false,false,false,0,255,0,0,0,0};

void cameraPoll() {
  static uint8_t buf[9]; static uint8_t idx = 0;
  while (CAM_PORT.available()) {
    uint8_t b = (uint8_t)CAM_PORT.read();
    if (idx == 0)      { if (b == 0xAA) buf[idx++] = b; }
    else if (idx == 1) { if (b == 0x55) buf[idx++] = b; else idx = 0; }
    else {
      buf[idx++] = b;
      if (idx >= 9) {
        idx = 0;
        uint8_t sum = (uint8_t)(buf[2]+buf[3]+buf[4]+buf[5]+buf[6]+buf[7]);
        if (sum == buf[8]) {
          cam.attackSeen    = buf[2] & 0x01;
          cam.ownSeen       = buf[2] & 0x02;
          cam.keeperSeen    = buf[2] & 0x04;
          cam.attackBearing = (int8_t)buf[3];
          cam.attackDist    = buf[4];
          cam.openCornerBear= (int8_t)buf[5];
          cam.keeperBearing = (int8_t)buf[6];
          cam.ownGoalBearing= (int8_t)buf[7];
          cam.lastMs = millis();
        }
      }
    }
  }
}
bool cameraStale() { return (millis() - cam.lastMs) > CAM_STALE_MS; }
float g_bootHeading = 0;
#endif

// --------------------------------- comms ------------------------------------
#if USE_COMMS
bool g_running = false;
void commsBegin() {
  pinMode(COMMS_GO_PIN,   COMMS_ACTIVE_LOW ? INPUT_PULLUP : INPUT);
  pinMode(COMMS_STOP_PIN, COMMS_ACTIVE_LOW ? INPUT_PULLUP : INPUT);
}
bool commsRunning() {
  bool go   = COMMS_ACTIVE_LOW ? (digitalRead(COMMS_GO_PIN)   == LOW) : (digitalRead(COMMS_GO_PIN)   == HIGH);
  bool stop = COMMS_ACTIVE_LOW ? (digitalRead(COMMS_STOP_PIN) == LOW) : (digitalRead(COMMS_STOP_PIN) == HIGH);
  if (stop) g_running = false; else if (go) g_running = true;
  return g_running;
}
#endif

// --------------------------------- OLED -------------------------------------
#if USE_OLED
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
void oledBegin() {
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled.clearDisplay(); oled.display();
}
void oledShow(const char* st, float dir, int vx, int vy, float corr, char dsrc) {
  oled.clearDisplay();
  oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE); oled.setCursor(0,0);
  oled.print(st); oled.print(g_ballSeen ? " *" : "  ");
  oled.print(" dir="); oled.println(dir, 0);
  oled.print("ln="); if (lineSeesLine()) oled.print(line.depthMm); else oled.print("--");
  oled.print("/"); oled.print(line.sideDeg);
  oled.print(" B="); oled.println(us.mm[US_BACK]);
  oled.print("cap="); oled.print(g_captureMm); oled.print(" k="); oled.println((int)g_kick);
  oled.print("vx="); oled.print(vx); oled.print(" vy="); oled.print(vy);
  oled.print(dsrc); oled.print(" c="); oled.println(corr, 0);
  oled.display();
}
#endif

// ------------------------------ state machine -------------------------------
// (enum DefState is declared near the top, before the first function, so the
//  Arduino auto-prototyper sees it ahead of stateName()'s generated prototype.)
DefState      state    = ST_GUARD;
unsigned long stateTs  = 0;

const char* stateName(DefState s) {
  switch (s) { case ST_GUARD:return "GUARD"; case ST_TRACK:return "TRACK";
               case ST_CLEAR:return "CLEAR"; case ST_RECOVER:return "RECOVER";
               default:return "STOPPED"; }
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  IR_SERIAL.begin(IR_BAUD);
  // NOTE (level shifting): the Ultrasonic PCB is a 5V Nano Every; its UART TX
  // drives Teensy RX3 (pin 15), which is 3.3V-ONLY. Confirm the Ultrasonic PCB
  // has a level shifter / divider on TX -> pin 15. If not, add one before power.
  US_PORT.begin(US_BAUD);
  LINE_PORT.begin(LINE_BAUD);       // Line PCB on Serial8 (RX8 = pin 34)

  pinMode(SOLENOID_PIN, OUTPUT);
  solenoidOff();

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

#if USE_CAMERA
  CAM_PORT.begin(CAM_BAUD);
#endif
#if USE_COMMS
  commsBegin();
#endif
#if USE_OLED
  oledBegin();
#endif

  if (!bno.begin(OPERATION_MODE_IMUPLUS)) {
    Serial.println("BNO055 not found - check Wire2 wiring or try address 0x29.");
    while (true) { stopMotors(); delay(100); }
  }
  bno.setExtCrystalUse(true);

  delay(1000);
  float sum = 0;
  for (int i = 0; i < 20; i++) { sum += readHeading(); delay(10); }
  setpoint = sum / 20.0;
  lastError = 0; integral = 0; lastLoop = millis();
#if USE_CAMERA
  g_bootHeading = setpoint;
#endif

  Serial.print("Defender_Full ready. Up-field heading = "); Serial.println(setpoint, 1);
}

// ---------------------------------------------------------------------------
void loop() {
  pollIR();
  ultrasonicPoll(US_PORT);
  linePoll(LINE_PORT);
  pollCapture();
  updateKicker();
#if USE_CAMERA
  cameraPoll();
#endif

  unsigned long now = millis();
  if (now - lastLoop < LOOP_MS) return;
  float dt = (now - lastLoop) / 1000.0;
  lastLoop = now;

  if (now - g_lastIR > IR_TIMEOUT_MS) g_ballSeen = false;

  float dir = g_ballDir;
  if (IR_DIR_INVERT) dir = -dir;

  // ---- heading-hold PID (attacker maths, unchanged) ----
  float correction = 0;
  if (HOLD_HEADING) {
    float heading = readHeading();
    float error   = wrap180(heading - setpoint);
    if (fabs(error) < HEADING_DEADBAND_DEG) error = 0;  // ignore gyro jitter / tiny errors
    integral += error * dt;
    integral  = constrain(integral, -200, 200);
    float deriv = (error - lastError) / dt;
    lastError = error;
    correction = K_SIGN * (Kp * error + Ki * integral + Kd * deriv);
    correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);
  }

#if USE_CAMERA
  if (cam.attackSeen && !cameraStale()) {
    setpoint += K_CAM_TRIM * (float)cam.attackBearing;
    float off = wrap180(setpoint - g_bootHeading);
    off = constrain(off, -CAM_TRIM_MAX, CAM_TRIM_MAX);
    setpoint = wrap180(g_bootHeading + off);
  }
#endif

#if USE_COMMS
  if (!commsRunning())            state = ST_STOPPED;
  else if (state == ST_STOPPED)   state = ST_GUARD;
#endif

  int vx = 0, vy = 0;
  char dsrc = '-';     // which sensor drove the depth PID this loop (L/U/-)

  switch (state) {
    case ST_GUARD:
      vx = centerVx();
      vy = backWallGuard(depthPID((float)depthErr(&dsrc, dt), dt));
      if (g_ballSeen) { state = ST_TRACK; resetLat(); resetBearingRate(); }
      break;

    case ST_TRACK: {
      float rate = bearingRate(dir, now);
      float gscale = (fabs(rate) > INTERCEPT_RATE_DPS) ? INTERCEPT_BOOST : 1.0f;
      vx = clampLateral(lateralPID(dir, dt, gscale));
      vy = backWallGuard(depthPID((float)depthErr(&dsrc, dt), dt));
      if (ballInCaptureZone())      { state = ST_CLEAR; stateTs = now; }
      else if (!g_ballSeen)         { state = ST_GUARD; resetDepth(); }
      break;
    }

    case ST_CLEAR: {
      vy = CLEAR_PUSH_VY;
      vx = lateralPID(dir, dt, 1.0f);
#if USE_CAMERA
      if (cam.attackSeen && !cameraStale())
        vx = constrain(vx + (int)(CAM_AIM_GAIN * KP_LAT * cam.openCornerBear), -255, 255);
#endif
      vx = clampLateral(vx);
      if (now - stateTs >= CLEAR_PUSH_MS) { state = ST_RECOVER; stateTs = now; resetLat(); resetDepth(); }
      break;
    }

    case ST_RECOVER: {
      // Re-home with whatever depth source is alive (line preferred, US fallback)
      // and re-centre on the ultrasonics; re-assert heading via the corr term.
      vx = centerVx();
      vy = backWallGuard(depthPID((float)depthErr(&dsrc, dt), dt));
      bool depthHomed = (dsrc == 'L') ? (abs((int)line.depthMm - LINE_DEPTH_SETPOINT) < RECOVER_TOL_MM)
                       : (dsrc == 'U') ? (abs(backStandoffErr()) < RECOVER_TOL_MM)
                       : false;
      bool centred = (!usValid(US_RIGHT) || !usValid(US_LEFT) ||
                      abs((int)us.mm[US_RIGHT] - (int)us.mm[US_LEFT]) < RECOVER_TOL_MM * 2);
      if ((depthHomed && centred) || (now - stateTs) > RECOVER_TIMEOUT_MS) { state = ST_GUARD; }
      break;
    }

    case ST_STOPPED:
    default:
      vx = 0; vy = 0;
      break;
  }

  if (state == ST_STOPPED) {
    stopMotors();
    integral = 0; lastError = 0; resetLat(); resetDepth();
  } else if (vx == 0 && vy == 0 && !IDLE_HOLD_HEADING) {
    // Nothing to translate: relax like the proven attacker at idle instead of
    // applying a pure-yaw correction, so we don't chase IMU yaw-drift into a slow
    // spin. Heading re-locks the instant vx or vy becomes non-zero.
    stopMotors();
    integral = 0; lastError = 0;
  } else {
    driveXY(vx, vy, correction);
  }

  // ---- debug (every ~150 ms, no motion required) ----
  static unsigned long lastPrint = 0;
  if (now - lastPrint > 150) {
    lastPrint = now;
    const char* ks[] = { "READY", "ARMING", "FIRING", "COOLDOWN" };
    Serial.print("["); Serial.print(stateName(state)); Serial.print("] ");
    Serial.print("ball="); Serial.print(g_ballSeen ? "Y" : "n");
    Serial.print(" dir=");  Serial.print(dir, 1);
    Serial.print(lineStale() ? "  LN(stale)" : "  LN");
    Serial.print(" d="); if (lineSeesLine()) Serial.print(line.depthMm); else Serial.print("--");
    Serial.print(" s="); Serial.print(line.sideDeg);
    Serial.print(" a="); Serial.print(line.arms, BIN);
    Serial.print(ultrasonicStale() ? "  US(stale)" : "  US");
    Serial.print(" F="); Serial.print(us.mm[US_FRONT]);
    Serial.print(" R="); Serial.print(us.mm[US_RIGHT]);
    Serial.print(" B="); Serial.print(us.mm[US_BACK]);
    Serial.print(" L="); Serial.print(us.mm[US_LEFT]);
    Serial.print("  cap="); Serial.print(g_captureMm);
    Serial.print("mm/");    Serial.print(ks[g_kick]);
    Serial.print("  vx="); Serial.print(vx);
    Serial.print(" vy=");  Serial.print(vy);
    Serial.print("(");     Serial.print(dsrc); Serial.print(")");
    Serial.print(" corr="); Serial.println(correction, 1);
  }
#if USE_OLED
  static unsigned long lastOled = 0;
  if (now - lastOled > 150) { lastOled = now; oledShow(stateName(state), dir, vx, vy, correction, dsrc); }
#endif
}
