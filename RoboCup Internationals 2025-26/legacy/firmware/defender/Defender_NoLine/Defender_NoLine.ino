/*
  ============================================================================
  DEFENDER (goalkeeper) - NO LINE BOARD - MINIMAL + gyro + wall avoid  - Teensy 4.1
  ----------------------------------------------------------------------------
  Simple, predictable keeper. Every loop, in strict priority order:

    1) WALL ESCAPE: if the RIGHT ultrasonic <= SIDE_AVOID_MM -> drive LEFT;
                    if the LEFT ultrasonic <= SIDE_AVOID_MM  -> drive RIGHT.
                    (If both are close, move away from the CLOSER wall.)

    2) BE IN RANGE (back ultrasonic):
                    back > BACK_STANDOFF_MAX_MM (above 40 cm) -> drive BACK;
                    back < BACK_STANDOFF_MIN_MM (below 30 cm) -> drive FORWARD.

    3) ONCE IN RANGE, track the ball by the SIGN of the IR bearing. On this keeper
       the IR ring's 0 deg faces the OWN goal, so a ball straight ahead reads near
       +/-180, NOT 0:
                    |dir| >= IR_FRONT_DEG (near +/-180) or no ball -> hold (lined up);
                    dir < 0 -> RIGHT;   dir > 0 -> LEFT.

    + ALWAYS: a BNO055 heading PID holds the boot ("forward") heading. The
      correction is layered onto every move AND applied while holding station,
      so the robot continuously faces forward.

  Motion is ONE translation axis at a time (depth OR lateral), bang-bang at fixed
  speeds, with the yaw correction on top. No kicker, no capture, no state machine.

  Reused VERBATIM from ir_ball_chase_avoid.ino: motor pins + sign tables +
  setMotor/stopMotors/driveSector, the BNO055 heading PID (gains, K_SIGN, boot
  setpoint averaging, IMUPLUS @0x28 on Wire2, wrap180/readHeading), the Serial3
  ultrasonic CRC8 parser (+ the {US_BACK,US_LEFT,US_FRONT,US_RIGHT} enum), and the
  Serial4 IR text parser.

  BUILD: Arduino IDE, Tools > Board > Teensy 4.1, USB Type "Serial". Libraries
  (Library Manager): Adafruit BNO055 + Adafruit Unified Sensor.

  Wiring (Main PCB):
    SLEEP -> 6 ; M1: 2,3 ; M2: 4,5 ; M3: 9,10 ; M4: 11,12
    BNO055 on Wire2 (SDA=25, SCL=24, VIN 3.3V)
    IR UART        : Serial4 (RX4 = pin 16 <- IR board TX)
    Ultrasonic PCB : Serial3 (RX3 = pin 15 <- US board TX, TX3 = pin 14)
    Kicker relay   : pin 22  (held OFF here - this sketch does not kick)
  ============================================================================
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "config.h"

// ----------------------------- Motor pins -----------------------------------
const int SLEEP_PIN = 6;
const int M1_EN = 2,  M1_DIR = 3;
const int M2_EN = 4,  M2_DIR = 5;
const int M3_EN = 10, M3_DIR = 9;
const int M4_EN = 12, M4_DIR = 11;
const int SOLENOID_PIN = 22;        // kicker relay - forced OFF (no kicking here)

// ------------------------ Movement sign patterns ----------------------------
const int turnSign[4] = { -1, +1, -1, +1 };
enum Sector { S_FWD = 0, S_RIGHT = 1, S_LEFT = 2, S_BACK = 3, S_NONE = 4 };
const int moveSign[4][4] = {
  /* S_FWD   */ { +1, +1, +1, +1 },
  /* S_RIGHT */ { -1, +1, +1, -1 },   // opposite of LEFT
  /* S_LEFT  */ { +1, -1, -1, +1 },
  /* S_BACK  */ { -1, -1, -1, -1 },
};
// // ---- STRAFE_FIX (use if left/right SPIN instead of sliding sideways) ----
// const int moveSign[4][4] = {
//   /* S_FWD   */ { -1, -1, -1, -1 },
//   /* S_RIGHT */ { -1, +1, +1, -1 },
//   /* S_LEFT  */ { +1, -1, -1, +1 },
//   /* S_BACK  */ { +1, +1, +1, +1 },
// };

// ------------------------------- Gyro (heading PID) -------------------------
// Verbatim attacker gains. Flip K_SIGN to +1.0 if holding heading SPINS the bot.
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire2);
float       Kp = 6.0, Ki = 0.0, Kd = 0.5;
const float MAX_CORRECTION = 200;
float       K_SIGN = 1.0;
float       setpoint = 0, integral = 0, lastError = 0;

// ============================================================================
//  ULTRASONIC RECEIVER (Serial3) - parser copied UNCHANGED from the attacker.
//  US enum is the robot's REAL orientation; >>> fix it here if the board moves.
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
bool usValid(uint8_t i) { return !ultrasonicStale() && us.status[i] == US_ST_OK; }
// ============================ end ultrasonic =================================

// ------------------------------ IR UART (Serial4) ---------------------------
#define IR_SERIAL Serial4
#define IR_BAUD   115200
const unsigned long IR_TIMEOUT_MS = 250;

String        irBuf = "";
float         g_ballDir  = 500.0;   // 500 = no-ball sentinel
bool          g_ballSeen = false;
unsigned long g_lastIR   = 0;

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

// ------------------------------ motor helpers -------------------------------
void setMotor(int enPin, int dirPin, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) { digitalWrite(dirPin, HIGH); analogWrite(enPin, speed); }
  else            { digitalWrite(dirPin, LOW);  analogWrite(enPin, -speed); }
}

void stopMotors() {
  setMotor(M1_EN, M1_DIR, 0); setMotor(M2_EN, M2_DIR, 0);
  setMotor(M3_EN, M3_DIR, 0); setMotor(M4_EN, M4_DIR, 0);
}

// Sector translation at `speed`, with the heading-hold correction layered on
// top. driveSector(S_FWD, 0, correction) => pure heading hold (no translation).
void driveSector(Sector s, int speed, float correction) {
  setMotor(M1_EN, M1_DIR, moveSign[s][0]*speed + (int)(turnSign[0]*correction));
  setMotor(M2_EN, M2_DIR, moveSign[s][1]*speed + (int)(turnSign[1]*correction));
  setMotor(M3_EN, M3_DIR, moveSign[s][2]*speed + (int)(turnSign[2]*correction));
  setMotor(M4_EN, M4_DIR, moveSign[s][3]*speed + (int)(turnSign[3]*correction));
}

// ------------------------------ heading PID ---------------------------------
float wrap180(float a){ while (a > 180) a -= 360; while (a < -180) a += 360; return a; }
float readHeading(){ return bno.getVector(Adafruit_BNO055::VECTOR_EULER).x(); }

// Yaw correction to hold the boot heading. Deadbanded so gyro noise won't jitter.
float headingCorrection(float dt) {
  float heading = readHeading();
  float error   = wrap180(heading - setpoint);
  if (fabs(error) < HEADING_DEADBAND_DEG) error = 0;
  integral += error * dt;
  integral  = constrain(integral, -200, 200);
  float deriv = (dt > 0) ? (error - lastError) / dt : 0;
  lastError = error;
  float corr = K_SIGN * (Kp * error + Ki * integral + Kd * deriv);
  return constrain(corr, -MAX_CORRECTION, MAX_CORRECTION);
}

// ----------------------------- loop timing ----------------------------------
const unsigned long LOOP_MS = 10;          // 100 Hz
unsigned long lastLoop = 0;

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  IR_SERIAL.begin(IR_BAUD);
  // NOTE (level shifting): the Ultrasonic PCB is a 5V Nano Every; its UART TX
  // drives Teensy RX3 (pin 15), which is 3.3V-ONLY. Confirm a level shifter /
  // divider sits on TX -> pin 15. This sketch does not change that.
  US_PORT.begin(US_BAUD);

  pinMode(SOLENOID_PIN, OUTPUT);
  digitalWrite(SOLENOID_PIN, LOW);         // kicker relay OFF (boot-safe; unused)

  pinMode(SLEEP_PIN, OUTPUT);
  pinMode(M1_EN, OUTPUT); pinMode(M1_DIR, OUTPUT);
  pinMode(M2_EN, OUTPUT); pinMode(M2_DIR, OUTPUT);
  pinMode(M3_EN, OUTPUT); pinMode(M3_DIR, OUTPUT);
  pinMode(M4_EN, OUTPUT); pinMode(M4_DIR, OUTPUT);
  digitalWrite(SLEEP_PIN, HIGH);
  stopMotors();

  // IMU fusion: relative heading from gyro+accel, magnetometer ignored.
  if (!bno.begin(OPERATION_MODE_IMUPLUS)) {
    Serial.println("BNO055 not found - check Wire2 wiring or try address 0x29.");
    while (true) { stopMotors(); delay(100); }
  }
  bno.setExtCrystalUse(true);

  delay(1000);                             // let fusion settle
  float sum = 0;
  for (int i = 0; i < 20; i++) { sum += readHeading(); delay(10); }
  setpoint = sum / 20.0;                    // the "forward" heading it will hold
  lastError = 0; integral = 0; lastLoop = millis();

  Serial.print("Defender_NoLine ready. Forward heading = "); Serial.println(setpoint, 1);
}

// ---------------------------------------------------------------------------
void loop() {
  pollIR();
  ultrasonicPoll(US_PORT);

  unsigned long now = millis();
  if (now - lastLoop < LOOP_MS) return;
  float dt = (now - lastLoop) / 1000.0;
  lastLoop = now;

  if (now - g_lastIR > IR_TIMEOUT_MS) g_ballSeen = false;

  float dir = g_ballDir;
  if (IR_DIR_INVERT) dir = -dir;

  // ---- heading-hold correction (ALWAYS applied so it continuously faces fwd) ----
  float correction = headingCorrection(dt);

  // ---- pick ONE translation by priority (wall escape > range > ball L/R) ----
  bool rNear = usValid(US_RIGHT) && (int)us.mm[US_RIGHT] <= SIDE_AVOID_MM;
  bool lNear = usValid(US_LEFT)  && (int)us.mm[US_LEFT]  <= SIDE_AVOID_MM;

  Sector cmd = S_FWD;          // default = hold position (speed 0) but keep facing forward
  int    speed = 0;
  const char* why = "hold (face fwd)";

  if (rNear || lNear) {
    // PRIORITY 1: too close to a side wall -> slide AWAY from the closer one.
    bool awayLeft = (rNear && lNear) ? ((int)us.mm[US_RIGHT] <= (int)us.mm[US_LEFT]) : rNear;
    if (awayLeft) { cmd = S_LEFT;  speed = LATERAL_SPEED; why = "wall RIGHT<=20cm -> LEFT"; }
    else          { cmd = S_RIGHT; speed = LATERAL_SPEED; why = "wall LEFT<=20cm -> RIGHT"; }
  } else if (!usValid(US_BACK)) {
    cmd = S_FWD; speed = 0; why = "back US invalid -> hold";          // fail-safe
  } else if ((int)us.mm[US_BACK] > BACK_STANDOFF_MAX_MM) {
    cmd = S_BACK; speed = DEPTH_SPEED; why = "above range -> BACK";
  } else if ((int)us.mm[US_BACK] < BACK_STANDOFF_MIN_MM) {
    cmd = S_FWD;  speed = DEPTH_SPEED; why = "below range -> FORWARD";
  } else {
    // in range -> track the ball by the SIGN of the IR bearing. On this keeper the
    // IR 0 deg faces the own goal, so a ball straight ahead reads near +/-180:
    //   |dir| >= IR_FRONT_DEG -> FRONT (lined up) -> hold;  dir<0 -> RIGHT;  dir>0 -> LEFT.
    if      (!g_ballSeen)                    { cmd = S_FWD;   speed = 0;             why = "in range, no ball -> hold"; }
    else if (fabs(dir) >= IR_FRONT_DEG)      { cmd = S_FWD;   speed = 0;             why = "in range, ball FRONT -> hold"; }
    else if (dir < 0)                        { cmd = S_RIGHT; speed = LATERAL_SPEED; why = "in range, ball (-) -> RIGHT"; }
    else                                     { cmd = S_LEFT;  speed = LATERAL_SPEED; why = "in range, ball (+) -> LEFT"; }
  }

  // speed 0 => moveSign term is 0, so only the heading correction is applied.
  driveSector(cmd, speed, correction);

  // ---- debug (every ~150 ms, prints even when holding) ----
  static unsigned long lastPrint = 0;
  if (now - lastPrint > 150) {
    lastPrint = now;
    Serial.print("B=");  Serial.print(us.mm[US_BACK]);
    Serial.print(" L="); Serial.print(us.mm[US_LEFT]);
    Serial.print(" R="); Serial.print(us.mm[US_RIGHT]);
    Serial.print(ultrasonicStale() ? "(stale)" : "");
    Serial.print("  ball="); Serial.print(g_ballSeen ? "Y" : "n");
    Serial.print(" dir=");   Serial.print(dir, 1);
    Serial.print(" corr=");  Serial.print(correction, 1);
    Serial.print("   -> ");  Serial.println(why);
  }
}
