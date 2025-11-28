#include <Arduino.h>

const uint8_t TRIG = 2;
const uint8_t ECHO = 3;

const uint8_t STBY = 6;
const uint8_t AIN1 = 7;
const uint8_t AIN2 = 8;
const uint8_t PWMA = 9;

const int TRIGGER_CM = 7;

const unsigned long FWD_MS   = 300;
const unsigned long FWD_HOLD = 10;
const unsigned long BACK_MS  = 300;
const unsigned long BACK_HOLD= 10;
const unsigned long COOLDOWN = 0;
const unsigned long TIME_BETWEEN_KICKS = 0;

const int FWD_SPEED  = 300;
const int BACK_SPEED = 120;

enum { IDLE, FWD, FWDH, BACK, BACKH, CD } st = IDLE;
unsigned long t0 = 0;
unsigned long lastKick = 0;

long readDistanceCM() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(0);

  digitalWrite(TRIG, HIGH);
  delayMicroseconds(0);
  digitalWrite(TRIG, LOW);

  unsigned long dur = pulseIn(ECHO, HIGH, 20000UL);
  if (!dur) return 9999;
  return (long)(dur / 58UL);
}

inline void stopMotor() {
  analogWrite(PWMA, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
}

inline void fwd(int sp) {
  sp = constrain(sp, 0, 255);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, sp);
}

inline void rev(int sp) {
  sp = constrain(sp, 0, 255);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, sp);
}

void setup() {
  Serial.begin(115200);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  pinMode(STBY, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  digitalWrite(STBY, HIGH);
  stopMotor();

  lastKick = millis() - TIME_BETWEEN_KICKS;
}

void loop() {
  long cm = readDistanceCM();
  Serial.print("Distance: ");
  Serial.print(cm);
  Serial.println(" cm");

  switch (st) {
    case IDLE:
      if (cm <= TRIGGER_CM && (millis() - lastKick >= TIME_BETWEEN_KICKS)) {
        fwd(FWD_SPEED);
        t0 = millis();
        st = FWD;
      }
      break;

    case FWD:
      if (millis() - t0 >= FWD_MS) {
        fwd(FWD_SPEED);
        t0 = millis();
        st = FWDH;
      }
      break;

    case FWDH:
      if (millis() - t0 >= FWD_HOLD) {
        rev(BACK_SPEED);
        t0 = millis();
        st = BACK;
      }
      break;

    case BACK:
      if (millis() - t0 >= BACK_MS) {
        rev(BACK_SPEED);
        t0 = millis();
        st = BACKH;
      }
      break;

    case BACKH:
      if (millis() - t0 >= BACK_HOLD) {
        stopMotor();
        t0 = millis();
        st = CD;
      }
      break;

    case CD:
      if (millis() - t0 >= COOLDOWN) {
        st = IDLE;
        lastKick = millis();
      }
      break;
  }
}
