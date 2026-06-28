/*  ============================================================
    DRV8263H Motor Current-Limit Supervisor - Arduino Nano Every
    (external watchdog for the VVS Ballers Main PCB)
    ------------------------------------------------------------
    Reads each motor's current from the driver's IPROPI output and
    coasts any motor that stays stalled, so a wheel pinned against a
    wall can't cook the gearmotor. Runs entirely on the external Nano;
    the Teensy keeps doing the motor PWM on the IN pins as usual.

    *** MAIN PCB WIRING (done) ***
    Each DRV8263H's DRVOFF is lifted off the GND rail and run to the Nano
    (pins below): the Nano holds DRVOFF LOW to run and drives it HIGH to coast
    a stalled motor. That is what lets this supervisor actually cut a pinned
    wheel's current.
    (Side effect: the Nano is now in the enable path - if it isn't running,
    those motors stay coasted. Fail-safe, but expected.)

    WIRING  -  Main PCB  ->  Nano Every
        Driver #1 IPROPI -> A0      Driver #1 DRVOFF -> D2
        Driver #2 IPROPI -> A1      Driver #2 DRVOFF -> D3
        Driver #3 IPROPI -> A2      Driver #3 DRVOFF -> D4
        Driver #4 IPROPI -> A3      Driver #4 DRVOFF -> D5
        Nano GND -> Main PCB GND rail   *** shared ground, mandatory ***
    Leave SLEEP on Teensy pin 6 as-is; the Nano does not touch it.

    PROTECTION STYLE: from a separate board we can only coast on/off, so
    a sustained stall is duty-cycled (STALL_MS driven, COOLDOWN_MS
    coasted, repeat). To protect harder, raise COOLDOWN_MS or lower
    STALL_MS.
    ============================================================ */

const uint8_t NUM_MOTORS = 4;              // your 4 drive motors

const uint8_t IPROPI_PIN[NUM_MOTORS] = { A0, A1, A2, A3 };  // match your IPROPI wiring
const uint8_t DRVOFF_PIN[NUM_MOTORS] = {  2,  3,  4,  5 };  // DRVOFF lines (lifted from GND)

// ---- tuning ----
const float    SENSE_V_PER_A = 0.300f;     // IPROPI sensitivity (volts per amp)
const float    ADC_VREF      = 5.0f;       // ADC reference (5 V default; see note)
const uint16_t ADC_MAX       = 1023;       // 10-bit ADC
const float    I_LIMIT_A     = 1.0f;       // sustained-current ceiling, amps (stall ~1.6 A)
const uint16_t STALL_MS      = 200;        // time over the limit that counts as a stall
const uint16_t COOLDOWN_MS   = 400;        // coast time before retrying a stalled motor
const uint8_t  AVG_SAMPLES   = 8;          // averaging to ride out PWM ripple

// ---- state ----
uint32_t overSince[NUM_MOTORS]  = { 0 };   // when a motor first went over the limit
uint32_t coastUntil[NUM_MOTORS] = { 0 };   // coast this motor until this time
float    lastAmps[NUM_MOTORS]   = { 0 };   // most recent reading (for telemetry)

float readCurrent(uint8_t i) {
  uint32_t acc = 0;
  for (uint8_t s = 0; s < AVG_SAMPLES; s++) acc += analogRead(IPROPI_PIN[i]);
  float v = (acc / (float)AVG_SAMPLES) * (ADC_VREF / ADC_MAX);
  return v / SENSE_V_PER_A;
}

void setup() {
  // Optional: better resolution at our low IPROPI voltages.
  // analogReference(INTERNAL2V5);  // then set ADC_VREF = 2.5 and leave AREF unconnected
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    pinMode(DRVOFF_PIN[i], OUTPUT);
    digitalWrite(DRVOFF_PIN[i], LOW);      // LOW = motor allowed to run
  }
  Serial.begin(115200);
}

void loop() {
  static uint32_t lastPrint = 0;
  uint32_t now = millis();

  for (uint8_t i = 0; i < NUM_MOTORS; i++) {

    if (now < coastUntil[i]) {             // still cooling down -> stay coasted
      digitalWrite(DRVOFF_PIN[i], HIGH);
      lastAmps[i] = 0.0f;
      continue;
    }
    digitalWrite(DRVOFF_PIN[i], LOW);      // release coast, allow drive

    float amps  = readCurrent(i);
    lastAmps[i] = amps;

    if (amps > I_LIMIT_A) {
      if (overSince[i] == 0) overSince[i] = now;
      if (now - overSince[i] >= STALL_MS) {     // pinned -> coast + start cooldown
        digitalWrite(DRVOFF_PIN[i], HIGH);
        coastUntil[i] = now + COOLDOWN_MS;
        overSince[i]  = 0;
        lastAmps[i]   = 0.0f;
      }
    } else {
      overSince[i] = 0;                     // recovered before timeout
    }
  }

  // ---- live telemetry (throttled); comment out if not needed ----
  if (now - lastPrint >= 100) {
    lastPrint = now;
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
      Serial.print('M'); Serial.print(i); Serial.print('=');
      Serial.print(lastAmps[i], 2); Serial.print("A ");
    }
    Serial.println();
  }
}
