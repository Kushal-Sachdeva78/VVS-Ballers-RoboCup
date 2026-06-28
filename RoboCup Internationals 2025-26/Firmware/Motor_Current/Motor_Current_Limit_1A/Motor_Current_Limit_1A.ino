/*  Simple 1 A current limit  -  Arduino Nano Every + DRV8263H x4
    ----------------------------------------------------------------
    Coasts a motor (DRVOFF HIGH) the instant its current crosses the
    limit and releases it when back under. Per motor, no timers - it
    chops on/off around ~1 A, holding the current near the cap instead
    of letting it run up to the ~1.6 A stall.

    WIRING: each driver's DRVOFF is lifted off the GND rail and run to the
    Nano; shared ground between Nano and Main PCB. The Teensy still generates
    the motor PWM on the IN pins.

        Driver #1  IPROPI -> A0   DRVOFF -> D2
        Driver #2  IPROPI -> A1   DRVOFF -> D3
        Driver #3  IPROPI -> A2   DRVOFF -> D4
        Driver #4  IPROPI -> A3   DRVOFF -> D5
    ---------------------------------------------------------------- */

const uint8_t NUM_MOTORS = 4;
const uint8_t IPROPI_PIN[NUM_MOTORS] = { A0, A1, A2, A3 };
const uint8_t DRVOFF_PIN[NUM_MOTORS] = {  2,  3,  4,  5 };

const float I_LIMIT_A = 1.0;     // <-- current ceiling, amps
const float V_PER_A   = 0.300;   // IPROPI sensitivity: 300 mV/A
const float ADC_VREF  = 5.0;     // Nano Every ADC reference (V)
const int   ADC_MAX   = 1023;    // 10-bit ADC

// limit expressed in raw ADC counts (computed once -> keeps the loop fast)
const int LIMIT_COUNTS = (int)(I_LIMIT_A * V_PER_A / ADC_VREF * ADC_MAX);

void setup() {
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    pinMode(DRVOFF_PIN[i], OUTPUT);
    digitalWrite(DRVOFF_PIN[i], LOW);          // LOW = run
  }
}

void loop() {
  for (uint8_t i = 0; i < NUM_MOTORS; i++)
    digitalWrite(DRVOFF_PIN[i],
                 analogRead(IPROPI_PIN[i]) > LIMIT_COUNTS ? HIGH : LOW);  // over -> coast
}
