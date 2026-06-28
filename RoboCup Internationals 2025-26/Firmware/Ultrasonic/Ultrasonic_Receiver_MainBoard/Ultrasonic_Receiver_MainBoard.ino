/*
 * Ultrasonic_Receiver_MainBoard.ino
 * Reference parser for the main PCB (Teensy / Arduino / etc.) that receives the
 * ultrasonic board's framed packet over a hardware UART.
 *
 * Drop ultrasonicPoll() + the UltrasonicData struct into your main firmware and
 * call ultrasonicPoll(<the Serial port wired to the ultrasonic board>) every
 * loop. It is non-blocking, resyncs on the 2-byte header, and rejects any frame
 * whose CRC-8 fails, so a glitch on the wire can never inject a bad distance.
 *
 * Wiring: ultrasonic-board D1 (TX) -> main-board RX, D0 (RX) <- main-board TX
 *         (TX only needed if you later add commands), common GND. 5 V logic.
 *         If the main board is 3.3 V, level-shift the ultrasonic TX down to RX.
 *
 * Packet (13 bytes, little-endian), distances in millimetres:
 *   [0]=0xAA [1]=0x55 | F.lo F.hi R.lo R.hi B.lo B.hi L.lo L.hi | status seq | crc8
 *   status: 2 bits/sensor (F=bits1:0,R=3:2,B=5:4,L=7:6): 0=OK 1=out-of-range 2=held
 *   crc8: poly 0x07, init 0x00, over the 10 payload bytes [2..11].
 */
#include <Arduino.h>

#define US_SYNC0 0xAA
#define US_SYNC1 0x55
#define US_PORT  Serial2          // <-- the UART wired to the ultrasonic board
#define US_BAUD  115200
#define US_STALE_MS 200           // flag data as stale if no good frame this long

enum { US_FRONT = 0, US_RIGHT, US_BACK, US_LEFT };
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

// --------------------------- example usage ---------------------------------
void setup() {
  US_PORT.begin(US_BAUD);
  Serial.begin(115200);
}

void loop() {
  if (ultrasonicPoll(US_PORT)) {
    // Fresh distances are in us.mm[] (mm). Example: simple wall-following margin.
    Serial.print("F="); Serial.print(us.mm[US_FRONT]);
    Serial.print(" R="); Serial.print(us.mm[US_RIGHT]);
    Serial.print(" B="); Serial.print(us.mm[US_BACK]);
    Serial.print(" L="); Serial.print(us.mm[US_LEFT]);
    Serial.print(" mm  seq="); Serial.println(us.seq);
  }
  if (ultrasonicStale()) {
    // No valid data for >US_STALE_MS: link dropped or board reset. Fail safe here
    // (e.g. don't trust ranges; slow down) rather than acting on old distances.
  }

  // ... rest of your main-board control loop ...
}
