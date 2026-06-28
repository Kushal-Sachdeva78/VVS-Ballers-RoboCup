#include "RobotLink.h"

RobotLinkClass RobotLink;   // the one global instance

// Self / partner reading addresses (5 bytes each). Each robot READS its own and
// WRITES the partner's. The 6th byte is the string NUL; RF24 uses 5-byte addrs.
static const uint8_t ADDR0[6] = RF_ADDR_0;
static const uint8_t ADDR1[6] = RF_ADDR_1;

// House CRC-8: poly 0x07, init 0x00, no reflection, no final XOR - byte-identical
// to the main board's us_crc8() so encoder and decoder stay symmetric.
uint8_t RobotLinkClass::crc8(const uint8_t* d, uint8_t n) {
  uint8_t c = 0x00;
  while (n--) {
    c ^= *d++;
    for (uint8_t b = 0; b < 8; b++)
      c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
  }
  return c;
}

void RobotLinkClass::begin() {
  _role    = ROLE_DEFENDER;     // fail-safe default until we actually hear a partner
  _radioOk = false;

  // Route SPI1 to its Teensy pins (defaults are already 26/39/27; setting them
  // explicitly is honest and harmless).  >>> BENCH-VERIFY these against the board.
  RF_SPI_BUS.setMOSI(RF_SPI_MOSI);
  RF_SPI_BUS.setMISO(RF_SPI_MISO);
  RF_SPI_BUS.setSCK(RF_SPI_SCK);

  // begin() returns false (and isChipConnected() is false) if the module is
  // absent or mis-wired. EITHER WAY we must not stall: stay _radioOk = false and
  // the robot just runs its static MY_BASE_ROLE (see update()).
  bool ok = _radio.begin(&RF_SPI_BUS);
  if (!ok || !_radio.isChipConnected()) {
    _radioOk = false;
    return;                     // graceful degrade - dynamic switching off, robot still plays
  }

  _radio.setPALevel(RF24_PA_LOW);   // >>> BENCH-VERIFY: LOW is plenty desk-to-desk; raise to
                                    //     RF24_PA_HIGH for full-field range once the link is proven.
  _radio.setDataRate(RF24_1MBPS);
  _radio.setChannel(RF_CHANNEL);
  _radio.setAutoAck(false);         // broadcast heartbeats: no per-packet ack/retry (keeps TX non-blocking)
  _radio.setRetries(0, 0);
  _radio.setPayloadSize(RF_PAYLOAD_LEN);
  _radio.setCRCLength(RF24_CRC_8);  // nRF hardware CRC, IN ADDITION to our house CRC-8 in the payload

  const uint8_t* selfAddr    = (MY_ROBOT_ID == 0) ? ADDR0 : ADDR1;
  const uint8_t* partnerAddr = (MY_ROBOT_ID == 0) ? ADDR1 : ADDR0;
  _radio.openWritingPipe(partnerAddr);
  _radio.openReadingPipe(1, selfAddr);
  _radio.startListening();

  _radioOk = true;
}

void RobotLinkClass::setRefereeGo(bool go) {
  _refGoUsed = true;
  if (go && !_refGo) _refGoEdge = true;   // latch a rising edge (kick-off) for arbitrate()
  _refGo = go;
}

void RobotLinkClass::update() {
  if (!_radioOk) { _role = (Role)MY_BASE_ROLE; return; }   // degraded: static role, never stall

  pollReceive();

  uint32_t now = millis();
  if (now - _lastTxMs >= HEARTBEAT_MS) { _lastTxMs = now; sendHeartbeat(); }

  arbitrate();
}

// Non-blocking receive: drain the RX FIFO, validate framing + CRC, timestamp.
void RobotLinkClass::pollReceive() {
  uint8_t buf[RF_PAYLOAD_LEN];
  while (_radio.available()) {
    _radio.read(buf, RF_PAYLOAD_LEN);
    if (buf[0] != RF_SYNC0 || buf[1] != RF_SYNC1) continue;   // not our framing
    if (crc8(&buf[2], 5) != buf[7])               continue;   // CRC fail -> silently drop
    if (buf[2] == MY_ROBOT_ID)                    continue;   // ignore our own echo, if any
    // payload [2..6] = id, baseRole, currentRole, state, seq
    _partnerId     = buf[2];
    _partnerRole   = buf[4];                                  // partner's reported currentRole
    _lastPartnerMs = millis();
  }
}

// Non-blocking transmit: with auto-ack OFF this is a sub-millisecond TX burst
// (RF24::write() polls micros for TX_DS; it does NOT call delay()). We hop to TX,
// fire one frame, and return to RX so we never miss the partner for long.
void RobotLinkClass::sendHeartbeat() {
  uint8_t pkt[RF_PAYLOAD_LEN];
  pkt[0] = RF_SYNC0;
  pkt[1] = RF_SYNC1;
  pkt[2] = (uint8_t)MY_ROBOT_ID;
  pkt[3] = (uint8_t)MY_BASE_ROLE;
  pkt[4] = (uint8_t)_role;          // our current ARBITRATED role
  pkt[5] = 0x01;                    // alive/state byte: bit0 = alive/running (extend as needed)
  pkt[6] = _txSeq++;
  pkt[7] = crc8(&pkt[2], 5);

  _radio.stopListening();
  _radio.write(pkt, RF_PAYLOAD_LEN);   // auto-ack off -> returns when the burst is out
  _radio.startListening();
}

// Role arbitration. Called every update(); pure millis() logic, no blocking.
void RobotLinkClass::arbitrate() {
  uint32_t now = millis();
  bool rawAlive = (now - _lastPartnerMs) < PARTNER_TIMEOUT_MS;

  // ---- debounce raw-alive -> _partnerAlive so it can't flap ----
  if (rawAlive) {
    if (_aliveSinceMs == 0) _aliveSinceMs = now;                 // start of a fresh alive run
    if (!_partnerAlive && (now - _aliveSinceMs) >= ALIVE_DEBOUNCE_MS) _partnerAlive = true;
  } else {
    _aliveSinceMs = 0;
    _partnerAlive = false;                                        // LOST is immediate (safety)
  }

  // ---- role decision ----
  if (MY_BASE_ROLE == ROLE_DEFENDER) {
    _role = ROLE_DEFENDER;                                        // a base-keeper is always the keeper
    return;
  }

  // base role is ATTACKER from here.
  if (!_partnerAlive) {
    _role = ROLE_DEFENDER;                                        // partner down -> cover the goal NOW
    _refGoEdge = false;                                           // require a fresh GO before coming back
    return;
  }

  // Partner is alive. Promote back to ATTACKER only at a safe re-sync point.
  if (_role != ROLE_ATTACKER) {
    bool stable  = (now - _aliveSinceMs) >= ALIVE_DEBOUNCE_MS;
    bool promote = _refGoUsed ? (_refGoEdge && stable)            // referee GO wired: wait for kick-off
                              : ((now - _aliveSinceMs) >= PROMOTE_RESUME_MS);  // not wired: time fallback
    if (promote) { _role = ROLE_ATTACKER; _refGoEdge = false; }
  }
}

Role    RobotLinkClass::role()         { return _role; }
bool    RobotLinkClass::partnerAlive() { return _partnerAlive; }
bool    RobotLinkClass::radioOk()      { return _radioOk; }
uint8_t RobotLinkClass::partnerRole()  { return _partnerRole; }
uint8_t RobotLinkClass::partnerId()    { return _partnerId; }
