#ifndef ROBOTLINK_H
#define ROBOTLINK_H
#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>            // TMRh20 RF24 library:  https://github.com/nRF24/RF24

// ============================================================================
//  RobotLink  -  inter-robot 2.4 GHz role arbitration  (VVS Ballers, Teensy 4.1)
//  ----------------------------------------------------------------------------
//  ONE firmware flashed to BOTH robots (they differ only by MY_ROBOT_ID /
//  MY_BASE_ROLE below). Each robot broadcasts a framed heartbeat over an
//  nRF24L01+ and listens for its partner's:
//
//      partner ALIVE  ->  play MY_BASE_ROLE
//      partner LOST   ->  force ROLE_DEFENDER   (never leave the goal open)
//
//  Demote-to-keeper on partner loss is IMMEDIATE (a safety move). Promoting BACK
//  to the base role waits for a safe re-sync point: a referee GO edge with the
//  partner stable, or - if the GO line isn't wired - a time-debounced resume.
//
//  FAIL-SAFE / GRACEFUL DEGRADATION
//    * Before the first partner heartbeat is heard -> ROLE_DEFENDER (goal safe).
//    * If the radio won't init (module absent / dead / mis-wired) the robot runs
//      its STATIC MY_BASE_ROLE and NEVER stalls - a dead radio loses dynamic
//      switching only, it can never brick the robot.
//
//  House framing (same convention as the wired sensor links): 2-byte sync +
//  little-endian payload + uint8 rolling seq + CRC-8 (poly 0x07, init 0x00 -
//  byte-identical to the main board's us_crc8()).
//
//  RCJ COMPLIANCE: peer-to-peer, robot-to-robot 2.4 GHz only. No PC / phone /
//  access point in the link.
//
//  Non-blocking everywhere: update() never calls delay().
// ============================================================================

// ---------------------------------------------------------------------------
//  Roles. ROLE_DEFENDER = 0 so the zero / default / fail-safe value is the safe
//  one (cover the goal).
// ---------------------------------------------------------------------------
enum Role { ROLE_DEFENDER = 0, ROLE_ATTACKER = 1 };

// ============================================================================
//  CONFIG BLOCK  -  every pin / address / timing is a #define.
// ============================================================================

// ---- THIS robot's identity (the ONLY thing that differs between the two) ----
//  Flash robot A with MY_ROBOT_ID 0 + ROLE_ATTACKER, robot B with 1 +
//  ROLE_DEFENDER. (Could instead be read from EEPROM or a jumper pin; kept as
//  one define by default per spec.)
#define MY_ROBOT_ID     0                 // 0 or 1                 >>> SET PER ROBOT
#define MY_BASE_ROLE    ROLE_ATTACKER     // ROLE_ATTACKER / ROLE_DEFENDER  >>> SET PER ROBOT

// ---- nRF24L01+ SPI wiring --------------------------------------------------
//  Teensy 4.1 default SPI0 (MOSI 11 / MISO 12 / SCK 13) is OFF-LIMITS: pins 11
//  and 12 are Motor 4 (M4: 11,12). So this link uses SPI1 (MOSI 26 / MISO 39 /
//  SCK 27). CE/CSN sit on free GPIO.
//  >>> BENCH-VERIFY against Main_PCB_2.0 - confirm SPI1 (26/39/27) and the CE/CSN
//  >>> pins below are broken out and FREE (not shared with any board function).
//  Occupied main-board pins (do NOT reuse): 2,3,4,5,6,7,8,9,10,11,12,14,15,16,
//  20,21,22,24,25,34 ; plus Serial7/comms 28/29 and Wire1/OLED 18/19.
#define RF_SPI_BUS      SPI1              // NOT SPI0 (11/12 = Motor 4)
#define RF_SPI_MOSI     26               // SPI1 MOSI   >>> BENCH-VERIFY
#define RF_SPI_MISO     39               // SPI1 MISO   >>> BENCH-VERIFY
#define RF_SPI_SCK      27               // SPI1 SCK    >>> BENCH-VERIFY
#define RF_CE_PIN       30               // free GPIO   >>> BENCH-VERIFY broken out & free
#define RF_CSN_PIN      31               // free GPIO   >>> BENCH-VERIFY broken out & free
//  POWER: the nRF24L01+ is marginal off Teensy 3V3 - solder a 10 uF cap across
//  the module's VCC/GND (a bare module browns out on TX bursts otherwise).

// ---- RF link ---------------------------------------------------------------
//  Both robots MUST share the same channel; pick one clear of arena Wi-Fi
//  (0..125; the band centre is 2400 + channel MHz). Two 5-byte pipe addresses:
//  each robot WRITES to the partner's address and READS on its own (chosen by
//  MY_ROBOT_ID).
#define RF_CHANNEL      108              // >>> BENCH-VERIFY clear of arena 2.4 GHz traffic
#define RF_ADDR_0       "VVSb0"          // robot 0's reading address (5 bytes)
#define RF_ADDR_1       "VVSb1"          // robot 1's reading address (5 bytes)
#define RF_PAYLOAD_LEN  8                // sync0 sync1 | id baseRole curRole state seq | crc8

// ---- timing (non-blocking; all millis()-based) -----------------------------
//  >>> BENCH-VERIFY/tune for your link reliability. PARTNER_TIMEOUT_MS = ~7
//  missed beats; long enough to ride out a few dropped packets, short enough to
//  cover the goal quickly when a robot really dies.
#define HEARTBEAT_MS         100         // broadcast a heartbeat this often (~10 Hz)
#define PARTNER_TIMEOUT_MS   750         // no valid partner frame this long -> LOST
#define ALIVE_DEBOUNCE_MS    300         // partner heard this long before "alive" latches
#define PROMOTE_RESUME_MS    3000        // GO-not-wired fallback: resume base role after this stable

// ---- framing (house convention) --------------------------------------------
#define RF_SYNC0        0xC3             // distinct from US(AA/55), line(A5/5A), cam(AA/55)
#define RF_SYNC1        0x3C

// ============================================================================
//  API  (tiny, non-blocking)
// ============================================================================
class RobotLinkClass {
public:
  void    begin();                 // init the radio (non-blocking); safe if module absent
  void    update();                // call EVERY loop(); non-blocking, never delay()s
  void    setRefereeGo(bool go);   // feed the referee GO/STOP state (optional)
  Role    role();                  // current ARBITRATED role
  bool    partnerAlive();          // debounced partner-alive
  bool    radioOk();               // false -> running static MY_BASE_ROLE (degraded mode)
  uint8_t partnerRole();           // partner's reported currentRole (debug)
  uint8_t partnerId();             // partner's reported id (debug)

private:
  RF24     _radio = RF24(RF_CE_PIN, RF_CSN_PIN);
  bool     _radioOk      = false;
  Role     _role         = ROLE_DEFENDER;   // arbitrated; safe default before first beat
  uint8_t  _txSeq        = 0;
  uint32_t _lastTxMs     = 0;
  uint32_t _lastPartnerMs = 0;              // millis() of last CRC-valid partner frame
  bool     _partnerAlive = false;           // debounced
  uint32_t _aliveSinceMs = 0;               // when the current raw-alive run began
  bool     _refGo        = false;           // last referee GO state
  bool     _refGoUsed    = false;           // has setRefereeGo() ever been called?
  bool     _refGoEdge    = false;           // pending rising edge of GO (consumed on promote)
  uint8_t  _partnerRole  = ROLE_DEFENDER;
  uint8_t  _partnerId    = 0xFF;

  static uint8_t crc8(const uint8_t* d, uint8_t n);   // poly 0x07, init 0x00 (== us_crc8)
  void sendHeartbeat();
  void pollReceive();
  void arbitrate();
};

extern RobotLinkClass RobotLink;   // the one global instance (use like Serial)

#endif // ROBOTLINK_H
