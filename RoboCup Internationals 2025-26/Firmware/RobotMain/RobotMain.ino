/*
  ============================================================================
  RobotMain  -  ONE firmware, EITHER role, chosen live by the 2.4 GHz link.
  ----------------------------------------------------------------------------
  RobotLink (nRF24L01+) arbitrates the role every loop:
     partner ALIVE -> play MY_BASE_ROLE      (your flashed role)
     partner LOST  -> cover the goal as keeper (never leave it open)
  This sketch runs the matching behaviour body each pass and safe-stops the
  OUTGOING one (motors + solenoid) the instant the role changes.

  FLASH: set MY_ROBOT_ID and MY_BASE_ROLE in RobotLink.h per robot (one
  ATTACKER, one DEFENDER), then flash this SAME sketch to both robots. With no
  nRF24 fitted each robot simply plays its static base role (RobotLink degrades
  gracefully - see Firmware/Comms/README.md).

  BUILD: Arduino IDE, Teensy 4.1, USB Type "Serial". Needs the RF24 library
  (TMRh20). This folder is self-contained: RobotLink.{h,cpp} are bundled here,
  and the two role bodies are AttackerRole.cpp / DefenderRole.cpp (generated from
  the canonical Attacker/Defender sketches - edit those and re-generate).
  ============================================================================
*/
#include <Arduino.h>
#include "RobotLink.h"
#include "Roles.h"

static Role g_lastRole = ROLE_DEFENDER;

// Feed the referee GO/STOP state so a base-attacker that dropped to keeper only
// resumes attacking at kick-off. Wire this to your GO reader when available; until
// then RobotLink falls back to its timed resume.
static bool refereeGo() {
  // return commsRunning();   // <- your GO/STOP reader
  return false;
}

void setup() {
  Serial.begin(115200);
  RobotLink.begin();        // non-blocking; safe even if the nRF24 module is absent

  // Both roles share one chassis + sensor stack, so initialise both once at boot.
  // Each role unit owns its own copies of the drivers; only the ACTIVE role's
  // loop polls sensors and drives the motors on any given pass.
  attackerSetup();
  defenderSetup();

  g_lastRole = RobotLink.role();
}

void loop() {
  RobotLink.update();                  // receive + heartbeat + arbitrate (non-blocking)
  RobotLink.setRefereeGo(refereeGo());

  Role r = RobotLink.role();
  if (r != g_lastRole) {               // role just changed -> stop the outgoing behaviour safely
    if (g_lastRole == ROLE_ATTACKER) attackerSafeStop();
    else                             defenderSafeStop();
    g_lastRole = r;
  }

  if (r == ROLE_ATTACKER) attackerLoop();
  else                    defenderLoop();   // also the boot + fail-safe role
}
