/*
  ============================================================================
  RoleDispatch_Example  -  how to drop RobotLink into the existing main-board
  firmware so ONE build runs EITHER role, chosen at runtime by the radio.
  ----------------------------------------------------------------------------
  This is a SCAFFOLD, not a full robot. It shows exactly where RobotLink hooks
  into a normal setup()/loop() and how the two existing behaviour bodies become
  runAttacker() / runDefender(). You paste the real bodies in (see the README).

  TO BUILD THIS EXAMPLE: put RobotLink.h and RobotLink.cpp next to this .ino
  (or install them as a library), and install the RF24 library (TMRh20).

  >>> NOT hardware-tested. Confirm the radio pins in RobotLink.h on Main_PCB_2.0.
  ============================================================================
*/
#include "RobotLink.h"

// ---------------------------------------------------------------------------
//  The two existing loop BODIES, refactored into functions. In the real merge:
//    runAttacker() = the body of Attacker_Chase_Aim_Kick_Line.ino loop()
//    runDefender() = the body of Defender_Full.ino       loop()
//  Their one-time init goes in the matching setup function. Anything the two
//  share (motor pins, BNO055, IR/US/line/camera links) should be initialised
//  ONCE in setup() - see the README note on de-duplicating shared init.
// ---------------------------------------------------------------------------
void setupShared()   { /* motors, BNO055, IR/US/line/camera links, kicker pins */ }
void setupAttacker() { /* attacker-only init (if any) */ }
void setupDefender() { /* defender-only init (if any) */ }
void runAttacker()   { /* <- paste Attacker_Chase_Aim_Kick_Line.ino loop() body here */ }
void runDefender()   { /* <- paste Defender_Full.ino loop() body here */ }

// Optional: feed the referee GO/STOP so a base-attacker only resumes attacking at
// kick-off. Return your real comms state here (e.g. the USE_COMMS reader). If you
// leave this returning false forever, RobotLink falls back to a timed resume.
static bool refereeGo() {
  // return commsRunning();        // <- wire to your GO/STOP reader when available
  return false;
}

void setup() {
  Serial.begin(115200);
  RobotLink.begin();      // non-blocking; safe even if the nRF24 module is absent
  setupShared();
  setupAttacker();
  setupDefender();
}

void loop() {
  RobotLink.update();                 // non-blocking: receive, heartbeat, arbitrate
  RobotLink.setRefereeGo(refereeGo());

  if (RobotLink.role() == ROLE_ATTACKER) runAttacker();
  else                                   runDefender();   // also the boot/fail-safe role

  // (optional) heartbeat-light debug at a slow cadence:
  // static uint32_t t; if (millis() - t > 500) { t = millis();
  //   Serial.printf("role=%s partner=%s radio=%s\n",
  //     RobotLink.role()==ROLE_ATTACKER?"ATK":"DEF",
  //     RobotLink.partnerAlive()?"alive":"LOST",
  //     RobotLink.radioOk()?"ok":"absent"); }
}
