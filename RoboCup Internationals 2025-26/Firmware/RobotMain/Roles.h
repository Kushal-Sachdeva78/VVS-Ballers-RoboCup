#ifndef ROLES_H
#define ROLES_H
// ============================================================================
//  Role behaviour entry points for the combined build (RobotMain).
//  The bodies live in AttackerRole.cpp / DefenderRole.cpp, each generated from
//  the canonical Attacker / Defender sketches and wrapped in an anonymous
//  namespace so their (identical) globals and (clashing) #defines stay isolated.
//  Only the ACTIVE role's *Loop() runs each pass; *SafeStop() stops that role's
//  motors and releases the solenoid when the arbitrated role changes.
// ============================================================================
void attackerSetup();   void attackerLoop();   void attackerSafeStop();
void defenderSetup();   void defenderLoop();   void defenderSafeStop();
#endif // ROLES_H
