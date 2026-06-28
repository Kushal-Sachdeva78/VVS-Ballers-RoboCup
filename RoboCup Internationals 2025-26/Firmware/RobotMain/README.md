# RobotMain — combined firmware (one build, either role)

The single firmware that makes inter-robot role switching real: it runs **both**
behaviours and lets [`RobotLink`](../Comms/RobotLink) pick which one each loop.

> partner alive → play your base role · partner lost → **cover the goal as keeper**

Flash the **same** sketch to both robots; they differ only by two `#define`s.

## Files

| File | What it is |
|---|---|
| `RobotMain.ino` | Thin dispatcher: `RobotLink.update()` → run `attackerLoop()` or `defenderLoop()`, and safe-stop the outgoing role on a switch. |
| `Roles.h` | Declares the role entry points. |
| `AttackerRole.cpp` | The attacker body, **generated** from `../Attacker/Attacker_Chase_Aim_Kick_Line`, wrapped in an anonymous namespace. |
| `DefenderRole.cpp` | The defender body, **generated** from `../Defender/Defender_Full` (its `config.h` inlined). |
| `RobotLink.h` / `.cpp` | A bundled copy of the radio module (canonical source: `../Comms/RobotLink`). |

The two `*Role.cpp` are **generated copies** — they're wrapped in anonymous
namespaces so the two sketches' identical globals (`setMotor`, `us_crc8`, …) and
clashing `#define`s (`CAPTURE_BALL_MM` 45 vs 50, the two kicker tables) stay
isolated and link cleanly. Edit the **canonical** Attacker/Defender sketches and
regenerate these, not the copies.

## Set up per robot

1. In **`RobotLink.h`** set, on each robot:
   ```c
   #define MY_ROBOT_ID   0               // 0 on one robot, 1 on the other
   #define MY_BASE_ROLE  ROLE_ATTACKER   // ATTACKER on one, DEFENDER on the other
   ```
2. Install the **RF24** library (TMRh20). Wire the nRF24L01+ on **SPI1** (CE/CSN
   per `RobotLink.h`, all `>>> BENCH-VERIFY`'d) with a 10 µF cap across its VCC/GND.
3. Build for **Teensy 4.1** and flash the same sketch to both robots.

> **Compile once in the Arduino IDE before flashing.** This combined build is new —
> open `RobotMain.ino`, verify it builds, and run the
> [bench test](../Comms/README.md#7-bench-test-procedure). Your standalone
> `Attacker_Chase_Aim_Kick_Line` and `Defender_Full` sketches are untouched and
> remain the per-role builds.

## How dispatch works

- `setup()` initialises **both** role units once (they share one chassis; only the
  active role drives each pass).
- Each loop: `RobotLink.update()` arbitrates the role; the dispatcher runs that
  role's loop. On a role change it calls the **outgoing** role's `…SafeStop()`
  (motors off + solenoid released) so nothing is left latched across the switch.
- No nRF24 fitted, or it fails to init → each robot plays its static `MY_BASE_ROLE`
  (graceful degradation; the robot never stalls).

See [`../Comms/README.md`](../Comms/README.md) for the link protocol, arbitration
rules, fail-safe details, and the bench-test procedure.
