<p align="center">
  <img src="RoboCup%20Internationals%202025-26/Docs/Images/VVS_Ballers_Logo.jpeg" alt="VVS Ballers" width="220">
</p>

<h1 align="center">VVS Ballers — RoboCupJunior Soccer</h1>

<p align="center">
  Autonomous RoboCupJunior Soccer robots by Team VVS Ballers,
  designed and iterated across multiple seasons — from EV3 regional robots to a
  custom Teensy-based international platform.<br>
  Firmware (C++) · multi-board electronics (KiCad) · mechanical design (Fusion 360).
</p>

<p align="center">
  <img src="RoboCup%20Internationals%202025-26/Docs/Images/Robots/Both_Robots.jpg" alt="The two Internationals 2025-26 robots with the IR ball" width="760">
</p>

---

## Seasons

| Season | What's inside |
|---|---|
| **[RoboCup Internationals 2025-26](RoboCup%20Internationals%202025-26/)** ⭐ *current* | Two autonomous Soccer-Infrared robots (attacker + defender): a Teensy 4.1 multi-board stack — IR ball ring, line ring, ultrasonic, OpenMV goal camera, 48 V solenoid kicker. Firmware + KiCad PCBs + Fusion CAD. |
| **[RoboCup Nationals 2025-26](RoboCup%20Nationals%202025-26/)** | Nationals build: custom IR board (KiCad) + updated IR firmware for the Teensy stack. |
| **[RoboCup Regionals 2025-26](RoboCup%20Regionals%202025-26/)** | EV3 holonomic robot (diamond omni drive, dual underside colour sensors, side IR distance, IR seeker, compass heading hold) + an Arduino Nano Every ultrasonic kicker + Fusion/PETG parts + poster. |
| **[RoboCup 2024-25](RoboCup%202024-25/)** | Prior season: international Team Description Paper, posters, and earlier EV3/Arduino reference designs. |

> The **Internationals 2025-26** robots are the current flagship — the rest of this
> page focuses on them. The full detail lives in that folder's
> **[detailed README](RoboCup%20Internationals%202025-26/README.md)** and a complete
> **[technical deep-dive](RoboCup%20Internationals%202025-26/Docs/HOW_IT_WORKS.md)**.
> Competition media: **[TDP](RoboCup%20Internationals%202025-26/Docs/TDP.pdf)** ·
> **[poster](RoboCup%20Internationals%202025-26/Docs/RoboCup_Internationals_Poster.png)** ·
> **[TDP video](https://github.com/Kushal-Sachdeva78/VVS-Ballers-RoboCup/releases/download/internationals-2025-26-media/VVS_Ballers_TDP_Video.mp4)** ·
> **[photo gallery](RoboCup%20Internationals%202025-26/Docs/Images)**.
>
> Code-span paths below (e.g. `Firmware/Attacker`) are relative to
> [`RoboCup Internationals 2025-26/`](RoboCup%20Internationals%202025-26/).

### Earlier-season highlights

- **Regionals 2025-26 (EV3):** 4× EV3 Medium Motors driving aluminium omni wheels in
  a **diamond** layout; two underside colour sensors for white-line / goal-area
  avoidance; two side IR sensors for walls; an IR seeker for 10-zone ball tracking;
  and a compass for heading correction. An external **Arduino Nano Every** kicker
  (TBFNG H-bridge + Pololu 20D gearmotor, ultrasonic ball detect) runs an
  IDLE → KICK → HOLD → RETRACT → COOLDOWN state machine.
- **Nationals 2025-26 direction:** moving to a custom KiCad electronics stack and a
  Teensy main controller with refined sensing — the foundation the Internationals
  robots are built on.

---

## Internationals 2025-26 — the robots at a glance

Both robots share one electronics stack and one chassis; only the role firmware on
the main board differs.

| | Attacker | Defender (goalkeeper) |
|---|---|---|
| Job | Win the ball, aim, and shoot the open corner | Hold the goal mouth and clear the ball |
| Main firmware | `Firmware/Attacker/Attacker_Chase_Aim_Kick_Line` | `Firmware/Defender/Defender_Full` |
| Ball | IR ring (16 sensors) | IR ring |
| Staying in bounds | **Line ring + ultrasonic fused** boundary escape | **Line** keeps it on the goal-area ("D") line; ultrasonic cross-check |
| Goal / aim | OpenMV camera → open-corner bearing | (camera optional) |
| Kicker | 48 V solenoid, camera-gated | Short clearing kick |

## Electronics — the boards

A **multi-board design**: each subsystem has its own microcontroller, and the boards
talk to the main board over compact **UART links** (CRC-8-framed, bar the plain-ASCII
IR link), keeping the fast sensor loops
off the main control CPU.

| Board | MCU | Does | KiCad | Firmware |
|---|---|---|---|---|
| **Main** | Teensy 4.1 | Drive (4× DRV8263H), BNO055 IMU, capture sensor, kicker relay, all links | `PCB/Main` | `Firmware/Attacker`, `Firmware/Defender` |
| **IR ring** | Teensy 4.1 | 16× TSSP58038 → ball bearing & distance | `PCB/IR` | `Firmware/IR_Ball_Sensor` |
| **Line ring** | Teensy 4.1 | 4× QTR-MD-05A (18 ch) → white-line detection | `PCB/Line` | `Firmware/Line_Sensor` |
| **Ultrasonic** | Arduino Nano Every | 4× HC-SR04 → wall distances | `PCB/Ultrasonic` | `Firmware/Ultrasonic` |
| **Camera** | OpenMV H7 | Goal / keeper / open-corner vision | — | `Firmware/Camera` |
| **Power** | — | 12 V / 5 V / 3.3 V rails + 48 V solenoid boost | `PCB/Power` | — |
| **Motor-current** (optional) | Arduino Nano Every | DRV8263H stall / over-current watchdog | — | `Firmware/Motor_Current` |

Human-readable wiring for every board is in
**[`PCB/Wiring/`](RoboCup%20Internationals%202025-26/PCB/Wiring)** (one PDF per board).

## How it plays — design choices vs. the game

Everything is built around what actually wins (and loses) a RoboCupJunior Soccer
match. (A goal counts when the ball touches the **back wall of the goal**; touching a
wall or driving fully into the marked penalty area is an **out-of-bounds penalty**.)

- **Score in the goal.** The IR ring gives the ball's bearing; the main board chases
  it with an omni-wheel drive held straight by a BNO055 heading PID. Once the ball is
  captured, the OpenMV camera finds the goal, picks the **open corner** (dodging the
  keeper), and the chassis turns in place until that corner is dead ahead before the
  solenoid fires. Aiming the open corner instead of the goal centre is how you beat a
  keeper.

- **Never cross the line (the expensive mistake).** The attacker treats a side as the
  field boundary **only when the line ring and the ultrasonics agree** (white line
  seen *and* wall within 300 mm), then flees away from it. Two independent sensors
  must concur, so one noisy reading can't make the robot dart out of bounds.

- **Keep the keeper on its line, not inside the box.** No robot may be *fully* inside
  the penalty area, so the defender uses the line ring to ride the **edge of the goal
  area ("the D")** while tracking the ball laterally and clearing it on contact.

- **Stay legal in the Infrared league.** IR-emitting rangefinders (ToF, LiDAR, IR
  distance sensors) are **banned** because they'd swamp other robots' IR ball
  sensors — which is why walls are ranged with **ultrasonic** HC-SR04s here, not IR.

- **Power within the rules.** The rules cap robot power at **48 V DC**; the solenoid
  kicker runs from a 48 V boost converter — right at the legal ceiling for the hardest
  legal kick — while logic and motors run from 3.3 V / 5 V / 12 V rails.

- **Referee start/stop.** The main board reads the RoboCupJunior **Communication
  Module** GO/STOP lines, so a referee can start and halt the robot as required.

The binary inter-board links (ultrasonic, line, camera → main) are short frames, each
guarded by an identical **CRC-8 (poly 0x07)**; the IR link is plain delimited ASCII.
Full byte-level layouts are in the
[technical deep-dive](RoboCup%20Internationals%202025-26/Docs/HOW_IT_WORKS.md).

## Team

| Member | Roles |
|---|---|
| **Kushal Sachdeva** (Team Leader) | Electrical, Software & CAD |
| **Darsh Goel** | Mechanical & CAD |

Region: **India** · League: **RoboCupJunior Soccer Infrared**

## Sponsors

Our thanks to **Amazon · DLF · Havells · OpenMV · Magnum Ventures · Budtree
Management**, and to our mentors, school, and the RoboCup Junior community for their
support.

## License

Released under the **MIT License** — see [`LICENSE`](LICENSE).
© 2026 VVS Ballers (Kushal Sachdeva, Darsh Goel). You're welcome to learn from and
build on this work; please keep the attribution.
