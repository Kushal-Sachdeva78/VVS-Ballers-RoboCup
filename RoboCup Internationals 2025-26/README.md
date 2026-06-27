<p align="center">
  <img src="docs/images/vvs-ballers-logo.jpeg" alt="VVS Ballers" width="220">
</p>

<h1 align="center">VVS Ballers — RoboCupJunior Soccer (Internationals 2025-26)</h1>

<p align="center">
  Two fully autonomous robots — an <b>attacker</b> and a <b>defender</b> — for the
  <b>RoboCupJunior Soccer Infrared</b> (formerly Lightweight) league.<br>
  Firmware · multi-board electronics (KiCad) · mechanical design (Fusion 360).
</p>

---

## About

This folder is the **Internationals 2025-26** season of the VVS Ballers
RoboCupJunior Soccer robots — the firmware that runs them, the PCBs that make up
their electronics, and the CAD for their chassis. The robots chase an infrared-emitting ball, find the goal with a
camera, kick with a solenoid, and stay inside the field using a line-sensor ring
backed up by ultrasonic ranging.

In **RoboCupJunior Soccer**, two teams of two robots play autonomous soccer on a
green field with white boundary lines. A goal is scored when the ball touches the
**back wall of the goal**; touching a wall or driving fully into the marked
penalty area is an **out-of-bounds penalty**. Every design decision here
is made against those facts — score in the goal, never cross the line, stay legal.
(See [How the robot works](docs/HOW_IT_WORKS.md) for the full technical write-up.)

> **2026 ball note:** the Soccer Infrared league switched to the new 42 mm
> open-source [IR golf ball](https://github.com/robocup-junior/ir-golf-ball) — the
> same size as the Vision-league golf ball. The IR ring firmware here is tuned for it.

## Team

| Member | Roles |
|---|---|
| **Kushal Sachdeva** (Team Leader) | Electrical, Software & CAD |
| **Darsh Goel** | Mechanical & CAD |

Region: **India** · League: **RoboCupJunior Soccer Infrared**

## The robots at a glance

Both robots share one electronics stack and one chassis; only the role firmware on
the main board differs.

| | Attacker | Defender (goalkeeper) |
|---|---|---|
| Job | Win the ball, aim, and shoot the open corner | Hold the goal mouth and clear the ball |
| Main firmware | `firmware/attacker/attacker_chase_aim_kick_line` | `firmware/defender/Defender_Full` |
| Ball | IR ring (16 sensors) | IR ring |
| Staying in bounds | **Line ring + ultrasonic fused** boundary escape | **Line** keeps it on the goal-area ("D") line; ultrasonic cross-check |
| Goal / aim | OpenMV camera → open-corner bearing | (camera optional) |
| Kicker | 48 V solenoid, camera-gated | Short clearing kick |

## Electronics — the boards

The robot is a **multi-board design**: each subsystem has its own microcontroller,
and the boards talk to the main board over **CRC-checked UART links**. This keeps
the fast sensor loops off the main control CPU.

| Board | MCU | Does | KiCad | Firmware |
|---|---|---|---|---|
| **Main** | Teensy 4.1 | Drive (4× DRV8263H), BNO055 IMU, capture sensor, kicker relay, all links | `pcb/main` | `firmware/attacker`, `firmware/defender` |
| **IR ring** | Teensy 4.1 | 16× TSSP58038 → ball bearing & distance | `pcb/ir` | `firmware/ir-ball-sensor` |
| **Line ring** | Teensy 4.1 | 4× QTR-MD-05A (18 ch) → white-line detection | `pcb/line` | `firmware/line-sensor` |
| **Ultrasonic** | Arduino Nano Every | 4× HC-SR04 → wall distances | `pcb/ultrasonic` | `firmware/ultrasonic` |
| **Camera** | OpenMV H7 | Goal / keeper / open-corner vision | — | `firmware/camera` |
| **Power** | — | 12 V / 5 V / 3.3 V rails + 48 V solenoid boost | `pcb/power` | — |
| **Motor-current** (optional) | Arduino Nano Every | DRV8263H stall / over-current watchdog | — | `firmware/motor-current` |

Wiring for every board is in **[`pcb/wiring/`](pcb/wiring)** (one PDF per board) and
explained in [How the robot works](docs/HOW_IT_WORKS.md).

## How it plays — design choices vs. the game

Everything here is built around what actually wins (and loses) a RoboCupJunior
Soccer match:

- **Score in the goal.** The IR ring gives the ball's bearing; the main board chases
  it with an omni-wheel drive held straight by a BNO055 heading PID. Once the ball
  is captured, the OpenMV camera finds the goal, picks the **open corner** (dodging
  the keeper), and the chassis turns in place until that corner is dead ahead before
  the solenoid fires. Aiming the open corner instead of the goal centre is how you
  beat a keeper.

- **Never cross the line (the expensive mistake).** Touching a wall *or* driving
  fully into the marked penalty area is an **out-of-bounds penalty** — the robot is
  removed for a minute, and any goal scored while it's penalised is wiped. The
  attacker therefore treats a side as the field boundary **only when the line ring
  and the ultrasonics agree** (white line seen *and* wall within 300 mm), then flees
  away from it. Two independent sensors must concur, so one noisy reading can't make
  the robot dart out of bounds.

- **Keep the keeper on its line, not inside the box.** No robot may be *fully* inside
  the penalty area (that's out of bounds), so the defender uses the line ring to ride
  the **edge of the goal area ("the D")** — staying on the line rather than crossing
  it — while tracking the ball laterally and clearing it on contact.

- **Stay legal in the Infrared league.** IR-emitting rangefinders (ToF, LiDAR, IR
  distance sensors) are **banned** in the Infrared league because they'd swamp other
  robots' IR ball sensors — which is exactly why walls are ranged with **ultrasonic**
  HC-SR04s here, not IR. The reflectance line sensors face **down at the floor** and
  are shrouded so their light can't be mistaken for the orange ball by other teams'
  cameras.

- **Power within the rules.** The rules cap robot power at **48 V DC**; the solenoid
  kicker runs from a 48 V boost converter — right at the legal ceiling for the
  hardest legal kick — while logic and motors run from 3.3 V / 5 V / 12 V rails.

- **Referee start/stop.** The main board reads the RoboCupJunior **Communication
  Module** GO/STOP lines, so a referee can start and halt the robot as the rules
  require at the international competition.

## Folder structure

```
RoboCup Internationals 2025-26/
├─ README.md                  ← you are here
├─ docs/
│  ├─ HOW_IT_WORKS.md         ← full technical deep-dive
│  ├─ MAPPING.md              ← original → new file/folder mapping
│  └─ images/
├─ firmware/                  ← all microcontroller code
│  ├─ attacker/               ← main-board attacker (current: …_line)
│  ├─ defender/               ← main-board goalkeeper (Defender_Full)
│  ├─ ir-ball-sensor/         ← IR ring (Code2_Calibrated = flight build)
│  ├─ line-sensor/            ← QTR line ring (detect / baseline / raw)
│  ├─ ultrasonic/             ← Nano Every HC-SR04 board + main-board parser
│  ├─ camera/                 ← OpenMV H7 goal vision (MicroPython)
│  ├─ motor-current/          ← optional over-current watchdog
│  └─ README.md
├─ pcb/                       ← KiCad sources, Gerbers, wiring PDFs, fab zips
│  ├─ main/ ir/ line/ ultrasonic/ power/
│  ├─ wiring/   ← human-readable wiring per board (PDF)
│  ├─ fab/      ← ready-to-order Gerber zips
│  └─ README.md
├─ cad/                       ← Fusion 360 sources + printable STLs
│  ├─ fusion/  print/   └─ README.md
└─ legacy/                    ← superseded designs, kept for history (see legacy/README.md)
```

## Build & flash

All boards build in the **Arduino IDE** (each sketch folder is named to match its
`.ino`); the Teensy/Nano sketches also build under **PlatformIO** via the
`platformio.ini` in each folder.

| Target | Board setting | Libraries / notes |
|---|---|---|
| Main board (attacker/defender) | Teensy 4.1, USB Type *Serial* | Adafruit BNO055 + Adafruit Unified Sensor (Defender OLED option also needs SSD1306 + GFX) |
| IR ring | Teensy 4.1 | none (core `analogRead`) — flash `Code2_Calibrated` |
| Line ring | Teensy 4.1 | none |
| Ultrasonic | Arduino Nano Every (megaAVR) | none; *Registers emulation → None (ATMEGA4809)* |
| Motor-current | Arduino Nano Every | none |
| Camera | OpenMV H7 | save `goal_cam.py` to the H7 flash as `main.py` |

PlatformIO example: `pio run -e teensy41 -t upload` (Teensy) or
`pio run -e nano_every -t upload` (Nano).

## Inter-board links (summary)

Three independent UART links, each a short binary frame with a 2-byte sync and an
identical **CRC-8 (poly 0x07)**, so the main board can reject any corrupted frame:

- **IR → Main** (Serial4): ASCII `"<dir>a … <dist>b"`, `500` = no ball.
- **Ultrasonic → Main** (Serial3): 13-byte frame, four `uint16` mm distances + status.
- **Line → Main** (Serial8): 9-byte frame, per-side white-line bitmask + counts.
- **Camera → Main** (Serial2): 9-byte frame, goal/keeper flags + bearings + open-corner angle.

Full byte-level layouts are in [How the robot works](docs/HOW_IT_WORKS.md).

## Sponsors

Our thanks to **Amazon · DLF · Havells · OpenMV · Magnum Ventures · Budtree
Management** for supporting the team.

## Credits & references

- **Rules:** [RoboCupJunior Soccer Rules 2026](https://robocup-junior.github.io/soccer-rules/master/rules.html)
  and [scoring / award rubrics](https://robocup-junior.github.io/soccer-rules/master/scoring.html).
- **IR ball:** [robocup-junior/ir-golf-ball](https://github.com/robocup-junior/ir-golf-ball).
- **Communication module:** [robocup-junior/soccer-communication-module](https://github.com/robocup-junior/soccer-communication-module).
- The IR-ring algorithm and camera vision build on ideas openly shared by other RCJ
  teams (Aegis, Crestwood Lions, Hyperion, AIR, chaBots, Reset); see the in-code
  headers for per-file attribution, as the rules require.

## License

Released under the [MIT License](LICENSE) © 2026 VVS Ballers. You're welcome to
learn from and build on this work — please keep the attribution.
