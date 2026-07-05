<p align="center">
  <img src="Docs/Images/VVS_Ballers_Logo.jpeg" alt="VVS Ballers" width="220">
</p>

<h1 align="center">VVS Ballers — RoboCupJunior Soccer (Internationals 2025-26)</h1>

<p align="center">
  Two fully autonomous robots — an <b>attacker</b> and a <b>defender</b> — for the
  <b>RoboCupJunior Soccer Infrared</b> (formerly Lightweight) league.<br>
  Firmware(C++) · multi-board electronics (KiCad) · mechanical design (Fusion 360).
</p>

<p align="center">
  <img src="Docs/Images/Robots/Both_Robots.jpg" alt="The two robots — one white, one green — facing off over the IR ball" width="760">
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
(See [How the robot works](Docs/HOW_IT_WORKS.md) for the full technical write-up.)

> **2026 ball note:** the Soccer Infrared league switched to the new 42 mm
> open-source [IR golf ball](https://github.com/robocup-junior/ir-golf-ball) — the
> same size as the Vision-league golf ball. The IR ring firmware here is tuned for it.

## Competition documents & media

| | |
|---|---|
| 📄 **Team Description Paper** | [`Docs/TDP.pdf`](Docs/TDP.pdf) |
| 🎬 **TDP video** | [Watch / download](https://github.com/Kushal-Sachdeva78/VVS-Ballers-RoboCup/releases/download/internationals-2025-26-media/VVS_Ballers_TDP_Video.mp4) (154 MB, hosted under [Releases](https://github.com/Kushal-Sachdeva78/VVS-Ballers-RoboCup/releases)) |
| 🖼️ **Poster** | [`Docs/RoboCup_Internationals_Poster.png`](Docs/RoboCup_Internationals_Poster.png) |
| 🎤 **Group-interview deck** | [`Docs/VVS_Ballers_GroupInterview_Deck.pptx`](Docs/VVS_Ballers_GroupInterview_Deck.pptx) |
| 📊 **Block diagram & BOM** | [`Docs/VVS_Ballers_Electronics_Block_Diagram.png`](Docs/VVS_Ballers_Electronics_Block_Diagram.png) · [`Docs/VVS_Ballers_BOM.xlsx`](Docs/VVS_Ballers_BOM.xlsx) |
| 📷 **Photo gallery** | [`Docs/Images/`](Docs/Images) — robots · electronics · mechanical · diagrams · team |

<p align="center">
  <a href="Docs/RoboCup_Internationals_Poster.png"><img src="Docs/RoboCup_Internationals_Poster.png" alt="RoboCup Internationals 2025-26 poster" width="340"></a>
</p>

## Team

| Member | Roles |
|---|---|
| **Kushal Sachdeva** (Team Leader) | Electrical, Software & CAD |
| **Darsh Goel** | Mechanical & CAD |

Region: **India** · League: **RoboCupJunior Soccer Infrared**

<p align="center">
  <img src="Docs/Images/Team/Team_Photo.jpg" alt="Team VVS Ballers" width="520">
</p>

## The robots at a glance

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

The robot is a **multi-board design**: each subsystem has its own microcontroller,
and the boards talk to the main board over compact **UART links** (CRC-8-framed, bar
the plain-ASCII IR link). This keeps
the fast sensor loops off the main control CPU.

| Board | MCU | Does | KiCad | Firmware |
|---|---|---|---|---|
| **Main** | Teensy 4.1 | Drive (4× DRV8263H), BNO055 IMU, capture sensor, kicker relay, all links | `PCB/Main` | `Firmware/Attacker`, `Firmware/Defender` |
| **IR ring** | Teensy 4.1 | 16× TSSP58038 → ball bearing & distance | `PCB/IR` | `Firmware/IR_Ball_Sensor` |
| **Line ring** | Teensy 4.1 | 4× QTR-MD-05A (18 ch) → white-line detection | `PCB/Line` | `Firmware/Line_Sensor` |
| **Ultrasonic** | Arduino Nano Every | 4× HC-SR04 → wall distances | `PCB/Ultrasonic` | `Firmware/Ultrasonic` |
| **Camera** | OpenMV H7 | Goal / keeper / open-corner vision | — | `Firmware/Camera` |
| **Power** | — | 12 V / 5 V / 3.3 V rails + 48 V solenoid boost | `PCB/Power` | — |
| **Motor-current** (optional) | Arduino Nano Every | DRV8263H stall / over-current watchdog | — | `Firmware/Motor_Current` |

Wiring for every board is in **[`PCB/Wiring/`](PCB/Wiring)** (one PDF per board) and
explained in [How the robot works](Docs/HOW_IT_WORKS.md). A whole-robot
[electronics block diagram](Docs/VVS_Ballers_Electronics_Block_Diagram.png) and the
full [bill of materials](Docs/VVS_Ballers_BOM.xlsx) live in `Docs/`.

<p align="center">
  <img src="Docs/Images/Electronics/Main_PCB_2.0.png" alt="Main PCB 2.0" width="19%">
  <img src="Docs/Images/Electronics/IR_PCB.png" alt="IR ring PCB" width="19%">
  <img src="Docs/Images/Electronics/Line_PCB.png" alt="Line ring PCB" width="19%">
  <img src="Docs/Images/Electronics/Power_PCB.png" alt="Power PCB" width="19%">
  <img src="Docs/Images/Electronics/Ultrasonic_PCB.png" alt="Ultrasonic PCB" width="19%">
</p>
<p align="center"><i>The five boards — Main 2.0 · IR ring · Line ring · Power · Ultrasonic
(KiCad renders; build photos in <a href="Docs/Images/Electronics">Docs/Images/Electronics</a>)</i></p>

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
├─ Docs/
│  ├─ HOW_IT_WORKS.md         ← full technical deep-dive
│  ├─ MAPPING.md              ← original → new file/folder mapping
│  ├─ TDP.pdf                 ← Team Description Paper
│  ├─ RoboCup_Internationals_Poster.png
│  ├─ VVS_Ballers_GroupInterview_Deck.pptx
│  ├─ VVS_Ballers_BOM.xlsx    ← bill of materials
│  ├─ VVS_Ballers_Electronics_Block_Diagram.png
│  └─ Images/                 ← gallery: Robots · Electronics · Mechanical · Diagrams · Team
├─ Firmware/                  ← all microcontroller code
│  ├─ Attacker/               ← main-board attacker (current: …_Line)
│  ├─ Defender/               ← main-board goalkeeper (Defender_Full)
│  ├─ IR_Ball_Sensor/         ← IR ring (Code2_Calibrated = flight build)
│  ├─ Line_Sensor/            ← QTR line ring (Detect / Baseline / Raw)
│  ├─ Ultrasonic/             ← Nano Every HC-SR04 board + main-board parser
│  ├─ Camera/                 ← OpenMV H7 goal vision (MicroPython)
│  ├─ Motor_Current/          ← optional over-current watchdog
│  └─ README.md
├─ PCB/                       ← KiCad sources, Gerbers, wiring PDFs, fab zips
│  ├─ Main/ IR/ Line/ Ultrasonic/ Power/
│  ├─ Wiring/        ← human-readable wiring per board (PDF)
│  ├─ Fabrication/   ← ready-to-order Gerber zips
│  └─ README.md
├─ CAD/                       ← Fusion 360 sources + printable STLs
│  ├─ Fusion/   Printable/{Chassis, Drive, Ball_Capture, Sensor_Mounts}
│  └─ README.md
└─ Legacy/                    ← superseded designs, kept for history (see Legacy/README.md)
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
| Camera | OpenMV H7 | save `Goal_Cam.py` to the H7 flash as `main.py` |

PlatformIO example: `pio run -e teensy41 -t upload` (Teensy) or
`pio run -e nano_every -t upload` (Nano).

## Inter-board links (summary)

Three of the four links are short binary frames with a 2-byte sync and an identical
**CRC-8 (poly 0x07)**, so the main board can reject any corrupted frame; the IR link
is plain delimited ASCII:

- **IR → Main** (Serial4): ASCII `"<dir>a … <dist>b"`, `500` = no ball (no checksum).
- **Ultrasonic → Main** (Serial3): 13-byte frame, four `uint16` mm distances + status.
- **Line → Main** (Serial8): 9-byte frame, per-side white-line bitmask + counts.
- **Camera → Main** (Serial2): 11-byte frame, goal/keeper/ball flags + bearings,
  open-corner angle, and the orange-ball bearing/distance (camera+IR fusion).

Full byte-level layouts are in [How the robot works](Docs/HOW_IT_WORKS.md).

## Sponsors

Our thanks to **Amazon · DLF · Havells · OpenMV · Magnum Ventures · Budtree
Management** for supporting the team.

## License

Released under the [MIT License](LICENSE) © 2026 VVS Ballers. You're welcome to
learn from and build on this work — please keep the attribution.
