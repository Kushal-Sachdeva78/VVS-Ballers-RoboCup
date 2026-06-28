# File & folder mapping (original → repo)

This repository is a **reorganised copy** of the original working folder
`RCJ Main 2026/RoboCup Internationals 2025-26` (plus two assets from just outside
it). The originals were never modified. This document records where everything went
and how it was renamed.

## Naming convention

- **Category folders** use readable **Title case**, with acronyms in caps:
  `Firmware`, `PCB`, `CAD`, `Docs`, `Legacy`, and within them `IR`, `Line`, `Main`,
  `Power`, `Ultrasonic`, `Fusion`, `Printable`, `Wiring`, `Fabrication`,
  `IR_Ball_Sensor`, `Line_Sensor`, `Motor_Current`, …
- **Sketch folders / KiCad projects** keep their base name (a sketch folder must
  match its `.ino`, and a KiCad project's `.kicad_pcb/.sch/.pro` must share one base
  name). Spaces became underscores throughout, so there are **no spaces anywhere**.

## Top-level

| Original | Repo |
|---|---|
| `…/Codes/` | `Firmware/` |
| `…/PCB/PCB Main/` | `PCB/` |
| `PCB/PCB Wiring/` *(outside base)* | `PCB/Wiring/` |
| `…/PCB/Zip PCB/` | `PCB/Fabrication/` |
| `…/CAD/` | `CAD/` |
| `VVS Ballers new logo.jpeg` *(outside base)* | `Docs/Images/VVS_Ballers_Logo.jpeg` |

## Firmware

| Original (`Codes/…`) | Repo (`Firmware/…`) |
|---|---|
| `Attacker/attacker_chase_aim_kick_line` | `Attacker/Attacker_Chase_Aim_Kick_Line` |
| `Defender/Defender_Full` | `Defender/Defender_Full` |
| `IR Code/Code2_Calibrated` | `IR_Ball_Sensor/Code2_Calibrated` |
| `IR Code/Code2_Refined` | `IR_Ball_Sensor/Code2_Refined` |
| `IR Code/Calibration` | `IR_Ball_Sensor/Calibration` |
| `Firmware Ultrasonic/Ultrasonic_NanoEvery` | `Ultrasonic/Ultrasonic_NanoEvery` |
| `Firmware Ultrasonic/Ultrasonic_Receiver_MainBoard` | `Ultrasonic/Ultrasonic_Receiver_MainBoard` |
| `Firmware Ultrasonic/README.md` | `Ultrasonic/README_ultrasonic.md` |
| `Line PCB Codes/line_pcb_detect` | `Line_Sensor/Line_PCB_Detect` |
| `Line PCB Codes/line_pcb_baseline` | `Line_Sensor/Line_PCB_Baseline` |
| `Line PCB Codes/line_pcb_raw_reader` | `Line_Sensor/Line_PCB_Raw_Reader` |
| `Camera Code/goal_cam.py` | `Camera/Goal_Cam.py` |
| `Camera Code/goal_cam_ball.py` | `Camera/Goal_Cam_Ball.py` |
| `Motor Current Control Code/motor_current_supervisor` | `Motor_Current/Motor_Current_Supervisor` |
| `Motor Current Control Code/motor_current_limit_1A` | `Motor_Current/Motor_Current_Limit_1A` |

## PCB

| Original (`PCB/PCB Main/…`) | Repo (`PCB/…`) |
|---|---|
| `Main_PCB 2.0` | `Main` (files → `Main_PCB_2.0.*`) |
| `IR_PCB` | `IR` |
| `Line_PCB` | `Line` |
| `Ultrasonic_PCB` | `Ultrasonic` |
| `Power PCB` | `Power` (files → `Power_PCB.*`) |

## CAD

| Original (`CAD/…`) | Repo (`CAD/…`) |
|---|---|
| `Fusion 1.0/` | `Fusion/` (files de-spaced, e.g. `Main Robot.f3z` → `Main_Robot.f3z`) |
| `Print/Print Round 1..3/` | `Printable/` — re-grouped **by subsystem** (`Chassis/`, `Drive/`, `Ball_Capture/`, `Sensor_Mounts/`) rather than by print round |

## Legacy (archived, superseded)

| Original | Repo (`Legacy/…`) |
|---|---|
| `Codes/Attacker/attacker_chase_aim_kick` | `Firmware/Attacker/Attacker_Chase_Aim_Kick` |
| `Codes/Attacker/ir_ball_chase_avoid_angle` | `Firmware/Attacker/IR_Ball_Chase_Avoid_Angle` |
| `Codes/Defender/Defender_NoLine` | `Firmware/Defender/Defender_NoLine` |
| `PCB/PCB Main/Main_PCB` | `PCB/Main_v1` |
| `CAD/Old Models/` | `CAD/Old_Models/` |
| `CAD/Not Using/` | `CAD/Unused/` |

## What was intentionally left out

Build/cache/version-control noise was not copied (it is regenerated, and is
`.gitignore`d): PlatformIO `.pio/` build trees, KiCad `*-backups/`, `.history/`
stores, `fp-info-cache`, `_autosave-*`, editor `.vscode/`, and Fusion script `out/`
directories.

The wider top-level working tree (`Codes/{In Progress, Motor Overheat, RCJ Codes
Refined, Test Codes}`, and the duplicate top-level `CAD/` and `PCB/`) was **not**
copied. Those originals remain untouched in their original location.
