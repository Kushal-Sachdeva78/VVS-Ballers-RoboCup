# File & folder mapping (original → repo)

This repository is a **reorganised copy** of the original working folder
`RCJ Main 2026/RoboCup Internationals 2025-26` (plus two assets from just outside
it). The originals were never modified. This document records where everything went
and how it was renamed, so nothing is lost.

## Naming convention

- **New top-level / category folders:** lowercase `kebab-case`
  (`firmware`, `pcb`, `cad`, `docs`, `legacy`, `ir-ball-sensor`, `line-sensor`,
  `motor-current`, `main`, `wiring`, `fab`, …).
- **Preserved names (sketch folders, KiCad projects, asset files):** kept as-is with
  **spaces → underscores** only. Arduino requires a sketch folder to match its
  `.ino`, and a KiCad project's `.kicad_pcb/.sch/.pro` must share one base name, so
  these names were not otherwise changed.
- Result: **no spaces anywhere** in the repo.

## Top-level

| Original | Repo |
|---|---|
| `RoboCup Internationals 2025-26/Codes/` | `firmware/` |
| `RoboCup Internationals 2025-26/PCB/PCB Main/` | `pcb/` |
| `PCB/PCB Wiring/` *(outside base)* | `pcb/wiring/` |
| `RoboCup Internationals 2025-26/PCB/Zip PCB/` | `pcb/fab/` |
| `RoboCup Internationals 2025-26/CAD/` | `cad/` |
| `VVS Ballers new logo.jpeg` *(outside base)* | `docs/images/vvs-ballers-logo.jpeg` |

## Firmware

| Original (`Codes/…`) | Repo (`firmware/…`) |
|---|---|
| `Attacker/attacker_chase_aim_kick_line` | `attacker/attacker_chase_aim_kick_line` |
| `Defender/Defender_Full` | `defender/Defender_Full` |
| `IR Code/Code2_Calibrated` | `ir-ball-sensor/Code2_Calibrated` |
| `IR Code/Code2_Refined` | `ir-ball-sensor/Code2_Refined` |
| `IR Code/Calibration` | `ir-ball-sensor/Calibration` |
| `Firmware Ultrasonic/Ultrasonic_NanoEvery` | `ultrasonic/Ultrasonic_NanoEvery` |
| `Firmware Ultrasonic/Ultrasonic_Receiver_MainBoard` | `ultrasonic/Ultrasonic_Receiver_MainBoard` |
| `Firmware Ultrasonic/README.md` | `ultrasonic/README_ultrasonic.md` |
| `Line PCB Codes/line_pcb_detect` | `line-sensor/line_pcb_detect` |
| `Line PCB Codes/line_pcb_baseline` | `line-sensor/line_pcb_baseline` |
| `Line PCB Codes/line_pcb_raw_reader` | `line-sensor/line_pcb_raw_reader` |
| `Camera Code/goal_cam.py` | `camera/goal_cam.py` |
| `Camera Code/goal_cam_ball.py` | `camera/goal_cam_ball.py` |
| `Motor Current Control Code/motor_current_supervisor` | `motor-current/motor_current_supervisor` |
| `Motor Current Control Code/motor_current_limit_1A` | `motor-current/motor_current_limit_1A` |

## PCB

| Original (`PCB/PCB Main/…`) | Repo (`pcb/…`) |
|---|---|
| `Main_PCB 2.0` | `main` (files → `Main_PCB_2.0.*`) |
| `IR_PCB` | `ir` |
| `Line_PCB` | `line` |
| `Ultrasonic_PCB` | `ultrasonic` |
| `Power PCB` | `power` (files → `Power_PCB.*`) |

## CAD

| Original (`CAD/…`) | Repo (`cad/…`) |
|---|---|
| `Fusion 1.0/` | `fusion/` (files de-spaced, e.g. `Main Robot.f3z` → `Main_Robot.f3z`) |
| `Print/Print Round 1..3/` | `print/Print_Round_1..3/` |

## Legacy (archived, superseded)

| Original | Repo (`legacy/…`) |
|---|---|
| `Codes/Attacker/attacker_chase_aim_kick` | `firmware/attacker/attacker_chase_aim_kick` |
| `Codes/Attacker/ir_ball_chase_avoid_angle` | `firmware/attacker/ir_ball_chase_avoid_angle` |
| `Codes/Defender/Defender_NoLine` | `firmware/defender/Defender_NoLine` |
| `PCB/PCB Main/Main_PCB` | `pcb/main-v1` |
| `CAD/Old Models/` | `cad/old-models/` |
| `CAD/Not Using/` | `cad/not-using/` |

## What was intentionally left out

Build/cache/version-control noise was not copied (it is regenerated, and is
`.gitignore`d): PlatformIO `.pio/` build trees, KiCad `*-backups/`, `.history/`
(local-history `.git` stores), `fp-info-cache`, `_autosave-*`, editor `.vscode/`,
and Fusion script `out/` directories.

The wider top-level working tree (`Codes/{In Progress, Motor Overheat, RCJ Codes
Refined, Test Codes}`, and the duplicate top-level `CAD/` and `PCB/`) was **not**
copied — the curated `RoboCup Internationals 2025-26` folder already supersedes it.
Those originals remain untouched in their original location.
