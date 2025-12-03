# VVS Ballers RoboCup Projects

## About

VVS Ballers design, build, and iterate RoboCup Junior Soccer robots across multiple seasons.  
This repository collects our full-season artifacts: CAD, EV3 programs, Arduino/Teensy firmware, electronics designs, and posters for regionals, nationals, and international events.

We focus on:
- **Holonomic EV3 robots** with omni wheels and compass-based heading control  
- **External kicker subsystems** using Arduino, ultrasonic sensing, and Pololu gearmotors  
- **Custom electronics** (IR boards, power distribution, control) for our newer Teensy-based platforms  

---

## Repository structure

> Folder names may differ slightly from this summary – check each season folder for exact filenames.

- **`RoboCup Regionals 2025-26/`**  
  EV3-based holonomic regional robot:
  - `RoboCup Regionals Robot Code.ev3`  
    - LEGO Mindstorms Education EV3 program for:
      - 4× Medium Motor holonomic drive (diamond omni wheel layout)  
      - Dual underside colour sensors (front & back) for white-line / goal-area detection  
      - Left/right IR distance sensors for wall avoidance  
      - IR seeker for 10-zone ball tracking  
      - Compass-based heading correction before each movement  
    - Behaviour:
      - If only front colour sensor sees white → move back (line ahead, avoid goal D)  
      - If only back colour sensor sees white → move forward (line behind)  
      - If both see white → use IR distances to escape diagonally away from danger  
      - If no line: first avoid walls with side IR, then chase the ball with the IR seeker  
  - `Robocup_Regionals_Kicker_Code.ino`  
    - Arduino Nano Every kicker firmware:
      - Ultrasonic sensor to detect the ball in the capture area  
      - TBFNG H-bridge driving a Pololu 20D metal gearmotor (450 rpm, 2.4 kg·cm torque) from a 12 V battery  
      - State machine handling **IDLE → KICK → HOLD → RETRACT → COOLDOWN** with tunable speeds/timings  
  - `RoboCup Regionals 3D Models/`  
    - Fusion 360–designed and PETG 3D-printed parts:
      - Kicker mechanism  
      - Mounts, covers, and brackets for sensors and electronics  
  - `RoboCup Regionals Poster/` (PDF/PNG/MD)  
    - Regional poster summarizing robot concept, hardware, software, and results.

- **`RoboCup Nationals 2025-26/`**  
  In-progress nationals robot:
  - `IR_board/` (KiCad project)  
    - Custom IR board schematic and PCB  
    - STEP/DXF exports for mechanical integration
  - `Nationals_IR_Code.ino`  
    - Updated IR firmware targeting a Teensy-based controller stack  
    - Experiments with improved sensing, filtering, and response time.

- **`RoboCup 2024-25/`**  
  Prior-season artifacts:
  - International TDP (Team Description Paper)  
  - National and international posters  
  - Reference designs from our earlier EV3/Arduino platforms.

- **`VVS Ballers Logo.jpg`**  
  Team branding asset.

---

## Regional robot highlights (2025–26)

**Drive system**
- 4 × EV3 Medium Motors (approx. 240–250 rpm, 8 N·cm running torque, 12 N·cm stall torque)  
- 1:1 gear ratio to aluminium omni wheels in a **diamond** configuration  
- Jenga-stacked motor mounting for compact packaging  
- Smooth holonomic movement with **diagonal back moves** for safe escapes near lines/walls  

**Field awareness**
- 2 × colour sensors (centre/front and back) to detect white lines and avoid entering goal D areas (foul zones)  
- 2 × side IR distance sensors (left/right) to detect walls and obstacles, choose safe escape directions  
- 1 × IR seeker (0–9 zones) for ball direction tracking  
- 1 × compass sensor to correct heading before each movement and prevent slow rotation over time  

**Kicker subsystem**
- External module (not driven directly by EV3) built around:
  - Arduino Nano Every  
  - 12 V battery + buck converter  
  - TBFNG H-bridge motor driver  
  - Pololu 20D metal gearmotor (450 rpm, 2.4 kg·cm torque)  
  - HC-SR04-style ultrasonic sensor to detect ball presence in the capture area  
- Firmware implements a **state machine**:
  - IDLE → forward kick → forward hold → reverse retract → reverse hold → cooldown  
  - Tunable timing constants for consistent kicking & safe mechanical limits  

**Design & tooling**
- CAD: Fusion 360  
- 3D printing: Bambu Lab, PETG material  
- Programming:
  - LEGO Mindstorms Education EV3 for field logic  
  - Arduino IDE (C++) for the kicker subsystem  

---

## Current nationals direction

For nationals, we are evolving the platform towards a more powerful and modular stack:

- Custom BCBS-style electronics and IR boards designed in KiCad  
- Teensy-based main controller and refined Pololu motor selection  
- Solenoid-style (or high-power) kicker concept with improved energy delivery  
- Focus on:
  - Cleaner wiring and power distribution  
  - More robust sensing (filtering, calibration)  
  - Mechanical durability under heavy match collisions  

---

## How to use this repository

### Code

- **Arduino sketches**  
  - Open `.ino` files (e.g. `Robocup_Regionals_Kicker_Code.ino`, `Nationals_IR_Code.ino`) in:
    - Arduino IDE, or  
    - PlatformIO (if you prefer a VS Code workflow).  

- **EV3 program**  
  - Open `.ev3` project files in **LEGO Mindstorms Education EV3** software.  
  - Use USB or SD card transfer to deploy to the EV3 brick.

### Electronics

- **Nationals IR board** (`RoboCup Nationals 2025-26/IR_board/`):
  - Use **KiCad 7+** to open the schematic (`.sch`) and PCB (`.kicad_pcb`).  
  - STEP/DXF exports are provided for checking board fit in CAD assemblies.

### CAD & 3D prints

- **Regionals 3D models** (`RoboCup Regionals 3D Models/`):
  - Open `.f3d`, `.step`, `.3mf`, or `.stl` files in Fusion 360 or your preferred CAD/slicer.  
  - PETG printing recommended (as used on our Bambu Lab).

### Posters & documentation

- Poster PDFs/PNGs and markdown files summarise:
  - Robot architecture  
  - Sensor logic  
  - Kicker design  
  - Match results and future work  

They are useful for:
- Judging and pit-area displays  
- Outreach presentations  
- Onboarding new team members.

---

## Credits

- **Team:** VVS Ballers – RoboCup Junior Soccer  

We are grateful to our mentors, school, sponsors, and the RoboCup Junior community for support and inspiration.

---
