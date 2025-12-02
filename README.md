# VVS Ballers RoboCup Projects

## About
VVS Ballers build and iterate RoboCup Junior Soccer robots across multiple seasons, documenting CAD, electronics, firmware, and posters for regionals, nationals, and international events. The repository gathers full-season artifacts, from EV3-based holonomic drives and Arduino kicker subsystems to new Teensy-powered control boards and Pololu-driven mechanisms for upcoming nationals.

## Repository structure
- **RoboCup Regionals 2025-26/**: EV3 holonomic robot materials, including LEGO Mindstorms program (`RoboCup Regionals Robot Code.ev3`), Arduino Nano Every kicker firmware (`Robocup_Regionals_Kicker_Code.ino`), PETG 3D-printable covers and kicker parts (`RoboCup Regionals 3D Models/`), and the regional poster summarizing performance and design.
- **RoboCup Nationals 2025-26/**: In-progress nationals work with a KiCad IR board design (`IR_board/` with schematic, PCB, STEP, and DXF exports) and updated infrared firmware targeting a Teensy-based stack (`Nationals_IR_Code.ino`).
- **RoboCup 2024-25/**: Prior-season reference posters and team description paper, including the internationals TDP and national/international poster graphics.
- **VVS Ballers Logo.jpg**: Team branding asset.

## Highlights from regional robot
- **Drive system:** Four EV3 Medium Motors with omni wheels in a diamond layout for smooth holonomic motion and diagonal escape moves.
- **Field awareness:** Dual underside color sensors for line detection, side IR distance sensors for wall avoidance, a compass for heading correction, and an IR seeker for ball tracking across 10 zones.
- **Kicker subsystem:** Arduino-controlled Pololu 20D metal-gear motor with ultrasonic ball detection, TBFNG H-bridge driver, and 3D-printed PETG kicker; implemented as a state machine for consistent kick, retract, and cooldown cycles.
- **Design & tooling:** Fusion 360 CAD, Bambu Lab 3D printing, LEGO Mindstorms Education EV3 programming for field logic, and Arduino IDE for kicker control.

## Current nationals direction
We are evolving the platform for nationals with custom BCBS electronics, a Teensy controller, Pololu motors, and a solenoid-based kicker, building on the lessons from regionals while refining sensing, power, and durability.

## How to use this repository
- **Code:** Open `.ino` sketches in the Arduino IDE or PlatformIO for firmware review. The EV3 program file can be examined in the LEGO Mindstorms EV3 software.
- **Electronics:** KiCad 7+ can load the nationals IR board project (`IR_board/`) for schematic and PCB edits; STEP/DXF exports are provided for mechanical integration.
- **CAD & prints:** STL, 3MF, and STEP files in `RoboCup Regionals 3D Models/` are ready for slicing or remixing.
- **Posters & docs:** PDF/PNG/MD assets capture season summaries and can be used for outreach, judging, or team onboarding.

## Suggested tags
RoboCup, RoboCup Junior Soccer, EV3, Teensy, Arduino, Holonomic Drive, Omni Wheels, Kicker Mechanism, Pololu Motor, Compass Navigation, IR Seeker, KiCad, 3D Printing, PETG, Fusion 360
