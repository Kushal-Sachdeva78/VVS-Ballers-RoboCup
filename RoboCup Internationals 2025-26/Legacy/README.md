# Legacy / archive

Superseded designs, kept for history and reference. **Nothing here is the current
build** — see the top-level `Firmware/`, `PCB/`, and `CAD/` for what the robot
actually runs and uses today.

## What's here and why it was replaced

### `Firmware/Attacker/`
- **`Attacker_Chase_Aim_Kick`** — the attacker *before* the line ring was added. It
  flees walls on **ultrasonic alone**. Superseded by
  `Firmware/Attacker/Attacker_Chase_Aim_Kick_Line`, which only flees when the line
  ring **and** ultrasonics agree (far fewer false escapes; safer against the
  out-of-bounds penalty).
- **`IR_Ball_Chase_Avoid_Angle`** — the original foundation sketch: IR chase +
  ultrasonic avoidance + heading hold + **HC-05 Bluetooth telemetry**, no camera. The
  current attacker grew out of this; kept because its comments document the drive
  base and parsers it all started from.

### `Firmware/Defender/`
- **`Defender_NoLine`** — a simple bang-bang goalkeeper using **ultrasonics only**
  (back-wall standoff band + side-wall avoidance + IR left/right tracking), no line
  ring and no state machine. A dependable fallback; superseded by `Defender_Full`,
  which uses the line ring as the primary box reference.

### `PCB/Main_v1/`
- The **first main-board revision** (`Main_PCB`), replaced by `PCB/Main`
  (`Main_PCB 2.0`).

### `CAD/`
- **`Old_Models/`** and **`Unused/`** — earlier chassis, motor-bracket, and
  support-beam studies that were iterated past.

> These were **archived rather than deleted** so the design history — and the
> "why we changed it" — stays available.
