# Legacy / archive

Superseded designs, kept for history and reference. **Nothing here is the current
build** — see the top-level `firmware/`, `pcb/`, and `cad/` for what the robot
actually runs and uses today.

## What's here and why it was replaced

### `firmware/attacker/`
- **`attacker_chase_aim_kick`** — the attacker *before* the line ring was added. It
  flees walls on **ultrasonic alone**. Superseded by
  `firmware/attacker/attacker_chase_aim_kick_line`, which only flees when the line
  ring **and** ultrasonics agree (far fewer false escapes; safer against the
  out-of-bounds penalty).
- **`ir_ball_chase_avoid_angle`** — the original foundation sketch: IR chase +
  ultrasonic avoidance + heading hold + **HC-05 Bluetooth telemetry**, no camera. The
  current attacker grew out of this; kept because its comments document the drive
  base and parsers it all started from.

### `firmware/defender/`
- **`Defender_NoLine`** — a simple bang-bang goalkeeper using **ultrasonics only**
  (back-wall standoff band + side-wall avoidance + IR left/right tracking), no line
  ring and no state machine. A dependable fallback; superseded by `Defender_Full`,
  which uses the line ring as the primary box reference.

### `pcb/main-v1/`
- The **first main-board revision** (`Main_PCB`), replaced by `pcb/main`
  (`Main_PCB 2.0`).

### `cad/`
- **`old-models/`** and **`not-using/`** — earlier chassis, motor-bracket, and
  support-beam studies that were iterated past.

> These were **archived rather than deleted** so the design history — and the
> "why we changed it" — stays available.
