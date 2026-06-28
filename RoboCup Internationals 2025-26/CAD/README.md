# CAD

Mechanical design for the chassis, done in **Fusion 360**.

| Folder | Contents |
|---|---|
| `Fusion/` | Fusion sources — `.f3z` archives (full assemblies + version history) and `.f3d` designs. |
| `Printable/` | 3D-print-ready meshes (`.stl`, `.3mf`), grouped **by subsystem**: `Chassis/`, `Drive/`, `Ball_Capture/`, `Sensor_Mounts/`. |

Older/superseded chassis, motor-bracket and support-beam studies are in
[`../Legacy/CAD`](../Legacy/CAD).

## Notes

- **Large files / Git LFS.** The `.f3z` archives are large (the full-robot versions
  are ~66 MB each). This repo tracks `.f3z`, `.f3d`, `.step`, `.stl` and `.3mf`
  through **Git LFS** (see [`../.gitattributes`](../.gitattributes)); run
  `git lfs install` once before cloning or committing.
- **What prints what.** `Printable/` is organised by subsystem:
  - `Chassis/` — side walls, base / top base, support beams, lifting handle.
  - `Drive/` — omni wheels, rollers, couplings.
  - `Ball_Capture/` — the capture-area dribbler mouth (all iterations).
  - `Sensor_Mounts/` — IR cover, camera holders, ultrasonic holders/plates.
- The **capture area** is sized to keep the ball within the rules' ≤ 1.5 cm
  ball-capturing-zone limit; the **handle** satisfies the rules' handle/clearance
  requirement.
