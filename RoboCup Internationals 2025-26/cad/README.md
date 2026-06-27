# CAD

Mechanical design for the chassis, done in **Fusion 360**.

| Folder | Contents |
|---|---|
| `fusion/` | Fusion sources — `.f3z` archives (full assemblies + version history) and `.f3d` designs. Includes the support-beam study in `fusion/Claude/`. |
| `print/` | 3D-print-ready meshes (`.stl`, `.3mf`), grouped by print round (1–3). |

Older/superseded chassis and motor-bracket studies are in
[`../legacy/cad`](../legacy/cad).

## Notes

- **Large files / Git LFS.** The `.f3z` archives are large (the full-robot versions
  are ~66 MB each). This repo tracks `.f3z`, `.f3d`, `.step`, `.stl` and `.3mf`
  through **Git LFS** (see [`../.gitattributes`](../.gitattributes)); run
  `git lfs install` once before cloning or committing.
- **What prints what.** The `print/` meshes cover the side walls, omni wheels and
  rollers, base / top base, the capture-area dribbler mouth, the IR cover, camera
  holders, couplings, ultrasonic holders, and the lifting handle.
- The **capture area** is sized to keep the ball within the rules' ≤ 1.5 cm
  ball-capturing-zone limit; the **handle** satisfies the rules' handle/clearance
  requirement.
