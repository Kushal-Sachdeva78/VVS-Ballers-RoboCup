# Main Robot — Support Beam Modification Report

Design modified: **Main Robot** (cloud design, KS HUB). Saved as a new version — the previous state is recoverable from version history. All work done inside Fusion; no PCB, hole, or case dimensions were changed. All coordinates below in mm, assembly (root) space, Y = up.

## 1. Every modification made

**A. New component `Claude Support Beams`** (one base feature, 4 solid bodies):

1. **Main Beam L** — vertical beam at the left Teensy-side screw hole (86.75, −22.97):
   - Foot flange 16×18mm on the Main PCB top (y 62.54→82.5), Ø5.85×10.5mm spigot down through the board's Ø6 hole into the matching Ø6 tray bore in the mounting structure (Component30).
   - Column 14×11.5→12.8mm rising to the IR Cover underside (y 105.08), cross-section biased forward to clear the Power PCB board edge (1.0mm) and front terminal block (1.1mm).
   - Ø3.05×11mm pin at top, entering the IR Cover's existing Ø3.1 through-barrel at the same XZ.
   - Power-support rail (4mm tall, y 78.5→82.5) running under the Power PCB's left edge with 2 bosses + Ø2.7 pilots under the board's Ø6 corner holes at (73, −32) and (73, −89).
2. **Main Beam R** — mirror of the above at (171.75, −22.97), rail bosses at (184.5, −32) and (184.5, −88.5) (matches the board's own 0.5mm hole asymmetry).
3. **IR-US Post L** — 9.5×7×15mm post standing on the IR PCB at its rear hole (97.27, −49.53): Ø4.85 spigot into the board's Ø5 hole, Ø2.7 pilot from below (M3 + washer through the cover's central opening clamps board to post), top arm with Ø5.85 spigot up into the Ultrasonic case plate's Ø6 hole at (97.5, −54.29) + Ø2.7 pilot from above.
4. **IR-US Post R** — same at (158.18, −49.53) → plate hole (160.5, −54.29).

**B. Alignment correction — `External1`** (block behind Power PCB): was rotated 1.418° off-axis (only misaligned part in the assembly) and its front face clipped the Power PCB board edge by ~0.7mm. De-rotated about its center and shifted 0.5mm back; front face now at z = −96.15, giving 0.65mm clearance. Position snapshot captured.

**C. Helper scripts** left in `claude_scripts\` (audit/build script + JSON logs). Safe to delete; the `claude_audit` entry can be removed from Scripts and Add-Ins.

## 2. Re-centering check

- Chassis stack (Base with Blocks / top base / Main PCB tray) concentric about x = 129.25 within 0.01mm — no correction needed.
- IR Cover, IR PCB, Ultrasonic case and PCB concentric within 0.8mm of each other — left as-is per design intent.
- **Only External1 required correction** (see 1B). Nothing else was moved. Beams are symmetric about the robot centerline x = 129.25.

## 3. Required confirmations

- **Teensy-side screw holes used as Main PCB attachment points: YES** — the two Ø6 holes at (86.75, −22.97) and (171.75, −22.97), which flank the Teensy 4.1 (x 99–161, z −32→−14). One beam per hole.
- **Power PCB no longer floating: YES** — previously hovering with zero support; now rigidly held at all 4 corner holes by bosses on the two main beams (M3 self-tap + washer from top). Its back edge clearance to External1 corrected from interference to +0.65mm.
- **Two beams connect Main PCB ↔ IR PCB case: YES** — and they land exactly in the cover's two pre-existing Ø3.1 support barrels (the cover was clearly designed for posts there).
- **Two beams connect IR PCB ↔ Ultrasonic PCB case: YES** — symmetric rear posts, bolted both ends.
- Interference analysis (new bodies vs Power PCB, top base incl. Main PCB, IR PCB, Ultrasonic case, External1): **0 collisions**.

## 4. Concerns found during the work

**Pre-existing issues (not introduced, not fixed — your call):**
1. **IR PCB sinks 1.08mm into the IR Cover**: board underside y 116.30 vs cover top face y 117.38. Physically the board will sit ~1.1mm higher than CAD shows. Recommend raising IR_PCB + everything above by 1.08mm in CAD, or recessing the cover rim.
2. **Two overlapping ultrasonic assemblies**: `Ultrasonic_PCB:1` (old) and `Ultrasonic_PCB 2.0:1` occupy the same space; the old one appears stale and should be removed or hidden.

**Notes on the new parts:**
3. Tight CAD clearances to respect when printing (PETG, ±0.2mm or better): right rail ↔ Power PCB pin headers 0.5mm; right rail ↔ CW-019 module 0.8mm; left column ↔ front terminal block 1.1mm; left front boss ↔ terminal block 1.3mm (use a Ø8 washer there, not Ø9).
4. Cover pins are a snug press fit (Ø3.05 in Ø3.1–3.15). Drill the cover holes 3.2mm if too tight; a drop of CA glue adds uplift rigidity — top-down bolting is impossible there because the IR PCB is solid above the barrels.
5. Hardware: 4× M3×10 + washers (Power PCB corners, from top), 2× M3×10 + Ø9 washers (post bottoms, inserted upward through the cover's central opening), 2× M3×12 + washers (post tops, down through the Ultrasonic plate). Pilots are Ø2.7 for M3 self-tapping.
6. Assembly order: fit main beams to Main PCB first (spigots into board+tray bores), lower the IR Cover onto the pins, then IR PCB, bolt posts from below, then the Ultrasonic case on top.
7. Printing: main beams lie on their flat side with tree supports (spigot, rail, bosses); posts print upright, small support under the 5mm top-arm overhang. No section thinner than 1.2mm.
