# Main Robot — Support Structure Correction Report

Design: **Main Robot** (cloud, KS HUB), saved as a new version. All coordinates in mm, assembly space, Y up. The previous `Claude Support Beams` bodies were deleted and rebuilt from scratch; the old state is recoverable from version history.

## 1. What the audit found wrong with the previous modification

The whole `Claude Support Beams` occurrence carried a stray transform of (−0.26, +0.43, +0.15) mm, so every spigot, boss and pilot was off its mating hole by ~0.3 mm — the parts would not have assembled. Worse, the old report's "Ø3.1 cover barrels" at (86.75, −22.97) and (171.75, −22.97) **do not exist**: the cover's real mounts are two centerline tabs — a rear tab with a Ø3.1 through-hole at **(129.267, −66.464)** and a front Ø8 lug at **(129.27, 33.28)** that is solid (no hole). The old press-fit pins poked into open air inside the cover ring. The old beam columns also stood directly in front of Power-PCB terminal blocks T1/T6 (left) and T4/T5 (right), and the right column sat 4 mm in front of the Main-PCB Teensy's micro-USB connector (which faces +X at the board's right end, shell at y 71.7–74.8, z −27…−19).

## 2. What was built instead

Six new bodies in `Claude Support Beams` (PETG, ±0.2 mm or better):

**Main Support Frame** (one rigid piece, x 60–197.7). Feet with Ø5.85 spigots drop through the Main PCB's two central Ø6 holes at (86.75, −22.97) / (171.75, −22.97) into the Component30 tray bores below. Low arms (y 62.5–66, under the USB cable line) run to two vertical columns relocated where nothing needs access: left column outboard at x 66.5–76.5, z −25.2…−14.2 (left of terminal T1, 0.3 mm in front of the Power-PCB edge); right column forward at x 179.5–187.3, z −13.5…−4.5 (in front of the Teensy, 0.8 mm from CW-019, fully outside the USB envelope). Rails at y 80.5–82.5 run under the Power-PCB edges, segmented on the left to clear the relay's underside pins, with Ø8.4 boss seats + Ø5.85 locating spigots + Ø2.7 pilots at the board's four Ø6 corner holes — now at the exact hole centers (73, −32), (73, −89), (184.5, −32), (184.5, −88.5). Rear risers stand on the Main PCB just outside the Power-PCB footprint (x 60–65.8 and 191.9–197.7) and carry a rear bridge bar (y 100.6–105.08) whose pad sits under the cover's rear tab with a Ø2.7 pilot exactly at (129.267, −66.464).

**Front Bridge** (separate piece). Two legs with Ø5.85 spigots into the Main PCB's unused front Ø6 holes at (101.75, 26.03) / (156.75, 26.03) (tray bores beneath, like the central pair), crossbar at y 100.6–105.08, and a pad under the front lug with a Ø2.7 pilot at (129.27, 33.28). The cover ring also rests on both column tops, the rear bar and the crossbar (all topping out at exactly the cover underside, y 105.08).

**Four IR↔Ultrasonic posts** (RL, RR, FL, FR) — symmetric pairs at all four hole sets. Each: Ø4.85 bottom spigot into the IR PCB's Ø5 hole, Ø2.7 pilot from below (M3 + washer up through the open cover ring), body rising to the ultrasonic plate, Ø5.85 top spigot into the plate's Ø6 hole (the ultrasonic PCB has matching Ø6 holes at the same centers), Ø2.7 pilot from above (M3 down through the plate). Rear posts carry short top arms to span the 4.8 mm offset between the rear IR holes (97.3/158.2, −49.5) and plate holes (97.5/160.5, −54.3); front posts are straight blocks (offsets ≤ 2.3 mm). The RL post has a corner notch clearing the resistor at x ≤ 95, z ≤ −52.6.

The six Ø4.12 solid cylinders that plugged the top-base plate holes at (143.15, 20.33), (216.16, −9.68), (41.15, −38.67), (114.14, 20.39), (216.12, −38.66), (41.15, −9.67) were deleted — all six holes are open through the plate again (M4 clearance).

## 3. Verification results

1. **Hole matching/concentricity** — 18 mating interfaces checked programmatically against the actual created geometry: worst center offset 0.0000 mm. Spigot-to-hole fits are Ø5.85/Ø6 and Ø4.85/Ø5 (0.075 mm radial); pilots Ø2.7 for M3 self-tap; cover tab Ø3.1 passes M3.
2. **Power PCB screw terminals** — wire-entry corridors of all six Phoenix MKDS blocks (4 front-facing, 2 rear-facing) boolean-tested against every new body: 0 mm³ intersection, all CLEAR. Nothing new stands within 48 mm of any front terminal face at terminal height.
3. **Teensy micro-USB** — the cable envelope (x 160.8–188, y 66.5–80, z −31…−15) is empty: 0 mm³. The low arm passes 3+ mm below the plug line; the strut crosses 3.2 mm above it; the column is 1.5 mm in front of the envelope's z-range.
4. **Interference** — full check of the new bodies vs Power PCB, Main PCB + components, tray, top-base plate, IR PCB, Ultrasonic 2.0 assembly, External1, capture area: 0 real collisions (only µm-thin coincident-face artifacts at intended bearing planes: post tops against the plate, etc.).
5. **Four posts** — confirmed, symmetric about x = 129.25, sensors unobstructed (0.4–2 mm CAD gaps to the nearest components are listed below).
6. **Cover alignment** — rear pad pilot is exactly concentric with the cover's rear tab hole; an M3 passes down through the tab into the pad (7.5 mm engagement). Front pad pilot is exactly under the front lug center.
7. **No placeholder cylinders remain** — old press-fit pins replaced by screw pilots; the six plate plugs deleted; every intended screw location is a real hole.
8. Remaining concerns: below.

## 4. Hardware

- 4× M3×10 self-tap + washers — Power PCB corners, from above into the rail bosses.
- 1× M3×8 self-tap — down through the cover's rear tab into the rear pad.
- (1× M3×8 for the front lug after drilling — see concerns.)
- 4× M3×10 + Ø7 washers — up through the IR PCB's Ø5 holes into the post bottoms (driver reaches through the open cover ring from below).
- 4× M3×12 + washers — down through the ultrasonic plate's Ø6 holes into the post tops.

Assembly order: frame + front bridge onto the Main PCB (spigots into board + tray bores) → Power PCB drops vertically onto the four bosses (verified: all rear components pass under the rear bar with ≥ 4 mm) → cover on, rear tab screw → IR PCB with posts, bolted from below → ultrasonic assembly bolted from above.

## 5. Remaining concerns

1. **Front lug has no hole in the supplied cover mesh.** The cover is a mesh body and this Fusion build has no mesh-to-BRep API, so I could not cut it in CAD. The pad below it has an exactly-aligned Ø2.7 pilot: drill the lug Ø3.1–3.2 at its center (it's a Ø8 lug — center punch and drill) or add the hole in the cover's source file, then one M3 completes the joint. Until then the cover is held by the rear screw plus seating on four supports.
2. **Pre-existing, unchanged:** the IR PCB still sinks 1.08 mm into the cover top face (board underside y 116.30 vs cover top y 117.38), and the stale duplicate `Ultrasonic_PCB:1` still overlaps `Ultrasonic_PCB 2.0:1` — I recommend deleting the old one.
3. **Tight CAD clearances to respect when printing:** RL post to resistor leads 0.4–0.7 mm; front-left post to the vertical resistor 0.5 mm; right column to CW-019 0.8 mm; left column to terminal T1 1.0 mm laterally; risers to the Power-PCB board edge 0.2 mm. Print at ±0.2 mm or better and deburr.
4. **Power PCB removal** requires unscrewing the cover first (the rear bar passes 14 mm above the board's rear half), then it lifts straight out.
5. **Top-base plate holes** (the six reopened Ø4.12) sit ~0.15–0.4 mm past the edges of the side-cover-step walls below them, and those walls have no pilot holes — original-design issue I did not touch; check intended hardware before relying on them.
6. The six small printed parts: frame prints flat on its back (z −93.5 face down) with tree supports under bosses/arms; bridge upright; posts upright, support under the rear posts' 5 mm arm overhangs. No wall thinner than 1.2 mm except the noted 0.5 mm lip over the RL spigot.
7. During the session Fusion crashed once and the design history (timeline) had to be handled carefully; the final saved version is parametric with a clean timeline (4 Remove features + 1 Base feature added). The pre-session state is the previous version in version history.
