# PCB

KiCad sources, Gerbers, drill and job files for every board, plus human-readable
wiring and ready-to-order fab packages.

| Folder | Board | Contents |
|---|---|---|
| `Main/` | Main board (Teensy 4.1 carrier) | KiCad source + Gerbers — drivers, IMU, links, kicker relay (Main_PCB 2.0) |
| `IR/` | IR ball ring | 16× TSSP58038 + Teensy |
| `Line/` | Line ring | 4× QTR-MD-05A + Teensy |
| `Ultrasonic/` | Ultrasonic board | 4× HC-SR04 + Nano Every |
| `Power/` | Power board | 12 V / 5 V / 3.3 V rails + 48 V solenoid boost |
| `Wiring/` | — | **one wiring PDF per board** (the netlist in plain language) |
| `Fabrication/` | — | ready-to-order Gerber **zips** |

The previous main board revision is archived in
[`../Legacy/PCB/Main_v1`](../Legacy/PCB/Main_v1).

## Opening the sources

Each board folder is a KiCad project — open the `*.kicad_pro` (the matching
`.kicad_pcb` and `.kicad_sch` sit beside it). The loose `*.gbr` / `*.drl` /
`*.gbrjob` files are **generated** fabrication outputs; for ordering, prefer the
zipped packages in [`Fabrication/`](Fabrication). KiCad local-settings files
(`*.kicad_prl`) and auto-backups are intentionally git-ignored.

## Start with the wiring PDFs

If you only read one thing, read [`Wiring/`](Wiring): `Main_PCB.pdf`, `Power_PCB.pdf`,
`IR_PCB.pdf`, and `Line_PCB.pdf` list every connection (rails, sensor pins, UART
links) in plain language. They are the reference the firmware pin maps were verified
against.
