# Ultrasonic Wall / Obstacle Ranging Board — Arduino Nano Every

Firmware for the **Ultrasonic_PCB**: an **Arduino Nano Every (ABX00028,
ATmega4809)** carrier with **four HC-SR04** sensors, one on each side. It ranges
the four field walls / nearby robots and streams the four distances to the main
PCB over a **framed, CRC-checked UART packet** — built for a fast robot on the
**182 cm × 243 cm** field, so the values are *accurate, fast, and correct*.

Two sketches, each opens directly in the **Arduino IDE** and also builds under
PlatformIO:

| Folder (open the matching `.ino`) | What it is |
|---|---|
| [`Ultrasonic_NanoEvery/`](Ultrasonic_NanoEvery) | **The board firmware.** Pings the 4 sensors, filters, and emits the packet on `Serial1`. |
| [`Ultrasonic_Receiver_MainBoard/`](Ultrasonic_Receiver_MainBoard) | **Reference parser** for the main board — drop `ultrasonicPoll()` into your main firmware. |

---

## 1. Hardware — pin map (single source of truth)

Read straight from `Ultrasonic_PCB.kicad_pcb` (verified against the routed
netlist, not guessed). Index order is **clockwise: FRONT, RIGHT, BACK, LEFT**.

| Idx | Side | PCB sensor | Board edge | TRIG | ECHO |
|---|---|---|---|---|---|
| 0 | FRONT | U4 | top | **D8** | **D9** |
| 1 | RIGHT | U3 | right | **D4** | **D5** |
| 2 | BACK | U2 | bottom | **D6** | **D7** |
| 3 | LEFT | U1\* | left | **D2** | **D3** |

\*The PCB labels a *sensor* as `U1`; the Arduino module's reference is
`"Arduino Nano"`. The four sensors sit at the four board edges around the Nano.

**Other nets**
- **UART to main PCB** — connector **J2**: pin 1 → **D0 (RX)**, pin 2 → **D1
  (TX)**. On the Nano Every these are the hardware **`Serial1`** UART, so the USB
  `Serial` stays free for debugging.
- **Power** — connector **J1**: pin 1 → **VIN**, pin 2 → **GND**. Every HC-SR04
  `VCC` is tied to the VIN rail, so feed J1 a clean **5 V** (HC-SR04 needs 5 V;
  the Nano Every's regulator also accepts it on VIN). Common GND with the main
  board is required for the UART.

> **"Which sensor is FRONT?"** `FRONT` is defined here as the board edge carrying
> **U4**. The true forward direction depends on how you bolt the board into the
> robot. If your nose points at a different edge — or the board is mounted
> components-down, which swaps LEFT/RIGHT — just re-order the four entries in
> `TRIG_PIN[]`/`ECHO_PIN[]` in `config.h` (keep each TRIG+ECHO pair together) so
> index 0 faces forward, then bench-check (§6).

---

## 2. How it works (and why these choices)

**Opposite-pair pinging (the key trick).** Firing all four HC-SR04 at once would
let an adjacent sensor (only ~5 cm away on the board, 90° apart) hear another's
burst directly and report a falsely-short distance. Firing strictly **one at a
time** is safe but slow (up to ~60 ms/cycle). So the firmware pings the two
**opposite-facing** sensors together — **FRONT+BACK**, then **RIGHT+LEFT**. Two
adjacent sensors never fire at the same instant (no cross-talk), yet the cycle is
halved. The two echoes of a pair are timed **concurrently** in a single tight
polling loop, so there is no interrupt jitter and no second blocking wait.

**Distance.** `distance_mm = echo_µs × c / 2`, with a temperature-compensated
speed of sound `c = 331.4 + 0.6·T` m/s (`AMBIENT_TEMP_C`, default 20 °C). Timing
resolution is a few µs ≈ sub-mm — well below the HC-SR04's real ~±1 cm accuracy.

**Accuracy filtering.** A per-sensor **median-of-3** removes the HC-SR04's
occasional single-sample fliers at just one sample of lag (ideal for a fast
robot). An optional EWMA (`EWMA_ALPHA`, default off) adds smoothing if you want
it. On a missed echo the sensor **coasts** on its last good value for
`HOLD_CYCLES` cycles before declaring out-of-range, so one dropped ping doesn't
make a wall "blink away" under the main board.

**Range bound.** The field's longest span is 243 cm, so `MAX_RANGE_MM` defaults
to **2600 mm**. This also caps the per-ping timeout (~16 ms) — anything farther
is reported as out-of-range rather than stalling the loop.

**Rate.** A full four-sensor set runs at up to **~50 Hz** (`MEASUREMENT_PERIOD_MS
= 20`). That floor also bounds how often each sensor re-pings, which keeps arena
reverberation from leaking into the next ping.

---

## 3. UART wire protocol

One **13-byte** little-endian frame per cycle on `Serial1` @ **115200 8N1**.
Distances are **millimetres**.

```
byte  0   0xAA              sync 0
byte  1   0x55              sync 1
byte  2-3 FRONT  mm  (u16, little-endian)
byte  4-5 RIGHT  mm
byte  6-7 BACK   mm
byte  8-9 LEFT   mm
byte 10   status            2 bits/sensor: F=1:0 R=3:2 B=5:4 L=7:6
                              0 = OK, 1 = out-of-range (mm = MAX_RANGE), 2 = held
byte 11   seq               u8 counter, +1 per packet (gaps = dropped packets)
byte 12   crc8              poly 0x07, init 0x00, over payload bytes [2..11]
```

Integrity, by design: the **2-byte sync** makes false frame-locks unlikely, the
**CRC-8** lets the receiver throw away any corrupted/partial frame, and the
**sequence counter** lets it spot dropped packets / a board reset. The 13 bytes
take ~1.1 ms at 115200, so the link is never the bottleneck.

A human-readable mirror prints on the **USB Serial** (`DEBUG_USB`) for bring-up —
it does not touch the `Serial1` data link.

---

## 4. Build & upload

### Arduino IDE (primary)
1. **Boards Manager →** install **"Arduino megaAVR Boards"**.
2. **File ▸ Open…** `Ultrasonic_NanoEvery/Ultrasonic_NanoEvery.ino` (the IDE
   loads `config.h` as a tab automatically).
3. **Tools ▸ Board ▸ Arduino megaAVR Boards ▸ Arduino Nano Every.**
4. **Tools ▸ Registers emulation ▸ "None (ATMEGA4809)"** (fastest; recommended).
5. Select the **Port**, click **Upload**.
6. **Tools ▸ Serial Monitor @115200** → live `F=… R=… B=… L=… mm  ~Hz` stream.

### PlatformIO (optional — same folder)
```powershell
python -m platformio run   -d "Ultrasonic_NanoEvery"            # build
python -m platformio run   -d "Ultrasonic_NanoEvery" -t upload  # build + upload
python -m platformio device monitor -b 115200                   # USB monitor
```

---

## 5. Main-board integration

Wire **ultrasonic D1 (TX) → main-board RX**, **GND ↔ GND** (and D0←main TX only
if you later add commands). Then copy the `UltrasonicData` struct and
`ultrasonicPoll()` from
[`Ultrasonic_Receiver_MainBoard/`](Ultrasonic_Receiver_MainBoard) into your main
firmware and call it every loop on the port wired to this board:

```cpp
if (ultrasonicPoll(Serial2)) {
    uint16_t front = us.mm[US_FRONT];   // mm; also RIGHT / BACK / LEFT
    // us.status[i] : 0 OK · 1 out-of-range · 2 held
}
if (ultrasonicStale()) { /* link dropped — fail safe, don't trust old ranges */ }
```

It is non-blocking, resyncs on the header, and **only** updates on a CRC-valid
frame. If the main board runs at 3.3 V, level-shift this board's 5 V TX down.

---

## 6. Bench-check & tuning

1. **Sanity.** Power up, open the USB Serial Monitor. Point each side at a wall
   ~30–100 cm away; the matching `F/R/B/L` value should read roughly that
   distance in mm and track as you move the board.
2. **Confirm the FRONT mapping.** Wave a hand only in front of the robot — only
   `F` should drop. Repeat per side. If the wrong letter responds, re-order
   `TRIG_PIN[]`/`ECHO_PIN[]` in `config.h` (§1 note).
3. **Temperature.** For best absolute accuracy set `AMBIENT_TEMP_C` to the room
   temp (each 5 °C off ≈ 1 % distance error).
4. **Speed vs. smoothing.** Default `MEDIAN_WINDOW=3`, `EWMA_ALPHA=1.0` is the
   snappy choice. If readings are jumpy near a hard wall, set `EWMA_ALPHA=0.5`;
   if you need even more rate, drop `MEASUREMENT_PERIOD_MS` toward 15 (watch for
   reverberation artifacts in a small, hard-walled arena).

### Tuning knobs — all in `Ultrasonic_NanoEvery/config.h`

| Knob | Default | Trade-off |
|---|---|---|
| `MAX_RANGE_MM` | 2600 | Covers the 243 cm field; also caps per-ping timeout. |
| `MEASUREMENT_PERIOD_MS` | 20 | Lower = faster updates, more self-echo risk. |
| `INTER_PHASE_GAP_MS` | 2 | Quiet settle between the two ping phases. |
| `MEDIAN_WINDOW` | 3 | 1 = no spike rejection; 5 = stronger but +1 sample lag. |
| `EWMA_ALPHA` | 1.0 (off) | Lower = smoother but laggier (worse for a fast robot). |
| `HOLD_CYCLES` | 2 | Cycles to coast on a dropout before "out-of-range". |
| `AMBIENT_TEMP_C` | 20 | Speed-of-sound compensation. |
| `UART_BAUD` | 115200 | Match the main-board port. |
| `ENABLE_WATCHDOG` | 1 | Resets the MCU if `loop()` ever stalls > ~1 s. |

---

## 7. Notes & limits

- **Wall vs. robot is the main board's job.** A single ranging sensor only gives
  distance. The main board can flag "robot" when a side reads *much* shorter than
  the wall distance expected for the robot's known pose — this firmware gives it
  clean, fresh, trustworthy distances + per-sensor status to do that.
- **HC-SR04 caveats:** ~15° beam, soft/angled surfaces and acute corners can
  scatter the echo (occasional dropouts — handled by coasting + status). Min
  reliable range ~2 cm (values clamp to `MIN_RANGE_MM`).
- **For maximum cross-talk immunity** at half the rate, ping one sensor per phase
  instead of in opposite pairs (edit `PING_PHASE`/the `loop()` phases).
