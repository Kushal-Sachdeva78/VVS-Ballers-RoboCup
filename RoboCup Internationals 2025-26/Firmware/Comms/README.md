# Comms — inter-robot role switching (`RobotLink`)

A peer-to-peer **2.4 GHz nRF24L01+** link that lets the two robots share one
firmware and pick their role at runtime:

> **partner alive → play your base role · partner lost → cover the goal as keeper**

So if a robot dies or is removed, its partner drops to goalkeeper within
`PARTNER_TIMEOUT_MS` and the net is never left open. When the partner returns, a
base-attacker resumes attacking at the next referee **GO** (kick-off).

- **Module:** [`RobotLink/RobotLink.h`](RobotLink/RobotLink.h) +
  [`RobotLink/RobotLink.cpp`](RobotLink/RobotLink.cpp)
- **Scaffold:** [`RoleDispatch_Example/RoleDispatch_Example.ino`](RoleDispatch_Example/RoleDispatch_Example.ino)
- **Library required:** RF24 (TMRh20) — <https://github.com/nRF24/RF24>

> Radio pins, the RF channel, and the timings are marked `>>> BENCH-VERIFY` in
> `RobotLink.h` — confirm them against Main_PCB_2.0 before wiring. A dead or absent
> radio is safe by design — see *Fail-safe* below.

> **RCJ compliance:** robot-to-robot only. No PC, phone, or access point in the
> link; 2.4 GHz peer-to-peer is the permitted inter-robot band.

---

## 1. Hardware / wiring

One nRF24L01+ per main board, on **SPI1** — **not** SPI0, because SPI0's MOSI/MISO
(pins 11/12) are Motor 4.

| nRF24L01+ | Teensy 4.1 | Notes |
|---|---|---|
| MOSI | 26 | SPI1 MOSI · `>>> BENCH-VERIFY` |
| MISO | 39 | SPI1 MISO · `>>> BENCH-VERIFY` |
| SCK | 27 | SPI1 SCK · `>>> BENCH-VERIFY` |
| CE | 30 | free GPIO · `>>> BENCH-VERIFY broken out & free` |
| CSN | 31 | free GPIO · `>>> BENCH-VERIFY broken out & free` |
| VCC | 3V3 | **add a 10 µF cap across VCC/GND** (the module browns out on TX bursts otherwise) |
| GND | GND | common ground |
| IRQ | — | unused (we poll, non-blocking) |

Occupied main-board pins the radio must avoid: `2,3,4,5,6,7,8,9,10,11,12,14,15,16,
20,21,22,24,25,34`, plus Serial7/comms `28/29` and Wire1/OLED `18/19`. Confirm
26/27/30/31/39 are actually broken out on **Main_PCB_2.0** before wiring.

## 2. Config (in `RobotLink.h`)

Set **per robot** (the only difference between the two builds):

```c
#define MY_ROBOT_ID   0               // 0 on one robot, 1 on the other
#define MY_BASE_ROLE  ROLE_ATTACKER   // ROLE_ATTACKER on one, ROLE_DEFENDER on the other
```

Shared tunables (both robots identical): `RF_CHANNEL`, `RF_ADDR_0/1`, `RF_CE_PIN`,
`RF_CSN_PIN`, `RF_SPI_*`, `HEARTBEAT_MS` (100), `PARTNER_TIMEOUT_MS` (750),
`ALIVE_DEBOUNCE_MS` (300), `PROMOTE_RESUME_MS` (3000). All are documented and
bench-flagged in the header.

## 3. Wire protocol (house framing)

8-byte payload, same convention as the wired links — 2-byte sync, little-endian
body, rolling seq, CRC-8 (poly 0x07, init 0x00 = `us_crc8`). The nRF's own
hardware CRC runs underneath as a second integrity layer.

```
[0]=0xC3 [1]=0x3C [2]=robotId [3]=baseRole [4]=currentRole [5]=state [6]=seq [7]=crc8(over [2..6])
  state: bit0 = alive/running (room to grow: ballSeen, kicking, etc.)
```

## 4. Integrating into the existing firmware

The current role is compile-time (you flash attacker XOR defender). To make it a
**runtime** variable, wrap each existing `loop()` body in a function and let
`RobotLink.role()` pick:

1. Copy `RobotLink.h` / `RobotLink.cpp` next to your main sketch (or install as a
   library) and `#include "RobotLink.h"`.
2. In `setup()`: call `RobotLink.begin();` **once** (it's non-blocking and safe
   even if the module is missing). Keep your existing sensor/motor init.
3. Refactor the two behaviours into functions (no logic changes):
   - `runAttacker()` ← the body of `Attacker_Chase_Aim_Kick_Line.ino` `loop()`
   - `runDefender()` ← the body of `Defender_Full.ino` `loop()`
   Shared init (motor pins, BNO055, IR/US/line/camera links, kicker) should run
   **once** in `setup()` — don't double-init a peripheral (e.g. call `bno.begin()`
   a single time). Role-specific-only init can stay in a per-role setup.
4. Dispatch each pass:

```c
void loop() {
  RobotLink.update();                       // non-blocking: rx + heartbeat + arbitrate
  RobotLink.setRefereeGo(commsRunning());   // optional: feed your GO/STOP reader

  if (RobotLink.role() == ROLE_ATTACKER) runAttacker();
  else                                   runDefender();   // also boot + fail-safe role
}
```

`RobotLink.update()` is non-blocking (no `delay()`); call it every pass. See
[`RoleDispatch_Example`](RoleDispatch_Example/RoleDispatch_Example.ino) for a
compilable skeleton.

> **Referee GO hook (recommended):** feeding `setRefereeGo()` the GO/STOP state
> means a base-attacker that dropped to keeper only **returns to attacking at the
> next kick-off**, not mid-play. If you never call it, `RobotLink` falls back to a
> timed resume (`PROMOTE_RESUME_MS`) once the partner is stable again.

## 5. Arbitration behaviour

| Situation | Role |
|---|---|
| Boot, no partner heard yet | **DEFENDER** (goal never open) |
| Partner alive | your `MY_BASE_ROLE` |
| Partner lost (`> PARTNER_TIMEOUT_MS`) | **DEFENDER**, immediately |
| Partner back, base = attacker | stay DEFENDER until the next GO edge (or timed resume) |
| Base = defender | always DEFENDER |

Demote-to-keeper is immediate (safety); promote-back is gated so roles can't flap
mid-play. The alive↔lost edge is debounced (`ALIVE_DEBOUNCE_MS`).

## 6. Fail-safe / graceful degradation

- **No partner yet →** DEFENDER (zero/default `Role` is the safe one).
- **Radio won't init (absent/dead/mis-wired) →** `radioOk()` is false and the
  robot runs its **static `MY_BASE_ROLE`** forever. It never blocks or stalls —
  you simply lose dynamic switching. This is explicit in `begin()`/`update()`.

## 7. Bench-test procedure

1. **Both robots powered, on a desk, ~1 m apart.** Open both USB serials. Confirm
   each reports `radio=ok`. The attacker-base robot should settle to `role=ATK`
   (after the GO edge / timed resume), the defender-base to `role=DEF`. Each should
   show `partner=alive`.
2. **Kill one robot** (power it down or pull its nRF24). Within
   `PARTNER_TIMEOUT_MS` (~0.75 s) the survivor must report `partner=LOST` and flip
   to `role=DEF`. Verify a base-attacker survivor is now the keeper.
3. **Restore the partner.** The survivor should report `partner=alive` again but
   **stay** `DEF` — it must **not** snap back to attacker on its own.
4. **Simulate kick-off:** drive `setRefereeGo(true)` (press GO, or temporarily
   hard-code a GO pulse). Only then should the base-attacker return to `role=ATK`.
   (With no GO wired, confirm it instead resumes after `PROMOTE_RESUME_MS`.)
5. **Dead-radio check:** unplug one nRF24 and power up that robot alone. It must
   boot and run its static base role normally (`radio=absent`), never hang.

Tick all five off on real hardware before relying on the link in a match.
