# Firmware

All microcontroller code for the robot. Each sub-folder targets one board. Each
sketch folder is named to match its `.ino`, because the Arduino IDE requires the
folder name to match the sketch, and most folders also carry a `platformio.ini` for
PlatformIO.

See [`../Docs/HOW_IT_WORKS.md`](../Docs/HOW_IT_WORKS.md) for the full explanation of
each subsystem and the inter-board protocols.

| Folder | Board | Flash this | Status |
|---|---|---|---|
| `Attacker/Attacker_Chase_Aim_Kick_Line` | Main (Teensy 4.1) | ✅ current attacker | IR chase + fused line/ultrasonic boundary escape + camera aim + kick |
| `Defender/Defender_Full` | Main (Teensy 4.1) | ✅ current keeper | line-primary box keeper + ultrasonic + state machine |
| `IR_Ball_Sensor/Code2_Calibrated` | IR ring (Teensy 4.1) | ✅ flight build | calibration baked in (2026-06-27) |
| `IR_Ball_Sensor/Code2_Refined` | IR ring | reference | same code, uncalibrated defaults |
| `IR_Ball_Sensor/Calibration` | IR ring | tool | baseline capture + Serial-Plotter readout |
| `Ultrasonic/Ultrasonic_NanoEvery` | Ultrasonic (Nano Every) | ✅ board firmware | 4× HC-SR04, opposite-pair pinging |
| `Ultrasonic/Ultrasonic_Receiver_MainBoard` | Main | reference | drop-in parser for the main firmware |
| `Line_Sensor/Line_PCB_Baseline` | Line ring (Teensy 4.1) | ✅ auto-learn detector | 1 s green baseline, then drop-by-5 |
| `Line_Sensor/Line_PCB_Detect` | Line ring | ✅ fixed-threshold detector | per-sensor thresholds + hysteresis |
| `Line_Sensor/Line_PCB_Raw_Reader` | Line ring | tool | raw ADC streamer for calibration |
| `Camera/Goal_Cam.py` | OpenMV H7 | ✅ match build | goal / keeper / open-corner vision |
| `Camera/Goal_Cam_Ball.py` | OpenMV H7 | diagnostic | same frame + orange-ball overlay |
| `Motor_Current/Motor_Current_Supervisor` | Nano Every (optional) | watchdog | stall-timer over-current protection |
| `Motor_Current/Motor_Current_Limit_1A` | Nano Every (optional) | watchdog | simple per-motor 1 A chop |
| `Comms/RobotLink` | Main (Teensy 4.1) + nRF24L01+ | optional link | inter-robot 2.4 GHz role switching — survivor covers the goal ([`Comms/README.md`](Comms/README.md)) |

Superseded no-line versions live in [`../Legacy/Firmware`](../Legacy/Firmware).

## Build quick-reference

- **Teensy 4.1** sketches: Arduino IDE → *Teensy 4.1*, USB Type *Serial*; or
  `pio run -e teensy41 -t upload`. The main board needs **Adafruit BNO055** +
  **Adafruit Unified Sensor** (and SSD1306 + GFX only if the Defender OLED option is
  enabled).
- **Nano Every** sketches: Arduino IDE → *Arduino Nano Every*, Registers emulation
  *None (ATMEGA4809)*; or `pio run -e nano_every -t upload`.
- **OpenMV H7:** copy `Goal_Cam.py` onto the camera's flash as `main.py`.
