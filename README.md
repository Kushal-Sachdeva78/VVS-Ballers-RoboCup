# RoboCup Junior Soccer Lightweight – VVS Ballers

This repository contains all the **hardware, PCB, and 3D model files** for the RoboCup Junior Soccer Lightweight robot developed by **Team VVS Ballers** from Vasant Valley School.

> ⚙️ Code and firmware will be added once testing and integration are complete.

---

## ⚽ Overview

Our robot is designed for the **RoboCup Junior Soccer Lightweight** competition.  
It is a fully autonomous soccer-playing robot built from scratch — including **custom PCBs**, **3D printed structures**, and **complete electrical wiring** between all modules.

This repository documents:
- PCB schematics and design files  
- 3D models for mechanical parts  
- Hardware layout and wiring setup between all boards  

---

## 🧩 System Layout

The robot is made up of multiple PCBs connected via wire, including:
- **Main controller PCB** (with Teensy microcontroller)  
- **Power board** (for distribution and regulation)  
- **Line sensor / IR sensor board** (for ball detection and boundary tracking)  

All PCBs are connected through dedicated power and data lines for stability and modular repair.

---

## 🧠 Electronics and Components

| Component | Function |
|------------|-----------|
| **Teensy** | Main controller board for logic and sensor data processing |
| **Line Sensor Board** | Detects lines and ball position on the field |
| **IR Sensor Board** | Tracks IR signals for ball localization |
| **Solenoid** | Used for the kicking mechanism |
| **Voltage Converters** | Regulate power between components |
| **Resistors & Capacitors** | Used for signal conditioning and stability |
| **Speeduino Board** | Used inline with the line and IR sensor board |
| **Wired Connections** | Connect power, control, and sensor data between all PCBs |

---

## 🧾 PCB Design

- All PCBs are **designed in KiCad**.  
- The main controller, IR sensor, and power PCBs are linked through wired connectors for reliability and ease of debugging.  
- Each board includes proper component labeling and trace routing optimized for signal integrity.
