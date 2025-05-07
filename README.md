# 🚲 Smart Bike System – Arduino Firmware Suite

This repository contains the Arduino firmware used in the Smart Bike System, developed for an engineering dissertation project. The firmware runs across **two microcontrollers** and enables data sensing, LED control, alarm triggering, and communication with a Raspberry Pi hub over serial connections.

---

## 🔧 Firmware Overview

| Firmware File | Board        | Core Functions |
|---------------|--------------|----------------|
| `main_controller.ino` | Arduino Uno (or Mega) | Manages LEDs, GPS, accelerometer, buzzer, light sensor, serial communication |
| `radar_ir_module.ino` | Arduino Nano (or Uno) | Manages radar motion detection and IR-based thermal presence sensing |

Each board communicates over serial USB to the Raspberry Pi. The Pi aggregates and interprets this data, relaying it to the GUI or a mobile device via MQTT.

---

## 🔋 Arduino 1 – Main Controller (`main_controller.ino`)

### ✅ Functions:
- **Serial Communication**:
  - Sends GPS, speed, altitude, and alarm status to the Raspberry Pi.
  - Receives brightness, lock/unlock commands, and mode from the Pi.
- **LED Control**:
  - Manages 8 LED strips (rear, logo, middle, and front – both sides).
  - Supports 4 operation modes including animated effects and safety cues.
- **Security Alarm**:
  - Accelerometer-based motion detection triggers a visual and audible alarm.
- **Light Sensing**:
  - Reads ambient brightness and adjusts LEDs for visibility and energy efficiency.
- **Indicator System**:
  - Button-based indicators trigger directional LED animations.

### 🔌 Pin Highlights:
- Uses **SoftwareSerial** to read from GPS module (TX/RX: A1/A2).
- LED strips connected across D2–D9.
- Buzzer output: D12.
- Light sensor input: A6.

---

## 📡 Arduino 2 – Radar and IR Module (`radar_ir_module.ino`)

### ✅ Functions:
- **Radar Detection**:
  - Communicates with a serial radar module to detect motion (approaching, departing, sustained).
- **IR Heat Mapping**:
  - Uses an 8×8 thermal sensor (AMG88xx) to detect presence in 4 vertical zones.
- **Combined Detection Logic**:
  - Only sends data to the Raspberry Pi if both radar and IR detect a meaningful event.
- **Serial Data Output**:
  - Sends motion type and segment-encoded IR data via serial using custom framing.

### 🔌 Pin Highlights:
- Uses `SoftwareSerial` (D8/D7) to communicate with the radar sensor.
- Uses I²C (Wire) for AMG88xx IR sensor.

---

## 📄 Communication Protocols

### 🔁 Serial Communication (Arduino ↔ Raspberry Pi)

**Main Arduino:**
- **TX** → Pi reads:
  - Longitude, Latitude, Altitude, Speed, Satellites, Date, Time, Alarm
- **RX** ← Pi sends:
  - Mode, Lock state, Alarm reset, Brightness values

**Radar Arduino:**
- Sends:
  - `Start Byte (0xAA)`
  - `Radar Reading (1–4)`
  - `IR Segment Mask (1 byte)`
  - `Padding Byte (0x00)`
  - `End Byte (0xFF)`

### 🧠 Data Integrity:
- Packets framed with **start (0xAA)** and **end (0xFF)** bytes.
- Radar uses **CRC-16** for reliable communication with the sensor module.

---

## 🧪 Operational Modes

| Mode | Description |
|------|-------------|
| 1    | Adaptive brightness based on ambient light |
| 2    | User-controlled brightness per zone |
| 3    | Enhanced visibility (minimum brightness enforced) |
| 4    | Party mode with choreographed LED sequences and animations |

---

## ⚠️ Alarm Logic

- Monitors motion via accelerometer.
- If excessive motion is detected **while locked**, the system:
  - Flashes all LEDs in pulsing amber.
  - Sounds the buzzer.
  - Transmits alarm state to Raspberry Pi and mobile device via MQTT.

---

## 🔮 Advanced Features

- **Indicator animation** plays sequential LED sweep for turning signals.
- **LED palette blending** and HSV color transitions for dynamic lighting.
- **Finale mode** includes randomized strobes, rainbow shimmer, and electric storm sequences.

---

## 📚 Integration in Dissertation

These firmware files are referenced in the **Hardware Implementation** and **Firmware Architecture** chapters of the dissertation. The serial communication protocol, LED control logic, and sensor integrations are each mapped to the project’s system block diagram and evaluated in the testing section.

---

## 🛠 Future Improvements

- Implement EEPROM saving for last-used mode or brightness settings.
- Extend heatmap tracking to detect directionality of IR movement.
- Introduce capacitive touch interface instead of buttons.

---

## 📦 Libraries Required

- `FastLED.h`
- `Wire.h`
- `TinyGPSPlus.h`
- `Adafruit_AMG88xx.h`
- `SoftwareSerial.h`
- `PinChangeInterrupt.h` (optional for interrupt handling)

---

For setup instructions, hardware wiring diagrams, and test results, refer to the dissertation’s **Appendix D: Embedded Code and Communication Flow**.
