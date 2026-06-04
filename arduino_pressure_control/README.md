# arduino_pressure_control (PlatformIO)

## Overview
This module contains the hardware control code for the bidirectional pneumatic limb system.

- Implemented using Arduino framework via PlatformIO
- Designed for a 2-segment bidirectional pneumatic limb
- Main execution entry: `src/main.cpp`
- Additional testing and tuning scripts are provided under `test/`

> Note: This directory is part of a larger repository (including Python console and MATLAB controller), but this README focuses only on the hardware control module. 

---

## Project Main Structure

```text
arduino_pressure_control/
├── src/
│   └── main.cpp              # Main execution file
├── test/
|   ├── Unit Test/            # 3 unit tests
│   └── tuningPI.cpp          # PI controller tuning script
├── platformio.ini            # PlatformIO configuration
```

---

## Requirements

### VSCode Extensions

- PlatformIO IDE (required)
- Teleplot (recommended for PI tuning)

### platformio.ini

This repository includes a `platformio.ini` file configured for our hardware setup. If you use Arduino Mega2560, in most cases, no manual installation of libraries is required, as PlatformIO will automatically resolve and install dependencies.

If you are using a different microcontroller or board, you should modify the configuration accordingly. At minimum, make sure the following structure is preserved:

```ini
[env:customizable]
platform = <your_platform>
board = <your_board>
framework = arduino

lib_deps =
    adafruit/Adafruit MPRLS Library
    adafruit/Adafruit MCP4728@^1.0.10

monitor_echo = yes
monitor_filters = send_on_enter
monitor_speed = 115200
```

Notes: 
- `platform` and `board` must match your specific hardware (e.g., ESP32, Arduino Uno, etc.).
- `monitor_speed` should match the baud rate defined in the code (`Serial.begin(...)`). We recommend using 115200.
- Required libraries (`lib_deps`) will be automatically installed by PlatformIO.
- The serial monitor settings (`monitor_*`) are tuned for interactive command input and debugging.


---

## Installation

### Step 1: Install PlatformIO
We recommend installing PlatformIO via the VSCode Extension Marketplace. (refer to <https://docs.platformio.org/en/stable/integration/ide/vscode.html#installation>)

### Step 2: Open Project in VSCode (IMPORTANT)
This project must be opened as *a standalone folder* in VSCode.

- Do NOT open the entire large repository as the workspace
- Instead, add the `arduino_pressure_control/` folder directly into VSCode
  
Otherwise, PlatformIO may fail to initialize properly.

### Step 3: Initialize PlatformIO
After opening the folder, PlatformIO should automatically detect `platformio.ini` and set the configuration to *PlatformIO*. If not, check the bottom-right corner and set C/C++ configuration to PlatformIO manually.

---

## Usage
Open `src/main.cpp` in VSCode and use the PlatformIO toolbar (bottom-left bar):

- Build: click ✔️ (Ctrl+Alt+B)
- Upload: click ➡️ (Ctrl+Alt+U)
- Monitor: click 🔌 (Ctrl+Alt+S)


---

## PI Controller Tuning
The code provides a PI tuning file. PlatformIO only allows one active `main.cpp`. To run the tuning script:

- Save a backup of your `main.cpp`
- Copy `test/tuningPI.cpp`
- Replace `src/main.cpp`
- Build & upload as usual

You may use VSCode extension `Teleplot` to visualize the pressure, install it via VSCode extension marketplace. 

For more info of using the tuning script, refers to the notes in the beginning of the script.

---

## Code Structure Overview
Core classes:

- `ABValve`
- `MultiPressure`
- `ProportionalValveController`
- `PumpController`

Consider checking source files for implementation details.

---

## Optional: Hardware Bring-up Tests

During development, several test files were used to validate individual hardware components and core classes.

These tests are not required for running the final system, but may be useful if you are building the robot from scratch or debugging a new hardware setup.

Examples are in `test/Unit Test/`, refer to the file description block in the beginning of each script for more details.

---

## Known Issues & Tips
### 1. PlatformIO not recognized
- Check VSCode bottom-right corner
- Ensure configuration is set to PlatformIO
- Ensure the `arduino_pressure_control` is opened as a standalone folder.

### 2. Only main.cpp is executable
- PlatformIO does not support multiple entry points
- Rename or replace files accordingly

---


