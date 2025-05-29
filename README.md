# Mat@ir - ESP32 Firmware

[![Language](https://img.shields.io/badge/language-C%2B%2B-blue.svg)](https://isocpp.org/)
[![Framework](https://img.shields.io/badge/framework-Arduino-00979D.svg)](https://www.arduino.cc/)
[![MCU](https://img.shields.io/badge/MCU-ESP32-E7352C.svg)](https://www.espressif.com/en/products/socs/esp32)

C++ firmware for an ESP32-based Spherical Chess Robot. This robot is designed to physically execute chess moves on a spherical board, receiving game instructions from an online game server via WebSockets.

## Table of Contents

- [Project Overview](#project-overview)
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Pin Assignments](#pin-assignments)
- [Software & Libraries](#software--libraries)
- [Firmware Architecture](#firmware-architecture)
- [Getting Started](#getting-started)
  - [Prerequisites](#prerequisites)
  - [Hardware Assembly](#hardware-assembly)
  - [Software Setup](#software-setup)
- [Configuration](#configuration)
  - [WiFi & Server Credentials](#wifi--server-credentials)
  - [Nextion HMI](#nextion-hmi)
- [Building & Uploading](#building--uploading)
- [Operation](#operation)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

## Project Overview

The Spherical Chess Robot translates digital chess moves from a remote server into physical actions on a custom-built spherical chessboard. It utilizes an ESP32 microcontroller to manage WebSocket communication, interpret game logic, and precisely control various motors and sensors to manipulate chess pieces.

## Features

*   **WebSocket Connectivity:** Connects to a remote game server to receive chess move commands.
*   **Chess Move Interpretation:** Parse game commands.
*   **Precise Motor Control:**
    *   Three stepper motors (cart, orb, capture zone) for accurate positioning.
    *   Two servo motors for gripper rotation and piece manipulation.
    *   One linear actuator for horizontal gripper movement.
*   **Automated Homing:** Employs limit switches for reliable homing of all stepper motors.
*   **Physical Capture Zone:** Manages a dedicated area for storing captured pieces.
*   **User Interface:** Nextion touchscreen display for status updates, configuration, and manual controls.
*   **Safety Mechanisms:** Implements checks to prevent mechanical collisions and damage.
*   **Manual Control:** Offers options via push buttons and Nextion display for testing and calibration.
*   **Persistent Configuration:** Stores WiFi and server settings in ESP32 flash memory.

## Hardware Requirements

1.  **Microcontroller:** ESP32 (`esp32dev` board profile).
2.  **Stepper Motors (3) with Drivers:**
    *   **Cart Stepper:** Moves the robot assembly along a rail.
        *   Limit Switch: For homing.
    *   **Orb Stepper:** Rotates the spherical chessboard.
        *   Limit Switch: For homing.
    *   **Capture Stepper:** Rotates a 32-tooth gear serving as the capture zone.
        *   Limit Switch: For homing.
3.  **Servo Motors (2):**
    *   **Rotation Servo (Servo1):** Rotates the entire gripper assembly.
    *   **Gripper Servo (Servo2):** Opens and closes the gripper fingers.
4.  **Linear Actuator (1):**
    *   Extends/retracts the gripper (approx. 50mm travel).
    *   Controlled by an L293N motor driver (or similar).
    *   Features internal mechanical limit switches.
    *   Additional retraction sensor circuit (relay-based) connected to ESP32 GPIO 13.
5.  **Limit Switches (3):** One dedicated to each stepper motor for homing routines.
6.  **Push Buttons (2):** For manual Orb Stepper rotation when the system is idle.
7.  **Display:** Nextion Touchscreen Display.
    *   Connected via Serial2 (ESP32: GPIO 16 (RX2), GPIO 17 (TX2)).
8.  **Connectivity:** Onboard ESP32 WiFi module.
9.  **Power Supply:** 12V, 60W (or as required by your components).

*(A detailed Fritzing diagram or connection schematic would be a valuable addition here or in a separate `docs/` folder.)*

## Pin Assignments

All GPIO pin assignments are centrally defined in `src/hardware_pins.h`. Refer to this file for the most up-to-date and detailed pinout.

Key assignments include:
*   **Cart Stepper:** `CART_STEP_PIN`, `CART_DIR_PIN`, `CART_ENDSTOP_PIN`
*   **Orb Stepper:** `ORB_STEP_PIN`, `ORB_DIR_PIN`, `ORB_ENDSTOP_PIN`
*   **Capture Stepper:** `CAPTURE_STEP_PIN`, `CAPTURE_DIR_PIN`, `CAPTURE_ENDSTOP_PIN`
*   **Rotation Servo (Servo1):** `ROTATION_SERVO_PIN`
*   **Gripper Servo (Servo2):** `GRIPPER_SERVO_PIN`
*   **Linear Actuator (L293N):** `ACTUATOR_IN1_PIN`, `ACTUATOR_IN2_PIN`
*   **Linear Actuator Retracted Sensor:** `ACTUATOR_RETRACTED_SENSOR_PIN` 
*   **Manual Orb Button +:** `BUTTON_PIN_1` 
*   **Manual Orb Button -:** `BUTTON_PIN_2` 
*   **Nextion Serial2:** ESP32 Default Serial2 pins (TX2: GPIO17, RX2: GPIO16)

## Software & Libraries

This project is built using the PlatformIO IDE with the Arduino framework for ESP32.

**Key Libraries:**
*   **`AccelStepper`:** For advanced stepper motor control (acceleration, deceleration, non-blocking movement).
*   **`ESP32Servo`:** For servo motor control on ESP32.
*   **`ArduinoWebsockets`:** For WebSocket client communication.
*   **`ArduinoJson`:** For efficient parsing and serialization of JSON messages.
*   **`EasyNextionLibrary`:** For simplified communication with the Nextion display.
*   **`Preferences`:** ESP32 library for storing data persistently in flash memory (e.g., WiFi credentials).

See `platformio.ini` for a complete list of dependencies and versions.

## Firmware Architecture

The firmware is organized within the `src/` directory:

*   **`main.cpp`:**
    *   Main application entry point (`setup()` and `loop()`).
    *   Manages WiFi and WebSocket connections.
    *   Handles incoming WebSocket messages (`handle_data()`).
    *   Interfaces with the Nextion display for UI updates (`load_nextion_page()`) and event handling (`triggerX()` functions).
    *   Implements logic for manual Orb rotation buttons.
    *   Instantiates and coordinates global `Board` and `MotionController` objects.
*   **`MotionController.h` / `MotionController.cpp`:**
    *   Encapsulates all hardware control logic (steppers, servos, linear actuator).
    *   Implements a non-blocking state machine (`MotionState`) for executing complex movement sequences:
        *   Homing all steppers (`HOMING`).
        *   Standard piece moves (`DO_MOVE_PICKUP`, `DO_MOVE_PLACE`).
        *   Capture sequences (`DO_CAPTURE_PICKUP`, `DO_CAPTURE_PLACE`).
        *   Pawn Promotion (simplified physical move, logical update).
        *   Full Physical Board Reset.
    *   Includes safety interlocks (e.g., gripper rotation restrictions at certain cart positions).
    *   Manages actuator retraction confirmation via the dedicated sensor.
*   **`board.h` / `board.cpp`:**
    *   Represents the logical state of the 8x8 chessboard using `Piece` objects.
    *   Provides methods for board initialization, piece manipulation (add, move), and state querying.
    *   Includes utilities for converting between board coordinates and algebraic notation.
*   **`piece.h`:**
    *   Defines the base `Piece` class and derived classes for each chess piece type (Pawn, Rook, Knight, Bishop, Queen, King).
    *   Contains `PieceType` and `PieceColor` enumerations.
    *   Provides methods for retrieving piece attributes (symbol, Nextion image ID).
*   **`enums.h`:**
    *   Centralizes various enumerations used throughout the project (e.g., `MESSAGE_TYPE`, `ORB_STATUS`, `SPECIAL_MOVES`, `NEXTION_PAGE`).
*   **`config.h`:**
    *   Contains project-specific compile-time configurations (e.g., `ORB_ID`).
*   **`hardware_pins.h`:**
    *   Defines all GPIO pin assignments for easy management and modification.

## Getting Started

### Prerequisites

*   Familiarity with C++ and Arduino programming.
*   PlatformIO IDE installed (typically within VS Code).
*   All hardware components listed under [Hardware Requirements](#hardware-requirements).
*   A Nextion display with the corresponding HMI project file (`.tft`).
*   A running instance of the [Mat@ir Game Server](https://github.com/Nasser404/matair-server).

### Hardware Assembly

1.  Carefully connect all motors, drivers, sensors, buttons, and the Nextion display to the ESP32 according to the pin assignments in `src/hardware_pins.h`.
    *   **Important:** Double-check wiring, especially for power and motor drivers, to prevent damage.
2.  Ensure stepper motor drivers are correctly configured for current limits suitable for your motors.
3.  Power the system using the specified power supply.

### Software Setup

1.  **Clone the Repository:**
    ```bash
    git clone https://github.com/Nasser404/matair-esp32
    cd matair-esp32
    ```
2.  **Open in PlatformIO:** Open the cloned project folder in VS Code with the PlatformIO extension installed.
3.  **Select Environment:** Ensure the correct PlatformIO environment for your ESP32 board is selected (e.g., `esp32dev` in `platformio.ini`).
4.  **Install Libraries:** PlatformIO should automatically detect and prompt to install the necessary libraries listed in `platformio.ini`. If not, you can install them manually via the PlatformIO Library Manager or by running `pio lib install`.

## Configuration

### WiFi & Server Credentials

*   The firmware uses the ESP32 `Preferences` library to store WiFi SSID, password, WebSocket server host/IP, and port.
*   **Initial Setup:**
    *   These settings can be entered via the "Settings" screen on the Nextion display (if your HMI project supports this).
    *   Alternatively, for initial testing, you might temporarily hardcode these values in `main.cpp` or a configuration file, but using the Nextion interface and Preferences is recommended for flexibility.
    *   The ESP32 will attempt to load saved credentials on boot.
### Nextion HMI

1.  Ensure you have the Nextion Editor software.
2.  Open your `.hmi` project file for the display.
3.  Compile the HMI project to generate a `.tft` file.
4.  Upload the `.tft` file to your Nextion display (SD card or direct serial connection).

## Building & Uploading

1.  **Connect ESP32:** Connect your ESP32 board to your computer via USB.
2.  **Build:** In PlatformIO, click the "Build" button (checkmark icon) or run `pio run` in the terminal.
3.  **Upload:** If the build is successful, click the "Upload" button (right arrow icon) or run `pio run --target upload`.


## Troubleshooting

*   **Serial Monitor:** The primary tool for debugging. Open the PlatformIO Serial Monitor (baud rate usually 115200, as set in `setup()`). Look for `Serial.println()` messages for status and errors.
*   **No WiFi/WebSocket Connection:**
    *   Verify WiFi SSID and password are correct via Nextion settings or `Preferences`.
    *   Ensure the WebSocket server IP/host and port are correctly configured.
    *   Check that the WebSocket server is running and accessible on the network.
    *   Monitor serial output for connection attempt details and error messages.
*   **Stepper Motor Issues (No movement, erratic movement, wrong direction):**
    *   **Wiring:** Triple-check motor phase wiring to the drivers and driver connections to the ESP32 (Step, Direction, Enable pins).
    *   **Power:** Ensure motors and drivers have adequate and stable power.
    *   **Driver Settings:** Verify microstepping and current limit settings on the stepper drivers.
    *   **`AccelStepper` Config:** Review speed, acceleration, and steps-per-revolution settings in `MotionController.cpp`.
    *   **Endstops:** Ensure limit switches are wired correctly and triggering reliably. Check logic levels (NO/NC).
*   **Servo Jitter or No Movement:**
    *   Ensure servos have a stable and sufficient power supply (often separate from ESP32 logic power).
    *   Check signal wire connection to the ESP32.
*   **Linear Actuator Issues:**
    *   Verify L293N wiring and power.
    *   Check the retraction sensor circuit and its connection to GPIO 13. Monitor serial output related to actuator state.
*   **Nextion Display Not Working or Gibberish:**
    *   Check Serial2 wiring: ESP32 TX2 (GPIO17) to Nextion RX, ESP32 RX2 (GPIO16) to Nextion TX.
    *   Ensure baud rates match between `EasyNextionLibrary` initialization in `main.cpp` and the Nextion HMI project settings.
    *   Confirm the correct `.tft` file is uploaded to the display.
*   **PONG Not Sent / WebSocket Disconnects:** The `client.poll()` in the main loop is responsible for handling PING/PONG with the server. Ensure this is not blocked by long-running synchronous code.
