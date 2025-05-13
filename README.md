# Spherical Chess Robot - ESP32 Firmware

C++ firmware for an ESP32-based Spherical Chess Robot.
The robot is designed to physically execute chess moves on a spherical board, receiving game instructions from an online game server via WebSockets.
## Project Overview
The core functionality involves:
*   Connecting to a WebSocket server to receive game commands.
*   Interpreting chess moves
*   Controlling three stepper motors for precise movement of the cart (rail), orb (board rotation), and capture zone rotation.
*   Operating two servo motors for gripper rotation and gripping action.
*   Driving a linear actuator for vertical gripper movement.
*   Homing all steppers using limit switches for accurate positioning.
*   Managing a physical capture zone for storing captured pieces.
*   Providing a user interface via a Nextion touchscreen display.
*   Implementing safety checks to prevent mechanical damage.
*   Offering manual control options for testing and calibration.

## Hardware Components

1.  **MCU:** ESP32 (programmed using PlatformIO, `esp32dev` board profile).
2.  **Stepper Motors (3) with Drivers**
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
    *   Controlled by an L293N motor driver.
    *   Features internal mechanical limit switches. An additional sensor circuit (using a relay connected to one of the actuator's DC motor pins) provides feedback to the ESP32 (on GPIO 13) to confirm full retraction.
5.  **Limit Switches (3):** One for each stepper motor for homing.
6.  **Push Buttons (2):** Used for manual Orb Stepper rotation when the system is idle.
7.  **Display:** Nextion Touchscreen Display.
    *   Connected via Serial2 (ESP32: GPIO 16 (RX2), GPIO 17 (TX2)).
    *   Controlled using the `EasyNextionLibrary`.
8.  **Connectivity:** Onboard ESP32 WiFi module.
9.  **Power Supply:** 12V 60W.

## Pin Assignments

Refer to `src/hardware_pins.h` for detailed pin assignments. Key assignments include:

*   **Cart Stepper:** Step, Direction, Endstop
*   **Orb Stepper:** Step, Direction, Endstop
*   **Capture Stepper:** Step, Direction, Endstop
*   **Rotation Servo (Servo1):** Control Pin
*   **Gripper Servo (Servo2):** Control Pin
*   **Linear Actuator (L293N):** IN1, IN2
*   **Linear Actuator Retracted Sensor:** GPIO 13 (Active HIGH signal assumed)
*   **Manual Button 1 (Orb +):** GPIO 34
*   **Manual Button 2 (Orb -):** GPIO 35
*   **Nextion Serial2:** ESP32 Default Serial2 pins (GPIO16_RX, GPIO17_TX)

## Software & Libraries

*   **PlatformIO IDE:** For project management and building.
*   **Arduino Framework:** For ESP32 programming.
*   **`AccelStepper`:** For stepper motor control (acceleration and non-blocking movement).
*   **`ESP32Servo`:** For servo motor control.
*   **`ArduinoWebsockets`:** For WebSocket client communication with the game server.
*   **`ArduinoJson`:** For parsing and serializing JSON messages.
*   **`EasyNextionLibrary`:** For communication with the Nextion display.
*   **`Preferences`:** For storing WiFi credentials and server settings in ESP32 flash.

## Core Firmware Structure (`src/`)

*   **`main.cpp`:**
    *   Main setup and loop.
    *   WiFi and WebSocket connection management.
    *   WebSocket message handling (`handle_data`).
    *   Nextion display updates (`load_nextion_page`) and event handling (`triggerX` functions).
    *   Manual Orb rotation button logic.
    *   Global `Board` and `MotionController` instances.
    *   Coordination of logical board updates post-physical moves.
*   **`MotionController.h` / `MotionController.cpp`:**
    *   Manages all physical hardware (steppers, servos, actuator).
    *   Implements a non-blocking state machine (`MotionState`) for executing complex sequences:
        *   Homing all steppers.
        *   Standard piece moves ("DO" sequence).
        *   Capture sequences (moving a piece to the capture zone).
        *   Pawn Promotion (simplified: moves pawn, logical/visual update).
        *   Full Physical Board Reset sequence.
    *   Includes safety checks (e.g., gripper rotation for low cart positions, capture motor homing for very low cart positions).
    *   Handles actuator retract confirmation using the sensor
*   **`board.h` / `board.cpp`:**
    *   Represents the logical state of the chessboard (8x8 grid of `Piece` pointers).
    *   Methods for initializing the board, adding pieces, moving pieces logically, and printing the board state.
    *   Helper to convert board coordinates to/from algebraic notation.
*   **`piece.h`:**
    *   Defines the base `Piece` class and derived classes for each piece type (Pawn, Rook, Knight, Bishop, Queen, King).
    *   Includes `PieceType` and `PieceColor` enums.
    *   Methods for getting piece symbol, position, Nextion image ID.
*   **`enums.h`:**
    *   Defines various enums used throughout the project (e.g., `MESSAGE_TYPE` for WebSockets, `ORB_STATUS`, `SPECIAL_MOVES`, `NEXTION_PAGE`).
*   **`config.h`:**
    *   Project-specific configuration values (e.g., `ORB_ID`).
*   **`hardware_pins.h`:**
    *   Centralized definitions for all GPIO pin assignments.

## Setup & Usage

1.  **Hardware Assembly:** Connect all components according to `hardware_pins.h` (or modify to match).
2.  **PlatformIO:**
    *   Open the project in PlatformIO (VS Code).
    *   Select the correct `env` for your ESP32 board (e.g., `esp32dev`).
3.  **WiFi & Server Configuration:**
    *   On first boot, or if credentials are not set, the ESP32 may attempt to connect to default/empty credentials.
    *   Use the Nextion "Settings" screen (if implemented) to input your WiFi SSID, Password, WebSocket Server Host/IP, and Port. These are saved to ESP32 Preferences.
    *   Alternatively, you ca, hardcode initial credentials.
4.  **Nextion HMI:** Upload the corresponding `.tft` file to your Nextion display.
5.  **Build & Upload:** Build and upload the firmware to the ESP32.
6.  **Operation:**
    *   On boot, the robot will home itself.
    *   It will then attempt to connect to WiFi and the WebSocket server.
    *   The Nextion display should show connection status and the ORB Code.
    *   The robot waits for `MOVE` commands from the server.

## Troubleshooting
*   **Serial Monitor:** Use the Arduino Serial Monitor (baud rate 115200) for detailed debugging output from the ESP32.
*   **Stepper Motor Issues:** Check wiring, driver current, and `AccelStepper` settings.
*   **Servo Jitter:** Ensure a stable power supply for servos.
*   **WebSocket Disconnects:** Verify server address/port, network stability, and PING/PONG handling. The server should PING, ESP32 should PONG (handled by `client.poll()`).
*   **Nextion Communication:** Check Serial2 wiring (TX->RX, RX->TX) and baud rates.
