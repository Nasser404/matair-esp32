# Mat@ir - ESP32 Firmware

[![Language](https://img.shields.io/badge/language-C%2B%2B-blue.svg)](https://isocpp.org/)
[![Framework](https://img.shields.io/badge/framework-Arduino-00979D.svg)](https://www.arduino.cc/)
[![MCU](https://img.shields.io/badge/MCU-ESP32-E7352C.svg)](https://www.espressif.com/en/products/socs/esp32)

C++ firmware for an spherical Chess Robot. This robot is designed to physically execute chess moves on a spherical board, receiving game instructions from an online game server via WebSockets.

## Table of Contents


- [Software Setup](#software-setup)
- [Configuration](#configuration)
  - [WiFi & Server Credentials](#wifi--server-credentials)
  - [Nextion HMI](#nextion-hmi)
- [Building & Uploading](#building--uploading)
- [Operation](#operation)
- [Troubleshooting](#troubleshooting)


This project is built using the PlatformIO IDE with the Arduino framework for ESP32.

**Libraries:**
*   **`AccelStepper`:** 
*   **`ESP32Servo`:**
*   **`ArduinoWebsockets`:** 
*   **`ArduinoJson`:** 
*   **`EasyNextionLibrary`:**
*   **`Preferences`:** 



## Software Setup

1.  **Clone the Repository:**
    ```bash
    git clone https://github.com/Nasser404/matair-esp32
    cd matair-esp32
    ```
2.  **Open in PlatformIO:** Open the cloned project folder in VS Code with the PlatformIO extension installed.
3.  **Select Environment:** Ensure the correct PlatformIO environment for your ESP32 board is selected ( `esp32dev`).
4.  **Install Libraries:** PlatformIO should automatically detect and prompt to install the necessary libraries listed in `platformio.ini`.

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
*   **No WiFi/WebSocket Connection:**
    *   Verify WiFi SSID and password are correct via Nextion settings or `Preferences`.
    *   Ensure the WebSocket server IP/host and port are correctly configured.
    *   Check that the WebSocket server is running and accessible on the network.
    *   Monitor serial output for connection attempt details and error messages.
