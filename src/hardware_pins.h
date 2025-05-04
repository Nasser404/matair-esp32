// --- Add to main.cpp (or a new hardware_pins.h) ---

#ifndef HARDWARE_PINS_H
#define HARDWARE_PINS_H

// === Stepper Pins ===
// Stepper 1 (Capture)
const int CAPTURE_STEP_PIN = 23; // XX;
const int CAPTURE_DIR_PIN  = 19; // XX;
const int ENDSTOP_CAPTURE_PIN = 27; // XX;

// Stepper 2 (Cart)
const int CART_STEP_PIN = 18; // XX;
const int CART_DIR_PIN  = 5; // XX;
const int ENDSTOP_CART_PIN = 25; // XX;

// Stepper 3 (Orb)
const int ORB_STEP_PIN = 4; // XX;
const int ORB_DIR_PIN  = 0; // XX;
const int ENDSTOP_ORB_PIN = 26; // XX;

// === Servo Pins ===
const int ROTATION_SERVO_PIN = 14; // XX; // Servo1 in previous code
const int GRIPPER_SERVO_PIN = 12; // XX;  // Servo2 in previous code

// === Linear Actuator Pins ===
const int ACTUATOR_IN1_PIN = 32; // XX; // Motor Pin 1
const int ACTUATOR_IN2_PIN = 33; // XX; // Motor Pin 2

// === Manual Buttons ===
const int BUTTON_PIN_1 = 34; // XX;
const int BUTTON_PIN_2 = 35; // XX;

// === Nextion Serial ===
// Defined by EasyNex library (Serial2: RX=16, TX=17)

#endif // HARDWARE_PINS_H