#ifndef HARDWARE_PINS_H
#define HARDWARE_PINS_H

// === Stepper Pins ===

// Stepper 1 (Capture)
const int CAPTURE_STEP_PIN = 23; 
const int CAPTURE_DIR_PIN  = 19; 
const int ENDSTOP_CAPTURE_PIN = 27; 

// Stepper 2 (Cart)
const int CART_STEP_PIN = 18; 
const int CART_DIR_PIN  = 5; 
const int ENDSTOP_CART_PIN = 25; 

// Stepper 3 (Orb)
const int ORB_STEP_PIN = 4; 
const int ORB_DIR_PIN  = 0; 
const int ENDSTOP_ORB_PIN = 26; 

// === Servo Pins ===
const int ROTATION_SERVO_PIN = 14;  // Servo1 
const int GRIPPER_SERVO_PIN = 12;   // Servo2

// === Linear Actuator Pins ===
const int ACTUATOR_IN1_PIN = 32;  // Motor Pin 1
const int ACTUATOR_IN2_PIN = 33;  // Motor Pin 2
const int ACTUATOR_RETRACTED_SENSE_PIN = 13; 
// === Manual Buttons ===
const int BUTTON_PIN_1 = 34; 
const int BUTTON_PIN_2 = 35; 

// === Nextion Serial ===
// (Serial2: RX=16, TX=17)

#endif // HARDWARE_PINS_H