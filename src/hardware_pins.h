#ifndef HARDWARE_PINS_H
#define HARDWARE_PINS_H

// === Stepper Pins ===

// Stepper 1 (Capture)
const uint8_t CAPTURE_STEP_PIN      = 23; 
const uint8_t CAPTURE_DIR_PIN       = 19; 
const uint8_t ENDSTOP_CAPTURE_PIN   = 27; 

// Stepper 2 (Cart)
const uint8_t CART_STEP_PIN     = 18; 
const uint8_t CART_DIR_PIN      = 5; 
const uint8_t ENDSTOP_CART_PIN  = 25; 

// Stepper 3 (Orb)
const uint8_t ORB_STEP_PIN      = 4; 
const uint8_t ORB_DIR_PIN       = 0; 
const uint8_t ENDSTOP_ORB_PIN   = 26; 

// === Servo Pins ===
const uint8_t ROTATION_SERVO_PIN    = 14;  // Servo1 
const uint8_t GRIPPER_SERVO_PIN     = 12;  // Servo2

// === Linear Actuator Pins ===
const uint8_t ACTUATOR_IN1_PIN              = 32;  // Motor Pin 1
const uint8_t ACTUATOR_IN2_PIN              = 33;  // Motor Pin 2
const uint8_t ACTUATOR_RETRACTED_SENSE_PIN  = 13; 
// === Manual Buttons ===
const uint8_t BUTTON_PIN_1 = 34; 
const uint8_t BUTTON_PIN_2 = 35; 

// === Nextion Serial ===
// (Serial2: RX=16, TX=17)

#endif // HARDWARE_PINS_H