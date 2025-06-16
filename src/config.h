#pragma once
#include <Arduino.h>


#define SSID_MAX_LEN    32
#define PWD_MAX_LEN     64
#define HOST_MAX_LEN    32

static const String  ORB_ID         = "ORB IVRY";
static const String  DEFAULT_SSID   = "";
static const String  DEFAULT_PWD    = "";
static const String  DEFAULT_HOST   = "127.0.0.1";
const uint32_t       DEFAULT_PORT   = 29920;

// --- POSITION CONFIG ----
const uint16_t orbTargets[8] = {
    3550, // a
    2750, // b
    1950, // c
    1150, // d
    350,  // e
    5950, // f
    5150, // g
    4350  // h
};
const uint16_t cartTargets[8] = {4450, 3850, 3270, 2670, 2000, 1380, 680, 0};     // ranks 1-8
const uint16_t captureTargets[32] = {
  2760, 2580, 2400, 2220, 2040, 1860, 1680, 1500,
  1300, 1130, 940, 760, 580, 400, 220, 0,
  6260, 6080, 5890, 5710, 5530, 5350, 5170, 4970,
  4800, 4610, 4430, 4240, 4050, 3870, 8680, 3510
};
// --- Constants --- 
const uint16_t STEPPER_SPEED = 4000;
const uint16_t STEPPER_ACCEL = 5000;
const uint16_t HOMING_SPEED_CAPTURE = 1000;
const uint16_t HOMING_SPEED_CART_ORB = 1000;
const uint16_t HOMING_ACCEL = 1500;

const uint8_t GRIPPER_ROT_BOARD = 180;
const uint8_t GRIPPER_ROT_CAPTURE = 62;
const uint16_t CART_SAFETY_THRESHOLD = 2250;
const uint16_t CART_CAPTURE_HOME_THRESHOLD = 800;
const uint16_t CART_CAPTURE_POS = 2250;

const uint8_t GripperOpen = 160;
const uint8_t GripperClose = 50;

const uint16_t ACTUATOR_TRAVEL_TIME_MS = 650;


const uint8_t CAPTURE_HOME_BACKUP_STEPS = 200; 

// WITH BUTTON
const uint8_t ORB_MANUAL_MIN_POS = 10;
const uint16_t ORB_MANUAL_MAX_POS = 6000;
const uint16_t MANUAL_ORB_SPEED = 500;

// WITH NEXTION
const uint16_t MANUAL_JOG_CART_SPEED = 1500;
const uint16_t MANUAL_JOG_ORB_SPEED = 1000;
const uint16_t MANUAL_JOG_CAPTURE_SPEED = 800;
const uint8_t   MANUAL_JOG_SERVO_INCREMENT = 3; // Degrees per jog step for servos