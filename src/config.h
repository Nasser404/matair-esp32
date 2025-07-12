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
const uint16_t orbTargets[8] = {4000, 3180, 2420, 1630, 840, 20, 5650, 4820};   // a-h
const uint16_t cartTargets[8] = {4480, 3850, 3200, 2520, 1960, 1360, 680, 0};     // ranks 1-8
const uint16_t captureTargets[32] = {
  2760, 2580, 2400, 2210, 2020, 1850, 1680, 1480,
  1310, 1120, 920, 740, 560, 380, 200, 30,
  6240, 6060, 5880, 5690, 5510, 5330, 5140, 4950,
  4780, 4590, 4430, 4220, 4050, 3860, 3680, 3500,
};

// --- Constants ---
const uint16_t STEPPER_SPEED = 4000;
const uint16_t STEPPER_ACCEL = 5000;
const uint16_t HOMING_SPEED_CAPTURE = 1000;
const uint16_t HOMING_SPEED_CART_ORB = 1000;
const uint16_t HOMING_ACCEL = 1500;

const uint8_t GRIPPER_ROT_BOARD = 172;
const uint8_t GRIPPER_ROT_CAPTURE = 63;
const uint16_t CART_SAFETY_THRESHOLD = 2250;
const uint16_t CART_CAPTURE_HOME_THRESHOLD = 800;
const uint16_t CART_CAPTURE_POS = 2250;

const uint8_t GripperOpen = 140;
const uint8_t GripperClose = 45;

const uint16_t ACTUATOR_TRAVEL_TIME_MS = 650;

const uint8_t CAPTURE_HOME_BACKUP_STEPS = 200;

// WITH BUTTON
const uint8_t ORB_MANUAL_MIN_POS = 10;
const uint16_t ORB_MANUAL_MAX_POS = 6000;
const uint16_t MANUAL_ORB_SPEED = 500;

// WITH NEXTION / Python App
const uint16_t MANUAL_JOG_CART_SPEED = 1500;
const uint16_t MANUAL_JOG_ORB_SPEED = 1000;
const uint16_t MANUAL_JOG_CAPTURE_SPEED = 800;
const uint8_t  MANUAL_JOG_SERVO_INCREMENT = 3;
