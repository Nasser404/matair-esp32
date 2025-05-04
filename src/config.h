#pragma once
#include <Arduino.h>

// maximum lengths (including null terminator)
#define SSID_MAX_LEN    32
#define PWD_MAX_LEN     64
#define HOST_MAX_LEN    32
// Orb ID can remain const:
static const String ORB_ID = "ORB IVRY";