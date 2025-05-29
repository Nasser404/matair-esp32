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