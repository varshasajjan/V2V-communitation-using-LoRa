#pragma once
#include <Arduino.h>

#define CAR_ID     1
#define CALLSIGN   "CAR1"

// India ISM band (change for your region)
static const long LORA_FREQ = 866E6;

// Safety thresholds (meters)
static const float WARN_HEADWAY_M   = 2.0;
static const float DANGER_HEADWAY_M = 1.0;

// Beaconing / timing
static const unsigned long baseIntervalMs = 800;
static const unsigned long minIntervalMs  = 200;
static const unsigned long maxIntervalMs  = 2000;

// LoRa RF limits
static const int MIN_TX_PWR = 2;     // dBm
static const int MAX_TX_PWR = 17;
static const int MIN_SF     = 7;
static const int MAX_SF     = 12;