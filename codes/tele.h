#pragma once
#include <Arduino.h>

struct Telemetry {
  uint8_t  carId;
  int32_t  latE7;      // latitude * 1e7
  int32_t  lonE7;      // longitude * 1e7
  int16_t  speed10;    // km/h * 10
  uint16_t proxCm;     // ultrasonic cm
  uint16_t seq;        // packet counter
} __attribute__((packed));
