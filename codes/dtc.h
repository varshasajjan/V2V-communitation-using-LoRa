#pragma once
#include <Arduino.h>

struct DtcParams {
  int sf;
  int txPower;
  unsigned long intervalMs;
};

DtcParams decideDtc(float myProxM, float peerDistM, int linkRssi, float linkSnr);