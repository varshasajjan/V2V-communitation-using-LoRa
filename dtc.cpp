#include "dtc.h"
#include "config.h"

DtcParams decideDtc(float myProxM, float peerDistM, int linkRssi, float linkSnr) {
  DtcParams p;
  p.sf = 9;
  p.txPower = 14;
  p.intervalMs = baseIntervalMs;

  // Link-quality adaptation
  if (linkRssi < -115 || linkSnr < -7) {
    p.sf = min(MAX_SF, p.sf + 2);
    p.txPower = min(MAX_TX_PWR, p.txPower + 2);
  } else if (linkRssi < -100) {
    p.sf = min(MAX_SF, p.sf + 1);
    p.txPower = min(MAX_TX_PWR, p.txPower + 1);
  } else if (linkRssi > -90 && linkSnr > 7) {
    p.sf = max(MIN_SF, p.sf - 1);
    p.txPower = max(MIN_TX_PWR, p.txPower - 1);
  }

  // Proximity urgency
  if (myProxM < DANGER_HEADWAY_M || peerDistM < DANGER_HEADWAY_M) {
    p.intervalMs = max(minIntervalMs, baseIntervalMs / 3);
    p.sf = min(MAX_SF, p.sf + 1);
  } else if (myProxM < WARN_HEADWAY_M || peerDistM < WARN_HEADWAY_M) {
    p.intervalMs = max(minIntervalMs,  baseIntervalMs / 3UL);        // /3

  } else {
    p.intervalMs = min(maxIntervalMs, (baseIntervalMs * 3UL) / 2UL); // 1.5x

  }

  return p;
}