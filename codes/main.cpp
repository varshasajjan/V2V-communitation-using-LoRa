#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <NewPing.h>

#include "pins.h"
#include "config.h"
#include "tele.h"
#include "dtc.h"

// GPS serial
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);
TinyGPSPlus gps;

// Ultrasonic
NewPing sonar(US_TRIG, US_ECHO, US_MAX_CM);

// State
uint16_t seqCounter = 0;
float lastPeerLat = NAN, lastPeerLon = NAN;
int   lastRssi = -200;
float lastSnr  = -99.0;
unsigned long lastSendMs = 0;

// Helpers
static inline int16_t f2i10(float kmh) {
  long v = lround(kmh * 10.0);
  if (v < -32768) v = -32768;
  if (v >  32767) v =  32767;
  return (int16_t)v;
}

float cmToMeters(uint16_t cm) { return cm / 100.0; }

float haversineMeters(float lat1, float lon1, float lat2, float lon2) {
  const float R = 6371000.0;
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  float a = sin(dLat/2)*sin(dLat/2) +
            cos(radians(lat1))*cos(radians(lat2)) *
            sin(dLon/2)*sin(dLon/2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

void motorStop() {
  digitalWrite(M_IN1, LOW);
  digitalWrite(M_IN2, LOW);
  analogWrite(M_EN, 0);
}

void motorForward(uint8_t pwm) {
  digitalWrite(M_IN1, HIGH);
  digitalWrite(M_IN2, LOW);
  analogWrite(M_EN, pwm);
}

void warnBuzzer(bool on) { digitalWrite(BUZZER_PIN, on ? HIGH : LOW); }

void applyLoRa(const DtcParams& p) {
  LoRa.setSpreadingFactor(p.sf);
  LoRa.setTxPower(p.txPower);
  // LoRa.setSignalBandwidth(125E3);
  // LoRa.setCodingRate4(5);
}

void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(M_IN1, OUTPUT);
  pinMode(M_IN2, OUTPUT);
  pinMode(M_EN, OUTPUT);
  motorStop();
  warnBuzzer(false);

  Serial.begin(9600);
  gpsSerial.begin(9600);

  // LoRa init
  pinMode(LORA_RST, OUTPUT);
  digitalWrite(LORA_RST, HIGH); delay(10);
  digitalWrite(LORA_RST, LOW);  delay(10);
  digitalWrite(LORA_RST, HIGH); delay(10);

  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println(F("LoRa init failed. Check wiring/freq."));
    while (1) { delay(1000); }
  }
  LoRa.setSpreadingFactor(9);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setTxPower(14);

  Serial.println(F("V2V LoRa DTC (UNO) ready."));
}

void loop() {
  // GPS feed
  while (gpsSerial.available()) gps.encode(gpsSerial.read());
  bool hasFix = gps.location.isValid() && gps.location.isUpdated();
  float myLat = hasFix ? gps.location.lat() : NAN;
  float myLon = hasFix ? gps.location.lng() : NAN;
  float mySpeedKmh = gps.speed.isValid() ? gps.speed.kmph() : 0.0;

  // Ultrasonic
  uint16_t proxCm = sonar.ping_cm();
  if (proxCm == 0) proxCm = US_MAX_CM + 1;
  float proxM = cmToMeters(proxCm);

  // Receive LoRa
  int packetSize = LoRa.parsePacket();
  if (packetSize == sizeof(Telemetry)) {
    Telemetry rx;
    uint8_t *p = (uint8_t*)&rx;
    for (int i = 0; i < packetSize; i++) p[i] = LoRa.read();
    if (rx.carId != CAR_ID) {
      lastRssi = LoRa.packetRssi();
      lastSnr  = LoRa.packetSnr();
      lastPeerLat = rx.latE7 / 1e7;
      lastPeerLon = rx.lonE7 / 1e7;

      Serial.print(F("[RX] car=")); Serial.print(rx.carId);
      Serial.print(F(" RSSI=")); Serial.print(lastRssi);
      Serial.print(F(" SNR="));  Serial.println(lastSnr, 1);
    }
  } else {
    while (LoRa.available()) LoRa.read(); // flush unexpected sizes
  }

  // Peer distance
  float peerDistM = 9999.0;
  if (!isnan(myLat) && !isnan(myLon) && !isnan(lastPeerLat) && !isnan(lastPeerLon)) {
    peerDistM = haversineMeters(myLat, myLon, lastPeerLat, lastPeerLon);
  }

  // DTC
  DtcParams dtc = decideDtc(proxM, peerDistM, lastRssi, lastSnr);
  applyLoRa(dtc);

  // Safety actions
  bool warn = (proxM < WARN_HEADWAY_M) || (peerDistM < WARN_HEADWAY_M);
  bool danger = (proxM < DANGER_HEADWAY_M) || (peerDistM < DANGER_HEADWAY_M);

  if (danger) { warnBuzzer(true); motorStop(); }
  else if (warn) { warnBuzzer(true); motorForward(120); }
  else { warnBuzzer(false); motorForward(200); }

  // Transmit
  unsigned long now = millis();
  if (now - lastSendMs >= dtc.intervalMs) {
    Telemetry tx;
    tx.carId = CAR_ID;
    if (!isnan(myLat) && !isnan(myLon)) {
      tx.latE7 = (int32_t)lround(myLat * 1e7);
      tx.lonE7 = (int32_t)lround(myLon * 1e7);
    } else { tx.latE7 = 0; tx.lonE7 = 0; }
    tx.speed10 = (int16_t)lround(mySpeedKmh * 10.0);
    tx.proxCm  = proxCm;
    tx.seq     = ++seqCounter;

    LoRa.beginPacket();
    LoRa.write((uint8_t*)&tx, sizeof(Telemetry));
    LoRa.endPacket(false);

    lastSendMs = now;

    Serial.print(F("[TX] ")); Serial.print(CALLSIGN);
    Serial.print(F(" sf=")); Serial.print(dtc.sf);
    Serial.print(F(" pwr=")); Serial.print(dtc.txPower);
    Serial.print(F(" int=")); Serial.print(dtc.intervalMs);
    Serial.print(F(" proxCm=")); Serial.print(proxCm);
    Serial.print(F(" seq=")); Serial.println(tx.seq);
  }

  delay(5);
}