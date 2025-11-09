#pragma once

// LoRa (RFM95/SX1278) → adjust if your wiring differs
#define LORA_SS   10
#define LORA_RST  9
#define LORA_DIO0 2

// GPS over SoftwareSerial
#define GPS_RX    4   // Arduino RX  (to GPS TX)
#define GPS_TX    3   // Arduino TX  (to GPS RX)

// Ultrasonic HC-SR04
#define US_TRIG   6
#define US_ECHO   5
#define US_MAX_CM 400

// Buzzer
#define BUZZER_PIN 7

// L298N motor
#define M_IN1  8
#define M_IN2  12
#define M_EN   11     // PWM