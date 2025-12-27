#pragma once

// Global project configuration: serial logging toggle and pin assignments.
#ifndef USE_SERIAL
// #define USE_SERIAL
#endif

#ifdef USE_SERIAL
  #define SERIAL_BEGIN(baud) Serial.begin(baud)
  #define SERIAL_PRINTLN(msg) Serial.println(msg)
#else
  #define SERIAL_BEGIN(baud) ((void)0)
  #define SERIAL_PRINTLN(msg) ((void)0)
#endif

// Motor driver pins (Arduino pin names map via board variant)
#define OE   A6
#define AIN1 7
#define PWMA_LEFT 5
#define BIN1 A0
#define PWMB_RIGHT 6
#define STBY_PIN 8

// Encoder pins
#define ENCODER_LEFT_A_PIN 2
#define ENCODER_RIGHT_A_PIN 4

// WiFi / mDNS hostname
#define WIFI_HOSTNAME "finland-tumbller-01"
