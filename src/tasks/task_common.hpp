#pragma once

#include "../config.hpp"

#include <Arduino.h>
#include <SensirionI2cSht3x.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// Motor command types shared between tasks
enum class MotorCommand : uint8_t {
  Forward,
  Back,
  Left,
  Right,
  Stop
};

struct MotorCommandMsg {
  MotorCommand cmd;
  unsigned long timeoutMs; // how long to run before auto-stop; 0 means no timer
};

// Shared resources
extern QueueHandle_t g_motorQueue;
extern String motorState;
extern const char *const MOTOR_STATE_STRINGS[5];
extern SensirionI2cSht3x sensor;
extern bool sht3xReady;

// Motor behavior constants
constexpr int MOTOR_SPEED = 60;
constexpr unsigned long MOTOR_FORWARD_BACK_TIME = 2000; // ms
constexpr unsigned long MOTOR_TURN_TIME = 1000; // ms

// Initialize shared queue and state
void task_common_init();
