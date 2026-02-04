#pragma once

#include "../config.hpp"

#include <Arduino.h>
#include <SensirionI2cSht3x.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "../drivers/Motor.hpp"
#include "../drivers/Balancer.hpp"

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

// Motor and balancer instances
extern Motor g_motor;
extern Balancer g_balancer;
extern bool g_balancerEnabled;  // true = use balancer, false = direct motor control

// Motor behavior constants
constexpr int MOTOR_SPEED = 60;
constexpr unsigned long MOTOR_FORWARD_BACK_TIME = 2000; // ms
constexpr unsigned long MOTOR_TURN_TIME = 1000; // ms

// Initialize shared queue and state
void task_common_init();
