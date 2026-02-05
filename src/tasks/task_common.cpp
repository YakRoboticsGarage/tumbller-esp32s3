#include <Arduino.h>
#include <Wire.h>

#include "tasks/task_common.hpp"

QueueHandle_t g_motorQueue = nullptr;
String motorState;
SensirionI2cSht3x sensor;
bool sht3xReady = false;
SemaphoreHandle_t g_i2cMutex = nullptr;  // Protects Wire bus

// Motor and balancer instances
Motor g_motor;
Balancer g_balancer;
bool g_balancerEnabled = true;  // Enable self-balancing by default

const char *const MOTOR_STATE_STRINGS[5] = {
    "FORWARD",
    "BACK",
    "LEFT",
    "RIGHT",
    "STOP"};

void task_common_init() {
  if (!g_motorQueue) {
    g_motorQueue = xQueueCreate(8, sizeof(MotorCommandMsg));
  }
  motorState = MOTOR_STATE_STRINGS[4];

  // Create I2C mutex BEFORE any I2C operations
  if (!g_i2cMutex) {
    g_i2cMutex = xSemaphoreCreateMutex();
  }

  // Initialize I2C bus with conservative settings for ESP32 + motor EMI
  Wire.begin();
  Wire.setClock(100000);   // 100kHz - slower is more reliable with EMI
  Wire.setTimeOut(50);     // 50ms timeout to prevent hangs

  // Initialize SHT3x sensor (I2C)
  sensor.begin(Wire, SHT30_I2C_ADDR_44);
  sensor.stopMeasurement();
  delay(1);
  sensor.softReset();
  delay(100);
  sht3xReady = true;
#ifdef USE_SERIAL
  Serial.println("SHT3x initialized");
#endif
}
