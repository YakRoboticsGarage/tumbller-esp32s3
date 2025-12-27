#include <Arduino.h>
#include <Wire.h>

#include "tasks/task_common.hpp"

QueueHandle_t g_motorQueue = nullptr;
String motorState;
SensirionI2cSht3x sensor;
bool sht3xReady = false;

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

  // Initialize SHT3x sensor (I2C)
  Wire.begin();
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
