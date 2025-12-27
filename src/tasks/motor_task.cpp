#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "../drivers/Motor.hpp"
#include "task_common.hpp"
#include "motor_task.hpp"

static void motorTask(void *pvParameters);
static Motor motor;

void motor_task_start() {
  motor.Pin_init();
  motor.Stop(0);
  xTaskCreatePinnedToCore(motorTask, "motorTask", 4096, nullptr, 2, nullptr, 1);
}

static void motorTask(void *pvParameters) {
  (void)pvParameters;
  bool motorRunning = false;
  unsigned long motorStartTime = 0;
  unsigned long motorTimeout = 0;

  for (;;) {
    MotorCommandMsg msg;
    if (g_motorQueue && xQueueReceive(g_motorQueue, &msg, pdMS_TO_TICKS(5)) == pdPASS) {
      switch (msg.cmd) {
        case MotorCommand::Forward:
          motor.Forward(MOTOR_SPEED);
          motorState = MOTOR_STATE_STRINGS[0];
          motorRunning = true;
          motorTimeout = msg.timeoutMs;
          motorStartTime = millis();
          break;
        case MotorCommand::Back:
          motor.Back(MOTOR_SPEED);
          motorState = MOTOR_STATE_STRINGS[1];
          motorRunning = true;
          motorTimeout = msg.timeoutMs;
          motorStartTime = millis();
          break;
        case MotorCommand::Left:
          motor.Left(MOTOR_SPEED);
          motorState = MOTOR_STATE_STRINGS[2];
          motorRunning = true;
          motorTimeout = msg.timeoutMs;
          motorStartTime = millis();
          break;
        case MotorCommand::Right:
          motor.Right(MOTOR_SPEED);
          motorState = MOTOR_STATE_STRINGS[3];
          motorRunning = true;
          motorTimeout = msg.timeoutMs;
          motorStartTime = millis();
          break;
        case MotorCommand::Stop:
        default:
          motor.Stop(0);
          motorState = MOTOR_STATE_STRINGS[4];
          motorRunning = false;
          motorTimeout = 0;
          break;
      }
    }

    if (motorRunning && motorTimeout > 0 && (millis() - motorStartTime >= motorTimeout)) {
      motor.Stop(0);
      motorRunning = false;
      motorState = MOTOR_STATE_STRINGS[4];
#ifdef USE_SERIAL
      Serial.println("Motor stopped after time limit");
#endif
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}
