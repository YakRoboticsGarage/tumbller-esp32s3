#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "../drivers/Motor.hpp"
#include "task_common.hpp"
#include "motor_task.hpp"

static void motorTask(void *pvParameters);

void motor_task_start() {
  g_motor.Pin_init();
  g_motor.Encoder_init();
  g_motor.Stop(0);
  
  // Start balancer if enabled
  if (g_balancerEnabled) {
    g_balancer.begin(&g_motor);
  }
  
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
      
      if (g_balancerEnabled) {
        // In balancer mode, set setpoints instead of direct motor control
        switch (msg.cmd) {
          case MotorCommand::Forward:
            g_balancer.setSetpoints(30, 0);  // forward speed setpoint
            motorState = MOTOR_STATE_STRINGS[0];
            motorRunning = true;
            motorTimeout = msg.timeoutMs;
            motorStartTime = millis();
            break;
          case MotorCommand::Back:
            g_balancer.setSetpoints(-30, 0);  // backward speed setpoint
            motorState = MOTOR_STATE_STRINGS[1];
            motorRunning = true;
            motorTimeout = msg.timeoutMs;
            motorStartTime = millis();
            break;
          case MotorCommand::Left:
            g_balancer.setSetpoints(0, -30);  // turn left setpoint
            motorState = MOTOR_STATE_STRINGS[2];
            motorRunning = true;
            motorTimeout = msg.timeoutMs;
            motorStartTime = millis();
            break;
          case MotorCommand::Right:
            g_balancer.setSetpoints(0, 30);  // turn right setpoint
            motorState = MOTOR_STATE_STRINGS[3];
            motorRunning = true;
            motorTimeout = msg.timeoutMs;
            motorStartTime = millis();
            break;
          case MotorCommand::Stop:
          default:
            g_balancer.setSetpoints(0, 0);  // hold position
            motorState = MOTOR_STATE_STRINGS[4];
            motorRunning = false;
            motorTimeout = 0;
            break;
        }
      } else {
        // Direct motor control (original behavior)
        switch (msg.cmd) {
          case MotorCommand::Forward:
            g_motor.Forward(MOTOR_SPEED);
            motorState = MOTOR_STATE_STRINGS[0];
            motorRunning = true;
            motorTimeout = msg.timeoutMs;
            motorStartTime = millis();
            break;
          case MotorCommand::Back:
            g_motor.Back(MOTOR_SPEED);
            motorState = MOTOR_STATE_STRINGS[1];
            motorRunning = true;
            motorTimeout = msg.timeoutMs;
            motorStartTime = millis();
            break;
          case MotorCommand::Left:
            g_motor.Left(MOTOR_SPEED);
            motorState = MOTOR_STATE_STRINGS[2];
            motorRunning = true;
            motorTimeout = msg.timeoutMs;
            motorStartTime = millis();
            break;
          case MotorCommand::Right:
            g_motor.Right(MOTOR_SPEED);
            motorState = MOTOR_STATE_STRINGS[3];
            motorRunning = true;
            motorTimeout = msg.timeoutMs;
            motorStartTime = millis();
            break;
          case MotorCommand::Stop:
          default:
            g_motor.Stop(0);
            motorState = MOTOR_STATE_STRINGS[4];
            motorRunning = false;
            motorTimeout = 0;
            break;
        }
      }
    }

    if (motorRunning && motorTimeout > 0 && (millis() - motorStartTime >= motorTimeout)) {
      if (g_balancerEnabled) {
        g_balancer.setSetpoints(0, 0);  // hold position
      } else {
        g_motor.Stop(0);
      }
      motorRunning = false;
      motorState = MOTOR_STATE_STRINGS[4];
#ifdef USE_SERIAL
      Serial.println("Motor stopped after time limit");
#endif
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}
