#ifndef BALANCER_HPP
#define BALANCER_HPP

#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"
#include "KalmanFilter.h"
#include "Motor.hpp"

// Orientation config: IMU rotated 90 deg clockwise, Y axis faces forward
// We treat forward tilt angle derived from (ay, az). Adjust if needed.

class Balancer {
public:
  struct Gains { float kp=55.0f, kd=0.75f; float kp_speed=10.0f, ki_speed=0.26f; float kp_turn=2.5f, kd_turn=0.5f; };

  void begin(Motor* motor);
  void stop();
  void setSetpoints(float forward, float turn);
  bool isUpright() const { return fabsf(_kf.angle) < _angleLimit; }
  float getAngle() const { return _kf.angle; }

private:
  static void taskEntry(void* arg);
  void runLoop();
  void calibrate(float dt);

  MPU6050 _mpu;
  KalmanFilter _kf;
  Motor* _motor = nullptr;
  Gains _gains;
  float _targetSpeed = 0.0f; // user set forward
  float _targetTurn = 0.0f;  // user set turn
  float _speedEstimate = 0.0f; // TODO: from encoders
  float _speedI = 0.0f;
  float _gyroBias = 0.0f;
  float _angleZero = 0.0f;
  float _angleLimit = 15.0f; // upright check
  float _angleTrip = 30.0f;  // safety cutoff
  TaskHandle_t _taskHandle = nullptr;
  bool _running = false;
};

#endif
