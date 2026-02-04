#ifndef BALANCER_HPP
#define BALANCER_HPP

#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"
#include "KalmanFilter.h"
#include "Motor.hpp"

// Orientation config: X-axis forward, Z-axis up
// Tilt angle from atan2(ax, az), pitch rate from gy, yaw rate from gz

// State machine matching AVR Tumbller startup behavior
enum class BalanceState {
  INIT,      // Just started, motors off, gyro calibrating
  LEAN_BACK, // Pushing backward to lean off support
  START,     // Waiting for robot to enter valid angle range
  BALANCING, // Active self-balancing
  FALLEN     // Fell over, waiting to recover
};

class Balancer {
public:
  struct Gains { float kp=55.0f, kd=0.75f; float kp_speed=10.0f, ki_speed=0.26f; float kp_turn=2.5f, kd_turn=0.5f; };

  void begin(Motor* motor);
  void stop();
  void setSetpoints(float forward, float turn);
  bool isUpright() const { return fabsf(_kf.angle) < _angleLimit; }
  float getAngle() const { return _kf.angle; }
  BalanceState getState() const { return _state; }

private:
  static void taskEntry(void* arg);
  void runLoop();
  void calibrate(float dt);
  bool isInValidRange() const;
  void applyBalanceControl();

  MPU6050 _mpu;
  KalmanFilter _kf;
  Motor* _motor = nullptr;
  Gains _gains;
  float _targetSpeed = 0.0f; // user set forward
  float _targetTurn = 0.0f;  // user set turn
  float _speedEstimate = 0.0f; // Updated from encoder feedback in applyBalanceControl()
  float _speedI = 0.0f;
  float _gyroBias = 0.0f;
  float _angleZero = 0.0f;
  float _angleLimit = 15.0f; // upright check
  float _angleTrip = 30.0f;  // safety cutoff (AVR uses ~27)
  float _balanceAngleMin = -22.0f;  // valid balance range (from AVR)
  float _balanceAngleMax = 22.0f;
  TaskHandle_t _taskHandle = nullptr;
  bool _running = false;

  // State machine variables
  BalanceState _state = BalanceState::INIT;
  unsigned long _stateStartTime = 0;
  unsigned long _leanBackDuration = 0;

  // Encoder-based speed estimation (200Hz - full rate on ESP32)
  int _encoderLeftAccum = 0;
  int _encoderRightAccum = 0;
  float _speedFilter = 0.0f;

  // Turn damping
  float _gyroZ = 0.0f;

  // Track last PWM signs for encoder direction
  int _lastLeftPWM = 0;
  int _lastRightPWM = 0;
};

#endif
