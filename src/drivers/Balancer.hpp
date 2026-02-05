#ifndef BALANCER_HPP
#define BALANCER_HPP

#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"
#include "KalmanFilter.h"
#include "Motor.hpp"

// Orientation config: X-axis forward, Z-axis up
// Tilt angle from atan2(ax, az), pitch rate from gy, yaw rate from gz

// ============================================================================
// PID TUNING GUIDE
// ============================================================================
// Use HTTP API to tune at runtime:
//   curl http://<hostname>/balance/kp/40    # Set kp to 40
//   curl http://<hostname>/balance/status   # View current gains and state
//
// Step 1: Balance PD (kp, kd) - Get robot to stand still
//   - Start with kp=30, kd=0.5
//   - If robot falls over slowly: increase kp
//   - If robot oscillates/vibrates: decrease kp or increase kd
//   - If robot is sluggish: decrease kd
//   - Goal: robot stands still without oscillation
//
// Step 2: Speed PI (kp_speed, ki_speed) - Resist being pushed
//   - Start with kp_speed=5, ki_speed=0.1
//   - Push robot gently - it should resist and return
//   - If it drifts: increase ki_speed
//   - If it oscillates when pushed: decrease kp_speed
//
// Step 3: Turn PD (kp_turn, kd_turn) - Smooth turning
//   - kp_turn controls turn response to commands
//   - kd_turn dampens rotation (gyro Z feedback)
//   - If turns are jerky: increase kd_turn
//   - If turns are slow: increase kp_turn
//
// Once tuned, update these defaults and reflash.
// ============================================================================

// PID gain defaults (from AVR Tumbller)
#define DEFAULT_KP          25.0f    // Balance P gain
#define DEFAULT_KD          0.75f    // Balance D gain
#define DEFAULT_KP_SPEED    10.0f    // Speed P gain
#define DEFAULT_KI_SPEED    0.26f    // Speed I gain
#define DEFAULT_KP_TURN     2.5f     // Turn P gain
#define DEFAULT_KD_TURN     0.5f     // Turn D gain (gyro damping)

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
  struct Gains {
    float kp = DEFAULT_KP;
    float kd = DEFAULT_KD;
    float kp_speed = DEFAULT_KP_SPEED;
    float ki_speed = DEFAULT_KI_SPEED;
    float kp_turn = DEFAULT_KP_TURN;
    float kd_turn = DEFAULT_KD_TURN;
  };

  void begin(Motor* motor);
  void stop();
  void setSetpoints(float forward, float turn);
  bool isUpright() const { return fabsf(_kf.angle) < _angleLimit; }
  float getAngle() const { return _kf.angle; }
  float getSpeedEstimate() const { return _speedEstimate; }
  BalanceState getState() const { return _state; }
  int getI2CFailures() const { return _i2cTotalFailures; }

  // State control for HTTP API
  void restart();                     // Reset to INIT state
  void fall();                        // Set to FALLEN state

  // Gains access for HTTP API
  Gains getGains() const { return _gains; }
  void setGains(const Gains& g) { _gains = g; }

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
  unsigned long _leanDuration = 0;
  int _leanDirection = 0;  // +1 = forward, -1 = backward
  bool _manualStop = false;  // Prevents auto-recovery when manually stopped

  // Encoder-based speed estimation (200Hz - full rate on ESP32)
  int _encoderLeftAccum = 0;
  int _encoderRightAccum = 0;

  // I2C diagnostics
  int _i2cTotalFailures = 0;
  float _speedFilter = 0.0f;

  // Turn damping
  float _gyroZ = 0.0f;

  // Track last PWM signs for encoder direction
  int _lastLeftPWM = 0;
  int _lastRightPWM = 0;
};

#endif
