#include "Balancer.hpp"

// ============================================================================
// CONSTANTS
// ============================================================================
#ifndef RAD_TO_DEG
#define RAD_TO_DEG        57.29578f    // 180 / PI
#endif
#define GYRO_SCALE_500DPS 65.5f        // LSB per deg/s at ±500 dps (32768/500)
#define LOOP_FREQ_HZ      200          // Control loop frequency
#define LOOP_DT           0.005f       // 1 / LOOP_FREQ_HZ (seconds)

// Kalman filter tuning (adjust based on sensor noise characteristics)
#define KF_PROCESS_NOISE_ANGLE     0.001f   // Q_angle: gyro integration drift
#define KF_PROCESS_NOISE_GYRO_BIAS 0.005f   // Q_gyro: gyro bias random walk
#define KF_MEASUREMENT_NOISE       0.5f     // R: accelerometer noise

// Balance point calibration
// This is the angle (in degrees) where the robot is perfectly balanced.
// Adjust this value based on your robot's center of gravity.
// Positive = IMU tilted forward when balanced, Negative = tilted backward
#define BALANCE_ANGLE_OFFSET       0.0f     // Fine-tune this for your robot!

// Startup timing (from AVR Tumbller)
#define INIT_DELAY_MS              2000     // Wait before lean-back
#define LEAN_BACK_PWM              110      // PWM value for lean-back motor pulse
#define START_TIMEOUT_MS           2000     // Max time to wait for valid angle in START
#define FALLEN_RECOVERY_MS         1000     // Time in valid range before recovering

// ============================================================================
// BALANCER CONTROL LOOP - ASCII DIAGRAM
// ============================================================================
/*
 *  ┌─────────────────────────────────────────────────────────────────────────┐
 *  │                    SELF-BALANCING CONTROL LOOP (200 Hz)                 │
 *  └─────────────────────────────────────────────────────────────────────────┘
 *
 *   User Commands                    Sensors
 *   (HTTP /motor/*)                  (MPU6050)
 *        │                               │
 *        ▼                               ▼
 *  ┌───────────┐                 ┌───────────────┐
 *  │ setpoints │                 │ Accel + Gyro  │
 *  │ forward,  │                 │  ax,ay,az     │
 *  │ turn      │                 │  gx,gy,gz     │
 *  └─────┬─────┘                 └───────┬───────┘
 *        │                               │
 *        │                               ▼
 *        │                       ┌───────────────┐
 *        │                       │ Kalman Filter │
 *        │                       │ (sensor fusion)│
 *        │                       └───────┬───────┘
 *        │                               │
 *        │                               ▼
 *        │                         angle, angleRate
 *        │                               │
 *        ▼                               ▼
 *  ┌─────────────────────────────────────────────────────┐
 *  │                  PID CONTROLLERS                    │
 *  │                                                     │
 *  │  ┌─────────────┐     ┌─────────────┐                │
 *  │  │  Speed PI   │     │ Balance PD  │                │
 *  │  │             │     │             │                │
 *  │  │ error =     │     │ output =    │                │
 *  │  │ target -    │     │ Kp*angle +  │                │
 *  │  │ estimate    │     │ Kd*angleRate│                │
 *  │  └──────┬──────┘     └──────┬──────┘                │
 *  │         │                   │                       │
 *  │         ▼                   ▼                       │
 *  │       speedOut           balance                    │
 *  │         │                   │      ┌─────────────┐  │
 *  │         │                   │      │  Turn P     │  │
 *  │         │                   │      │ Kp * turn   │  │
 *  │         │                   │      └──────┬──────┘  │
 *  │         │                   │             │         │
 *  │         └─────────┬─────────┴──────┬──────┘         │
 *  │                   │                │                │
 *  │                   ▼                ▼                │
 *  │             ┌──────────────────────────┐            │
 *  │             │     MIXER                │            │
 *  │             │ left  = speed-balance-turn│           │
 *  │             │ right = speed-balance+turn│           │
 *  │             └────────────┬─────────────┘            │
 *  └──────────────────────────┼──────────────────────────┘
 *                             │
 *                             ▼
 *                    ┌────────────────┐
 *                    │  Motor::Drive  │
 *                    │ leftPWM,       │
 *                    │ rightPWM       │
 *                    └────────┬───────┘
 *                             │
 *                             ▼
 *                    ┌────────────────┐
 *                    │    MOTORS      │
 *                    │   ◄──────►     │
 *                    │  Left   Right  │
 *                    └────────────────┘
 *
 *  Why "speedOut - balance"?
 *  ─────────────────────────
 *  - Robot tilts forward (angle > 0) → balance > 0 → motors reverse to catch up
 *  - Robot tilts backward (angle < 0) → balance < 0 → motors go forward
 *  - The robot "falls forward" in the direction it wants to move
 */

void Balancer::begin(Motor* motor) {
  _motor = motor;
  Wire.begin();
  _mpu.initialize();
  _running = true;
  _taskHandle = nullptr;
  // Create 200 Hz control loop pinned to core 1
  xTaskCreatePinnedToCore(&Balancer::taskEntry, "balancer", 4096, this, 2, &_taskHandle, 1);
}

void Balancer::stop() { _running = false; }

void Balancer::setSetpoints(float forward, float turn) {
  _targetSpeed = constrain(forward, -100.0f, 100.0f);
  _targetTurn = constrain(turn, -100.0f, 100.0f);
}

void Balancer::taskEntry(void* arg) { static_cast<Balancer*>(arg)->runLoop(); }

bool Balancer::isInValidRange() const {
  return _kf.angle >= _balanceAngleMin && _kf.angle <= _balanceAngleMax;
}

void Balancer::applyBalanceControl() {
  // ─────────────────────────────────────────────────────────────────────
  // BALANCE PD CONTROLLER
  // ─────────────────────────────────────────────────────────────────────
  float balance = _gains.kp * _kf.angle + _gains.kd * _kf.angleRate;

  // ─────────────────────────────────────────────────────────────────────
  // SPEED PI CONTROLLER
  // ─────────────────────────────────────────────────────────────────────
  float speedError = _targetSpeed - _speedEstimate;
  _speedI += speedError * LOOP_DT;
  _speedI = constrain(_speedI, -100.0f, 100.0f);
  float speedOut = _gains.kp_speed * speedError + _gains.ki_speed * _speedI;

  // ─────────────────────────────────────────────────────────────────────
  // TURN P CONTROLLER
  // ─────────────────────────────────────────────────────────────────────
  float turnOut = _gains.kp_turn * _targetTurn;

  // ─────────────────────────────────────────────────────────────────────
  // MIX OUTPUTS → MOTOR COMMANDS
  // ─────────────────────────────────────────────────────────────────────
  float leftCmd = speedOut - balance - turnOut;
  float rightCmd = speedOut - balance + turnOut;

  int leftPWM = (int)constrain(leftCmd, -255.0f, 255.0f);
  int rightPWM = (int)constrain(rightCmd, -255.0f, 255.0f);

  _motor->Drive(leftPWM, rightPWM);
}

void Balancer::runLoop() {
  const TickType_t delayTicks = pdMS_TO_TICKS((int)(LOOP_DT * 1000));

  // Initialize state machine
  _state = BalanceState::INIT;
  _stateStartTime = millis();

  // Calibrate gyro during INIT phase (robot must be still but can be tilted)
  calibrate(LOOP_DT);

  for (;;) {
    if (!_running) {
      _motor->Stop(0);
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    // ─────────────────────────────────────────────────────────────────────
    // READ SENSORS (always, regardless of state)
    // ─────────────────────────────────────────────────────────────────────
    int16_t ax, ay, az, gx, gy, gz;
    _mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Convert to physical units (X-axis forward, Z-axis up)
    float measuredAngle = atan2f((float)ax, (float)az) * RAD_TO_DEG - _angleZero;
    float measuredGyro = ((float)gy - _gyroBias) / GYRO_SCALE_500DPS;

    // ─────────────────────────────────────────────────────────────────────
    // SENSOR FUSION (Kalman Filter)
    // ─────────────────────────────────────────────────────────────────────
    _kf.update(measuredAngle, measuredGyro, LOOP_DT,
               KF_PROCESS_NOISE_ANGLE, KF_PROCESS_NOISE_GYRO_BIAS, KF_MEASUREMENT_NOISE);

    // ─────────────────────────────────────────────────────────────────────
    // STATE MACHINE (matches AVR Tumbller startup sequence)
    // ─────────────────────────────────────────────────────────────────────
    switch (_state) {
      case BalanceState::INIT:
        // Motors off, waiting for gyro calibration to settle
        _motor->Stop(0);
        if (millis() - _stateStartTime > INIT_DELAY_MS) {
          // Calculate lean-back duration from current angle (AVR formula)
          float angleDiff = _kf.angle - 30.0f;
          _leanBackDuration = (unsigned long)(angleDiff * angleDiff / 8.0f);
          _state = BalanceState::LEAN_BACK;
          _stateStartTime = millis();
        }
        break;

      case BalanceState::LEAN_BACK:
        // Push backward to tip robot off its forward support
        _motor->Drive(-LEAN_BACK_PWM, -LEAN_BACK_PWM);
        if (millis() - _stateStartTime > _leanBackDuration) {
          _motor->Stop(0);
          _state = BalanceState::START;
          _stateStartTime = millis();
        }
        break;

      case BalanceState::START:
        // Balance control active, waiting for robot to stabilize
        applyBalanceControl();
        if (isInValidRange()) {
          _speedI = 0;  // Reset integrator for clean start
          _state = BalanceState::BALANCING;
        } else if (millis() - _stateStartTime > START_TIMEOUT_MS) {
          // Timed out without reaching valid angle - give up
          _motor->Stop(0);
          _state = BalanceState::FALLEN;
          _stateStartTime = millis();
        }
        break;

      case BalanceState::BALANCING:
        // Normal operation - full balance control
        applyBalanceControl();
        if (fabsf(_kf.angle) > _angleTrip) {
          // Fell over - cut motors
          _motor->Stop(0);
          _state = BalanceState::FALLEN;
          _stateStartTime = millis();
        }
        break;

      case BalanceState::FALLEN:
        // Motors off, waiting for recovery
        _motor->Stop(0);
        if (isInValidRange()) {
          // Robot is back in valid range - check if stable
          if (millis() - _stateStartTime > FALLEN_RECOVERY_MS) {
            _speedI = 0;  // Reset integrator
            _state = BalanceState::BALANCING;
          }
        } else {
          // Still fallen - reset timer
          _stateStartTime = millis();
        }
        break;
    }

    vTaskDelay(delayTicks);
  }
}

void Balancer::calibrate(float dt) {
  // Calibrate gyro bias only - robot can be tilted during this phase!
  // The robot will auto-balance from any starting angle within _angleTrip.
  //
  // NOTE: Robot must be STATIONARY (not moving) but can rest on a support.
  //       Gyro measures rotation rate, so stillness is all that matters.
  
  const int numSamples = 500;  // ~2.5 seconds at 5ms intervals
  long gyroSum = 0;
  
  for (int i = 0; i < numSamples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    _mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    gyroSum += gy;
    vTaskDelay(pdMS_TO_TICKS((int)(dt * 1000)));
  }
  
  _gyroBias = (float)(gyroSum / numSamples);
  
  // Use fixed balance point offset instead of measuring at startup
  // This allows robot to start tilted and automatically balance
  _angleZero = BALANCE_ANGLE_OFFSET;
}
