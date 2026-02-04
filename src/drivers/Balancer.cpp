#include "Balancer.hpp"

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

void Balancer::runLoop() {
  const float dt = 0.005f; // 200 Hz
  const TickType_t delayTicks = pdMS_TO_TICKS((int)(dt * 1000));
  // Simple gyro bias/angle zero calibration
  calibrate(dt);
  for (;;) {
    if (!_running) { 
      _motor->Stop(0); 
      vTaskDelay(pdMS_TO_TICKS(50)); 
      continue; 
    }
    int16_t ax, ay, az, gx, gy, gz;
    _mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    // Angle around wheel axis: with Y forward, use atan2(ay, az)
    float angle_m = atan2f((float)ay, (float)az) * 57.29578f - _angleZero;
    // Gyro x/y selection: with 90CW rotation and Y forward, pitch rate around X? Original used gx.
    // Gyro config in MPU6050::initialize() uses +/-500 dps => scale 65.5 LSB/deg/s
    float gyro_m = ((float)gx - _gyroBias) / 65.5f;
    _kf.Kalman_Filter(angle_m, gyro_m, dt, 0.001f, 0.005f, 0.5f, 1.0f);

    // Balance PD on angle and angle rate
    float balance = _gains.kp * _kf.angle + _gains.kd * _kf.angle_dot;

    // Very simple speed PI placeholder using encoder counts if available (optional)
    float speedError = _targetSpeed - _speedEstimate;
    _speedI += speedError * dt;
    _speedI = constrain(_speedI, -100.0f, 100.0f);
    float speedOut = _gains.kp_speed * speedError + _gains.ki_speed * _speedI;

    float turnOut = _gains.kp_turn * _targetTurn; // simple P for now

    float leftCmd = speedOut - balance - turnOut;
    float rightCmd = speedOut - balance + turnOut;

    // Map to PWM, clamp
    int leftPWM = (int)constrain(leftCmd, -255.0f, 255.0f);
    int rightPWM = (int)constrain(rightCmd, -255.0f, 255.0f);

    // If fallen beyond limits, cut power
    if (fabsf(_kf.angle) > _angleTrip) { 
      _motor->Stop(0); 
      vTaskDelay(pdMS_TO_TICKS(50)); 
      continue; 
    }

    _motor->Drive(leftPWM, rightPWM);

    vTaskDelay(delayTicks);
  }
}

void Balancer::calibrate(float dt) {
  // quick average for gyro bias and angle zero
  const int n = 500;
  long gsum = 0; long asum = 0; long zsum = 0;
  for (int i=0;i<n;i++) {
    int16_t ax, ay, az, gx, gy, gz;
    _mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
    gsum += gx; asum += ay; zsum += az;
    vTaskDelay(pdMS_TO_TICKS((int)(dt*1000)));
  }
  _gyroBias = (float)(gsum / n);
  _angleZero = atan2f((float)(asum / n), (float)(zsum / n)) * 57.29578f;
}
