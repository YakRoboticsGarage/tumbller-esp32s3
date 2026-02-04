#ifndef KALMANFILTER_H
#define KALMANFILTER_H
#include <Arduino.h>

/**
 * 1D Kalman Filter for angle estimation using accelerometer + gyroscope fusion.
 * 
 * State vector: [angle, gyroBias]
 * - angle: estimated tilt angle (degrees)
 * - gyroBias: estimated gyroscope drift bias (deg/s)
 * 
 * The filter predicts angle from gyro integration, then corrects using
 * accelerometer-derived angle measurement.
 * 
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │                         KALMAN FILTER FLOW                              │
 * └─────────────────────────────────────────────────────────────────────────┘
 * 
 *    ┌──────────────┐      ┌──────────────┐
 *    │  Gyroscope   │      │Accelerometer │
 *    │  (deg/s)     │      │  (g-force)   │
 *    └──────┬───────┘      └──────┬───────┘
 *           │                     │
 *           ▼                     ▼
 *    ┌──────────────┐      ┌──────────────┐
 *    │ measuredGyro │      │atan2(ax,az)  │
 *    └──────┬───────┘      └──────┬───────┘
 *           │                     │
 *           │                     ▼
 *           │              ┌──────────────┐
 *           │              │measuredAngle │
 *           │              └──────┬───────┘
 *           │                     │
 *           ▼                     │
 *    ╔══════════════════════════════════════╗
 *    ║         1. PREDICT STEP              ║
 *    ╠══════════════════════════════════════╣
 *    ║  angle += (gyro - gyroBias) * dt     ║
 *    ║  P = F*P*F' + Q  (covariance update) ║
 *    ╚══════════════════╤═══════════════════╝
 *                       │
 *                       ▼
 *    ╔══════════════════════════════════════╗
 *    ║         2. UPDATE STEP               ║◄───── measuredAngle
 *    ╠══════════════════════════════════════╣
 *    ║  innovation = measuredAngle - angle  ║
 *    ║  S = H*P*H' + R  (innov. covariance) ║
 *    ║  K = P*H' / S    (Kalman gain)       ║
 *    ║  angle += K * innovation             ║
 *    ║  gyroBias += K * innovation          ║
 *    ║  P = (I - K*H) * P                   ║
 *    ╚══════════════════╤═══════════════════╝
 *                       │
 *                       ▼
 *    ┌──────────────────────────────────────┐
 *    │  OUTPUT: angle, angleRate            │
 *    │  (fused estimate, bias-corrected)    │
 *    └──────────────────────────────────────┘
 * 
 * Key Concepts:
 *   Innovation - The difference between measured and predicted angle.
 *                "How wrong was my prediction?"
 *                Large innovation → prediction was off → apply bigger correction
 *                Small innovation → prediction was good → apply smaller correction
 *                This is how the accelerometer "pulls back" when gyro drifts.
 * 
 *   Kalman Gain (K) - Scales how much innovation corrects the state (0 to 1).
 *                     K ≈ 1: trust measurement (accel) more
 *                     K ≈ 0: trust prediction (gyro) more
 * 
 * Tuning Parameters:
 *   Q_angle  (processNoiseAngle)    - Trust gyro less → increase
 *   Q_gyro   (processNoiseGyroBias) - Gyro bias drifts fast → increase  
 *   R_angle  (measurementNoise)     - Accel is noisy → increase
 */
class KalmanFilter {
public:
  /**
   * Run one iteration of the Kalman filter.
   * 
   * @param measuredAngle   Angle from accelerometer (degrees), e.g. atan2(ay, az)
   * @param measuredGyro    Raw gyroscope rate (deg/s)
   * @param dt              Time step (seconds)
   * @param processNoiseAngle   Process noise variance for angle state (tuning param)
   * @param processNoiseGyroBias Process noise variance for gyro bias (tuning param)
   * @param measurementNoise    Measurement noise variance for angle (tuning param)
   * @param observationGain     Observation matrix coefficient (typically 1.0)
   */
  void update(float measuredAngle, float measuredGyro, float dt,
              float processNoiseAngle, float processNoiseGyroBias,
              float measurementNoise, float observationGain = 1.0f) {
    
    // === PREDICT STEP ===
    // Predict angle using gyro (corrected for estimated bias)
    angle += (measuredGyro - gyroBias) * dt;
    
    // Predict error covariance: P = F*P*F' + Q
    // For this 1D case, simplified update of 2x2 covariance matrix
    float covDot[4];
    covDot[0] = processNoiseAngle - errorCovariance[0][1] - errorCovariance[1][0];
    covDot[1] = -errorCovariance[1][1];
    covDot[2] = -errorCovariance[1][1];
    covDot[3] = processNoiseGyroBias;
    
    errorCovariance[0][0] += covDot[0] * dt;
    errorCovariance[0][1] += covDot[1] * dt;
    errorCovariance[1][0] += covDot[2] * dt;
    errorCovariance[1][1] += covDot[3] * dt;
    
    // === UPDATE STEP ===
    // Innovation (measurement residual): y = z - H*x
    float innovation = measuredAngle - angle;
    
    // Innovation covariance: S = H*P*H' + R
    float PH_angle = observationGain * errorCovariance[0][0];
    float PH_bias  = observationGain * errorCovariance[1][0];
    float innovationCovariance = measurementNoise + observationGain * PH_angle;
    
    // Kalman gain: K = P*H' * S^-1
    float kalmanGainAngle = PH_angle / innovationCovariance;
    float kalmanGainBias  = PH_bias / innovationCovariance;
    
    // Update error covariance: P = (I - K*H) * P
    float PH_01 = observationGain * errorCovariance[0][1];
    errorCovariance[0][0] -= kalmanGainAngle * PH_angle;
    errorCovariance[0][1] -= kalmanGainAngle * PH_01;
    errorCovariance[1][0] -= kalmanGainBias * PH_angle;
    errorCovariance[1][1] -= kalmanGainBias * PH_01;
    
    // Update state estimate: x = x + K*y
    angle    += kalmanGainAngle * innovation;
    gyroBias += kalmanGainBias * innovation;
    
    // Compute corrected angular rate (for derivative control)
    angleRate = measuredGyro - gyroBias;
  }
  
  // Current state estimates (public for easy access)
  float angle = 0.0f;       // Estimated angle (degrees)
  float angleRate = 0.0f;   // Estimated angular velocity (deg/s), bias-corrected
  
private:
  float gyroBias = 0.0f;    // Estimated gyroscope bias (deg/s)
  
  // Error covariance matrix P (2x2)
  // P[0][0] = angle variance, P[1][1] = bias variance
  // P[0][1], P[1][0] = cross-covariance
  float errorCovariance[2][2] = {{ 1.0f, 0.0f }, { 0.0f, 1.0f }};
};

#endif
