# Tumbller ESP32 Development Notes

## Project Overview

ESP32-S3 port of the AVR Tumbller self-balancing robot. The original AVR code is in the `Tumbller/` folder for reference.

## Architecture

```
setup()
  └─> task_common_init()      // Queue, I2C, SHT3x sensor
  └─> motor_task_start()
        ├─> Motor pin/encoder init
        └─> g_balancer.begin()  // If g_balancerEnabled=true
              ├─> MPU6050 init
              └─> Creates balancer task (core 1, 200Hz)
  └─> server_task_start()     // HTTP server for remote control
```

### FreeRTOS Tasks

| Task | Core | Priority | Rate | Purpose |
|------|------|----------|------|---------|
| balancer | 1 | 2 | 200Hz | Self-balancing control loop |
| motorTask | 1 | 2 | ~200Hz | Command queue processing |
| serverTask | 0 | 1 | - | HTTP request handling |

## Balancer State Machine

Matches AVR Tumbller startup sequence:

```
INIT (2s)          Motors OFF, gyro calibration
    │
    ▼
LEAN_BACK          Push backward to tip off support
    │              Duration = (angle - 30)² / 8 ms
    ▼
START (2s max)     Balance control active, wait for valid angle
    │
    ├─> angle in [-22°, +22°] ──> BALANCING
    │
    └─> timeout ──> FALLEN

BALANCING          Normal operation
    │
    └─> |angle| > 30° ──> FALLEN

FALLEN             Motors OFF, auto-recover when angle valid for 1s
```

## IMU Configuration

- **Orientation**: X-axis forward, Z-axis up
- **Tilt angle**: `atan2(ax, az)`
- **Pitch rate**: `gy` (for Kalman filter)
- **Yaw rate**: `gz` (for turn damping)

## Control Loop

```
┌─────────────────────────────────────────────────────────────┐
│                   applyBalanceControl()                     │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  1. ENCODER SPEED (200Hz)                                   │
│     - Read encoder counts, infer direction from PWM sign    │
│     - Low-pass filter: 0.7 * old + 0.3 * new                │
│                                                             │
│  2. BALANCE PD                                              │
│     balance = Kp * angle + Kd * angleRate                   │
│                                                             │
│  3. SPEED PI                                                │
│     speedOut = Kp_speed * error + Ki_speed * integral       │
│                                                             │
│  4. TURN PD (with gyro damping)                             │
│     turnOut = Kp_turn * target + Kd_turn * gyroZ            │
│                                                             │
│  5. MIXER                                                   │
│     leftPWM  = speedOut - balance - turnOut                 │
│     rightPWM = speedOut - balance + turnOut                 │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

## PID Gains (from AVR)

```cpp
// Balance PD
kp_balance = 55.0f, kd_balance = 0.75f

// Speed PI
kp_speed = 10.0f, ki_speed = 0.26f

// Turn PD
kp_turn = 2.5f, kd_turn = 0.5f
```

## Kalman Filter Parameters

```cpp
dt = 0.005f           // 5ms (200Hz)
Q_angle = 0.001f      // Process noise - angle
Q_gyro = 0.005f       // Process noise - gyro bias
R_angle = 0.5f        // Measurement noise
```

## Key Differences from AVR

| Aspect | AVR | ESP32 Port |
|--------|-----|------------|
| Speed control rate | 25Hz (every 8th cycle) | 200Hz (every cycle) |
| Gyro scale | 131 (±250 dps) | 65.5 (±500 dps) for both pitch and yaw |
| Gyro bias | Hardcoded 128.1 | Runtime calibration |
| K1 complementary filter | Defined but unused | Not implemented (dead code in AVR) |

## Files

### Core Balance Logic
- `src/drivers/Balancer.hpp` - State machine, gains, variables
- `src/drivers/Balancer.cpp` - Control loop, Kalman filter integration
- `src/drivers/KalmanFilter.h` - Sensor fusion (header-only)
- `src/drivers/MPU6050.h/.cpp` - IMU driver

### Motor Control
- `src/drivers/Motor.hpp/.cpp` - PWM output, pin control
- `src/drivers/Measuring_speed.cpp` - Encoder interrupts

### Tasks
- `src/tasks/task_common.cpp` - Shared state, queue, g_balancer instance
- `src/tasks/motor_task.cpp` - Command processing, balancer startup
- `src/tasks/server_task.cpp` - HTTP API

## I2C Devices

| Device | Address | Notes |
|--------|---------|-------|
| MPU6050 | 0x68 | IMU for balancing |
| SHT3x | 0x44 | Temperature/humidity sensor |

Both share the I2C bus (Wire). `Wire.begin()` is called in `task_common_init()` and redundantly in `Balancer::begin()`.

## HTTP API

Motor commands via HTTP when `g_balancerEnabled = true`:
- Sets balancer setpoints instead of direct motor control
- Forward/Back: speed setpoint ±30
- Left/Right: turn setpoint ±30
- Stop: both setpoints to 0

## Thread Safety

### Encoder Reads
Encoder counts are modified in ISR and read in the balancer task. Protected with:
```cpp
noInterrupts();
leftCount = Motor::encoder_count_left_a;
rightCount = Motor::encoder_count_right_a;
Motor::encoder_count_left_a = 0;
Motor::encoder_count_right_a = 0;
interrupts();
```

Encoder variables declared `volatile` in Motor.hpp and Measuring_speed.cpp.

### Known Limitations
- **Direction inference at low PWM**: When PWM is near zero but robot is still moving (inertia), the sign-based direction inference may be incorrect. Could add a deadband if this causes issues.

## TODOs / Future Work

- [ ] Tune PID gains for ESP32 (currently using AVR values)
- [ ] Add HTTP endpoint to adjust gains at runtime
- [ ] Add telemetry endpoint (angle, speed, state)
- [ ] Consider removing redundant `Wire.begin()` in Balancer::begin()
- [ ] Test encoder direction inference with actual hardware
- [ ] Add deadband for direction inference if low-PWM issues occur
