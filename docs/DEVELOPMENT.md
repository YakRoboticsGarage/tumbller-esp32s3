# Tumbller ESP32 Development Notes

## Project Overview

ESP32-S3 port of the AVR Tumbller self-balancing robot. The original AVR code is in the `Tumbller/` folder for reference.

## Architecture

```
setup()
  └─> task_common_init()      // Queue, I2C mutex, SHT3x sensor
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

Matches AVR Tumbller startup sequence, adapted for bi-directional startup:

```
INIT (2s)          Motors OFF, gyro calibration
    │
    ▼
LEAN_BACK          Push to tip toward vertical
    │              • Detects current lean direction (forward or backward)
    │              • Spins wheels opposite to lean to tip toward upright
    │              • Duration = max(50, angle² / 4) ms
    │              • Skipped if already near vertical (|angle| < 15°)
    ▼
START (2s max)     Balance control active, wait for valid angle
    │
    ├─> angle in [-22°, +22°] ──> BALANCING
    │
    └─> timeout ──> FALLEN

BALANCING          Normal operation
    │
    └─> |angle| > 30° ──> FALLEN

FALLEN             Motors OFF
    │              Auto-recover when angle valid for 1s
    │              (unless manually stopped via /balance/stop)
```

### Startup from Any Lean Direction

The current robot hardware rests leaning **backward** when powered off. The LEAN_BACK state automatically detects the lean direction and compensates:

- **Leaning backward** (angle < 0): wheels spin backward → body tips forward
- **Leaning forward** (angle > 0): wheels spin forward → body tips backward

This allows the robot to start balancing from either resting position without code changes.

## IMU Configuration

- **Orientation**: X-axis forward, Z-axis up
- **Tilt angle**: `atan2(ax, az)`
- **Pitch rate**: `gy` (for Kalman filter)
- **Yaw rate**: `gz` (for turn damping)
- **Gyro scale**: ±250 dps (131 LSB/°/s) - matches AVR setting
- **Accel scale**: ±2g - matches AVR setting

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
│  5. MIXER (matches AVR: balance - speed - turn)             │
│     leftPWM  = balance - speedOut - turnOut                 │
│     rightPWM = balance - speedOut + turnOut                 │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

## PID Gains (defaults from AVR)

```cpp
// Balance PD
kp = 25.0f, kd = 0.75f

// Speed PI
kp_speed = 10.0f, ki_speed = 0.26f

// Turn PD
kp_turn = 2.5f, kd_turn = 0.5f
```

Tune at runtime via HTTP API (see below).

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
| Gyro scale | 131 LSB/°/s (±250 dps) | Same (±250 dps) |
| Accel scale | ±2g | Same (±2g) |
| Gyro bias | Hardcoded 128.1 | Runtime calibration |
| K1 complementary filter | Defined but unused | Not implemented (dead code in AVR) |
| I2C protection | N/A (single task) | Mutex for multi-task access |

## Files

### Core Balance Logic
- `src/drivers/Balancer.hpp` - State machine, gains, variables
- `src/drivers/Balancer.cpp` - Control loop, Kalman filter integration
- `src/drivers/KalmanFilter.h` - Sensor fusion (header-only)
- `src/drivers/MPU6050.h` - IMU driver (header-only)

### Motor Control
- `src/drivers/Motor.hpp/.cpp` - PWM output, pin control
- `src/drivers/Measuring_speed.cpp` - Encoder interrupts

### Tasks
- `src/tasks/task_common.cpp` - Shared state, queue, I2C mutex, g_balancer instance
- `src/tasks/motor_task.cpp` - Command processing, balancer startup
- `src/tasks/server_task.cpp` - HTTP API

## I2C Bus

| Device | Address | Notes |
|--------|---------|-------|
| MPU6050 | 0x68 | IMU for balancing (200Hz reads) |
| SHT3x | 0x44 | Temperature/humidity sensor |

### I2C Configuration
- Clock: 100kHz (conservative for EMI tolerance)
- Timeout: 50ms
- Protected by `g_i2cMutex` for thread safety

### I2C Failure Recovery
The balancer detects I2C failures (all-zero reads) and:
1. Increments failure counter (visible via `/balance/status`)
2. After 3 consecutive failures, resets the I2C bus
3. Kalman filter coasts on previous state during failures

## HTTP API

### Motor Commands
- `GET /motor/forward` - Move forward
- `GET /motor/back` - Move backward
- `GET /motor/left` - Turn left
- `GET /motor/right` - Turn right
- `GET /motor/stop` - Stop

### Balance Control
- `GET /balance/status` - JSON with state, angle, speed, gains, i2c_fails
- `GET /balance/start` - Restart (reset to INIT state)
- `GET /balance/stop` - Stop balancing (enter FALLEN, no auto-recover)

### PID Tuning (runtime)
- `GET /balance/kp/<value>` - Set balance P gain
- `GET /balance/kd/<value>` - Set balance D gain
- `GET /balance/kp_speed/<value>` - Set speed P gain
- `GET /balance/ki_speed/<value>` - Set speed I gain
- `GET /balance/kp_turn/<value>` - Set turn P gain
- `GET /balance/kd_turn/<value>` - Set turn D gain

### Other
- `GET /info` - Hostname and IP
- `GET /sensor/ht` - Temperature/humidity from SHT3x

## Thread Safety

### I2C Bus
Protected by `g_i2cMutex` (FreeRTOS mutex):
- Balancer task: 2ms timeout (time-critical)
- Server task: 100ms timeout (can wait)

### Encoder Reads
Encoder counts are modified in ISR and read in the balancer task:
```cpp
noInterrupts();
leftCount = Motor::encoder_count_left_a;
rightCount = Motor::encoder_count_right_a;
Motor::encoder_count_left_a = 0;
Motor::encoder_count_right_a = 0;
interrupts();
```

Encoder variables declared `volatile` in Motor.hpp and Measuring_speed.cpp.

## Hardware Notes

### Known Issues
- **I2C failures**: Can occur due to motor EMI or power supply noise
  - See `docs/I2C_TROUBLESHOOTING.md` for diagnosis and fixes
- **Direction inference at low PWM**: When PWM is near zero but robot is still moving (inertia), the sign-based direction inference may be incorrect

### Recommended Hardware
- External I2C pull-ups (4.7kΩ)
- Decoupling capacitors (100µF + 100nF) near GY-521
- Star ground topology (all grounds meet at battery)
- Separate LDOs for logic (3.3V) and motors

## Tests

- `test/test_hardware/` - Motor and IMU basic tests
- `test/test_imu_orientation/` - IMU orientation detection, I2C scan

Run with:
```bash
pio test -f test_imu_orientation --without-testing  # Build only
pio test -f test_imu_orientation                     # Build and upload
```

## TODOs / Future Work

- [ ] Tune PID gains for ESP32 (currently using AVR values)
- [ ] Test encoder direction inference with actual hardware
- [ ] Add deadband for direction inference if low-PWM issues occur
- [ ] Persist tuned gains to NVS (non-volatile storage)
- [ ] Add WebSocket for real-time telemetry streaming
