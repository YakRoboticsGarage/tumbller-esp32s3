# AVR Tumbller vs ESP32 Control Loop Comparison

Side-by-side analysis of `Tumbller/BalanceCar.h` (AVR) and `src/drivers/Balancer.cpp` (ESP32).

## Critical Differences

| Aspect | AVR | ESP32 | Impact |
|--------|-----|-------|--------|
| **Balance Kp** | `55` | `25` | ESP32 response is ~half strength |
| **Speed PI rate** | 25 Hz (every 8th cycle) | 200 Hz (every cycle) | ESP32 runs 8x faster |
| **Speed integral formula** | `integral += speed + (-setpoint)` | `I += error * dt` | Completely different dynamics |
| **Speed integral clamp** | [-3000, 3000] | [-100, 100] | ESP32 saturates way earlier |
| **Speed output signs** | `-kp * speed - ki * integral` | `+kp * error + ki * I` | Opposite sign convention |
| **kp_turn** | Declared at 2.5 but **never used** | Multiplied by target | ESP32 turn is 2.5x stronger |
| **Encoder window** | 40ms (8 cycles accumulated) | 5ms (1 cycle) | ESP32 speed estimate much noisier |
| **Accel angle axis** | `atan2(ay, az)` | `atan2(ax, az)` | Different IMU orientation |
| **Gyro balance axis** | `gx` | `gy` | Matches accel axis swap |
| **Gyro bias** | Hard-coded `128.1` | Calibrated (500 samples) | ESP32 adapts to actual sensor |
| **D-term source** | Raw gyro (fixed bias only) | Kalman-corrected rate | ESP32 uses better estimate |
| **Angle trip (fall detect)** | 22 degrees | 30 degrees | ESP32 allows more tilt |
| **Timer mechanism** | AVR Timer2 ISR (precise 5ms) | FreeRTOS vTaskDelay (approximate) | ESP32 has jitter |

## Detailed Analysis

### 1. Balance PD Controller

**AVR** (`BalanceCar.h:86`):
```cpp
balance_control_output = kp_balance * (kalmanfilter_angle - angle_zero)
                       + kd_balance * (kalmanfilter.Gyro_x - angular_velocity_zero);
// kp_balance = 55, kd_balance = 0.75
```

**ESP32** (`Balancer.cpp`):
```cpp
float balance = _gains.kp * _kf.angle + _gains.kd * _kf.angleRate;
// kp = 25.0, kd = 0.75
```

Differences:
- **Kp is 55 vs 25** - the most obvious tuning difference
- AVR subtracts `angle_zero` in the PID formula; ESP32 subtracts `_angleZero` at the Kalman input (mathematically equivalent)
- AVR D-term uses raw gyro with hard-coded bias `128.1`; ESP32 uses Kalman-bias-corrected `angleRate`

### 2. Speed PI Controller (Biggest Difference)

**AVR** (`BalanceCar.h:88-100`) - runs every 8th cycle (25 Hz):
```cpp
speed_control_period_count++;
if (speed_control_period_count >= 8) {
    speed_control_period_count = 0;
    // Encoder pulses accumulated over 8 balance cycles (40ms)
    car_speed = (encoder_left_pulse_num_speed + encoder_right_pulse_num_speed) * 0.5;
    encoder_left_pulse_num_speed = 0;
    encoder_right_pulse_num_speed = 0;

    // Low-pass filter
    speed_filter = speed_filter_old * 0.7 + car_speed * 0.3;
    speed_filter_old = speed_filter;

    // Accumulator-style integral (NOT standard PI)
    car_speed_integeral += speed_filter;        // accumulate raw filtered speed
    car_speed_integeral += -setting_car_speed;  // subtract setpoint separately
    car_speed_integeral = constrain(car_speed_integeral, -3000, 3000);

    // Output with NEGATIVE signs
    speed_control_output = -kp_speed * speed_filter - ki_speed * car_speed_integeral;
}
```

**ESP32** (`Balancer.cpp`) - runs every cycle (200 Hz):
```cpp
// Encoder pulses from a single 5ms cycle
float carSpeed = (_encoderLeftAccum + _encoderRightAccum) * 0.5f;
_encoderLeftAccum = 0;
_encoderRightAccum = 0;
_speedFilter = _speedFilter * 0.7f + carSpeed * 0.3f;
_speedEstimate = _speedFilter;

// Standard textbook PI
float speedError = _targetSpeed - _speedEstimate;
_speedI += speedError * LOOP_DT;
_speedI = constrain(_speedI, -100.0f, 100.0f);
float speedOut = _gains.kp_speed * speedError + _gains.ki_speed * _speedI;
```

Key differences:
1. **Rate**: AVR runs at 25 Hz (decimated 8x). ESP32 runs at 200 Hz
2. **Encoder window**: AVR accumulates pulses over 40ms. ESP32 uses 5ms (noisier)
3. **Integral formula**: AVR uses `integral += speed; integral += -setpoint` (raw accumulator). ESP32 uses `I += error * dt` (textbook PI)
4. **Clamp**: AVR [-3000, 3000] vs ESP32 [-100, 100]
5. **Output sign**: AVR uses `-kp * speed - ki * integral`. ESP32 uses `+kp * error + ki * I`

The AVR integral is NOT a standard PI controller. It accumulates raw filtered speed values each 40ms cycle, effectively creating a position-like estimate. The `* dt` in the ESP32 version makes the values much smaller per iteration.

### 3. Turn Controller

**AVR** (`BalanceCar.h:101`):
```cpp
rotation_control_output = setting_turn_speed + kd_turn * kalmanfilter.Gyro_z;
// kp_turn = 2.5 is declared but NEVER USED here
```

**ESP32** (`Balancer.cpp`):
```cpp
float turnOut = _gains.kp_turn * _targetTurn + _gains.kd_turn * _gyroZ;
// kp_turn = 2.5 IS multiplied
```

The AVR has `kp_turn = 2.5` declared but the actual formula uses `setting_turn_speed` directly without multiplying by `kp_turn`. The ESP32 correctly multiplies, making its turn response 2.5x stronger per unit of turn command.

### 4. Motor Mix

Both identical in structure:
```cpp
left  = balance - speed_output - turn_output
right = balance - speed_output + turn_output
```

However, the values feeding into the mixer differ in sign and scale due to the speed PI differences described above.

### 5. Sensor Reading

| Aspect | AVR | ESP32 |
|--------|-----|-------|
| Accel angle | `atan2(ay, az)` | `atan2(ax, az)` |
| Gyro balance | `gx` | `gy` |
| Gyro bias | Hard-coded `128.1` | Dynamic calibration (500 samples) |
| Gyro scale | `131` (±250 dps) | `131.0f` (±250 dps) |
| Gyro Z (turn) | `-gz / 131` | `-(float)gz / 131.0f` |
| I2C error handling | None | Mutex + failure detection + bus recovery |

The axis swap (ay/gx → ax/gy) reflects a different IMU mounting orientation on the ESP32 board.

### 6. Kalman Filter

Mathematically identical. Same algorithm, same parameters:
- `Q_angle = 0.001`, `Q_gyro = 0.005`, `R_angle = 0.5`, `C_0 = 1`
- ESP32 renames variables for clarity but the computation is the same

One subtle difference: AVR's balance D-term uses `Gyro_x` (raw gyro with fixed bias), while ESP32 uses `_kf.angleRate` (Kalman-bias-corrected rate).

### 7. Loop Timing

**AVR**: Hardware Timer2 ISR, precisely every 5ms (200 Hz). Speed PI decimated to every 8th call (25 Hz).

**ESP32**: FreeRTOS `vTaskDelay(5ms)`, approximate 200 Hz (execution time adds to period). Speed PI runs every cycle (200 Hz) with no decimation.

### 8. State Machine

| AVR | ESP32 |
|-----|-------|
| `START` → `STANDBY` → `STOP` | `INIT` → `LEAN_BACK` → `START` → `BALANCING` → `FALLEN` |
| 7 mixed motion/balance states | 5 clean balance states |
| Lean-back via key command only | Automatic lean-back with dynamic duration |
| Fall threshold: 22° | Fall threshold: 30° (valid range still ±22°) |

## Recommended Changes to Match AVR

1. **Speed PI**: Decimate to every 8th cycle, use accumulator-style integral, match signs and clamp
2. **Balance Kp**: Change from 25 to 55
3. **kp_turn**: Set effective gain to 1.0 (matching AVR's unused declaration)
4. **Fall threshold**: Reduce `_angleTrip` from 30 to 22 to match AVR

## Reference

- AVR source: `Tumbller/BalanceCar.h` (lines 76-153)
- ESP32 source: `src/drivers/Balancer.cpp`
