# I2C Troubleshooting - GY-521 (MPU6050)

## Problem Summary
- **Symptom:** High I2C failure count (~2646 failures observed)
- **Hardware Clue:** LED on GY-521 breakout blinks intermittently
- **Diagnosis:** MPU6050 is browning out or resetting during operation

## Current Setup
- ✅ Dedicated 3.3V LDO for logic
- ✅ Dedicated 5V LDO  
- ✅ Motors powered directly from battery (separate from logic)
- ✅ External I2C pull-up resistors installed

## Software Fixes Applied
Added I2C mutex protection to prevent bus contention between:
- Balancer task (Core 1, 200Hz MPU6050 reads)
- Server task (Core 0, occasional SHT3x sensor reads)

Files modified:
- `src/tasks/task_common.hpp` - Added `g_i2cMutex` declaration
- `src/tasks/task_common.cpp` - Created mutex, configured Wire with conservative settings
- `src/tasks/server_task.cpp` - Protected SHT3x reads with mutex
- `src/drivers/Balancer.cpp` - Protected MPU6050 reads with mutex

## Hardware Investigation Needed

### Questions to Answer
1. **GY-521 power source:** Is it powered from 3.3V or 5V LDO?
   - 5V to VCC → uses onboard regulator → ✅ recommended
   - 3.3V to VCC → bypasses onboard regulator → check if board supports this

2. **Ground topology:** How are grounds connected?
   ```
   GOOD (Star Ground):
        Battery GND ─────┬──► Motor Driver GND
                         ├──► LDO GND
                         └──► (single point, no daisy chain)
   
   BAD (Daisy Chain):
        Battery GND ──► Motor Driver GND ──► LDO GND ──► ESP32
                        (motor current creates ground bounce)
   ```

3. **I2C wire length:** How long are the wires to GY-521?
   - <10cm is ideal
   - >20cm may need lower I2C clock or better shielding

4. **Motor isolation test:** Does LED blink with motors physically disconnected?
   - If YES → power supply issue on logic side
   - If NO → EMI or ground loop from motors

## Potential Causes (Ranked by Likelihood)

### 1. Ground Loop / Ground Bounce
Motor current spikes create voltage differences in shared ground paths. The MPU6050 sees this as a power glitch.

**Fix:** Star ground - all grounds meet at ONE point (battery negative terminal).

### 2. EMI on I2C Lines
Long wires act as antennas picking up motor PWM noise.

**Fixes:**
- Keep I2C wires short (<10cm)
- Twist SDA/SCL/GND together
- Route away from motor wires
- Add 100Ω series resistors on SDA/SCL near ESP32

### 3. Insufficient Decoupling
LDO output capacitance may be inadequate for transients.

**Fixes:**
- Add 100µF electrolytic + 100nF ceramic at GY-521 VCC/GND
- Add 100µF at each LDO output
- Add 100nF ceramic across motor terminals

### 4. Motor Driver Back-EMF
Motor direction changes create voltage spikes that can couple through parasitic paths.

**Fixes:**
- Add flyback diodes if not present in motor driver
- Add snubber capacitors (100nF ceramic) across motor terminals

## Testing Commands

```bash
# Watch I2C failures in real-time
watch -n 0.5 'curl -s http://finland-tumbller-01.local/balance/status | jq .i2c_fails'

# Get full status
curl http://finland-tumbller-01.local/balance/status

# Stop balancing (motors off) to test
curl http://finland-tumbller-01.local/balance/stop

# Restart balancing
curl http://finland-tumbller-01.local/balance/start
```

## Hardware Shopping List (If Needed)
- [ ] 100µF electrolytic capacitors (x3-4)
- [ ] 100nF ceramic capacitors (x4-6)
- [ ] 100Ω resistors for I2C series termination (x2)
- [ ] Ferrite beads for power line filtering (optional)

## Next Steps
1. Answer the questions above
2. Test with motors disconnected to isolate the issue
3. Implement hardware fixes based on findings
4. Retest and monitor `i2c_fails` count
