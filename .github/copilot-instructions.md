# Rotary Inverted Pendulum - AI Coding Agent Instructions

## Project Overview

Arduino-based rotary inverted pendulum control system. Hardware: Arduino Mega 2560, TMC2209 stepper driver (24V), 17HS4401-D motor, two AS5600 magnetic encoders. Control: Bang-bang swing-up + PID balance. See `README.md` for hardware specs.

## Critical Architecture Patterns

### Dual-Loop Control Structure
**DO NOT break this pattern** - it's essential for smooth motor operation:

```cpp
// Main loop: Continuous stepper updates at MCU speed
void loop() {
  if (currentState == STATE_SWING_UP || currentState == STATE_BALANCE) {
    stepper.run();  // MUST call every iteration - generates step pulses
  }
  
  // Control calculations at fixed 100Hz
  if (millis() - lastControlTime >= CONTROL_LOOP_MS) {
    controlTick();  // Updates stepper.moveTo() targets
  }
}
```

**Why**: AccelStepper generates ONE step per `run()` call. Throttling this loop = slow motor regardless of `setMaxSpeed()`. Control math (sensor reads, PID) runs slower (100Hz) to avoid timing jitter.

### Sensor Reading Pattern - Always Use Median Filter
Both AS5600 sensors suffer from EMI corruption during motor operation. **Never read sensors directly**:

```cpp
// WRONG - susceptible to EMI bit flips
uint16_t raw = as5600_pendulum.readAngle();

// CORRECT - 3-sample median filter (standard pattern)
uint16_t raw = readAngleMedian(as5600_pendulum);
```

Located in `updateSensors()` - both Hardware I2C (pendulum, pins 20/21) and Software I2C (motor, pins 22/24) use this. Median rejects single-sample spikes, delays only 300µs total. **Do not remove filtering** unless you've solved the hardware EMI issue.

### Angle Wraparound Handling - Use normalizeAngleDelta()
AS5600 outputs 0-360°. **Naive derivatives break near boundaries**:

```cpp
// WRONG - 179° → -179° gives -358° derivative spike
float velocity = (pendulumAngle - previousPendulumAngle) / dt;

// CORRECT - wraps to shortest path
float delta = normalizeAngleDelta(pendulumAngle - previousPendulumAngle);
float velocity = delta / dt;
```

Use `normalizeAngleDelta()` for ANY angle difference calculation. Applied in both `runSwingUp()` and `runBalance()`.

## State Machine Flow

**Mandatory sequence** before control starts:
1. Diagnostics (Option 1) → Verify sensors + motor
2. Set Zero (Option 2) → Records `pendulumZeroAngle`, `motorZeroAngle`, `stepsAtZero`
3. Calibrate Limits (Option 4) → Sets `stepsAtLeftLimit`, `stepsAtRightLimit`, computes `motorStepsPerDeg`

**Control modes** - mutually exclusive:
- `STATE_SWING_UP`: Bang-bang energy pumping, switches to BALANCE when `|α| < 12°, |α̇| < 1.5 rad/s, base centered`
- `STATE_BALANCE`: PID on pendulum angle + motor centering term, exits to IDLE if `|α| > 60°`

State transitions happen in `controlTick()` and `runBalance()`. **Do not add control logic outside these functions** - breaks the fixed-rate timing.

## Hardware Safety - 6-Layer Limit Enforcement

Limits are **hardcoded** in `stepsAtLeftLimit` / `stepsAtRightLimit` during calibration. System has multiple redundant checks:

1. **Constrain targets** in `runBalance()` - clamps `targetSteps` before `moveTo()`
2. **Mode-specific guards** - both swing-up and balance check limits
3. **Global watchdog** in `controlTick()` - independent of control logic
4. **Manual jog protection** - calibration menu respects limits
5. **Emergency stop** - forces `MODE_IDLE` if position exceeds limits by >2 steps
6. **AccelStepper stopping** - **CRITICAL**: Use `stepper.setSpeed(0.0)` NOT `stepper.stop()` when using `runSpeed()` mode

**If you add new motor commands**, wrap targets with limit checks:
```cpp
long targetSteps = /* your calculation */;
if (targetSteps < stepsAtLeftLimit) targetSteps = stepsAtLeftLimit;
if (targetSteps > stepsAtRightLimit) targetSteps = stepsAtRightLimit;
stepper.moveTo(targetSteps);
```

## Motion Profiles - Context-Specific Speed/Accel

Three profiles defined at top of `main.cpp`:
- **High performance** (12k steps/s, 18k accel): General moves, jogging
- **Swing-up** (20k steps/s, 80k accel): Aggressive energy pumping
- **Balance** (15k steps/s, 40k accel): Precise control, lower speed for stability

Apply via `applyMotionProfile(speed, accel)` before mode changes. **Do not set `setMaxSpeed()` directly** - use these wrappers to maintain consistency. Diagnostics uses low speed (1000 steps/s) for validation.

## Development Workflow

### Build & Upload (PlatformIO)
```bash
pio run                    # Compile only
pio run --target upload    # Compile + upload to Arduino
pio device monitor         # Serial monitor (115200 baud)
```

Or use VS Code tasks: `PlatformIO: Build`, `PlatformIO: Upload` (see `platformio.ini`).

### Serial Menu Interface
**All control happens via serial commands** - no autonomous startup. Menu printed by `printMenu()` in STATE_IDLE. Commands parsed in `parseSerialCommand()` - non-blocking architecture. **If adding new commands**, follow the pattern:
```cpp
case 'N': case 'n':
  // Your logic here
  currentState = STATE_YOUR_NEW_STATE;
  return;  // Important - don't fall through
```

### Calibration Workflow (Must Do After Code Changes)
1. Run Diagnostics (1) - verify sensors respond
2. Set Zero (2) - hold pendulum up, press Z
3. **Skip limits if unchanged** - calibration persists across restarts if `stepsAtZero`, `stepsAtLeftLimit`, `stepsAtRightLimit` hardcoded
4. Live Monitor (3) - watch for sensor noise/drift before trusting control

**Motor sensor drift debugging**: Compare `motorAngle` (from sensor) vs `(stepper.currentPosition() - stepsAtZero) * 1.8°` (from steps). If diverging >5°, check EMI shielding or increase median filter samples.

## Control Tuning - Live Adjustment (24V Configuration)

Balance gains are **constants** at top of file. Current values tuned for 24V, lightweight PLA build:
```cpp
const float Kp_balance = 4.0f;   // Reaction strength to pendulum angle
const float Kd_balance = 0.8f;   // Damping (derivative)
const float Ki_balance = 0.05f;  // Eliminate steady-state error (integral)
const float Kp_motor = 0.03f;    // Motor centering (prevents drift)
```

**If changing power supply voltage** (12V ↔ 24V):
- 24V → 12V: Reduce Kp by ~60%, increase Kd by ~80%, reduce speed/accel by 3×
- 12V → 24V: Increase Kp by ~150%, reduce Kd by ~40%, increase speed/accel by 3×

See `TUNING_GUIDE.md` for systematic tuning process. **Start with balance test (Option B)** before swing-up - safer for gain validation.

## Key Files Reference

- **`src/main.cpp`**: All control logic, state machine, sensor interface (967 lines - read control sections 430-730)
- **`platformio.ini`**: Library dependencies (AS5600, AccelStepper, SoftwareWire), build config
- **`docs/PinConfiguration.md`**: Hardware wiring, power supply specs, capacitor placement
- **`TUNING_GUIDE.md`**: Control gain tuning, performance expectations, emergency procedures
- **`Log.md`**: Detailed development history, bug fixes, hardware modifications

## Common Pitfalls

1. **AccelStepper `stop()` doesn't work with `runSpeed()` mode** - use `stepper.setSpeed(0.0)` + `thetaDesiredSteps = motorSteps` to halt
2. **Don't throttle `stepper.run()` calls** - motor speed = call frequency × step timing, not just `setMaxSpeed()`
3. **Angle derivatives without wraparound handling** - use `normalizeAngleDelta()` or get 358° spikes
4. **Direct sensor reads during motor operation** - always use median filter, EMI causes bit flips
5. **Changing control gains without re-calibrating** - zero position and limits are relative to current motor state
6. **Using `moveTo()` without limit checks** - bypasses safety, can damage hardware (pendulum hits table)

## Hardware-Specific Notes

- **TMC2209 Vref**: 2.11V (~2.4A motor current). Lower = less torque but cooler operation.
- **24V power supply**: Required for current speed settings. 12V works but needs gain retuning (see v4.5 in `Log.md`).
- **Aluminum foil shielding**: I2C wires shielded with foil + drain wire to Arduino GND. **Don't remove** - reduces EMI corruption from 20% to <1%.
- **AS5600 motor sensor**: Optional - system uses step counting as "virtual encoder". Sensor only for drift detection/correction.

## Testing Strategy

**Before committing control changes**:
1. Compile and check for warnings
2. Upload and run Diagnostics (Option 1)
3. Test balance mode FIRST (Option B) - safer, easier to validate gains
4. Use Live Monitor (Option 3) to watch for sensor issues
5. Document gain changes in commit message with performance notes

**If motor behavior is erratic**: Check sensor readings in Live Monitor before debugging control logic. 90% of "control issues" are actually sensor noise or missed steps.
