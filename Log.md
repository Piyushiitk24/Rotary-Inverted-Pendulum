# Rotary Inverted Pendulum - Development Log

## November 4, 2025

### Hardware Setup Completed
- **Arduino**: Mega 2560 (ATmega2560)
- **Motor Driver**: TMC2209 (Step/Direction interface)
- **Stepper Motor**: 200 steps/revolution (1.8° per step)
- **Sensors**: 2x AS5600 magnetic encoders
  - Pendulum sensor on Hardware I2C (pins 20/21)
  - Motor sensor on Software I2C (pins 22/24)

### Motor Configuration
- **Control Pins**: Step=5, Dir=6, Enable=7
- **Motor Settings**: 
  - Speed: 1000 steps/sec
  - Acceleration: 2000 steps/sec²
- **Direction Mapping**: 
  - CCW (counter-clockwise) = positive steps = "right"
  - CW (clockwise) = negative steps = "left"

### Sensor Characterization
- **Both sensors**: Decreasing angle with CW rotation, increasing with CCW
- **Pendulum Sensor**: Mounted on Hardware I2C, magnet embedded in cylindrical pendulum arm
- **Motor Sensor**: Mounted on Software I2C, reads motor shaft angle

### Calibration Achievements
- **Zero Position**: Successfully set with pendulum upright and arm centered
- **Physical Limits**: ±280 steps from center (560 total steps = 1008° range)
- **Step Tracking**: Working reliably using AccelStepper position counter
- **Manual Jogging**: Implemented direct digitalWrite pulse generation (bypasses AccelStepper for single steps)

### Issues Discovered & Resolved

#### Issue 1: AccelStepper Not Moving Motor
**Problem**: AccelStepper `run()` and `runToPosition()` methods weren't generating step pulses for single-step movements.

**Solution**: Created `manualStep()` function that directly pulses STEP pin:
```cpp
void manualStep(int direction) {
  digitalWrite(EN_PIN, LOW);
  digitalWrite(DIR_PIN, direction > 0 ? HIGH : LOW);
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(STEP_PIN, LOW);
  stepper.setCurrentPosition(stepper.currentPosition() + direction);
}
```

#### Issue 2: Severe Sensor Noise & Erratic Readings
**Problem**: Pendulum sensor showing massive jumps (±300°) even when stationary. Initial tests showed 20-60% error rate with readings jumping randomly.

**Root Causes Identified**:
1. **Missing I2C Pull-up Resistors** (CRITICAL)
2. **Motor power supply noise** coupling into sensor readings
3. **Magnet distance** initially too close causing saturation

**Solutions Implemented**:
1. ✅ Added 5kΩ pull-up resistors (4 total):
   - Pin 20 (SDA) to 5V
   - Pin 21 (SCL) to 5V  
   - Pin 22 (SDA) to 5V
   - Pin 24 (SCL) to 5V

2. ✅ Adjusted magnet distance to ~2-3mm from AS5600 chip

**Results After Pull-ups**:
- **Motor power OFF**: 56 stable readings, only 2 large jumps (96% stable!)
- **Motor power ON**: Still noisy (0-4 stable readings, 21-32 jumps) due to power supply noise

### Diagnostic Tools Created
- **Live Sensor Test**: Real-time angle monitoring with delta tracking
- **EMI Test**: Motor driver disable test to isolate noise sources
- **I2C Bus Scanner**: Detects all devices on both I2C buses
- **Step-based Calibration**: Interactive jogging with A/D keys

### Code Architecture
**Main Script**: `main.cpp` - Step-based calibration system
- Menu-driven interface
- Manual jogging with immediate feedback
- Step-based limit recording
- Safety warnings and emergency stop (press 'S')

**Backup Scripts**:
- `main_step_calibration.cpp.bak` - Working calibration (current)
- `main_full_pid_control.cpp.bak` - Full PID balance control (pre-calibration)
- `main_angle_calibration.cpp.bak` - Failed angle-based approach (sensor wraparound issues)

### Tomorrow's Work (November 5, 2025)

#### Hardware Modifications Required

**Add Capacitors for Noise Reduction** (CRITICAL):

**Available Capacitors (from user's collection)**:
- 100µF / 50V electrolytic (polarized - RED/ORANGE)
- 10µF / 50V electrolytic (polarized - BLUE)
- 1µF / 63V electrolytic (polarized - RED/ORANGE)

**Detailed Installation Guide**: See `docs/CapacitorInstallationGuide.md` for complete beginner-friendly instructions with diagrams

1. **Motor Driver Power (TMC2209 VMOT)**:
   - 100µF electrolytic capacitor between VMOT and GND
   - Place as close as possible to driver
   - **Polarity**: + to VMOT, - to GND (stripe = negative)

2. **Arduino 5V Rail**:
   - 10µF electrolytic capacitor between 5V and GND
   - **Polarity**: + to 5V, - to GND (stripe = negative)

3. **Each AS5600 Sensor (2 total)**:
   - 1µF electrolytic capacitor between VCC and GND (at each sensor)
   - Place directly at sensor pins
   - **Polarity**: + to VCC, - to GND (stripe = negative)

**Expected Results**:
- Dramatically reduced sensor noise during motor operation
- Stable readings even with motor enabled
- Ready for closed-loop control implementation

#### Next Development Steps
1. Test sensors with capacitors installed
2. Verify <5% error rate during motor movement
3. Implement swing-up controller
4. Add PID balance control at upright position

---

## November 6, 2025

### Hardware Modifications Completed

**Capacitors Installed** ✅:
- 1x 100µF / 50V on TMC2209 VMOT to GND (motor power filtering)
- 1x 10µF / 50V on Arduino 5V to GND (logic power filtering)
- 2x 1µF / 63V on AS5600 sensors VCC to GND (local sensor filtering)

**Breadboard Reorganization**:
- Separated power domains for better noise isolation
- **Breadboard 1**: TMC2209 driver and motor control
- **Breadboard 2**: Both AS5600 sensors, pull-up resistors, capacitors, Arduino 5V/GND distribution
- Connected breadboards via shared 5V and GND rails

### Issues Encountered

#### Issue 1: Sensor Detection Failure After Breadboard Reorganization

**Problem**: After moving sensor connections to second breadboard, both AS5600 sensors stopped being detected by Arduino. System hangs during initialization at "Testing Pendulum Sensor..."

**Symptoms**:
- Code hangs at sensor initialization
- No I2C communication detected
- Motor driver still works (unaffected)

**Debugging Steps Taken**:
1. Verified power connections between breadboards
2. Checked SDA/SCL routing through breadboard
3. Confirmed pull-up resistors still in place
4. Switched VCC/GND positions on breadboard power rails

**Root Cause (Suspected)**: 
- Possible defective breadboard with broken internal connections
- Power rail continuity issues
- Signal integrity problems with longer traces through breadboard

**Code Improvements Made**:
- Added I2C timeout protection (`Wire.setWireTimeout()`)
- Implemented raw I2C device detection before AS5600 library calls
- Added granular debug output to identify exact hang point
- Changed from `detectMagnet()` to direct angle reading (less blocking)
- Increased initialization delays for capacitor stabilization (200ms)

**Status**: Pending hardware troubleshooting with replacement breadboard

### Code Changes

**File**: `src/main.cpp`

**Improvements**:
```cpp
// Added timeout protection
Wire.setWireTimeout(5000, true);  // 5ms timeout, reset on timeout

// Test I2C bus with raw transmission first
Wire.beginTransmission(0x36);
uint8_t error = Wire.endTransmission();

// Direct angle reading instead of blocking detectMagnet()
uint16_t test_angle = as5600_pendulum.readAngle();
```

**Benefits**:
- Prevents infinite hangs on I2C communication failures
- Provides detailed error messages for each failure point
- Easier to diagnose hardware vs software issues
- Graceful degradation instead of system freeze

### Documentation Updates

**New Files Created**:

## November 7, 2025

### Motor Sensor Characterisation & Integration
- Built a dedicated AS5600 motor-sensor calibration sketch (`src/motor_sensor_cal_tool.cpp.bak`) that runs on the SoftwareWire bus (SDA=22, SCL=24). The tool lets us zero the magnet, jog with adjustable increments, capture left/right limits, and prints both controller steps and absolute degrees for every move. Added auto-scaling so the jog increment can target ~10° moves once a span is known.
- Discovered the TMC2209 is still in high microstep mode (≈1/32), so one "step" reported by AccelStepper is only ~0.065–0.08° of shaft rotation. The calibration tool now computes `deg/step` directly from the measured sensor span and reuses it for consistent reporting.
- Verified the AS5600 output is linear and symmetric when zeroed at the midpoint. Any apparent asymmetry was due to re-zeroing by eye after calibration. The tool now auto-zeroes at the midpoint and reminds us to re-align the stepper count only after jogging to the sensor's 0°.

### Main Control Firmware Updates
- Restored the full pendulum controller in `src/main.cpp` and baked in the motor-mounted AS5600 via SoftwareWire. `updateSensors()` now samples both pendulum and motor encoders with the same 3-sample median filter used during diagnostics.
- Zeroing (menu option 2) now captures both pendulum and motor angles; limit calibration (option 4) records sensor angles alongside step counts, derives `motorStepsPerDeg`, and requires zero to be set first. The setup sequence refuses to start swing-up or balance until zero + limits + scale are all recorded.
- Live monitoring prints pendulum angle, motor angle, raw values, and steps in a single row so drift is immediately visible. Diagnostics (option 1) checks both hardware-I²C and SoftwareWire sensors for magnet presence.
- Balance control uses the AS5600 motor angle for the centering term (`Kp_motor` now acts in steps-per-motor-degree), and the "Return to Center" action leverages the sensor reading to remove accumulated drift instead of trusting the virtual encoder alone.
- Documented the new workflow in the menu output and kept the calibration helper (`motor_sensor_cal_tool.cpp.bak`) as a reference for future hardware checks.
- Added continuous sensor refresh in the main loop so motor angles remain valid outside the control states, fixed zeroing output to show the absolute motor angle captured, and updated the `Return to Center` routine to read the AS5600 before issuing a move—resolving the "key 5 does nothing" behavior.

---

## November 7, 2025

### Software Noise-Mitigation Sprint
- Tightened AS5600 plausibility filtering (`src/main.cpp`) by:
  - Dropping the per-step change threshold from 90° to 12°
  - Rejecting raw readings near 0/4095 and requiring 5 consecutive, matching samples before accepting any value
  - Adding continuous magnet-status checks so corrupted fields no longer overwrite the last good angle
- Introduced post-step settling delays (500 µs) after every manual jog and AccelStepper move to keep AS5600 reads away from driver current spikes.
- Added “large jump hold” logic that automatically accepts a big transition once five identical samples arrive; prevents the controller from staying stuck on stale data when the arm really moves.

### Diagnostic Session Results (Options D, N, R)
- **Live Sensor Test (motor disabled)**: Still shows repeated 70–90° spikes on both pendulum (≈98↔118°) and motor (≈193↔203°) channels even while the mechanism is stationary; firmware now flags and suppresses them instead of letting the angle walk away.
- **Motor Power Noise Test**: With the driver energized but idle, success rate improved to ~94%, yet warnings persist every cycle, confirming that EMI on the I2C wiring—not just motor motion—is the dominant issue.
- **Raw Diagnostic Mode**: Median filter remains stable, but single-read data oscillates wildly (e.g., pendulum raw counts 1 100–3 000 while filtered holds at 2 979). Software safeguards are working, but hardware noise remains unsolved.

### Outstanding Issues / Next Steps
1. I2C wiring still acts like an antenna; need to re-route SDA/SCL as short twisted pairs with ground, add 22–47 Ω series resistors, and keep them away from stepper phase leads.
2. Verify sensor magnets are perfectly centered and ~2 mm from each chip; misalignment can trigger the ML/MH bits and amplify noise.
3. Once cabling is improved, re-run calibration steps 3–5 to confirm the new filters can lock onto clean angles without constant warnings.

> Status: Noise reductions in firmware are complete, but the root EMI source persists. Hardware rework deferred to the next session.
- `docs/CapacitorInstallationGuide.md` - Comprehensive beginner guide for capacitor installation with polarity warnings, step-by-step instructions, and diagrams

**Updated Files**:
- `Log.md` - Added Nov 5 capacitor specifications
- Code comments - Enhanced I2C initialization documentation

### Lessons Learned

1. **Breadboard Quality Matters**: Not all breadboards have reliable internal connections. Power rail continuity can be intermittent.

2. **Capacitors Need Stabilization Time**: Added 200ms delays after capacitor installation for power to stabilize before I2C communication.

3. **Blocking I2C Calls**: AS5600 library's `detectMagnet()` and `begin()` functions can hang indefinitely without timeout protection.

4. **Debug Incrementally**: Break initialization into small steps with print statements between each to identify exact failure point.

5. **Test Power First**: Before debugging communication, verify power delivery to all components (5V at sensor VCC pins).

### Tomorrow's Work (November 7, 2025)

#### Priority 1: Hardware Verification
1. Test with different breadboard
2. Verify power rail continuity with multimeter
3. Check all connections with continuity tester
4. Measure voltage at critical points:
   - Arduino 5V pin
   - Breadboard power rails
   - Sensor VCC pins
   - Pull-up resistor connections

#### Priority 2: Sensor Communication Test
1. Upload latest code with debug output
2. Run I2C scanner (option 0)
3. Check which devices detected
4. Test sensors individually (disconnect one at a time)

#### Priority 3: If Sensors Work
1. Test sensor stability with motor power ON
2. Measure noise reduction from capacitors
3. Verify >90% stable readings during motor operation
4. Proceed with swing-up controller implementation

### System Status: ⚠️ Hardware Troubleshooting Required

**What Works**:
- Motor driver and control
- Code improvements (timeout protection, debug output)
- Capacitors installed correctly

**What Needs Fix**:
- Sensor I2C communication (breadboard issue suspected)
- Power distribution verification

**Blocked**:
- Cannot test capacitor effectiveness until sensors working
- Cannot proceed to control algorithms

---

## November 10, 2025

### Hardware Modifications Completed

**Capacitors Installed** ✅:
- 1x 100µF / 50V on TMC2209 VMOT to GND (motor power filtering)
- 1x 10µF / 50V on Arduino 5V to GND (logic power filtering)
- 2x 1µF / 63V on AS5600 sensors VCC to GND (local sensor filtering)

**Breadboard Reorganization**:
- Separated power domains for better noise isolation
- **Breadboard 1**: TMC2209 driver and motor control
- **Breadboard 2**: Both AS5600 sensors, pull-up resistors, capacitors, Arduino 5V/GND distribution
- Connected breadboards via shared 5V and GND rails

### Issues Encountered

#### Issue 1: Sensor Detection Failure After Breadboard Reorganization

**Problem**: After moving sensor connections to second breadboard, both AS5600 sensors stopped being detected by Arduino. System hangs during initialization at "Testing Pendulum Sensor..."

**Symptoms**:
- Code hangs at sensor initialization
- No I2C communication detected
- Motor driver still works (unaffected)

**Debugging Steps Taken**:
1. Verified power connections between breadboards
2. Checked SDA/SCL routing through breadboard
3. Confirmed pull-up resistors still in place
4. Switched VCC/GND positions on breadboard power rails

**Root Cause (Suspected)**: 
- Possible defective breadboard with broken internal connections
- Power rail continuity issues
- Signal integrity problems with longer traces through breadboard

**Code Improvements Made**:
- Added I2C timeout protection (`Wire.setWireTimeout()`)
- Implemented raw I2C device detection before AS5600 library calls
- Added granular debug output to identify exact hang point
- Changed from `detectMagnet()` to direct angle reading (less blocking)
- Increased initialization delays for capacitor stabilization (200ms)

**Status**: Pending hardware troubleshooting with replacement breadboard

### Code Changes

**File**: `src/main.cpp`

**Improvements**:
```cpp
// Added timeout protection
Wire.setWireTimeout(5000, true);  // 5ms timeout, reset on timeout

// Test I2C bus with raw transmission first
Wire.beginTransmission(0x36);
uint8_t error = Wire.endTransmission();

// Direct angle reading instead of blocking detectMagnet()
uint16_t test_angle = as5600_pendulum.readAngle();
```

**Benefits**:
- Prevents infinite hangs on I2C communication failures
- Provides detailed error messages for each failure point
- Easier to diagnose hardware vs software issues
- Graceful degradation instead of system freeze

### Documentation Updates

**New Files Created**:

## November 7, 2025

### Motor Sensor Characterisation & Integration
- Built a dedicated AS5600 motor-sensor calibration sketch (`src/motor_sensor_cal_tool.cpp.bak`) that runs on the SoftwareWire bus (SDA=22, SCL=24). The tool lets us zero the magnet, jog with adjustable increments, capture left/right limits, and prints both controller steps and absolute degrees for every move. Added auto-scaling so the jog increment can target ~10° moves once a span is known.
- Discovered the TMC2209 is still in high microstep mode (≈1/32), so one "step" reported by AccelStepper is only ~0.065–0.08° of shaft rotation. The calibration tool now computes `deg/step` directly from the measured sensor span and reuses it for consistent reporting.
- Verified the AS5600 output is linear and symmetric when zeroed at the midpoint. Any apparent asymmetry was due to re-zeroing by eye after calibration. The tool now auto-zeroes at the midpoint and reminds us to re-align the stepper count only after jogging to the sensor's 0°.

### Main Control Firmware Updates
- Restored the full pendulum controller in `src/main.cpp` and baked in the motor-mounted AS5600 via SoftwareWire. `updateSensors()` now samples both pendulum and motor encoders with the same 3-sample median filter used during diagnostics.
- Zeroing (menu option 2) now captures both pendulum and motor angles; limit calibration (option 4) records sensor angles alongside step counts, derives `motorStepsPerDeg`, and requires zero to be set first. The setup sequence refuses to start swing-up or balance until zero + limits + scale are all recorded.
- Live monitoring prints pendulum angle, motor angle, raw values, and steps in a single row so drift is immediately visible. Diagnostics (option 1) checks both hardware-I²C and SoftwareWire sensors for magnet presence.
- Balance control uses the AS5600 motor angle for the centering term (`Kp_motor` now acts in steps-per-motor-degree), and the "Return to Center" action leverages the sensor reading to remove accumulated drift instead of trusting the virtual encoder alone.
- Documented the new workflow in the menu output and kept the calibration helper (`motor_sensor_cal_tool.cpp.bak`) as a reference for future hardware checks.
- Added continuous sensor refresh in the main loop so motor angles remain valid outside the control states, fixed zeroing output to show the absolute motor angle captured, and updated the `Return to Center` routine to read the AS5600 before issuing a move—resolving the "key 5 does nothing" behavior.

---

## November 7, 2025

### Software Noise-Mitigation Sprint
- Tightened AS5600 plausibility filtering (`src/main.cpp`) by:
  - Dropping the per-step change threshold from 90° to 12°
  - Rejecting raw readings near 0/4095 and requiring 5 consecutive, matching samples before accepting any value
  - Adding continuous magnet-status checks so corrupted fields no longer overwrite the last good angle
- Introduced post-step settling delays (500 µs) after every manual jog and AccelStepper move to keep AS5600 reads away from driver current spikes.
- Added “large jump hold” logic that automatically accepts a big transition once five identical samples arrive; prevents the controller from staying stuck on stale data when the arm really moves.

### Diagnostic Session Results (Options D, N, R)
- **Live Sensor Test (motor disabled)**: Still shows repeated 70–90° spikes on both pendulum (≈98↔118°) and motor (≈193↔203°) channels even while the mechanism is stationary; firmware now flags and suppresses them instead of letting the angle walk away.
- **Motor Power Noise Test**: With the driver energized but idle, success rate improved to ~94%, yet warnings persist every cycle, confirming that EMI on the I2C wiring—not just motor motion—is the dominant issue.
- **Raw Diagnostic Mode**: Median filter remains stable, but single-read data oscillates wildly (e.g., pendulum raw counts 1 100–3 000 while filtered holds at 2 979). Software safeguards are working, but hardware noise remains unsolved.

### Outstanding Issues / Next Steps
1. I2C wiring still acts like an antenna; need to re-route SDA/SCL as short twisted pairs with ground, add 22–47 Ω series resistors, and keep them away from stepper phase leads.
2. Verify sensor magnets are perfectly centered and ~2 mm from each chip; misalignment can trigger the ML/MH bits and amplify noise.
3. Once cabling is improved, re-run calibration steps 3–5 to confirm the new filters can lock onto clean angles without constant warnings.

> Status: Noise reductions in firmware are complete, but the root EMI source persists. Hardware rework deferred to the next session.
- `docs/CapacitorInstallationGuide.md` - Comprehensive beginner guide for capacitor installation with polarity warnings, step-by-step instructions, and diagrams

**Updated Files**:
- `Log.md` - Added Nov 5 capacitor specifications
- Code comments - Enhanced I2C initialization documentation

### Lessons Learned

1. **Breadboard Quality Matters**: Not all breadboards have reliable internal connections. Power rail continuity can be intermittent.

2. **Capacitors Need Stabilization Time**: Added 200ms delays after capacitor installation for power to stabilize before I2C communication.

3. **Blocking I2C Calls**: AS5600 library's `detectMagnet()` and `begin()` functions can hang indefinitely without timeout protection.

4. **Debug Incrementally**: Break initialization into small steps with print statements between each to identify exact failure point.

5. **Test Power First**: Before debugging communication, verify power delivery to all components (5V at sensor VCC pins).

### Tomorrow's Work (November 7, 2025)

#### Priority 1: Hardware Verification
1. Test with different breadboard
2. Verify power rail continuity with multimeter
3. Check all connections with continuity tester
4. Measure voltage at critical points:
   - Arduino 5V pin
   - Breadboard power rails
   - Sensor VCC pins
   - Pull-up resistor connections

#### Priority 2: Sensor Communication Test
1. Upload latest code with debug output
2. Run I2C scanner (option 0)
3. Check which devices detected
4. Test sensors individually (disconnect one at a time)

#### Priority 3: If Sensors Work
1. Test sensor stability with motor power ON
2. Measure noise reduction from capacitors
3. Verify >90% stable readings during motor operation
4. Proceed with swing-up controller implementation

### System Status: ⚠️ Hardware Troubleshooting Required

**What Works**:
- Motor driver and control
- Code improvements (timeout protection, debug output)
- Capacitors installed correctly

**What Needs Fix**:
- Sensor I2C communication (breadboard issue suspected)
- Power distribution verification

**Blocked**:
- Cannot test capacitor effectiveness until sensors working
- Cannot proceed to control algorithms

---

## November 10, 2025

### Hardware Modifications Completed

**Capacitors Installed** ✅:
- 1x 100µF / 50V on TMC2209 VMOT to GND (motor power filtering)
- 1x 10µF / 50V on Arduino 5V to GND (logic power filtering)
- 2x 1µF / 63V on AS5600 sensors VCC to GND (local sensor filtering)

**Breadboard Reorganization**:
- Separated power domains for better noise isolation
- **Breadboard 1**: TMC2209 driver and motor control
- **Breadboard 2**: Both AS5600 sensors, pull-up resistors, capacitors, Arduino 5V/GND distribution
- Connected breadboards via shared 5V and GND rails

### Issues Encountered

#### Issue 1: Sensor Detection Failure After Breadboard Reorganization

**Problem**: After moving sensor connections to second breadboard, both AS5600 sensors stopped being detected by Arduino. System hangs during initialization at "Testing Pendulum Sensor..."

**Symptoms**:
- Code hangs at sensor initialization
- No I2C communication detected
- Motor driver still works (unaffected)

**Debugging Steps Taken**:
1. Verified power connections between breadboards
2. Checked SDA/SCL routing through breadboard
3. Confirmed pull-up resistors still in place
4. Switched VCC/GND positions on breadboard power rails

**Root Cause (Suspected)**: 
- Possible defective breadboard with broken internal connections
- Power rail continuity issues
- Signal integrity problems with longer traces through breadboard

**Code Improvements Made**:
- Added I2C timeout protection (`Wire.setWireTimeout()`)
- Implemented raw I2C device detection before AS5600 library calls
- Added granular debug output to identify exact hang point
- Changed from `detectMagnet()` to direct angle reading (less blocking)
- Increased initialization delays for capacitor stabilization (200ms)

**Status**: Pending hardware troubleshooting with replacement breadboard

### Code Changes

**File**: `src/main.cpp`

**Improvements**:
```cpp
// Added timeout protection
Wire.setWireTimeout(5000, true);  // 5ms timeout, reset on timeout

// Test I2C bus with raw transmission first
Wire.beginTransmission(0x36);
uint8_t error = Wire.endTransmission();

// Direct angle reading instead of blocking detectMagnet()
uint16_t test_angle = as5600_pendulum.readAngle();
```

**Benefits**:
- Prevents infinite hangs on I2C communication failures
- Provides detailed error messages for each failure point
- Easier to diagnose hardware vs software issues
- Graceful degradation instead of system freeze

### Documentation Updates

**New Files Created**:

## November 7, 2025

### Motor Sensor Characterisation & Integration
- Built a dedicated AS5600 motor-sensor calibration sketch (`src/motor_sensor_cal_tool.cpp.bak`) that runs on the SoftwareWire bus (SDA=22, SCL=24). The tool lets us zero the magnet, jog with adjustable increments, capture left/right limits, and prints both controller steps and absolute degrees for every move. Added auto-scaling so the jog increment can target ~10° moves once a span is known.
- Discovered the TMC2209 is still in high microstep mode (≈1/32), so one "step" reported by AccelStepper is only ~0.065–0.08° of shaft rotation. The calibration tool now computes `deg/step` directly from the measured sensor span and reuses it for consistent reporting.
- Verified the AS5600 output is linear and symmetric when zeroed at the midpoint. Any apparent asymmetry was due to re-zeroing by eye after calibration. The tool now auto-zeroes at the midpoint and reminds us to re-align the stepper count only after jogging to the sensor's 0°.

### Main Control Firmware Updates
- Restored the full pendulum controller in `src/main.cpp` and baked in the motor-mounted AS5600 via SoftwareWire. `updateSensors()` now samples both pendulum and motor encoders with the same 3-sample median filter used during diagnostics.
- Zeroing (menu option 2) now captures both pendulum and motor angles; limit calibration (option 4) records sensor angles alongside step counts, derives `motorStepsPerDeg`, and requires zero to be set first. The setup sequence refuses to start swing-up or balance until zero + limits + scale are all recorded.
- Live monitoring prints pendulum angle, motor angle, raw values, and steps in a single row so drift is immediately visible. Diagnostics (option 1) checks both hardware-I²C and SoftwareWire sensors for magnet presence.
- Balance control uses the AS5600 motor angle for the centering term (`Kp_motor` now acts in steps-per-motor-degree), and the "Return to Center" action leverages the sensor reading to remove accumulated drift instead of trusting the virtual encoder alone.
- Documented the new workflow in the menu output and kept the calibration helper (`motor_sensor_cal_tool.cpp.bak`) as a reference for future hardware checks.
- Added continuous sensor refresh in the main loop so motor angles remain valid outside the control states, fixed zeroing output to show the absolute motor angle captured, and updated the `Return to Center` routine to read the AS5600 before issuing a move—resolving the "key 5 does nothing" behavior.

---

## November 7, 2025

### Software Noise-Mitigation Sprint
- Tightened AS5600 plausibility filtering (`src/main.cpp`) by:
  - Dropping the per-step change threshold from 90° to 12°
  - Rejecting raw readings near 0/4095 and requiring 5 consecutive, matching samples before accepting any value
  - Adding continuous magnet-status checks so corrupted fields no longer overwrite the last good angle
- Introduced post-step settling delays (500 µs) after every manual jog and AccelStepper move to keep AS5600 reads away from driver current spikes.
- Added “large jump hold” logic that automatically accepts a big transition once five identical samples arrive; prevents the controller from staying stuck on stale data when the arm really moves.

### Diagnostic Session Results (Options D, N, R)
- **Live Sensor Test (motor disabled)**: Still shows repeated 70–90° spikes on both pendulum (≈98↔118°) and motor (≈193↔203°) channels even while the mechanism is stationary; firmware now flags and suppresses them instead of letting the angle walk away.
- **Motor Power Noise Test**: With the driver energized but idle, success rate improved to ~94%, yet warnings persist every cycle, confirming that EMI on the I2C wiring—not just motor motion—is the dominant issue.
- **Raw Diagnostic Mode**: Median filter remains stable, but single-read data oscillates wildly (e.g., pendulum raw counts 1 100–3 000 while filtered holds at 2 979). Software safeguards are working, but hardware noise remains unsolved.

### Outstanding Issues / Next Steps
1. I2C wiring still acts like an antenna; need to re-route SDA/SCL as short twisted pairs with ground, add 22–47 Ω series resistors, and keep them away from stepper phase leads.
2. Verify sensor magnets are perfectly centered and ~2 mm from each chip; misalignment can trigger the ML/MH bits and amplify noise.
3. Once cabling is improved, re-run calibration steps 3–5 to confirm the new filters can lock onto clean angles without constant warnings.

> Status: Noise reductions in firmware are complete, but the root EMI source persists. Hardware rework deferred to the next session.
- `docs/CapacitorInstallationGuide.md` - Comprehensive beginner guide for capacitor installation with polarity warnings, step-by-step instructions, and diagrams

**Updated Files**:
- `Log.md` - Added Nov 5 capacitor specifications
- Code comments - Enhanced I2C initialization documentation

### Lessons Learned

1. **Breadboard Quality Matters**: Not all breadboards have reliable internal connections. Power rail continuity can be intermittent.

2. **Capacitors Need Stabilization Time**: Added 200ms delays after capacitor installation for power to stabilize before I2C communication.

3. **Blocking I2C Calls**: AS5600 library's `detectMagnet()` and `begin()` functions can hang indefinitely without timeout protection.

4. **Debug Incrementally**: Break initialization into small steps with print statements between each to identify exact failure point.

5. **Test Power First**: Before debugging communication, verify power delivery to all components (5V at sensor VCC pins).

### Tomorrow's Work (November 7, 2025)

#### Priority 1: Hardware Verification
1. Test with different breadboard
2. Verify power rail continuity with multimeter
3. Check all connections with continuity tester
4. Measure voltage at critical points:
   - Arduino 5V pin
   - Breadboard power rails
   - Sensor VCC pins
   - Pull-up resistor connections

#### Priority 2: Sensor Communication Test
1. Upload latest code with debug output
2. Run I2C scanner (option 0)
3. Check which devices detected
4. Test sensors individually (disconnect one at a time)

#### Priority 3: If Sensors Work
1. Test sensor stability with motor power ON
2. Measure noise reduction from capacitors
3. Verify >90% stable readings during motor operation
4. Proceed with swing-up controller implementation

### System Status: ⚠️ Hardware Troubleshooting Required

**What Works**:
- Motor driver and control
- Code improvements (timeout protection, debug output)
- Capacitors installed correctly

**What Needs Fix**:
- Sensor I2C communication (breadboard issue suspected)
- Power distribution verification

**Blocked**:
- Cannot test capacitor effectiveness until sensors working
- Cannot proceed to control algorithms

---

## November 10, 2025

### Hardware Modifications Completed

**Capacitors Installed** ✅:
- 1x 100µF / 50V on TMC2209 VMOT to GND (motor power filtering)
- 1x 10µF / 50V on Arduino 5V to GND (logic power filtering)
- 2x 1µF / 63V on AS5600 sensors VCC to GND (local sensor filtering)

**Breadboard Reorganization**:
- Separated power domains for better noise isolation
- **Breadboard 1**: TMC2209 driver and motor control
- **Breadboard 2**: Both AS5600 sensors, pull-up resistors, capacitors, Arduino 5V/GND distribution
- Connected breadboards via shared 5V and GND rails

### Issues Encountered

#### Issue 1: Sensor Detection Failure After Breadboard Reorganization

**Problem**: After moving sensor connections to second breadboard, both AS5600 sensors stopped being detected by Arduino. System hangs during initialization at "Testing Pendulum Sensor..."

**Symptoms**:
- Code hangs at sensor initialization
- No I2C communication detected
- Motor driver still works (unaffected)

**Debugging Steps Taken**:
1. Verified power connections between breadboards
2. Checked SDA/SCL routing through breadboard
3. Confirmed pull-up resistors still in place
4. Switched VCC/GND positions on breadboard power rails

**Root Cause (Suspected)**: 
- Possible defective breadboard with broken internal connections
- Power rail continuity issues
- Signal integrity problems with longer traces through breadboard

**Code Improvements Made**:
- Added I2C timeout protection (`Wire.setWireTimeout()`)
- Implemented raw I2C device detection before AS5600 library calls
- Added granular debug output to identify exact hang point
- Changed from `detectMagnet()` to direct angle reading (less blocking)
- Increased initialization delays for capacitor stabilization (200ms)

**Status**: Pending hardware troubleshooting with replacement breadboard

### Code Changes

**File**: `src/main.cpp`

**Improvements**:
```cpp
// Added timeout protection
Wire.setWireTimeout(5000, true);  // 5ms timeout, reset on timeout

// Test I2C bus with raw transmission first
Wire.beginTransmission(0x36);
uint8_t error = Wire.endTransmission();

// Direct angle reading instead of blocking detectMagnet()
uint16_t test_angle = as5600_pendulum.readAngle();
```

**Benefits**:
- Prevents infinite hangs on I2C communication failures
- Provides detailed error messages for each failure point
- Easier to diagnose hardware vs software issues
- Graceful degradation instead of system freeze

### Documentation Updates

**New Files Created**:

## November 7, 2025

### Motor Sensor Characterisation & Integration
- Built a dedicated AS5600 motor-sensor calibration sketch (`src/motor_sensor_cal_tool.cpp.bak`) that runs on the SoftwareWire bus (SDA=22, SCL=24). The tool lets us zero the magnet, jog with adjustable increments, capture left/right limits, and prints both controller steps and absolute degrees for every move. Added auto-scaling so the jog increment can target ~10° moves once a span is known.
- Discovered the TMC2209 is still in high microstep mode (≈1/32), so one "step" reported by AccelStepper is only ~0.065–0.08° of shaft rotation. The calibration tool now computes `deg/step` directly from the measured sensor span and reuses it for consistent reporting.
- Verified the AS5600 output is linear and symmetric when zeroed at the midpoint. Any apparent asymmetry was due to re-zeroing by eye after calibration. The tool now auto-zeroes at the midpoint and reminds us to re-align the stepper count only after jogging to the sensor's 0°.

### Main Control Firmware Updates
- Restored the full pendulum controller in `src/main.cpp` and baked in the motor-mounted AS5600 via SoftwareWire. `updateSensors()` now samples both pendulum and motor encoders with the same 3-sample median filter used during diagnostics.
- Zeroing (menu option 2) now captures both pendulum and motor angles; limit calibration (option 4) records sensor angles alongside step counts, derives `motorStepsPerDeg`, and requires zero to be set first. The setup sequence refuses to start swing-up or balance until zero + limits + scale are all recorded.
- Live monitoring prints pendulum angle, motor angle, raw values, and steps in a single row so drift is immediately visible. Diagnostics (option 1) checks both hardware-I²C and SoftwareWire sensors for magnet presence.
- Balance control uses the AS5600 motor angle for the centering term (`Kp_motor` now acts in steps-per-motor-degree), and the "Return to Center" action leverages the sensor reading to remove accumulated drift instead of trusting the virtual encoder alone.
- Documented the new workflow in the menu output and kept the calibration helper (`motor_sensor_cal_tool.cpp.bak`) as a reference for future hardware checks.
- Added continuous sensor refresh in the main loop so motor angles remain valid outside the control states, fixed zeroing output to show the absolute motor angle captured, and updated the `Return to Center` routine to read the AS5600 before issuing a move—resolving the "key 5 does nothing" behavior.

---

## November 7, 2025

### Software Noise-Mitigation Sprint
- Tightened AS5600 plausibility filtering (`src/main.cpp`) by:
  - Dropping the per-step change threshold from 90° to 12°
  - Rejecting raw readings near 0/4095 and requiring 5 consecutive, matching samples before accepting any value
  - Adding continuous magnet-status checks so corrupted fields no longer overwrite the last good angle
- Introduced post-step settling delays (500 µs) after every manual jog and AccelStepper move to keep AS5600 reads away from driver current spikes.
- Added “large jump hold” logic that automatically accepts a big transition once five identical samples arrive; prevents the controller from staying stuck on stale data when the arm really moves.

### Diagnostic Session Results (Options D, N, R)
- **Live Sensor Test (motor disabled)**: Still shows repeated 70–90° spikes on both pendulum (≈98↔118°) and motor (≈193↔203°) channels even while the mechanism is stationary; firmware now flags and suppresses them instead of letting the angle walk away.
- **Motor Power Noise Test**: With the driver energized but idle, success rate improved to ~94%, yet warnings persist every cycle, confirming that EMI on the I2C wiring—not just motor motion—is the dominant issue.
- **Raw Diagnostic Mode**: Median filter remains stable, but single-read data oscillates wildly (e.g., pendulum raw counts 1 100–3 000 while filtered holds at 2 979). Software safeguards are working, but hardware noise remains unsolved.

### Outstanding Issues / Next Steps
1. I2C wiring still acts like an antenna; need to re-route SDA/SCL as short twisted pairs with ground, add 22–47 Ω series resistors, and keep them away from stepper phase leads.
2. Verify sensor magnets are perfectly centered and ~2 mm from each chip; misalignment can trigger the ML/MH bits and amplify noise.
3. Once cabling is improved, re-run calibration steps 3–5 to confirm the new filters can lock onto clean angles without constant warnings.

> Status: Noise reductions in firmware are complete, but the root EMI source persists. Hardware rework deferred to the next session.
- `docs/CapacitorInstallationGuide.md` - Comprehensive beginner guide for capacitor installation with polarity warnings, step-by-step instructions, and diagrams

**Updated Files**:
- `Log.md` - Added Nov 5 capacitor specifications
- Code comments - Enhanced I2C initialization documentation

### Lessons Learned

1. **Breadboard Quality Matters**: Not all breadboards have reliable internal connections. Power rail continuity can be intermittent.

2. **Capacitors Need Stabilization Time**: Added 200ms delays after capacitor installation for power to stabilize before I2C communication.

3. **Blocking I2C Calls**: AS5600 library's `detectMagnet()` and `begin()` functions can hang indefinitely without timeout protection.

4. **Debug Incrementally**: Break initialization into small steps with print statements between each to identify exact failure point.

5. **Test Power First**: Before debugging communication, verify power delivery to all components (5V at sensor VCC pins).

### Tomorrow's Work (November 7, 2025)

#### Priority 1: Hardware Verification
1. Test with different breadboard
2. Verify power rail continuity with multimeter
3. Check all connections with continuity tester
4. Measure voltage at critical points:
   - Arduino 5V pin
   - Breadboard power rails
   - Sensor VCC pins
   - Pull-up resistor connections

#### Priority 2: Sensor Communication Test
1. Upload latest code with debug output
2. Run I2C scanner (option 0)
3. Check which devices detected
4. Test sensors individually (disconnect one at a time)

#### Priority 3: If Sensors Work
1. Test sensor stability with motor power ON
2. Measure noise reduction from capacitors
3. Verify >90% stable readings during motor operation
4. Proceed with swing-up controller implementation

### System Status: ⚠️ Hardware Troubleshooting Required

**What Works**:
- Motor driver and control
- Code improvements (timeout protection, debug output)
- Capacitors installed correctly

**What Needs Fix**:
- Sensor I2C communication (breadboard issue suspected)
- Power distribution verification

**Blocked**:
- Cannot test capacitor effectiveness until sensors working
- Cannot proceed to control algorithms

---

## November 10, 2025

### Hardware Modifications Completed

**Capacitors Installed** ✅:
- 1x 100µF / 50V on TMC2209 VMOT to GND (motor power filtering)
- 1x 10µF / 50V on Arduino 5V to GND (logic power filtering)
- 2x 1µF / 63V on AS5600 sensors VCC to GND (local sensor filtering)

**Breadboard Reorganization**:
- Separated power domains for better noise isolation
- **Breadboard 1**: TMC2209 driver and motor control
- **Breadboard 2**: Both AS5600 sensors, pull-up resistors, capacitors, Arduino 5V/GND distribution
- Connected breadboards via shared 5V and GND rails

### Issues Encountered

#### Issue 1: Sensor Detection Failure After Breadboard Reorganization

**Problem**: After moving sensor connections to second breadboard, both AS5600 sensors stopped being detected by Arduino. System hangs during initialization at "Testing Pendulum Sensor..."

**Symptoms**:
- Code hangs at sensor initialization
- No I2C communication detected
- Motor driver still works (unaffected)

**Debugging Steps Taken**:
1. Verified power connections between breadboards
2. Checked SDA/SCL routing through breadboard
3. Confirmed pull-up resistors still in place
4. Switched VCC/GND positions on breadboard power rails

**Root Cause (Suspected)**: 
- Possible defective breadboard with broken internal connections
- Power rail continuity issues
- Signal integrity problems with longer traces through breadboard

**Code Improvements Made**:
- Added I2C timeout protection (`Wire.setWireTimeout()`)
- Implemented raw I2C device detection before AS5600 library calls
- Added granular debug output to identify exact hang point
- Changed from `detectMagnet()` to direct angle reading (less blocking)
- Increased initialization delays for capacitor stabilization (200ms)

**Status**: Pending hardware troubleshooting with replacement breadboard

### Code Changes

**File**: `src/main.cpp`

**Improvements**:
```cpp
// Added timeout protection
Wire.setWireTimeout(5000, true);  // 5ms timeout, reset on timeout

// Test I2C bus with raw transmission first
Wire.beginTransmission(0x36);
uint8_t error = Wire.endTransmission();

// Direct angle reading instead of blocking detectMagnet()
uint16_t test_angle = as5600_pendulum.readAngle();
```

**Benefits**:
- Prevents infinite hangs on I2C communication failures
- Provides detailed error messages for each failure point
- Easier to diagnose hardware vs software issues
- Graceful degradation instead of system freeze

### Documentation Updates

**New Files Created**:

## November 7, 2025

### Motor Sensor Characterisation & Integration
- Built a dedicated AS5600 motor-sensor calibration sketch (`src/motor_sensor_cal_tool.cpp.bak`) that runs on the SoftwareWire bus (SDA=22, SCL=24). The tool lets us zero the magnet, jog with adjustable increments, capture left/right limits, and prints both controller steps and absolute degrees for every move. Added auto-scaling so the jog increment can target ~10° moves once a span is known.
- Discovered the TMC2209 is still in high microstep mode (≈1/32), so one "step" reported by AccelStepper is only ~0.065–0.08° of shaft rotation. The calibration tool now computes `deg/step` directly from the measured sensor span and reuses it for consistent reporting.
- Verified the AS5600 output is linear and symmetric when zeroed at the midpoint. Any apparent asymmetry was due to re-zeroing by eye after calibration. The tool now auto-zeroes at the midpoint and reminds us to re-align the stepper count only after jogging to the sensor's 0°.

### Main Control Firmware Updates
- Restored the full pendulum controller in `src/main.cpp` and baked in the motor-mounted AS5600 via SoftwareWire. `updateSensors()` now samples both pendulum and motor encoders with the same 3-sample median filter used during diagnostics.
- Zeroing (menu option 2) now captures both pendulum and motor angles; limit calibration (option 4) records sensor angles alongside step counts, derives `motorStepsPerDeg`, and requires zero to be set first. The setup sequence refuses to start swing-up or balance until zero + limits + scale are all recorded.
- Live monitoring prints pendulum angle, motor angle, raw values, and steps in a single row so drift is immediately visible. Diagnostics (option 1) checks both hardware-I²C and SoftwareWire sensors for magnet presence.
- Balance control uses the AS5600 motor angle for the centering term (`Kp_motor` now acts in steps-per-motor-degree), and the "Return to Center" action leverages the sensor reading to remove accumulated drift instead of trusting the virtual encoder alone.
- Documented the new workflow in the menu output and kept the calibration helper (`motor_sensor_cal_tool.cpp.bak`) as a reference for future hardware checks.
- Added continuous sensor refresh in the main loop so motor angles remain valid outside the control states, fixed zeroing output to show the absolute motor angle captured, and updated the `Return to Center` routine to read the AS5600 before issuing a move—resolving the "key 5 does nothing" behavior.

---

## November 7, 2025

### Software Noise-Mitigation Sprint
- Tightened AS5600 plausibility filtering (`src/main.cpp`) by:
  - Dropping the per-step change threshold from 90° to 12°
  - Rejecting raw readings near 0/4095 and requiring 5 consecutive, matching samples before accepting any value
  - Adding continuous magnet-status checks so corrupted fields no longer overwrite the last good angle
- Introduced post-step settling delays (500 µs) after every manual jog and AccelStepper move to keep AS5600 reads away from driver current spikes.
- Added “large jump hold” logic that automatically accepts a big transition once five identical samples arrive; prevents the controller from staying stuck on stale data when the arm really moves.

### Diagnostic Session Results (Options D, N, R)
- **Live Sensor Test (motor disabled)**: Still shows repeated 70–90° spikes on both pendulum (≈98↔118°) and motor (≈193↔203°) channels even while the mechanism is stationary; firmware now flags and suppresses them instead of letting the angle walk away.
- **Motor Power Noise Test**: With the driver energized but idle, success rate improved to ~94%, yet warnings persist every cycle, confirming that EMI on the I2C wiring—not just motor motion—is the dominant issue.
- **Raw Diagnostic Mode**: Median filter remains stable, but single-read data oscillates wildly (e.g., pendulum raw counts 1 100–3 000 while filtered holds at 2 979). Software safeguards are working, but hardware noise remains unsolved.

### Outstanding Issues / Next Steps
1. I2C wiring still acts like an antenna; need to re-route SDA/SCL as short twisted pairs with ground, add 22–47 Ω series resistors, and keep them away from stepper phase leads.
2. Verify sensor magnets are perfectly centered and ~2 mm from each chip; misalignment can trigger the ML/MH bits and amplify noise.
3. Once cabling is improved, re-run calibration steps 3–5 to confirm the new filters can lock onto clean angles without constant warnings.

> Status: Noise reductions in firmware are complete, but the root EMI source persists. Hardware rework deferred to the next session.
- `docs/CapacitorInstallationGuide.md` - Comprehensive beginner guide for capacitor installation with polarity warnings, step-by-step instructions, and diagrams

**Updated Files**:
- `Log.md` - Added Nov 5 capacitor specifications
- Code comments - Enhanced I2C initialization documentation

### Lessons Learned

1. **Breadboard Quality Matters**: Not all breadboards have reliable internal connections. Power rail continuity can be intermittent.

2. **Capacitors Need Stabilization Time**: Added 200ms delays after capacitor installation for power to stabilize before I2C communication.

3. **Blocking I2C Calls**: AS5600 library's `detectMagnet()` and `begin()` functions can hang indefinitely without timeout protection.

4. **Debug Incrementally**: Break initialization into small steps with print statements between each to identify exact failure point.

5. **Test Power First**: Before debugging communication, verify power delivery to all components (5V at sensor VCC pins).

### Tomorrow's Work (November 7, 2025)

#### Priority 1: Hardware Verification
1. Test with different breadboard
2. Verify power rail continuity with multimeter
3. Check all connections with continuity tester
4. Measure voltage at critical points:
   - Arduino 5V pin
   - Breadboard power rails
   - Sensor VCC pins
   - Pull-up resistor connections

#### Priority 2: Sensor Communication Test
1. Upload latest code with debug output
2. Run I2C scanner (option 0)
3. Check which devices detected
4. Test sensors individually (disconnect one at a time)

#### Priority 3: If Sensors Work
1. Test sensor stability with motor power ON
2. Measure noise reduction from capacitors
3. Verify >90% stable readings during motor operation
4. Proceed with swing-up controller implementation

### System Status: ⚠️ Hardware Troubleshooting Required

**What Works**:
- Motor driver and control
- Code improvements (timeout protection, debug output)
- Capacitors installed correctly

**What Needs Fix**:
- Sensor I2C communication (breadboard issue suspected)
- Power distribution verification

**Blocked**:
- Cannot test capacitor effectiveness until sensors working
- Cannot proceed to control algorithms

---

## November 10, 2025

### Hardware Modifications Completed

**Capacitors Installed** ✅:
- 1x 100µF / 50V on TMC2209 VMOT to GND (motor power filtering)
- 1x 10µF / 50V on Arduino 5V to GND (logic power filtering)
- 2x 1µF / 63V on AS5600 sensors VCC to GND (local sensor filtering)

**Breadboard Reorganization**:
- Separated power domains for better noise isolation
- **Breadboard 1**: TMC2209 driver and motor control
- **Breadboard 2**: Both AS5600 sensors, pull-up resistors, capacitors, Arduino 5V/GND distribution
- Connected breadboards via shared 5V and GND rails

### Issues Encountered

#### Issue 1: Sensor Detection Failure After Breadboard Reorganization

**Problem**: After moving sensor connections to second breadboard, both AS5600 sensors stopped being detected by Arduino. System hangs during initialization at "Testing Pendulum Sensor..."

**Symptoms**:
- Code hangs at sensor initialization
- No I2C communication detected
- Motor driver still works (unaffected)

**Debugging Steps Taken**:
1. Verified power connections between breadboards
2. Checked SDA/SCL routing through breadboard
3. Confirmed pull-up resistors still in place
4. Switched VCC/GND positions on breadboard power rails

**Root Cause (Suspected)**: 
- Possible defective breadboard with broken internal connections
- Power rail continuity issues
- Signal integrity problems with longer traces through breadboard

**Code Improvements Made**:
- Added I2C timeout protection (`Wire.setWireTimeout()`)
- Implemented raw I2C device detection before AS5600 library calls
- Added granular debug output to identify exact hang point
- Changed from `detectMagnet()` to direct angle reading (less blocking)
- Increased initialization delays for capacitor stabilization (200ms)

**Status**: Pending hardware troubleshooting with replacement breadboard

### Code Changes

**File**: `src/main.cpp`

**Improvements**:
```cpp
// Added timeout protection
Wire.setWireTimeout(5000, true);  // 5ms timeout, reset on timeout

// Test I2C bus with raw transmission first
Wire.beginTransmission(0x36);
uint8_t error = Wire.endTransmission();

// Direct angle reading instead of blocking detectMagnet()
uint16_t test_angle = as5600_pendulum.readAngle();
```

**Benefits**:
- Prevents infinite hangs on I2C communication failures
- Provides detailed error messages for each failure point
- Easier to diagnose hardware vs software issues
- Graceful degradation instead of system freeze

### Documentation Updates

**New Files Created**:

## November 7, 2025

### Motor Sensor Characterisation & Integration
- Built a dedicated AS5600 motor-sensor calibration sketch (`src/motor_sensor_cal_tool.cpp.bak`) that runs on the SoftwareWire bus (SDA=22, SCL=24). The tool lets us zero the magnet, jog with adjustable increments, capture left/right limits, and prints both controller steps and absolute degrees for every move. Added auto-scaling so the jog increment can target ~10° moves once a span is known.
- Discovered the TMC2209 is still in high microstep mode (≈1/32), so one "step" reported by AccelStepper is only ~0.065–0.08° of shaft rotation. The calibration tool now computes `deg/step` directly from the measured sensor span and reuses it for consistent reporting.
- Verified the AS5600 output is linear and symmetric when zeroed at the midpoint. Any apparent asymmetry was due to re-zeroing by eye after calibration. The tool now auto-zeroes at the midpoint and reminds us to re-align the stepper count only after jogging to the sensor's 0°.

### Main Control Firmware Updates
- Restored the full pendulum controller in `src/main.cpp` and baked in the motor-mounted AS5600 via SoftwareWire. `updateSensors()` now samples both pendulum and motor encoders with the same 3-sample median filter used during diagnostics.
- Zeroing (menu option 2) now captures both pendulum and motor angles; limit calibration (option 4) records sensor angles alongside step counts, derives `motorStepsPerDeg`, and requires zero to be set first. The setup sequence refuses to start swing-up or balance until zero + limits + scale are all recorded.
- Live monitoring prints pendulum angle, motor angle, raw values, and steps in a single row so drift is immediately visible. Diagnostics (option 1) checks both hardware-I²C and SoftwareWire sensors for magnet presence.
- Balance control uses the AS5600 motor angle for the centering term (`Kp_motor` now acts in steps-per-motor-degree), and the "Return to Center" action leverages the sensor reading to remove accumulated drift instead of trusting the virtual encoder alone.
- Documented the new workflow in the menu output and kept the calibration helper (`motor_sensor_cal_tool.cpp.bak`) as a reference for future hardware checks.
- Added continuous sensor refresh in the main loop so motor angles remain valid outside the control states, fixed zeroing output to show the absolute motor angle captured, and updated the `Return to Center` routine to read the AS5600 before issuing a move—resolving the "key 5 does nothing" behavior.

---

## November 7, 2025

### Software Noise-Mitigation Sprint
- Tightened AS5600 plausibility filtering (`src/main.cpp`) by:
  - Dropping the per-step change threshold from 90° to 12°
  - Rejecting raw readings near 0/4095 and requiring 5 consecutive, matching samples before accepting any value
  - Adding continuous magnet-status checks so corrupted fields no longer overwrite the last good angle
- Introduced post-step settling delays (500 µs) after every manual jog and AccelStepper move to keep AS5600 reads away from driver current spikes.
- Added “large jump hold” logic that automatically accepts a big transition once five identical samples arrive; prevents the controller from staying stuck on stale data when the arm really moves.

### Diagnostic Session Results (Options D, N, R)
- **Live Sensor Test (motor disabled)**: Still shows repeated 70–90° spikes on both pendulum (≈98↔118°) and motor (≈193↔203°) channels even while the mechanism is stationary; firmware now flags and suppresses them instead of letting the angle walk away.
- **Motor Power Noise Test**: With the driver energized but idle, success rate improved to ~94%, yet warnings persist every cycle, confirming that EMI on the I2C wiring—not just motor motion—is the dominant issue.
- **Raw Diagnostic Mode**: Median filter remains stable, but single-read data oscillates wildly (e.g., pendulum raw counts 1 100–3 000 while filtered holds at 2 979). Software safeguards are working, but hardware noise remains unsolved.

### Outstanding Issues / Next Steps
1. I2C wiring still acts like an antenna; need to re-route SDA/SCL as short twisted pairs with ground, add 22–47 Ω series resistors, and keep them away from stepper phase leads.
2. Verify sensor magnets are perfectly centered and ~2 mm from each chip; misalignment can trigger the ML/MH bits and amplify noise.
3. Once cabling is improved, re-run calibration steps 3–5 to confirm the new filters can lock onto clean angles without constant warnings.

> Status: Noise reductions in firmware are complete, but the root EMI source persists. Hardware rework deferred to the next session.
- `docs/CapacitorInstallationGuide.md` - Comprehensive beginner guide for capacitor installation with polarity warnings, step-by-step instructions, and diagrams

**Updated Files**:
- `Log.md` - Added Nov 5 capacitor specifications
- Code comments - Enhanced I2C initialization documentation

### Lessons Learned

1. **Breadboard Quality Matters**: Not all breadboards have reliable internal connections. Power rail continuity can be intermittent.

2. **Capacitors Need Stabilization Time**: Added 200ms delays after capacitor installation for power to stabilize before I2C communication.

3. **Blocking I2C Calls**: AS5600 library's `detectMagnet()` and `begin()` functions can hang indefinitely without timeout protection.

4. **Debug Incrementally**: Break initialization into small steps with print statements between each to identify exact failure point.

5. **Test Power First**: Before debugging communication, verify power delivery to all components (5V at sensor VCC pins).

### Tomorrow's Work (November 7, 2025)

#### Priority 1: Hardware Verification
1. Test with different breadboard
2. Verify power rail continuity with multimeter
3. Check all connections with continuity tester
4. Measure voltage at critical points:
   - Arduino 5V pin
   - Breadboard power rails
   - Sensor VCC pins
   - Pull-up resistor connections

#### Priority 2: Sensor Communication Test
1. Upload latest code with debug output
2. Run I2C scanner (option 0)
3. Check which devices detected
4. Test sensors individually (disconnect one at a time)

#### Priority 3: If Sensors Work
1. Test sensor stability with motor power ON
2. Measure noise reduction from capacitors
3. Verify >90% stable readings during motor operation
4. Proceed with swing-up controller implementation

### System Status: ⚠️ Hardware Troubleshooting Required

**What Works**:
- Motor driver and control
- Code improvements (timeout protection, debug output)
- Capacitors installed correctly

**What Needs Fix**:
- Sensor I2C communication (breadboard issue suspected)
- Power distribution verification

**Blocked**:
- Cannot test capacitor effectiveness until sensors working
- Cannot proceed to control algorithms

---

## November 10, 2025

### Hardware Modifications Completed

**Capacitors Installed** ✅:
- 1x 100µF / 50V on TMC2209 VMOT to GND (motor power filtering)
- 1x 10µF / 50V on Arduino 5V to GND (logic power filtering)
- 2x 1µF / 63V on AS5600 sensors VCC to GND (local sensor filtering)

**Breadboard Reorganization**:
- Separated power domains for better noise isolation
- **Breadboard 1**: TMC2209 driver and motor control
- **Breadboard 2**: Both AS5600 sensors, pull-up resistors, capacitors, Arduino 5V/GND distribution
- Connected breadboards via shared 5V and GND rails

### Issues Encountered

#### Issue 1: Sensor Detection Failure After Breadboard Reorganization

**Problem**: After moving sensor connections to second breadboard, both AS5600 sensors stopped being detected by Arduino. System hangs during initialization at "Testing Pendulum Sensor..."

**Symptoms**:
- Code hangs at sensor initialization
- No I2C communication detected
- Motor driver still works (unaffected)

**Debugging Steps Taken**:
1. Verified power connections between breadboards
2. Checked SDA/SCL routing through breadboard
3. Confirmed pull-up resistors still in place
4. Switched VCC/GND positions on breadboard power rails

**Root Cause (Suspected)**: 
- Possible defective breadboard with broken internal connections
- Power rail continuity issues
- Signal integrity problems with longer traces through breadboard

**Code Improvements Made**:
- Added I2C timeout protection (`Wire.setWireTimeout()`)
- Implemented raw I2C device detection before AS5600 library calls
- Added granular debug output to identify exact hang point
- Changed from `detectMagnet()` to direct angle reading (less blocking)
- Increased initialization delays for capacitor stabilization (200ms)

**Status**: Pending hardware troubleshooting with replacement breadboard

### Code Changes

**File**: `src/main.cpp`

**Improvements**:
```cpp
// Added timeout protection
Wire.setWireTimeout(5000, true);  // 5ms timeout, reset on timeout

// Test I2C bus with raw transmission first
Wire.beginTransmission(0x36);
uint8_t error = Wire.endTransmission();

// Direct angle reading instead of blocking detectMagnet()
uint16_t test_angle = as5600_pendulum.readAngle();
```

**Benefits**:
- Prevents infinite hangs on I2C communication failures
- Provides detailed error messages for each failure point
- Easier to diagnose hardware vs software issues
- Graceful degradation instead of system freeze

### Documentation Updates

**New Files Created**:

## November 7, 2025

### Motor Sensor Characterisation & Integration
- Built a dedicated AS5600 motor-sensor calibration sketch (`src/motor_sensor_cal_tool.cpp.bak`) that runs on the SoftwareWire bus (SDA=22, SCL=24). The tool lets us zero the magnet, jog with adjustable increments, capture left/right limits, and prints both controller steps and absolute degrees for every move. Added auto-scaling so the jog increment can target ~10° moves once a span is known.
- Discovered the TMC2209 is still in high microstep mode (≈1/32), so one "step" reported by AccelStepper is only ~0.065–0.08° of shaft rotation. The calibration tool now computes `deg/step` directly from the measured sensor span and reuses it for consistent reporting.
- Verified the AS5600 output is linear and symmetric when zeroed at the midpoint. Any apparent asymmetry was due to re-zeroing by eye after calibration. The tool now auto-zeroes at the midpoint and reminds us to re-align the stepper count only after jogging to the sensor's 0°.

### Main Control Firmware Updates
- Restored the full pendulum controller in `src/main.cpp` and baked in the motor-mounted AS5600 via SoftwareWire. `updateSensors()` now samples both pendulum and motor encoders with the same 3-sample median filter used during diagnostics.
- Zeroing (menu option 2) now captures both pendulum and motor angles; limit calibration (option 4) records sensor angles alongside step counts, derives `motorStepsPerDeg`, and requires zero to be set first. The setup sequence refuses to start swing-up or balance until zero + limits + scale are all recorded.
- Live monitoring prints pendulum angle, motor angle, raw values, and steps in a single row so drift is immediately visible. Diagnostics (option 1) checks both hardware-I²C and SoftwareWire sensors for magnet presence.
- Balance control uses the AS5600 motor angle for the centering term (`Kp_motor` now acts in steps-per-motor-degree), and the "Return to Center" action leverages the sensor reading to remove accumulated drift instead of trusting the virtual encoder alone.
- Documented the new workflow in the menu output and kept the calibration helper (`motor_sensor_cal_tool.cpp.bak`) as a reference for future hardware checks.
- Added continuous sensor refresh in the main loop so motor angles remain valid outside the control states, fixed zeroing output to show the absolute motor angle captured, and updated the `Return to Center` routine to read the AS5600 before issuing a move—resolving the "key 5 does nothing" behavior.

---

## November 7, 2025

### Software Noise-Mitigation Sprint
- Tightened AS5600 plausibility filtering (`src/main.cpp`) by:
  - Dropping the per-step change threshold from 90° to 12°
  - Rejecting raw readings near 0/4095 and requiring 5 consecutive, matching samples before accepting any value
  - Adding continuous magnet-status checks so corrupted fields no longer overwrite the last good angle
- Introduced post-step settling delays (500 µs) after every manual jog and AccelStepper move to keep AS5600 reads away from driver current spikes.
- Added “large jump hold” logic that automatically accepts a big transition once five identical samples arrive; prevents the controller from staying stuck on stale data when the arm really moves.

### Diagnostic Session Results (Options D, N, R)
- **Live Sensor Test (motor disabled)**: Still shows repeated 70–90° spikes on both pendulum (≈98↔118°) and motor (≈193↔203°) channels even while the mechanism is stationary; firmware now flags and suppresses them instead of letting the angle walk away.
- **Motor Power Noise Test**: With the driver energized but idle, success rate improved to ~94%, yet warnings persist every cycle, confirming that EMI on the I2C wiring—not just motor motion—is the dominant issue.
- **Raw Diagnostic Mode**: Median filter remains stable, but single-read data oscillates wildly (e.g., pendulum raw counts 1 100–3 000 while filtered holds at 2 979). Software safeguards are working, but hardware noise remains unsolved.

### Outstanding Issues / Next Steps
1. I2C wiring still acts like an antenna; need to re-route SDA/SCL as short twisted pairs with ground, add 22–47 Ω series resistors, and keep them away from stepper phase leads.
2. Verify sensor magnets are perfectly centered and ~2 mm from each chip; misalignment can trigger the ML/MH bits and amplify noise.
3. Once cabling is improved, re-run calibration steps 3–5 to confirm the new filters can lock onto clean angles without constant warnings.

> Status: Noise reductions in firmware are complete, but the root EMI source persists. Hardware rework deferred to the next session.
- `docs/CapacitorInstallationGuide.md` - Comprehensive beginner guide for capacitor installation with polarity warnings, step-by-step instructions, and diagrams

**Updated Files**:
- `Log.md` - Added Nov 5 capacitor specifications
- Code comments - Enhanced I2C initialization documentation

### Lessons Learned

1. **Breadboard Quality Matters**: Not all breadboards have reliable internal connections. Power rail continuity can be intermittent.

2. **Capacitors Need Stabilization Time**: Added 200ms delays after capacitor installation for power to stabilize before I2C communication.

3. **Blocking I2C Calls**: AS5600 library's `detectMagnet()` and `begin()` functions can hang indefinitely without timeout protection.

4. **Debug Incrementally**: Break initialization into small steps with print statements between each to identify exact failure point.

5. **Test Power First**: Before debugging communication, verify power delivery to all components (5V at sensor VCC pins).

### Tomorrow's Work (November 7, 2025)

#### Priority 1: Hardware Verification
1. Test with different breadboard
2. Verify power rail continuity with multimeter
3. Check all connections with continuity tester
4. Measure voltage at critical points:
   - Arduino 5V pin
   - Breadboard power rails
   - Sensor VCC pins
   - Pull-up resistor connections

#### Priority 2: Sensor Communication Test
1. Upload latest code with debug output
2. Run I2C scanner (option 0)
3. Check which devices detected
4. Test sensors individually (disconnect one at a time)

#### Priority 3: If Sensors Work
1. Test sensor stability with motor power ON
2. Measure noise reduction from capacitors
3. Verify >90% stable readings during motor operation
4. Proceed with swing-up controller implementation

### System Status: ⚠️ Hardware Troubleshooting Required

**What Works**:
- Motor driver and control
- Code improvements (timeout protection, debug output)
- Capacitors installed correctly

**What Needs Fix**:
- Sensor I2C communication (breadboard issue suspected)
- Power distribution verification

**Blocked**:
- Cannot test capacitor effectiveness until sensors working
- Cannot proceed to control algorithms

---

## November 10, 2025

### Hardware Modifications Completed

**Capacitors Installed** ✅:
- 1x 100µF / 50V on TMC2209 VMOT to GND (motor power filtering)
- 1x 10µF / 50V on Arduino 5V to GND (logic power filtering)
- 2x 1µF / 63V on AS5600 sensors VCC to GND (local sensor filtering)

**Breadboard Reorganization**:
- Separated power domains for better noise isolation
- **Breadboard 1**: TMC2209 driver and motor control
- **Breadboard 2**: Both AS5600 sensors, pull-up resistors, capacitors, Arduino 5V/GND distribution
- Connected breadboards via shared 5V and GND rails

### Issues Encountered

#### Issue 1: Sensor Detection Failure After Breadboard Reorganization

**Problem**: After moving sensor connections to second breadboard, both AS5600 sensors stopped being detected by Arduino. System hangs during initialization at "Testing Pendulum Sensor..."

**Symptoms**:
- Code hangs at sensor initialization
- No I2C communication detected
- Motor driver still works (unaffected)

**Debugging Steps Taken**:
1. Verified power connections between breadboards
2. Checked SDA/SCL routing through breadboard
3. Confirmed pull-up resistors still in place
4. Switched VCC/GND positions on breadboard power rails

**Root Cause (Suspected)**: 
- Possible defective breadboard with broken internal connections
- Power rail continuity issues
- Signal integrity problems with longer traces through breadboard



**Code Improvements Made**:
- Added I2C timeout protection (`Wire.setWireTimeout()`)
- Implemented raw I2C device detection before AS5600 library calls
- Added granular debug output to identify exact hang point
- Changed from `detectMagnet()` to direct angle reading (less blocking)
- Increased initialization delays for capacitor stabilization (200ms)

**Status**: Pending hardware troubleshooting with replacement breadboard

### Code Changes

**File**: `src/main.cpp`

**Improvements**:
```cpp
// Added timeout protection
Wire.setWireTimeout(5000, true);  // 5ms timeout, reset on timeout

// Test I2C bus with raw transmission first
Wire.beginTransmission(0x36);
uint8_t error = Wire.endTransmission();

// Direct angle reading instead of blocking detectMagnet()
uint16_t test_angle = as5600_pendulum.readAngle();
```

**Benefits**:
- Prevents infinite hangs on I2C communication failures
- Provides detailed error messages for each failure point
- Easier to diagnose hardware vs software issues
- Graceful degradation instead of system freeze

### Documentation Updates

**New Files Created**:

## November 7, 2025

### Motor Sensor Characterisation & Integration
- Built a dedicated AS5600 motor-sensor calibration sketch (`src/motor_sensor_cal_tool.cpp.bak`) that runs on the SoftwareWire bus (SDA=22, SCL=24). The tool lets us zero the magnet, jog with adjustable increments, capture left/right limits, and prints both controller steps and absolute degrees for every move. Added auto-scaling so the jog increment can target ~10° moves once a span is known.
- Discovered the TMC2209 is still in high microstep mode (≈1/32), so one "step" reported by AccelStepper is only ~0.065–0.08° of shaft rotation. The calibration tool now computes `deg/step` directly from the measured sensor span and reuses it for consistent reporting.
- Verified the AS5600 output is linear and symmetric when zeroed at the midpoint. Any apparent asymmetry was due to re-zeroing by eye after calibration. The tool now auto-zeroes at the midpoint and reminds us to re-align the stepper count only after jogging to the sensor's 0°.

### Main Control Firmware Updates
- Restored the full pendulum controller in `src/main.cpp` and baked in the motor-mounted AS5600 via SoftwareWire. `updateSensors()` now samples both pendulum and motor encoders with the same 3-sample median filter used during diagnostics.
- Zeroing (menu option 2) now captures both pendulum and motor angles; limit calibration (option 4) records sensor angles alongside step counts, derives `motorStepsPerDeg`, and requires zero to be set first. The setup sequence refuses to start swing-up or balance until zero + limits + scale are all recorded.
- Live monitoring prints pendulum angle, motor angle, raw values, and steps in a single row so drift is immediately visible. Diagnostics (option 1) checks both hardware-I²C and SoftwareWire sensors for magnet presence.
- Balance control uses the AS5600 motor angle for the centering term (`Kp_motor` now acts in steps-per-motor-degree), and the "Return to Center" action leverages the sensor reading to remove accumulated drift instead of trusting the virtual encoder alone.
- Documented the new workflow in the menu output and kept the calibration helper (`motor_sensor_cal_tool.cpp.bak`) as a reference for future hardware checks.
- Added continuous sensor refresh in the main loop so motor angles remain valid outside the control states, fixed zeroing output to show the absolute motor angle captured, and updated the `Return to Center` routine to read the AS5600 before issuing a move—resolving the "key 5 does nothing" behavior.

---

## November 7, 2025

### Software Noise-Mitigation Sprint
- Tightened AS5600 plausibility filtering (`src/main.cpp`) by:
  - Dropping the per-step change threshold from 90° to 12°
  - Rejecting raw readings near 0/4095 and requiring 5 consecutive, matching samples before accepting any value
  - Adding continuous magnet-status checks so corrupted fields no longer overwrite the last good angle
- Introduced post-step settling delays (500 µs) after every manual jog and AccelStepper move to keep AS5600 reads away from driver current spikes.
- Added “large jump hold” logic that automatically accepts a big transition once five identical samples arrive; prevents the controller from staying stuck on stale data when the arm really moves.

### Diagnostic Session Results (Options D, N, R)
- **Live Sensor Test (motor disabled)**: Still shows repeated 70–90° spikes on both pendulum (≈98↔118°) and motor (≈193↔203°) channels even while the mechanism is stationary; firmware now flags and suppresses them instead of letting the angle walk away.
- **Motor Power Noise Test**: With the driver energized but idle, success rate improved to ~94%, yet warnings persist every cycle, confirming that EMI on the I2C wiring—not just motor motion—is the dominant issue.
- **Raw Diagnostic Mode**: Median filter remains stable, but single-read data oscillates wildly (e.g., pendulum raw counts 1 100–3 000 while filtered holds at 2 979). Software safeguards are working, but hardware noise remains unsolved.

### Outstanding Issues / Next Steps
1. I2C wiring still acts like an antenna; need to re-route SDA/SCL as short twisted pairs with ground, add 22–47 Ω series resistors, and keep them away from stepper phase leads.
2. Verify sensor magnets are perfectly centered and ~2 mm from each chip; misalignment can trigger the ML/MH bits and amplify noise.
3. Once cabling is improved, re-run calibration steps 3–5 to confirm the new filters can lock onto clean angles without constant warnings.

> Status: Noise reductions in firmware are complete, but the root EMI source persists. Hardware rework deferred to the next session.
- `docs/CapacitorInstallationGuide.md` - Comprehensive beginner guide for capacitor installation with polarity warnings, step-by-step instructions, and diagrams

**Updated Files**:
- `Log.md` - Added Nov 5 capacitor specifications
- Code comments - Enhanced I2C initialization documentation

### Lessons Learned

1. **Breadboard Quality Matters**: Not all breadboards have reliable internal connections. Power rail continuity can be intermittent.

2. **Capacitors Need Stabilization Time**: Added 200ms delays after capacitor installation for power to stabilize before I2C communication.

3. **Blocking I2C Calls**: AS5600 library's `detectMagnet()` and `begin()` functions can hang indefinitely without timeout protection.

4. **Debug Incrementally**: Break initialization into small steps with print statements between each to identify exact failure point.

5. **Test Power First**: Before debugging communication, verify power delivery to all components (5V at sensor VCC pins).

### Tomorrow's Work (November 7, 2025)

#### Priority 1: Hardware Verification
1. Test with different breadboard
2. Verify power rail continuity with multimeter
3. Check all connections with continuity tester
4. Measure voltage at critical points:
   - Arduino 5V pin
   - Breadboard power rails
   - Sensor VCC pins
   - Pull-up resistor connections

#### Priority 2: Sensor Communication Test
1. Upload latest code with debug output
2. Run I2C scanner (option 0)
3. Check which devices detected
4. Test sensors individually (disconnect one at a time)

#### Priority 3: If Sensors Work
1. Test sensor stability with motor power ON
2. Measure noise reduction from capacitors
3. Verify >90% stable readings during motor operation
4. Proceed with swing-up controller implementation

### System Status: ⚠️ Hardware Troubleshooting Required

**What Works**:
- Motor driver and control
- Code improvements (timeout protection, debug output)
- Capacitors installed correctly

**What Needs Fix**:
- Sensor I2C communication (breadboard issue suspected)
- Power distribution verification

**Blocked**:
- Cannot test capacitor effectiveness until sensors working
- Cannot proceed to control algorithms

---

## November 19, 2025 – Control Loop Timing & Authority Fix

### Diagnosis: The "Meek" Motor
- User reported the motor felt weak and "meek," with the base crawling slowly despite high PID output.
- **Root Cause Analysis**: The control loop was running at **1kHz (1000µs)**, but the `SoftwareWire` implementation for the motor sensor takes **~2-3ms** per read (bit-banged I2C).
- **Effect**: The loop was overrunning its deadline by 200-300%. `AccelStepper` requires frequent calls to `run()` to generate pulses. By blocking for 3ms inside a 1ms loop, the stepper driver was being starved, effectively capping the step rate to ~200-300 steps/sec regardless of the speed setting.
- **Secondary Issue**: `resyncStepperFromSensor()` was being called unconditionally or too frequently. This function calls `setCurrentPosition()`, which has the side effect of **resetting the current speed to 0.0**. This prevented the motor from ever accelerating to high speeds.

### Firmware Architecture Changes (`src/main.cpp`)
1. **Loop Rate Reduction**: Lowered `CONTROL_LOOP_US` from 1000 (1kHz) to **5000 (200Hz)**. This guarantees the 3ms I2C read finishes within the timeslot, leaving 2ms for stepper pulse generation.
2. **Absolute Position Control**: Switched from relative `stepper.move(delta)` to absolute `stepper.moveTo(target)`.
   - Introduced `balanceTargetDegF` accumulator to track the "virtual" setpoint.
   - This ensures that even if a loop cycle is jittery, the driver knows the absolute destination and can accelerate to catch up.
3. **Gated Synchronization**: Modified `resyncStepperFromSensor()` to only run when `stepper.distanceToGo() == 0`. This keeps the virtual position aligned when static but prevents speed resets during active motion.
4. **Manual I2C Implementation**: Replaced the incompatible `AS5600` library calls for the motor sensor with manual `SoftwareWire` register reads, fixing compilation errors and ensuring reliable data.
5. **High-Torque Profiles**: Restored aggressive acceleration (`MOTOR_ACCEL = 12000`, `BALANCE_ACCEL = 10000`) now that the timing budget allows the driver to actually execute them.

### Telemetry & Tooling
- Added `[BALANCE_GAINS]` packet injection to the serial stream so logs automatically record the active PID values.
- Updated `tools/balance_plot.py` to parse the new gains and handle the `base_deg` column correctly.
- Fixed "runaway centering" bug where stale sensor data caused the motor to jog endlessly; added `updateSensors()` calls before critical moves.

### Current Status
- Firmware compiles and uploads.
- Control loop is physically realizable (200Hz).
- Motor authority should be restored (no more speed resets).
- **Next Steps**: Manual upload, calibration (C->Z->L), and PID tuning. The new "position control" scheme is more aggressive, so gains may need to be lowered (start Kp ~10-15).

## November 19, 2025 (Session 2) – Calibration & Sensor Error Fixes

### Issue 1: "Rail Hammering" During Calibration
**Symptoms**: During limit calibration (Mode 4), the motor would move very slightly and then stop, or hammer against the rail without moving far enough.
**Diagnosis**: Circular dependency in `calibrateLimits()`. The function used `degPerStep` to calculate how many steps to move (`target_angle / degPerStep`). However, `degPerStep` was being dynamically updated by `resyncStepperFromSensor()` during the move. If the initial `degPerStep` was wrong (e.g., default 1.8° vs actual 0.06°), the calculated step target was tiny.
**Fix**: 
- Modified `calibrateLimits()` to use **raw steps** (200 steps) for the calibration move instead of calculated degrees.
- Disabled `resyncStepperFromSensor` during the calibration move to prevent the scale factor from being corrupted while we are trying to measure it.
- This ensures the motor physically moves a known distance (200 microsteps) so we can measure the angle change and compute a valid `degPerStep`.

### Issue 2: False "Sensor Failed" Errors
**Symptoms**: The system would trigger an Emergency Stop with "Sensor Failed" whenever the motor passed through the zero position.
**Diagnosis**: The `readMotorRaw()` function returned `0` to indicate an I2C error. However, `0` is also a valid angle reading (0 degrees). When the sensor read exactly 0, the system interpreted it as a disconnection.
**Fix**:
- Changed the error sentinel value in `readMotorRaw()` from `0` to `0xFFFF` (65535).
- Updated `updateSensors()` to check for `0xFFFF` instead of `0`.
- Added a retry mechanism (3 attempts) in `readMotorRaw()` to make the I2C reading more robust against noise.

### Current Status
- **Firmware**: `src/main.cpp` contains all fixes.
- **Ready for**:
  1. Upload.
  2. Run Calibration (Mode 4) to finally compute the correct `degPerStep`.
  3. Verify no E-Stops at zero position.

## November 19, 2025 (Session 3) – Centering Drift & Control Authority

### Work Summary
- Verified the latest firmware on hardware. Menu now prints live motor/pendulum angles after each command, and centering uses the AS5600 plus iterative resync (`centerMoveCalibrating`).
- Repeated zero override, limit calibration, and balance attempts with multiple gain sets (Kp=-20…-40, Ki/Kd sweeps, motor-centering on/off).
- Tested both position-based and velocity-based control (temporarily switched to `setSpeed/runSpeed` to see if open-loop speed would restore torque—reverted afterward).

### Observations
1. **Motor Zero Drift**: Hardcoded zero (159.96°) does not match the actual mechanical center (~2° raw). Without overriding, the `5` command drives straight into a limit. After overriding, the menu shows the correct angle, so the override path works.
2. **Base Hardly Moves During Balance**: Logs show control output saturating at ±30° while the base encoder moves <3°. The pendulum then drifts/falls despite the PID going full scale. This happens even with newly calibrated limits.
3. **deg/step Learning Blind Spot**: During limit calibration we still resync the stepper to the sensor, so `currentPosition()` mirrors `motorAngle`/`degPerStep`. When we compute `(Δdeg)/(Δsteps)` we’re effectively dividing the sensor by itself, so the scale never updates. The controller therefore believes 30° equals a handful of microsteps, and the base looks “weak”.
4. **Velocity Experiment**: Switching to velocity control (no position targets, only `setSpeed`) produced slightly more motion but ran open loop; without encoder feedback the step count immediately diverged and the controller lost track of where the base actually was.

### Diagnosis
- The core issue is still an incorrect `degPerStep`. Because we resync AccelStepper on every sensor read, the step count used for calibration is not raw steps; it’s derived from the same angle we divide by. As a result, aggressive control commands translate into tiny physical moves, making the system look torque-starved even though the driver is saturated.

### Next Actions
1. Rework calibration to record **raw microsteps** (no resync) between left/right limits, or maintain a separate raw step counter.
2. Once `degPerStep` reflects reality, re-run Ziegler–Nichols or similar tuning with the sensor still in the loop.
3. Reconfirm direction (`DIR_SIGN`) and zero so that `moveTo(±30°)` physically sweeps the arm to each edge before re-attempting velocity control ideas.
