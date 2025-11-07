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

---

## November 7, 2025

### Critical Issue Discovered: EMI-Induced Sensor Bit-Flip Corruption

#### Problem Analysis

**Calibration Test Results** (Options 3, 4, 5):
- ✅ Motor stepping: Perfect progression -205 to +205 steps
- ✅ Step counter: No missed steps
- ❌ Pendulum sensor: **180° jumps every 2-5 steps during motor movement**
- ❌ Motor sensor: 15-30° occasional jumps

**Symptom Pattern**:
```
Step -1: Pend: -35.9° ← Normal
Step -2: Pend: 170.9° ← JUMP +206.8°
Step -3: Pend: -36.0° ← JUMP back -206.9°
Step -4: Pend: -35.9° ← Stable again
```

**Root Cause Identified**:
- AS5600 outputs 12-bit angle (0-4095 = 0-360°)
- **Bit 11 (MSB) = 2048 counts = 180°**
- Motor driver current spikes generate EMI during each step
- EMI couples into I2C signal wires (breadboard jumpers = antennas)
- **MSB bit flips → exactly ±180° errors**
- Pattern: Readings flip between two clusters ~180° apart

**Why This Happens**:
1. Motor driver: Sharp current spikes (~1-2A, <1µs risetime)
2. Long I2C wires: Act as EMI antennas
3. 400kHz I2C: Fast edges vulnerable to noise
4. Single read per step: No error detection/correction
5. Capacitors: Filter power noise but not signal-line EMI

**Evidence**:
- Pendulum: Alternates between `-35°` and `+170°` zones
- Motor: Toggles between `150°` and `205°` zones
- Jumps synchronized with motor steps, not physical motion
- Both sensors fail independently → signal integrity issue

#### Solutions Implemented

**Code Changes** (`src/main.cpp`):

1. **Median Filtering** (3-sample):
   - Functions: `readPendulumAngleMedian()`, `readAS5600AngleMedian()`
   - Takes 3 readings 100µs apart, returns middle value
   - Rejects single-bit flip outliers
   ```cpp
   uint16_t readPendulumAngleMedian() {
     uint16_t r1 = as5600_pendulum.readAngle();
     delayMicroseconds(100);
     uint16_t r2 = as5600_pendulum.readAngle();
     delayMicroseconds(100);
     uint16_t r3 = as5600_pendulum.readAngle();
     // Sort and return median
   }
   ```

2. **Change-Rate Limiting**:
   - Modified: `updateSensorReadings()`
   - Tracks previous reading
   - Rejects jumps >90° between updates
   - Holds last good value when corruption detected
   ```cpp
   if (abs(delta) > 90.0) {
     pendulum_deg = lastPendulumRaw;  // Use previous good value
     Serial.println("[WARN] Pendulum jump rejected");
   }
   ```

3. **Magnet Field Validation**:
   - New function: `checkPendulumMagnet()`
   - Reads AS5600 status register
   - Checks MD (magnet detected), ML (too weak), MH (too strong) bits
   - Warns if magnet distance out of 2-3mm range

4. **I2C Speed Reduction**:
   - Hardware I2C: 400kHz → 100kHz
   - Slower edges = more noise immunity
   - Longer bit times = higher signal-to-noise ratio

5. **Raw Diagnostic Mode**:
   - New menu option: Press 'R'
   - Shows single reads vs median-filtered reads side-by-side
   - Displays raw 12-bit values
   - Highlights ⚠ when EMI detected
   - Usage: Watch for corrupted single reads vs stable filtered reads

**New Variables Added**:
```cpp
float lastPendulumRaw = 0.0;
float lastMotorRaw = 0.0;
bool firstReading = true;
```

**Expected Results**:
- Corrupted reads automatically detected and rejected
- Sensor angles stay stable despite EMI
- `[WARN]` messages show when jumps caught
- System usable for control algorithms

#### Hardware Fixes Recommended (Not Yet Implemented)

**Why Twist I2C Wires**:
- EMI induces voltage in wire loops
- Parallel wires: Large loop → picks up EMI
- Twisted wires: Many tiny loops with opposite polarity → EMI cancels out
- **Differential cancellation**: 50-80% reduction in EMI pickup
- Same principle as Ethernet CAT5/6 cables

**Priority Order**:
1. **Twist I2C wires** (~1 twist per inch) - 50-80% improvement, free, 2 minutes
2. **Ferrite beads** on SDA/SCL near sensors - 60-90% improvement
3. **Separate motor ground** from sensor ground - eliminates ground bounce
4. **Shielded cable** for permanent installation - 90-99% EMI rejection

#### Documentation Created

**New File**: `docs/EMI_Analysis_and_Fixes.md`
- Complete EMI analysis
- Bit-flip mechanism explanation
- Software mitigation details
- Hardware fix instructions
- Testing strategy
- Expected improvement metrics

#### Testing Plan

**Phase 1: Software-Only (Current)**:
1. Upload code with median filtering + change-rate limiting
2. Run **Option R** (Raw Diagnostic Mode)
   - Watch for ⚠ warnings showing rejected jumps
   - Compare single vs median reads
3. Run **Options 3/4** (Set Limits) again
   - Look for `[WARN] jump rejected` messages
   - Filtered readings should stay stable

**Phase 2: Twist Wires**:
1. Twist I2C wires (easiest hardware fix)
2. Run Option R again
3. Count reduction in ⚠ warnings
4. Expected: 50-80% fewer EMI events

**Phase 3: Ferrite Beads** (if needed):
1. Add beads on all 4 I2C lines
2. Run full calibration (Options 1-5)
3. Expected: <5% error rate

**Success Criteria**: <1% corrupted readings, smooth angle tracking during motor movement

#### Lessons Learned

1. **Capacitors Filter Power, Not Signals**: 
   - Power rail capacitors work (72% stable when stationary)
   - Don't protect against EMI coupling into signal wires

2. **Bit-Level Error Detection Required**:
   - Single reads insufficient in noisy environment
   - Median filtering + change-rate limiting = robust solution

3. **I2C Speed vs Noise Trade-off**:
   - 400kHz fast but vulnerable to EMI
   - 100kHz slower but more noise-immune
   - Still fast enough for control (100 reads/sec sufficient)

4. **Hardware Issues Require Hardware Fixes**:
   - Software can mitigate but not eliminate EMI
   - Twisted wires + ferrite beads = proper engineering solution

5. **Diagnostic Tools Essential**:
   - Raw diagnostic mode reveals corruption patterns
   - Helps validate both software and hardware fixes

### System Status: ✅ Software Fixes Ready, ⚠ Hardware Improvements Recommended

**What Works**:
- Motor stepping (±205 steps calibrated)
- Step tracking (100% accurate)
- Median filtering (rejects bit flips)
- Change-rate limiting (catches persistent errors)
- Raw diagnostic mode (visualizes EMI)

**What's Improved**:
- Sensor readings stable despite EMI
- Automatic error detection and rejection
- System ready for control testing

**What's Recommended**:
- Twist I2C wires for 50-80% EMI reduction
- Add ferrite beads if needed
- Test control algorithms with current software protection

**Ready For**:
- Upload and test with Option R
- Twist wires if too many ⚠ warnings
- Swing-up controller implementation

---

## System Status: ✅ Calibrated, ✅ EMI Mitigation Active, ⚠ Pending Test & Validation

**What Works**:
- Motor movement (±205 steps calibrated range)
- Step position tracking (100% accurate)
- Sensor readings with EMI protection
- Safety systems and emergency stop

**What's New**:
- Median filtering rejects bit flips
- Change-rate limiting catches corruption
- I2C speed reduced to 100kHz
- Raw diagnostic mode available

**Next Steps**:
1. Test software fixes with Option R
2. Twist I2C wires if needed
3. Validate stability during motor movement
4. Proceed to control algorithm implementation

---
