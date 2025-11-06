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

## System Status: ✅ Calibrated, ⚠ Pending Hardware Verification

**What Works**:
- Motor movement (280 steps calibrated range)
- Step position tracking
- Sensor readings (when motor power off)
- Safety systems and emergency stop

**What Needs Fix**:
- Add capacitors for noise filtering
- Sensor stability during motor operation

**Ready For**:
- Capacitor installation tomorrow
- Control algorithm implementation after hardware fixes

---