# Backup Scripts Reference

**Last Updated:** November 4, 2025

This document catalogs all code versions and provides restoration commands.

---

## Current Active Script

### `main.cpp` (Step-Based Calibration System)

**Purpose:** Interactive calibration with manual jogging and safety features

**Features:**
- Zero position setting (pendulum upright, arm centered)
- Live sensor monitoring with EMI testing
- Manual single-step jogging (A/D keys)
- Step-based limit recording
- Full range testing between calibrated limits
- Emergency stop (press 'S' during movement)
- I2C bus scanning
- Safety warnings before motor enable

**Calibration Status:**
- ✅ Left Limit: -280 steps
- ✅ Right Limit: +280 steps
- ✅ Zero position: Pendulum upright, arm center

**Key Functions:**
- `manualStep()` - Direct motor pulse generation (bypasses AccelStepper)
- `setLeftLimit()` / `setRightLimit()` - Interactive jogging to edges
- `liveSensorTest()` - Real-time readings with motor driver disabled for EMI diagnosis
- `testFullRange()` - Automated movement between calibrated limits

---

## Backup Scripts (in `src/` directory)

### 1. `main_step_calibration.cpp.bak` ✓ RECOMMENDED

**Status:** Verified working, identical to current main.cpp

**Date Created:** November 4, 2025

**Purpose:** Backup of working calibration system

**Use Case:** Restore this if you need the stable, tested calibration interface

**Restore:**
```bash
cd /Users/piyush/code/Rotary-Inverted-Pendulum
cp src/main_step_calibration.cpp.bak src/main.cpp
platformio run --target upload
```

---

### 2. `main_full_pid_control.cpp.bak`

**Status:** Not tested with current hardware

**Date Created:** November 4, 2025 (early version)

**Purpose:** Complete PID balance control system

**Features:**
- Automatic homing procedure
- PID controller for pendulum balancing (KP=15, KI=0.1, KD=5)
- Position limits (hard and soft)
- Emergency stop
- Multiple operating modes (IDLE, HOMING, BALANCE_CONTROL)

**Why Backup:** Created before calibration workflow was established

**Use Case:** Reference for future control algorithm implementation

**Restore:**
```bash
cp src/main_full_pid_control.cpp.bak src/main.cpp
platformio run --target upload
```

**Note:** Will need adjustment for current calibration values (±280 steps)

---

### 3. `main_angle_calibration.cpp.bak` ❌ DEPRECATED

**Status:** Failed approach, kept for reference

**Date Created:** November 4, 2025

**Purpose:** Attempted angle-based limit calibration using AS5600 sensor readings

**Why It Failed:**
- AS5600 sensor angle wraparound at 0°/360° boundary
- Erratic readings when pendulum crosses this boundary
- Readings jumped wildly: 70° → -30° → 90° → -50°

**Lesson Learned:** Step-based tracking is more reliable than sensor angles for limit detection

**Do NOT Use** - Kept only as documentation of failed approach

---

### 4. `test_all_sensors.cpp.bak`

**Purpose:** Basic dual-sensor test script

**Features:**
- Tests both AS5600 sensors simultaneously
- Reads angles from Hardware I2C (pendulum) and Software I2C (motor)
- Simple continuous output

**Use Case:** Quick sensor connectivity verification

**Restore:**
```bash
cp src/test_all_sensors.cpp.bak src/main.cpp
platformio run --target upload
```

---

### 5. `test_all_direction.cpp.bak`

**Purpose:** Motor and sensor direction characterization

**Features:**
- Tests motor movement in both directions
- Correlates motor direction with sensor angle changes
- Determines CW vs CCW behavior

**Results Documented:**
- Motor moves CCW then CW
- Both sensors: CW decreases angle, CCW increases angle

**Use Case:** Hardware characterization for new builds

---

### 6. `motor_test.cpp.bak`

**Purpose:** Basic motor driver functionality test

**Features:**
- Simple stepper motor movement test
- No sensor integration

**Use Case:** Verify motor driver wiring without sensors connected

---

### 7. `main.cpp.backup` (Original)

**Purpose:** Very first hardware test code

**Features:**
- Basic motor direction test
- Sensor streaming
- I2C bus scanning

**Use Case:** Historical reference only

---

## Quick Command Reference

### View Available Backups
```bash
cd /Users/piyush/code/Rotary-Inverted-Pendulum/src
ls -lh *.bak
```

### Restore Specific Backup
```bash
# Replace <backup_name> with desired backup file
cp src/<backup_name>.bak src/main.cpp
platformio run --target upload
```

### Create New Backup
```bash
# Backup current main.cpp before making changes
cp src/main.cpp src/main_$(date +%Y%m%d_%H%M%S).cpp.bak
```

### Build and Upload Current Script
```bash
platformio run --target upload
```

### Open Serial Monitor
```bash
platformio device monitor --baud 115200
```

---

## Development Workflow

### Before Making Major Changes:

1. **Backup current working version:**
   ```bash
   cp src/main.cpp src/main_backup_YYYY-MM-DD.cpp.bak
   ```

2. **Document in this file** what the backup contains

3. **Make your changes**

4. **Test thoroughly**

5. **If successful:** Update "Current Active Script" section above

6. **If failed:** Restore previous backup

---

## Script Selection Guide

**Need to calibrate system from scratch?**
→ Use `main_step_calibration.cpp.bak` (current main.cpp)

**Need to test sensors only?**
→ Use `test_all_sensors.cpp.bak`

**Need to characterize new hardware?**
→ Use `test_all_direction.cpp.bak`

**Need PID control reference?**
→ See `main_full_pid_control.cpp.bak` (not yet adapted)

**Need to test motor driver only?**
→ Use `motor_test.cpp.bak`

---

## Code Evolution History

1. **Initial Hardware Test** (`main.cpp.backup`)
   - Basic connectivity verification

2. **Sensor Direction Test** (`test_all_direction.cpp.bak`)
   - Determined motor/sensor direction mapping

3. **Angle-Based Calibration** (`main_angle_calibration.cpp.bak`) ❌
   - FAILED: Sensor wraparound issues

4. **Step-Based Calibration** (`main_step_calibration.cpp.bak`) ✓
   - SUCCESS: Current working solution
   - Fixed AccelStepper movement issues
   - Added manual pulse generation
   - Implemented safety features

5. **Next: Control Implementation**
   - Will use PID control framework from `main_full_pid_control.cpp.bak`
   - Adapt to step-based limits (±280 steps)
   - Add after capacitor installation (Nov 5)

---

## Notes

- All backups are in `src/` directory
- Only `src/main.cpp` is compiled by PlatformIO
- Keep this document updated when creating new backups
- Always test in safe conditions with emergency stop ready
- Document calibration values in backup filename if specific to hardware setup

**Safety:** Always have emergency power disconnect ready when testing motor movements!

---

**Repository:** [github.com/Piyushiitk24/Rotary-Inverted-Pendulum](https://github.com/Piyushiitk24/Rotary-Inverted-Pendulum)


## Available Scripts

### 1. `main_step_calibration.cpp.bak` (LATEST WORKING)
**Purpose:** Step-based calibration with direct motor control (bypasses AccelStepper for jogging)
**Features:**
- Manual single-step jogging using A/D keys
- Direct digitalWrite pulse generation for reliable movement
- Step-based limit recording (more reliable than sensor angles)
- Live sensor testing
- Full range testing between limits

**Status:** ✓ Motor movement confirmed working
**Calibration achieved:** Left=-280, Right=+280 steps (560 total range = 1008°)

**To restore:**
```bash
cp src/main_step_calibration.cpp.bak src/main.cpp
platformio run --target upload
```

---

### 2. `main.cpp` (CURRENT - SENSOR DIAGNOSTIC TOOL)
**Purpose:** Comprehensive sensor testing and troubleshooting
**Features:**
- Full diagnostic suite for both AS5600 sensors
- I2C bus scanning (Hardware and Software I2C)
- Magnet detection and strength testing
- AGC (Automatic Gain Control) verification
- Live angle readings with change indicators
- Pin configuration display

**Menu Options:**
1. Full Diagnostic (run all tests)
2. I2C Bus Scan
3. Test Pendulum Sensor (Hardware I2C)
4. Test Motor Sensor (Software I2C)
5. Live Angle Readings
6. Show Pin Configuration

**Use this to:**
- Diagnose motor sensor issues
- Verify magnet positioning
- Check I2C communication
- Confirm sensor wiring

---

### 3. `main_full_pid_control.cpp.bak`
**Purpose:** Complete PID balance control system with automatic homing
**Features:**
- Automatic homing procedure
- PID controller for pendulum balancing
- Position limits (hard and soft)
- Emergency stop
- Multiple operating modes

**To use:** Copy to `main.cpp`
```bash
cp src/main_full_pid_control.cpp.bak src/main.cpp
```

---

### 2. `main.cpp` (CURRENT)
**Purpose:** Step-by-step calibration and testing
**Features:**
- Manual zero position setting
- Live sensor testing and verification
- Manual edge limit recording
- Motor movement testing within limits
- Manual jogging controls

**Steps to follow:**
1. Press `1` - Set zero position (center arm, pendulum up, press Z)
2. Press `2` - Live sensor test (verify both sensors work)
3. Press `3` - Record LEFT edge (move arm manually to left limit, press L)
4. Press `4` - Record RIGHT edge (move arm manually to right limit, press R)
5. Press `5` - View calibration data
6. Press `6` - Test motor movement within limits
7. Press `7/8` - Manual jog CW/CCW
8. Press `9` - Return to zero

---

### 3. `main.cpp.backup` (Original test code)
**Purpose:** Original hardware characterization test
**Features:**
- Basic motor direction test
- Sensor streaming
- I2C bus scanning

---

## Quick Commands

### Restore a backup:
```bash
# Full PID control
cp src/main_full_pid_control.cpp.bak src/main.cpp

# Original test code
cp src/main.cpp.backup src/main.cpp
```

### Build and upload:
```bash
platformio run --target upload
```

### Open serial monitor:
```bash
platformio device monitor --baud 115200
```

---

## Next Steps (After Calibration)

Once you've completed calibration with the current script:

1. **Record your calibration values** (from option 5):
   - Pendulum Zero
   - Motor Zero
   - Motor MIN (left edge)
   - Motor MAX (right edge)

2. **Create swing-up script** with these hardcoded limits

3. **Test swing-up motion** before attempting balance control

---

## Notes

- All backups are in the `src/` directory
- Only `main.cpp` is compiled by PlatformIO
- Keep this document updated when creating new scripts
- Always test in safe conditions with emergency stop ready
