# Motor AS5600 Sensor Test Guide

## Quick Start

**Current Code**: `src/main.cpp` → Motor sensor isolated test script
**Full Control Backup**: `src/main_full_control.cpp.bak`

## Hardware Configuration

### Magnetic Disk Setup
- ✅ Custom plastic housing (3.6mm length)
- ✅ Disk embedded in plastic piece
- ✅ Attached to motor shaft bottom
- ✅ Disk now visible (not buried)

### AS5600 Sensor Mounting
- ✅ Cushion sheet between sensor and motor base
- ✅ Target air gap: **0.2-0.5mm**
- ✅ Pin 22 (SDA) - Pin 24 (SCL)

### Pin Configuration
```
Motor Stepper:
- STEP: Pin 5
- DIR:  Pin 6
- EN:   Pin 7

Motor AS5600 Sensor (Software I2C):
- SDA: Pin 22
- SCL: Pin 24
- I2C Address: 0x36
```

## Testing Procedure

### 1. Upload and Initialize
```bash
platformio run --target upload
platformio device monitor
```

**Expected Output**:
```
[OK] Stepper motor configured
Testing I2C bus for device 0x36... found! ✓
Checking magnet field... OK ✓
Reading initial angle... XX.XX° (raw: XXXX)
```

### 2. Start with Magnet Diagnostics (Option 3)

**Purpose**: Verify magnetic field strength

**Expected Results**:
- ✅ **PERFECT**: `MD=YES, ML=NO, MH=NO` → Air gap optimal
- ⚠ **WEAK**: `MD=YES, ML=YES` → Reduce air gap (move sensor closer)
- ⚠ **STRONG**: `MD=YES, MH=YES` → Increase air gap (move sensor away)
- ❌ **ERROR**: `MD=NO` → Check alignment/installation

**Action**:
- If not PERFECT, adjust sensor-magnet distance
- Re-run diagnostics until PERFECT

### 3. Live Monitor Test (Option 1)

**Purpose**: Manual shaft rotation test

**Procedure**:
1. Motor will be disabled (safe to touch)
2. Rotate motor shaft by hand slowly
3. Watch readings update in real-time
4. Press 'S' to stop and see statistics

**Good Signs**:
- Smooth angle progression
- Small deltas (<10°)
- Few or no warnings (⚠)

**Bad Signs**:
- Large jumps (>120°)
- Invalid raw values
- Frequent warnings

**Statistics to Check**:
- Stability: >95% = EXCELLENT
- Jump rate: <2% = EXCELLENT
- 85-95% stability = GOOD (usable with filtering)
- <85% = POOR (use step counter instead)

### 4. Manual Jog Test (Keys A/D)

**Purpose**: Test sensor during actual motor movement

**Controls**:
- `A` = Jog left (1 step)
- `D` = Jog right (1 step)
- `Q/E` = Move 10 steps left/right
- `W/R` = Move 50 steps left/right
- `Z` = Return to zero

**Watch For**:
- Sensor angle changes match step direction
- Raw values consistent (~1.8° per step expected)
- No large jumps during movement

### 5. Stepped Motor Test (Option 2)

**Purpose**: Automated continuous movement test

**Behavior**:
- Motor steps every 500ms
- Reverses direction every 20 steps
- Shows angle delta per step
- Press 'S' to stop

**Expected**:
- Each step → ~1.8° change (or microstepping value)
- Consistent deltas (±0.5° variation)
- No cumulative drift

**If Drift Occurs**:
- Sensor readings drifting from step count
- Indicates magnetic interference or loose magnet
- May need better shielding or mechanical fix

## Decision Criteria

### ✅ INTEGRATE SENSOR (Good Results)
- Stability >90%
- Jump rate <5%
- Angle changes match motor steps
- Max delta <30° during operation

**Action**: Restore full control code with motor sensor enabled

### ❌ USE STEP COUNTER (Poor Results)
- Stability <85%
- Jump rate >10%
- Erratic readings during movement
- Sensor interfered by motor power

**Action**: Restore full control code with step-based position tracking

## Troubleshooting

### Sensor Not Found (I2C Error)
```
Testing I2C bus for device 0x36... error 2
```
**Check**:
- Pin 22 (SDA) connected to sensor SDA
- Pin 24 (SCL) connected to sensor SCL
- Sensor has 5V and GND
- Wires not loose

### Magnet Too Weak (ML bit set)
```
ML (Too Weak): YES ⚠
```
**Fix**:
- Remove cushion sheet or use thinner sheet
- Move sensor closer to magnet
- Target: 0.2mm air gap

### Magnet Too Strong (MH bit set)
```
MH (Too Strong): YES ⚠
```
**Fix**:
- Add thicker cushion sheet
- Move sensor away from magnet
- Target: 0.5mm air gap

### Large Jumps During Motor Movement
**Possible Causes**:
1. Motor power noise → Add/improve capacitors
2. Loose magnet → Check mechanical attachment
3. Misalignment → Ensure magnet centered on sensor

**Test**:
- Run Live Monitor (motor off) - should be stable
- Run Stepped Motor Test (motor on) - if unstable, it's EMI

## Quick Commands Summary

```
Main Menu:
1 - Live Monitor (manual rotation test)
2 - Stepped Motor Test (automated movement)
3 - Magnet Field Diagnostics
4 - Show Statistics
5 - Reset Statistics

Manual Control:
A/D - Jog 1 step left/right
Q/E - Move 10 steps left/right
W/R - Move 50 steps left/right
Z   - Return to zero
Space - Single reading

All Tests:
S - Stop current test
```

## After Testing

### If Sensor Works Well
```bash
# Restore full control code
cd /Users/piyush/code/Rotary-Inverted-Pendulum/src
cp main_full_control.cpp.bak main.cpp

# Enable motor sensor in code
# Add motor sensor readings to control loop
# Test swing-up and balance with dual sensors
```

### If Sensor Doesn't Work
```bash
# Keep step-based tracking
cd /Users/piyush/code/Rotary-Inverted-Pendulum/src
cp main_full_control.cpp.bak main.cpp

# main.cpp already uses step counter
# No changes needed - proceed with control testing
```

## Files Reference

```
src/
├── main.cpp                          ← CURRENT: Motor sensor test
├── main_full_control.cpp.bak        ← BACKUP: Full control system
├── main_angle_calibration.cpp.bak
├── main_full_pid_control.cpp.bak
├── main_step_calibration.cpp.bak
└── ... other backups

Log.md                               ← Updated with troubleshooting notes
MOTOR_SENSOR_TEST_GUIDE.md          ← This file
```

---

**Good luck with testing! 🎯**

The hardware modifications sound promising - custom plastic housing and precise air gap control should give much better results than the buried disk setup.
