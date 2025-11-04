# Backup Scripts Reference

This document lists all backup scripts and their purposes.

## Available Scripts

### 1. `main_full_pid_control.cpp.bak`
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
