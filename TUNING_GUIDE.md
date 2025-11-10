# Rotary Inverted Pendulum - Tuning Guide

## Quick Start After Upload

### 1. System Diagnostics (Option 1)
Verify all hardware is working:
- Motor driver enabled
- Pendulum sensor responding
- Motor sensor responding
- Motor movement test passes

### 2. Zero Position (Option 2)
- Position motor arm at CENTER
- Hold pendulum STRAIGHT UP
- Press 'Z' to set zero

### 3. Calibrate Limits (Option 4)
- Use A/D keys to jog left/right
- **Important**: Stop BEFORE pendulum hits table!
- Press 'L' at safe left limit
- Press 'R' at safe right limit
- Press 'Q' to finish

### 4. Test Balance FIRST (Option B - NEW!)
**DO THIS BEFORE SWING-UP!**

Start with current conservative gains:
- `Kp_balance = 5.0`
- `Kd_balance = 1.0`
- `Ki_balance = 0.1`
- `balanceScale = 2.0`

**Test procedure:**
1. Press 'B' for balance test
2. Manually hold pendulum at ~10-15° from upright
3. Release gently
4. Observe response

**Good signs:**
- ✅ Motor moves smoothly toward pendulum
- ✅ Oscillations dampen quickly
- ✅ Settles near upright without hitting limits

**Bad signs:**
- ❌ Motor slams into limits immediately → Reduce `Kp_balance` (try 3.0)
- ❌ Oscillates wildly → Reduce all gains by 50%
- ❌ Pendulum falls immediately → Increase `Kp_balance` (try 7.0)
- ❌ Slow, sluggish response → Increase `Kd_balance` (try 2.0)

### 5. Test Swing-Up (Option S)
**Only after balance works!**

Current setting: `Kp_swing = 150` (steps)

**Test procedure:**
1. Press 'S' to start swing-up
2. Pull pendulum down to ~45-60°
3. Release and watch

**Expected behavior:**
- Motor should move back and forth
- Pendulum amplitude should GROW with each swing
- When near upright AND slow, switches to balance

**Tuning:**
- Too slow / no growth → Increase `Kp_swing` (try 200, 250, 300)
- Too violent / hits limits → Decrease `Kp_swing` (try 100)
- Switches too early → Reduce `balanceThreshold` (try 20°)
- Never switches → Increase `balanceThreshold` (try 30°)

---

## Motor Position Drift Issue

**Symptom**: After moving left/right, "Return to Center" (Option 5) is ~10° off

**Possible causes:**
1. **Missed steps** - Motor can't keep up with commanded speed
2. **Sensor noise** - AS5600 readings jumping during movement
3. **Control using steps, not sensor** - System trusts step counter, not sensor feedback

**Fixes to try:**

### Fix 1: Add Sensor Feedback to Return-to-Center
Currently uses step position only. Modify `returnToCenter()`:

```cpp
void returnToCenter() {
  Serial.println("\n╔════════════════════════════════════════════╗");
  Serial.println("║      RETURNING TO CENTER                   ║");
  Serial.println("╚════════════════════════════════════════════╝\n");
  
  stepper.setMaxSpeed(MOTOR_SPEED);
  stepper.setAcceleration(MOTOR_ACCEL);
  
  // First, move to step-based zero
  stepper.moveTo(stepsAtZero);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
  
  // Then, correct using sensor feedback
  updateSensors();
  Serial.print("Step-based center. Sensor says: ");
  Serial.print(motorAngle, 2);
  Serial.println("°");
  
  if (abs(motorAngle) > 2.0) {  // If off by more than 2°
    Serial.print("Correcting ");
    Serial.print(motorAngle, 1);
    Serial.println("° offset...");
    
    // Small correction moves
    while (abs(motorAngle) > 1.0) {
      updateSensors();
      long correction = -(long)(motorAngle * 10);  // proportional correction
      stepper.move(correction);
      while (stepper.distanceToGo() != 0) {
        stepper.run();
      }
      delay(100);
    }
  }
  
  updateSensors();
  Serial.print("✓ At center | Motor angle: ");
  Serial.print(motorAngle, 2);
  Serial.println("°");
  
  // Update step zero to match sensor
  stepsAtZero = stepper.currentPosition();
  Serial.println("════════════════════════════════════════════\n");
}
```

### Fix 2: Add Median Filtering to Motor Sensor
Reduce noise in `readAS5600Angle()`:

```cpp
uint16_t readAS5600AngleMedian(SoftwareWire &wire) {
  uint16_t r1 = readAS5600Angle(wire);
  delayMicroseconds(100);
  uint16_t r2 = readAS5600Angle(wire);
  delayMicroseconds(100);
  uint16_t r3 = readAS5600Angle(wire);
  
  // Sort and return middle value
  if (r1 > r2) { uint16_t t = r1; r1 = r2; r2 = t; }
  if (r2 > r3) { uint16_t t = r2; r2 = r3; r3 = t; }
  if (r1 > r2) { uint16_t t = r1; r1 = r2; r2 = t; }
  return r2;
}
```

Then use this in `updateSensors()` instead of `readAS5600Angle()`.

### Fix 3: Reduce Speed During Calibration
Motor might miss steps at high speed. In `calibrateLimits()`:

```cpp
stepper.setMaxSpeed(JOG_SPEED);  // Already doing this - good!
```

Make sure `JOG_SPEED` is reasonable (currently 2000 - should be fine).

---

## Gain Tuning Reference

### Balance Mode PID
```cpp
float Kp_balance = 5.0;   // Proportional: reaction strength
float Kd_balance = 1.0;   // Derivative: damping
float Ki_balance = 0.1;   // Integral: eliminate steady-state error
float balanceScale = 2.0; // Steps per control unit
```

**Tuning process:**
1. Start with `Ki = 0` (disable integral)
2. Increase `Kp` until system oscillates
3. Back off `Kp` by 30-50%
4. Increase `Kd` to dampen oscillations
5. Add small `Ki` (0.1-0.5) to fix steady-state offset

### Swing-Up Bang-Bang
```cpp
float Kp_swing = 150.0;  // Step size per swing
```

**Tuning:**
- Start low (100-150)
- Increase until pendulum amplitude grows noticeably
- Too high → slams limits
- Too low → no energy pumping, stalls at ~45°

### Mode Transition
```cpp
float balanceThreshold = 25.0;  // Angle to switch (degrees)
// Velocity check: abs(alpha_dot) < 2.0
```

**Tuning:**
- `balanceThreshold` too high → tries to balance from far angles, fails
- `balanceThreshold` too low → never reaches it during swing-up
- Velocity check prevents switching while flying through upright

---

## Emergency Procedures

### If Motor Slams Limits Repeatedly
1. Press 'X' to stop immediately
2. Press 'E' for emergency stop (disables motor)
3. Reduce all balance gains by 50%
4. Reduce `balanceScale` (try 1.0)

### If Pendulum Falls During Balance
1. Check signs: pendulum lean direction vs motor movement
2. May need to negate control: `-totalControl` instead of `+totalControl`
3. Or negate motor correction: `+0.5 * motorAngle` instead of `-0.5 * motorAngle`

### If Swing-Up Never Builds Energy
1. Increase `Kp_swing` gradually (200, 300, 400...)
2. Check limit range - may be too narrow
3. Try starting from different initial angle (try 90° down)
4. Watch motor movement pattern - should alternate left/right with pendulum

---

## Expected Performance

**Good balance control:**
- Settles within ±5° of upright
- Minimal oscillation (1-2 cycles to settle)
- Motor stays within ±30° from center
- Can recover from 15-20° disturbance

**Good swing-up:**
- Amplitude grows ~10-20° per cycle
- Reaches upright in 3-5 swings
- Smooth transition to balance (no slam)
- Motor uses full range but doesn't hit limits

**Current limitations:**
- Limited travel (~70-80° each side) - constrains swing-up aggression
- No feedforward compensation - purely reactive
- Step-based positioning - can accumulate error
- Simple bang-bang swing-up - not energy-optimal

---

## Next Improvements (Future)

1. **Add sensor-based position correction** (drift fix)
2. **Implement proper energy-based swing-up** with target energy
3. **Add Kalman filter** for smoother sensor readings
4. **Tune LQR gains** properly (currently just PID)
5. **Add disturbance rejection** tests
6. **Measure and log balance duration** statistics
