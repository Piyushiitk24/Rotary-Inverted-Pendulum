# EMI Analysis and Mitigation - Nov 7, 2025

## Problem Summary

Your calibration test (Options 3, 4, 5) revealed **severe EMI-induced bit-flip corruption** on both AS5600 I2C sensor lines during motor operation.

## Symptoms Observed

### Pendulum Sensor (Hardware I2C - Pins 20/21)
- **180° jumps every 2-5 steps** during motor movement
- Readings flip between two clusters: `-35°` to `-47°` and `+150°` to `+180°`
- Pattern: `-36° → +170° → -36° → +160° → -36°...`
- **Root cause**: MSB (bit 11) of 12-bit angle value flipping due to EMI

### Motor Sensor (Software I2C - Pins 22/24)
- **15-30° jumps** during movement (less severe but still problematic)
- Readings toggle between `~150°` and `~205°` zones
- Non-physical behavior: angle doesn't sweep smoothly as motor jogs

### What Works Perfectly
- ✅ Motor stepping: Clean progression from -205 to +205 steps
- ✅ Step counter: No missed or extra steps
- ✅ Limit storage: ±205 steps correctly saved
- ✅ Capacitors: Successfully filtered power supply noise (72% stable when stationary)

## Root Cause Analysis

### 1. Bit-Flip Mechanism
AS5600 outputs 12-bit raw angle (0-4095 counts = 0-360°):
```
Bit 11 (MSB) = 2048 counts = 180° contribution
Bit 10       = 1024 counts = 90° contribution
```

**When EMI flips bit 11**: Reading jumps exactly ±180°

**Example from your log**:
```
Step -1: Pend: -35.9° (bit 11 = 0, stable)
Step -2: Pend: 170.9° (bit 11 flipped to 1, +206.8° error)
Step -3: Pend: -36.0° (bit 11 back to 0, corrected)
```

### 2. EMI Source - CRITICAL DISTINCTION

**NOT the motor's magnetic field** (which AS5600 is designed to measure near motors)

**The real culprit: TMC2209 driver high-frequency switching**

Every motor step generates:
- **Sharp current spike** in TMC2209 driver (~1-2A risetime <1µs)
- **High-frequency EM radiation** (100kHz-10MHz range) from fast dI/dt
- **Long breadboard wires** (10-30cm) act as antennas, picking up this radiation
- **I2C digital communication corrupted** - bit flips during transmission

**Key insight**:
- Motor's DC magnetic field (0-100Hz): AS5600 measures this perfectly ✅
- Driver's switching noise (100kHz-10MHz): Corrupts I2C digital lines ❌
- These are **completely different phenomena**

### 3. Why This Works in Production Systems

**PCB-based designs** (SimpleFOC, ODrive, commercial servos):
- AS5600 placed **directly on PCB** near motor driver
- I2C traces <2cm long (vs your 10-30cm breadboard wires)
- Ground plane shields from EM radiation
- 0.1µF ceramic caps at each chip filter high-frequency noise
- Result: <1% error rate despite motors right next to sensors

**Your breadboard challenge**:
- Long jumper wires = excellent antennas for 100kHz-10MHz EMI
- No ground plane shielding
- Distributed capacitance and inductance in wires
- This is why breadboard testing is hard, not because of fundamental design flaw

### 3. Why 400kHz I2C Failed
- Long breadboard wires act as antennas
- 400kHz edges are ~2.5ns fast → sensitive to even small EMI pulses
- Pull-ups can't overcome coupled noise at that speed

### 4. Why Both Sensors Failed
- Hardware I2C (400kHz): High speed + long leads = vulnerable
- Software I2C (100kHz): Slower but routed near motor phases = equal vulnerability

## Software Fixes Implemented

### 1. Median Filtering (3-sample)
**Function**: `readPendulumAngleMedian()`, `readAS5600AngleMedian()`
- Takes 3 readings, 100µs apart
- Returns middle value (rejects single-bit flip outliers)
- **Effectiveness**: Rejects isolated EMI pulses

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

### 2. Change-Rate Limiting
**Function**: `updateSensorReadings()` - Modified
- Tracks previous reading
- Rejects jumps >90° between updates
- Holds last good value when corruption detected
- **Effectiveness**: Catches bit flips that survive median filter

```cpp
if (abs(delta) > 90.0) {
  pendulum_deg = lastPendulumRaw;  // Use previous good value
  Serial.println("[WARN] Pendulum jump rejected");
}
```

### 3. Magnet Field Validation
**Function**: `checkPendulumMagnet()`
- Reads AS5600 status register
- Checks MD (magnet detected), ML (too weak), MH (too strong) bits
- Warns if field out of range (2-3mm distance required)
- **Effectiveness**: Catches mechanical misalignment issues

### 4. I2C Clock Reduction
**Changed**: 400kHz → 100kHz on Hardware I2C
- Slower edges = more noise immunity
- Longer bit times = higher signal-to-noise ratio
- **Trade-off**: 4x slower sensor reads (still fast enough for control)

### 5. Raw Diagnostic Mode
**New Menu Option 'R'**:
- Shows single reads vs median-filtered reads side-by-side
- Displays raw 12-bit values to see bit patterns
- Highlights differences when EMI detected
- **Usage**: Press 'R', watch for ⚠ warnings showing corrupted single reads

## Hardware Fixes Required

### Immediate Actions (High Priority)

#### 1. Twist I2C Wires
**Why**: Twisted pairs reject common-mode EMI through field cancellation
**How**:
- Twist SDA+SCL together for pendulum (pins 20/21)
- Twist SDA+SCL together for motor (pins 22/24)
- Use ~1 twist per inch
- **Expected improvement**: 50-80% reduction in EMI pickup

#### 2. Add Ferrite Beads
**Why**: Blocks high-frequency EMI while passing DC/low-freq signals
**Where**: On each I2C line near sensors
**Part**: 100Ω@100MHz ferrite bead (e.g., Murata BLM18AG121SN1D)
**Expected improvement**: 60-90% reduction in high-frequency coupling

#### 3. Separate Motor Power Ground
**Why**: Motor current spikes create ground bounce
**How**:
- Run separate ground wire from motor driver to Arduino GND
- Keep motor ground away from sensor grounds until single connection point at Arduino
- **Expected improvement**: Eliminates conducted noise on ground rail

### Medium Priority

#### 4. Shielded Cable for I2C
**Why**: Complete EM shielding
**How**: Use CAT5/6 ethernet cable
- Use one twisted pair for SDA+SCL
- Ground shield at Arduino end only (avoid ground loops)
- **Expected improvement**: 90-99% EMI rejection

#### 5. Physical Separation
**Current**: Sensors on breadboard near motor driver
**Recommended**: Move sensors ≥10cm away from driver/motor
**Expected improvement**: 30-50% reduction in magnetic coupling

### Low Priority (Optimization)

#### 6. Add 100Ω Series Resistors
**Where**: On SDA/SCL lines at Arduino pins
**Why**: Limits slew rate, reduces high-frequency emissions
**Trade-off**: Slightly slower I2C (still within spec at 100kHz)

#### 7. Shorter Wires
**Current**: Likely 10-20cm breadboard jumpers
**Recommended**: Cut to minimum needed length
**Why**: Shorter wires = smaller antenna area

## Testing Strategy

### Phase 1: Software-Only (Current)
1. Upload code with median filtering + change-rate limiting
2. Run **Option R** (Raw Diagnostic Mode)
   - Watch for ⚠ warnings showing rejected jumps
   - Compare single vs median reads
3. Run **Options 3/4** (Set Limits) again
   - Look for `[WARN] Pendulum jump rejected` messages
   - Check if filtered readings stay stable

**Expected**: Jumps still occur but get rejected → readings appear more stable

### Phase 2: Twist Wires
1. Twist I2C wires (easiest hardware fix)
2. Run **Option R** again
3. Count reduction in ⚠ warnings

**Expected**: 50-80% fewer EMI events detected

### Phase 3: Ferrite Beads
1. Add beads on all 4 I2C lines
2. Run full calibration (Options 1-5)
3. Check if sensor readings smooth during movement

**Expected**: <5% error rate, system usable for control

### Phase 4: Full Mitigation
1. Implement all hardware fixes
2. Run continuous jogging test (1000+ steps)
3. Measure error rate

**Success Criteria**: <1% corrupted readings, smooth angle tracking

## Raw Diagnostic Mode Usage

### How to Use
1. Upload new code
2. Open serial monitor (115200 baud)
3. Press **'R'** in main menu
4. Manually move pendulum and motor arm
5. Watch output:

```
───────────────────────────────────────────────────────────────
Pend_RAW  Pend_Filt  Motor_RAW  Motor_Filt  Pend(°)  Motor(°)
───────────────────────────────────────────────────────────────
  1234      1234       2048       2048       108.3°   180.0°
⚠ 3282      1234       2045       2048       108.3°   179.9°   ← BIT FLIP!
  1236      1236       2050       2050       108.4°   180.1°
```

**⚠ symbol** = Single read corrupted, median filter caught it

### What to Look For
- **Frequency of ⚠**: How often EMI strikes
- **Pattern**: Random or synchronized with motor position?
- **Magnitude**: How different are RAW vs Filtered values?
- **Success**: Filtered values should stay smooth even when RAW jumps

## Expected Results

### Before Hardware Fixes
- 20-40% of readings show ⚠ warnings
- Filtered angles stay stable (software protection working)
- System usable but not ideal

### After Twisted Wires
- 5-10% of readings show ⚠ warnings
- Noticeable improvement in smoothness

### After Ferrite Beads
- <2% of readings show ⚠ warnings
- Ready for control algorithm testing

### After Full Mitigation
- <0.5% of readings show ⚠ warnings
- Production-quality signal integrity

## Your Analysis Was Correct

All your observations were accurate:
1. ✅ Step counter behaves perfectly
2. ✅ 180° quantization = MSB corruption (bit 11 flip)
3. ✅ Both sensors failing independently = signal integrity problem
4. ✅ Hardware I2C at 400kHz too fast for noisy environment
5. ✅ `detectAS5600Magnet()` never called = no runtime validation
6. ✅ Single-read trust = no redundancy against bit flips

You correctly identified:
- EMI from motor driver as root cause
- Bit-level corruption mechanism
- Need for signal integrity improvements
- Inadequate error checking in current code

## Next Steps

### Immediate (Test First)
1. **Add 0.1µF ceramic capacitors** at both AS5600 sensor chips
   - Most effective single fix (50-70% error reduction expected)
   - Place capacitor leads directly at sensor VCC/GND pins
   - Keep leads <5mm long
   
2. **Run live sensor test** (Option 2)
   - Current status: 20-25% corruption rate
   - Target: <5% after capacitor addition

### If Still >5% Errors (Hardware Escalation)

3. **Shielded cable for I2C lines**
   - Replace breadboard jumpers with shielded twisted-pair cable
   - Ground shield at Arduino end only (avoid ground loops)
   - Expected: 60-80% error reduction
   
4. **100Ω series resistors**
   - Add on all 4 I2C lines (both sensors, SDA+SCL)
   - Slows down edges, reduces high-frequency radiation
   - Trade-off: Still within 100kHz I2C spec

### Engineering Discussion - NOT A PARADOX

**Question**: "How do engineers place encoders near motors when motor drivers cause EMI?"

**Answer**: They do it all the time successfully! The key differences:

**PCB-based production systems** (SimpleFOC, ODrive, commercial servos):
- ✅ AS5600 placed ON-BOARD with motor driver (same PCB)
- ✅ I2C traces: 1-2cm long (vs your 10-30cm breadboard wires)
- ✅ Ground plane acts as shield between driver and sensor
- ✅ 0.1µF ceramic caps at every IC (high-freq filtering)
- ✅ Twisted-pair or differential I2C routing
- ✅ Result: <1% error rate despite intense EMI environment

**Your breadboard prototype** (inherently challenging):
- ❌ Long jumper wires = excellent antennas for 100kHz-10MHz
- ❌ No ground plane shielding
- ❌ Distributed parasitic capacitance/inductance
- ❌ This is a **prototyping limitation**, not a design flaw

**Why your custom AS5600 mount is CORRECT**:
- Fixed magnet distance = perfect
- Proximity to motor shaft = necessary and fine
- Problem is purely the **communication wires**, not the sensing

**Real-world examples**:
1. **SimpleFOC library**: Thousands of projects using AS5600/AS5048 next to BLDC/stepper drivers
2. **ODrive controller**: AS5047 SPI encoder on same board as 40A motor driver
3. **Industrial servo motors**: Encoders **inside motor housing** with shielded cables out

The sensors work fine near motors. The challenge is getting clean digital data out via long wires.

### Migration Path to Production

When you move from breadboard to final design:

**Option A - PCB design**:
- AS5600 sensors directly on main PCB
- I2C traces <2cm with ground plane
- All EMI issues disappear

**Option B - Remote sensors** (if PCB not possible):
- Keep AS5600 near motor (your current mount is good)
- Use shielded twisted-pair cable for I2C
- Or switch to differential encoder (e.g., AS5047 with SPI)

## Next Steps

1. **Test software fixes first** (already implemented)
   - Upload and run Option R
   - Check if median filter catches most errors
   
2. **Twist I2C wires** (easiest hardware fix)
   - Takes 2 minutes
   - Big improvement expected
   
3. **Order ferrite beads** if needed
   - Only if twisted wires insufficient
   
4. **Report back** with:
   - How many ⚠ warnings per 100 readings in Option R?
   - Do filtered angles stay stable during Options 3/4?
   - Any improvement after twisting wires?

## Files Modified

- `src/main.cpp`:
  - Added median filtering functions
  - Added change-rate limiting
  - Added magnet field validation
  - Reduced I2C clock to 100kHz
  - Added raw diagnostic mode (Option R)

## References

- AS5600 Datasheet: Status register bits (MD, ML, MH) on page 18
- I2C Specification: Rise time requirements vs capacitance
- EMI Coupling: Common-mode rejection in twisted pairs
- Ferrite Bead Selection: Impedance vs frequency curves
