# Rotary Inverted Pendulum - Pin Configuration

**Last Updated:** November 4, 2025

---

## Quick Reference

| Component | Arduino Pins | Notes |
|-----------|--------------|-------|
| **Stepper Motor** | 5 (STEP), 6 (DIR), 7 (EN) | TMC2209 driver |
| **Pendulum Sensor** | 20 (SDA), 21 (SCL) | Hardware I2C, 5kΩ pull-ups |
| **Motor Sensor** | 22 (SDA), 24 (SCL) | Software I2C, 5kΩ pull-ups |

---

## Detailed Pin Assignments

### 1. Stepper Motor Driver (TMC2209)

#### Control Signals

| Arduino Pin | TMC2209 Pin | Function | Logic Level |
|-------------|-------------|----------|-------------|
| 5 | STEP | Step pulse | HIGH = one step |
| 6 | DIR | Direction | LOW = CW, HIGH = CCW |
| 7 | EN | Enable | LOW = enabled, HIGH = disabled |

#### Power Connections

| Source | TMC2209 Pin | Voltage | Notes |
|--------|-------------|---------|-------|
| Motor PSU (+) | VMOT | 12-24V | Motor power |
| Motor PSU (-) | GND (motor side) | 0V | Motor ground |
| Arduino 5V | VIO | 5V | Logic power |
| Arduino GND | GND (logic side) | 0V | Logic ground |

**Required Capacitors:**
- 100µF electrolytic: VMOT to GND (+ to VMOT, - to GND)
- 0.1µF ceramic: VMOT to GND

#### Motor Wiring

| TMC2209 | Stepper Motor | Wire Color (typical) |
|---------|---------------|----------------------|
| A1 | Coil A+ | Red |
| A2 | Coil A- | Blue |
| B1 | Coil B+ | Green |
| B2 | Coil B- | Black |

---

### 2. Pendulum Sensor (AS5600 #1)

**I2C Bus:** Hardware I2C (Primary)

| AS5600 Pin | Arduino Pin | Connection |
|------------|-------------|------------|
| VCC | 5V | Power |
| GND | GND | Ground |
| SDA | 20 | I2C Data + 5kΩ pull-up to 5V |
| SCL | 21 | I2C Clock + 5kΩ pull-up to 5V |

**Required Components:**
- 5kΩ resistor: Pin 20 to 5V
- 5kΩ resistor: Pin 21 to 5V
- 0.1µF ceramic capacitor: VCC to GND (at sensor)

**Configuration:**
- I2C Address: 0x36 (fixed)
- Clock Speed: 400kHz (fast mode)
- Magnet Distance: 2-3mm from chip

---

### 3. Motor Sensor (AS5600 #2)

**I2C Bus:** Software I2C (Secondary)

| AS5600 Pin | Arduino Pin | Connection |
|------------|-------------|------------|
| VCC | 5V | Power |
| GND | GND | Ground |
| SDA | 22 | I2C Data + 5kΩ pull-up to 5V |
| SCL | 24 | I2C Clock + 5kΩ pull-up to 5V |

**Required Components:**
- 5kΩ resistor: Pin 22 to 5V
- 5kΩ resistor: Pin 24 to 5V
- 0.1µF ceramic capacitor: VCC to GND (at sensor)

**Configuration:**
- I2C Address: 0x36 (fixed)
- Clock Speed: 100kHz (standard mode)
- Magnet Distance: 2-3mm from chip

---

## Power Supply Architecture

```
External 12-24V PSU
    |
    ├─→ TMC2209 VMOT (motor power)
    |    └─→ [100µF + 0.1µF capacitors]
    |
    └─→ GND ──────┬─→ TMC2209 GND
                  └─→ Arduino GND (COMMON GROUND)

Arduino USB/VIN
    |
    └─→ Arduino 5V ─┬─→ [10µF + 0.1µF capacitors]
                    ├─→ TMC2209 VIO
                    ├─→ AS5600 #1 VCC
                    └─→ AS5600 #2 VCC
```

**Critical:** All components must share a common ground!

---

## Hardware Modifications

### ✅ Completed (Nov 4, 2025)

**I2C Pull-up Resistors:**
- Pin 20 (SDA) → [5kΩ] → 5V
- Pin 21 (SCL) → [5kΩ] → 5V
- Pin 22 (SDA) → [5kΩ] → 5V
- Pin 24 (SCL) → [5kΩ] → 5V

**Result:** Sensors stable when motor power off (96% stable readings)

### ⚠️ Pending (Nov 5, 2025)

**Decoupling Capacitors:**

1. **TMC2209 Motor Power:**
   - 100µF electrolytic between VMOT and GND (+ to VMOT)
   - 0.1µF ceramic between VMOT and GND
   - Place close to driver pins

2. **Arduino 5V Rail:**
   - 10µF electrolytic between 5V and GND (+ to 5V)
   - 0.1µF ceramic between 5V and GND

3. **AS5600 Sensors (both):**
   - 0.1µF ceramic between VCC and GND (at each sensor)

**Purpose:** Eliminate power supply noise causing sensor instability during motor operation

---

## Calibration Reference

**Zero Position:**
- Pendulum: Upright
- Motor arm: Center
- Stepper position: 0 steps

**Movement Limits:**
- Left: -280 steps (CW)
- Right: +280 steps (CCW)
- Range: 560 steps = 1008° = 2.8 revolutions

**Motor Characteristics:**
- 200 steps/revolution (1.8° per step)
- Speed: 1000 steps/sec
- Acceleration: 2000 steps/sec²

---

## Troubleshooting

### Sensor Not Detected

**Hardware I2C (Pendulum):**
1. Check connections: Pin 20 (SDA), Pin 21 (SCL)
2. Verify 5kΩ pull-up resistors installed
3. Magnet distance: 2-3mm from chip
4. Run I2C scanner (option 0 in menu)

**Software I2C (Motor):**
1. Check connections: Pin 22 (SDA), Pin 24 (SCL)
2. Verify 5kΩ pull-up resistors installed
3. Magnet distance: 2-3mm from chip
4. Check SoftwareWire library installed

### Motor Not Moving

1. Check enable pin: LOW = enabled
2. Verify STEP/DIR/EN connections (pins 5, 6, 7)
3. Check motor power supply (12-24V to VMOT)
4. Test with manual step function (option 7/8)

### Noisy Sensor Readings

**Without motor power:**
- Should be stable (±2° max)
- If not: Check pull-up resistors, magnet distance

**With motor power:**
- Install decoupling capacitors (see pending section)
- Expected: <10% error rate after capacitors

---

## Wiring Checklist

- [ ] Stepper motor connected to TMC2209 (A1, A2, B1, B2)
- [ ] Motor power 12-24V to TMC2209 VMOT
- [ ] Arduino 5V to TMC2209 VIO
- [ ] Common ground: Motor PSU, TMC2209, Arduino
- [ ] Control pins: 5 (STEP), 6 (DIR), 7 (EN)
- [ ] Pendulum sensor: 20 (SDA), 21 (SCL), 5V, GND
- [ ] Motor sensor: 22 (SDA), 24 (SCL), 5V, GND
- [ ] Pull-up resistors: 4x 5kΩ installed ✓
- [ ] Capacitors: Pending installation
- [ ] Magnet distance: 2-3mm from both AS5600 chips

---

**Status:** Partial - awaiting capacitor installation
**Next:** Add decoupling capacitors (Nov 5, 2025)
