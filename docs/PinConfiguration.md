# Rotary Inverted Pendulum - Pin Configuration

**Last Updated:** November 4, 2025

---

## Quick Reference

| Component | Arduino Pins | Notes |
|-----------|--------------|-------|
| **Stepper Motor** | 11 (STEP), 6 (DIR), 7 (EN) | TMC2209 driver |
| **Pendulum Sensor** | 20 (SDA), 21 (SCL) | Hardware I2C, 5kΩ pull-ups |
| **Motor Sensor** | A15 (OUT) | PWM mode, DIR to 5V |

---

## Detailed Pin Assignments

### 1. Stepper Motor Driver (TMC2209)

#### Control Signals

| Arduino Pin | TMC2209 Pin | Function | Logic Level |
|-------------|-------------|----------|-------------|
| 11 | STEP | Step pulse | HIGH = one step |
| 6 | DIR | Direction | LOW = CW, HIGH = CCW |
| 7 | EN | Enable | LOW = enabled, HIGH = disabled |

#### Power Connections

| Source | TMC2209 Pin | Voltage | Notes |
|--------|-------------|---------|-------|
| Motor PSU (+) | VMOT | 24V | Motor power (upgraded Nov 10, 2025) |
| Motor PSU (-) | GND (motor side) | 0V | Motor ground |
| Arduino 5V | VIO | 5V | Logic power |
| Arduino GND | GND (logic side) | 0V | Logic ground |

**Required Capacitors:**
- 100µF electrolytic: VMOT to GND (+ to VMOT, - to GND)
- 0.1µF ceramic: VMOT to GND

#### Motor Wiring

| TMC2209 | Stepper Motor | Wire Color (typical) |
|---------|---------------|----------------------|
| A1 | Coil A- | Blue |
| A2 | Coil A+ | Red |
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

## 4. Optional: I2C Multiplexer (PCA9548A)

If you connect multiple I2C devices with the same address or you need to isolate noisy peripherals, a PCA9548A I2C multiplexer can be used. In this project we use a PCA9548A board to separate the pendulum and motor AS5600 sensors onto dedicated channels.

Hardware connections (multiplexer side):

| Multiplexer Pin | Arduino Pin | Notes |
|------------------|-------------|-------|
| VIN | 5V | Power (Arduino 5V) |
| GND | GND | Ground |
| SDA | 20 (SDA) | Hardware I2C SDA on Mega |
| SCL | 21 (SCL) | Hardware I2C SCL on Mega |

Multiplexer channel wiring (from PCA9548A):

| PCA9548A Channel | Device | Device SDA -> | Device SCL -> |
|-------------------|--------|----------------|----------------|
| SD0 / SC0 | Pendulum AS5600 | SD0 -> AS5600 SDA | SC0 -> AS5600 SCL |
| SD1 / SC1 | Motor Base AS5600 | SD1 -> AS5600 SDA | SC1 -> AS5600 SCL |

I²C Address Pins

- Leave A0, A1, A2 floating (default address 0x70). If you change them, update the code using the address you set.

Notes and Best Practices:

- The multiplexer is controlled via the hardware I2C bus (pins 20/21 on Mega). When a channel is selected, the device(s) on that channel appear on the shared I2C bus.
- Keep the I2C lines short and twisted where possible to reduce EMI. The multiplexer helps with noisy sensors but does not solve signal wiring EMI.
- Pull-up resistors should remain on the main bus as required. The devices on each channel should not reintroduce conflicting pull-ups across channels.
By default this repo uses the manual `Wire` channel selection helper included in sample code above (`tcaSelect()`), but you can optionally use a third-party library like `WifWaf/TCA9548A` if you prefer a library API.

### Sample: Using PCA9548A & AS5600 in `src/main.cpp`

Below is a minimal example showing how to initialize the multiplexer and read from the sensors on each channel using the Adafruit TCA driver and `AS5600` library. In the firmware, select the channel before initializing or reading the sensor.

```cpp
#include <Wire.h>
#include <TCA9548A.h> // or use Wire directly to select channels
#include <AS5600.h>

TCA9548A tca;
AS5600 pendulum(&Wire);
AS5600 motorSensor(&Wire);

void setup(){
   Serial.begin(115200);
   Wire.begin();
   if (!tca.begin()) {
      Serial.println("TCA9548A not found. Check I2C connections.");
      while(1);
   }

   // Initialize pendulum (chan 0) - library call or tca.select(0)
   tca.select(0);
   pendulum.begin();

   // Initialize motor sensor (chan 1) - library call or tca.select(1)
   tca.select(1);
   motorSensor.begin();

   // Return to channel 0 for normal operation
   tca.select(0);
}

void loop(){
   // Read pendulum on channel 0
   tca.select(0);
   uint16_t pRaw = pendulum.readAngle();

   // Read motor on channel 1
   tca.select(1);
   uint16_t mRaw = motorSensor.readAngle();

   Serial.print("Pend: "); Serial.print(pRaw);
   Serial.print(" | Motor: "); Serial.println(mRaw);
   delay(200);
}
```

Alternative: Manual channel selection using Wire (no extra library):

```cpp
void tcaSelect(uint8_t ch) {
   if (ch > 7) return;
   Wire.beginTransmission(0x70);
   Wire.write(1 << ch);
   Wire.endTransmission();
}

// Then use tcaSelect(0) / tcaSelect(1) before reading sensors
```


---

### 3. Motor Sensor (AS5600 #2)

**Mode:** PWM Output (Updated Nov 20, 2025)

| AS5600 Pin | Arduino Pin | Connection |
|------------|-------------|------------|
| VCC | 5V | Power |
| GND | GND | Ground |
| DIR | 5V | Enable PWM mode |
| OUT | A15 | PWM signal input |

**Required Components:**
- 0.1µF ceramic capacitor: VCC to GND (at sensor)

**Configuration:**
- PWM Period: 1.024ms
- Duty Cycle: 0-100% (0-360°)
- Magnet Distance: 2-3mm from chip

---

## Power Supply Architecture

```
External 24V, 2A PSU (Upgraded Nov 10, 2025)
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
- Speed: 12000 steps/sec (max)
- Acceleration: 30000 steps/sec²
- TMC2209 Vref: 2.112V (~2.4A motor current)
- Power supply: 24V, 2A adapter

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
3. Check motor power supply (24V to VMOT)
4. Verify Vref setting: 2.112V for proper torque
5. Test with manual step function (calibration menu)

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
- [ ] Motor power 24V, 2A adapter to TMC2209 VMOT
- [ ] TMC2209 Vref adjusted to 2.112V (⚠️ Important for torque!)
- [ ] Arduino 5V to TMC2209 VIO
- [ ] Common ground: Motor PSU, TMC2209, Arduino
- [ ] Control pins: 11 (STEP), 6 (DIR), 7 (EN)
- [ ] Pendulum sensor: 20 (SDA), 21 (SCL), 5V, GND
- [ ] Motor sensor: 22 (SDA), 24 (SCL), 5V, GND
- [ ] Pull-up resistors: 4x 5kΩ installed ✓
- [ ] Capacitors: Pending installation
- [ ] Magnet distance: 2-3mm from both AS5600 chips

---

**Status:** Partial - awaiting capacitor installation
**Next:** Add decoupling capacitors (Nov 5, 2025)
