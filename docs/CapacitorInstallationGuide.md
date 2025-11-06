# Capacitor Installation Guide for Beginners

**Date:** November 5, 2025  
**Your Available Capacitors:**
- 100µF / 50V (electrolytic, polarized - RED/ORANGE in your photo)
- 10µF / 50V (electrolytic, polarized - BLUE in your photo)
- 1µF / 63V (electrolytic, polarized - RED/ORANGE in your photo)

---

## ⚠️ CRITICAL: Understanding Polarity

Your capacitors are **ELECTROLYTIC** = **POLARIZED** = **HAVE POSITIVE (+) AND NEGATIVE (-) SIDES**

### How to Identify Polarity

**Look at the capacitor body:**
- **Negative side:** Has a stripe with minus signs (----) printed on it
- **Positive side:** No stripe, or sometimes has "+" marking
- **Longer leg:** Usually positive (+)
- **Shorter leg:** Usually negative (-)

**⚠️ IMPORTANT:** If you connect these backwards, they can:
- Fail to work
- Get hot
- Bulge or leak
- **Explode** (rare but possible)

**Always double-check before connecting power!**

---

## What Capacitors Do (Simple Explanation)

Think of capacitors as tiny rechargeable batteries that:
1. **Store electrical energy** when voltage is high
2. **Release energy** when voltage drops suddenly
3. **Smooth out voltage bumps** caused by motor drawing power

**Why you need them:**
- Your motor suddenly draws lots of current when moving
- This creates voltage "dips" in the power supply
- These dips appear as electrical noise
- This noise confuses your sensors (making readings jump around)
- Capacitors "fill in" these dips, keeping voltage stable

---

## Installation Plan - 3 Locations

You'll install capacitors at **3 different locations**:

```
Location 1: TMC2209 Motor Power (VMOT)
Location 2: Arduino 5V Rail
Location 3: AS5600 Sensors (both sensors)
```

---

## Location 1: TMC2209 Motor Driver Power (MOST IMPORTANT)

### Why Here?
The motor driver is where the noise starts. This is the **highest priority** installation.

### What You Need
- **1x 100µF / 50V** capacitor (your orange/red large one)

### Where Exactly?

**Find these pins on your TMC2209 driver:**
- **VMOT** (motor power input) - This connects to your 12-24V power supply
- **GND** (ground near VMOT) - This connects to power supply ground

### Step-by-Step Installation

#### Step 1: Power OFF Everything
```
❌ Disconnect Arduino USB cable
❌ Disconnect motor power supply
❌ Wait 10 seconds
```

#### Step 2: Identify Capacitor Polarity
```
Look at the 100µF capacitor:
→ Find the side with stripe and minus signs (----)  ← This is NEGATIVE
→ The other side is POSITIVE
```

#### Step 3: Bend and Trim Leads
```
1. Bend the capacitor leads (wires) at 90° about 5mm from the body
2. Trim leads to appropriate length (leave ~10mm for soldering/connecting)
3. Keep enough length to reach from VMOT to GND on your driver
```

#### Step 4: Connect Capacitor
```
POSITIVE leg (+) → Connect to VMOT pin
NEGATIVE leg (-) → Connect to GND pin (the one near VMOT)
```

**Connection Methods:**

**Option A - Soldering (BEST):**
```
1. Tin (pre-coat with solder) the capacitor leads
2. Hold positive lead against VMOT pad
3. Heat with soldering iron and apply solder
4. Repeat for negative lead to GND
5. Check connections are solid
```

**Option B - Wire Wrapping (TEMPORARY):**
```
1. Wrap positive lead tightly around VMOT wire/pin
2. Wrap negative lead tightly around GND wire/pin
3. Add electrical tape for insulation
⚠️ This is less reliable, solder when possible
```

**Option C - Breadboard (IF driver is on breadboard):**
```
1. Insert positive lead into same row as VMOT
2. Insert negative lead into same row as GND
```

#### Step 5: Physical Placement
```
→ Place capacitor as CLOSE as possible to the driver pins
→ Keep leads SHORT (reduces inductance)
→ Ideally within 1-2 cm of the driver
→ Secure with hot glue or zip tie so it doesn't flop around
```

#### Step 6: Visual Check
```
✓ Positive leg connected to VMOT?
✓ Negative leg connected to GND?
✓ No bare metal touching between legs? (no short circuit)
✓ Capacitor secure and not touching other components?
✓ Stripe side (negative) definitely going to GND?
```

### Diagram for Location 1

```
     Motor Power Supply
           |
           | 12-24V
           |
        ╔══╧══════════════════╗
        ║  VMOT (+ input)    ║
        ║       ║            ║
        ║   [100µF]          ║  ← Capacitor HERE
        ║   +   -            ║
        ║   ║   ║            ║
        ║  GND (motor side)  ║
        ╚══╤══════════════════╝
           | 
           └→ To Arduino GND (common ground)
```

---

## Location 2: Arduino 5V Power Rail

### Why Here?
The 5V rail powers your sensors. Stabilizing it prevents sensor noise.

### What You Need
- **1x 10µF / 50V** capacitor (your blue one)

### Where Exactly?

**Find these on your Arduino Mega:**
- **5V pin** (red wire from your power connections)
- **GND pin** (black wire, next to 5V)

### Step-by-Step Installation

#### Step 1: Identify Capacitor Polarity
```
Look at the 10µF blue capacitor:
→ Find the side with stripe (----)  ← This is NEGATIVE
→ The other side is POSITIVE
```

#### Step 2: Connect Capacitor
```
POSITIVE leg (+) → Connect to Arduino 5V pin
NEGATIVE leg (-) → Connect to Arduino GND pin
```

**Best Method - Use Breadboard:**
```
If your Arduino is plugged into a breadboard:
1. Find the 5V power rail row (usually marked red)
2. Find the GND rail row (usually marked blue/black)
3. Insert positive leg into 5V rail
4. Insert negative leg into GND rail
```

**Alternative - Direct Connection:**
```
1. Solder or wrap positive lead to 5V pin header
2. Solder or wrap negative lead to GND pin header
⚠️ Make sure no short circuit between pins!
```

#### Step 3: Visual Check
```
✓ Positive leg to 5V?
✓ Negative leg to GND?
✓ No shorts between pins?
```

### Diagram for Location 2

```
    Arduino Mega 2560
    ┌─────────────────┐
    │                 │
    │  5V  ───┬───→ (to sensors)
    │         │       │
    │      [10µF]     │
    │      +   -      │
    │      │   │      │
    │  GND ───┴───→ (to sensors)
    │                 │
    └─────────────────┘
```

---

## Location 3: AS5600 Sensors (Both Sensors)

### Why Here?
Local decoupling at each sensor prevents noise from traveling through power wires.

### What You Need
- **2x 1µF / 63V** capacitors (your small orange/red ones)
- One for pendulum sensor, one for motor sensor

### Where Exactly?

**At EACH AS5600 sensor board:**
- **VCC pin** (power input, connects to Arduino 5V)
- **GND pin** (ground, connects to Arduino GND)

### Step-by-Step Installation

#### For Pendulum Sensor (Hardware I2C)

**Step 1: Identify Capacitor Polarity**
```
Look at one 1µF capacitor:
→ Stripe side is NEGATIVE
→ Other side is POSITIVE
```

**Step 2: Connect at Sensor**
```
POSITIVE leg (+) → Connect to VCC pin on AS5600 board
NEGATIVE leg (-) → Connect to GND pin on AS5600 board
```

**Best Method:**
```
1. Place capacitor directly on sensor board pins
2. Solder leads to VCC and GND pads on the sensor PCB
3. Keep leads very short (< 5mm)
4. Capacitor should be touching the sensor board
```

**Alternative (if can't solder on sensor):**
```
1. Place capacitor where wires connect to Arduino
2. At the breadboard or connection point:
   - Positive leg in same row as sensor VCC wire
   - Negative leg in same row as sensor GND wire
```

**Step 3: Visual Check**
```
✓ Positive to VCC?
✓ Negative to GND?
✓ Capacitor secure?
```

#### For Motor Sensor (Software I2C)

**Repeat exact same process:**
```
1. Take second 1µF capacitor
2. Connect positive to VCC of motor sensor
3. Connect negative to GND of motor sensor
4. Place as close to sensor as possible
```

### Diagram for Location 3

```
    AS5600 Sensor #1 (Pendulum)
    ┌─────────────────┐
    │  VCC ──┬──→ Arduino 5V
    │        │        
    │      [1µF]      
    │      +  -       
    │      │  │       
    │  GND ──┴──→ Arduino GND
    │                 
    │  SDA ────→ Pin 20
    │  SCL ────→ Pin 21
    └─────────────────┘

    AS5600 Sensor #2 (Motor)
    ┌─────────────────┐
    │  VCC ──┬──→ Arduino 5V
    │        │        
    │      [1µF]      
    │      +  -       
    │      │  │       
    │  GND ──┴──→ Arduino GND
    │                 
    │  SDA ────→ Pin 22
    │  SCL ────→ Pin 24
    └─────────────────┘
```

---

## Complete Wiring Diagram with ALL Capacitors

```
External 12-24V Power Supply
    │
    ├──→ VMOT (TMC2209) ──┬──→ Motor Coils
    │                     │
    │                 [100µF]  ← Location 1 (MOST IMPORTANT)
    │                   + -
    │                   │ │
    │                   │ │
    └──→ GND ──────┬────┴─┴──→ Arduino GND (COMMON GROUND)
                   │
                   │
Arduino Mega       │
    │              │
    5V ────┬───────┴───┬──→ AS5600 #1 VCC ──┬──→ Pullups
           │           │                     │
       [10µF]          │                  [1µF]  ← Location 3a
        + -            │                   + -
        │ │            │                   │ │
        │ │            │   AS5600 #1 GND ──┴─┤
    GND ┴─┴────────────┼──────────────────────┤
                       │                      │
                       └──→ AS5600 #2 VCC ──┬─┤
                                            │ │
                                         [1µF] ← Location 3b
                                          + -
                                          │ │
                           AS5600 #2 GND ─┴─┘
                           
    Location 2 ↑
```

---

## Testing After Installation

### Step 1: Visual Inspection
```
Before applying power, check:
✓ All positive legs connected to positive (VMOT, 5V, VCC)
✓ All negative legs connected to ground
✓ No capacitors backwards
✓ No bare wires touching each other
✓ All capacitors physically secure
```

### Step 2: Multimeter Check (OPTIONAL but SAFE)
```
If you have a multimeter:
1. Set to continuity/resistance mode
2. Check no short between positive and negative rails
3. Should read >1kΩ or open circuit
4. If it beeps or reads <10Ω, you have a short! Fix before powering!
```

### Step 3: Power On Test
```
1. Connect Arduino USB (this powers logic only)
2. Open serial monitor (115200 baud)
3. Check sensors respond (option 0 - I2C scan)
4. If OK, disconnect USB
5. Connect motor power supply
6. Reconnect USB and check serial monitor
7. Try sensor stability test (option 9)
```

### Step 4: Success Criteria
```
Expected results with capacitors installed:
→ Sensor readings stable with motor power OFF: >95% (already achieved)
→ Sensor readings stable with motor power ON: >90% (NEW with capacitors)
→ Pendulum angle shouldn't jump more than ±10° randomly
→ Motor angle should be smooth

If still noisy:
→ Check capacitor polarity (most common mistake)
→ Check capacitors are close to pins (within 2cm)
→ Try adding more capacitors in parallel if needed
```

---

## Common Beginner Mistakes (AVOID THESE!)

### ❌ Mistake 1: Wrong Polarity
```
Problem: Connected + to GND and - to positive rail
Result: Capacitor may bulge, get hot, or fail
Fix: Always check stripe goes to negative/ground
```

### ❌ Mistake 2: Capacitors Too Far Away
```
Problem: Placed capacitor 10cm away from component
Result: Won't filter noise effectively
Fix: Place within 1-2cm of power pins
```

### ❌ Mistake 3: Long Wires
```
Problem: Used long wires to connect capacitor
Result: Wires act as antennas, make noise worse
Fix: Keep leads short and direct
```

### ❌ Mistake 4: Forgot Common Ground
```
Problem: Motor power ground not connected to Arduino ground
Result: Noise travels between systems, sensors fail
Fix: ALL grounds must connect together (common ground)
```

### ❌ Mistake 5: Only One Location
```
Problem: Added capacitor only at motor driver
Result: Some noise still remains
Fix: Install at all 3 locations for best results
```

---

## Safety Checklist

Before connecting power:
- [ ] All capacitor polarities double-checked
- [ ] No bare wires or leads touching each other
- [ ] Common ground connected (motor PSU, Arduino, driver)
- [ ] Capacitors physically secured (not flopping around)
- [ ] No shorts between power and ground (multimeter check)
- [ ] Emergency power disconnect ready (unplug cable quickly if needed)

---

## What to Expect After Installation

### Immediate Effect
- System will power on normally (no visual change)
- Sensors will report same values at rest
- Voltage rails will be more stable (measure with multimeter)

### During Motor Movement
**BEFORE capacitors:**
```
Sensor stability: 0-20% (readings jump wildly)
Pendulum: 70° → -30° → 90° → -50° (chaos!)
Motor: 180° → 120° → 200° → 150° (unstable)
```

**AFTER capacitors:**
```
Sensor stability: >90% (readings smooth)
Pendulum: 90° → 91° → 90° → 89° (stable!)
Motor: 180° → 181° → 180° → 179° (smooth!)
```

---

## Quick Shopping List (If You Need More)

For future projects, ideal capacitor kit:
- **100µF / 50V** electrolytic (power supply filtering)
- **10µF / 25V** electrolytic (5V rail filtering)
- **0.1µF (100nF)** ceramic (high-frequency noise)
- **1µF** electrolytic or ceramic (sensor power)

**Note:** You have everything needed for this project! 🎉

---

## Still Confused? Contact Points

If you're unsure about anything:
1. Take a photo of your setup
2. Point out where you're stuck
3. Ask specific questions like:
   - "Is this the positive side?"
   - "Where exactly is VMOT on my driver?"
   - "Which wire is ground?"

**Better to ask than risk damaging components!**

---

## Summary - What Goes Where

| Location | Component | Capacitor | Positive To | Negative To |
|----------|-----------|-----------|-------------|-------------|
| 1 | TMC2209 Driver | 100µF | VMOT pin | GND (motor side) |
| 2 | Arduino | 10µF | 5V pin | GND pin |
| 3a | AS5600 Pendulum | 1µF | VCC pin | GND pin |
| 3b | AS5600 Motor | 1µF | VCC pin | GND pin |

**Total capacitors used: 4 pieces (all from your collection)**

---

## Next Steps After Installation

1. Install capacitors following this guide
2. Power on and test sensor stability (option 9 in serial menu)
3. If stable (>90% success), proceed to calibration verification
4. Test full range movement (option 6)
5. If all OK, ready to implement swing-up control! 🎉

---

**Good luck! Take your time, double-check polarity, and you'll be fine!** 👍

