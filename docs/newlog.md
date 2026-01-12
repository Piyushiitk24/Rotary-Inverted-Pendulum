Part 1: Upload & Safety Setup

Initialised at zero position and got:
((.venv) ) piyush@Piyushs-MacBook-Pro Rotary-Inverted-Pendulum % python3 tools/balance_plot.py --port /dev/cu.usbmodem212301 --baud 500000
[SYSTEM] Starting logger on /dev/cu.usbmodem212301 @ 500000
[SYSTEM] Connected.
[SYSTEM] Log directory: /Users/piyush/code/Rotary-Inverted-Pendulum/logs

[UI] Controls: S,X,P#,I#,D#,M#,Y,Z,T. 'q' to quit.

[DEV] RIP V8.0 Ready. Baud: 500000
[DEV] Use 'Y'/'Z' to zero. 'V100' to test motor. 'S' to balance.
z
[CMD] > z
[DEV] Motor Zero Set: 270.35

then moved to short of one side edge approx and got:
t
[CMD] > t
[DEV] Pend: -179.65 | Motor: -63.72 | P_Vel: -0.69 | TargetHz: 0.00

Then moved to opposite edge approx and got:
t
[CMD] > t
[DEV] Pend: -178.60 | Motor: 62.40 | P_Vel: -0.00 | TargetHz: 0.00

Then manually back to approx zero position and got:
[CMD] > t
[DEV] Pend: 180.00 | Motor: 0.53 | P_Vel: 0.04 | TargetHz: 0.00

Diagnosis: Success.

We are good to proceed.

Here is what the data proved:

Zeroing Works: The system correctly defined our center point (0.53 is close enough to 0).

Symmetry: We have roughly ±63° of range before we hit our physical stops.

Note: The code has a SOFT_LIMIT at 60°. This means during balancing, if the arm goes past 60°, it will start braking. This is perfect for our desk setup.

Part 2: The Polarity Tests
Zeroed the pendulum:
[CMD] > y
[DEV] Pend Zero Set: 63.37

Then moved the pendulum to right:
[CMD] > t
[DEV] Pend: 18.02 | Motor: 0.18 | P_Vel: 0.00 | TargetHz: 0.00

Then moved the pendulum to left:
t
[CMD] > t
[DEV] Pend: -18.72 | Motor: -0.09 | P_Vel: -0.00 | TargetHz: 0.00

Zeroed rotary arm again:
[CMD] > z
[DEV] Motor Zero Set: 271.58

Moved rotary arm to right:
[CMD] > t
[DEV] Pend: 1.58 | Motor: -28.83 | P_Vel: -0.33 | TargetHz: 0.00

Test with V200 command to verify motor direction:
[CMD] > V200
[DEV] Test Motor Hz: 200.00
The motor moved rotary arm to the right in the direction assumed right for rotary arm in previous test.

Pendulum: CORRECT. (Tilted Right = Positive).

Motor Actuation: CORRECT. (Positive Command = Moves Right).

Physics Check: If the pendulum falls Right (+), the controller sends a Positive command. Your motor moves Right to catch it. This is exactly what we need.

Motor Sensor: INVERTED. (Moved Right = Negative -28.83).

The Problem: If the base drifts Right, the code thinks it went Left (Negative). The "Centering" logic will try to push it "Right" to fix it, causing a runaway acceleration into the wall.

The Fix: We need to invert the Motor Sensor logic so that Moving Right reads Positive.
// FIX: Inverted sign so Right movement becomes Positive
  float mDeg = -(mDegRaw - motorZeroDeg);
  
  float currentMotorAngle = normalizeAngle(mDeg);
Reran the tests after code change:

[CMD] > z
[DEV] Motor Zero Set: 178.86

Moved rotary arm to right:
t
[CMD] > t
[DEV] Pend: -179.92 | Motor: 30.76 | P_Vel: 0.60 | TargetHz: 0.00

What I was also thinking was to establish how well our motor sensor fixed at the bottom of stepper motor tracks the actual motor shaft rotation. I want to check it at slow and fast speeds both, maybe we can see with a sinusoidal signal maintaining the 60 degree limits on either side. We can give a desired trajectory which has both the low frequency and high frequency as for rotary inverted pendulum the movement needs to be very small but fast, like vibrations. So, we can define a trajectory which compares the response from sensor and actual motor. If there are variations in this tracking then no matter what we do we won't be able to control, so let's check this first. I also want to run this because I suspect that the sensor at the bottom of the motor is not reliable and might not work perfectly at high speeds and also I am highly suspicious that the zeroing that we do for motor sensor is varying and changes unreliably.

Diagnosis:
Either the rotary arm is slipping on the motor shaft or the motor is skipping steps at higher speeds.

Tested with marker the rotary arm did not slip.
Now trying to reduce acceleration to see if motor skips steps.
Changing acceleration to 5000 from 40000.


## 07 Jan 2026 — Upload, safety checks, polarity checks, and first diagnosis

### Part 1 — Upload & safety setup (range + zero sanity)
- Started serial logger:
  - `python3 tools/balance_plot.py --port /dev/cu.usbmodem212301 --baud 500000`
- Firmware banner confirmed:
  - `RIP V8.0 Ready. Baud: 500000`
  - Commands available: `S,X,P#,I#,D#,M#,Y,Z,T` (+ `V###` motor test)
- Motor zero check:
  - Sent `Z` → `Motor Zero Set: 270.35`
  - Moved arm near one side and checked `T`:
    - Motor ≈ `-63.72°`
  - Moved to other side and checked `T`:
    - Motor ≈ `+62.40°`
  - Moved back near center and checked `T`:
    - Motor ≈ `+0.53°`
- Conclusion:
  - Motor zeroing works (center near 0°).
  - Mechanical range is about **±63°**.
  - Keeping SOFT_LIMIT at **60°** makes sense for desk tests.

---

### Part 2 — Polarity tests (pendulum, motor command, motor encoder)
#### 1) Pendulum sensor polarity
- Sent `Y` → `Pend Zero Set: 63.37`
- Manual tilt checks with `T`:
  - Pendulum Right tilt → `+18.02°`
  - Pendulum Left tilt → `-18.72°`
- Conclusion: **Pendulum sign is correct** (Right = +, Left = −).

#### 2) Motor actuation direction (command polarity)
- Sent `V200` (motor test)
- Observation: motor moved the rotary arm **to the right** for a positive command.
- Conclusion: **Motor command sign is correct** (Positive command = Right).

#### 3) Motor encoder polarity (base angle sensor sign)
- Sent `Z` again → `Motor Zero Set: 271.58`
- Moved rotary arm Right and checked `T`:
  - Motor reported `-28.83°` (negative) while physically moving Right.
- Conclusion: **Motor encoder sign was inverted** (Right was reading negative).

---

### Fix applied (motor encoder sign)
- Implemented sign invert so that Right movement reads positive:
  - `mDeg = -(mDegRaw - motorZeroDeg);`
  - `currentMotorAngle = normalizeAngle(mDeg);`
- Re-test:
  - Sent `Z` → `Motor Zero Set: 178.86`
  - Moved rotary arm Right and checked `T`:
    - Motor ≈ `+30.76°`
- Conclusion: **Motor encoder polarity fixed**.

---

### Follow-up suspicion: tracking reliability at speed
- Concern:
  - If the motor encoder is unreliable at high speed (or if steps are skipped), balancing will be impossible.
- Mechanical slip check:
  - Marked arm/shaft → **no slipping observed**.
- Next hypothesis:
  - Possible skipping at high acceleration.
- Change to try:
  - Reduce acceleration from **40000 → 5000** (to see if behavior improves at higher-speed tests).

---

### Staircase / hysteresis test (initial interpretation)
- Ran staircase test (`Z` then `S`) and logged:
  - `step_count, expected_deg, actual_deg, error_deg`
- Observation:
  - No obvious stalling/skipping in motion,
  - But expected vs actual had opposite sign trend (expected + while sensor was −, etc.).
- Conclusion:
  - This looked like a **direction/sign mapping issue**, not step loss.
- Change made:
  - In `setMotorVelocity()`, swapped `runForward` and `runBackward` calls.