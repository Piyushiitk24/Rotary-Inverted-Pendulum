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


07 Jan 2026
Swapped driver with two trees tmc 2209 driver.
Checking for polarity again:
Proved correct polarity for both motor and pendulum.

Now hysteresis test:
Result:
[DEV] Staircase Test Ready.
[DEV] 1. Manually Center Arm.
[DEV] 2. Type 'Z' to Zero everything.
[DEV] 3. Type 'S' to start the Step-by-Step sweep.
z
[CMD] > z
[DEV] System Zeroed. Stepper=0, Sensor=0.
s
[CMD] > s

[LOG] Session #0 Started (/dev/cu.usbmodem212301)
[DEV] step_count,expected_deg,actual_deg,error_deg
[DEV] 0,0.00,-2.46,2.46
[DEV] 20,4.50,-6.77,11.27
[DEV] 40,9.00,-11.34,20.34
[DEV] 60,13.50,-15.91,29.41
[DEV] 80,18.00,-20.30,38.30
[DEV] 100,22.50,-24.70,47.20
[DEV] 120,27.00,-29.18,56.18
[DEV] 140,31.50,-33.66,65.16
[DEV] 160,36.00,-38.06,74.06
[DEV] 180,40.50,-42.36,82.86
[DEV] 200,45.00,-46.85,91.85
[DEV] 220,49.50,-51.42,100.92
[DEV] 240,54.00,-55.72,109.72
[DEV] 250,56.25,-57.92,114.17
[DEV] 230,51.75,-53.53,105.28
[DEV] 210,47.25,-49.13,96.38
[DEV] 190,42.75,-44.82,87.57
[DEV] 170,38.25,-40.34,78.59
[DEV] 150,33.75,-35.77,69.52
[DEV] 130,29.25,-31.46,60.71
[DEV] 110,24.75,-26.98,51.73
[DEV] 90,20.25,-22.59,42.84
[DEV] 70,15.75,-18.02,33.77
[DEV] 50,11.25,-13.62,24.87
[DEV] 30,6.75,-9.23,15.98
[DEV] 10,2.25,-4.75,7.00
[DEV] -10,-2.25,-0.09,-2.16
[DEV] -30,-6.75,4.31,-11.06
[DEV] -50,-11.25,8.79,-20.04
[DEV] -70,-15.75,13.27,-29.02
[DEV] -90,-20.25,17.84,-38.09
[DEV] -110,-24.75,22.32,-47.07
[DEV] -130,-29.25,26.81,-56.06
[DEV] -150,-33.75,31.29,-65.04
[DEV] -170,-38.25,35.95,-74.20
[DEV] -190,-42.75,40.34,-83.09
[DEV] -210,-47.25,44.91,-92.16
[DEV] -230,-51.75,49.39,-101.14
[DEV] -250,-56.25,54.05,-110.30
[DEV] -250,-56.25,54.05,-110.30
[DEV] -230,-51.75,49.48,-101.23
[DEV] -210,-47.25,44.91,-92.16
[DEV] -190,-42.75,40.52,-83.27
[DEV] -170,-38.25,36.04,-74.29
[DEV] -150,-33.75,31.38,-65.13
[DEV] -130,-29.25,26.81,-56.06
[DEV] -110,-24.75,22.41,-47.16
[DEV] -90,-20.25,17.84,-38.09
[DEV] -70,-15.75,13.27,-29.02
[DEV] -50,-11.25,8.79,-20.04
[DEV] -30,-6.75,4.39,-11.14
[DEV] -10,-2.25,-0.09,-2.16
[LOG] END tag received, reason=test_complete
[WARN] No data to save - session empty

This means no stalling, no skipping but the setMotorVelocity is reversed. Changing setMotorVelocity run forward to run backward.

In setMotorVelocity function swapped the runForward and runBackward calls.