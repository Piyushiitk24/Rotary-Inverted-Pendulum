Rotary Inverted Pendulum
A control systems project implementing a rotary inverted pendulum using an Arduino Mega, TMC2209 stepper driver, 17HS4401-D motor, and two AS5600 magnetic encoders.
Core Hardware Components
Microcontroller: Arduino Mega 2560
Motor Driver: TMC2209 (V3.0)
Motor Power Supply: 12V DC adapter
Stepper Motor: 17HS4401-D (1.8° step angle, 200 steps/rev)
Sensor 1 (Pendulum): AS5600 12-bit magnetic encoder (Hardware I2C pins 20/21)
Sensor 2 (Motor): AS5600 12-bit magnetic encoder (Software I2C pins 22/24)
Pin Configuration
See docs/ProjectPinMap.md for the complete, definitive wiring details. This map is the single source of truth.
⚠️ CRITICAL: A common ground between the external PSU and the Arduino GND is required for the system to function.
Project Structure
Rotary-Inverted-Pendulum/
├── platformio.ini          # PlatformIO configuration with library dependencies
├── docs/
│   └── ProjectPinMap.md    # Complete hardware wiring reference
└── src/
    ├── test_all_sensors.cpp    # Diagnostic for both AS5600 sensors
    └── test_motor_with_feedback.cpp # Diagnostic for motor control and feedback


Getting Started
Prerequisites
PlatformIO installed (via VS Code extension or CLI)
Arduino Mega 2560 board
USB cable for programming
Installation
Clone the repository:
git clone <your-repo-url>
cd Rotary-Inverted-Pendulum


Open in VS Code with the PlatformIO extension.
PlatformIO will automatically install the required libraries specified in platformio.ini:
robtillaart/AS5600 - AS5600 magnetic encoder library
waspinator/AccelStepper - Stepper motor control library
Testato/SoftwareWire - For the secondary software I2C bus
Running Diagnostic Tests
To run a test, rename the corresponding file to src/main.cpp.
Test 1: Dual Sensor Test
Tests both AS5600 encoders on their separate buses to verify wiring and determine rotation directions.
Rename src/test_all_sensors.cpp to src/main.cpp.
Build and upload:
pio run --target upload


Open serial monitor (pio device monitor).
Move the pendulum and the motor arm by hand and observe the angle readings.
Note which direction (CW/CCW) increases the angle value for each sensor.
Expected Output:
Dual Sensor Test
---------------------------------
Pendulum (Hardware I2C) connecting... Success!
Motor (Software I2C) connecting... Success!
Pendulum Angle: 180.12     Motor Angle: 45.30
Pendulum Angle: 182.40     Motor Angle: 45.30
...


Test 2: Motor Test with Feedback
Tests the stepper motor movement and verifies the motor's AS5600 is reading the motion correctly.
Rename src/test_motor_with_feedback.cpp to src/main.cpp.
Build and upload.
Open serial monitor at 115200 baud.
Follow the on-screen prompts:
Manually center the rotary arm.
Press ENTER to zero the position.
Use commands: R (right), L (left), Z (center).
Safety Features:
Movement limited to $\pm75^\circ$ ($\pm666$ steps)
Non-blocking motor control using AccelStepper::run()
Enable pin (active LOW) for emergency stop
Expected Output:
Motor Test with Feedback
---------------------------------
Motor (Software I2C) sensor connected!
ACTION: Manually move arm to CENTER, then press [ENTER].
Motor position zeroed.
Arm Angle: 0.12
...
Command: R
Moving to 666...
Arm Angle: 25.50
Arm Angle: 50.12
Arm Angle: 74.95
Move complete.


Development Notes
Motor Configuration
$V_{\text{ref}}$ Setting: 1.01V (provides $\approx 0.77\text{A}$ RMS current)
Steps per revolution: 3200 (200 base steps $\times$ 1/16 microstepping, default on TMC2209)
75° limit: 666 steps (75 * 3200 / 360)
Sensor Architecture
Resolution: 12-bit (4096 positions per revolution, 0.088° resolution)
Interface: I2C (default address: 0x36)
Bus Conflict Solution:
Pendulum Sensor: Uses Hardware I2C (Wire) on Pins 20 (SDA) & 21 (SCL).
Motor Sensor: Uses Software I2C (SoftwareWire) on Pins 22 (SDA) & 24 (SCL).
Troubleshooting
AS5600 Not Detected
Verify 5V and GND connections.
Ensure magnet is positioned correctly over the sensor.
Pendulum Sensor: Check Wire connections (SDA $\rightarrow$ 20, SCL $\rightarrow$ 21).
Motor Sensor: Check SoftwareWire connections (SDA $\rightarrow$ 22, SCL $\rightarrow$ 24).
Motor Not Moving
Check enable pin is LOW (driver enabled).
Verify motor power supply (12V) is connected to VM & GND.
Confirm Common Ground between PSU and Arduino.
Check STEP and DIR pin connections (Pins 5, 6).
Motor Moves Wrong Direction
Swap the DIR pin logic (HIGH $\leftrightarrow$ LOW) in code, or
Power down and swap one motor coil pair (e.g., B1 $\leftrightarrow$ B2).
Next Steps
After successful diagnostic tests:
Implement PID/LQR control loop in a new src/main.cpp.
Add state estimation.
Develop swing-up controller.
Integrate sensor feedback from both sensors into the main control loop.
Author
Piyush Tiwari (@Piyushiitk24)