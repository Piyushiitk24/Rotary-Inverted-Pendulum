Rotary Inverted Pendulum - Pin Map Reference (v3)
This document details all physical connections for the project.
This version uses one hardware I2C bus and one software I2C bus to solve the sensor address conflict.
1. Power Distribution
A "common ground" is required for all components to communicate.


Source
Wire
Destination
Purpose
External 12V-28V PSU
(+) Positive
TMC2209 VM pin
Motor Power
External 12V-28V PSU
(–) Negative
TMC2209 GND pin (next to VM)
Motor Power Ground
External 12V-28V PSU
(–) Negative
Arduino Mega GND pin
Common Ground (CRITICAL)
Arduino Mega
5V
TMC2209 VIO pin
Logic Power
Arduino Mega
GND
TMC2209 GND pin (next to VIO)
Logic Power Ground
Arduino Mega
5V
AS5600 (Pendulum) VCC
Pendulum Sensor Power
Arduino Mega
GND
AS5600 (Pendulum) GND
Pendulum Sensor Ground
Arduino Mega
5V
AS5600 (Motor) VCC
Motor Sensor Power
Arduino Mega
GND
AS5600 (Motor) GND
Motor Sensor Ground

2. Stepper Motor & Driver
Controls the rotary arm movement.
Component
Pin / Wire
Connects To
Component
Pin
Arduino Mega
Digital Pin 5
➡️
TMC2209
STEP
Arduino Mega
Digital Pin 6
➡️
TMC2209
DIR
Arduino Mega
Digital Pin 7
➡️
TMC2209
EN (Enable)
TMC2209
A1
➡️
Stepper Motor
Pin 1 (Red)
TMC2209
A2
➡️
Stepper Motor
Pin 4
TMC2209
B1
➡️
Stepper Motor
Pin 3 (Black)
TMC2209
B2
➡️
Stepper Motor
Pin 6 (Green)

3. Sensors (I2C)
Measures the pendulum and motor angles independently.
I2C Bus 0 (Hardware: Wire) - PENDULUM SENSOR
Speed: Fast (Hardware)
Pins: Connect to the dedicated I2C pins.
Component
Pin
Connects To
Component
Pin
AS5600 (Pendulum)
SDA (Data)
➡️
Arduino Mega
Pin 20 (SDA)
AS5600 (Pendulum)
SCL (Clock)
➡️
Arduino Mega
Pin 21 (SCL)

I2C Bus 1 (Software: SoftwareWire) - MOTOR SENSOR
Speed: Slower (Software-driven)
Pins: Connect to two standard digital pins.
Component
Pin
Connects To
Component
Pin
AS5600 (Motor)
SDA (Data)
➡️
Arduino Mega
Pin 22
AS5600 (Motor)
SCL (Clock)
➡️
Arduino Mega
Pin 24


