Rotary Inverted Pendulum - Pin Map Reference
==========================================

This document details all physical connections for the project, combining the Arduino Mega, TMC2209 driver, 17HS4401-D motor, AS5600 sensor, and power.

## 1. Power Distribution

This is the most critical part. A "common ground" is required for all components to communicate.

| Source | Wire | Destination | Purpose |
|---|---|---|---|
| External 12V-28V PSU | (+) Positive | TMC2209 VM pin | Motor Power |
| External 12V-28V PSU | (–) Negative | TMC2209 GND pin (next to VM) | Motor Power Ground |
| External 12V-28V PSU | (–) Negative | Arduino Mega GND pin | Common Ground (CRITICAL) |
| Arduino Mega | 5V | TMC2209 VIO pin | Logic Power |
| Arduino Mega | GND | TMC2209 GND pin (next to VIO) | Logic Power Ground |
| Arduino Mega | 5V | AS5600 VCC (or 5V) pin | Sensor Power |
| Arduino Mega | GND | AS5600 GND pin | Sensor Ground |

## 2. Stepper Motor & Driver

Controls the rotary arm movement.

| Component | Pin / Wire | Connects To | Component | Pin |
|---|---:|---|---|---:|
| Arduino Mega | Digital Pin 5 | ➡️ | TMC2209 | STEP |
| Arduino Mega | Digital Pin 6 | ➡️ | TMC2209 | DIR |
| Arduino Mega | Digital Pin 7 | ➡️ | TMC2209 | EN (Enable) |
| TMC2209 | A1 | ➡️ | Stepper Motor | Pin 1 (Red) |
| TMC2209 | A2 | ➡️ | Stepper Motor | Pin 4 |
| TMC2209 | B1 | ➡️ | Stepper Motor | Pin 3 (Black) |
| TMC2209 | B2 | ➡️ | Stepper Motor | Pin 6 (Green) |

## 3. Pendulum Sensor (AS5600)

Measures the pendulum's angle using the I2C protocol.

| Component | Pin | Connects To | Component | Pin |
|---|---:|---|---|---:|
| AS5600 | SDA (Data) | ➡️ | Arduino Mega | Pin 20 (SDA) |
| AS5600 | SCL (Clock) | ➡️ | Arduino Mega | Pin 21 (SCL) |
| AS5600 | VCC / 5V | ➡️ | Arduino Mega | 5V |
| AS5600 | GND | ➡️ | Arduino Mega | GND |

Notes
-----

- Ensure the external PSU negative is connected to the Arduino Mega GND — a common ground is critical.
- TMC2209 VM should receive the motor supply (12–28V). VIO should be powered from the Arduino 5V if your TMC2209 breakout expects 5V logic.
