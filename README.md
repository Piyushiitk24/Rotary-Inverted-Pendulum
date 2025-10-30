# Rotary Inverted Pendulum

A control systems project implementing a rotary inverted pendulum using an Arduino Mega, TMC2209 stepper driver, 17HS4401-D motor, and AS5600 magnetic encoder.

## Hardware Components

- **Microcontroller**: Arduino Mega 2560
- **Motor Driver**: TMC2209 (configured for 3200 steps/rev with 1/16 microstepping)
- **Stepper Motor**: 17HS4401-D (1.8° step angle, 200 steps/rev)
- **Angle Sensor**: AS5600 12-bit magnetic encoder (I2C)
- **Power Supply**: External 12V-28V PSU for motor power

## Pin Configuration

See [`docs/ProjectPinMap.md`](docs/ProjectPinMap.md) for complete wiring details.

### Quick Reference

| Component | Pin | Arduino Mega |
|-----------|-----|--------------|
| TMC2209 STEP | → | Digital Pin 5 |
| TMC2209 DIR | → | Digital Pin 6 |
| TMC2209 EN | → | Digital Pin 7 |
| AS5600 SDA | → | Pin 20 (SDA) |
| AS5600 SCL | → | Pin 21 (SCL) |

**⚠️ CRITICAL**: Ensure external PSU ground is connected to Arduino GND for common ground.

## Project Structure

```
Rotary-Inverted-Pendulum/
├── platformio.ini          # PlatformIO configuration with library dependencies
├── docs/
│   └── ProjectPinMap.md    # Complete hardware wiring reference
└── src/
    ├── motor_test.cpp      # Stepper motor diagnostic test
    └── sensor_test.cpp     # AS5600 sensor diagnostic test
```

## Getting Started

### Prerequisites

- [PlatformIO](https://platformio.org/) installed (via VS Code extension or CLI)
- Arduino Mega 2560 board
- USB cable for programming

### Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/Piyushiitk24/Rotary-Inverted-Pendulum.git
   cd Rotary-Inverted-Pendulum
   ```

2. Open in VS Code with PlatformIO extension, or use the CLI.

3. PlatformIO will automatically install required libraries:
   - `robtillaart/AS5600` - AS5600 magnetic encoder library
   - `waspinator/AccelStepper` - Stepper motor control library

### Running Diagnostic Tests

#### Test 1: Sensor Direction Test

Tests the AS5600 magnetic encoder to verify wiring and determine rotation direction.

1. Rename `src/sensor_test.cpp` to `src/main.cpp` (or comment out `motor_test.cpp`)
2. Build and upload:
   ```bash
   pio run --target upload
   ```
3. Open serial monitor:
   ```bash
   pio device monitor
   ```
4. Move the pendulum by hand and observe the angle readings.
5. Note which direction (CW/CCW) increases the angle value.

**Expected Output:**
```
AS5600 Sensor Direction Test
---------------------------------
AS5600 connected.
Move the pendulum by hand and observe the 'Raw Angle'.
Raw Angle: 2048    Degrees: 180.00
Raw Angle: 2055    Degrees: 180.61
...
```

#### Test 2: Motor Direction Test

Tests the stepper motor movement and verifies direction control.

1. Rename `src/motor_test.cpp` to `src/main.cpp`
2. Build and upload:
   ```bash
   pio run --target upload
   ```
3. Open serial monitor at 115200 baud
4. Follow the on-screen prompts:
   - Manually center the rotary arm
   - Press `ENTER` to zero the position
   - Use commands: `R` (right), `L` (left), `Z` (center)

**Safety Features:**
- Movement limited to ±75° (±666 steps)
- Non-blocking motor control using `AccelStepper::run()`
- Enable pin (active LOW) for emergency stop

**Expected Output:**
```
Interactive Motor Direction Test
---------------------------------
ACTION: Manually move the rotary arm to the CENTER position.
Press [ENTER] to confirm and zero the motor.
Motor position zeroed.

Commands:
 'R' -> Move to +75 degrees (RIGHT limit)
 'L' -> Move to -75 degrees (LEFT limit)
 'Z' -> Move to Zero (CENTER)
```

## Development Notes

### Motor Configuration

- **Steps per revolution**: 3200 (200 base steps × 1/16 microstepping)
- **75° limit**: 666 steps `(75 × 3200 / 360)`
- **Max speed**: 1000 steps/sec (safe testing value)
- **Acceleration**: 2000 steps/sec²

### Sensor Specifications

- **Resolution**: 12-bit (4096 positions per revolution)
- **Angular resolution**: 0.088° (360° / 4096)
- **Interface**: I2C (default address: 0x36)
- **Update rate**: ~10 Hz in diagnostic test (100ms delay)

## Troubleshooting

### AS5600 Not Detected
- Verify I2C connections (SDA → Pin 20, SCL → Pin 21)
- Check 5V and GND connections
- Ensure magnet is positioned correctly over sensor

### Motor Not Moving
- Check enable pin is LOW (driver enabled)
- Verify motor power supply (12-28V) is connected to VM pin
- Confirm STEP and DIR pin connections (Pins 5, 6)
- Check common ground between PSU and Arduino

### Motor Moves Wrong Direction
- Swap the DIR pin logic (HIGH ↔ LOW) in code, or
- Swap motor coil pairs (A1↔A2 or B1↔B2)

## Next Steps

After successful diagnostic tests:
1. Implement PID control loop
2. Add state estimation (Kalman filter)
3. Develop swing-up controller
4. Integrate sensor feedback with motor control

## License

This project is open source. See LICENSE file for details.

## Contributing

Contributions welcome! Please open an issue or submit a pull request.

## Author

Piyush Tiwari ([@Piyushiitk24](https://github.com/Piyushiitk24))
