# Rotary Inverted Pendulum - Pin Configuration

**Last Updated:** May 26, 2026

This document reflects the current `src/main.cpp` wiring. The default firmware
uses AS5600 sensors on an I2C multiplexer. The optional optical pendulum encoder
is selected at compile time with `PENDULUM_SENSOR_BACKEND=1`.

## Quick Reference

| Component | Arduino Mega Pins | Notes |
|-----------|-------------------|-------|
| Stepper driver | 11 (STEP), 6 (DIR), 7 (EN) | TMC2209 STEP/DIR/EN interface |
| I2C bus | 20 (SDA), 21 (SCL) | PCA9548A/TCA9548A mux at address `0x70` |
| Pendulum AS5600 (default) | mux channel 0 | `PENDULUM_SENSOR_BACKEND=0` |
| Base AS5600 | mux channel 1 | Always used for base angle |
| Optional pendulum optical encoder | 2 (A), 3 (B) | `PENDULUM_SENSOR_BACKEND=1`, 600 PPR, 4x decode |

## Stepper Motor Driver (TMC2209)

| Arduino Pin | TMC2209 Pin | Function | Logic Level |
|-------------|-------------|----------|-------------|
| 11 | STEP | Step pulse | HIGH pulse = one step |
| 6 | DIR | Direction | Direction depends on wiring and `motorSign` |
| 7 | EN | Enable | LOW = enabled, HIGH = disabled |

Power wiring:

| Source | TMC2209 Pin | Voltage | Notes |
|--------|-------------|---------|-------|
| Motor PSU + | VMOT | 24 V | Motor power |
| Motor PSU - | GND | 0 V | Common motor ground |
| Arduino 5V | VIO | 5 V | Logic power |
| Arduino GND | GND | 0 V | Common logic ground |

Recommended capacitors:

- 100 uF electrolytic from VMOT to GND near the driver.
- 0.1 uF ceramic from VMOT to GND near the driver.

## I2C Multiplexer And AS5600 Sensors

The current firmware uses one hardware I2C bus and manually selects the mux
channel before reading an AS5600.

| PCA9548A/TCA9548A Pin | Arduino Mega Pin | Notes |
|-----------------------|------------------|-------|
| VIN | 5V | Mux power |
| GND | GND | Common ground |
| SDA | 20 (SDA) | Hardware I2C data |
| SCL | 21 (SCL) | Hardware I2C clock |

Mux address:

- Default address is `0x70` with A0/A1/A2 left at the default state.
- If the mux address is changed in hardware, update `MUX_ADDR` in
  `src/main.cpp`.

Channel wiring:

| Mux Channel | Sensor | Sensor Wiring |
|-------------|--------|---------------|
| SD0/SC0 | Pendulum AS5600 | VCC=5V, GND=GND, SDA=SD0, SCL=SC0 |
| SD1/SC1 | Base AS5600 | VCC=5V, GND=GND, SDA=SD1, SCL=SC1 |

AS5600 notes:

- AS5600 I2C address is fixed at `0x36`; the mux allows two sensors with the
  same address.
- Keep magnet distance roughly 2-3 mm from each chip.
- Use short I2C wiring and 4.7k-10k pullups to 5V on SDA/SCL if the breakout
  boards do not already provide suitable pullups.

## Optional 600 PPR Optical Pendulum Encoder

Use this only with firmware built as:

```bash
pio run -e megaatmega2560_enc600
```

The base angle still comes from the base AS5600 on mux channel 1. Only the
pendulum angle source changes.

| Encoder Lead | Arduino Mega Pin | Notes |
|--------------|------------------|-------|
| A phase | D2 | Interrupt pin, 5V TTL-safe input |
| B phase | D3 | Interrupt pin, 5V TTL-safe input |
| VCC | 5V | Use 5V supply for direct Mega wiring |
| GND | GND | Must share ground with Arduino and motor driver |
| Z / index | Unused | Leave disconnected unless a future firmware uses it |

Firmware interpretation:

- Encoder resolution is `600 PPR * 4 = 2400 counts/rev`.
- Angle scale is `360 / 2400 = 0.15 deg/count`.
- `Z` calibration resets the incremental encoder count to zero while the
  pendulum is held upright.
- Use the existing `S` sign wizard and `ALPHA_SIGN` setting to correct physical
  direction.

Electrical warnings:

- Do not connect 12 V or 24 V encoder outputs directly to Arduino pins.
- Direct D2/D3 wiring assumes the encoder A/B outputs are 5V TTL-safe.
- If the encoder outputs are open-collector/open-drain, add external 4.7k-10k
  pullups to 5V on A and B. The firmware also enables internal pullups, but
  external pullups are more reliable for fast edges.
- If using a higher-voltage encoder supply or differential outputs, add proper
  level shifting, line receiver, or opto isolation before D2/D3.

## Power And Grounding

All modules must share a common ground:

```text
External 24V PSU -
  -> TMC2209 GND
  -> Arduino GND
  -> sensor/encoder GND
```

Arduino 5V powers:

- TMC2209 VIO.
- PCA9548A/TCA9548A mux.
- AS5600 sensors.
- Optional optical encoder only when its outputs are 5V TTL-safe.

## Calibration Reference

- Pendulum: held upright.
- Base arm: centered.
- Stepper position: reset to 0 steps by firmware during `Z`.
- Default AS5600 pendulum: `Z` samples and stores the upright absolute angle.
- Optional optical pendulum encoder: `Z` resets the incremental count to zero.

## Wiring Checklist

- [ ] TMC2209 STEP/DIR/EN connected to 11/6/7.
- [ ] TMC2209 VMOT powered from motor PSU and VIO powered from Arduino 5V.
- [ ] Common ground between PSU, TMC2209, Arduino, and sensors.
- [ ] I2C mux connected to Mega SDA/SCL on 20/21.
- [ ] Base AS5600 connected to mux channel 1.
- [ ] Default build: pendulum AS5600 connected to mux channel 0.
- [ ] Optical build only: encoder A/B connected to D2/D3 and outputs verified
      as 5V TTL-safe.
