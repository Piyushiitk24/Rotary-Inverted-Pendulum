# PCB Design Brief - Rotary Inverted Pendulum

This document is the starting point for turning the working bench wiring into a
PCB. It is based on the live firmware in `src/main.cpp`, the thesis hardware
chapter, and the current experiment workflow.

The safest first PCB is an Arduino Mega shield / carrier board, not a full
custom controller. Keep the known-good Arduino Mega and TMC2209 module style,
then use the PCB to remove breadboard wiring, improve grounding, add protected
connectors, and make the I2C sensor wiring repeatable.

## Current System

Functional blocks:

```text
Host PC
  |
  | USB serial, firmware baud 500000
  v
Arduino Mega 2560
  | D11 STEP, D6 DIR, D7 EN
  v
TMC2209 stepper driver module
  | A1/A2/B1/B2 motor phase outputs
  v
Stepper motor and rotary arm

Arduino Mega 2560
  | D20 SDA, D21 SCL
  v
PCA9548A / TCA9548A I2C mux, address 0x70
  | channel 0
  v
Pendulum AS5600 sensor, alpha
  | channel 1
  v
Base AS5600 sensor, theta
```

Power domains:

- Motor domain: external 24 V DC supply into the TMC2209 VMOT rail.
- Logic/sensor domain: 5 V from the Arduino Mega/USB for TMC2209 VIO, I2C mux,
  and AS5600 sensor modules.
- Grounds must be common, but motor-current return must not share narrow traces
  with sensor return.

Current firmware assumptions:

- `STEP_PIN = 11`
- `DIR_PIN = 6`
- `EN_PIN = 7`
- `MUX_ADDR = 0x70`
- mux channel 0 is pendulum AS5600
- mux channel 1 is base AS5600
- `Wire.setClock(400000)` in current firmware
- control loop is 200 Hz
- AS5600 readings are absolute I2C angle readings, not PWM output readings
- base safety limit is about +/-80 deg
- pendulum safety limit is about +/-30 deg

## Recommended First PCB Scope

Build a carrier PCB that plugs into or wires cleanly to the Arduino Mega and
hosts:

- Arduino Mega header connections for D11, D6, D7, D20, D21, 5 V, and GND.
- TMC2209 StepStick/module footprint or sockets.
- 24 V motor input connector with protection and bulk capacitance.
- 4-pin stepper motor connector.
- PCA9548A/TCA9548A mux, either as an IC footprint or a small module footprint.
- two keyed AS5600 cable connectors: pendulum and base.
- optional shield-drain pads next to each AS5600 connector.
- test points for 24 V, 5 V, GND, STEP, DIR, EN, SDA, SCL, mux channel lines.
- optional hardware e-stop / motor-disable connector.

Do not put the AS5600 chips on the main PCB unless the PCB is physically mounted
at the two rotating axes. The sensors must remain mechanically centered on their
magnets; the main PCB should normally connect to small sensor boards or existing
AS5600 modules through short shielded cables.

## Net Map

### Arduino Mega to TMC2209

| Signal | Arduino Mega | TMC2209 module | Notes |
| --- | --- | --- | --- |
| STEP | D11 | STEP | FastAccelStepper uses this pin with `FAS_TIMER_MODULE=1`. |
| DIR | D6 | DIR | Direction is handled by firmware signs and `motorSign`. |
| EN | D7 | EN / ENABLE | TMC enable is normally active-low on modules. Add pull-up so driver defaults disabled. |
| 5 V | 5 V | VIO | Logic only. Do not connect to VMOT. |
| GND | GND | GND | Common reference for logic and motor driver. |

Add these default-state parts:

- 10 kOhm pull-up from EN to 5 V so the driver is disabled while the Arduino is
  resetting or disconnected.
- optional 100 kOhm pulldown on STEP so no false pulse is created while the
  controller is floating.
- optional 100 kOhm pulldown on DIR so direction has a defined reset state.

### TMC2209 power and motor

| Net | Connection | PCB requirement |
| --- | --- | --- |
| VMOT_24V | 24 V input positive to TMC2209 VMOT | wide trace/pour, fused input, keep away from I2C. |
| PGND | 24 V input negative to TMC2209 power GND | wide trace/pour, return directly to supply connector. |
| MOTOR_A1/A2/B1/B2 | TMC2209 motor outputs to 4-pin motor connector | route as high-current switching nets; keep away from sensors. |
| VIO_5V | Arduino 5 V to TMC2209 logic | local 0.1 uF decoupling. |

Recommended power protection and filtering:

- 24 V screw terminal or locking connector rated above motor current.
- fuse or resettable fuse in series with 24 V input.
- reverse-polarity protection if board space allows.
- TVS diode footprint across 24 V input if using long supply leads.
- 100 uF electrolytic, 50 V or higher, close to VMOT/GND pins.
- 0.1 uF ceramic close to VMOT/GND pins.
- do not route 24 V under the sensor mux or sensor connectors.

If using a bare TMC2209 IC instead of a module, stop and redesign from the
datasheet: current sense resistors, thermal pad, copper area, charge-pump caps,
and UART/config pins become first-order design items. For a first PCB, use the
known module footprint.

### Arduino Mega to I2C mux

| Signal | Arduino Mega | Mux side | Notes |
| --- | --- | --- | --- |
| SDA | D20 | SDA | main I2C bus |
| SCL | D21 | SCL | main I2C bus |
| 5 V | 5 V | VCC/VIN | match the mux/module used |
| GND | GND | GND | common logic ground |
| A0/A1/A2 | GND | address pins | keeps mux at 0x70 |
| RESET | pull high | reset pin | if using IC, pull up to VCC; expose test pad if possible |

Pull-up rule:

- Fit optional pull-ups on the main SDA/SCL bus, 4.7 kOhm to 5 V is a normal
  starting value.
- Make pull-ups configurable with solder jumpers or DNP footprints because many
  AS5600 and PCA9548A breakout modules already include pull-ups.
- Avoid making total pull-up resistance too low by accidentally paralleling
  every module pull-up.

### Mux to AS5600 sensor connectors

Use two identical sensor connectors:

| Connector pin | Pendulum connector | Base connector | Notes |
| --- | --- | --- | --- |
| 1 | 5 V_SENSOR | 5 V_SENSOR | AS5600 module supply |
| 2 | GND_SENSOR | GND_SENSOR | sensor return |
| 3 | SDA_CH0 | SDA_CH1 | muxed I2C data |
| 4 | SCL_CH0 | SCL_CH1 | muxed I2C clock |
| 5 | SHIELD_DRAIN | SHIELD_DRAIN | connect cable foil/shield at PCB end only |

Recommended per-channel parts:

- 22 to 47 Ohm series resistor in SDA near the mux/main PCB connector.
- 22 to 47 Ohm series resistor in SCL near the mux/main PCB connector.
- optional common-mode choke footprint or ferrite option for each sensor cable.
- optional ESD protection footprint if cables are frequently unplugged.
- 0.1 uF and 1 uF from sensor 5 V to GND near the connector, and another 0.1 uF
  on the actual sensor board/module.

Cable rule:

- Keep sensor cables short.
- Keep them physically away from motor phase wires and the 24 V driver loop.
- Use twisted and shielded wiring for the I2C runs.
- Tie shield/drain to PCB ground at the controller/main-board end only.
- Leave the sensor-end shield floating.

The project previously saw large AS5600 jumps and bit-flip-like corruption when
I2C wiring acted like an antenna near the motor driver. The PCB must treat this
as a primary design constraint, not a cosmetic cleanup.

## Layout Rules

Board partitioning:

- Put 24 V input, TMC2209 module, VMOT bulk cap, and motor connector on one side
  of the board.
- Put the I2C mux and sensor connectors on the opposite side.
- Put Arduino header pins between them only if it helps shorten STEP/DIR/EN and
  SDA/SCL without crossing motor-current paths.

Grounding:

- Use a solid ground plane if possible.
- Make the high-current motor return path wide and direct from TMC2209 back to
  the 24 V input connector.
- Do not force AS5600 sensor return current to share a thin section with motor
  current.
- Join logic ground and motor power ground at a low-impedance region near the
  driver/power entry, not through the sensor connector area.

Routing:

- VMOT, PGND, and motor phase traces should be wide. Use copper pours for motor
  supply and return if board size allows.
- STEP/DIR/EN are logic traces; keep them short and away from motor phase nets.
- Route SDA/SCL as short, quiet traces with adjacent ground. Do not run them
  parallel to motor phase traces.
- Avoid long stubs on I2C.
- Keep AS5600 cable connectors far from the TMC2209 and motor connector.
- Put decoupling capacitors on the same side and close to the pins they serve.

Mechanical and usability:

- Use keyed connectors so pendulum and base sensors cannot be swapped casually.
  If the same connector style is used, label them clearly on silkscreen:
  `PEND_CH0` and `BASE_CH1`.
- Label motor coil connector pins clearly. Do not trust wire colors alone.
- Mark 24 V polarity with large silkscreen.
- Mark TMC2209 module orientation.
- Add mounting holes and keep them clear of high-current copper.
- Add test pads large enough for a multimeter probe.

## Safety And Bring-Up Features

Recommended hardware features:

- 24 V input fuse.
- 24 V power LED.
- 5 V logic LED.
- driver EN defaults to disabled by hardware pull-up.
- optional normally-closed e-stop input that forces driver disable independent
  of firmware.
- optional 24 V kill switch connector if you want a physical motor-power stop.

The current firmware has a serial emergency disarm command `!`, but a PCB should
not rely only on USB serial for safety. At minimum, make it easy to cut 24 V
motor power and make EN fail-safe disabled.

## Design Decisions Still Needed

Before schematic capture, choose these explicitly:

1. Main board form factor: Arduino Mega shield, standalone carrier with jumpers
   to Mega, or fully integrated custom controller.
2. TMC2209 implementation: exact StepStick/module footprint, or bare IC.
3. AS5600 implementation: existing sensor modules in housings, or custom small
   sensor PCBs.
4. Connector family: JST-XH/PH, screw terminals, Molex KK, Dupont headers, etc.
5. Sensor cable length and cable type.
6. Motor current and exact stepper model.
7. 24 V supply current rating and connector rating.
8. Whether the board should power only from Arduino USB/5 V, or include a 24 V
   to 5 V buck converter for standalone use.
9. Whether to expose TMC2209 UART/PDN, DIAG, MS1/MS2, and VREF on headers.

For a first revision, the conservative choices are:

- Arduino Mega shield/carrier.
- TMC2209 module footprint.
- existing AS5600 modules or small remote sensor PCBs.
- 24 V motor input kept separate from Arduino 5 V.
- no onboard buck converter unless standalone operation is required.
- optional footprints for filters/protection rather than mandatory exotic parts.

## Bring-Up Checklist

Bring the PCB up in stages:

1. No Arduino, no driver, no sensors: check for shorts between 24 V, 5 V, and GND.
2. Apply 5 V only: verify 5 V rail, mux power, pull-ups, and sensor connector
   polarity.
3. Connect Arduino only: run an I2C scan and confirm mux address 0x70.
4. Connect one AS5600 on CH0: confirm sensor reads through mux channel 0.
5. Connect one AS5600 on CH1: confirm sensor reads through mux channel 1.
6. Connect TMC2209 logic only, no 24 V motor supply: verify EN default disabled
   and STEP/DIR pins toggle under firmware.
7. Apply 24 V with motor disconnected: verify VMOT polarity and no heating.
8. Connect motor, command low-speed motion, and check direction/sign with `S`.
9. Run `G` and verify sensor status bits, raw angles, signs, and EEPROM state.
10. Run `Z`, then low-risk `E` engagement tests with one hand ready to cut 24 V.

Do not start closed-loop balancing until sensor readings are stable with motor
power on and with the motor stepping.

## Schematic Page Plan

A clean KiCad project should use these pages:

1. `01_power`: 24 V input, fuse, reverse protection, TVS footprint, VMOT bulk
   caps, optional 5 V buck placeholder if selected.
2. `02_arduino_interface`: Mega headers, D11/D6/D7/D20/D21/5V/GND, test pads.
3. `03_stepper_driver`: TMC2209 module footprint, motor connector, EN fail-safe,
   optional e-stop.
4. `04_i2c_mux`: PCA9548A/TCA9548A, address pins, pull-ups, reset, test pads.
5. `05_sensors`: pendulum/base connectors, shield drains, per-channel series
   resistors/filter footprints, local sensor-rail capacitors.
6. `06_debug`: LEDs, extra test pads, optional spare headers.

## Firmware Coupling

The PCB is coupled to firmware in these places:

- changing STEP/DIR/EN pins requires editing `STEP_PIN`, `DIR_PIN`, `EN_PIN` in
  `src/main.cpp` and checking FastAccelStepper timer/pin constraints.
- changing mux address requires editing `MUX_ADDR`.
- swapping mux channels requires swapping the `selectMux(0)` pendulum read and
  `selectMux(1)` base read assumptions, or crossing the connector labels.
- changing telemetry has no PCB impact but affects `tools/run_balance.py`.
- changing sensor type away from AS5600 requires firmware and analysis updates,
  because the controller assumes absolute wrapped angle readings.

