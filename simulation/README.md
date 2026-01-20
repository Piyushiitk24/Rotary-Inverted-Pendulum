# Furuta Pendulum Simulation

Simulation environment matching the actual hardware from FreeCAD designs.

## Quick Start - Just Use `simulate.py`

```bash
cd simulation

# 1. Check hardware parameters
python3 simulate.py --params

# 2. Run simulation with default gains  
python3 simulate.py

# 3. Test different gains
python3 simulate.py --kp 800 --kd 60 --ktheta 20 --kthetadot 80

# 4. Compare old vs new outer loop sign
python3 simulate.py --compare-signs

# 5. Just check stability (no simulation plot)
python3 simulate.py --stability-only --kp 1000 --kd 80

# 6. Different initial tilt angle
python3 simulate.py --alpha0 5
```

## What the Gains Mean

| Gain | Firmware Variable | Purpose |
|------|-------------------|---------|
| `--kp` | `ACC_KP` | Pendulum angle → acceleration (inner loop) |
| `--kd` | `ACC_KD` | Pendulum rate → acceleration (inner loop) |
| `--ktheta` | `KTHETA` | Arm position → acceleration (outer loop) |
| `--kthetadot` | `KTHETADOT` | Arm velocity → acceleration (outer loop) |

## Workflow

1. **Before hardware testing**: Run simulation to verify stability
2. **If simulation shows STABLE**: Update firmware gains and test on hardware
3. **If simulation shows UNSTABLE**: Adjust gains until stable

## Files

| File | Description |
|------|-------------|
| **`simulate.py`** | ⭐ **USE THIS** - Main simulation tool |
| `furuta_model.py` | Physics model with your hardware parameters |
| `requirements.txt` | Python dependencies (`pip install -r requirements.txt`) |

Other `.py` files are debug artifacts from development and can be ignored.

## Hardware Parameters

All parameters are from your FreeCAD macros:
- Arm: 210mm plate, 190mm effective length
- Pendulum: L-shaped rod (170mm axle + 120mm vertical), d=7.95mm
- Sphere: 34mm diameter, 7.7g
- Total swinging mass: 11.96g
- Natural frequency: ~10 rad/s (1.6 Hz)
