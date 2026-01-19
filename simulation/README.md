# Furuta Pendulum Simulation Environment

A Python-based simulation environment that exactly mirrors the hardware system.

## Purpose

- Test controller gains **before** hardware deployment
- Debug controller behavior in a controlled environment  
- Compare simulation vs hardware logs
- Perform stability analysis and gain sweeps

## Architecture

```
simulation/
├── furuta_model.py      # Nonlinear dynamics (Lagrangian EOMs)
├── controller.py        # Discrete-time controller (matches firmware)
├── simulator.py         # Closed-loop simulation engine
├── visualize.py         # Plotting and animation
├── pendulum_simulation.ipynb  # Interactive experiments
└── requirements.txt     # Python dependencies
```

## Quick Start

1. **Install dependencies:**
   ```bash
   cd simulation
   pip install -r requirements.txt
   ```

2. **Run a quick test:**
   ```bash
   python simulator.py
   ```

3. **Open the notebook for interactive experiments:**
   ```bash
   jupyter notebook pendulum_simulation.ipynb
   ```

## Model Details

### Nonlinear Dynamics
The simulation uses the **full nonlinear equations of motion** from Lagrangian mechanics:

**Arm equation:**
```
(Ĵ₀ + Ĵ₂sin²α)θ̈ + K·cos(α)·α̈ + Ĵ₂·sin(2α)·θ̇·α̇ - K·sin(α)·α̇² = τ
```

**Pendulum equation:**
```
K·cos(α)·θ̈ + Ĵ₂·α̈ - ½Ĵ₂·sin(2α)·θ̇² - G·sin(α) = 0
```

### Controller
The discrete controller matches the firmware **exactly**:
- 200 Hz loop rate
- Same filter coefficients
- Same saturation limits
- Same engage ramp
- Same outer loop logic

### Parameters
All physical parameters match your hardware:
| Parameter | Value | Description |
|-----------|-------|-------------|
| L_r | 190 mm | Arm length |
| L_p | 120 mm | Pendulum length |
| m_p | 11.96 g | Swinging mass |
| l_p | 87.7 mm | Pendulum COM |
| J_1 | 1.021e-4 kg·m² | Pendulum inertia |
| Ĵ₀ | 1.13e-3 kg·m² | Arm + tip inertia |

## Key Features

### 1. Gain Sweep
Test stability across a range of KP/KD values:
```python
from simulator import run_gain_sweep
results = run_gain_sweep(
    kp_values=[400, 600, 742, 900],
    kd_values=[40, 54.6, 70]
)
```

### 2. Disturbance Test
Apply impulse disturbances:
```python
config = SimulationConfig(
    disturbance_time_s=1.0,
    disturbance_alpha_deg=10.0  # 10° kick
)
```

### 3. Noise Injection
Test robustness to sensor noise:
```python
config = SimulationConfig(
    sensor_noise_alpha_deg=0.1  # 0.1° std dev
)
```

### 4. Export to Hardware Log Format
Save simulation data in the exact format as firmware logs:
```python
result.save_csv("simulation_log.csv")
```

## Validation

To validate the simulation against hardware:

1. Run the same initial conditions on simulation and hardware
2. Export both logs in CSV format
3. Compare trajectories using `tools/analyze_balance.ipynb`

Expected agreement:
- Natural frequency: ±5%
- Fall time constant: ±10%
- Qualitative behavior should match

## Troubleshooting

**Simulation unstable but hardware works:**
- Check for missing friction (add `b_alpha`, `b_theta` in FurutaParams)
- Verify controller gains are identical
- Check sign conventions

**Simulation works but hardware unstable:**
- Check sensor noise levels
- Verify I²C timing is consistent
- Check for mechanical backlash
