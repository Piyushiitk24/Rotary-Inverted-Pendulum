import pandas as pd
import matplotlib.pyplot as plt
import glob
import os
import sys

# Finds the newest CSV in the logs/ folder
list_of_files = glob.glob('logs/*.csv')
if not list_of_files:
    print("No log files found in logs/!")
    sys.exit()

latest_file = max(list_of_files, key=os.path.getctime)
print(f"Analyzing: {latest_file}")

try:
    # Read CSV, skipping metadata rows if necessary, but the logger puts them in separate cols
    # The logger structure is: time_s, setpoint, pend, base, p_vel, b_vel, cmd_hz, ...
    df = pd.read_csv(latest_file)
except Exception as e:
    print(f"Error reading CSV: {e}")
    sys.exit()

# Plotting
plt.figure(figsize=(12, 8))

# Subplot 1: Angles
plt.subplot(3, 1, 1)
plt.plot(df['time_s'], df['pendulum_deg'], label='Pendulum (deg)', color='red')
plt.plot(df['time_s'], df['base_deg'], label='Base (deg)', color='blue')
plt.axhline(0, color='black', linestyle='--', alpha=0.3)
plt.title(f"Angles - {latest_file}")
plt.legend()
plt.grid(True)

# Subplot 2: Velocities
plt.subplot(3, 1, 2)
plt.plot(df['time_s'], df['pendulum_vel'], label='Pend Vel (deg/s)', color='orange', alpha=0.7)
plt.legend()
plt.grid(True)

# Subplot 3: Control Output
plt.subplot(3, 1, 3)
plt.plot(df['time_s'], df['control_output'], label='Motor Command (Hz)', color='green')
plt.ylabel('Hz')
plt.xlabel('Time (s)')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()

# Time Step Analysis
dt = df['time_s'].diff()
print(f"Mean DT: {dt.mean()*1000:.2f} ms")
print(f"Max DT:  {dt.max()*1000:.2f} ms")