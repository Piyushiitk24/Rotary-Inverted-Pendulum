#!/usr/bin/env python3
"""
Balance Session Logger for Rotary Inverted Pendulum
Logs telemetry from full-state feedback controller to timestamped CSV files.
"""
import serial
import time
import csv
import os
from datetime import datetime
import threading

# --- CONFIGURATION ---
PORT = '/dev/cu.usbmodem12301'   # Change if needed
BAUD = 500000
BASE_LOG_DIR = 'logs/balance'

# Expected CSV format from firmware (main.cpp line ~796):
# t_ms,alphaRaw100,alphaDot100,theta100,thetaDot100,accCmd,velCmd,posMeasSteps,clamped
HEADER = [
    "timestamp_ms",
    "alpha_deg",           # Pendulum angle from upright (deg)
    "alpha_dot_deg_s",     # Pendulum angular velocity (deg/s)
    "theta_deg",           # Base position (deg)
    "theta_dot_deg_s",     # Base velocity (deg/s)
    "acc_cmd_steps_s2",    # Commanded acceleration (steps/s²)
    "vel_cmd_steps_s",     # Commanded velocity (steps/s)
    "pos_meas_steps",      # Measured position (steps)
    "position_clamped",    # 1 if near limits, 0 otherwise
]

# --- SETUP SESSION ---
session_name = datetime.now().strftime("session_%Y%m%d_%H%M%S")
session_dir = os.path.join(BASE_LOG_DIR, session_name)
os.makedirs(session_dir, exist_ok=True)

csv_path = os.path.join(session_dir, 'balance_log.csv')
events_path = os.path.join(session_dir, 'events.txt')

print("=" * 60)
print("   ROTARY INVERTED PENDULUM - BALANCE SESSION LOGGER")
print("=" * 60)
print(f"Session: {session_name}")
print(f"Logs:    {session_dir}/")
print(f"CSV:     balance_log.csv (50 Hz telemetry)")
print(f"Events:  events.txt (commands, state changes)")
print("\nFirmware telemetry format:")
print("  t_ms, alpha×100, alphaDot×100, theta×100, thetaDot×100,")
print("  accCmd, velCmd, posSteps, clamped")
print("=" * 60)
print()

stop_event = threading.Event()

def write_event(msg: str):
    with open(events_path, 'a') as ef:
        ef.write(f"{datetime.now()}: {msg}\n")

def read_serial(ser, writer):
    """Read serial data from Arduino, parse CSV telemetry, log events."""
    while not stop_event.is_set():
        try:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                # Parse CSV telemetry line (9 comma-separated values)
                if "," in line and not any(x in line for x in ["[", "FORMAT", "RIP", "---"]):
                    parts = line.split(",")
                    if len(parts) == 9:
                        try:
                            # Parse raw values from firmware
                            t_ms = int(parts[0])
                            alpha_raw100 = int(parts[1])
                            alpha_dot_raw100 = int(parts[2])
                            theta_raw100 = int(parts[3])
                            theta_dot_raw100 = int(parts[4])
                            acc_cmd = int(parts[5])
                            vel_cmd = int(parts[6])
                            pos_steps = int(parts[7])
                            clamped = int(parts[8])

                            # Convert ×100 scaled values back to real units
                            alpha = alpha_raw100 / 100.0
                            alpha_dot = alpha_dot_raw100 / 100.0
                            theta = theta_raw100 / 100.0
                            theta_dot = theta_dot_raw100 / 100.0

                            # Write to CSV
                            writer.writerow([
                                t_ms, alpha, alpha_dot, theta, theta_dot,
                                acc_cmd, vel_cmd, pos_steps, clamped
                            ])
                        except ValueError:
                            # Not valid numeric data, print as device message
                            print(f"[DEVICE]: {line}")
                            # Log important events even if the line contains commas
                            if any(key in line for key in [
                                "CALIBRATED", "ENGAGED", "FALLEN", "armEnabled",
                                "motorSign=", "ALPHA_SIGN=", "THETA_SIGN=", "CTRL_SIGN=",
                                "pendRawDeg=", "baseRawDeg=", "targetRawDeg=", "targetRawDegCal=", "motorZeroDeg=",
                                "Alpha trim",
                                "K_THETA=", "K_ALPHA=", "K_THETADOT=", "K_ALPHADOT=",
                                "ACC_KP=", "ACC_KD=", "KTHETA=", "KTHETADOT=",
                                "stateFeedback=", "Derivative Filter", "omega_c=",
                                "speedStopHz=",
                                "VEL_LEAK=", "leakAlphaWinDeg=",
                                "Saved settings to EEPROM", "Cleared EEPROM settings",
                                "GLITCH", "SENSOR", "[DBG]", "[WARN]"
                            ]):
                                write_event(f"Device: {line}")
                    else:
                        # Wrong number of columns, treat as device message
                        print(f"[DEVICE]: {line}")
                        # Log important events even if the line contains commas
                        if any(key in line for key in [
                            "CALIBRATED", "ENGAGED", "FALLEN", "armEnabled",
                            "motorSign=", "ALPHA_SIGN=", "THETA_SIGN=", "CTRL_SIGN=",
                            "pendRawDeg=", "baseRawDeg=", "targetRawDeg=", "targetRawDegCal=", "motorZeroDeg=",
                            "Alpha trim",
                            "K_THETA=", "K_ALPHA=", "K_THETADOT=", "K_ALPHADOT=",
                            "ACC_KP=", "ACC_KD=", "KTHETA=", "KTHETADOT=",
                            "stateFeedback=", "Derivative Filter", "omega_c=",
                            "speedStopHz=",
                            "VEL_LEAK=", "leakAlphaWinDeg=", "leakThetaWinDeg=",
                            "Saved settings to EEPROM", "Cleared EEPROM settings",
                            "GLITCH", "SENSOR", "[DBG]", "[WARN]"
                        ]):
                            write_event(f"Device: {line}")
                else:
                    # Status/command lines from firmware
                    print(f"[DEVICE]: {line}")

                    # Log important events
                    if any(key in line for key in [
                        "CALIBRATED", "ENGAGED", "FALLEN", "armEnabled",
                        "motorSign=", "ALPHA_SIGN=", "THETA_SIGN=", "CTRL_SIGN=",
                        "pendRawDeg=", "baseRawDeg=", "targetRawDeg=", "targetRawDegCal=", "motorZeroDeg=",
                        "Alpha trim",
                        "K_THETA=", "K_ALPHA=", "K_THETADOT=", "K_ALPHADOT=",
                        "ACC_KP=", "ACC_KD=", "KTHETA=", "KTHETADOT=",
                        "stateFeedback=", "Derivative Filter", "omega_c=",
                        "speedStopHz=",
                        "VEL_LEAK=", "leakAlphaWinDeg=", "leakThetaWinDeg=",
                        "Saved settings to EEPROM", "Cleared EEPROM settings",
                        "GLITCH", "SENSOR", "[DBG]", "[WARN]"
                    ]):
                        write_event(f"Device: {line}")

        except Exception as e:
            print(f"[ERROR] Serial: {e}")
            break

def main():
    """Main entry point: connect to Arduino, start logging, handle user commands."""
    try:
        print("Connecting to Arduino...")
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
        time.sleep(2)  # Wait for Arduino reset/initialization
        print("✓ Connected\n")
    except Exception as e:
        print(f"✗ Could not open port {PORT}: {e}")
        print("  Check: PORT variable, USB cable, device permissions")
        return

    # Open CSV file for writing
    f = open(csv_path, 'w', newline='')
    writer = csv.writer(f)
    writer.writerow(HEADER)

    # Start background thread to read serial data
    t = threading.Thread(target=read_serial, args=(ser, writer), daemon=True)
    t.start()

    print("WORKFLOW (recommended):")
    print("  After uploading firmware (once):")
    print("    R             Clear EEPROM + apply defaults")
    print("    G             Print current config (verify signs + tuning)")
    print()
    print("  Every balance attempt:")
    print("    Z             Calibrate (arm centered; pendulum UPRIGHT)")
    print("    E             Arm/disarm (auto-engages when upright & still)")
    print("    Q             Quit logger")
    print()
    print("OTHER COMMANDS (advanced):")
    print("─" * 60)
    print("  Setup/Signs:")
    print("    S             Run sign diagnostic wizard (rare)")
    print("    A 1/-1        Set ALPHA_SIGN")
    print("    H             Toggle THETA_SIGN")
    print("    B             Toggle CTRL_SIGN")
    print("    M 1/-1        Set motorSign")
    print("    Y             Save signs/tuning to EEPROM")
    print("    R             Clear EEPROM (defaults on reset)")
    print("    G             Print current config (signs + tuning)")
    print()
    print("  Tuning:")
    print("    W <val>       omega_c derivative cutoff (rad/s)")
    print("    N <val>       motor velocity deadband (steps/s); N0 disables")
    print("    U <val>       VEL_LEAK (1/s)")
    print("    1/2/4/5 <val>  K_THETA, K_ALPHA, K_THETADOT, K_ALPHADOT")
    print()
    print("  Mode/Testing:")
    print("    O             Toggle state feedback ON/OFF")
    print("    J <deg>       Jog arm by <deg> degrees")
    print("    T             Toggle alpha debug output")
    print("─" * 60)
    print()

    try:
        while True:
            cmd = input().strip()
            if cmd.lower() == 'q':
                print("\nShutting down...")
                break
            if cmd:
                ser.write((cmd + '\n').encode())
                print(f"→ Sent: {cmd}")
                write_event(f"User command: {cmd}")

    except KeyboardInterrupt:
        print("\n\nInterrupted by user (Ctrl+C)")

    # Cleanup
    stop_event.set()
    t.join(timeout=1)
    ser.close()
    f.close()
    
    print()
    print("=" * 60)
    print("SESSION COMPLETE")
    print("=" * 60)
    print(f"CSV log:   {csv_path}")
    print(f"Events:    {events_path}")
    print(f"Directory: {session_dir}")
    print("=" * 60)

if __name__ == '__main__':
    main()
