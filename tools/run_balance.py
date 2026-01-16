#!/usr/bin/env python3
import serial
import time
import csv
import os
from datetime import datetime
import threading

# --- CONFIGURATION ---
PORT = '/dev/cu.usbmodem212301'   # CHANGE IF NEEDED
BAUD = 500000
BASE_LOG_DIR = 'logs/balance'

# Expected numeric columns from firmware:
# time_ms, alphaDeg_x100, alphaDotFilt_x100, accCmd_steps_s2, thetaDotCmd_steps_s, sat
HEADER = [
    "timestamp_ms",
    "alphaDeg",
    "alphaDotFilt",
    "accCmd",
    "thetaDotCmd",
    "sat",
]

# --- SETUP SESSION ---
session_name = datetime.now().strftime("session_%Y%m%d_%H%M%S")
session_dir = os.path.join(BASE_LOG_DIR, session_name)
os.makedirs(session_dir, exist_ok=True)

csv_path = os.path.join(session_dir, 'telemetry.csv')
events_path = os.path.join(session_dir, 'events.txt')

print(f"--- BALANCING SESSION STARTED ---")
print(f"Logging to: {session_dir}/")
print("Firmware numeric format:")
print("  t_ms, alphaDeg_x100, alphaDotFilt_x100, accCmd, thetaDotCmd, sat")
print("Python will convert *_x100 back to real deg and deg/s when saving.\n")

stop_event = threading.Event()

def write_event(msg: str):
    with open(events_path, 'a') as ef:
        ef.write(f"{datetime.now()}: {msg}\n")

def read_serial(ser, writer):
    while not stop_event.is_set():
        try:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                # If it looks like a numeric CSV line, parse it
                if "," in line and not line.startswith("[") and not "FORMAT" in line:
                    parts = line.split(",")
                    if len(parts) == 6:
                        try:
                            t_ms = int(parts[0])

                            alphaDeg = int(parts[1]) / 100.0
                            alphaDotFilt = int(parts[2]) / 100.0

                            accCmd = int(parts[3])
                            thetaDotCmd = int(parts[4])

                            sat = int(parts[5])

                            writer.writerow([t_ms, alphaDeg, alphaDotFilt, accCmd, thetaDotCmd, sat])
                        except ValueError:
                            # Not purely numeric, treat as device text
                            print(f"[RIP]: {line}")
                    else:
                        # non-matching CSV, treat as device text
                        print(f"[RIP]: {line}")
                else:
                    # Device/status lines
                    print(f"[RIP]: {line}")

                    # Record gain changes if seen
                    if line.startswith("ACC_KP=") or line.startswith("ACC_KD=") or line.startswith("D_FILT_ALPHA="):
                        write_event(f"Device: {line}")

        except Exception as e:
            print(f"Serial Error: {e}")
            break

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
        time.sleep(2)  # Wait for Arduino reset
    except Exception as e:
        print(f"Could not open port {PORT}: {e}")
        return

    f = open(csv_path, 'w', newline='')
    writer = csv.writer(f)
    writer.writerow(HEADER)

    t = threading.Thread(target=read_serial, args=(ser, writer), daemon=True)
    t.start()

    print("\nCOMMANDS:")
    print("  Z       = Calibrate Zero (Hold Pendulum Down)")
    print("  P 742   = Set ACC_KP (steps/s^2 per deg)")
    print("  D 55    = Set ACC_KD (steps/s^2 per deg/s)")
    print("  F 0.2   = Set derivative LPF alpha")
    print("  Q       = Quit")
    print("-" * 40)

    try:
        while True:
            cmd = input().strip()
            if cmd.lower() == 'q':
                break
            if cmd:
                ser.write((cmd + '\n').encode())
                print(f"Sent: {cmd}")
                write_event(f"Sent: {cmd}")

    except KeyboardInterrupt:
        print("\nStopping...")

    stop_event.set()
    t.join(timeout=1)
    ser.close()
    f.close()
    print(f"Session saved to: {csv_path}")
    print(f"Events saved to:   {events_path}")

if __name__ == '__main__':
    main()
