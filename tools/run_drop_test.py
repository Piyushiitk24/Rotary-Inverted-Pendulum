#!/usr/bin/env python3
"""
Pendulum Drop Test Logger
Captures high-speed (200Hz) angle data during free oscillation.
"""

import serial
import time
import csv
import os
import sys
import select
import threading
from datetime import datetime

# === CONFIGURATION ===
DEFAULT_PORT = '/dev/cu.usbmodem212301'  # Change if needed
BAUD = 500000
LOG_DIR = 'logs/pendulum'

# === PORT DETECTION ===
def detect_port():
    """Auto-detect Arduino port on macOS/Linux"""
    import glob
    ports = glob.glob('/dev/cu.usbmodem*') + glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
    return ports[0] if ports else DEFAULT_PORT

# === BIDIRECTIONAL SERIAL ===
def keyboard_input_thread(ser):
    """Send keyboard input to Arduino (for 'Z' zero command)"""
    print("  Commands: Type 'Z' + Enter to zero sensor")
    while True:
        try:
            if sys.platform != 'win32':
                # Unix/macOS: use select
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    user_input = sys.stdin.readline().strip()
                    if user_input:
                        ser.write((user_input + '\n').encode())
            else:
                # Windows: blocking input
                import msvcrt
                if msvcrt.kbhit():
                    user_input = input()
                    ser.write((user_input + '\n').encode())
        except:
            break

# === MAIN ===
def main():
    # Auto-detect port
    port = detect_port()
    print(f"Connecting to {port} at {BAUD} baud...")
    
    # Create log directory
    os.makedirs(LOG_DIR, exist_ok=True)
    
    # Create timestamped filename
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    filename = f"pendulum_drop_{timestamp}.csv"
    filepath = os.path.join(LOG_DIR, filename)
    
    # Open serial
    ser = serial.Serial(port, BAUD, timeout=1)
    time.sleep(2)  # Wait for Arduino reset
    
    # Start keyboard input thread
    input_thread = threading.Thread(target=keyboard_input_thread, args=(ser,), daemon=True)
    input_thread.start()
    
    print(f"Logging to {filepath}...")
    print("Instructions:")
    print("  1. Hang pendulum straight down (6 o'clock)")
    print("  2. Type 'Z' + Enter to zero")
    print("  3. Lift pendulum to 90° (horizontal)")
    print("  4. Release and let it swing freely")
    print("  5. Press Ctrl+C after ~10 seconds")
    print("-" * 60)
    
    data_count = 0
    with open(filepath, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['timestamp_ms', 'angle_deg'])
        
        try:
            while True:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    if line.startswith('#'):
                        print(f"# {line[1:].strip()}")
                    elif ',' in line:
                        # Don't print every line (200Hz is too fast)
                        # Just count and show heartbeat
                        parts = line.split(',')
                        if len(parts) == 2:
                            writer.writerow(parts)
                            data_count += 1
                            if data_count % 100 == 0:  # Print every 0.5s
                                print(f"Logged {data_count} samples ({data_count/200:.1f}s)", end='\r')
                            f.flush()  # Force write to disk periodically
        except KeyboardInterrupt:
            print(f"\n\nSaved {data_count} samples to {filepath}")
            print(f"Duration: ~{data_count/200:.1f} seconds")
            ser.close()

if __name__ == '__main__':
    main()
