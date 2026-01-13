#!/usr/bin/env python3
"""
Pendulum Clock Test Logger
Captures static angle readings for manual verification (0°, 90°, 180°, -90°).
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
    print("  Commands: Type 'Z' + Enter to zero sensor at 6 o'clock position")
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
    filename = f"pendulum_clock_{timestamp}.csv"
    filepath = os.path.join(LOG_DIR, filename)
    
    # Open serial
    ser = serial.Serial(port, BAUD, timeout=1)
    time.sleep(2)  # Wait for Arduino reset
    
    # Start keyboard input thread
    input_thread = threading.Thread(target=keyboard_input_thread, args=(ser,), daemon=True)
    input_thread.start()
    
    print(f"Logging to {filepath}...")
    print("Instructions:")
    print("  1. Hang pendulum at 6 o'clock (straight down)")
    print("  2. Type 'Z' + Enter to zero")
    print("  3. Move to 3 o'clock (90°), 12 o'clock (180°), 9 o'clock (-90°)")
    print("  4. Press Ctrl+C to stop")
    print("-" * 60)
    
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
                        print(line)
                        parts = line.split(',')
                        if len(parts) == 2:
                            writer.writerow(parts)
                            f.flush()  # Force write to disk
        except KeyboardInterrupt:
            print(f"\n\nSaved to {filepath}")
            ser.close()

if __name__ == '__main__':
    main()
