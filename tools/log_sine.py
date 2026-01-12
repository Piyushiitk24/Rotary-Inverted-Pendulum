#!/usr/bin/env python3
"""
Sine Wave Test Logger

Purpose: Capture CSV telemetry from sine wave position control test
Connects to Arduino Mega 2560 running sine_wave_position.cpp firmware

Features:
- Auto-detects Arduino serial port or uses specified port
- 500000 baud serial connection
- Creates timestamped CSV file in logs/sine/ directory
- Mirrors serial output to console
- Robust 5+ minute operation with proper error handling
- Graceful shutdown on Ctrl+C

Usage:
    python3 tools/log_sine.py                    # Auto-detect port
    python3 tools/log_sine.py --port /dev/ttyACM0  # Specify port

CSV Format: timestamp_ms,target_deg,sensor_deg,stepper_deg,error_deg
"""

import serial
import serial.tools.list_ports
import sys
import os
import time
from datetime import datetime
import argparse
import signal
import threading
import select

# Configuration
BAUD_RATE = 500000
TIMEOUT = 1.0
RECONNECT_DELAY = 2.0

# Global flag for graceful shutdown
running = True

def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    global running
    print("\n\n# Shutting down logger...")
    running = False

def keyboard_input_thread(ser):
    """Thread to handle keyboard input and send to serial"""
    global running
    while running:
        try:
            # Check if there's keyboard input available (with timeout)
            if sys.platform == 'win32':
                # Windows doesn't support select on stdin
                import msvcrt
                if msvcrt.kbhit():
                    char = msvcrt.getch().decode('utf-8', errors='ignore')
                    if ser and ser.is_open:
                        ser.write(char.encode('utf-8'))
                        ser.flush()
            else:
                # Unix/Linux/macOS
                rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                if rlist:
                    char = sys.stdin.read(1)
                    if ser and ser.is_open:
                        ser.write(char.encode('utf-8'))
                        ser.flush()
        except Exception as e:
            if running:  # Only print error if not shutting down
                print(f"# Input error: {e}")
            break

def find_arduino_port():
    """
    Auto-detect Arduino Mega 2560 serial port
    Returns port name or None if not found
    """
    ports = serial.tools.list_ports.comports()
    
    # Look for Arduino Mega patterns
    for port in ports:
        # Check for common Arduino identifiers
        if any(keyword in port.description.lower() for keyword in ['arduino', 'mega', 'ch340', 'ftdi']):
            return port.device
        # macOS specific
        if port.device.startswith('/dev/cu.usbmodem') or port.device.startswith('/dev/cu.usbserial'):
            return port.device
        # Linux specific
        if port.device.startswith('/dev/ttyACM') or port.device.startswith('/dev/ttyUSB'):
            return port.device
    
    return None

def create_log_directory():
    """
    Create logs/sine/ directory if it doesn't exist
    Returns the directory path
    """
    # Get script directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)
    log_dir = os.path.join(project_root, 'logs', 'sine')
    
    os.makedirs(log_dir, exist_ok=True)
    return log_dir

def create_log_filename(log_dir):
    """
    Create timestamped CSV filename
    Format: sine_YYYYMMDD-HHMMSS.csv
    """
    timestamp = datetime.now().strftime('%Y%m%d-%H%M%S')
    filename = f'sine_{timestamp}.csv'
    return os.path.join(log_dir, filename)

def connect_serial(port, baudrate=BAUD_RATE):
    """
    Connect to serial port with error handling
    Returns serial object or None on failure
    """
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=TIMEOUT,
            write_timeout=TIMEOUT
        )
        # Wait for Arduino to reset after serial connection
        time.sleep(2)
        return ser
    except serial.SerialException as e:
        print(f"# ERROR: Could not open serial port {port}: {e}")
        return None
    except Exception as e:
        print(f"# ERROR: Unexpected error connecting to {port}: {e}")
        return None

def is_csv_data_line(line):
    """
    Check if line is CSV data (not a comment)
    CSV lines start with a digit (timestamp)
    """
    line = line.strip()
    return len(line) > 0 and line[0].isdigit()

def main():
    global running
    
    # Set up argument parser
    parser = argparse.ArgumentParser(description='Log sine wave test data from Arduino')
    parser.add_argument('--port', type=str, help='Serial port (e.g., /dev/ttyACM0 or COM3)')
    parser.add_argument('--baud', type=int, default=BAUD_RATE, help=f'Baud rate (default: {BAUD_RATE})')
    parser.add_argument('--duration', type=int, help='Auto-stop after N seconds (optional)')
    args = parser.parse_args()
    
    # Set up signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    
    print("# ========================================")
    print("# Sine Wave Test Logger")
    print("# ========================================")
    
    # Determine serial port
    if args.port:
        port = args.port
        print(f"# Using specified port: {port}")
    else:
        print("# Auto-detecting Arduino port...")
        port = find_arduino_port()
        if port is None:
            print("# ERROR: Could not find Arduino. Please specify port with --port")
            print("# Available ports:")
            for p in serial.tools.list_ports.comports():
                print(f"#   {p.device}: {p.description}")
            sys.exit(1)
        print(f"# Detected Arduino on port: {port}")
    
    # Create log directory and file
    log_dir = create_log_directory()
    log_file = create_log_filename(log_dir)
    print(f"# Log file: {log_file}")
    print(f"# Baud rate: {args.baud}")
    print("# ========================================")
    print("# Waiting for connection...")
    
    # Connect to serial port
    ser = connect_serial(port, args.baud)
    if ser is None:
        sys.exit(1)
    
    print("# Connected successfully")
    print("# ========================================")
    print("# Use Arduino commands to control test:")
    print("#   E - Enable motor")
    print("#   Z - Zero position")
    print("#   S - Start sine wave test")
    print("#   X - Stop test")
    print("# Press Ctrl+C to quit logger")
    print("# ========================================")
    print()
    
    # Start keyboard input thread
    input_thread = threading.Thread(target=keyboard_input_thread, args=(ser,), daemon=True)
    input_thread.start()
    
    # Open log file
    try:
        with open(log_file, 'w') as f:
            f.write("# Sine Wave Position Control Test\n")
            f.write(f"# Port: {port}\n")
            f.write(f"# Start Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write("#\n")
            
            start_time = time.time()
            line_count = 0
            data_lines = 0
            last_heartbeat = time.time()
            
            # Main logging loop
            while running:
                # Check duration limit if specified
                if args.duration and (time.time() - start_time) > args.duration:
                    print(f"\n# Duration limit reached ({args.duration}s)")
                    break
                
                # Heartbeat every 30 seconds
                if time.time() - last_heartbeat > 30:
                    elapsed = time.time() - start_time
                    print(f"# [Heartbeat] Running for {elapsed:.1f}s, {data_lines} data lines logged")
                    last_heartbeat = time.time()
                
                try:
                    # Read line from serial
                    if ser.in_waiting > 0:
                        line = ser.readline().decode('utf-8', errors='replace').rstrip()
                        
                        if len(line) > 0:
                            line_count += 1
                            
                            # Write to file
                            f.write(line + '\n')
                            f.flush()  # Ensure data is written immediately
                            
                            # Print to console
                            print(line)
                            
                            # Track data lines
                            if is_csv_data_line(line):
                                data_lines += 1
                    else:
                        # Small delay to prevent busy waiting
                        time.sleep(0.001)
                        
                except serial.SerialException as e:
                    print(f"\n# ERROR: Serial connection lost: {e}")
                    print("# Attempting to reconnect...")
                    ser.close()
                    time.sleep(RECONNECT_DELAY)
                    
                    ser = connect_serial(port, args.baud)
                    if ser is None:
                        print("# Could not reconnect. Exiting.")
                        break
                    print("# Reconnected successfully")
                    
                except Exception as e:
                    print(f"\n# ERROR: Unexpected error: {e}")
                    break
            
            # Write footer
            end_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            duration = time.time() - start_time
            f.write("#\n")
            f.write(f"# End Time: {end_time}\n")
            f.write(f"# Duration: {duration:.2f} seconds\n")
            f.write(f"# Total Lines: {line_count}\n")
            f.write(f"# Data Lines: {data_lines}\n")
            
            print("\n# ========================================")
            print("# Logging Session Summary")
            print("# ========================================")
            print(f"# Log file: {log_file}")
            print(f"# Duration: {duration:.2f} seconds")
            print(f"# Total lines: {line_count}")
            print(f"# Data lines: {data_lines}")
            print("# ========================================")
            
    except IOError as e:
        print(f"# ERROR: Could not write to log file: {e}")
    finally:
        if ser and ser.is_open:
            ser.close()
            print("# Serial connection closed")

if __name__ == '__main__':
    main()
