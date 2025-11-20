#!/usr/bin/env python3
"""
Rotary Inverted Pendulum - Balance Logger & Controller
======================================================
Connects to the Arduino firmware, mirrors the menu interface, and 
automatically captures balance sessions into CSV files.
"""

import argparse
import csv
import queue
import sys
import threading
import time
from dataclasses import dataclass, field, asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import List, Optional, Dict, Any

import serial
import serial.tools.list_ports

# --- Configuration & Constants ---

DEFAULT_BAUD = 115200
DEFAULT_LOG_DIR = Path("logs")

# Protocol Tags (Must match C++ Firmware)
TAG_START = "[BALANCE_LOG_START]"
TAG_GAINS = "[BALANCE_GAINS]"
TAG_END   = "[BALANCE_LOG_END]"
TAG_COLS  = "[BALANCE_COLUMNS]"

# CSV Column Definitions
TELEMETRY_COLS = [
    "time_s", "setpoint_deg", "pendulum_deg", "base_deg", 
    "pendulum_vel", "base_vel", "control_output"
]

METADATA_COLS = [
    "kp_balance", "ki_balance", "kd_balance", "kp_motor",
    "origin", "reason", "session_id", "device_start_ms", "host_timestamp"
]

@dataclass
class BalanceSession:
    """Represents a single balance attempt."""
    session_id: int
    origin: str
    device_start_ms: int
    host_start: datetime = field(default_factory=lambda: datetime.now(timezone.utc))
    gains: Dict[str, float] = field(default_factory=dict)
    rows: List[List[float]] = field(default_factory=list)
    reason: Optional[str] = None

    def save_to_csv(self, output_dir: Path) -> Path:
        """Writes session data to a labeled CSV file."""
        output_dir.mkdir(parents=True, exist_ok=True)
        
        # Format filename: balance_YYYYMMDD-HHMMSS_origin_reason.csv
        ts_str = self.host_start.astimezone().strftime("%Y%m%d-%H%M%S")
        safe_origin = "".join([c if c.isalnum() else "-" for c in self.origin])
        safe_reason = "".join([c if c.isalnum() else "-" for c in (self.reason or "incomplete")])
        filename = f"balance_{ts_str}_{safe_origin}_{safe_reason}.csv"
        filepath = output_dir / filename

        # Prepare metadata values for every row
        meta_values = [
            self.gains.get("kp_balance", float("nan")),
            self.gains.get("ki_balance", float("nan")),
            self.gains.get("kd_balance", float("nan")),
            self.gains.get("kp_motor", float("nan")),
            self.origin,
            self.reason or "unknown",
            self.session_id,
            self.device_start_ms,
            self.host_start.isoformat()
        ]

        with filepath.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(TELEMETRY_COLS + METADATA_COLS)
            for row in self.rows:
                writer.writerow(row + meta_values)
        
        return filepath

class SerialMonitor:
    def __init__(self, port: str, baud: int, output_dir: Path, flush: bool = False):
        self.port = port
        self.baud = baud
        self.output_dir = output_dir
        self.flush_on_connect = flush
        
        self.ser: Optional[serial.Serial] = None
        self.running = False
        self.current_session: Optional[BalanceSession] = None
        self.input_queue: queue.Queue[str] = queue.Queue()

    def connect(self):
        """Attempts to open the serial connection."""
        print(f"[SYSTEM] Connecting to {self.port} @ {self.baud}...")
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1.0)
            time.sleep(2.0) # Allow Arduino reset
            if self.flush_on_connect:
                self.ser.reset_input_buffer()
            print("[SYSTEM] Connected. Requesting menu...")
            self.send_command("X") # Trigger menu refresh
        except serial.SerialException as e:
            print(f"[ERROR] Failed to connect: {e}")
            sys.exit(1)

    def send_command(self, cmd: str):
        """Queues a command to be sent to the device."""
        if self.ser and self.ser.is_open:
            try:
                full_cmd = f"{cmd}\n".encode('utf-8')
                self.ser.write(full_cmd)
                self.ser.flush()
                print(f"[CMD] > {cmd}")
            except serial.SerialException as e:
                print(f"[ERROR] Write failed: {e}")

    def _parse_line(self, line: str):
        """Decides if a line is protocol data, telemetry, or menu text."""
        line = line.strip()
        if not line:
            return

        # 1. Protocol: Start Session
        if line.startswith(TAG_START):
            parts = line.split(",")[1:]
            if len(parts) >= 3:
                self.current_session = BalanceSession(
                    session_id=int(parts[0]),
                    device_start_ms=int(parts[1]),
                    origin=parts[2]
                )
                print(f"\n[LOG] Session #{parts[0]} Started ({parts[2]})")
            return

        # 2. Protocol: Gains
        if line.startswith(TAG_GAINS):
            if self.current_session:
                parts = line.split(",")[1:]
                keys = ["kp_balance", "ki_balance", "kd_balance", "kp_motor"]
                for k, v in zip(keys, parts):
                    try:
                        self.current_session.gains[k] = float(v)
                    except ValueError:
                        pass
                print(f"[LOG] Gains Captured: {self.current_session.gains}")
            return

        # 3. Protocol: End Session
        if line.startswith(TAG_END):
            if self.current_session:
                parts = line.split(",")[1:]
                self.current_session.reason = parts[2] if len(parts) > 2 else "unknown"
                
                path = self.current_session.save_to_csv(self.output_dir)
                count = len(self.current_session.rows)
                print(f"[LOG] Session Ended ({self.current_session.reason}).")
                print(f"[LOG] Saved {count} samples to: {path.name}\n")
                self.current_session = None
            return

        # 4. Ignore Column Headers in stream
        if line.startswith(TAG_COLS):
            return

        # 5. Telemetry (Heuristic: Comma separated numbers)
        if "," in line:
            try:
                # Try parsing as list of floats
                parts = [float(x) for x in line.split(",")]
                if len(parts) == len(TELEMETRY_COLS):
                    if self.current_session:
                        self.current_session.rows.append(parts)
                    return # It was telemetry, don't print to console
            except ValueError:
                pass # Not telemetry, just text containing commas

        # 6. Default: Print to console (Menu text, debug info)
        print(f"[DEV] {line}")

    def _input_worker(self):
        """Thread to handle user keyboard input."""
        print("\n[UI] Controls: S(tart), X(Stop), P/I/D/M (Gains). 'q' to quit.\n")
        while self.running:
            try:
                # Non-blocking input approach not trivial in cross-platform Python.
                # Using standard input() which blocks this thread.
                cmd = input()
                if cmd.lower() in ['q', 'quit', 'exit']:
                    self.running = False
                    break
                self.input_queue.put(cmd)
            except (EOFError, KeyboardInterrupt):
                self.running = False
                break

    def run(self):
        """Main loop."""
        self.connect()
        self.running = True

        # Start input thread
        t_input = threading.Thread(target=self._input_worker, daemon=True)
        t_input.start()

        try:
            while self.running:
                # 1. Process Input Queue
                while not self.input_queue.empty():
                    cmd = self.input_queue.get()
                    self.send_command(cmd)

                # 2. Read Serial
                if self.ser.in_waiting:
                    try:
                        # Read line, decode, ignore errors
                        raw = self.ser.readline()
                        line = raw.decode('utf-8', errors='replace')
                        self._parse_line(line)
                    except serial.SerialException:
                        print("[ERROR] Serial disconnected.")
                        self.running = False
                else:
                    time.sleep(0.005) # Prevent CPU spin

        except KeyboardInterrupt:
            print("\n[SYSTEM] Stopping...")
        finally:
            self.running = False
            if self.ser:
                self.ser.close()
            print("[SYSTEM] Closed.")

def auto_detect_port() -> str:
    """Helper to find Arduino ports automatically."""
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        # Common keywords for Arduino devices
        if "usbmodem" in p.device or "USB Serial" in p.description or "Arduino" in p.description:
            return p.device
    return ""

# --- Entry Point ---

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Rotary Pendulum Balance Logger")
    parser.add_argument("--port", help="Serial port (e.g. COM3, /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Baud rate")
    parser.add_argument("--out", type=Path, default=DEFAULT_LOG_DIR, help="Output directory")
    args = parser.parse_args()

    target_port = args.port or auto_detect_port()
    
    if not target_port:
        print("[ERROR] No port specified and none found automatically.")
        print("Available ports:")
        for p in serial.tools.list_ports.comports():
            print(f" - {p.device} ({p.description})")
        sys.exit(1)

    monitor = SerialMonitor(target_port, args.baud, args.out)
    monitor.run()