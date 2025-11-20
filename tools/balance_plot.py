#!/usr/bin/env python3

"""
Rotary Inverted Pendulum - Balance Logger & Controller
======================================================
Connects to Arduino, mirrors menu interface, and automatically captures 
balance sessions into timestamped CSV files in the `tools/` folder.
"""

import argparse
import csv
import queue
import sys
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import List, Optional, Dict

import serial
import serial.tools.list_ports

# --- Configuration & Constants ---
DEFAULT_BAUD = 115200
DEFAULT_LOG_DIR = Path("tools")  # Changed to tools folder as requested

# Protocol Tags (Must match C++ Firmware)
TAG_START = "[BALANCE_LOG_START]"
TAG_GAINS = "[BALANCE_GAINS]"
TAG_END = "[BALANCE_LOG_END]"
TAG_COLS = "[BALANCE_COLUMNS]"

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
        """Writes session data to a labeled CSV file with atomic write."""
        output_dir.mkdir(parents=True, exist_ok=True)
        
        # Format filename: balance_YYYYMMDD-HHMMSS_origin_reason.csv
        ts_str = self.host_start.astimezone().strftime("%Y%m%d-%H%M%S")
        safe_origin = "".join([c if c.isalnum() else "-" for c in self.origin])
        safe_reason = "".join([c if c.isalnum() else "-" for c in (self.reason or "incomplete")])
        filename = f"balance_{ts_str}_{safe_origin}_{safe_reason}.csv"
        filepath = output_dir / filename
        
        # DEBUG: Print what we're trying to save
        print(f"[DEBUG] Saving {len(self.rows)} rows to {filepath}")
        
        # Prepare metadata values
        meta_values_raw = [
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
        
        # Sanitize metadata
        def _sanitize_value(v):
            if v is None: return ""
            if isinstance(v, (float, int)): return v
            s = str(v).replace("\n", " ").replace("\r", " ")
            return s
        
        meta_values = [_sanitize_value(v) for v in meta_values_raw]
        
        # Write to CSV atomically
        tmp_path = filepath.with_suffix(".tmp")
        try:
            with tmp_path.open("w", newline="") as f:
                writer = csv.writer(f, quoting=csv.QUOTE_MINIMAL)
                writer.writerow(TELEMETRY_COLS + METADATA_COLS)
                
                # Write telemetry rows
                for row in self.rows:
                    # Ensure row has correct length
                    if len(row) < len(TELEMETRY_COLS):
                        row = row + [0.0] * (len(TELEMETRY_COLS) - len(row))
                    elif len(row) > len(TELEMETRY_COLS):
                        row = row[:len(TELEMETRY_COLS)]
                    
                    writer.writerow(row + meta_values)
            
            # Atomic rename
            tmp_path.replace(filepath)
            print(f"[LOG] Successfully saved: {filepath.resolve()}")
            return filepath
            
        except Exception as e:
            print(f"[ERROR] Failed to save CSV: {e}")
            if tmp_path.exists():
                tmp_path.unlink()
            raise
        finally:
            if tmp_path.exists():
                try: tmp_path.unlink()
                except: pass

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
        self.last_line_time: float = 0.0
        self.AUTO_SAVE_TIMEOUT = 2.0
        self.next_session_id = 1
        
    def connect(self):
        """Attempts to open the serial connection."""
        print(f"[SYSTEM] Connecting to {self.port} @ {self.baud}...")
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1.0)
            time.sleep(2.0)  # Allow Arduino reset
            if self.flush_on_connect:
                self.ser.reset_input_buffer()
            print("[SYSTEM] Connected successfully!")
            
            # Create tools folder if it doesn't exist
            self.output_dir.mkdir(parents=True, exist_ok=True)
            print(f"[SYSTEM] Log directory: {self.output_dir.resolve()}")
            
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
        
        # Record receipt time
        self.last_line_time = time.time()
        
        # DEBUG: Show raw lines (comment out if too noisy)
        # print(f"[RAW] {line}")
        
        # 1. Protocol: Start Session
        if TAG_START in line:
            tail = line.replace(TAG_START, "").lstrip(",")
            parts = tail.split(",") if tail else []
            
            session_id = int(parts[0]) if len(parts) > 0 and parts[0].isdigit() else 0
            device_start_ms = int(parts[1]) if len(parts) > 1 and parts[1].isdigit() else 0
            origin = parts[2] if len(parts) > 2 else "unknown"
            
            self.current_session = BalanceSession(
                session_id=session_id,
                device_start_ms=device_start_ms,
                origin=origin
            )
            print(f"\n[LOG] Session #{session_id} Started ({origin})")
            return
        
        # 2. Protocol: Gains
        if line.startswith(TAG_GAINS):
            if self.current_session:
                parts = line.split(",")[1:]
                keys = ["kp_balance", "ki_balance", "kd_balance", "kp_motor"]
                for k, v in zip(keys, parts):
                    try:
                        self.current_session.gains[k] = float(v)
                    except (ValueError, IndexError):
                        pass
                print(f"[LOG] Gains Captured: {self.current_session.gains}")
            return
        
        # 3. Protocol: End Session
        if TAG_END in line:
            tail = line.replace(TAG_END, "").lstrip(",")
            parts = tail.split(",") if tail else []
            reason = parts[2] if len(parts) > 2 else "unknown"
            
            if not self.current_session:
                print("[WARN] Received END without active session - creating placeholder")
                self.current_session = BalanceSession(session_id=0, device_start_ms=0, origin=str(self.port))
            
            self.current_session.reason = reason
            self._save_session()
            return
        
        # 4. Ignore Column Headers
        if line.startswith(TAG_COLS):
            return
        
        # 5. Telemetry (Comma separated numbers)
        if "," in line and not any(tag in line for tag in [TAG_START, TAG_END, TAG_GAINS, TAG_COLS]):
            try:
                parts = [float(x) for x in line.split(",")]
                if len(parts) >= len(TELEMETRY_COLS):
                    if self.current_session:
                        self.current_session.rows.append(parts[:len(TELEMETRY_COLS)])
                    return  # Don't print telemetry to console
            except ValueError:
                pass  # Not telemetry, continue to print
        
        # 6. Auto-detect balance start/stop from firmware messages
        lower = line.lower()
        
        # Auto-create session on "Balance ON"
        if "balance on" in lower and not self.current_session:
            sid = self.next_session_id
            self.next_session_id += 1
            self.current_session = BalanceSession(session_id=sid, device_start_ms=0, origin=str(self.port))
            print(f"[LOG] Auto-created Session #{sid} (Balance ON)")
            return
        
        # Auto-save on pendulum fall
        if "pendulum fell" in lower and self.current_session:
            print(f"[AUTO] Pendulum fell - saving session #{self.current_session.session_id}")
            self.current_session.reason = "pendulum_fell"
            self._save_session()
            return
        
        # Default: Print to console
        print(f"[DEV] {line}")
    
    def _save_session(self):
        """Save current session and reset."""
        if not self.current_session:
            return
        
        try:
            if not self.current_session.rows:
                print("[WARN] No data to save - session empty")
                self.current_session = None
                return
                
            path = self.current_session.save_to_csv(self.output_dir)
            count = len(self.current_session.rows)
            print(f"[LOG] Session ended: {count} samples → {path.name}")
            print(f"[LOG] Full path: {path.resolve()}\n")
        except Exception as e:
            print(f"[ERROR] Failed to save session: {e}")
        finally:
            self.current_session = None
    
    def _input_worker(self):
        """Thread to handle user keyboard input."""
        print("\n[UI] Controls: S(tart), X(Stop), P/I/D/M (Gains). 'q' to quit.\n")
        while self.running:
            try:
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
                # Process Input Queue
                while not self.input_queue.empty():
                    cmd = self.input_queue.get()
                    self.send_command(cmd)
                
                # Read Serial
                if self.ser.in_waiting:
                    try:
                        raw = self.ser.readline()
                        line = raw.decode('utf-8', errors='replace').strip()
                        self._parse_line(line)
                    except serial.SerialException:
                        print("[ERROR] Serial disconnected.")
                        self.running = False
                else:
                    time.sleep(0.005)
                
                # Auto-save on timeout
                self._attempt_autosave()
                
        except KeyboardInterrupt:
            print("\n[SYSTEM] Stopping...")
        finally:
            self.running = False
            if self.ser:
                self.ser.close()
            print("[SYSTEM] Closed.")
    
    def _attempt_autosave(self):
        """Auto-save session if no data received for timeout period."""
        if not self.current_session or not self.current_session.rows:
            return
        
        if time.time() - self.last_line_time > self.AUTO_SAVE_TIMEOUT:
            print(f"[AUTO] Timeout - auto-saving session #{self.current_session.session_id}")
            self.current_session.reason = self.current_session.reason or "timeout"
            self._save_session()

def auto_detect_port() -> str:
    """Helper to find Arduino ports automatically."""
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
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
    
    print(f"[SYSTEM] Starting logger...")
    print(f"[SYSTEM] Port: {target_port}")
    print(f"[SYSTEM] Baud: {args.baud}")
    print(f"[SYSTEM] Output: {args.out.resolve()}")
    
    monitor = SerialMonitor(target_port, args.baud, args.out)
    monitor.run()
