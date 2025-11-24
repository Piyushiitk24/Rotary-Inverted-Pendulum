#!/usr/bin/env python3
"""
Rotary Inverted Pendulum - Balance Logger
=========================================

- Connects to the Arduino firmware.
- Mirrors the serial menu interface.
- Captures each balance attempt into a timestamped CSV in `logs/`.

Each CSV row contains:
    time_s, setpoint_deg, pendulum_deg, base_deg,
    pendulum_vel, base_vel, control_output,
plus metadata (Kp, Ki, Kd, Kp_motor, reason, etc.).
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

import re
import serial
import serial.tools.list_ports

# --------------------------------------------------------------------
# CONFIG
# --------------------------------------------------------------------

DEFAULT_BAUD = 500000
DEFAULT_LOG_DIR = Path("logs")

TAG_START = "[BALANCE_LOG_START]"
TAG_GAINS = "[BALANCE_GAINS]"
TAG_COLS  = "[BALANCE_COLUMNS]"
TAG_END   = "[BALANCE_LOG_END]"

TELEMETRY_COLS = [
    "time_s",
    "setpoint_deg",
    "pendulum_deg",
    "base_deg",
    "pendulum_vel",
    "base_vel",
    "control_output",  # firmware logs actual commanded Hz here
]

METADATA_COLS = [
    "kp_balance",
    "ki_balance",
    "kd_balance",
    "kp_motor",
    "origin",
    "reason",
    "session_id",
    "device_start_ms",
    "host_timestamp",
]


# --------------------------------------------------------------------
# DATA MODEL
# --------------------------------------------------------------------

@dataclass
class BalanceSession:
    session_id: int
    device_start_ms: float = 0.0
    origin: str = ""
    reason: str = ""
    gains: Dict[str, float] = field(default_factory=dict)
    rows: List[List[float]] = field(default_factory=list)
    host_start: datetime = field(default_factory=lambda: datetime.now(timezone.utc))

    def save_to_csv(self, output_dir: Path) -> Optional[Path]:
        if not self.rows:
            print("[WARN] No data to save - session empty")
            return None

        output_dir.mkdir(parents=True, exist_ok=True)

        ts_str = self.host_start.astimezone().strftime("%Y%m%d-%H%M%S")
        safe_origin = "".join(c if c.isalnum() else "-" for c in self.origin)
        safe_reason = "".join(
            c if c.isalnum() else "-" for c in (self.reason or "incomplete")
        )
        filename = f"balance_{ts_str}_{safe_origin}_{safe_reason}.csv"
        filepath = output_dir / filename

        print(f"[DEBUG] Saving {len(self.rows)} rows to {filepath}")

        meta_values_raw = [
            self.gains.get("kp_balance", float("nan")),
            self.gains.get("ki_balance", float("nan")),
            self.gains.get("kd_balance", float("nan")),
            self.gains.get("kp_motor", float("nan")),
            self.origin,
            self.reason,
            self.session_id,
            self.device_start_ms,
            self.host_start.isoformat(),
        ]

        with filepath.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(TELEMETRY_COLS + METADATA_COLS)

            for row in self.rows:
                if len(row) < len(TELEMETRY_COLS):
                    row = row + [float("nan")] * (len(TELEMETRY_COLS) - len(row))
                elif len(row) > len(TELEMETRY_COLS):
                    row = row[:len(TELEMETRY_COLS)]
                writer.writerow(row + meta_values_raw)

        print(f"[LOG] Successfully saved: {filepath}")
        return filepath


# --------------------------------------------------------------------
# SERIAL MONITOR
# --------------------------------------------------------------------

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

    # --------------------------------------------
    # Serial management
    # --------------------------------------------

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        except serial.SerialException as e:
            print(f"[ERROR] Could not open serial port {self.port}: {e}")
            sys.exit(1)

        print(f"[SYSTEM] Connected.")
        print(f"[SYSTEM] Log directory: {self.output_dir.resolve()}\n")

        if self.flush_on_connect:
            print("[SYSTEM] Flushing initial serial buffer...")
            time.sleep(1.0)
            self.ser.reset_input_buffer()

        print("[UI] Controls: S,X,P#,I#,D#,M#,Y,Z,T. 'q' to quit.\n")

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        print("[SYSTEM] Closed.")

    def send_command(self, cmd: str):
        if not self.ser or not self.ser.is_open:
            return
        to_send = (cmd.strip() + "\n").encode("utf-8")
        self.ser.write(to_send)
        print(f"[CMD] > {cmd}")

    # --------------------------------------------
    # Parsing
    # --------------------------------------------

    def _parse_line(self, line: str):
        line = line.strip()
        if not line:
            return

        self.last_line_time = time.time()
        # print(f"[RAW] {line}")  # uncomment if you want to debug raw output

        # 1. Explicit session start tag
        if TAG_START in line:
            tail = line.replace(TAG_START, "").lstrip(",")
            parts = tail.split(",") if tail else []

            sid = int(parts[0]) if len(parts) > 0 and parts[0].isdigit() else 0
            dev_ms = int(parts[1]) if len(parts) > 1 and parts[1].isdigit() else 0
            origin = parts[2] if len(parts) > 2 else str(self.port)

            self.current_session = BalanceSession(
                session_id=sid,
                device_start_ms=dev_ms,
                origin=origin,
            )
            print(f"\n[LOG] Session #{sid} Started ({origin})")
            return

        # 2. Gains line
        if line.startswith(TAG_GAINS):
            if self.current_session:
                parts = line.split(",")[1:]
                keys = ["kp_balance", "ki_balance", "kd_balance", "kp_motor"]
                for k, v in zip(keys, parts):
                    try:
                        self.current_session.gains[k] = float(v)
                    except Exception:
                        pass
                print(f"[LOG] Gains Captured: {self.current_session.gains}")
            return

        # 3. END tag
        if line.startswith(TAG_END):
            if self.current_session:
                parts = line.split(",")
                reason = parts[2] if len(parts) > 2 else "end"
                self.current_session.reason = reason
                print(f"[LOG] END tag received, reason={reason}")
                self._save_session()
            else:
                print("[WARN] END tag received but no active session.")
            return

        # 4. Column header line
        if line.startswith(TAG_COLS):
            print(f"[DEV] {line}")
            return

        # 5. Telemetry CSV (numeric CSV line)
        if "," in line and not any(
            tag in line for tag in [TAG_START, TAG_END, TAG_GAINS, TAG_COLS]
        ):
            try:
                nums = re.findall(r"[-+]?\d*\.\d+|[-+]?\d+", line)
                if len(nums) >= len(TELEMETRY_COLS):
                    vals = [float(x) for x in nums[: len(TELEMETRY_COLS)]]
                    if self.current_session:
                        self.current_session.rows.append(vals)
                    return  # don't echo telemetry
            except Exception:
                pass

        # 6. Plain-text device output
        print(f"[DEV] {line}")

    def _save_session(self):
        if not self.current_session:
            return

        try:
            if not self.current_session.rows:
                print("[WARN] No data to save - session empty")
                self.current_session = None
                return

            saved = self.current_session.save_to_csv(self.output_dir)
            if saved:
                print(
                    f"[LOG] Session #{self.current_session.session_id} "
                    f"saved to {saved.name}"
                )
        finally:
            self.current_session = None

    # --------------------------------------------
    # Input worker
    # --------------------------------------------

    def _input_worker(self):
        while self.running:
            try:
                cmd = input().strip()
                if cmd.lower() == "q":
                    self.running = False
                    break
                self.input_queue.put(cmd)
            except (EOFError, KeyboardInterrupt):
                self.running = False
                break

    # --------------------------------------------
    # Main loop
    # --------------------------------------------

    def run(self):
        self.connect()
        self.running = True

        t_input = threading.Thread(target=self._input_worker, daemon=True)
        t_input.start()

        try:
            while self.running:
                while not self.input_queue.empty():
                    cmd = self.input_queue.get()
                    self.send_command(cmd)

                if self.ser and self.ser.in_waiting:
                    try:
                        raw = self.ser.readline()
                        if raw:
                            line = raw.decode(errors="ignore")
                            self._parse_line(line)
                    except serial.SerialException as e:
                        print(f"[ERROR] Serial read error: {e}")
                        self.running = False
                        break

                self._attempt_autosave()

                time.sleep(0.01)

        finally:
            self.running = False
            t_input.join(timeout=0.1)
            if self.ser and self.ser.is_open:
                self.ser.close()
            print("[SYSTEM] Closed.")

    def _attempt_autosave(self):
        """Timeout-based save if firmware never sends END tag."""
        if not self.current_session or not self.current_session.rows:
            return

        # If we already know the reason, don't wait.
        if self.current_session.reason in ("pendulum_fell", "base_limit"):
            print(
                f"[AUTO] Immediate save on termination: "
                f"{self.current_session.reason}"
            )
            self._save_session()
            return

        if time.time() - self.last_line_time > self.AUTO_SAVE_TIMEOUT:
            print(
                f"[AUTO] Timeout - auto-saving session "
                f"#{self.current_session.session_id}"
            )
            self.current_session.reason = (
                self.current_session.reason or "timeout"
            )
            self._save_session()


def auto_detect_port() -> str:
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if "usbmodem" in p.device or "USB Serial" in p.description or "Arduino" in p.description:
            return p.device
    return ""


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Rotary Pendulum Balance Logger")
    parser.add_argument(
        "--port", type=str, default="", help="Serial port (if empty, auto-detect)"
    )
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument(
        "--out",
        type=Path,
        default=DEFAULT_LOG_DIR,
        help="Output directory (default: logs/)",
    )
    args = parser.parse_args()

    port = args.port or auto_detect_port()
    if not port:
        print("[ERROR] No port specified and none found automatically.")
        print("Available ports:")
        for p in serial.tools.list_ports.comports():
            print(f" - {p.device} ({p.description})")
        sys.exit(1)

    print(f"[SYSTEM] Starting logger on {port} @ {args.baud}")
    monitor = SerialMonitor(port, args.baud, args.out)
    monitor.run()
