#!/usr/bin/env python3
"""Balance logger for the Rotary Inverted Pendulum.

This utility opens the serial connection to the Arduino, mirrors the console
interaction so you can continue using the menu, and automatically captures each
balance attempt into a timestamped CSV file. The CSV rows contain the raw
telemetry emitted during balance along with the PID gains that were active for
that session.
"""

from __future__ import annotations

import argparse
import csv
import math
import signal
import sys
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable, List, Optional, Sequence

import serial

TELEMETRY_COLUMNS = (
    "time_s",
    "setpoint_deg",
    "pendulum_deg",
    "stepper_steps",
    "motor_deg",
    "control_output",
)

OUTPUT_COLUMNS = TELEMETRY_COLUMNS + (
    "kp_balance",
    "ki_balance",
    "kd_balance",
    "kp_motor",
    "origin",
    "reason",
    "session_id",
    "device_start_ms",
    "host_start_iso",
)


@dataclass
class BalanceSession:
    session_id: int
    origin: str
    device_start_ms: int
    host_start: datetime
    gains: dict[str, float] = field(default_factory=dict)
    rows: List[List[float]] = field(default_factory=list)
    reason: Optional[str] = None

    def host_timestamp(self) -> str:
        return self.host_start.astimezone(timezone.utc).isoformat()


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Capture balance telemetry to timestamped CSV files while still "
            "allowing interactive menu commands."
        )
    )
    parser.add_argument(
        "--port",
        required=True,
        help="Serial port of the Arduino (e.g. /dev/tty.usbmodem1101)",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Serial baud rate (default: 115200)",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("logs"),
        help="Directory where CSV logs will be written (default: logs)",
    )
    parser.add_argument(
        "--flush",
        action="store_true",
        help="Discard any buffered serial output immediately after connecting",
    )
    return parser.parse_args(list(argv) if argv is not None else None)


def open_serial(port: str, baud: int, *, flush_input: bool = False) -> serial.Serial:
    tried_ports: list[tuple[str, Exception]] = []
    candidate_ports: list[str] = [port]

    if port.startswith("/dev/tty."):
        candidate_ports.append("/dev/cu." + port[len("/dev/tty."):])

    last_exc: Optional[serial.SerialException] = None
    for candidate in candidate_ports:
        try:
            ser = serial.Serial(candidate, baudrate=baud, timeout=1)
            time.sleep(2.0)
            if flush_input:
                ser.reset_input_buffer()
            if candidate != port:
                print(f"[INFO] Opened {candidate} (auto fallback from {port})")
            return ser
        except serial.SerialException as exc:  # pragma: no cover - hardware specific
            tried_ports.append((candidate, exc))
            last_exc = exc

    if tried_ports:
        print("[DEBUG] Tried ports:")
        for path, exc in tried_ports:
            print(f"  - {path}: {exc}")
    raise last_exc if last_exc else serial.SerialException("Unable to open serial port")


def start_command_input_thread(ser: serial.Serial, stop_event: threading.Event) -> threading.Thread:
    def worker() -> None:
        print(
            "\n[INPUT] Type menu commands here (e.g. 'b', 's', 'x', 'P1.6')."
            " Type 'exit' or press Ctrl+C to quit.\n"
        )
        while not stop_event.is_set():
            try:
                line = input()
            except EOFError:
                break
            except KeyboardInterrupt:
                stop_event.set()
                break
            if stop_event.is_set():
                break
            if line is None:
                continue
            line = line.strip()
            if not line:
                continue
            if line.lower() in {"exit", "quit"}:
                stop_event.set()
                break
            try:
                ser.write((line + "\n").encode("utf-8"))
                ser.flush()
                print(f"[SENT] {line}")
            except serial.SerialException as exc:  # pragma: no cover - hardware specific
                print(f"[ERROR] Failed to send command: {exc}")
                stop_event.set()
                break

    thread = threading.Thread(target=worker, name="serial-input", daemon=True)
    thread.start()
    return thread


def request_initial_menu(ser: serial.Serial, *, delay: float = 0.25) -> None:
    """Send a benign command to prompt the firmware to reprint the menu."""

    try:
        time.sleep(delay)
        ser.write(b"X\n")
        ser.flush()
        print("[INFO] Requested menu refresh (sent 'X')")
    except serial.SerialException as exc:  # pragma: no cover - hardware specific
        print(f"[WARN] Failed to request menu refresh: {exc}")


def parse_telemetry(line: str) -> Optional[List[float]]:
    parts = line.split(",")
    if len(parts) != len(TELEMETRY_COLUMNS):
        return None
    values: List[float] = []
    for part in (p.strip() for p in parts):
        if part.lower() == "nan":
            values.append(float("nan"))
        else:
            try:
                values.append(float(part))
            except ValueError:
                return None
    return values


def normalise_name(component: str) -> str:
    safe = [c if c.isalnum() or c in ("-", "_") else "-" for c in component.strip()]
    return "".join(safe) or "unknown"


def write_session_csv(session: BalanceSession, output_dir: Path) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    timestamp = session.host_start.astimezone().strftime("%Y%m%d-%H%M%S")
    origin = normalise_name(session.origin)
    reason = normalise_name(session.reason or "incomplete")
    filename = f"balance_{timestamp}_{origin}_{reason}.csv"
    destination = output_dir / filename

    kp = session.gains.get("kp_balance", math.nan)
    ki = session.gains.get("ki_balance", math.nan)
    kd = session.gains.get("kd_balance", math.nan)
    km = session.gains.get("kp_motor", math.nan)
    host_iso = session.host_timestamp()

    with destination.open("w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(OUTPUT_COLUMNS)
        for row in session.rows:
            writer.writerow(
                [
                    *row,
                    kp,
                    ki,
                    kd,
                    km,
                    session.origin,
                    session.reason or "unknown",
                    session.session_id,
                    session.device_start_ms,
                    host_iso,
                ]
            )

    print(f"[SAVED] {destination} ({len(session.rows)} samples)")
    print(
        "         Gains – Kp: {0:.4f}, Ki: {1:.4f}, Kd: {2:.4f}, Km: {3:.4f}".format(
            kp if not math.isnan(kp) else float("nan"),
            ki if not math.isnan(ki) else float("nan"),
            kd if not math.isnan(kd) else float("nan"),
            km if not math.isnan(km) else float("nan"),
        )
    )
    return destination


def handle_log_start(payload: Iterable[str]) -> BalanceSession:
    parts = list(payload)
    if len(parts) < 3:
        raise ValueError("Unexpected BALANCE_LOG_START payload")
    session_id = int(parts[0])
    device_ms = int(parts[1])
    origin = parts[2]
    host_now = datetime.now().astimezone()
    print(
        f"\n[SESSION] #{session_id} origin={origin} device_ms={device_ms} host={host_now.isoformat()}"
    )
    return BalanceSession(
        session_id=session_id,
        origin=origin,
        device_start_ms=device_ms,
        host_start=host_now,
    )


def handle_gains(session: BalanceSession, payload: Iterable[str]) -> None:
    values = list(payload)
    if len(values) != 4:
        raise ValueError("Unexpected BALANCE_GAINS payload")
    keys = ("kp_balance", "ki_balance", "kd_balance", "kp_motor")
    for key, raw in zip(keys, values):
        try:
            session.gains[key] = float(raw)
        except ValueError:
            session.gains[key] = math.nan
    print(
        "[GAINS] Kp={0:.4f} Ki={1:.4f} Kd={2:.4f} Km={3:.4f}".format(
            session.gains.get("kp_balance", math.nan),
            session.gains.get("ki_balance", math.nan),
            session.gains.get("kd_balance", math.nan),
            session.gains.get("kp_motor", math.nan),
        )
    )


def handle_log_end(session: BalanceSession, payload: Iterable[str]) -> str:
    parts = list(payload)
    if len(parts) < 3:
        raise ValueError("Unexpected BALANCE_LOG_END payload")
    session_id = int(parts[0])
    device_ms = int(parts[1])
    reason = parts[2]
    print(f"[END]   #{session_id} reason={reason} device_ms={device_ms}")
    session.reason = reason
    return reason


def install_signal_handlers(stop_event: threading.Event) -> None:
    def handle_sigint(_signum, _frame):
        stop_event.set()

    signal.signal(signal.SIGINT, handle_sigint)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)

    try:
        ser = open_serial(args.port, args.baud, flush_input=args.flush)
    except serial.SerialException as exc:
        print(f"[ERROR] Could not open serial port {args.port}: {exc}", file=sys.stderr)
        return 1

    stop_event = threading.Event()
    install_signal_handlers(stop_event)
    input_thread = start_command_input_thread(ser, stop_event)
    request_initial_menu(ser)

    current_session: Optional[BalanceSession] = None

    try:
        while not stop_event.is_set():
            try:
                raw = ser.readline()
            except serial.SerialException as exc:  # pragma: no cover - hardware specific
                print(f"[ERROR] Serial read failed: {exc}")
                stop_event.set()
                break
            if not raw:
                continue
            try:
                line = raw.decode(errors="ignore").strip()
            except UnicodeDecodeError:
                continue
            if not line:
                continue

            if line.startswith("[BALANCE_LOG_START],"):
                payload = line.split(",")[1:]
                current_session = handle_log_start(payload)
                continue
            if line.startswith("[BALANCE_GAINS],"):
                if current_session is None:
                    print("[WARN] Received gains before log start; ignoring")
                else:
                    handle_gains(current_session, line.split(",")[1:])
                continue
            if line.startswith("[BALANCE_COLUMNS],"):
                print(line)
                continue
            if line.startswith("[BALANCE_LOG_END],"):
                if current_session is None:
                    print("[WARN] Received log end with no active session")
                    continue
                handle_log_end(current_session, line.split(",")[1:])
                if current_session.rows:
                    write_session_csv(current_session, args.output_dir)
                else:
                    print("[WARN] Session contained no telemetry samples; skipping save")
                current_session = None
                continue

            telemetry = parse_telemetry(line)
            if telemetry:
                if current_session is not None:
                    current_session.rows.append(telemetry)
                continue

            print(line)

    finally:
        stop_event.set()
        try:
            ser.close()
        except serial.SerialException:  # pragma: no cover - hardware specific
            pass
        if input_thread.is_alive():
            input_thread.join(timeout=0.5)

    return 0


def find_recent_logs(
    limit: Optional[int] = 5,
    directory: Optional[Path] = None,
    pattern: str = "balance_*.csv",
) -> List[Path]:
    """Return the most recent log files saved by this tool."""

    base_dir = Path(directory) if directory is not None else Path("logs")
    if not base_dir.is_absolute():
        base_dir = Path.cwd() / base_dir
    if not base_dir.exists():
        return []

    files = sorted(base_dir.glob(pattern))
    if limit is None or limit <= 0:
        return files
    return files[-limit:]


def load_log_dataframe(path: Path | str):
    """Load a log CSV as a pandas DataFrame (importing pandas on demand)."""

    try:
        import pandas as pd  # type: ignore
    except ImportError as exc:  # pragma: no cover - optional dependency
        raise RuntimeError(
            "pandas is required to load log CSV files. Install it in the analysis environment."
        ) from exc

    csv_path = Path(path)
    if not csv_path.exists():
        return pd.DataFrame(columns=OUTPUT_COLUMNS)
    return pd.read_csv(csv_path)


if __name__ == "__main__":
    raise SystemExit(main())
