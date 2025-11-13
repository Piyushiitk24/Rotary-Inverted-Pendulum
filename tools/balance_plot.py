#!/usr/bin/env python3
"""Live plotter for Rotary Inverted Pendulum balance telemetry.

Reads comma-separated telemetry emitted by `runBalance()` and displays
real-time plots for tuning PID gains. Optionally logs data to CSV for
offline analysis.
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from collections import deque
from typing import Deque, Optional, Sequence

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import serial

COLUMNS = (
    "time_s",
    "setpoint_deg",
    "pendulum_deg",
    "stepper_steps",
    "motor_deg",
    "control_output",
)


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot live balance telemetry from the rotary inverted pendulum."
    )
    parser.add_argument(
        "--port",
        required=True,
        help="Serial port of the Arduino (e.g. COM5 or /dev/tty.usbmodem1101)",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Serial baud rate (default: 115200)",
    )
    parser.add_argument(
        "--window",
        type=int,
        default=3000,
        help="Number of samples to retain in the sliding window (default: 3000)",
    )
    parser.add_argument(
        "--log",
        type=str,
        default=None,
        help="Optional path to write a CSV log of the captured data",
    )
    return parser.parse_args(list(argv) if argv is not None else None)


def open_serial(port: str, baud: int) -> serial.Serial:
    ser = serial.Serial(port, baudrate=baud, timeout=1)
    # Allow the Arduino to reboot after opening the port.
    time.sleep(2.0)
    # Flush any existing buffered data.
    ser.reset_input_buffer()
    return ser


def parse_line(line: str) -> Optional[tuple[float, ...]]:
    parts = line.strip().split(",")
    if len(parts) != len(COLUMNS):
        return None
    values = []
    for part in parts:
        part = part.strip()
        if part.lower() == "nan":
            values.append(float("nan"))
        else:
            try:
                values.append(float(part))
            except ValueError:
                return None
    return tuple(values)


def make_plot(window: int):
    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(11, 7))

    axes[0].set_title("Pendulum Angle vs Setpoint")
    axes[0].set_ylabel("Angle (deg)")

    axes[1].set_title("Stepper & Motor Feedback")
    axes[1].set_ylabel("Steps / Deg")
    axes[1].set_xlabel("Samples (most recent on the right)")

    buffers: dict[str, Deque[float]] = {
        name: deque(maxlen=window) for name in COLUMNS
    }

    lines = {
        "setpoint": axes[0].plot([], [], label="Setpoint", color="C1")[0],
        "pendulum": axes[0].plot([], [], label="Pendulum", color="C0")[0],
        "stepper": axes[1].plot([], [], label="Stepper Steps", color="C2")[0],
        "motor": axes[1].plot([], [], label="Motor Angle", color="C3")[0],
        "control": axes[1].plot([], [], label="Control Output", color="C4", linestyle="--")[0],
    }

    axes[0].legend(loc="upper right")
    axes[1].legend(loc="upper right")

    return fig, axes, buffers, lines


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)

    try:
        ser = open_serial(args.port, args.baud)
    except serial.SerialException as exc:
        print(f"[ERROR] Could not open serial port {args.port}: {exc}", file=sys.stderr)
        return 1

    fig, axes, buffers, lines = make_plot(args.window)

    csv_writer = None
    csv_file = None
    if args.log:
        csv_file = open(args.log, "w", newline="")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(COLUMNS)
        print(f"[INFO] Logging data to {args.log}")

    def update(_frame: int):
        while ser.in_waiting:
            raw = ser.readline().decode(errors="ignore")
            parsed = parse_line(raw)
            if not parsed:
                continue

            for name, value in zip(COLUMNS, parsed):
                buffers[name].append(value)

            if csv_writer:
                csv_writer.writerow(parsed)

        # Build X axis using sample index within the window.
        x_vals = range(len(buffers["time_s"]))

        lines["setpoint"].set_data(x_vals, list(buffers["setpoint_deg"]))
        lines["pendulum"].set_data(x_vals, list(buffers["pendulum_deg"]))
        lines["stepper"].set_data(x_vals, list(buffers["stepper_steps"]))
        lines["motor"].set_data(x_vals, list(buffers["motor_deg"]))
        lines["control"].set_data(x_vals, list(buffers["control_output"]))

        for ax in axes:
            ax.relim()
            ax.autoscale_view()

        return tuple(lines.values())

    ani = FuncAnimation(fig, update, interval=100, blit=False)

    try:
        plt.show()
    finally:
        ser.close()
        if csv_file:
            csv_file.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
