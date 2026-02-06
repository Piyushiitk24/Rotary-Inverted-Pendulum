from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Optional

from .types import (
    CtrlEvent,
    DbgEvent,
    EngageEvent,
    EndEvent,
    Events,
    MoveEvent,
    ParamSnapshot,
    StatusEvent,
    UserCommand,
)


def _to_float_or_str(value: str) -> Any:
    try:
        return float(value)
    except ValueError:
        return value


def _normalize_mode(raw: Any) -> Optional[str]:
    if raw is None:
        return None
    s = str(raw).strip().upper()
    if s in {"LIN", "LINEAR"}:
        return "LIN"
    if s in {"SMC"}:
        return "SMC"
    if s in {"SMC4", "SMC_FULL", "SMC4FULL"}:
        return "SMC4"
    return s or None


def _parse_kv_tokens(text: str) -> dict[str, Any]:
    cleaned = (
        text.replace("|", " ")
        .replace(",", " ")
        .replace("(", " ")
        .replace(")", " ")
        .replace("[", " ")
        .replace("]", " ")
    )
    out: dict[str, Any] = {}
    for tok in cleaned.split():
        if "=" not in tok:
            continue
        k, v = tok.split("=", 1)
        if not k:
            continue
        out[k.strip()] = _to_float_or_str(v.strip())
    return out


def _parse_status(msg: str, host_dt: datetime, host_s: float) -> Optional[StatusEvent]:
    # Example:
    # [STATUS] alpha=-0.03 alphaDot=0.0 | theta=-3.17 thetaDot=19.7 | acc=-20 vel=-11 | ... mode=SMC s=-0.4
    kv = _parse_kv_tokens(msg)
    required = ["alpha", "alphaDot", "theta", "thetaDot", "acc", "vel"]
    if any(k not in kv for k in required):
        return None

    try:
        alpha = float(kv["alpha"])
        alpha_dot = float(kv["alphaDot"])
        theta = float(kv["theta"])
        theta_dot = float(kv["thetaDot"])
        acc = float(kv["acc"])
        vel = float(kv["vel"])
    except Exception:
        return None

    mode = _normalize_mode(kv.get("mode"))
    extra = {k: v for k, v in kv.items() if k not in {"alpha", "alphaDot", "theta", "thetaDot", "acc", "vel", "mode"}}

    return StatusEvent(
        host_dt=host_dt,
        host_s=host_s,
        raw_line=msg,
        alpha_deg=alpha,
        alpha_dot_deg_s=alpha_dot,
        theta_deg=theta,
        theta_dot_deg_s=theta_dot,
        acc_cmd_steps_s2=acc,
        vel_cmd_steps_s=vel,
        mode=mode,
        extra=extra,
    )


def _parse_dbg(msg: str) -> dict[str, Any]:
    # Example:
    # [DBG] dtUsMin=5000 dtUsMax=5040 over=0 sat=0 | run%=0 start=0 stop=0 rev=0 | leak%=0 trim%=100 ...
    cleaned = msg.replace("|", " ")
    out: dict[str, Any] = {}
    for tok in cleaned.split():
        if "=" not in tok:
            continue
        key, val = tok.split("=", 1)
        key = key.strip()
        val = val.strip()

        if key.endswith("%"):
            key = key[:-1] + "_pct"

        # Handle paired counters like glitchA/T=0/0, derivA/T=0/0, i2cA/T=0/0, stallE/T=0/0
        if "/" in key and "/" in val and key.endswith(("A/T", "E/T")):
            base, suffix = key.split("/", 1)  # e.g. base="glitchA", suffix="T"
            if base and suffix and len(val.split("/")) == 2:
                left, right = val.split("/", 1)
                base_prefix = base[:-1]  # remove A or E -> "glitch"
                left_key = base_prefix + base[-1]  # "glitchA" or "stallE"
                right_key = base_prefix + suffix  # "glitchT" or "stallT"
                out[left_key] = _to_float_or_str(left)
                out[right_key] = _to_float_or_str(right)
                continue

        out[key] = _to_float_or_str(val)
    return out


def parse_events(session_dir: str | Path) -> Events:
    session_dir = Path(session_dir)
    events_path = session_dir / "events.txt"
    if not events_path.exists():
        raise FileNotFoundError(f"Missing events.txt: {events_path}")

    events = Events(session_dir=session_dir)

    for raw in events_path.read_text(errors="ignore").splitlines():
        line = raw.strip()
        if not line:
            continue

        # Format: "{host_dt}: Device: ..." or "{host_dt}: User command: ..."
        if ": " not in line:
            continue
        ts_str, rest = line.split(": ", 1)
        try:
            host_dt = datetime.fromisoformat(ts_str)
        except Exception:
            continue

        if events.start_dt is None:
            events.start_dt = host_dt
        host_s = (host_dt - events.start_dt).total_seconds()

        src = None
        msg = rest
        is_auto = False
        if rest.startswith("Device: "):
            src = "Device"
            msg = rest[len("Device: ") :]
        elif rest.startswith("Marker: "):
            # Logger-only marker (not sent to device). We store it in commands with cmd starting '#'.
            src = "Marker"
            msg = rest[len("Marker: ") :]
        elif rest.startswith("User command (auto): "):
            src = "User"
            is_auto = True
            msg = rest[len("User command (auto): ") :]
        elif rest.startswith("User command: "):
            src = "User"
            msg = rest[len("User command: ") :]
        else:
            # Unknown prefix; keep as device message
            src = "Other"

        if src in {"User", "Marker"}:
            events.commands.append(
                UserCommand(
                    host_dt=host_dt,
                    host_s=host_s,
                    raw_line=msg,
                    cmd=("# " + msg.strip()) if src == "Marker" else msg.strip(),
                    is_auto=is_auto,
                )
            )
            continue

        # Device messages
        msg = msg.strip()

        if msg.startswith("[CTRL]"):
            kv = _parse_kv_tokens(msg)
            mode = _normalize_mode(kv.get("mode"))
            if mode:
                events.ctrl.append(CtrlEvent(host_dt=host_dt, host_s=host_s, raw_line=msg, mode=mode))
            continue

        if msg.startswith("ENGAGED!"):
            events.engaged.append(EngageEvent(host_dt=host_dt, host_s=host_s, raw_line=msg))
            continue

        if msg.startswith("DISARMED"):
            events.ended.append(EndEvent(host_dt=host_dt, host_s=host_s, raw_line=msg, kind="DISARMED"))
            continue

        # Firmware typically prints two lines on a fall:
        #   1) "FALLEN limit=... | ..."  (contains the reason/telemetry)
        #   2) "FALLEN (will auto-rearm when upright)"  (informational)
        # Only treat the first form as a trial-ending marker.
        if msg.startswith("FALLEN limit="):
            limit = None
            kv = _parse_kv_tokens(msg)
            if "limit" in kv:
                limit = str(kv["limit"])
            events.ended.append(
                EndEvent(host_dt=host_dt, host_s=host_s, raw_line=msg, kind="FALLEN", limit=limit)
            )
            continue

        if msg.startswith("[MOVE]"):
            kind = "PAUSED" if "PAUSED" in msg else ("RESUMED" if "RESUMED" in msg else "MOVE")
            events.move.append(MoveEvent(host_dt=host_dt, host_s=host_s, raw_line=msg, kind=kind))
            continue

        if msg.startswith("[STATUS]"):
            st = _parse_status(msg, host_dt, host_s)
            if st:
                events.status.append(st)
            continue

        if msg.startswith("[DBG]"):
            fields = _parse_dbg(msg)
            events.dbg.append(DbgEvent(host_dt=host_dt, host_s=host_s, raw_line=msg, fields=fields))
            continue

        # Parameter snapshots / key-value dumps (G output, tuning prints, etc.)
        if "=" in msg:
            values = _parse_kv_tokens(msg)
            if values:
                events.params.append(ParamSnapshot(host_dt=host_dt, host_s=host_s, raw_line=msg, values=values))

    events.sort_in_place()
    return events
