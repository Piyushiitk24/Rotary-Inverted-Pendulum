from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Optional


@dataclass(frozen=True)
class StatusEvent:
    host_dt: datetime
    host_s: float
    raw_line: str

    alpha_deg: float
    alpha_dot_deg_s: float
    theta_deg: float
    theta_dot_deg_s: float
    acc_cmd_steps_s2: float
    vel_cmd_steps_s: float

    mode: Optional[str] = None  # "LIN" | "SMC" | "SMC4"
    extra: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class DbgEvent:
    host_dt: datetime
    host_s: float
    raw_line: str
    fields: dict[str, Any]


@dataclass(frozen=True)
class CtrlEvent:
    host_dt: datetime
    host_s: float
    raw_line: str
    mode: str  # "LIN" | "SMC" | "SMC4"


@dataclass(frozen=True)
class EngageEvent:
    host_dt: datetime
    host_s: float
    raw_line: str


@dataclass(frozen=True)
class EndEvent:
    host_dt: datetime
    host_s: float
    raw_line: str
    kind: str  # "FALLEN" | "DISARMED"
    limit: Optional[str] = None


@dataclass(frozen=True)
class MoveEvent:
    host_dt: datetime
    host_s: float
    raw_line: str
    kind: str  # "PAUSED" | "RESUMED"


@dataclass(frozen=True)
class UserCommand:
    host_dt: datetime
    host_s: float
    raw_line: str
    cmd: str
    is_auto: bool = False


@dataclass(frozen=True)
class ParamSnapshot:
    host_dt: datetime
    host_s: float
    raw_line: str
    values: dict[str, Any]


@dataclass
class Events:
    session_dir: Path
    start_dt: Optional[datetime] = None

    ctrl: list[CtrlEvent] = field(default_factory=list)
    engaged: list[EngageEvent] = field(default_factory=list)
    ended: list[EndEvent] = field(default_factory=list)
    status: list[StatusEvent] = field(default_factory=list)
    dbg: list[DbgEvent] = field(default_factory=list)
    move: list[MoveEvent] = field(default_factory=list)
    commands: list[UserCommand] = field(default_factory=list)
    params: list[ParamSnapshot] = field(default_factory=list)

    def sort_in_place(self) -> None:
        self.ctrl.sort(key=lambda e: e.host_dt)
        self.engaged.sort(key=lambda e: e.host_dt)
        self.ended.sort(key=lambda e: e.host_dt)
        self.status.sort(key=lambda e: e.host_dt)
        self.dbg.sort(key=lambda e: e.host_dt)
        self.move.sort(key=lambda e: e.host_dt)
        self.commands.sort(key=lambda e: e.host_dt)
        self.params.sort(key=lambda e: e.host_dt)


@dataclass(frozen=True)
class Match:
    status: StatusEvent
    csv_index: int
    csv_timestamp_ms: int
    error: float


@dataclass
class Alignment:
    reliable: bool
    n_matches: int
    slope_ms_per_s: float
    intercept_ms: float
    median_abs_residual_ms: float
    max_abs_residual_ms: float
    matches: list[Match] = field(default_factory=list)

    def host_s_to_device_ms(self, host_s: float) -> float:
        return self.slope_ms_per_s * host_s + self.intercept_ms


@dataclass
class Trial:
    session_dir: Path
    trial_index: int
    mode: str  # "LIN" | "SMC" | "SMC4" | "UNK"

    start_ms: int
    end_ms: int
    df: Any  # pandas.DataFrame (kept Any to avoid hard dependency here)

    engaged_event: Optional[EngageEvent] = None
    end_event: Optional[EndEvent] = None

    params: dict[str, Any] = field(default_factory=dict)
    diagnostics: dict[str, Any] = field(default_factory=dict)

