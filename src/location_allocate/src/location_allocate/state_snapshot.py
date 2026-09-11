"""Fresh, immutable swarm state snapshots for late task resolution."""
from __future__ import annotations

import math
from typing import Dict, Iterable, Optional, Sequence

from .lfs_types import StateSnapshot, UAVState, Vector3


class SnapshotError(RuntimeError):
    """Raised when a complete fresh snapshot cannot be created."""


def _vector3(value: Sequence[float], name: str) -> Vector3:
    if len(value) != 3:
        raise ValueError(f"{name} must contain exactly three values")
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{name} must contain finite values")
    return result  # type: ignore[return-value]


class FreshStateSnapshotManager:
    """Maintain timestamped states and produce all-or-nothing snapshots."""

    def __init__(
        self,
        state_timeout: float,
        snapshot_skew: float,
        *,
        require_velocity: bool = False,
        allow_receive_time_fallback: bool = True,
    ):
        if state_timeout <= 0.0 or snapshot_skew < 0.0:
            raise ValueError("invalid freshness configuration")
        self.state_timeout = float(state_timeout)
        self.snapshot_skew = float(snapshot_skew)
        self.require_velocity = bool(require_velocity)
        self.allow_receive_time_fallback = bool(allow_receive_time_fallback)
        self._states: Dict[int, UAVState] = {}

    def update(
        self,
        uav_id: int,
        position: Sequence[float],
        receive_timestamp: float,
        velocity: Optional[Sequence[float]] = None,
        source_timestamp: Optional[float] = None,
    ) -> None:
        if not math.isfinite(receive_timestamp):
            raise ValueError("receive_timestamp must be finite")
        warnings = []
        if source_timestamp is not None and (
            not math.isfinite(source_timestamp) or source_timestamp <= 0.0
        ):
            source_timestamp = None
        if source_timestamp is None:
            if not self.allow_receive_time_fallback:
                raise ValueError("valid source_timestamp is required")
            warnings.append("state timestamp used receive-time fallback")
        if self.require_velocity and velocity is None:
            raise ValueError("velocity is required")
        self._states[int(uav_id)] = UAVState(
            position=_vector3(position, "position"),
            velocity=None if velocity is None else _vector3(velocity, "velocity"),
            receive_timestamp=float(receive_timestamp),
            source_timestamp=(
                None if source_timestamp is None else float(source_timestamp)
            ),
            timestamp_source=(
                "source_timestamp"
                if source_timestamp is not None
                else "receive_time_fallback"
            ),
            warnings=tuple(warnings),
        )

    def snapshot(self, uav_ids: Iterable[int], now: float) -> StateSnapshot:
        ids = tuple(int(uid) for uid in uav_ids)
        if not ids or len(ids) != len(set(ids)):
            raise SnapshotError("snapshot needs unique participating UAV IDs")
        missing = sorted(uid for uid in ids if uid not in self._states)
        if missing:
            raise SnapshotError(f"missing UAV states: {missing}")
        selected = {uid: self._states[uid] for uid in ids}
        stale = sorted(
            uid
            for uid, state in selected.items()
            if now - state.effective_timestamp > self.state_timeout
            or state.effective_timestamp > now
        )
        if stale:
            raise SnapshotError(f"stale UAV states: {stale}")
        timestamps = [state.effective_timestamp for state in selected.values()]
        if max(timestamps) - min(timestamps) > self.snapshot_skew:
            raise SnapshotError("participating UAV states exceed snapshot_skew")
        warnings = tuple(
            warning
            for state in selected.values()
            for warning in state.warnings
        )
        return StateSnapshot(epoch=float(now), states=selected, warnings=warnings)
