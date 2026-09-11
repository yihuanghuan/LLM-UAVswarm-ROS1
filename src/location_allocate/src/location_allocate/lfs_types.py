"""Typed boundaries for the Candidate-to-Executable LFS pipeline."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple


Vector3 = Tuple[float, float, float]


@dataclass(frozen=True)
class UAVState:
    """One timestamped world-frame UAV state sample."""

    position: Vector3
    receive_timestamp: float
    velocity: Optional[Vector3] = None
    source_timestamp: Optional[float] = None
    timestamp_source: str = "receive_time"
    warnings: Tuple[str, ...] = ()

    @property
    def effective_timestamp(self) -> float:
        return (
            self.source_timestamp
            if self.source_timestamp is not None
            else self.receive_timestamp
        )


@dataclass(frozen=True)
class StateSnapshot:
    """Immutable, freshness-checked state used by one task or parallel group."""

    epoch: float
    states: Mapping[int, UAVState]
    warnings: Tuple[str, ...] = ()

    def positions(self, uav_ids: Sequence[int]) -> List[Vector3]:
        return [self.states[int(uid)].position for uid in uav_ids]


@dataclass
class ResolutionTrace:
    """Audit data kept outside the formal executable LFS tuple."""

    task_id: int
    candidate_lfs: Dict[str, Any]
    snapshot_epoch: Optional[float] = None
    state_timestamps: Dict[int, Dict[str, Any]] = field(default_factory=dict)
    center_source: Optional[str] = None
    resolved_center: Optional[Vector3] = None
    unit_geometry: Optional[str] = None
    delta_min: Optional[float] = None
    r_nominal: Optional[float] = None
    r_safe: Optional[float] = None
    r_exec: Optional[float] = None
    safety_factor: Optional[float] = None
    d_hard: Optional[float] = None
    d_plan: Optional[float] = None
    iapf_enter_distance: Optional[float] = None
    iapf_exit_distance: Optional[float] = None
    iapf_repulsion_scale: Optional[float] = None
    t_request: Any = None
    t_plan: Optional[float] = None
    t_exec: Optional[float] = None
    configuration_id: Optional[str] = None
    policy_hash: Optional[str] = None
    code_git_sha: Optional[str] = None
    schema_version: Optional[str] = None
    schema_hash: Optional[str] = None
    geometry_version: Optional[str] = None
    allocator_mode: Optional[str] = None
    allocator_version: Optional[str] = None
    assignment_mode: Optional[str] = None
    hungarian_initial_assignment: List[int] = field(default_factory=list)
    final_assignment: List[int] = field(default_factory=list)
    planning_assignment_metrics: Dict[str, Any] = field(default_factory=dict)
    final_assignment_metrics: Dict[str, Any] = field(default_factory=dict)
    per_uav_dynamics: List[Dict[str, Any]] = field(default_factory=list)
    corrections: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)
    rejection_reason: Optional[str] = None


@dataclass(frozen=True)
class ResolvedTaskIntent:
    """Late-resolved task data that precedes target allocation."""

    task_id: int
    uav_ids: Tuple[int, ...]
    formation: Mapping[str, Any]
    center: Vector3
    radius_request: Mapping[str, Any]
    time_request: Mapping[str, Any]
    motion_style: str
    safety_factor: float
    trigger_semantics: Mapping[str, Any]


@dataclass(frozen=True)
class UnitGeometry:
    """Formation shape independent of center and scale."""

    formation: Mapping[str, Any]
    offsets: Tuple[Vector3, ...]
    delta_min: float
    geometry_version: str


@dataclass(frozen=True)
class ExecutableLFS:
    """The formal executable tuple tau=(U,F,c,r,T,m,s,q)."""

    uav_ids: Tuple[int, ...]
    formation: Mapping[str, Any]
    center: Vector3
    radius: float
    duration: float
    motion_style: str
    safety_factor: float
    trigger_semantics: Mapping[str, Any]

    def as_dict(self) -> Dict[str, Any]:
        return {
            "U": list(self.uav_ids),
            "F": dict(self.formation),
            "c": list(self.center),
            "r": self.radius,
            "T": self.duration,
            "m": self.motion_style,
            "s": self.safety_factor,
            "q": dict(self.trigger_semantics),
        }


@dataclass(frozen=True)
class ExecutionProfile:
    """Central compiler output consumed by a per-UAV controller."""

    duration: float
    style: str
    omega_c: Vector3
    omega_o: Vector3
    velocity_limit: float
    acceleration_limit: float
    jerk_limit: float
    iapf_enter_distance: float
    iapf_exit_distance: float
    iapf_repulsion_scale: float
    configuration_id: str
    style_gain: float
    task_gain: float


@dataclass(frozen=True)
class WaitSpec:
    """Explicit graph-level wait behavior compiled from candidate semantics."""

    condition: str
    duration: Optional[float] = None


@dataclass(frozen=True)
class CompiledTaskNode:
    task: Dict[str, Any]
    completion_event: str
    wait: Optional[WaitSpec]


@dataclass(frozen=True)
class CompiledParallelGroup:
    tasks: Tuple[CompiledTaskNode, ...]
    completion_mode: str


@dataclass(frozen=True)
class CompiledMission:
    nodes: Tuple[Any, ...]


@dataclass(frozen=True)
class TaskStateMachine:
    """Per-task runtime FSM compiled from graph semantics, not Candidate q."""

    task: Dict[str, Any]
    states: Tuple[str, ...]
    completion_event: str
    post_completion_wait: Optional[WaitSpec]
