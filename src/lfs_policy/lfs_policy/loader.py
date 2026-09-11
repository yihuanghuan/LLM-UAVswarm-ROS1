"""Fail-fast typed loading for Candidate production policies."""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
from typing import Any, Dict, Mapping, Tuple

import yaml


class PolicyLoadError(ValueError):
    pass


Vector3 = Tuple[float, float, float]


def _mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, dict):
        raise PolicyLoadError(f"{path} must be a mapping")
    return value


def _required(mapping: Mapping[str, Any], key: str, path: str) -> Any:
    if key not in mapping:
        raise PolicyLoadError(f"missing key: {path}.{key}")
    value = mapping[key]
    if value is None:
        raise PolicyLoadError(f"null is not allowed: {path}.{key}")
    return value


def _number(value: Any, path: str, *, minimum: float | None = None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise PolicyLoadError(f"{path} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise PolicyLoadError(f"{path} must be finite")
    if minimum is not None and result < minimum:
        raise PolicyLoadError(f"{path} must be >= {minimum}")
    return result


def _positive(value: Any, path: str) -> float:
    result = _number(value, path)
    if result <= 0.0:
        raise PolicyLoadError(f"{path} must be positive")
    return result


def _vector(value: Any, path: str) -> Vector3:
    if not isinstance(value, list) or len(value) != 3:
        raise PolicyLoadError(f"{path} must contain exactly three values")
    return tuple(_number(item, f"{path}[{index}]") for index, item in enumerate(value))


def _labels(mapping: Mapping[str, Any], path: str) -> Dict[str, float]:
    expected = {"smooth", "normal", "aggressive"}
    if set(mapping) != expected:
        raise PolicyLoadError(f"{path} must define exactly {sorted(expected)}")
    return {key: _positive(mapping[key], f"{path}.{key}") for key in expected}


@dataclass(frozen=True)
class StatePolicy:
    state_timeout: float
    snapshot_skew: float
    fresh_state_wait_timeout: float
    allow_receive_time_fallback: bool
    require_velocity: bool


@dataclass(frozen=True)
class ControllerHardClamps:
    baseline_omega_c: Vector3
    baseline_omega_o: Vector3
    smoothing_alpha: float
    omega_c_min: Vector3
    omega_c_max: Vector3
    omega_o_min: Vector3
    omega_o_max: Vector3
    velocity_max: float
    acceleration_max: float
    jerk_max: float
    iapf_violation_distance: float
    iapf_enter_min: float
    iapf_enter_max: float
    iapf_exit_max: float
    iapf_repulsion_max: float

    def ros_parameters(self) -> Dict[str, Any]:
        return {
            "enable_execution_profiles": True,
            "omega_c_x": self.baseline_omega_c[0],
            "omega_c_y": self.baseline_omega_c[1],
            "omega_c_z": self.baseline_omega_c[2],
            "omega_o_x": self.baseline_omega_o[0],
            "omega_o_y": self.baseline_omega_o[1],
            "omega_o_z": self.baseline_omega_o[2],
            "execution_profile_smoothing_alpha": self.smoothing_alpha,
            "execution_profile_omega_c_min": list(self.omega_c_min),
            "execution_profile_omega_c_max": list(self.omega_c_max),
            "execution_profile_omega_o_min": list(self.omega_o_min),
            "execution_profile_omega_o_max": list(self.omega_o_max),
            "execution_profile_velocity_max": self.velocity_max,
            "execution_profile_acceleration_max": self.acceleration_max,
            "execution_profile_jerk_max": self.jerk_max,
            # d_violation is a system-level hard threshold owned by the
            # loaded policy, not a task-level semantic preference.
            "iapf_violation_distance": self.iapf_violation_distance,
            "execution_profile_iapf_enter_min": self.iapf_enter_min,
            "execution_profile_iapf_enter_max": self.iapf_enter_max,
            "execution_profile_iapf_exit_max": self.iapf_exit_max,
            "execution_profile_iapf_repulsion_max": self.iapf_repulsion_max,
        }


@dataclass(frozen=True)
class LoadedPolicy:
    configuration_id: str
    status: str
    state: StatePolicy
    geometry: Mapping[str, Any]
    safety: Mapping[str, Any]
    motion_limits: Mapping[str, float]
    timing: Mapping[str, Any]
    allocator: Mapping[str, Any]
    execution_profile: Mapping[str, Any]
    controller: ControllerHardClamps
    provenance: Mapping[str, str]
    policy_hash: str
    parameter_status: Mapping[str, str]
    warnings: Tuple[str, ...]


def _validate_policy(
    data: Mapping[str, Any], production: bool, policy_hash: str,
    expected_use: str | None = None,
) -> LoadedPolicy:
    status = str(_required(data, "policy_status", "policy"))
    configuration_id = str(_required(data, "configuration_id", "policy")).strip()
    if not configuration_id:
        raise PolicyLoadError("configuration_id must not be empty")
    allowed = {"paper_current", "paper_frozen"}
    if production and status not in allowed:
        raise PolicyLoadError(f"unsupported production policy_status={status}")
    if expected_use == "paper" and status not in {"paper_current", "paper_frozen"}:
        raise PolicyLoadError("paper runtime requires a paper_current policy")
    state_raw = _mapping(_required(data, "state_snapshot", "policy"), "state_snapshot")
    fallback = _required(state_raw, "allow_receive_time_fallback", "state_snapshot")
    require_velocity = _required(state_raw, "require_velocity", "state_snapshot")
    if not isinstance(fallback, bool) or not isinstance(require_velocity, bool):
        raise PolicyLoadError("state fallback/velocity flags must be boolean")
    state = StatePolicy(
        _positive(_required(state_raw, "state_timeout", "state_snapshot"), "state_snapshot.state_timeout"),
        _number(_required(state_raw, "snapshot_skew", "state_snapshot"), "state_snapshot.snapshot_skew", minimum=0.0),
        _positive(_required(state_raw, "fresh_state_wait_timeout", "state_snapshot"), "state_snapshot.fresh_state_wait_timeout"),
        fallback,
        require_velocity,
    )

    geometry = dict(_mapping(_required(data, "geometry", "policy"), "geometry"))
    bounds = _mapping(_required(geometry, "workspace_bounds", "geometry"), "geometry.workspace_bounds")
    lower = _vector(_required(bounds, "lower", "geometry.workspace_bounds"), "geometry.workspace_bounds.lower")
    upper = _vector(_required(bounds, "upper", "geometry.workspace_bounds"), "geometry.workspace_bounds.upper")
    if any(a >= b for a, b in zip(lower, upper)):
        raise PolicyLoadError("workspace lower must be strictly below upper")
    geometry["workspace_bounds"] = {"lower": lower, "upper": upper}
    geometry["nominal_spacing"] = _positive(_required(geometry, "nominal_spacing", "geometry"), "geometry.nominal_spacing")
    qualitative = _mapping(_required(geometry, "qualitative_multipliers", "geometry"), "geometry.qualitative_multipliers")
    if set(qualitative) != {"compact", "normal", "spacious"}:
        raise PolicyLoadError("qualitative multipliers must define compact/normal/spacious")
    geometry["qualitative_multipliers"] = {key: _positive(value, f"geometry.qualitative_multipliers.{key}") for key, value in qualitative.items()}

    safety = dict(_mapping(_required(data, "safety", "policy"), "safety"))
    mapping_type = _required(safety, "mapping_type", "safety")
    safety["mapping_type"] = mapping_type
    common_safety = (
        "d_hard", "d_plan_base", "iapf_enter_base", "iapf_exit_base",
        "s_min", "s_max",
    )
    for key in common_safety:
        safety[key] = _positive(_required(safety, key, "safety"), f"safety.{key}")
    repulsion_base = _required(safety, "iapf_repulsion_base", "safety")
    if repulsion_base is None:
        raise PolicyLoadError("missing key: safety.iapf_repulsion_base")
    safety["iapf_repulsion_base"] = _number(
        repulsion_base, "safety.iapf_repulsion_base", minimum=0.0
    )
    safety["iapf_repulsion_margin"] = _number(
        safety.get("iapf_repulsion_margin", 0.0),
        "safety.iapf_repulsion_margin", minimum=0.0,
    )
    if mapping_type == "hard_anchored_linear":
        d_plan_min = safety["d_hard"] + safety["s_min"] * (
            safety["d_plan_base"] - safety["d_hard"]
        )
        d_plan_max = safety["d_hard"] + safety["s_max"] * (
            safety["d_plan_base"] - safety["d_hard"]
        )
        enter_min = safety["d_hard"] + safety["s_min"] * (
            safety["iapf_enter_base"] - safety["d_hard"]
        )
        exit_min = safety["d_hard"] + safety["s_min"] * (
            safety["iapf_exit_base"] - safety["d_hard"]
        )
        max_enter = safety["d_hard"] + safety["s_max"] * (
            safety["iapf_enter_base"] - safety["d_hard"]
        )
        max_exit = safety["d_hard"] + safety["s_max"] * (
            safety["iapf_exit_base"] - safety["d_hard"]
        )
    else:
        raise PolicyLoadError("unsupported safety.mapping_type")
    if safety["s_min"] < 1.0 or safety["s_max"] < safety["s_min"]:
        raise PolicyLoadError("safety s range must satisfy 1 <= min <= max")
    repulsion_min = safety["iapf_repulsion_base"] + safety["iapf_repulsion_margin"] * (
        safety["s_min"] - 1.0
    )
    repulsion_max = safety["iapf_repulsion_base"] + safety["iapf_repulsion_margin"] * (
        safety["s_max"] - 1.0
    )
    if (
        not safety["d_hard"] <= d_plan_min <= d_plan_max
        or not safety["d_hard"] < enter_min < exit_min
        or not safety["d_hard"] < max_enter < max_exit
    ):
        raise PolicyLoadError("safety ordering or IAPF hysteresis is invalid")

    limits_raw = _mapping(
        _required(data, "motion_limits", "policy"), "motion_limits"
    )
    motion_limits = {
        key: _positive(
            _required(limits_raw, key, "motion_limits"),
            f"motion_limits.{key}",
        )
        for key in ("velocity", "acceleration", "jerk")
    }

    timing = dict(_mapping(_required(data, "timing", "policy"), "timing"))
    if _required(timing, "policy_type", "timing") != "minimum_jerk":
        raise PolicyLoadError("unsupported timing.policy_type")
    timing["minimum_duration"] = _positive(
        _required(timing, "minimum_duration", "timing"),
        "timing.minimum_duration",
    )
    timing["auto_style_factors"] = _labels(_mapping(_required(timing, "auto_style_factors", "timing"), "timing.auto_style_factors"), "timing.auto_style_factors")
    if _required(timing, "planning_distance_bound_type", "timing") != "max_pairwise":
        raise PolicyLoadError("unsupported planning distance bound")
    timing["final_recheck_tolerance"] = _number(_required(timing, "final_recheck_tolerance", "timing"), "timing.final_recheck_tolerance", minimum=0.0)

    allocator = dict(_mapping(_required(data, "allocator", "policy"), "allocator"))
    allocator_keys = ("sample_hz", "comparison_tolerance")
    for key in allocator_keys:
        allocator[key] = _positive(_required(allocator, key, "allocator"), f"allocator.{key}")
    if _required(allocator, "parallel_d_plan_aggregation", "allocator") != "max":
        raise PolicyLoadError("parallel_d_plan_aggregation must be max")

    profile = dict(_mapping(_required(data, "execution_profile", "policy"), "execution_profile"))
    if _required(profile, "task_adaptation_type", "execution_profile") != "identity":
        raise PolicyLoadError("paper-current baseline task adaptation must be identity")
    profile["baseline_omega_c"] = _vector(_required(profile, "baseline_omega_c", "execution_profile"), "execution_profile.baseline_omega_c")
    profile["baseline_omega_o"] = _vector(_required(profile, "baseline_omega_o", "execution_profile"), "execution_profile.baseline_omega_o")
    profile["style_gains"] = _labels(_mapping(_required(profile, "style_gains", "execution_profile"), "execution_profile.style_gains"), "execution_profile.style_gains")
    hard = _mapping(_required(data, "controller_hard_clamps", "policy"), "controller_hard_clamps")
    controller = ControllerHardClamps(
        baseline_omega_c=profile["baseline_omega_c"],
        baseline_omega_o=profile["baseline_omega_o"],
        smoothing_alpha=_number(
            _required(hard, "smoothing_alpha", "controller_hard_clamps"),
            "controller_hard_clamps.smoothing_alpha",
        ),
        omega_c_min=_vector(_required(hard, "omega_c_min", "controller_hard_clamps"), "controller_hard_clamps.omega_c_min"),
        omega_c_max=_vector(_required(hard, "omega_c_max", "controller_hard_clamps"), "controller_hard_clamps.omega_c_max"),
        omega_o_min=_vector(_required(hard, "omega_o_min", "controller_hard_clamps"), "controller_hard_clamps.omega_o_min"),
        omega_o_max=_vector(_required(hard, "omega_o_max", "controller_hard_clamps"), "controller_hard_clamps.omega_o_max"),
        velocity_max=_positive(_required(hard, "velocity_max", "controller_hard_clamps"), "controller_hard_clamps.velocity_max"),
        acceleration_max=_positive(_required(hard, "acceleration_max", "controller_hard_clamps"), "controller_hard_clamps.acceleration_max"),
        jerk_max=_positive(_required(hard, "jerk_max", "controller_hard_clamps"), "controller_hard_clamps.jerk_max"),
        iapf_violation_distance=safety["d_hard"],
        iapf_enter_min=_positive(_required(hard, "iapf_enter_min", "controller_hard_clamps"), "controller_hard_clamps.iapf_enter_min"),
        iapf_enter_max=_positive(_required(hard, "iapf_enter_max", "controller_hard_clamps"), "controller_hard_clamps.iapf_enter_max"),
        iapf_exit_max=_positive(_required(hard, "iapf_exit_max", "controller_hard_clamps"), "controller_hard_clamps.iapf_exit_max"),
        iapf_repulsion_max=_positive(_required(hard, "iapf_repulsion_max", "controller_hard_clamps"), "controller_hard_clamps.iapf_repulsion_max"),
    )
    if not 0.0 < controller.smoothing_alpha <= 1.0:
        raise PolicyLoadError("smoothing_alpha must be in (0, 1]")
    for low, high in zip(controller.omega_c_min, controller.omega_c_max):
        if low <= 0.0 or high < low:
            raise PolicyLoadError("invalid omega_c clamps")
    for low, high in zip(controller.omega_o_min, controller.omega_o_max):
        if low <= 0.0 or high < low:
            raise PolicyLoadError("invalid omega_o clamps")
    if any(
        value < low or value > high
        for value, low, high in zip(
            controller.baseline_omega_c,
            controller.omega_c_min,
            controller.omega_c_max,
        )
    ):
        raise PolicyLoadError("baseline_omega_c must be within controller clamps")
    if any(
        value < low or value > high
        for value, low, high in zip(
            controller.baseline_omega_o,
            controller.omega_o_min,
            controller.omega_o_max,
        )
    ):
        raise PolicyLoadError("baseline_omega_o must be within controller clamps")
    if not controller.iapf_violation_distance < controller.iapf_enter_min:
        raise PolicyLoadError("controller IAPF enter clamp must exceed d_hard")
    if (
        controller.iapf_enter_min > enter_min
        or controller.iapf_enter_max < max_enter
        or controller.iapf_exit_max < max_exit
        or controller.iapf_repulsion_max < repulsion_max
        or repulsion_min < 0.0
    ):
        raise PolicyLoadError("controller IAPF clamps do not cover safety mapping")
    if (
        controller.velocity_max < motion_limits["velocity"]
        or controller.acceleration_max < motion_limits["acceleration"]
        or controller.jerk_max < motion_limits["jerk"]
    ):
        raise PolicyLoadError("controller dynamics clamps do not cover motion limits")

    provenance_raw = _mapping(_required(data, "provenance", "policy"), "provenance")
    provenance = {str(key): str(value) for key, value in provenance_raw.items() if str(value).strip()}
    if not provenance:
        raise PolicyLoadError("provenance must not be empty")
    statuses_raw = data.get("parameter_status", {})
    parameter_status = {
        str(key): str(value) for key, value in _mapping(
            statuses_raw, "parameter_status"
        ).items()
    }
    if status.startswith("paper_"):
        required_status = {"architecture_rules", "physical_environment",
                           "algorithm_calibration", "semantic_controller"}
        if not required_status.issubset(parameter_status):
            raise PolicyLoadError(
                "paper policy parameter_status must cover all audit categories"
            )
        if timing["final_recheck_tolerance"] != 0.0:
            raise PolicyLoadError("paper final_recheck_tolerance must be zero")
        style_gains = profile["style_gains"]
        if not (
            style_gains["smooth"] < style_gains["normal"] == 1.0
            < style_gains["aggressive"]
        ):
            raise PolicyLoadError(
                "paper motion style gains must satisfy smooth < normal == 1 "
                "< aggressive"
            )
        auto_factors = timing["auto_style_factors"]
        if not (
            auto_factors["smooth"] >= auto_factors["normal"]
            >= auto_factors["aggressive"] >= 1.0
        ):
            raise PolicyLoadError(
                "paper auto timing factors must satisfy smooth >= normal >= "
                "aggressive >= 1"
            )
        for label, gain in style_gains.items():
            compiled_c = tuple(value * gain for value in profile["baseline_omega_c"])
            compiled_o = tuple(value * gain for value in profile["baseline_omega_o"])
            if any(
                value < low or value > high
                for value, low, high in zip(
                    compiled_c, controller.omega_c_min, controller.omega_c_max
                )
            ) or any(
                value < low or value > high
                for value, low, high in zip(
                    compiled_o, controller.omega_o_min, controller.omega_o_max
                )
            ):
                raise PolicyLoadError(
                    f"paper {label} execution profile must be within controller clamps"
                )
        if controller.smoothing_alpha != 1.0:
            raise PolicyLoadError("paper-current smoothing_alpha must be 1.0")
        if mapping_type != "hard_anchored_linear":
            raise PolicyLoadError("paper safety mapping must be hard_anchored_linear")
    qualitative_spacing = {
        label: max(
            geometry["nominal_spacing"] * multiplier,
            d_plan_min,
        )
        for label, multiplier in geometry["qualitative_multipliers"].items()
    }
    warnings = []
    if not (
        qualitative_spacing["compact"] < qualitative_spacing["normal"]
        < qualitative_spacing["spacious"]
    ):
        warnings.append(
            "qualitative scale ordering is safety-clamped at s=1: "
            f"{qualitative_spacing}"
        )
    return LoadedPolicy(
        configuration_id, status, state, geometry, safety, motion_limits,
        timing, allocator, profile, controller, provenance, policy_hash,
        parameter_status,
        tuple(warnings),
    )


def load_policy(
    path: str | Path, *, production: bool = True,
    expected_use: str | None = None,
) -> LoadedPolicy:
    policy_path = Path(path)
    if not policy_path.is_file():
        raise PolicyLoadError(f"policy file not found: {policy_path}")
    try:
        raw = policy_path.read_bytes()
        data = yaml.safe_load(raw.decode("utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise PolicyLoadError(f"failed to read policy: {exc}") from exc
    return _validate_policy(
        _mapping(data, "policy"), production, hashlib.sha256(raw).hexdigest(),
        expected_use,
    )


def load_paper_policy(path: str | Path) -> LoadedPolicy:
    return load_policy(path, production=True, expected_use="paper")
