"""Construct existing Candidate pipeline policies from one loaded YAML."""
from __future__ import annotations

import math
from pathlib import Path

from lfs_policy import LoadedPolicy, load_paper_policy

from .execution_profile_compiler import (
    ExecutionProfilePolicy,
    SoftSafetyParameters,
)
from .formation_geometry import ScalePolicy
from .late_resolution import LateResolutionPolicy, SafetyResolution
from .motion_limits import MotionLimits
from .safety_aware_allocator import SafetyAwareTopologyAllocator
from .timing_resolution import (
    ConfiguredMinimumJerkTimingPolicy,
    max_pairwise_distance_bound,
)
from .prompt_loader import load_paper_prompt_bundle
from .reproducibility import code_git_sha


def build_late_resolution_policy(config: LoadedPolicy) -> LateResolutionPolicy:
    geometry = config.geometry
    safety = config.safety
    timing = config.timing
    allocator = config.allocator
    profile = config.execution_profile
    controller = config.controller
    motion_limits = MotionLimits(**config.motion_limits)

    scale_policy = ScalePolicy(
        nominal_spacing=geometry["nominal_spacing"],
        qualitative_multipliers=geometry["qualitative_multipliers"],
        workspace_bounds=(
            geometry["workspace_bounds"]["lower"],
            geometry["workspace_bounds"]["upper"],
        ),
        configuration_id=config.configuration_id,
    )
    timing_policy = ConfiguredMinimumJerkTimingPolicy(
        motion_limits=motion_limits,
        minimum_duration=timing["minimum_duration"],
        auto_style_factors=timing["auto_style_factors"],
        configuration_id=config.configuration_id,
    )
    # Derive the compiler's scalar gain envelope from the controller's
    # per-axis hard clamps. The semantic policy selects a value inside this
    # envelope; the envelope itself remains only an abnormal-profile guard.
    minimum_gain = max(
        *(
            low / baseline
            for low, baseline in zip(
                controller.omega_c_min, profile["baseline_omega_c"]
            )
        ),
        *(
            low / baseline
            for low, baseline in zip(
                controller.omega_o_min, profile["baseline_omega_o"]
            )
        ),
    )
    maximum_gain = min(
        *(
            high / baseline
            for high, baseline in zip(
                controller.omega_c_max, profile["baseline_omega_c"]
            )
        ),
        *(
            high / baseline
            for high, baseline in zip(
                controller.omega_o_max, profile["baseline_omega_o"]
            )
        ),
    )
    profile_policy = ExecutionProfilePolicy(
        base_omega_c=profile["baseline_omega_c"],
        base_omega_o=profile["baseline_omega_o"],
        style_gains=profile["style_gains"],
        task_adaptation_type=profile["task_adaptation_type"],
        task_reference_speed=None,
        task_gain_intercept=None,
        task_gain_slope=None,
        task_gain_range=None,
        total_gain_range=(minimum_gain, maximum_gain),
        motion_limits=motion_limits,
        configuration_id=config.configuration_id,
    )

    def resolve_safety(s_value: float) -> SafetyResolution:
        if not math.isfinite(s_value):
            raise ValueError("s must be finite")
        if not safety["s_min"] <= s_value <= safety["s_max"]:
            raise ValueError(
                f"s={s_value} outside configured range "
                f"[{safety['s_min']}, {safety['s_max']}]"
            )
        def scaled(baseline):
            return safety["d_hard"] + s_value * (
                safety[baseline] - safety["d_hard"]
            )
        d_plan = scaled("d_plan_base")
        enter = scaled("iapf_enter_base")
        exit_distance = scaled("iapf_exit_base")
        repulsion_scale = safety["iapf_repulsion_base"] + safety[
            "iapf_repulsion_margin"
        ] * (s_value - 1.0)

        # The compiler owns task semantics. These clamps are the configured
        # safety envelope and should be no-ops for every validated policy.
        enter = max(controller.iapf_enter_min, min(controller.iapf_enter_max, enter))
        exit_distance = min(controller.iapf_exit_max, exit_distance)
        repulsion_scale = max(
            0.0, min(controller.iapf_repulsion_max, repulsion_scale)
        )
        resolution = SafetyResolution(
            d_hard=safety["d_hard"],
            d_plan=d_plan,
            soft_iapf=SoftSafetyParameters(
                enter_distance=enter,
                exit_distance=exit_distance,
                repulsion_scale=repulsion_scale,
            ),
        )
        resolution.validate()
        return resolution

    def allocator_factory(
        d_hard: float, d_plan: float
    ) -> SafetyAwareTopologyAllocator:
        return SafetyAwareTopologyAllocator(
            sample_hz=allocator["sample_hz"],
            d_hard=d_hard,
            d_plan=d_plan,
            comparison_tolerance=allocator["comparison_tolerance"],
        )

    prompt = load_paper_prompt_bundle()
    return LateResolutionPolicy(
        scale=scale_policy,
        timing=timing_policy,
        profile=profile_policy,
        resolve_safety=resolve_safety,
        planning_distance_bound=max_pairwise_distance_bound,
        timing_recheck_tolerance=timing["final_recheck_tolerance"],
        allocator_factory=allocator_factory,
        policy_hash=config.policy_hash,
        code_git_sha=code_git_sha(),
        schema_version=prompt.schema_version,
        schema_hash=prompt.schema_hash,
        allocator_mode="lexicographic-safety-aware-v2",
    )


def load_runtime_policy(path: str | Path):
    config = load_paper_policy(path)
    return config, build_late_resolution_policy(config)
