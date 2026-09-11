from pathlib import Path

import pytest

from location_allocate.execution_profile_compiler import (
    SoftSafetyParameters,
    compile_execution_profiles,
)
from location_allocate.lfs_types import ExecutableLFS
from location_allocate.policy_adapter import load_runtime_policy


PAPER_CURRENT = (
    Path(__file__).parents[2]
    / "lfs_policy"
    / "config"
    / "lfs_policy.paper_current.yaml"
)


def test_paper_policy_constructs_all_candidate_runtime_dependencies():
    config, policy = load_runtime_policy(PAPER_CURRENT)

    assert config.configuration_id == "paper-current-v11-c0-f-frozen"
    assert config.status == "paper_frozen"
    assert config.state.state_timeout == pytest.approx(0.022080)
    assert config.state.snapshot_skew == pytest.approx(0.022043)
    assert config.state.fresh_state_wait_timeout == pytest.approx(0.010000)
    assert config.parameter_status["c0_a_motion_ladrc"] == "frozen"
    assert config.parameter_status["c0_b_state_freshness"] == "frozen"
    assert config.parameter_status["c0_c_geometry_scale"] == "frozen"
    assert len(config.policy_hash) == 64
    assert policy.scale.nominal_spacing == 2.25
    assert policy.timing.motion_limits.jerk == 10.0
    assert policy.timing.motion_limits is policy.profile.motion_limits
    assert policy.profile.task_adaptation_type == "identity"
    assert policy.profile.total_gain_range == (0.75, 1.25)
    assert policy.resolve_safety(1.0).d_plan == 1.8
    assert policy.resolve_safety(2.0).soft_iapf.exit_distance == pytest.approx(1.9)
    allocator = policy.allocator_factory(1.0, 2.4)
    assert allocator.d_hard == 1.0
    assert allocator.d_plan == 2.4


def test_paper_safety_factor_has_explicit_policy_boundary():
    _config, policy = load_runtime_policy(PAPER_CURRENT)

    with pytest.raises(ValueError, match="outside configured range"):
        policy.resolve_safety(2.01)
    with pytest.raises(ValueError, match="finite"):
        policy.resolve_safety(float("nan"))


def test_safety_factor_compiles_one_monotonic_cross_layer_profile():
    config, policy = load_runtime_policy(PAPER_CURRENT)
    normal = policy.resolve_safety(1.0)
    safer = policy.resolve_safety(1.5)

    assert normal.d_hard == safer.d_hard == 1.5
    assert (normal.d_plan, normal.soft_iapf.enter_distance,
            normal.soft_iapf.exit_distance,
            normal.soft_iapf.repulsion_scale) == pytest.approx(
        (1.8, 1.6, 1.7, 1.0)
    )
    assert (safer.d_plan, safer.soft_iapf.enter_distance,
            safer.soft_iapf.exit_distance,
            safer.soft_iapf.repulsion_scale) == pytest.approx(
        (1.95, 1.65, 1.8, 1.125)
    )
    assert normal.d_plan < safer.d_plan
    assert normal.soft_iapf.enter_distance < safer.soft_iapf.enter_distance
    assert normal.soft_iapf.exit_distance < safer.soft_iapf.exit_distance
    assert normal.soft_iapf.repulsion_scale < safer.soft_iapf.repulsion_scale
    for resolved in (normal, safer, policy.resolve_safety(2.0)):
        resolved.validate()
        assert resolved.soft_iapf.enter_distance <= config.controller.iapf_enter_max
        assert resolved.soft_iapf.exit_distance <= config.controller.iapf_exit_max
        assert resolved.soft_iapf.repulsion_scale <= config.controller.iapf_repulsion_max


def test_current_qualitative_audit_preserves_c0c_scale_separation():
    config, _policy = load_runtime_policy(PAPER_CURRENT)

    assert config.warnings == ()


def test_normal_identity_profile_compiles_exact_canonical_ladrc_baseline():
    config, policy = load_runtime_policy(PAPER_CURRENT)
    executable = ExecutableLFS(
        uav_ids=(1,), formation={"type": "Line"}, center=(1.0, 0.0, 1.5),
        radius=1.0, duration=5.0, motion_style="normal",
        safety_factor=1.0, trigger_semantics={"mode": "direct"},
    )
    profile = compile_execution_profiles(
        executable, [(0.0, 0.0, 1.5)], [(1.0, 0.0, 1.5)],
        policy.profile, SoftSafetyParameters(1.5, 1.65, 1.0),
    )[0]

    assert profile.style_gain == 1.0
    assert profile.task_gain == 1.0
    assert profile.omega_c == config.controller.baseline_omega_c
    assert profile.omega_o == config.controller.baseline_omega_o


def test_current_style_profiles_change_bandwidth_but_not_task_gain():
    config, policy = load_runtime_policy(PAPER_CURRENT)
    profiles = {}
    for style in ("smooth", "normal", "aggressive"):
        item = ExecutableLFS(
            uav_ids=(1,), formation={"type": "Line"},
            center=(1.0, 0.0, 1.5), radius=1.0, duration=5.0,
            motion_style=style, safety_factor=1.0,
            trigger_semantics={"mode": "direct"},
        )
        profiles[style] = compile_execution_profiles(
            item, [(0.0, 0.0, 1.5)], [(1.0, 0.0, 1.5)],
            policy.profile, SoftSafetyParameters(1.5, 1.65, 1.0),
        )[0]

    for axis in range(3):
        assert (
            profiles["smooth"].omega_c[axis]
            < profiles["normal"].omega_c[axis]
            < profiles["aggressive"].omega_c[axis]
        )
        assert (
            profiles["smooth"].omega_o[axis]
            < profiles["normal"].omega_o[axis]
            < profiles["aggressive"].omega_o[axis]
        )
        assert all(
            config.controller.omega_c_min[axis]
            <= item.omega_c[axis]
            <= config.controller.omega_c_max[axis]
            for item in profiles.values()
        )
        assert all(
            config.controller.omega_o_min[axis]
            <= item.omega_o[axis]
            <= config.controller.omega_o_max[axis]
            for item in profiles.values()
        )
    assert {item.task_gain for item in profiles.values()} == {1.0}
