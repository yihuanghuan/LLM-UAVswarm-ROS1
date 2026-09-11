import dataclasses
import pytest

from location_allocate.execution_profile_compiler import (
    ExecutionProfilePolicy,
    ProfileCompileError,
    SoftSafetyParameters,
    compile_execution_profiles,
)
from location_allocate.lfs_types import ExecutableLFS
from location_allocate.motion_limits import MotionLimits


def policy():
    return ExecutionProfilePolicy(
        base_omega_c=(3.0, 3.0, 3.5),
        base_omega_o=(10.0, 10.0, 15.0),
        style_gains={"smooth": 0.8, "normal": 1.0, "aggressive": 1.2},
        task_adaptation_type="linear_speed",
        task_reference_speed=2.0,
        task_gain_intercept=0.8,
        task_gain_slope=0.2,
        task_gain_range=(0.7, 1.3),
        total_gain_range=(0.5, 1.8),
        motion_limits=MotionLimits(2.0, 1.5, 3.0),
        configuration_id="test-only",
    )


def safety():
    return SoftSafetyParameters(
        enter_distance=1.5,
        exit_distance=1.8,
        repulsion_scale=1.0,
    )


def executable(style="normal", duration=5.0):
    return ExecutableLFS(
        uav_ids=(1, 2),
        formation="Line",
        center=(0.0, 0.0, 1.0),
        radius=2.0,
        duration=duration,
        motion_style=style,
        safety_factor=1.0,
        trigger_semantics="direct",
    )


@pytest.mark.parametrize("style", ["smooth", "normal", "aggressive"])
def test_profile_uses_executable_t_as_its_only_duration(style):
    profiles = compile_execution_profiles(
        executable(style=style, duration=7.25),
        [[0.0, 0.0, 1.0], [0.0, 2.0, 1.0]],
        [[2.0, 0.0, 1.0], [5.0, 2.0, 1.0]],
        policy(),
        safety(),
    )

    assert [profile.duration for profile in profiles] == [7.25, 7.25]
    assert profiles[0].style_gain == policy().style_gains[style]
    assert profiles[0].task_gain != profiles[1].task_gain


def test_profile_policy_has_no_hidden_default_values():
    fields = {field.name for field in dataclasses.fields(ExecutionProfilePolicy)}

    assert "task_gain_intercept" in fields
    assert "task_gain_slope" in fields
    assert all(
        field.default is dataclasses.MISSING
        for field in dataclasses.fields(ExecutionProfilePolicy)
    )


def test_identity_task_adaptation_never_invents_semantic_gain():
    identity = dataclasses.replace(
        policy(),
        style_gains={"smooth": 1.0, "normal": 1.0, "aggressive": 1.0},
        task_adaptation_type="identity",
        task_reference_speed=None,
        task_gain_intercept=None,
        task_gain_slope=None,
        task_gain_range=None,
        total_gain_range=(1.0, 1.0),
    )

    profiles = compile_execution_profiles(
        executable(style="aggressive", duration=12.0),
        [[0, 0, 0], [0, 1, 0]],
        [[1, 0, 0], [10, 1, 0]],
        identity,
        safety(),
    )

    assert {item.style_gain for item in profiles} == {1.0}
    assert {item.task_gain for item in profiles} == {1.0}
    assert {item.omega_c for item in profiles} == {(3.0, 3.0, 3.5)}


def test_profile_rejects_non_finite_or_inconsistent_values():
    bad_policy = dataclasses.replace(
        policy(), motion_limits=MotionLimits(float("nan"), 1.5, 3.0)
    )
    bad_safety = SoftSafetyParameters(2.0, 1.0, 1.0)

    with pytest.raises(ProfileCompileError, match="finite and positive"):
        compile_execution_profiles(
            executable(), [[0, 0, 0], [0, 1, 0]], [[1, 0, 0], [1, 1, 0]],
            bad_policy, safety()
        )
    with pytest.raises(ProfileCompileError, match="exit distance"):
        compile_execution_profiles(
            executable(), [[0, 0, 0], [0, 1, 0]], [[1, 0, 0], [1, 1, 0]],
            policy(), bad_safety
        )




def test_soft_safety_does_not_expose_hard_violation_threshold():
    assert "hard" not in " ".join(
        field.name for field in dataclasses.fields(SoftSafetyParameters)
    )


def test_soft_safety_accepts_zero_repulsion_and_rejects_negative_scale():
    SoftSafetyParameters(1.5, 1.8, 0.0).validate()
    with pytest.raises(ProfileCompileError, match="non-negative"):
        SoftSafetyParameters(1.5, 1.8, -0.1).validate()


def test_profile_compiler_fails_fast_when_final_t_is_dynamically_infeasible():
    with pytest.raises(ProfileCompileError, match="violates shared motion limits"):
        compile_execution_profiles(
            executable(duration=0.5),
            [[0, 0, 0], [0, 1, 0]],
            [[10, 0, 0], [10, 1, 0]],
            policy(),
            safety(),
        )
