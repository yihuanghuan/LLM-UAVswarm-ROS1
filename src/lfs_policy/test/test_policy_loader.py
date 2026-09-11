from pathlib import Path

import pytest
import yaml

from lfs_policy import PolicyLoadError, load_paper_policy, load_policy


TEMPLATE = Path(__file__).parents[2] / "location_allocate" / "config" / "lfs_policy.template.yaml"
PAPER = Path(__file__).parents[1] / "config" / "lfs_policy.paper_current.yaml"


def write_policy(tmp_path, mutate):
    data = yaml.safe_load(PAPER.read_text(encoding="utf-8"))
    mutate(data)
    path = tmp_path / "policy.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    return path




def test_template_is_not_a_production_policy():
    with pytest.raises(PolicyLoadError):
        load_policy(TEMPLATE)


def test_frozen_paper_policy_has_hash_status_and_c0f_freeze_status():
    policy = load_paper_policy(PAPER)

    assert policy.configuration_id == "paper-current-v11-c0-f-frozen"
    assert policy.status == "paper_frozen"
    assert policy.state.state_timeout == pytest.approx(0.02208)
    assert policy.state.snapshot_skew == pytest.approx(0.022043)
    assert policy.state.fresh_state_wait_timeout == pytest.approx(0.010)
    assert len(policy.policy_hash) == 64
    assert policy.parameter_status["architecture_rules"] == "paper-frozen"
    assert policy.parameter_status["c0_c_geometry_scale"] == "frozen"
    assert policy.parameter_status["c0_d_planning_safety"] == "frozen"
    assert policy.parameter_status["c0_e_iapf"] == "frozen"
    assert policy.parameter_status["c0_f_motion_style"] == "frozen"
    assert policy.warnings == ()
    parameters = policy.controller.ros_parameters()
    assert [parameters[f"omega_c_{axis}"] for axis in "xyz"] == [
        1.5, 1.5, 1.75
    ]
    assert [parameters[f"omega_o_{axis}"] for axis in "xyz"] == [
        5.0, 5.0, 7.5
    ]
    assert policy.execution_profile["style_gains"] == {
        "smooth": 0.8,
        "normal": 1.0,
        "aggressive": 1.1,
    }
    assert policy.timing["auto_style_factors"] == {
        "smooth": 1.3,
        "normal": 1.15,
        "aggressive": 1.1,
    }
    assert policy.controller.omega_c_min == (1.125, 1.125, 1.3125)
    assert policy.controller.omega_c_max == (1.875, 1.875, 2.1875)
    assert parameters["iapf_violation_distance"] == 1.5
    assert policy.safety["d_plan_base"] == 1.8
    assert policy.safety["iapf_enter_base"] == 1.6
    assert policy.safety["iapf_exit_base"] == 1.7
    assert policy.safety["iapf_repulsion_base"] == 1.0
    assert policy.safety["iapf_repulsion_margin"] == 0.25


@pytest.mark.parametrize("family", ["omega_c", "omega_o"])
def test_all_paper_style_profiles_are_ordered_and_inside_hard_clamps(family):
    policy = load_paper_policy(PAPER)
    baseline = policy.execution_profile[f"baseline_{family}"]
    gains = policy.execution_profile["style_gains"]
    compiled = {
        style: tuple(value * gains[style] for value in baseline)
        for style in ("smooth", "normal", "aggressive")
    }
    lower = getattr(policy.controller, f"{family}_min")
    upper = getattr(policy.controller, f"{family}_max")

    for axis in range(3):
        assert (
            compiled["smooth"][axis]
            < compiled["normal"][axis]
            < compiled["aggressive"][axis]
        )
        assert all(
            lower[axis] <= values[axis] <= upper[axis]
            for values in compiled.values()
        )


def test_paper_runtime_rejects_legacy_policy(tmp_path):
    with pytest.raises(PolicyLoadError):
        load_paper_policy(write_policy(tmp_path, lambda d: d.update(policy_status="legacy")))


@pytest.mark.parametrize(
    "mutate, message",
    [
        (lambda data: data.pop("configuration_id"), "missing key"),
        (lambda data: data["motion_limits"].update(jerk=None), "null"),
        (lambda data: data["motion_limits"].update(jerk=float("nan")), "finite"),
        (lambda data: data["safety"].update(iapf_exit_base=1.1), "hysteresis"),
        (lambda data: data["safety"].update(d_plan_base=0.9), "ordering"),
        (
            lambda data: data["safety"].update(iapf_repulsion_margin=-0.1),
            "must be >= 0",
        ),
        (
            lambda data: data["controller_hard_clamps"].update(
                iapf_repulsion_max=1.1
            ),
            "cover safety mapping",
        ),
        (lambda data: data["controller_hard_clamps"].update(iapf_enter_max=1.6), "cover"),
        (
            lambda data: data["controller_hard_clamps"].update(
                velocity_max=4.0
            ),
            "cover motion limits",
        ),
        (
            lambda data: data["execution_profile"].update(
                baseline_omega_c=[2.0, 1.5, 1.75]
            ),
            "baseline_omega_c",
        ),
        (
            lambda data: data["execution_profile"]["style_gains"].update(
                aggressive=1.0
            ),
            "smooth < normal",
        ),
        (
            lambda data: data["timing"]["auto_style_factors"].update(
                aggressive=0.9
            ),
            "aggressive >= 1",
        ),
        (
            lambda data: data["controller_hard_clamps"].update(
                omega_c_max=[1.6, 1.6, 1.9]
            ),
            "aggressive execution profile",
        ),
    ],
)
def test_invalid_production_policy_fails_fast(tmp_path, mutate, message):
    path = write_policy(tmp_path, mutate)

    with pytest.raises(PolicyLoadError, match=message):
        load_policy(path)
