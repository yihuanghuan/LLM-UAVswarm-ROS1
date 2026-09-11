from types import SimpleNamespace

import pytest
import rospy  # noqa: F401

from location_allocate.location_allocate import execute_runtime_payload


def fake_node(mode, candidate=None, legacy=None):
    return SimpleNamespace(
        runtime_mode=mode,
        run_candidate_mission=candidate,
        run_mission=legacy,
    )


def test_candidate_is_default_style_dispatch_and_never_calls_legacy():
    called = []

    def candidate(_payload):
        called.append("candidate")
        raise RuntimeError("candidate failure")

    node = fake_node(
        "candidate_v2", candidate, lambda _payload: called.append("legacy")
    )
    with pytest.raises(RuntimeError, match="candidate failure"):
        execute_runtime_payload(node, {"mission": {}})

    assert called == ["candidate"]



def test_removed_runtime_is_rejected():
    node = fake_node("legacy_v1", lambda _: pytest.fail("must not dispatch"))
    with pytest.raises(ValueError, match="unsupported runtime mode"):
        execute_runtime_payload(node, {"task_sequences": []})
