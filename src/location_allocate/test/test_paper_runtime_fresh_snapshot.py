from types import SimpleNamespace

import pytest

from location_allocate.paper_runtime import PaperMissionRuntime
from location_allocate.state_snapshot import FreshStateSnapshotManager


class Clock:
    def __init__(self, timestamp=100.0):
        self.timestamp = timestamp

    def now(self):
        return SimpleNamespace(nanoseconds=int(self.timestamp * 1e9))


class Node:
    def __init__(self, clock):
        self.clock = clock

    def get_clock(self):
        return self.clock

    def get_parameter(self, name):
        values = {"candidate_dispatch_readiness_timeout": 0.02}
        return SimpleNamespace(value=values[name])


class Monotonic:
    def __init__(self):
        self.value = 0.0

    def __call__(self):
        return self.value


def runtime(manager, timeout=0.01, clock=None):
    return PaperMissionRuntime(
        Node(clock or Clock()),
        SimpleNamespace(state=SimpleNamespace(fresh_state_wait_timeout=timeout)),
        None, manager, {}, None, [1],
    )


def test_fresh_snapshot_is_returned_without_spinning(monkeypatch):
    manager = FreshStateSnapshotManager(0.1, 0.1)
    manager.update(1, [0, 0, 1], 100.0, source_timestamp=100.0)
    calls = []
    monkeypatch.setattr(
        "location_allocate.paper_runtime.ros_api.spin_once",
        lambda *_args, **_kwargs: calls.append(True),
    )

    assert runtime(manager)._fresh_snapshot([1]).states[1].position == (0.0, 0.0, 1.0)
    assert calls == []


def test_unavailable_snapshot_becomes_fresh_before_deadline(monkeypatch):
    manager = FreshStateSnapshotManager(0.1, 0.1)
    clock = Clock()
    monotonic = Monotonic()
    requested = []

    def spin(_node, timeout_sec):
        requested.append(timeout_sec)
        monotonic.value += 0.003
        manager.update(1, [0, 0, 1], 100.0, source_timestamp=100.0)

    monkeypatch.setattr("location_allocate.paper_runtime.time.monotonic", monotonic)
    monkeypatch.setattr("location_allocate.paper_runtime.ros_api.spin_once", spin)

    assert runtime(manager, clock=clock)._fresh_snapshot([1]).states[1].position[2] == 1.0
    assert requested == [pytest.approx(0.01)]


def test_unavailable_snapshot_never_spins_past_deadline(monkeypatch):
    manager = FreshStateSnapshotManager(0.1, 0.1)
    monotonic = Monotonic()
    requested = []

    def spin(_node, timeout_sec):
        requested.append(timeout_sec)
        monotonic.value += timeout_sec

    monkeypatch.setattr("location_allocate.paper_runtime.time.monotonic", monotonic)
    monkeypatch.setattr("location_allocate.paper_runtime.ros_api.spin_once", spin)

    with pytest.raises(RuntimeError, match="fresh state wait timed out"):
        runtime(manager)._fresh_snapshot([1])
    assert requested == [pytest.approx(0.01)]
    assert monotonic.value == pytest.approx(0.01)


def test_dispatch_snapshot_covers_all_mission_participants():
    mission = {
        "mission": {
            "nodes": [
                {"type": "task", "task": {"U": [2, 1]}},
                {
                    "type": "parallel",
                    "tasks": [{"U": [3, 2]}, {"U": [4]}],
                },
            ]
        }
    }

    assert PaperMissionRuntime._mission_participant_ids(mission) == (1, 2, 3, 4)


def test_first_resolution_consumes_dispatch_snapshot_without_rechecking(monkeypatch):
    manager = FreshStateSnapshotManager(0.1, 0.1)
    manager.update(1, [0, 0, 1], 100.0, source_timestamp=100.0)
    manager.update(2, [1, 0, 1], 100.0, source_timestamp=100.0)
    candidate_runtime = PaperMissionRuntime(
        Node(Clock()),
        SimpleNamespace(state=SimpleNamespace(fresh_state_wait_timeout=0.01)),
        None, manager, {}, None, [1, 2],
    )
    ready = manager.snapshot([1, 2], 100.0)
    candidate_runtime.prime_dispatch_snapshot([1, 2], ready)
    monkeypatch.setattr(
        candidate_runtime,
        "_fresh_snapshot",
        lambda _ids: pytest.fail("dispatch snapshot was checked twice"),
    )

    assert candidate_runtime._snapshot_for_resolution([1]) is ready
    assert candidate_runtime._dispatch_snapshot is None


def test_second_resolution_reacquires_after_single_dispatch_consumption(monkeypatch):
    manager = FreshStateSnapshotManager(0.1, 0.1)
    manager.update(1, [0, 0, 1], 100.0, source_timestamp=100.0)
    candidate_runtime = runtime(manager)
    ready = manager.snapshot([1], 100.0)
    candidate_runtime.prime_dispatch_snapshot([1], ready)
    assert candidate_runtime._snapshot_for_resolution([1]) is ready
    reacquired = object()
    monkeypatch.setattr(candidate_runtime, "_fresh_snapshot", lambda _ids: reacquired)
    assert candidate_runtime._snapshot_for_resolution([1]) is reacquired


def test_dispatch_readiness_rejects_stale_state(monkeypatch):
    manager = FreshStateSnapshotManager(0.01, 0.1)
    manager.update(1, [0, 0, 1], 100.0, source_timestamp=100.0)
    monotonic = Monotonic()
    monkeypatch.setattr("location_allocate.paper_runtime.time.monotonic", monotonic)
    monkeypatch.setattr(
        "location_allocate.paper_runtime.ros_api.spin_once",
        lambda _node, timeout_sec: setattr(monotonic, "value", monotonic.value + timeout_sec),
    )
    with pytest.raises(RuntimeError, match="dispatch readiness timed out"):
        runtime(manager, clock=Clock(100.1))._await_dispatch_snapshot([1])


def test_primed_dispatch_snapshot_rejects_wrong_participants():
    manager = FreshStateSnapshotManager(0.1, 0.1)
    manager.update(1, [0, 0, 1], 100.0, source_timestamp=100.0)
    runtime_instance = PaperMissionRuntime(
        Node(Clock()), SimpleNamespace(state=SimpleNamespace(fresh_state_wait_timeout=0.01)),
        None, manager, {}, None, [1, 2],
    )
    with pytest.raises(ValueError, match="participants"):
        runtime_instance.prime_dispatch_snapshot([1, 2], manager.snapshot([1], 100.0))


def test_command_publish_waits_for_controller_subscription(monkeypatch):
    manager = FreshStateSnapshotManager(0.1, 0.1)
    monotonic = Monotonic()
    events = []

    class Publisher:
        matched = False

        def get_subscription_count(self):
            return int(self.matched)

        def publish(self, command):
            events.append(("publish", command.uav_id))

    class Tracker:
        def arm(self, ids):
            events.append(("arm", tuple(ids)))

    publisher = Publisher()
    candidate_runtime = PaperMissionRuntime(
        Node(Clock()),
        SimpleNamespace(state=SimpleNamespace(fresh_state_wait_timeout=0.01)),
        None, manager, {1: publisher}, Tracker(), [1],
    )

    def spin(_node, timeout_sec):
        events.append(("spin", timeout_sec))
        monotonic.value += 0.001
        publisher.matched = True

    monkeypatch.setattr("location_allocate.paper_runtime.time.monotonic", monotonic)
    monkeypatch.setattr("location_allocate.paper_runtime.ros_api.spin_once", spin)
    candidate_runtime._publish_commands([SimpleNamespace(uav_id=1)])

    assert events[0][0] == "spin"
    assert events[1:] == [("arm", (1,)), ("publish", 1)]


def test_command_publish_does_not_mistake_bag_recorder_for_controller(monkeypatch):
    manager = FreshStateSnapshotManager(0.1, 0.1)
    monotonic = Monotonic()
    events = []

    class Publisher:
        def get_subscription_count(self):
            return 1

        def publish(self, command):
            events.append(("publish", command.uav_id))

    class Tracker:
        def arm(self, ids):
            events.append(("arm", tuple(ids)))

    class EndpointNode(Node):
        def __init__(self, clock):
            super().__init__(clock)
            self.controller_ready = False

        def get_subscriptions_info_by_topic(self, _topic):
            endpoints = [
                SimpleNamespace(
                    node_name="rosbag2_recorder", node_namespace="/"
                )
            ]
            if self.controller_ready:
                endpoints.append(
                    SimpleNamespace(
                        node_name="ladrc_position_controller_node",
                        node_namespace="/uav1",
                    )
                )
            return endpoints

    node = EndpointNode(Clock())
    candidate_runtime = PaperMissionRuntime(
        node,
        SimpleNamespace(state=SimpleNamespace(fresh_state_wait_timeout=0.01)),
        None,
        manager,
        {1: Publisher()},
        Tracker(),
        [1],
    )

    def spin(_node, timeout_sec):
        events.append(("spin", timeout_sec))
        monotonic.value += 0.001
        node.controller_ready = True

    monkeypatch.setattr("location_allocate.paper_runtime.time.monotonic", monotonic)
    monkeypatch.setattr("location_allocate.paper_runtime.ros_api.spin_once", spin)
    candidate_runtime._publish_commands([SimpleNamespace(uav_id=1)])

    assert events[0][0] == "spin"
    assert events[1:] == [("arm", (1,)), ("publish", 1)]
