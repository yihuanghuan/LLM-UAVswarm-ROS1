from types import SimpleNamespace

import pytest

from location_allocate.state_ingest import ingest_standardized_odometry
from location_allocate.state_snapshot import FreshStateSnapshotManager


def odometry(*, frame="world", child="uav2/base_link_enu", stamp=10.0):
    seconds = int(stamp)
    nanos = int(round((stamp - seconds) * 1e9))
    return SimpleNamespace(
        header=SimpleNamespace(
            frame_id=frame,
            stamp=SimpleNamespace(secs=seconds, nsecs=nanos),
        ),
        child_frame_id=child,
        pose=SimpleNamespace(pose=SimpleNamespace(
            position=SimpleNamespace(x=1.0, y=2.0, z=3.0)
        )),
        twist=SimpleNamespace(twist=SimpleNamespace(
            linear=SimpleNamespace(x=0.1, y=0.2, z=0.3)
        )),
    )


def production_manager():
    return FreshStateSnapshotManager(
        0.5, 0.15,
        require_velocity=True,
        allow_receive_time_fallback=False,
    )


def test_standardized_odometry_populates_candidate_state():
    manager = production_manager()
    ingest_standardized_odometry(manager, odometry(), 2, 10.05)

    state = manager.snapshot([2], 10.1).states[2]
    assert state.position == (1.0, 2.0, 3.0)
    assert state.velocity == (0.1, 0.2, 0.3)
    assert state.source_timestamp == 10.0


@pytest.mark.parametrize(
    "message, match",
    [
        (odometry(frame="map"), "frame_id"),
        (odometry(child="uav2/base_link"), "child_frame_id"),
        (odometry(stamp=0.0), "source_timestamp"),
    ],
)
def test_invalid_standardized_odometry_fails_closed(message, match):
    with pytest.raises(ValueError, match=match):
        ingest_standardized_odometry(production_manager(), message, 2, 10.0)
