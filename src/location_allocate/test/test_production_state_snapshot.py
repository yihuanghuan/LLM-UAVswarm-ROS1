import pytest

from location_allocate.state_snapshot import (
    FreshStateSnapshotManager,
    SnapshotError,
)


def manager(*, fallback=False):
    return FreshStateSnapshotManager(
        0.5,
        0.15,
        require_velocity=True,
        allow_receive_time_fallback=fallback,
    )


def test_production_state_preserves_position_velocity_and_both_timestamps():
    states = manager()
    states.update(
        1,
        [1.0, 2.0, 3.0],
        receive_timestamp=10.05,
        velocity=[0.1, 0.2, 0.3],
        source_timestamp=10.0,
    )

    sample = states.snapshot([1], now=10.1).states[1]
    assert sample.position == (1.0, 2.0, 3.0)
    assert sample.velocity == (0.1, 0.2, 0.3)
    assert sample.source_timestamp == 10.0
    assert sample.receive_timestamp == 10.05
    assert sample.timestamp_source == "source_timestamp"


def test_production_state_rejects_missing_velocity_and_zero_timestamp():
    with pytest.raises(ValueError, match="velocity is required"):
        manager().update(1, [0, 0, 1], 10.0, source_timestamp=9.9)
    with pytest.raises(ValueError, match="source_timestamp is required"):
        manager().update(1, [0, 0, 1], 10.0, [0, 0, 0], 0.0)


def test_receive_time_fallback_is_explicit_and_audited():
    states = manager(fallback=True)
    states.update(1, [0, 0, 1], 10.0, [0, 0, 0], 0.0)

    snapshot = states.snapshot([1], now=10.1)

    assert snapshot.states[1].timestamp_source == "receive_time_fallback"
    assert "receive-time fallback" in snapshot.warnings[0]


def test_future_stale_and_skewed_source_timestamps_are_rejected():
    future = manager()
    future.update(1, [0, 0, 1], 10.0, [0, 0, 0], 10.2)
    with pytest.raises(SnapshotError, match="stale UAV states"):
        future.snapshot([1], now=10.1)

    stale = manager()
    stale.update(1, [0, 0, 1], 10.0, [0, 0, 0], 9.0)
    with pytest.raises(SnapshotError, match="stale UAV states"):
        stale.snapshot([1], now=10.1)

    skewed = manager()
    skewed.update(1, [0, 0, 1], 10.0, [0, 0, 0], 10.0)
    skewed.update(2, [1, 0, 1], 10.2, [0, 0, 0], 10.2)
    with pytest.raises(SnapshotError, match="snapshot_skew"):
        skewed.snapshot([1, 2], now=10.25)
