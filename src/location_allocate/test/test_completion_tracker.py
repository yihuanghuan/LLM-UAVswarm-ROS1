from location_allocate.completion_tracker import CompletionGenerationTracker


def test_old_true_is_ignored_until_new_false_then_true_generation():
    tracker = CompletionGenerationTracker()
    tracker.arm([1, 2])

    tracker.update(1, True)
    tracker.update(2, True)
    assert not tracker.all_stable([1, 2])

    tracker.update(1, False)
    tracker.update(2, False)
    tracker.update(1, True)
    assert not tracker.all_stable([1, 2])
    tracker.update(2, True)
    assert tracker.all_stable([1, 2])


def test_rearming_invalidates_previous_completion():
    tracker = CompletionGenerationTracker()
    tracker.arm([1])
    tracker.update(1, False)
    tracker.update(1, True)
    assert tracker.is_stable(1)

    tracker.arm([1])
    assert not tracker.is_stable(1)
