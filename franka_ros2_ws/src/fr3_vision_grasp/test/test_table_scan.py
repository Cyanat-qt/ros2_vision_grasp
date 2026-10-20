"""Empty-table termination must not be triggered by missing or stale images."""
from fr3_vision_grasp.core import EmptyTableConfirmation


def test_empty_table_requires_distinct_frames_and_observation_time():
    scan = EmptyTableConfirmation(1_000_000_000)
    for _ in range(10):
        scan.observe(1_100_000_000, set())
    assert not scan.confirmed(1_100_000_000)
    for stamp in (1_200_000_000, 1_300_000_000, 1_400_000_000, 1_500_000_000):
        scan.observe(stamp, set())
    assert not scan.confirmed(1_500_000_000)
    scan.observe(3_100_000_000, set())
    assert scan.confirmed(3_100_000_000)
    assert not scan.confirmed(4_200_000_000), 'Camera stopped publishing'


def test_visible_block_or_frame_gap_restarts_empty_confirmation():
    scan = EmptyTableConfirmation(0)
    for stamp in range(500_000_000, 3_000_000_000, 500_000_000):
        scan.observe(stamp, set())
    assert scan.confirmed(2_500_000_000)
    scan.observe(3_000_000_000, {'blue'})
    assert not scan.confirmed(3_000_000_000)
    scan.observe(3_500_000_000, set())
    assert not scan.confirmed(3_500_000_000)
    scan.observe(8_000_000_000, set())
    assert not scan.confirmed(8_000_000_000)
    # Queued images from before the return home, or out-of-order frames, do not count.
    scan.observe(1_000_000_000, set())
    assert scan.empty_count == 1
