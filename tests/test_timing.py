"""D26: the 5 ms controller cycle is the only time base, and a file of N lines lasts
(N - 1) x 5 ms, the true time from its first line to its last."""
import pytest

from alix_studio_lite.motion import timing


def test_true_time():
    assert timing.duration_for(2) == pytest.approx(0.005)
    assert timing.duration_for(3839) == pytest.approx(19.19)      # commode C lid top, TOPP-RA (ALIX slot 101)
    assert timing.duration_for(12001) == pytest.approx(60.0)


def test_target_line_count():
    assert timing.lines_for(60.0) == 12001
    assert timing.lines_for(0.001) == 2                            # never fewer than two lines
    for n in (2, 857, 3839, 12001):
        assert timing.lines_for(timing.duration_for(n)) == n


def test_bad_targets_are_refused():
    for bad in (0, -1, float("nan"), float("inf")):
        with pytest.raises(ValueError):
            timing.lines_for(bad)
