"""ALIX's hardware-proven TOPP-RA chain runs from vendor/ and reproduces the 13 Oct research
measurement: commode C lid top (FILE_01.1) -> 3839 lines, 19.19 s at the 5 ms clock."""
import numpy as np
import pytest

pytest.importorskip("toppra")


@pytest.mark.slow
def test_toppra_chain_reproduces_research_measurement(commode_c_recordings):
    import local_speed as LS
    import taught_tracks as tt
    from alix_dispatch import track as tr

    raw = tt.read_q_deg(commode_c_recordings / "FILE_01.1.txt")
    saved = LS.RATE_DEG_S.copy()
    LS.RATE_DEG_S = np.array(tr.CANFD_RATED_DEG_S) * 0.95
    try:
        q, _ = tt.strip_pauses(tt.smooth_taught(raw, 5.0, 0.005))
        path = LS.TaughtPath(LS.arclen_smooth(LS.TaughtPath(q), 30.0))
        Q = LS.retime_toppra(path, LS.demand("max", path), 0.005)
    finally:
        LS.RATE_DEG_S = saved
    assert len(Q) == 3839
    assert len(Q) * 0.005 == pytest.approx(19.19, abs=0.01)
    step = np.abs(np.diff(Q, axis=0)).max()
    assert step <= 10.0                                  # controller rule: at most 10 deg per line
