"""Manual tuning is the trajectory HTML's algorithm as it is (the owner, 15 Oct: "the manual tuning
should be the default and its starting point is the algorithm in this html"), its limits shown and
never enforced; Time-optimal (TOPP-RA) is separate and never goes through it.

The expected bytes are the HTML's own: its functions (docs/reference/ui_v32/Alix_Studio_Lite_Trajectory_V2.html)
were run in Chrome on the same recording and settings on 15 Oct 2026, and their output hashed in the
HTML's "Download processed TXT" form (one {"point":[...]} per line)."""
import hashlib
import json

import pytest

HTML_SHA256 = {          # FILE_10.1, as the HTML makes it
    "suggestions": ("c06914419de9aa3924c774bd53c549cecb2775290653d5b384dead74b85403b1", 2463),
    "no trim, odd %": ("fb51d0a7e95f3030c92dae99b214f2848a1564e50cfd65343623b287229f4892", 4380),
    "threshold 0 is 8": ("27f121c2a9afc19830521fcb7ee01d63f0a935a989674569dee04e007b03e85f", 1148),
}


@pytest.fixture(scope="module")
def md(commode_c_recordings):
    from alix_studio_lite import vendor_paths

    vendor_paths.activate()
    from alix_studio_lite.motion import track

    return track.read_md(commode_c_recordings / "FILE_10.1.txt")


def html_text(md) -> str:
    return "".join(json.dumps({"point": [int(v) for v in row]}, separators=(",", ":")) + "\n" for row in md)


def test_the_same_bytes_as_the_html(md):
    from alix_studio_lite.optimise import manual
    from alix_studio_lite.optimise.analysis import analyse

    sugg = [dict(start=r["start"], end=r["end"], pct=r["scale_pct"]) for r in analyse(md)["regions"]]
    cases = {"suggestions": dict(segments=sugg),
             "no trim, odd %": dict(segments=[dict(start=10, end=900, pct=33), dict(start=1500, end=2400, pct=287)], trim=False),
             "threshold 0 is 8": dict(segments=[dict(start=1, end=len(md), pct=40)], idle_threshold_md=0)}
    for name, settings in cases.items():
        out = manual.process(md, settings, "FILE_10.1.txt")
        sha, lines = HTML_SHA256[name]
        assert out.md.shape[0] == lines and hashlib.sha256(html_text(out.md).encode()).hexdigest() == sha, name
        assert (out.md[0] == md[0]).all() and (out.md[-1] == md[-1]).all()
    rep = manual.process(md, cases["suggestions"], "FILE_10.1.txt").report
    assert rep["outputName"] == "FILE_10.1_trim_mean_scale_processed.txt"
    assert rep["parameters"] == dict(sampleMs=5.0, idleWindow=11, idleThreshold=8.0, idleMinRun=20, smoothWindow=101)
    assert rep["selectedStageLabel"] == "Trim + Mean Smooth + Scale" and len(rep["scalingRegions"]) == 5


def test_the_htmls_checks_and_defaults(md):
    from alix_studio_lite.optimise import manual

    assert manual.validate_odd(10, 11) == 11 and manual.validate_odd("", 101) == 101 and manual.validate_odd(1, 11) == 3
    assert manual.params({}) == dict(sample_ms=5.0, idle_window=11, idle_threshold_md=8.0, idle_min_run=20, smooth_window=101)
    with pytest.raises(ValueError, match="at least one processing stage"):
        manual.process(md, dict(trim=False, smooth=False, scale=False))
    with pytest.raises(ValueError, match="overlap after idle trimming"):
        manual.process(md, dict(trim=False, segments=[dict(start=1, end=40, pct=50), dict(start=40, end=90, pct=50)]))
    out = manual.process(md, dict(segments=[dict(start=200, end=230, pct=50)], idle_min_run=1, idle_threshold_md=1e6))
    assert out.report["skippedRegions"] and "no retained samples" in out.report["skippedRegions"][0]["reason"]


def test_limits_are_shown_never_enforced_and_the_time_is_the_htmls(md):
    """A 40 % file breaks the acceleration limit: it is made all the same, at the HTML's time (the
    owner, 15 Oct: a 15 s target came out 36 s because Lite stretched it to the limits)."""
    from alix_studio_lite.motion.timing import duration_for
    from alix_studio_lite.optimise import manual

    res = manual.optimise(md, "left", dict(segments=[dict(start=1, end=len(md), pct=40)]), "FILE_10.1.txt")
    s = res.summary()
    assert s["uploadable"] and not s["limits_ok"] and s["enforced"] is False and "L3" in s["limits_text"]
    assert s["achieved_s"] == pytest.approx(duration_for(res.lines)) and res.lines == res.report["finalRows"]
    assert "not enforced" in res.message and res.message.startswith("Generated successfully.")


def test_the_optimised_time_is_the_generated_files(md):
    """Analyze's optimised time: the suggestions fitted after Trim, so Generate makes that time."""
    from alix_studio_lite.optimise import manual
    from alix_studio_lite.optimise.analysis import analyse, fit_target

    a = analyse(md)
    for target in (6.0, 9.5):
        counts, kept = manual.trimmed_counts(md, {}, a["regions"])
        f = fit_target(a, target, counts=counts, kept_total=kept)
        assert f["target"]["reached"] and f["target"]["after_trim"]
        segs = [dict(start=r["start"], end=r["end"], pct=r["scale_pct"]) for r in f["regions"] if r["selected"]]
        assert all(1 <= x["pct"] <= 300 and x["pct"] == int(x["pct"]) for x in segs)
        made = manual.optimise(md, "left", dict(segments=segs))
        assert made.achieved_s == pytest.approx(f["target"]["estimated_s"], abs=1e-9)
        assert made.achieved_s == pytest.approx(target, abs=max(0.05, 0.01 * target))
    far = fit_target(a, 0.01, *manual.trimmed_counts(md, {}, a["regions"]))
    assert not far["target"]["reached"] and all(r["scale_pct"] == 1 for r in far["regions"])


def test_time_optimal_takes_nothing_of_manual_tuning():
    from alix_studio_lite.server.workers import is_manual, request_kwargs

    s = dict(optimiser="toppra", trim=False, smooth_window=21, segments=[dict(start=1, end=9, pct=50)], time_mode="target",
             target_s=12.0, max_dev_mm=8.0)
    assert not is_manual(s) and request_kwargs(s) == dict(optimiser="toppra", max_dev_mm=8.0, target_s=12.0)
    assert request_kwargs(dict(s, time_mode="fastest"))["target_s"] is None
    assert is_manual({}) and is_manual(dict(optimiser="default"))           # a region set before 15 Oct tunes manually
