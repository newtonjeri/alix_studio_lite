"""The owner's revisions of 17 October, evening: the 3D's lag (Chrome drawing in software), the arm moves of
Advanced Robot Controls in all three modes, S2 on the robot without a simulation (asked twice), the 15 %
default speed, the arm indicators."""
from pathlib import Path


def test_lites_chrome_window_starts_with_its_caches_cleared(tmp_path):
    from alix_studio_lite import server

    prof = tmp_path / "chrome"
    for name in ("GPUCache", "GrShaderCache", "GPUPersistentCache", "Default/GPUCache", "Default/Cache", "Default/Code Cache"):
        (prof / name).mkdir(parents=True)
        (prof / name / "data_0").write_bytes(b"x")
    (prof / "Default" / "Preferences").write_text("{}")
    lock = prof / "SingletonLock"
    lock.symlink_to("host-123")                               # Lite's Chrome window is open: left as it is
    server._fresh_caches(prof)
    assert (prof / "GPUCache").exists() and (prof / "Default/Cache").exists()
    lock.unlink()
    server._fresh_caches(prof)
    assert not any((prof / n).exists() for n in ("GPUCache", "GrShaderCache", "GPUPersistentCache", "Default/GPUCache",
                                                 "Default/Cache", "Default/Code Cache"))
    assert (prof / "Default" / "Preferences").exists()       # the rest of the profile stays
