"""ALIX Studio Lite — a guided tool for teaching, optimising, checking and running taught
dual-arm cleaning trajectories on the RealMan RM75 robot.

Everything Lite needs lives inside this repository: Lite's own code here in
``src/alix_studio_lite``, reused ALIX code and data in ``vendor/alix``, the control-program
runner in ``vendor/control_app`` and Lite's assets in ``assets``. Call
:func:`alix_studio_lite.vendor_paths.activate` before importing any vendored module.
"""

__version__ = "0.1.0"


def code_id() -> str:
    """A fingerprint of this copy's Python code. A second start compares it with the Lite already
    serving the port: the same code is simply opened; an older one must be stopped first (its
    server would keep running the old code; the web pages are read from disk on every request)."""
    import hashlib
    from pathlib import Path

    root = Path(__file__).resolve().parent
    h = hashlib.sha256()
    for f in sorted(root.rglob("*.py")):
        h.update(f.relative_to(root).as_posix().encode())
        h.update(f.read_bytes())
    return h.hexdigest()[:16]
