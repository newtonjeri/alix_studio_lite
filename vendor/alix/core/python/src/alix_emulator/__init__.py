"""alix_emulator — a stand-in for the RealMan SDK, faithful enough to test against.

`rm_emulator` presents the same import surface as `Robotic_Arm` and installs
itself over it in `sys.modules`, so code under test imports the emulator
without knowing it. That is why this package cannot import `alix_core`: it is
installed *before* the core imports the SDK, and importing back would be
circular (`rm_emulator.py:136`). The dependency runs one way, and the vendor's
offline `Algo` — which needs no controller handle — is captured before the
swap. See ARCHITECTURE.md §2.4.

**This module is a verbatim port.** It differs from the RM_API2 original by
exactly one hunk: the `sys.path` bootstrap that locates the vendored SDK,
which had to move when the file did. Every remaining line is byte-identical,
and `alix_core/tests/test_emulator_verbatim.py` enforces that. Re-synced
2026-08-18 (Newton-approved) to the measured-law upstream (RM_API2
2026-08-15/16): movel executes real Cartesian geometry with the measured
blend/corner/freeze laws, movec arcs, per-move v/r — retiring F20.

Behaviour-affecting changes are deliberately **not** made here. The emulator
has certified bugs the hardware rejected three times, so its remaining blind
spots (`plan/FINDINGS.md` F21-F24, and joint-rate prediction — the offline
solver's redundancy resolution is not the controller's) are documented
rather than repaired — a green emulated run is evidence about the things the
emulator models, and silence about the things it does not.

Usage::

    from alix_emulator import rm_emulator
    rm_emulator.install()          # now `import Robotic_Arm` gets the emulator
"""

from __future__ import annotations

__all__ = ["rm_emulator", "install"]


def install(*args, **kwargs):
    """Install the emulator over `Robotic_Arm` in `sys.modules`.

    Thin re-export so callers need not know the module layout. Imported lazily
    because importing `rm_emulator` has the side effect of loading the vendor
    solver, and a bare `import alix_emulator` should not do that.
    """
    from alix_emulator import rm_emulator

    return rm_emulator.install(*args, **kwargs)
