"""SDK adapters — the thin layer between alix and the RealMan SDK.

**Adapters only.** No vendor code lives here. The RealMan SDK itself is a
pinned submodule at ``external/butterfli-realman-alix`` (ARCHITECTURE.md §2.7, §4.5); this
package holds the things *we* write to talk to it:

* :mod:`alix_core.sdk.endpoints` — where the arms are, and on which ports
* ``algo.py`` — the per-arm FK/IK wrapper both the emulator and
  ``alix_core.kinematics`` depend on (ARCHITECTURE.md §2.4) *(not yet ported)*

Nothing in here may import ROS, and nothing in here may import
``alix_emulator`` — the emulator installs itself over the SDK before
``alix_core`` is imported, so an edge back would be circular.
"""
