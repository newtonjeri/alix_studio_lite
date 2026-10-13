"""Butterfli ALIX — Dual-Arm Workspace Analysis Toolkit (vendored into ALIX Studio Lite).

Lite patch: the package used to import ``reachability`` (a Workspace-analysis feature that Lite
excludes, X2) every time any submodule was imported. It is now imported only on first use of
``ReachabilityAnalyzer``. See vendor/PATCHES.md.
"""

__all__ = ["ReachabilityAnalyzer"]


def __getattr__(name):
    if name == "ReachabilityAnalyzer":
        from .reachability import ReachabilityAnalyzer
        return ReachabilityAnalyzer
    raise AttributeError(name)
