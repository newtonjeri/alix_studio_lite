"""alix_core.diagnostics — error codes, run recording, probes, capability census.

Owns everything that *observes* the robot. Per LAYERS.md §2 it must not know
about anything it might perturb.

Re-exports are lazy (PEP 562). Importing them eagerly would mean
`python3 -m alix_core.diagnostics.error_codes` loads the module twice and
warns — and that invocation is the documented way to decode a code at the
bench, so it should be clean.
"""

_LAZY = {
    "describe_api2_return": "error_codes",
    "describe_joint_err": "error_codes",
    "describe_system_err": "error_codes",
    "explain": "error_codes",
    "remedy": "error_codes",
}

__all__ = sorted(_LAZY)


def __getattr__(name):
    if name in _LAZY:
        import importlib

        mod = importlib.import_module(f"{__name__}.{_LAZY[name]}")
        value = getattr(mod, name)
        globals()[name] = value
        return value
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __dir__():
    return sorted(set(globals()) | set(_LAZY))
