# Patches applied to vendored files

| File | Change | Why |
|---|---|---|
| `alix/core/python/alix_workspace/src/alix_workspace/__init__.py` | Imports `reachability` lazily (module `__getattr__`) instead of at package import | Lite uses the kinematics and collision modules but not the Workspace-analysis features (X2); the eager import loaded them on every use of the package |

Runtime adjustments (no file changed) are made by `src/alix_studio_lite/vendor_paths.py`:
it puts the vendored packages on `sys.path`, points `ALIX_REPO_ROOT`, `ALIX_SRDF` and
`ALIX_TASKS_PKG` at `vendor/alix`, and makes `alix_workspace.package_index` find packages
inside `vendor/alix` (ALIX looks for a `.git` folder, which Lite's copy does not have).
