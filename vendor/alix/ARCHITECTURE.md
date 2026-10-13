# ALIX subset vendored into ALIX Studio Lite

This folder holds the parts of the ALIX repository that ALIX Studio Lite reuses, copied by
`tools/vendor_from_alix.py` in ALIX's own folder layout. This file is a marker: ALIX code finds
its repo root by looking for `ARCHITECTURE.md` next to `core/` and `ros/`, so keeping the layout
lets that code run unchanged inside Lite. See `../MANIFEST.json` for the source of every file
and `../PATCHES.md` for the few patches applied.
