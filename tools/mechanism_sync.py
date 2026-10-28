#!/usr/bin/env python3
"""Keep Lite's lid and seat tasks and plans the same as ALIX's (server/alix_sync.py says what is compared and checked).

    python3 tools/mechanism_sync.py                  # status: what differs, which side changed it, the plans' checks
    python3 tools/mechanism_sync.py pull             # ALIX -> Lite: what changed (or is new) in ALIX
    python3 tools/mechanism_sync.py push             # Lite -> ALIX: what changed in Lite (the owner commits ALIX)
    python3 tools/mechanism_sync.py pull --take-alix # a file changed on both sides: ALIX's kept
    python3 tools/mechanism_sync.py push --take-lite # ... Lite's kept
    --alix PATH (default: $LITE_ALIX_ROOT or ~/alix_ws/src/alix), --dry-run

Exit 0 when both sides hold the same files and every check passes, 1 when not. Run it after any change to a lid or seat
config or plan, on either side, and before committing Lite (tests/test_alix_sync.py fails while they differ).
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from alix_studio_lite import vendor_paths  # noqa: E402

vendor_paths.activate()
from alix_studio_lite.server import alix_sync  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("what", nargs="?", default="status", choices=("status", "pull", "push"))
    ap.add_argument("--alix", type=Path, default=None)
    ap.add_argument("--take-alix", action="store_true", help="pull: a file changed on both sides takes ALIX's")
    ap.add_argument("--take-lite", action="store_true", help="push: a file changed on both sides takes Lite's")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()
    alix = args.alix or alix_sync.alix_root()
    if args.what != "status" and alix is None:
        print("ALIX's checkout is not on this computer: pass --alix PATH or set LITE_ALIX_ROOT.")
        return 2
    if args.what == "pull":
        done = alix_sync.pull(alix, take_alix=args.take_alix, dry_run=args.dry_run)
        for path, what in done:
            print(f"  {'would be ' if args.dry_run else ''}{what}: {path}")
        print(f"{len(done)} file(s) {'would change' if args.dry_run else 'pulled'}")
    elif args.what == "push":
        done = alix_sync.push(alix, take_lite=args.take_lite, dry_run=args.dry_run)
        for path, what in done:
            print(f"  {'would be ' if args.dry_run else ''}{what}: {path}")
        print(f"{len(done)} file(s) {'would change' if args.dry_run else 'pushed'}" + (" (commit them in ALIX)" if done and not args.dry_run else ""))
    rep = alix_sync.status(alix)
    print("\n".join(rep.lines()))
    return 0 if rep.in_sync else 1


if __name__ == "__main__":
    sys.exit(main())
