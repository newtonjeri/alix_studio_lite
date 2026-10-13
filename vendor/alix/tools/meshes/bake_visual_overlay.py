#!/usr/bin/env python3
"""Bake the Studio's robot VISUAL overlay once, offline: per-material submeshes
with their real colours, error-bounded decimation, and a per-submesh integrity
gate. The server then serves the baked cache instead of parsing + serialising
1.2M triangles per session.

    python3 tools/meshes/bake_visual_overlay.py \
        --urdf core/python/alix_workspace/src/alix_workspace/urdf/alix.urdf \
        --out  core/python/alix_workspace/src/alix_workspace/workspace_data/visual_overlay_baked.npz

Pipeline per submesh (geometry in the LINK frame, like local_meshes()):
  extract   DAE/OBJ loaded WITHOUT force="mesh" -> one entry per material,
            coloured by that material's diffuse (RViz precedence); STL and
            primitives keep the URDF <material> colour
  weld      exact vertex weld (1e-6 m)
  simplify  meshoptimizer quadric, ERROR_ABSOLUTE cap, borders LOCKED;
            sheet-like meshes (boundary edges > 40% of faces) retried unlocked
            with a tighter cap; rejected results retried down a cap ladder
  GATE      accept only if the MEASURED two-sided sampled deviation stays
            <= --gate-mm AND no boundary (tear) length is added; otherwise the
            original mesh is kept. Integrity always wins over reduction.

Verified numbers on alix.urdf (2026-08): 1,183,924 -> 277,400 triangles (23.4%),
worst accepted deviation 1.48 mm, zero added tear length, bake ~15 s.

Dependencies: numpy, trimesh, meshoptimizer (pip), pycollada for DAE;
libigl (pip: libigl) if available makes verification exact+fast, else a
trimesh fallback is used.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import trimesh

try:
    import meshoptimizer as mo
except ImportError:
    sys.exit("pip install meshoptimizer (required for the simplify stage)")

try:
    import igl  # type: ignore
    _HAVE_IGL = True
except Exception:
    _HAVE_IGL = False


# --------------------------------------------------------------------------
# geometry helpers
def weld(V, F, digits=6):
    key = np.round(V, digits)
    uniq, inv = np.unique(key, axis=0, return_inverse=True)
    order = np.full(len(uniq), -1, np.int64)
    seen = np.zeros(len(uniq), bool)
    for i, j in enumerate(inv):
        if not seen[j]:
            seen[j] = True
            order[j] = i
    V2, F2 = V[order], inv[F]
    keep = (F2[:, 0] != F2[:, 1]) & (F2[:, 1] != F2[:, 2]) & (F2[:, 0] != F2[:, 2])
    return V2, F2[keep]


def boundary_len(V, F):
    e = np.sort(np.vstack([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]]), axis=1)
    eu, c = np.unique(e, axis=0, return_counts=True)
    b = eu[c == 1]
    if not len(b):
        return 0.0, 0
    return float(np.linalg.norm(V[b[:, 0]] - V[b[:, 1]], axis=1).sum()), int(len(b))


def mo_simplify(V, F, target_faces, cap_m, lock):
    idx = np.ascontiguousarray(F.ravel().astype(np.uint32))
    vp = np.ascontiguousarray(V.astype(np.float32))
    dest = np.empty(len(idx), np.uint32)
    opt = mo.SIMPLIFY_ERROR_ABSOLUTE | (mo.SIMPLIFY_LOCK_BORDER if lock else 0)
    n = mo.simplify(dest, idx, vp, target_index_count=int(target_faces * 3),
                    target_error=cap_m, options=opt)
    f = dest[:n].reshape(-1, 3).astype(np.int64)
    used = np.unique(f)
    remap = np.zeros(len(vp), np.int64)
    remap[used] = np.arange(len(used))
    return V[used].copy(), remap[f]


def true_dev_mm(Vs, Fs, Vd, Fd, n=6000):
    """two-sided sampled surface deviation in mm (meshes in metres)."""
    ms = trimesh.Trimesh(Vs, Fs, process=False)
    md = trimesh.Trimesh(Vd, Fd, process=False)
    a, _ = trimesh.sample.sample_surface(ms, n)
    b, _ = trimesh.sample.sample_surface(md, n)
    if _HAVE_IGL:
        d1 = np.sqrt(igl.point_mesh_squared_distance(np.ascontiguousarray(a), Vd, Fd)[0])
        d2 = np.sqrt(igl.point_mesh_squared_distance(np.ascontiguousarray(b), Vs, Fs)[0])
    else:  # slower fallback
        d1 = trimesh.proximity.closest_point(md, a)[1]
        d2 = trimesh.proximity.closest_point(ms, b)[1]
    d = np.concatenate([d1, d2]) * 1000.0
    return float(np.percentile(d, 99)), float(d.max())


# --------------------------------------------------------------------------
def optimize_entry(V0, F0, gate_mm, keep, min_faces):
    """Return (mode, V, F, p99_mm, max_mm) — passthrough when reduction would
    violate the gate."""
    V, F = weld(V0.astype(float), F0.astype(np.int64))
    if len(F) <= min_faces:
        return "passthrough", V, F, 0.0, 0.0
    bl0, bn0 = boundary_len(V, F)
    sheet = bn0 > 0.4 * len(F)
    target = max(400, int(len(F) * keep))
    ladder = [(0.0008, not sheet), (0.0004, not sheet), (0.0002, not sheet),
              (0.0002, False)]
    for cap, lock in ladder:
        v, f = mo_simplify(V, F, target, cap, lock)
        if len(f) >= 0.95 * len(F):
            continue                       # nothing gained at this rung
        p99, dmax = true_dev_mm(V, F, v, f)
        bl1, _ = boundary_len(v, f)
        if dmax <= gate_mm and (bl1 - bl0) <= max(0.02 * bl0, 0.003):
            return ("lock" if lock else "free") + f"@{cap * 1000:g}mm", v, f, p99, dmax
    return "passthrough(gate)", V, F, 0.0, 0.0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--out", required=True, help="output .npz cache")
    ap.add_argument("--gate-mm", type=float, default=1.5,
                    help="hard cap on measured deviation (default 1.5)")
    ap.add_argument("--keep", type=float, default=0.10,
                    help="target keep ratio before the error cap brakes (default 0.10)")
    ap.add_argument("--min-faces", type=int, default=900,
                    help="meshes at or below this stay untouched (default 900)")
    ap.add_argument("--no-simplify", action="store_true",
                    help="bake extraction + colours only (no decimation)")
    ap.add_argument("--pkg", action="append", default=[], metavar="NAME=DIR",
                    help="override/add a package dir for package:// resolution "
                         "(repeatable; useful for vendored packages)")
    args = ap.parse_args()

    # import here so the tool works from the repo without installation
    sys.path.insert(0, str(Path(args.urdf).resolve().parents[2]))
    from alix_workspace.collision import RobotCollisionModel
    from alix_workspace.package_index import PackageIndex

    index = PackageIndex.discover(args.urdf)
    for spec in args.pkg:
        name, _, d = spec.partition("=")
        index.name_to_dir[name] = Path(d)

    t0 = time.time()
    rcm = RobotCollisionModel.from_file(args.urdf, tag="visual", index=index)
    # requires the split_materials patch; falls back to plain entries without it
    try:
        entries = rcm.local_meshes(max_faces=0, split_materials=True)
    except TypeError:
        print("NOTE: collision.py without split_materials patch — baking per-link colours only")
        entries = rcm.local_meshes(max_faces=0)

    meta, blobs, rows = [], {}, []
    tot_in = tot_out = 0
    for i, e in enumerate(entries):
        V0 = np.asarray(e["vertices"], float)
        F0 = np.asarray(e["faces"], np.int64)
        tot_in += len(F0)
        if args.no_simplify:
            mode, v, f, p99, dmax = "passthrough", *weld(V0, F0), 0.0, 0.0
        else:
            mode, v, f, p99, dmax = optimize_entry(V0, F0, args.gate_mm,
                                                   args.keep, args.min_faces)
        tot_out += len(f)
        meta.append({"link": e["link"], "color": e.get("color"), "mode": mode})
        blobs[f"V{i}"] = v.astype(np.float32)
        blobs[f"F{i}"] = f.astype(np.uint32)
        rows.append((e["link"], len(F0), len(f), mode, p99, dmax))

    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.out, meta=json.dumps(meta), **blobs)
    print(f"{'link':26s} {'in':>8s} {'out':>8s} {'mode':>18s} {'p99mm':>6s} {'maxmm':>6s}")
    for link, fin, fout, mode, p99, dmax in rows:
        if fin != fout:
            print(f"{link:26s} {fin:8,} {fout:8,} {mode:>18s} {p99:6.2f} {dmax:6.2f}")
    print(f"\nTOTAL {tot_in:,} -> {tot_out:,} triangles "
          f"({tot_out / max(tot_in, 1) * 100:.1f}%) in {time.time() - t0:.1f}s "
          f"-> {args.out} ({Path(args.out).stat().st_size / 1e6:.1f} MB)")
    print("Serve it from do_robot_geometry() — see the integration notes in the report.")


if __name__ == "__main__":
    main()
