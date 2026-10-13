"""Generalised reachability analysis (URDF-driven, ROS-free).

Computes a 3-D voxel map of reachable end-effector positions for *any* serial
chain described by a URDF, by sampling the movable-joint space (limits read
from the URDF) and running forward kinematics
(:mod:`alix_workspace.urdf_kinematics`).

Multiple chains (e.g. a dual-arm robot's left/right arms) are supported and
voxelised on a shared grid, so their **overlap** (shared workspace / potential
self-collision zone) is the boolean intersection of their maps.

Configuration (``reachability`` section)::

    reachability:
      voxel_resolution_m: 0.05
      workspace_radius_m: 1.5
      workspace_height_min_m: -0.5
      workspace_height_max_m: 1.5
      samples_per_joint: 6
      chains:
        right:
          urdf: /abs/or/relative/rm_75.urdf
          base_link: base_link
          tip_link: Link7
          root_transform_xyz: [0.0, -0.124, 0.0]   # optional placement in a common frame
          root_transform_rpy: [0.0, 0.0, 0.0]
          joint_samples: { joint1: 8 }              # optional per-joint override
      overlaps: [[left, right]]                     # optional shared-workspace pairs
"""

from __future__ import annotations

import logging
import math
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
from scipy.spatial.transform import Rotation

from alix_workspace.urdf_kinematics import UrdfModel

logger = logging.getLogger("alix_workspace.reachability")


def _placement(spec: dict) -> np.ndarray:
    """4x4 transform from the common root frame to a chain's base_link."""
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler("xyz", spec.get("root_transform_rpy", [0, 0, 0])).as_matrix()
    T[:3, 3] = spec.get("root_transform_xyz", [0, 0, 0])
    return T


class ReachabilityAnalyzer:
    """Generate and query reachability voxel maps for arbitrary URDF chains."""

    # Content-addressed store for precomputed reach envelopes. The envelope is a
    # static property of the URDF kinematics + sweep params, so it is computed
    # once and reused. Two tiers: a WRITABLE per-user cache (the default write
    # AND first-read target) and the read-only fixtures committed in the repo.
    # NOTE on FIXTURE_CACHE_DIR: the store is a SIBLING of this file
    # (alix_workspace/workspace_data), not of the package dir. `parent.parent`
    # pointed at src/workspace_data after the src-layout move (same class of bug
    # fixed in server.py, paths.py and config_loader.py), so the committed
    # envelopes were never found and every cold start re-swept the envelope in
    # the worker process.
    #: Read-only fixtures shipped with the package (committed; WORKSPACE_ANALYSIS.md).
    #: NEVER written to except via scripts/precompute_reachability.py --write-fixtures.
    FIXTURE_CACHE_DIR = Path(__file__).resolve().parent / "workspace_data"

    #: Back-compat alias - external callers still read this name.
    DEFAULT_CACHE_DIR = FIXTURE_CACHE_DIR

    @staticmethod
    def _default_cache_dir() -> Path:
        """Writable cache home. NEVER the source tree: these files are keyed on the
        URDF's sha1, so a committed one is stale the moment the URDF moves, and the
        Studio's startup prewarm would otherwise dirty the repo on every run."""
        import os
        env = os.environ.get("ALIX_WORKSPACE_CACHE")
        if env:
            return Path(env).expanduser()
        xdg = os.environ.get("XDG_CACHE_HOME")
        root = Path(xdg).expanduser() if xdg else Path.home() / ".cache"
        return root / "alix_workspace"

    def __init__(self, config: dict, base_dir: Optional[str] = None,
                 cache_dir: Optional[str] = None):
        self._cfg = config.get("reachability", {})
        self._base_dir = Path(base_dir) if base_dir else Path.cwd()
        # An EXPLICIT cache_dir wins for reads as well as writes: a caller that
        # hands us a throwaway directory (a test forcing a cold compute) must not
        # be served a shipped fixture behind its back. Only the default path keeps
        # the read-only fixture tier. See _read_path().
        self._explicit_cache_dir = cache_dir is not None
        self._cache_dir = Path(cache_dir) if cache_dir else self._default_cache_dir()
        self._voxel_res = self._cfg.get("voxel_resolution_m", 0.05)
        self._radius = self._cfg.get("workspace_radius_m", 1.5)
        self._z_min = self._cfg.get("workspace_height_min_m", -0.5)
        self._z_max = self._cfg.get("workspace_height_max_m", 1.5)
        self._samples = self._cfg.get("samples_per_joint", 6)
        # The reach envelope is the *space* containing the reachable points, not
        # just the voxels an FK sample happened to land in. Sparse sampling
        # leaves interior holes, so close + fill the occupancy into a solid
        # envelope (any point inside it counts as reachable).
        self._fill_envelope = bool(self._cfg.get("fill_envelope", True))
        self._fill_closing_iter = int(self._cfg.get("fill_closing_iter", 2))
        self._chains: Dict[str, dict] = self._cfg.get("chains", {})
        self._overlaps: List[List[str]] = self._cfg.get("overlaps", [])

        self._models: Dict[str, UrdfModel] = {}
        self._voxels: Dict[str, np.ndarray] = {}
        self._grid_shape: Optional[Tuple[int, int, int]] = None
        self._grid_origin: Optional[np.ndarray] = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def chain_names(self) -> List[str]:
        return list(self._chains.keys())

    def chain_tip_link(self, chain: str) -> str:
        """The URDF link ``chain``'s envelope is swept for (its ``tip_link``).

        A caller whose own frame is rigidly fixed to the same wrist link
        queries the map at THIS frame's origin, offset by the constant
        transform between the two (`alix_taskgen.cspace_author.ArmOracle`).
        """
        spec = self._chains.get(chain)
        if spec is None:
            raise KeyError(f"no reachability chain '{chain}' (have {sorted(self._chains)})")
        return str(spec["tip_link"])

    # ------------------------------------------------------------------
    # Committed cache (content-addressed by URDF + params, machine-independent)
    # ------------------------------------------------------------------
    def cache_key(self) -> str:
        import hashlib
        import json
        parts = {
            "voxel_res": self._voxel_res, "radius": self._radius,
            "z_min": self._z_min, "z_max": self._z_max, "samples": self._samples,
            "fill": self._fill_envelope, "fill_iter": self._fill_closing_iter,
            "overlaps": self._overlaps, "chains": {},
        }
        for name, spec in self._chains.items():
            try:
                urdf_sha = hashlib.sha1(Path(self._resolve_urdf(spec)).read_bytes()).hexdigest()
            except Exception:
                urdf_sha = "missing"
            parts["chains"][name] = {
                "base": spec.get("base_link"), "tip": spec.get("tip_link"),
                "root_xyz": spec.get("root_transform_xyz"),
                "root_rpy": spec.get("root_transform_rpy"),
                "joint_samples": spec.get("joint_samples"), "urdf_sha": urdf_sha,
            }
        blob = json.dumps(parts, sort_keys=True, default=str)
        return hashlib.sha1(blob.encode()).hexdigest()[:16]

    def cache_path(self) -> Path:
        """Where we WRITE. Always the writable cache."""
        return self._cache_dir / f"reach_{self.cache_key()}.npz"

    def _read_path(self) -> Optional[Path]:
        """Where we READ: the writable cache first, then the shipped fixtures.

        The fixture tier is consulted ONLY when the cache home was defaulted. If the
        caller passed cache_dir= it gets that directory and nothing else — otherwise a
        test that points at an empty tmp dir to force a cold compute could still be
        served a committed .npz and never exercise the sweep.
        """
        p = self.cache_path()
        if p.is_file():
            return p
        if self._explicit_cache_dir:
            return None
        f = Path(self.FIXTURE_CACHE_DIR) / p.name
        return f if f.is_file() else None

    def cache_exists(self) -> bool:
        return self._read_path() is not None

    def _save_npz(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        names = list(self._voxels.keys())
        arrays = {f"v{i}": self._voxels[n] for i, n in enumerate(names)}
        arrays["chains"] = np.array(names)
        arrays["origin"] = np.asarray(self._grid_origin, dtype=float)
        arrays["shape"] = np.asarray(self._grid_shape, dtype=int)
        arrays["res"] = np.asarray(self._voxel_res, dtype=float)
        np.savez_compressed(path, **arrays)

    def _load_npz(self, path: Path) -> None:
        z = np.load(path, allow_pickle=False)
        names = [str(x) for x in z["chains"]]
        self._voxels = {names[i]: z[f"v{i}"] for i in range(len(names))}
        self._grid_origin = z["origin"]
        self._grid_shape = tuple(int(x) for x in z["shape"])
        self._voxel_res = float(z["res"])

    def compute_all(self, use_cache: bool = True) -> Dict[str, np.ndarray]:
        """Compute each chain's map plus configured overlaps.

        With ``use_cache`` (default), a precomputed envelope is loaded if present
        (instant, identical on any machine). Lookup is two-tier: the writable
        per-user cache first (``$ALIX_WORKSPACE_CACHE`` / ``$XDG_CACHE_HOME`` /
        ``~/.cache/alix_workspace``), then the read-only fixtures shipped in
        ``workspace_data/``. A miss computes and saves into the WRITABLE cache
        only - never the source tree.
        """
        if not self._chains:
            raise ValueError("reachability.chains is empty — nothing to compute")
        if use_cache:
            hit = self._read_path()
            if hit is not None:
                self._load_npz(hit)
                logger.info("Loaded reach envelope from %s", hit)
                return dict(self._voxels)
        t0 = time.perf_counter()
        self._make_grid()
        for name, spec in self._chains.items():
            raw = self._compute_chain(name, spec)
            self._voxels[name] = self._fill(raw)

        for pair in self._overlaps:
            if all(p in self._voxels for p in pair):
                key = "&".join(pair)
                inter = self._voxels[pair[0]].copy()
                for p in pair[1:]:
                    inter &= self._voxels[p]
                self._voxels[key] = inter
                logger.info("Overlap '%s': %d shared voxels", key, int(inter.sum()))

        logger.info(
            "Reachability computed in %.1f s for chains %s",
            time.perf_counter() - t0, list(self._voxels),
        )
        if use_cache:
            try:
                self._save_npz(self.cache_path())
                logger.info("Saved reach envelope to %s (per-user cache; use "
                            "scripts/precompute_reachability.py --write-fixtures to ship one)",
                            self.cache_path())
            except Exception as exc:  # pragma: no cover
                logger.warning("Could not save reach cache: %s", exc)
        return dict(self._voxels)

    def is_reachable(self, position, chain: str, voxel_tolerance: int = 1) -> bool:
        """Is a 3-D point (in the common root frame) reachable by ``chain``?

        The envelope is a discrete grid (default 40 mm voxels) built from a coarse
        joint sweep, so a point can sit just outside a reachable voxel and still be
        reachable in fact. Measured on the shipped commode_c configs: 8 of 1083
        commanded cleaning points reported unreachable, every one of them EXACTLY one
        voxel from a reachable cell, in tasks (bottomlid, hinge_area) that plan
        successfully through MoveIt IK. A hard lookup therefore reports false
        negatives at the boundary.

        ``voxel_tolerance`` accepts a point whose Chebyshev distance to a reachable
        voxel is within that many cells; 0 restores the strict lookup. Note the
        envelope is pure kinematics — reachable here never implies collision-free.
        """
        if chain not in self._voxels:
            raise RuntimeError(f"chain '{chain}' not computed; call compute_all()")
        idx = self._world_to_voxel(np.asarray(position, dtype=float))
        if idx is None:
            return False
        grid = self._voxels[chain]
        if bool(grid[idx]):
            return True
        if voxel_tolerance <= 0:
            return False
        lo = [max(0, idx[d] - voxel_tolerance) for d in range(3)]
        hi = [min(grid.shape[d], idx[d] + voxel_tolerance + 1) for d in range(3)]
        return bool(grid[lo[0]:hi[0], lo[1]:hi[1], lo[2]:hi[2]].any())

    def export_voxel(self, chain: str) -> np.ndarray:
        if chain not in self._voxels:
            raise RuntimeError("call compute_all() first")
        return self._voxels[chain]

    def reachable_points(self, chain: str) -> np.ndarray:
        """Nx3 array of reachable voxel-center positions (root frame)."""
        v = self.export_voxel(chain)
        idx = np.argwhere(v)
        return self._grid_origin + idx * self._voxel_res

    def grid_info(self) -> dict:
        return {
            "shape": self._grid_shape,
            "origin": self._grid_origin.tolist() if self._grid_origin is not None else None,
            "resolution_m": self._voxel_res,
        }

    # ------------------------------------------------------------------
    # Grid helpers
    # ------------------------------------------------------------------
    def _make_grid(self):
        half = self._radius
        x = np.arange(-half, half, self._voxel_res)
        y = np.arange(-half, half, self._voxel_res)
        z = np.arange(self._z_min, self._z_max, self._voxel_res)
        self._grid_origin = np.array([x[0], y[0], z[0]], dtype=float)
        self._grid_shape = (len(x), len(y), len(z))

    def _fill(self, grid: np.ndarray) -> np.ndarray:
        """Turn a sparse reachable-voxel occupancy into a solid envelope.

        Morphological closing bridges the gaps left by discrete joint sampling,
        then hole-filling solidifies any fully-enclosed interior — so the result
        is the volume containing the reachable points. Disabled via
        ``reachability.fill_envelope: false``.
        """
        if not self._fill_envelope or not grid.any():
            return grid
        from scipy import ndimage
        closed = ndimage.binary_closing(grid, iterations=max(1, self._fill_closing_iter))
        return ndimage.binary_fill_holes(closed)

    def _world_to_voxel(self, pt: np.ndarray):
        o, s, r = self._grid_origin, self._grid_shape, self._voxel_res
        i = int(round((pt[0] - o[0]) / r))
        j = int(round((pt[1] - o[1]) / r))
        k = int(round((pt[2] - o[2]) / r))
        if 0 <= i < s[0] and 0 <= j < s[1] and 0 <= k < s[2]:
            return (i, j, k)
        return None

    # ------------------------------------------------------------------
    # Per-chain computation
    # ------------------------------------------------------------------
    def _resolve_urdf(self, spec: dict) -> str:
        p = Path(spec["urdf"]).expanduser()
        if not p.is_absolute():
            p = (self._base_dir / p)
        if not p.is_file():
            raise FileNotFoundError(f"URDF not found for chain: {p}")
        return str(p)

    def _load_model(self, name: str, spec: dict) -> UrdfModel:
        if name not in self._models:
            self._models[name] = UrdfModel.from_file(self._resolve_urdf(spec))
        return self._models[name]

    def _joint_samples(self, jt, spec: dict) -> np.ndarray:
        n = spec.get("joint_samples", {}).get(jt.name, self._samples)
        if jt.type == "continuous":
            return np.linspace(-math.pi, math.pi, n, endpoint=False)
        if jt.lower == 0.0 and jt.upper == 0.0:
            # Locked joint / omitted <limit>: sample only 0 instead of the full
            # circle (which inflated the envelope with unreachable space).
            logger.warning(
                "Joint '%s' has lower==upper==0 — treating as FIXED at 0 "
                "(declare type=\"continuous\" if it is genuinely unlimited)",
                jt.name)
            return np.zeros(1)
        return np.linspace(jt.lower, jt.upper, n)

    @staticmethod
    def _batch_motion(jt, q: np.ndarray) -> np.ndarray:
        """Batched 4x4 joint-motion matrices for an array of joint values."""
        Q = q.shape[0]
        M = np.broadcast_to(np.eye(4), (Q, 4, 4)).copy()
        if jt.type == "prismatic":
            M[:, :3, 3] = jt.axis[None, :] * q[:, None]
        else:  # revolute / continuous — batched Rodrigues about the (unit) axis
            ax = jt.axis
            K = np.array([[0.0, -ax[2], ax[1]], [ax[2], 0.0, -ax[0]], [-ax[1], ax[0], 0.0]])
            s = np.sin(q)[:, None, None]
            c = np.cos(q)[:, None, None]
            M[:, :3, :3] = np.eye(3)[None] + s * K[None] + (1.0 - c) * (K @ K)[None]
        return M

    def _compute_chain(self, name: str, spec: dict) -> np.ndarray:
        """Vectorised FK sweep: batched numpy matmuls over all joint-sample
        combinations (chunked to bound memory), so dense sweeps are fast."""
        model = self._load_model(name, spec)
        chain = model.chain(spec["base_link"], spec["tip_link"])
        movable = model.movable_joints(chain)
        base_T = _placement(spec)

        grids = [self._joint_samples(jt, spec) for jt in movable]
        counts = [len(g) for g in grids]
        total = int(np.prod(counts)) if grids else 1
        logger.info(
            "Chain '%s' (%s->%s): %d movable joints, %d configurations (vectorised)",
            name, spec["base_link"], spec["tip_link"], len(movable), total,
        )
        origins = [jt.origin_matrix() for jt in chain]
        voxel = np.zeros(self._grid_shape, dtype=bool)
        shape = np.array(self._grid_shape)
        chunk = 200_000   # ~ Q*4*4*8 bytes per buffer; bounds peak memory

        for start in range(0, total, chunk):
            end = min(start + chunk, total)
            lin = np.arange(start, end)
            # decompose the linear index into per-joint sample indices (cartesian product)
            qvals, rem = [], lin
            for c in counts[::-1]:
                qvals.append(rem % c)
                rem = rem // c
            qvals = [grids[j][qvals[len(grids) - 1 - j]] for j in range(len(grids))]

            T = np.broadcast_to(base_T, (end - start, 4, 4)).copy()
            mi = 0
            for j, jt in enumerate(chain):
                T = T @ origins[j]
                if jt.movable:
                    T = np.matmul(T, self._batch_motion(jt, qvals[mi]))
                    mi += 1
            pos = T[:, :3, 3]
            vi = np.round((pos - self._grid_origin) / self._voxel_res).astype(np.int64)
            inb = np.all((vi >= 0) & (vi < shape), axis=1)
            vi = vi[inb]
            if len(vi):
                voxel[vi[:, 0], vi[:, 1], vi[:, 2]] = True
        logger.info("Chain '%s': %d/%d voxels reachable", name, int(voxel.sum()), voxel.size)
        return voxel
