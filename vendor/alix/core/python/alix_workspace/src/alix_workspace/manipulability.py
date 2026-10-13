"""Manipulability map (URDF-driven, ROS-free).

For each reachable voxel, the *best* (maximum) Yoshikawa manipulability index
``w = sqrt(det(Jv Jv^T))`` achievable by any sampled joint configuration whose
end-effector lands in that voxel. ``Jv`` is the 3xN translational geometric
Jacobian, so ``w`` reflects position dexterity — high where the arm moves freely
in all directions, ~0 near singularities. This is the dexterity analogue of the
binary reach envelope in :mod:`alix_workspace.reachability`, and reuses its
grid + vectorised joint sweep.

The map is a static property of the URDF + sweep params, so it is computed once
and cached (content-addressed). Lookup is two-tier: the writable per-user cache
(``$ALIX_WORKSPACE_CACHE`` / ``$XDG_CACHE_HOME`` / ``~/.cache/alix_workspace``)
first, then the read-only fixtures shipped in ``workspace_data``. Writes only
ever go to the writable cache.
"""

from __future__ import annotations

import logging
import time
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

from alix_workspace.reachability import ReachabilityAnalyzer, _placement

logger = logging.getLogger("alix_workspace.manipulability")


class ManipulabilityAnalyzer:
    """Per-voxel max translational manipulability for arbitrary URDF chains."""

    #: Read-only fixtures shipped with the package (see ReachabilityAnalyzer).
    FIXTURE_CACHE_DIR = ReachabilityAnalyzer.FIXTURE_CACHE_DIR

    #: Back-compat alias - external callers still read this name.
    DEFAULT_CACHE_DIR = FIXTURE_CACHE_DIR

    def __init__(self, config: dict, base_dir: Optional[str] = None,
                 cache_dir: Optional[str] = None):
        # Reuse the reachability analyzer for grid params, models and the sweep.
        self._reach = ReachabilityAnalyzer(config, base_dir=base_dir, cache_dir=cache_dir)
        # Mirrors ReachabilityAnalyzer: an explicit cache_dir wins for reads too.
        self._explicit_cache_dir = cache_dir is not None
        self._cache_dir = Path(cache_dir) if cache_dir else ReachabilityAnalyzer._default_cache_dir()
        self._chains = self._reach._chains
        self._maps: Dict[str, np.ndarray] = {}   # chain -> float32 grid (max w; -1 = unreached)
        self._filled_voxels: Dict[str, int] = {}   # interpolated to match the envelope
        self._outside_voxels: Dict[str, int] = {}  # sampled but outside the envelope

    def chain_names(self) -> List[str]:
        return list(self._chains.keys())

    # ------------------------------------------------------------------
    # Bump when the manipulability ALGORITHM changes (the content-addressed key
    # only hashes URDF + sweep params, so an algorithm change must be versioned
    # to invalidate committed artifacts). v2: dexterity over revolute joints only.
    # v3: the map's support is aligned to the reach ENVELOPE — see _align().
    ALGO_VERSION = "v3-revolute-envelope"

    def cache_key(self) -> str:
        import hashlib
        return hashlib.sha1((f"manip|{self.ALGO_VERSION}|" + self._reach.cache_key()).encode()).hexdigest()[:16]

    def cache_path(self) -> Path:
        """Where we WRITE. Always the writable cache."""
        return self._cache_dir / f"manip_{self.cache_key()}.npz"

    def _read_path(self) -> Optional[Path]:
        """Where we READ: the writable cache first, then the shipped fixtures.

        Fixture tier only when the cache home was defaulted — see
        ReachabilityAnalyzer._read_path for why.
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
        names = list(self._maps.keys())
        arrays = {f"w{i}": self._maps[n] for i, n in enumerate(names)}
        arrays["chains"] = np.array(names)
        arrays["origin"] = np.asarray(self._reach._grid_origin, dtype=float)
        arrays["shape"] = np.asarray(self._reach._grid_shape, dtype=int)
        arrays["res"] = np.asarray(self._reach._voxel_res, dtype=float)
        np.savez_compressed(path, **arrays)

    def _load_npz(self, path: Path) -> None:
        z = np.load(path, allow_pickle=False)
        names = [str(x) for x in z["chains"]]
        self._maps = {names[i]: z[f"w{i}"] for i in range(len(names))}
        self._reach._grid_origin = z["origin"]
        self._reach._grid_shape = tuple(int(x) for x in z["shape"])
        self._reach._voxel_res = float(z["res"])

    # ------------------------------------------------------------------
    def compute_all(self, use_cache: bool = True) -> Dict[str, np.ndarray]:
        if not self._chains:
            raise ValueError("reachability.chains is empty — nothing to compute")
        if use_cache:
            hit = self._read_path()
            if hit is not None:
                self._load_npz(hit)
                logger.info("Loaded manipulability map from %s", hit)
                return dict(self._maps)
        t0 = time.perf_counter()
        self._reach._make_grid()
        for name, spec in self._chains.items():
            self._maps[name] = self._align(name, spec, self._compute_chain(name, spec))
        logger.info("Manipulability computed in %.1f s for %s", time.perf_counter() - t0, list(self._maps))
        if use_cache:
            try:
                self._save_npz(self.cache_path())
                logger.info("Saved manipulability map to %s (per-user cache; use "
                            "scripts/precompute_reachability.py --write-fixtures to ship one)",
                            self.cache_path())
            except Exception as exc:  # pragma: no cover
                logger.warning("Could not save manipulability cache: %s", exc)
        return dict(self._maps)

    #: Voxels the envelope contains but the sweep never landed in are filled by
    #: grey dilation from their sampled neighbours, capped at this many passes.
    #: Beyond it they stay unvalued rather than being invented from far away.
    MAX_FILL_PASSES = 3

    def _align(self, name: str, spec: dict, grid: np.ndarray) -> np.ndarray:
        """Make the dexterity map cover exactly the reach ENVELOPE.

        The two maps disagreed, and silently. Reachability returns the *filled*
        envelope (morphological closing + hole fill, because a sparse joint
        sweep leaves interior gaps) while this map was the *raw* sweep — so on
        the shipped robot 5,339 voxels of the envelope, 14 % of it, were
        reported reachable with no dexterity value at all, and 23 voxels carried
        a dexterity value while sitting outside the envelope. In the Studio that
        reads as missing data; in a decision it reads as "no answer" where the
        honest answer is "the same as its neighbours".

        Both halves are fixed here. Voxels outside the envelope are dropped —
        the envelope is the authority on where the arm can be. Voxels inside it
        that the sweep missed take the MAXIMUM of their sampled neighbours,
        which is the same reduction the map already applies within a voxel, so
        the semantics do not change: it is still best-case dexterity.
        """
        from scipy import ndimage

        env = self._reach._fill(grid >= 0.0)
        sampled = grid >= 0.0
        missing = env & ~sampled
        out = np.where(sampled, grid, -1.0).astype(np.float32)
        out[~env] = -1.0                       # outside the envelope: no claim
        filled = 0
        for _ in range(self.MAX_FILL_PASSES):
            todo = env & (out < 0.0)
            if not todo.any():
                break
            grown = ndimage.grey_dilation(np.where(out < 0.0, -1.0, out), size=3)
            take = todo & (grown >= 0.0)
            out[take] = grown[take]
            filled += int(take.sum())
        self._filled_voxels[name] = filled
        self._outside_voxels[name] = int((sampled & ~env).sum())
        logger.info("Manip chain '%s': envelope %d voxels; %d interpolated from "
                    "neighbours, %d dropped as outside the envelope, %d still unvalued",
                    name, int(env.sum()), filled, self._outside_voxels[name],
                    int((env & (out < 0.0)).sum()))
        return out

    def grid_info(self) -> dict:
        info = self._reach.grid_info()
        info["interpolatedVoxels"] = dict(self._filled_voxels)
        info["droppedOutsideEnvelope"] = dict(self._outside_voxels)
        return info

    def points_and_values(self, chain: str, max_points: int = 20000, seed: int = 0):
        """(Nx3 voxel-center positions, N values w, w_max) for reached voxels."""
        if chain not in self._maps:
            raise RuntimeError("call compute_all() first")
        grid = self._maps[chain]
        idx = np.argwhere(grid >= 0.0)
        if not len(idx):
            return np.zeros((0, 3)), np.zeros(0), 0.0
        vals = grid[idx[:, 0], idx[:, 1], idx[:, 2]]
        pts = self._reach._grid_origin + idx * self._reach._voxel_res
        if len(pts) > max_points:
            sel = np.random.default_rng(seed).choice(len(pts), max_points, replace=False)
            pts, vals = pts[sel], vals[sel]
        return pts, vals, float(grid.max())

    # ------------------------------------------------------------------
    def _compute_chain(self, name: str, spec: dict) -> np.ndarray:
        """Vectorised sweep: per config compute EE position + translational
        manipulability, keep the max w per voxel. Mirrors the reachability sweep
        but also assembles the batched Jacobian."""
        model = self._reach._load_model(name, spec)
        chain = model.chain(spec["base_link"], spec["tip_link"])
        movable = model.movable_joints(chain)
        base_T = _placement(spec)
        grids = [self._reach._joint_samples(jt, spec) for jt in movable]
        counts = [len(g) for g in grids]
        total = int(np.prod(counts)) if grids else 1
        logger.info("Manip chain '%s': %d movable joints, %d configs", name, len(movable), total)

        origins = [jt.origin_matrix() for jt in chain]
        shape = self._reach._grid_shape
        flat = np.full(int(np.prod(shape)), -1.0, dtype=np.float32)
        gshape = np.array(shape)
        chunk = 100_000

        for start in range(0, total, chunk):
            end = min(start + chunk, total)
            lin = np.arange(start, end)
            qvals, rem = [], lin
            for c in counts[::-1]:
                qvals.append(rem % c)
                rem = rem // c
            qvals = [grids[j][qvals[len(grids) - 1 - j]] for j in range(len(grids))]

            Q = end - start
            T = np.broadcast_to(base_T, (Q, 4, 4)).copy()
            zs, ps = [], []   # per REVOLUTE joint: world axis, world origin
            mi = 0
            for j, jt in enumerate(chain):
                T = T @ origins[j]
                if jt.movable:
                    # Pole (prismatic) still drives FK/reach, but it is a
                    # gross-positioning DOF — excluded from the dexterity index
                    # (mixing prismatic m/m + revolute m/rad columns is unit-
                    # inconsistent and pole-axis-biased). Keep revolute columns only.
                    if jt.type != "prismatic":
                        zs.append(np.einsum("qij,j->qi", T[:, :3, :3], jt.axis))
                        ps.append(T[:, :3, 3].copy())
                    T = np.matmul(T, self._reach._batch_motion(jt, qvals[mi]))
                    mi += 1
            p_e = T[:, :3, 3]

            # translational Jacobian over the revolute joints -> Jv (Q,3,Nr)
            Nr = len(zs)
            Jv = np.empty((Q, 3, Nr), dtype=float)
            for i in range(Nr):
                Jv[:, :, i] = np.cross(zs[i], p_e - ps[i])
            A = np.einsum("qik,qjk->qij", Jv, Jv)         # Jv Jv^T  (Q,3,3)
            w = np.sqrt(np.clip(np.linalg.det(A), 0.0, None)).astype(np.float32)

            vi = np.round((p_e - self._reach._grid_origin) / self._reach._voxel_res).astype(np.int64)
            inb = np.all((vi >= 0) & (vi < gshape), axis=1)
            vi, wv = vi[inb], w[inb]
            if len(vi):
                lin_idx = (vi[:, 0] * shape[1] + vi[:, 1]) * shape[2] + vi[:, 2]
                # NOTE: per-voxel MAX over sampled configurations — the map
                # answers "best-case dexterity anywhere in this cell", NOT the
                # dexterity of a specific commanded pose. For the dexterity of
                # an actual joint state use IKSolver.manipulability(q)
                # (plan_analysis evaluates plans that way).
                np.maximum.at(flat, lin_idx, wv)
        grid = flat.reshape(shape)
        logger.info("Manip chain '%s': %d voxels, w_max=%.4f", name, int((grid >= 0).sum()), float(grid.max()))
        return grid
