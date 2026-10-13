"""Cleaning-path coverage analyzer for Butterfli ALIX (Deliverable E).

This module deliberately contains **no coverage maths of its own**. Coverage,
region-mesh loading and frame transforms are the same functions the path
generator uses, imported directly from ``alix_taskgen``:

* ``alix_taskgen.coverage.compute_coverage`` — continuous-sweep footprint
  coverage, redundancy and over-coverage over a region mesh,
* ``alix_taskgen.region_mesh_loader`` — labeled-OBJ region submeshes,
* ``alix_taskgen.scene_pose`` — fixture/component placement from scene.yaml,
* ``alix_taskgen.arm_filter`` — the generator's RL_paired arm split, applied to
  the region mesh exactly as generation does (the generator scores coverage
  against the ARM'S OWN half of a paired region, not the full fixture),
* ``alix_taskgen.mirror.detect_arm`` — which arm a config belongs to,
* ``alix_taskgen.transforms`` — base_link <-> alix_ref_frame,
* ``alix_taskgen.pose_generator.contact_axis_for_ik_frame`` — glove footprint.

A number reported here matches the one ``alix_taskgen`` (and the studio's
coverage recompute) prints: same mesh chain, same glove definitions, same
canonical sample count, and THE SAME ARM SPLIT AS THE LANE THAT WROTE THE
CONFIG. Since [N-3] (P16 S4b, 2026-09-23) the two lanes are told apart:

* a C-space config (``provenance.authoring == 'path_pipeline'``) of a PAIRED
  region is scored on the ruled share, ``arm_filter.arm_share_face_indices``
  with the half-band the config RECORDED (``provenance.stroke.coverage.
  regionScope.seamHalfBandM``) -- exact, even after a manifest change -- else
  the spec's resolved ``seam_intersection_m`` / 2 (``exact`` False, said why);
* a legacy plane-lane config is re-split with ``filter_waypoints_for_arm`` +
  ``filter_mesh_for_arm``, the split that lane cuts with -- the spec's
  half-band on every paired region since the S4b fix pass.

Until the S4b fix pass this analyzer re-split EVERY config on the legacy rule
(0 or 40 mm) and labelled it ``exact: True``: s4b_seam_front/front_left.yaml
(cut at W 60) scored a 0.056984 m2 share against the generator's 0.069675
(REPORT_A1 lists the divergence). This package only adds the threshold check
(against ``coverage.min_coverage_pct``) and RViz rendering of the coverage
samples.
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import List, Optional, Sequence

import numpy as np
import trimesh
import yaml

# --- reused, not reimplemented ------------------------------------------------
from alix_taskgen import glove_frames
# _plane_distances is arm_filter's own signed-distance convention (positive =
# robot's left); imported deliberately so the overlap-extent reconstruction can
# never drift from the split it feeds into.
from alix_taskgen.arm_filter import (
    _plane_distances,
    arm_share_face_indices,
    filter_mesh_for_arm,
    filter_waypoints_for_arm,
    seam_half_band_of,
)
from alix_taskgen.constants import COVERAGE_SAMPLES
from alix_taskgen.coverage import compute_coverage, resolve_pose
from alix_taskgen.mirror import detect_arm
from alix_taskgen.region_mesh_loader import (
    load_labeled_submeshes,
    load_region_manifest,
)
from alix_taskgen.regions import get_region_info
from alix_taskgen.scene_pose import (
    fixture_sagittal_plane,
    load_washroom_rotation,
    resolve_component_pose,
    standoff_delta,
)
from alix_taskgen.transforms import (
    base_to_ref_transform,
    ref_to_base_transform,
    transform_mesh,
)

logger = logging.getLogger("alix_workspace.coverage")

REF_FRAME = "alix_ref_frame"


# ---------------------------------------------------------------------------
# Waypoint helpers (reusing alix_taskgen's resolver / transforms)
# ---------------------------------------------------------------------------
def load_waypoint_config(yaml_path: str) -> dict:
    """Load a ``alix_taskgen`` waypoint YAML into its raw config dict."""
    with open(yaml_path, "r") as fh:
        return yaml.safe_load(fh)


def waypoint_positions_ref(config: dict) -> np.ndarray:
    """Resolve Nx3 waypoint positions (alix_ref_frame) from a config dict.

    Uses ``alix_taskgen.coverage.resolve_pose`` so the reconstruction
    matches the runtime resolver exactly.
    """
    start = config.get("cartesian_poses", {}).get("start_pose", [0, 0, 0, 0, 0, 0, 1])
    points = config.get("cleaning_points") or {}

    def _key(name: str) -> int:
        digits = "".join(c for c in name if c.isdigit())
        return int(digits) if digits else 0

    out = [resolve_pose(start, points[n])[0] for n in sorted(points, key=_key)]
    return np.array(out, dtype=float).reshape(-1, 3)


def waypoint_positions_base_link(config: dict) -> np.ndarray:
    """Same as :func:`waypoint_positions_ref` but mapped into ``base_link``."""
    pts_ref = waypoint_positions_ref(config)
    return ref_to_base_transform().apply_points(pts_ref)


# ---------------------------------------------------------------------------
# Coverage analyzer
# ---------------------------------------------------------------------------
class CoverageAnalyzer:
    """Mesh-based cleaning-path coverage, reusing ``alix_taskgen``."""

    def __init__(self, config: dict):
        self._cfg = config
        cov = config.get("coverage", {})
        self._min_pct = cov.get("min_coverage_pct", {})
        # Accuracy controls: more surface samples -> tighter estimate; a fixed
        # seed makes the (sampling-based) coverage reproducible. The default is
        # the SAME canonical count the studio uses for every generate/recompute
        # report (alix_taskgen.constants.COVERAGE_SAMPLES, F73) so the numbers
        # line up across tools without tuning.
        self._samples = int(cov.get("surface_samples", COVERAGE_SAMPLES))
        self._seed = int(cov.get("seed", 0))

    # ------------------------------------------------------------------
    def _placed_group_meshes(
        self,
        scene_path: str,
        models_dir: str,
        region_key: str,
        use_washroom_rotation: bool = True,
        ugv_standoff_m: float | None = None,
    ):
        """Region submeshes placed in BASE_LINK, plus their placement inputs.

        Returns ``(manifest, spec, groups_base, washroom_rot, delta_m)`` where
        ``groups_base`` is ``[(group, component, trimesh in base_link), ...]``
        — one entry per labeled submesh, exactly the granularity the generator
        plans (and arm-filters) at. Mirrors ``alix_taskgen``'s placement chain,
        including the two placement inputs the generator always applies and
        this module used to drop:

        * the VENUE washroom rotation (default now ON — this venue rotates the
          fixture 180 deg about Z, so without it the mesh sat 704 mm from the
          path and every config scored 0.0 %),
        * the UGV-standoff shift (config's ``task_parameters.ugv_standoff_m``
          first, the region's authoring default as fallback — a task authored
          at 0.38 m against the scene default 0.25 m is 130 mm of X).
        """
        manifest = load_region_manifest(models_dir)
        if manifest is None:
            raise FileNotFoundError(f"No regions.yaml under {models_dir}")
        spec = manifest.spec_for(region_key)
        if spec is None:
            raise KeyError(f"Region '{region_key}' not in {models_dir}/regions.yaml")

        washroom_rot = (
            load_washroom_rotation(scene_path) if use_washroom_rotation else None
        )
        standoff = ugv_standoff_m if ugv_standoff_m is not None else spec.ugv_standoff_m
        delta_m = standoff_delta(scene_path, standoff) if standoff is not None else 0.0

        # load_labeled_submeshes returns (group, component, mesh) 3-tuples —
        # the 2-tuple unpack this used to do raised ValueError on the very
        # first call, so this analyzer had never actually run. And a single
        # spec.component transform mis-places cross-component regions
        # (lid_seat_hinge_area spans lid AND seat): mirror the generator and
        # place EACH submesh with its own component's pose, in the
        # articulation state the region declares for that part -- by the ONE
        # rule the generator's placement and the collision world share
        # (`region_component_resolver.fixture_states_for_region`, plan/P17
        # L0b 2026-09-26). This module's own rule (part_states, then the
        # primary component at articulation_state) gave the same state on
        # every (region, articulated component) of every fixture (19 rows,
        # 0 disagreements, L0b probe/states_rules_agree.py); the generator's
        # did not, which is the bug L0b fixed.
        from alix_taskgen.fixture_parts import load_part_registry
        from alix_taskgen.region_component_resolver import fixture_states_for_region
        states = dict(fixture_states_for_region(
            region_key, manifest_spec=spec, registry=load_part_registry(scene_path)))
        poses = {}

        def _pose_for(component):
            comp = component or spec.component
            if comp not in poses:
                poses[comp] = resolve_component_pose(
                    scene_path, comp,
                    articulation_state=states.get(comp),
                    washroom_rotation=washroom_rot,
                    standoff_delta_m=delta_m,
                )
            return poses[comp]

        submeshes = load_labeled_submeshes(manifest, region_key)
        groups_base = [
            (group, component, transform_mesh(m, _pose_for(component)))
            for group, component, m in submeshes
        ]
        return manifest, spec, groups_base, washroom_rot, delta_m

    # ------------------------------------------------------------------
    def region_mesh_ref(
        self,
        scene_path: str,
        models_dir: str,
        region_key: str,
        use_washroom_rotation: bool = True,
        ugv_standoff_m: float | None = None,
    ) -> "trimesh.Trimesh":
        """The FULL region mesh placed in alix_ref_frame (no arm split).

        ``analyze()`` scores against the arm-filtered variant of this mesh
        (see :meth:`_arm_scored_region_ref`); this full-region form stays for
        display/debug callers.
        """
        _, _, groups_base, _, _ = self._placed_group_meshes(
            scene_path, models_dir, region_key,
            use_washroom_rotation, ugv_standoff_m,
        )
        base_to_ref = base_to_ref_transform()
        placed = [transform_mesh(m, base_to_ref) for _g, _c, m in groups_base]
        mesh = placed[0] if len(placed) == 1 else trimesh.util.concatenate(placed)
        logger.info(
            "Region '%s' mesh placed in %s: area=%.4f m^2",
            region_key, REF_FRAME, float(mesh.area),
        )
        return mesh

    # ------------------------------------------------------------------
    def _arm_scored_region_ref(
        self,
        config: dict,
        filename: str,
        scene_path: str,
        models_dir: str,
        region_key: str,
        ugv_standoff_m: float | None = None,
    ):
        """The region mesh the GENERATOR scores this config against, in ref.

        Returns ``(mesh_ref, arm_meta)``. The generator plans per labeled
        group, arm-filters waypoints about the fixture's sagittal plane, cuts
        each group's mesh with ``filter_mesh_for_arm`` and reports coverage on
        that union — so an RL_paired config is scored against ITS arm's half
        only. Scoring the full region here made every paired config read ~half
        the generation-time number.

        The split is the LANE's that wrote the config (module docstring): a
        C-space config of a paired region is cut with `arm_share_face_indices`
        at the half-band it recorded (``exact``) or, lacking the record, the
        spec's (``exact`` False); a legacy plane-lane config with the waypoint
        filter at the spec's half-band. ``arm_meta`` names the lane, the mode,
        the threshold, the intersection and where the width came from.
        """
        manifest, spec, groups_base, washroom_rot, delta_m = (
            self._placed_group_meshes(
                scene_path, models_dir, region_key,
                use_washroom_rotation=True, ugv_standoff_m=ugv_standoff_m,
            )
        )
        base_to_ref = base_to_ref_transform()
        fixture_name = Path(scene_path).resolve().parent.name
        region_info = get_region_info(
            fixture_name, region_key, mesh_manifest=manifest
        )
        # Same plane the generator splits about: the fixture's own symmetry
        # plane under the SAME washroom rotation + standoff as the placement.
        plane = fixture_sagittal_plane(
            scene_path, washroom_rotation=washroom_rot, standoff_delta_m=delta_m
        )
        meta = {"arm": None, "mode": "unfiltered", "threshold_m": None,
                "exact": True}
        paired = getattr(spec, "task_type", None) == "paired"
        cspace = ((config.get("provenance") or {}).get("authoring") == "path_pipeline")
        try:
            arm = detect_arm(config, filename)
            if cspace and paired:
                # THE C-SPACE LANE's share ([N-3]): the half-band the config was
                # CUT with, from its own record -- a manifest change since (front:
                # W 60 -> 30 declared) must not move a saved config's denominator
                scope = (((config.get("provenance") or {}).get("stroke") or {})
                         .get("coverage") or {}).get("regionScope") or {}
                recorded = scope.get("seamHalfBandM")
                if recorded is None and scope.get("thresholdM") is not None:
                    recorded = abs(float(scope["thresholdM"]))
                half_band = (float(recorded) if recorded is not None
                             else seam_half_band_of(spec))
                # ONE cut over the whole placed region, as cspace_author does --
                # never per group (filter_mesh_for_arm's empty-group fallback
                # returns the whole group, which is not the generator's rule)
                whole = (groups_base[0][2] if len(groups_base) == 1 else
                         trimesh.util.concatenate([m for _g, _c, m in groups_base]))
                faces, share = arm_share_face_indices(
                    np.asarray(whole.triangles_center), arm, spec, plane=plane,
                    seam_overlap_m=half_band)
                meta.update(arm=arm, lane="cspace", mode=share["mode"],
                            threshold_m=share.get("thresholdM"),
                            seam_intersection_m=round(2.0 * half_band, 6),
                            seam_source="config" if recorded is not None else "spec",
                            exact=recorded is not None)
                if recorded is None:
                    meta["why_not_exact"] = (
                        "the config records no regionScope.seamHalfBandM; cut at the "
                        "spec's CURRENT W/2, which may differ from the width it was "
                        "generated at")
                groups_scored = [whole.submesh([faces], append=True, repair=False)
                                 if len(faces) else whole]
            else:
                # THE LEGACY PLANE LANE's split, as that lane cuts it (the spec's
                # half-band on a paired region since the S4b fix pass)
                pts = waypoint_positions_base_link(config)
                wpts = [{"position": [float(x), float(y), float(z)]}
                        for x, y, z in pts]
                seam = seam_half_band_of(spec) if (paired and spec is not None) else None
                _kept, stats = filter_waypoints_for_arm(
                    wpts, arm, region_info, seam_overlap_m=seam, plane=plane
                )
                meta.update(arm=arm, lane="legacy plane", mode=stats["mode"],
                            threshold_m=stats.get("threshold_y"),
                            seam_intersection_m=(round(2.0 * seam, 6) if seam is not None else None),
                            seam_source=("spec" if seam is not None else None))
                groups_scored = [
                    filter_mesh_for_arm(m, arm, region_info, stats)
                    for _g, _c, m in groups_base
                ]
        except ValueError as exc:
            # No arm derivable / inconsistent config: score the full region
            # rather than fail the report, and say so.
            logger.warning(
                "Arm-filter reconstruction failed (%s); scoring the FULL "
                "region mesh for '%s'", exc, region_key,
            )
            meta["mode"] = f"unfiltered ({exc})"
            groups_scored = [m for _g, _c, m in groups_base]

        mesh_base = (groups_scored[0] if len(groups_scored) == 1
                     else trimesh.util.concatenate(groups_scored))
        mesh_ref = transform_mesh(mesh_base, base_to_ref)
        logger.info(
            "Region '%s' scored mesh (%s, %s): area=%.4f m^2",
            region_key, meta["arm"] or "no-arm", meta["mode"],
            float(mesh_ref.area),
        )
        return mesh_ref, meta

    # ------------------------------------------------------------------
    def analyze(
        self,
        waypoint_yaml: str,
        scene_path: str,
        models_dir: str,
        region_key: str,
        return_samples: bool = False,
    ) -> dict:
        """Compute the coverage report for one region's cleaning path.

        Returns the dict from ``alix_taskgen.coverage.compute_coverage``
        augmented with ``region``, ``threshold_pct`` and ``meets_threshold``.
        """
        config = load_waypoint_config(waypoint_yaml)
        start_pose = config.get("cartesian_poses", {}).get("start_pose")
        if start_pose is None:
            raise ValueError(f"{waypoint_yaml} has no cartesian_poses.start_pose")
        ik_frame = config.get("task_parameters", {}).get("ik_frame", "")
        # Resolve contact axis + glove cleaning area + contact tolerance from the
        # editable definition file (alix_taskgen/config/glove_frames.yaml).
        glove = glove_frames.resolve(ik_frame)

        mesh_ref, arm_meta = self._arm_scored_region_ref(
            config, Path(waypoint_yaml).name, scene_path, models_dir, region_key,
            ugv_standoff_m=config.get("task_parameters", {}).get("ugv_standoff_m"),
        )
        report = compute_coverage(
            mesh_ref, start_pose, config,
            contact_axis=glove["contact_axis"],
            footprint=glove["footprint"],
            depth_tol=glove["depth_tol"],
            samples=self._samples,
            seed=self._seed,
            return_samples=return_samples,
        )
        report["arm"] = arm_meta["arm"]
        report["arm_filter"] = {
            "mode": arm_meta["mode"],
            "threshold_m": arm_meta["threshold_m"],
            "exact": arm_meta["exact"],
            "lane": arm_meta.get("lane"),
            "seam_intersection_m": arm_meta.get("seam_intersection_m"),
            "seam_source": arm_meta.get("seam_source"),
        }
        if arm_meta.get("why_not_exact"):
            report["arm_filter"]["why_not_exact"] = arm_meta["why_not_exact"]
        report["ik_frame"] = ik_frame
        report["glove_footprint_cm"] = [round(glove["footprint"][0] * 100, 1),
                                        round(glove["footprint"][1] * 100, 1)]
        report["contact_axis"] = glove["contact_axis"]

        threshold = float(self._min_pct.get(region_key, self._min_pct.get("toilet", 0.0)))
        report["region"] = region_key
        report["threshold_pct"] = threshold
        report["meets_threshold"] = report["coverage_pct"] >= threshold

        (logger.info if report["meets_threshold"] else logger.warning)(
            "Coverage '%s': %.1f%% (threshold %.1f%%), redundancy x%.2f%s",
            region_key, report["coverage_pct"], threshold, report["redundancy"],
            " OVER-COVERED" if report.get("over_covered") else "",
        )
        return report

    # ------------------------------------------------------------------
    @staticmethod
    def coverage_paint(report: dict) -> dict:
        """Web-ready coverage overlay from an ``analyze(return_samples=True)`` report.

        Returns ``{"points": [[x,y,z],...], "covered": [0/1,...], "frame": ...}``
        for direct rendering as a colored point cloud in the Three.js viewer.
        """
        pts = report.get("samplePoints")
        cov = report.get("sampleCovered")
        if pts is None or cov is None:
            raise ValueError("report has no samples; call analyze(return_samples=True)")
        return {"points": pts, "covered": cov, "frame": REF_FRAME}
