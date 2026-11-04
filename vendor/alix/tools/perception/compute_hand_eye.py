#!/usr/bin/env python3
"""Solve the wrist D435's eye-in-hand calibration from a capture session — RealMan's method.

    python3 tools/perception/compute_hand_eye.py --data Resource/Data/hand_eye_2026-10-13
    python3 tools/perception/compute_hand_eye.py --data ... --intrinsics calibrate   # RealMan's own K
    python3 tools/perception/compute_hand_eye.py --data ... --exclude 4 11           # drop samples

Reads `<data>/samples.yaml` + `<data>/images/*.png` written by `capture_hand_eye.py`, re-detects
every board, runs `alix_perception_core.hand_eye.calibrate` and writes
`<data>/hand_eye_result.yaml`: the camera in the END frame (`R_CameraHolderLink`) by each of
OpenCV's five methods (Tsai = RealMan's) and by the joint refinement, the correction against the
URDF nominal, the consistency evidence, and the URDF mount `<origin>` that would apply it. Applying
it is a separate, deliberate step. ROS-free.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "core" / "python" / "alix_perception_core" / "src"))

from alix_perception_core import hand_eye as he          # noqa: E402

NOMINAL_MOUNT = dict(xyz=[0.0, 0.0625, 0.011], rpy=[0.0, -np.pi / 2, -np.pi / 2])   # alix.urdf.xacro sensor_d435 <origin>
# acceptance (this tool's, not RealMan's): RealMan states the translation within 1 cm; the board
# never moved, so its spread in the base frame is the end-to-end error the robot will see
ACCEPT_SPREAD_MM = 3.0
ACCEPT_REPROJ_PX = 1.5


def plain(v):
    if isinstance(v, dict):
        return {str(k): plain(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [plain(x) for x in v]
    if isinstance(v, np.ndarray):
        return plain(v.tolist())
    if isinstance(v, (np.floating,)):
        return float(v)
    if isinstance(v, (np.integer,)):
        return int(v)
    if isinstance(v, (np.bool_,)):
        return bool(v)
    if isinstance(v, float):
        return round(v, 6)
    return v


def load_session(data: Path) -> dict:
    s = yaml.safe_load((data / "samples.yaml").read_text())
    if not s or not s.get("samples"):
        raise SystemExit(f"{data / 'samples.yaml'}: no samples")
    return s


def detect_all(data: Path, session: dict, board: he.Board, exclude: set[int]):
    import cv2
    used, skipped = [], []
    for row in session["samples"]:
        idx = int(row["index"])
        if idx in exclude:
            skipped.append((idx, "excluded"))
            continue
        img = cv2.imread(str(data / row["image"]), cv2.IMREAD_GRAYSCALE)
        if img is None:
            skipped.append((idx, "image missing"))
            continue
        corners = he.detect(img, board)
        if corners is None:
            skipped.append((idx, "board not found / ends ambiguous"))
            continue
        used.append(dict(index=idx, T_base_end=np.array(row["T_base_end"], float), corners=corners,
                         size=(img.shape[1], img.shape[0])))
    return used, skipped


def summarise(name: str, res: dict, nominal_mount_cam, nominal_end_cam=None) -> dict:
    out = dict(intrinsics=name, samples=res["samples"], diversity=res["diversity"], end_frame="R_CameraHolderLink",
               board_tilt_deg=res["board_tilt_deg"], board_range_m=res["board_range_m"],
               board_pose_rms_px=res["board_pose_rms_px"], methods={})
    for m, v in res["methods"].items():
        if "error" in v:
            out["methods"][m] = dict(error=v["error"])
            continue
        out["methods"][m] = dict(T_end_cam=v["T_end_cam"], vs_nominal=v.get("vs_nominal"),
                                 board_spread=v["board_spread"], reprojection_rms_px=v["reprojection_rms_px"],
                                 reprojection_worst_px=v["reprojection_worst_px"])
    if "refined" in res:
        r = res["refined"]
        out["refined"] = dict(T_end_cam=r["T_end_cam"], vs_nominal=r.get("vs_nominal"), board_spread=r["board_spread"],
                              reprojection_rms_px=r["reprojection_rms_px"], reprojection_worst_px=r["reprojection_worst_px"],
                              reprojection_per_sample_px=r["reprojection_per_sample_px"], board_in_base=r["board_in_base"])
        if r.get("vs_nominal") is not None:
            out["refined"]["end_frame_correction"] = he.end_frame_correction(r["T_end_cam"], nominal_end_cam)
        if nominal_mount_cam is not None:
            out["refined"]["urdf_mount_origin"] = he.mount_origin(r["T_end_cam"], nominal_mount_cam)
        ok = (res["diversity"]["enough"] and r["board_spread"]["translation_rms_mm"] <= ACCEPT_SPREAD_MM
              and r["reprojection_rms_px"] <= ACCEPT_REPROJ_PX)
        out["verdict"] = dict(accepted=bool(ok), spread_limit_mm=ACCEPT_SPREAD_MM, reprojection_limit_px=ACCEPT_REPROJ_PX)
    for k in ("method_agreement", "leave_one_out"):
        if k in res:
            out[k] = res[k]
    return out


def print_report(s: dict, skipped) -> None:
    d = s["diversity"]
    print(f"\n== {s['intrinsics']} intrinsics: {s['samples']} samples"
          + (f" ({len(skipped)} skipped: " + ", ".join(f"#{i} {why}" for i, why in skipped) + ")" if skipped else ""))
    print(f"   rotation about end X/Y/Z: {d['max_rotation_about_end_xyz_deg']} deg (RealMan: > 30 each) "
          f"-> {'enough' if d['enough'] else 'NOT ENOUGH'}")
    tilts = s["board_tilt_deg"]
    print(f"   board tilt to the image plane: {min(tilts):.0f}-{max(tilts):.0f} deg; range "
          f"{min(s['board_range_m']):.2f}-{max(s['board_range_m']):.2f} m; per-image PnP "
          f"{max(s['board_pose_rms_px']):.2f} px worst")
    print(f"   {'method':11s} {'vs URDF: mm':>12s} {'deg':>6s} {'board spread mm':>16s} {'reproj px':>10s}")
    rows = list(s["methods"].items()) + ([("refined", s["refined"])] if "refined" in s else [])
    for m, v in rows:
        if "error" in v:
            print(f"   {m:11s} {v['error']}")
            continue
        vn = v.get("vs_nominal") or {}
        print(f"   {m:11s} {vn.get('translation_norm_mm', float('nan')):12.1f} {vn.get('rotation_deg', float('nan')):6.2f} "
              f"{v['board_spread']['translation_rms_mm']:16.2f} {v['reprojection_rms_px']:10.2f}")
    if "method_agreement" in s:
        a = s["method_agreement"]
        print(f"   five methods agree within {a['translation_max_mm']:.1f} mm / {a['rotation_max_deg']:.2f} deg")
    if "leave_one_out" in s:
        l = s["leave_one_out"]
        print(f"   leave-one-out ({l['method']}): {l['translation_max_mm']:.1f} mm / {l['rotation_max_deg']:.2f} deg")
    if "refined" in s:
        r = s["refined"]
        print(f"   refined correction: {r['vs_nominal']['translation_mm_in_end']} mm in the end frame, "
              f"{r['vs_nominal']['rotation_deg']:.2f} deg")
        if "end_frame_correction" in r:
            e = r["end_frame_correction"]
            print(f"   as a fixed joint {s.get('end_frame') or 'R_CameraHolderLink'} -> nominal mount: "
                  f"xyz {e['xyz_mm']} mm, rpy {e['rpy_deg']} deg")
        if "urdf_mount_origin" in r:
            o = r["urdf_mount_origin"]
            print(f"   or the mount <origin> replaced: xyz=\"{' '.join(f'{v:.5f}' for v in o['xyz'])}\" "
                  f"rpy=\"{' '.join(f'{v:.5f}' for v in o['rpy'])}\"  (nominal xyz=\"0.0 0.0625 0.011\" rpy=\"0 -pi/2 -pi/2\")")
        v = s["verdict"]
        print(f"   verdict: {'ACCEPTED' if v['accepted'] else 'NOT ACCEPTED'} "
              f"(board spread <= {v['spread_limit_mm']} mm, reprojection <= {v['reprojection_limit_px']} px, rotations enough)")


def run(data: Path, intrinsics: str = "factory", exclude=(), quiet: bool = False) -> dict:
    session = load_session(data)
    b = session.get("board") or {}
    board = he.Board(int(b.get("cols", 11)), int(b.get("rows", 8)), float(b.get("square_m", 0.030)))
    used, skipped = detect_all(data, session, board, set(int(i) for i in exclude))
    if len(used) < 3:
        raise SystemExit(f"only {len(used)} usable samples; RealMan asks for 15-20")
    info = session["camera_info"]
    K = np.array(info["K"], float).reshape(3, 3)
    D = np.array(info.get("D") or [0.0] * 5, float)
    nominal = session.get("nominal") or {}
    T_end_cam_nom = np.array(nominal["T_end_cam"], float) if nominal.get("T_end_cam") else None
    T_end_mount_nom = np.array(nominal["T_end_mount"], float) if nominal.get("T_end_mount") else None
    T_mount_cam_nom = he.inv(T_end_mount_nom) @ T_end_cam_nom if T_end_mount_nom is not None and T_end_cam_nom is not None else None

    result = dict(source="RealMan developer guide 'Hand-eye Calibration' (github.com/RealManRobot/rm_docs "
                         "docs/en/AI/developerGuide/hand/index.md @ f5444ba); alix_perception_core.hand_eye",
                  data=str(data), board=dict(cols=board.cols, rows=board.rows, square_m=board.square_m),
                  end_frame=session.get("end_frame"), camera_frame=session.get("camera_frame"),
                  base_frame=session.get("base_frame"), nominal=dict(T_end_cam=T_end_cam_nom, T_end_mount=T_end_mount_nom,
                                                                      T_end_depth=nominal.get("T_end_depth")),
                  nominal_urdf_mount_origin=NOMINAL_MOUNT, skipped=[dict(index=i, why=w) for i, w in skipped], runs={})
    runs = [("factory", K, D)]
    if intrinsics in ("calibrate", "both"):
        K2, D2, rms = he.calibrate_intrinsics([u["corners"] for u in used], board, used[0]["size"])
        result["calibrated_intrinsics"] = dict(K=K2, D=D2, rms_px=rms)
        runs = ([("calibrated", K2, D2)] if intrinsics == "calibrate" else runs + [("calibrated", K2, D2)])
    for name, Kx, Dx in runs:
        res = he.calibrate(used, board, Kx, Dx, T_end_cam_nom)
        s = summarise(name, res, T_mount_cam_nom, T_end_cam_nom)
        result["runs"][name] = s
        if not quiet:
            print_report(s, skipped)
    best = result["runs"].get("factory") or next(iter(result["runs"].values()))
    if "refined" in best:
        T = best["refined"]["T_end_cam"]
        result["recommended"] = dict(run=best["intrinsics"], method="refined", T_end_cam=T,
                                     T_end_depth=(T @ he.inv(T_end_cam_nom) @ np.array(nominal["T_end_depth"], float)
                                                  if nominal.get("T_end_depth") and T_end_cam_nom is not None else None),
                                     accepted=best["verdict"]["accepted"])
    (data / "hand_eye_result.yaml").write_text(yaml.safe_dump(plain(result), sort_keys=False, width=140))
    if not quiet:
        print(f"\nwrote {data / 'hand_eye_result.yaml'}")
    return result


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", required=True, type=Path, help="session folder with samples.yaml and images/")
    ap.add_argument("--intrinsics", choices=("factory", "calibrate", "both"), default="both",
                    help="factory = the camera's CameraInfo; calibrate = RealMan's calibrateCamera; both = report both")
    ap.add_argument("--exclude", nargs="*", type=int, default=[], help="sample indices to leave out")
    a = ap.parse_args()
    run(a.data, a.intrinsics, a.exclude)
    return 0


if __name__ == "__main__":
    sys.exit(main())
