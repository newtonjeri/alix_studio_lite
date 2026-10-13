#!/usr/bin/env python3
"""Build a single-cubicle wall model (STL + venue YAML) for ALIX Studio Lite.

The cubicle follows ALIX's standalone planning-scene cubicles
(`washroom_800mm_planning_scene.stl`, `washroom_1000mm_planning_scene.stl` in
`alix_description/models/meshes/`): three axis-aligned boxes, 2 m high,
0.1 m thick.

    back wall      x [-0.100, 0.000]   y [-(w/2 + 0.01), +(w/2 + 0.01)]
    side wall +y   x [-0.103, 1.879]   y [+w/2, +w/2 + 0.1]
    side wall -y   x [-0.098, 1.884]   y [-(w/2 + 0.1), -w/2]

where w is the inner width between the side walls. With w = 1.0 this rule
reproduces ALIX's 1000 mm cubicle exactly (checked by tests/test_cubicles.py).
The venue YAML uses the same pose as ALIX's 800 mm and 1000 mm cubicles:
position [1.215, 0, 0], yaw 180 deg, so the back wall's inner face sits
1.215 m in front of base_link.

Usage:
    python3 tools/make_cubicle.py 1400            # writes assets/scene/cubicles/cubicle_1400mm.{stl,yaml}
    python3 tools/make_cubicle.py 1000 --out DIR  # any width, any folder
"""
from __future__ import annotations

import argparse
import struct
from pathlib import Path

HEIGHT_M = 2.0
WALL_M = 0.1
BACK_OVERLAP_M = 0.01
POSE_POSITION = (1.215, 0.0, 0.0)
POSE_RPY_DEG = (0.0, 0.0, 180.0)


def wall_boxes(inner_width_m: float):
    """The three wall boxes as ((xmin, xmax), (ymin, ymax), (zmin, zmax)), in ALIX's order."""
    h = inner_width_m / 2.0
    z = (0.0, HEIGHT_M)
    return [
        ((-0.103, 1.879), (h, h + WALL_M), z),                                   # side wall +y
        ((-0.098, 1.884), (-(h + WALL_M), -h), z),                               # side wall -y
        ((-WALL_M, 0.0), (-(h + BACK_OVERLAP_M), h + BACK_OVERLAP_M), z),        # back wall
    ]


def box_triangles(box):
    """12 outward-facing triangles (normal, (v0, v1, v2)) for an axis-aligned box."""
    (x0, x1), (y0, y1), (z0, z1) = box
    faces = [
        ((-1, 0, 0), [(x0, y0, z0), (x0, y0, z1), (x0, y1, z1), (x0, y1, z0)]),
        ((1, 0, 0), [(x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1)]),
        ((0, -1, 0), [(x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1)]),
        ((0, 1, 0), [(x0, y1, z0), (x0, y1, z1), (x1, y1, z1), (x1, y1, z0)]),
        ((0, 0, -1), [(x0, y0, z0), (x0, y1, z0), (x1, y1, z0), (x1, y0, z0)]),
        ((0, 0, 1), [(x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]),
    ]
    tris = []
    for normal, (a, b, c, d) in faces:
        tris.append((normal, (a, b, c)))
        tris.append((normal, (a, c, d)))
    return tris


def write_binary_stl(path: Path, triangles, header: str) -> None:
    data = bytearray(header.encode("ascii")[:80].ljust(80, b" "))
    data += struct.pack("<I", len(triangles))
    for normal, verts in triangles:
        data += struct.pack("<3f", *normal)
        for v in verts:
            data += struct.pack("<3f", *v)
        data += struct.pack("<H", 0)
    path.write_bytes(bytes(data))


VENUE_TEMPLATE = """# =============================================================================
# {width} MM CUBICLE - single-cubicle wall model for ALIX Studio Lite
# =============================================================================
# Built by tools/make_cubicle.py the same way as ALIX's standalone 800 mm and
# 1000 mm planning-scene cubicles: a back wall 0.1 m thick at x = 0 (mesh
# frame) and two side walls 0.1 m thick, about 1.98 m deep and 2 m high, with
# inner side faces at y = +/-{half:.3f} m. Same pose as ALIX's 800 mm cubicle:
# the back wall's inner face sits 1.215 m in front of base_link (yaw 180 deg).
#
# Format: ALIX's venue scene format (washroom_scene.washroom_structure), so the
# vendored ALIX wall checker reads it unchanged. The mesh is a package:// URI in
# Lite's own asset package `alix_studio_lite` (assets/package.xml), which Lite
# registers with ALIX's resolver at start-up (src/alix_studio_lite/vendor_paths.py).
# {note}
# =============================================================================

washroom_scene:
  reference_frame: "base_link"

  washroom_structure:
    enabled: true
    pose:
      position: [{px}, {py}, {pz}]
      orientation_rpy_deg: [{rr}, {rp}, {ry}]
    mesh: "package://alix_studio_lite/scene/cubicles/{mesh}"
    scale: [1.0, 1.0, 1.0]
"""


def make(width_mm: int, out_dir: Path, note: str = "") -> tuple[Path, Path]:
    out_dir.mkdir(parents=True, exist_ok=True)
    stl = out_dir / f"cubicle_{width_mm}mm.stl"
    yml = out_dir / f"cubicle_{width_mm}mm.yaml"
    tris = [t for box in wall_boxes(width_mm / 1000.0) for t in box_triangles(box)]
    write_binary_stl(stl, tris, f"ALIX Studio Lite cubicle {width_mm} mm (make_cubicle.py)")
    yml.write_text(VENUE_TEMPLATE.format(
        width=width_mm, half=width_mm / 2000.0, note=note or "Generated cubicle.",
        px=POSE_POSITION[0], py=POSE_POSITION[1], pz=POSE_POSITION[2],
        rr=POSE_RPY_DEG[0], rp=POSE_RPY_DEG[1], ry=POSE_RPY_DEG[2], mesh=stl.name))
    return stl, yml


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("width_mm", type=int, help="inner width between the side walls, in mm")
    ap.add_argument("--out", type=Path, default=Path(__file__).resolve().parents[1] / "assets" / "scene" / "cubicles")
    ap.add_argument("--note", default="", help="one line added to the YAML header")
    args = ap.parse_args()
    stl, yml = make(args.width_mm, args.out, args.note)
    print(f"wrote {stl}\nwrote {yml}")


if __name__ == "__main__":
    main()
