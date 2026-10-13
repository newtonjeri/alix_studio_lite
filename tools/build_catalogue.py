#!/usr/bin/env python3
"""Build the app's region catalogue from the sign-off sheet.

    python3 tools/build_catalogue.py      # docs/region_catalogue_commode_c.csv -> assets/regions/commode_c_catalogue.json

The CSV is the document people sign off; the JSON is what Lite reads. Re-run after any change
to the CSV. Each unit gets its deck picture (``assets/regions/deck/s<slide>-T<task>-<L|R|LR>.jpg``).
"""
from __future__ import annotations

import csv
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CSV = ROOT / "docs/region_catalogue_commode_c.csv"
OUT = ROOT / "assets/regions/commode_c_catalogue.json"
DECK = ROOT / "assets/regions/deck"

KIND = {"Cleaning": "cleaning", "Lid/seat operation (not cleaning)": "lid_seat"}
#: the toilet's state while a step runs, as the operator is asked about it
PHASES = [
    ("lid_closed", "Lid closed", "Is the lid closed?"),
    ("lid_open", "Lid open, seat down", "Is the lid UP and the seat DOWN?"),
    ("seat_open", "Lid and seat up", "Are the lid and the seat both UP?"),
]


def phase_of(state: str) -> str | None:
    s = state.lower()
    if "→" in s or "any" in s:
        return None                    # an operation between phases, or not tied to one
    if s.startswith("lid closed"):
        return "lid_closed"
    if s.startswith("lid open"):
        return "lid_open"
    if s.startswith("seat open"):
        return "seat_open"
    return None


def picture(slide: str, task: str, arm: str) -> str | None:
    first = task.split("-")[0]
    side = "L" if arm == "LEFT" else "R"
    for name in (f"s{int(slide):02d}-T{first}-{side}.jpg", f"s{int(slide):02d}-T{first}-LR.jpg"):
        if (DECK / name).is_file():
            return name
    return None


def hand_counts(text: str) -> list[int] | None:
    m = re.search(r"([\d,\s]+)$", text or "")
    if not m:
        return None
    vals = [int(v) for v in m.group(1).replace(" ", "").split(",") if v]
    return vals if len(vals) == 6 else None


def main() -> int:
    units = []
    for r in csv.DictReader(CSV.open(encoding="utf-8")):
        kind = KIND.get(r["type"], "other")
        units.append(dict(
            key=r["key"], code=r["code"], part=r["part"] or None, name=r["name"].strip(),
            arm=r["arm"].lower(), step=int(r["step"]), deck_task=r["deck_task"], deck_slide=int(r["deck_slide"]),
            type=r["type"], kind=kind, toilet_state=r["toilet_state"], phase=phase_of(r["toilet_state"]),
            first_release=r["first_release"].strip().lower() == "yes",
            pole_mm=int(r["pole_mm_proposed"]) if r["pole_mm_proposed"].strip() else None,
            hand_preset="GRIP" if r["hand_preset_proposed"].upper().startswith("GRIP") else None,
            hand_counts=hand_counts(r["hand_preset_proposed"]),
            target_s=float(r["target_time_s"]) if r["target_time_s"].strip() else None,
            legacy_v6=r["v6_coverage"], note=r["deck_note"],
            picture=picture(r["deck_slide"], r["deck_task"], r["arm"]),
            slide_picture=f"slide-{int(r['deck_slide']):02d}.jpg"))
    missing = [u["key"] for u in units if u["first_release"] and not u["picture"]]
    doc = dict(fixture="commode_c", deck="Toilet_regions_V1.pptx", source=str(CSV.relative_to(ROOT)),
               phases=[dict(id=p, label=l, question=q) for p, l, q in PHASES],
               hand_presets={"GRIP": [0, 0, 0, 0, 150, 1000], "ALL OPEN": [1000] * 6},
               units=units)
    OUT.write_text(json.dumps(doc, indent=1, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"{OUT.relative_to(ROOT)}: {len(units)} units, "
          f"{sum(u['first_release'] for u in units)} in the first release"
          + (f"; no picture for {missing}" if missing else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
