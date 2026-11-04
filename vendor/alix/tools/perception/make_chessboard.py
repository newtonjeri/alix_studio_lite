#!/usr/bin/env python3
"""A print-ready chessboard for the hand-eye calibration — RealMan's default board.

    python3 tools/perception/make_chessboard.py --out board_12x9_30mm.svg

12 x 9 squares of 30 mm (11 x 8 inner corners), black top-left square, on an A3 landscape
page in real millimetres. Print at 100 % ("actual size", never "fit to page"), check the
100 mm scale bar with a ruler, measure ten squares with calipers and pass the measured
square to `capture_hand_eye.py --square-mm`. Glue it flat to a rigid sheet: a bent board
is a wrong board. The square counts must be even x odd, so the two diagonal outer corners
differ in colour; that is how the tools tell the board's ends apart at any wrist roll.
"""
import argparse
from pathlib import Path


def svg(cols_sq: int, rows_sq: int, square_mm: float, page=(420.0, 297.0), margin=10.0) -> str:
    w, h = cols_sq * square_mm, rows_sq * square_mm
    if w + 2 * margin > page[0] or h + 2 * margin > page[1]:
        raise SystemExit(f"{w:.0f} x {h:.0f} mm does not fit the {page[0]:.0f} x {page[1]:.0f} mm page")
    x0, y0 = margin, (page[1] - h) / 2
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{page[0]}mm" height="{page[1]}mm" '
             f'viewBox="0 0 {page[0]} {page[1]}">',
             f'<rect x="0" y="0" width="{page[0]}" height="{page[1]}" fill="white"/>']
    for r in range(rows_sq):
        for c in range(cols_sq):
            if (r + c) % 2 == 0:
                parts.append(f'<rect x="{x0 + c * square_mm:.3f}" y="{y0 + r * square_mm:.3f}" '
                             f'width="{square_mm:.3f}" height="{square_mm:.3f}" fill="black"/>')
    sx = x0 + w + 8.0                                   # the side strip: scale bar and caption
    if sx + 12 < page[0]:
        parts.append(f'<rect x="{sx:.1f}" y="{y0:.1f}" width="2" height="100" fill="black"/>')
        for k in range(11):
            parts.append(f'<rect x="{sx:.1f}" y="{y0 + 10 * k:.1f}" width="{5 if k % 5 == 0 else 3.5}" height="0.3" fill="black"/>')
        parts.append(f'<text x="{sx + 8:.1f}" y="{y0 + 4:.1f}" font-size="3.5" font-family="sans-serif" '
                     f'transform="rotate(90 {sx + 8:.1f} {y0 + 4:.1f})">100 mm scale bar | {cols_sq} x {rows_sq} squares '
                     f'of {square_mm:g} mm = {cols_sq - 1} x {rows_sq - 1} inner corners | print at 100 %</text>')
    parts.append("</svg>")
    return "\n".join(parts)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--squares", default="12x9", help="squares along x along y (even x odd)")
    ap.add_argument("--square-mm", type=float, default=30.0)
    ap.add_argument("--out", type=Path, default=Path("board_12x9_30mm.svg"))
    a = ap.parse_args()
    cols_sq, rows_sq = (int(v) for v in a.squares.lower().split("x"))
    if (cols_sq + rows_sq) % 2 == 0:
        ap.error("the square counts must be even x odd, or the board looks the same turned 180 deg")
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(svg(cols_sq, rows_sq, a.square_mm))
    print(f"wrote {a.out}: {cols_sq} x {rows_sq} squares of {a.square_mm:g} mm "
          f"(--cols {cols_sq - 1} --rows {rows_sq - 1} --square-mm <measured>)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
