#!/usr/bin/env python3
"""Write src/alix_studio_lite/web/lite/dark.css: the Lite UI's dark theme, completing the designer's.

The designer's page (web/index.html, V19) has a dark theme, html[data-theme="dark"]: it recolours
the page's variables and 24 components. Every other rule that paints a light background or dark
text stays light in the dark theme: white task numbers on white circles, white top-bar buttons,
white cards with light text, dark labels and badges on dark tints.

This reads every rule of the page's stylesheet and writes, for each one that paints a light
background, a light border or dark text, the same selector under html[data-theme="dark"] with the
colour moved onto the dark palette by its hue:

    light neutral background   ->  --card (white), --lite-d-sunken (grey panels), --lite-d-raised (controls)
    light tinted background    ->  --blue2, --green2, --orange2, --red2, --lite-d-violet2 (dark tints)
    translucent white          ->  --lite-d-glass (over the 3D view)
    dark text                  ->  --ink, --lite-d-ink2, --muted, or a light tint of its hue
    light border               ->  --line, or a dark edge of its hue (V19's borders are pastel)

The values are V19's own dark ones (web/lite/tokens.css), so the page keeps its look. Image frames
stay white, as V19 itself keeps them (its reference pictures are white).

The cascade stays the page's: each rule is scoped with :where(html[data-theme="dark"]), which adds
no specificity, so a state rule the page made stronger (.tab.active over .tab) still wins over its
base. Only rules that must beat one of V19's own dark rules (html[data-theme="dark"] .task-mini …)
carry the full prefix. Each rule keeps its place in the page's order, inside its own copy of the
@media and @supports rules around it. The page itself is never changed (tests/test_lite_ui.py).
Run this again when the designer's page changes:

    python3 tools/make_dark_css.py
"""
from __future__ import annotations

import colorsys
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PAGE = ROOT / "src/alix_studio_lite/web/index.html"
OUT = ROOT / "src/alix_studio_lite/web/lite/dark.css"
DARK = 'html[data-theme="dark"]'
SOFT = ':where(html[data-theme="dark"])'

# Kept as the designer made them: picture frames (the pictures are white), and the log-in and
# project screens' backdrops, which V19's dark theme already handles.
KEEP = re.compile(r"teach-reference-image-wrap|visual-carousel(?!-caption)|image-zoom|hero-art|\bimg\b|"
                  r"operator-login-screen|project-manager-screen|::selection|data-theme")
CONTROL = re.compile(r"btn|button|key|\btab|tab\b|toggle|pipe|chip|decision|pill|segment")
NAMED = {"white": (255, 255, 255, 1.0), "black": (0, 0, 0, 1.0)}


def strip_comments(css: str) -> str:
    return re.sub(r"/\*.*?\*/", "", css, flags=re.S)


def selectors(sel: str) -> list[str]:
    """A selector list's selectors. Only its top-level commas separate them: .a:not(.b, .c) and
    [title="x, y"] are one selector each."""
    out, depth, quote, start, i = [], 0, "", 0, 0
    while i < len(sel):
        ch = sel[i]
        if ch == "\\":
            i += 1                                                    # an escaped character: .a\,b
        elif quote:
            quote = "" if ch == quote else quote
        elif ch in "\"'":
            quote = ch
        elif ch in "([":
            depth += 1
        elif ch in ")]":
            depth -= 1
        elif ch == "," and not depth:
            out.append(sel[start:i])
            start = i + 1
        i += 1
    out.append(sel[start:])
    return [s.strip() for s in out if s.strip()]


def rules(css: str):
    """(at-rules around it, selector, declarations) for every style rule, in the page's order; the
    at-rules are the @media and @supports preludes it sits in, outermost first. @keyframes skipped."""
    out, i, n = [], 0, len(css)

    def block_end(j):
        depth = 0
        while j < n:
            if css[j] == "{":
                depth += 1
            elif css[j] == "}":
                depth -= 1
                if depth == 0:
                    return j
            j += 1
        return n

    def walk(lo, hi, at_rules):
        j = lo
        while j < hi:
            k = css.find("{", j, hi)
            if k < 0:
                return
            prelude = css[j:k].strip()
            end = block_end(k)
            if prelude.startswith("@media") or prelude.startswith("@supports"):
                walk(k + 1, end, at_rules + (prelude,))               # a nested one keeps the outer ones
            elif prelude.startswith("@"):
                pass                                                  # @keyframes, @font-face, @page
            elif prelude:
                out.append((at_rules, prelude, css[k + 1:end]))
            j = end + 1

    walk(0, n, ())
    return out


def parse_colour(tok: str):
    tok = tok.strip().lower()
    if tok in NAMED:
        return NAMED[tok]
    m = re.fullmatch(r"#([0-9a-f]{3,8})", tok)
    if m:
        h = m.group(1)
        if len(h) in (3, 4):
            h = "".join(c * 2 for c in h)
        r, g, b = (int(h[i:i + 2], 16) for i in (0, 2, 4))
        a = int(h[6:8], 16) / 255 if len(h) == 8 else 1.0
        return r, g, b, a
    m = re.fullmatch(r"rgba?\(([^)]*)\)", tok)
    if m:
        p = [x for x in re.split(r"[\s,/]+", m.group(1).strip()) if x]
        try:
            r, g, b = (float(x) for x in p[:3])
            a = float(p[3].rstrip("%")) / (100 if p[3].endswith("%") else 1) if len(p) > 3 else 1.0
        except ValueError:
            return None
        return r, g, b, a
    return None


COLOUR = re.compile(r"#[0-9a-fA-F]{3,8}\b|rgba?\([^)]*\)|\bwhite\b|\bblack\b")


def lum(c) -> float:
    def ch(v):
        v /= 255
        return v / 12.92 if v <= 0.03928 else ((v + 0.055) / 1.055) ** 2.4
    r, g, b = c[:3]
    return 0.2126 * ch(r) + 0.7152 * ch(g) + 0.0722 * ch(b)


def hue_chroma(c):
    r, g, b = (x / 255 for x in c[:3])
    h, _, _ = colorsys.rgb_to_hls(r, g, b)
    return h * 360, max(r, g, b) - min(r, g, b)


def family(c, light: bool) -> str:
    """neutral, blue, cyan, violet, green, orange or red."""
    h, chroma = hue_chroma(c)
    if light:
        if chroma < (0.045 if 195 <= h <= 235 else 0.025):           # blue-greys are V19's neutrals
            return "neutral"
    elif chroma < 0.2:
        return "neutral"
    if 175 <= h < 200:
        return "cyan"
    if 200 <= h < 245:
        return "blue"
    if 245 <= h < 320:
        return "violet"
    if 65 <= h < 175:
        return "green"
    if 15 <= h < 65:
        return "orange"
    return "red"


DARK_TINT = {"blue": "var(--blue2)", "cyan": "var(--blue2)", "violet": "var(--lite-d-violet2)",
             "green": "var(--green2)", "orange": "var(--orange2)", "red": "var(--red2)"}
LIGHT_TEXT = {"blue": "var(--lite-d-blue)", "cyan": "var(--lite-d-cyan)", "violet": "var(--lite-d-violet)",
              "green": "var(--lite-d-green)", "orange": "var(--lite-d-orange)", "red": "var(--lite-d-red)"}
EDGE = {f: f"var(--lite-d-{f}-edge)" for f in ("blue", "cyan", "violet", "green", "orange", "red")}


def dark_background(c, selector: str) -> str | None:
    if lum(c) < 0.5:
        return None                                                   # already dark, or an accent
    if c[3] < 0.99:
        return "var(--lite-d-glass)"
    fam = family(c, light=True)
    if fam != "neutral":
        return DARK_TINT[fam]
    if CONTROL.search(selector):
        return "var(--lite-d-raised)"
    return "var(--card)" if min(c[:3]) >= 254 else "var(--lite-d-sunken)"


def dark_text(c) -> str | None:
    L = lum(c)
    if L >= 0.2:
        return None
    fam = family(c, light=False)
    if fam != "neutral":
        return LIGHT_TEXT[fam]
    return "var(--ink)" if L < 0.03 else "var(--lite-d-ink2)" if L < 0.12 else "var(--muted)"


def dark_border(c) -> str | None:
    if lum(c) < 0.45:
        return None
    fam = family(c, light=True)
    return "var(--line)" if fam == "neutral" else EDGE[fam]


ACCENTS: dict[str, str] = {}        # the page's accent variables the dark theme keeps (--blue: #2563eb, …)


def resolve(v: str) -> str:
    """An accent variable as its value, so it is judged like a written colour: --blue stays
    #2563eb in V19's dark theme, too dark as text on a dark surface."""
    return re.sub(r"var\((--[\w-]+)\)", lambda m: ACCENTS.get(m.group(1), m.group(0)), v)


def declarations(body: str):
    body = re.sub(r"url\([^)]*\)", "url()", body)
    for d in body.split(";"):
        if ":" in d:
            k, v = d.split(":", 1)
            yield k.strip().lower(), resolve(v.strip())


def convert(selector: str, body: str) -> list[str]:
    out = []
    bg_kept = False
    decls = list(declarations(body))
    for k, v in decls:
        imp = " !important" if "!important" in v else ""
        v = v.replace("!important", "").strip()
        if k in ("background", "background-color"):
            cols = [parse_colour(t) for t in COLOUR.findall(v)]
            cols = [c for c in cols if c]
            if not cols:
                continue
            if all(lum(c) >= 0.5 for c in cols):
                to = dark_background(cols[0], selector)
                prop = "background-color" if "url(" in v else "background"
                out.append(f"{prop}: {to}{imp}")
            else:
                bg_kept = True                                        # an accent or a dark gradient
        elif k in ("border", "border-color", "border-top", "border-right", "border-bottom", "border-left",
                   "border-top-color", "border-right-color", "border-bottom-color", "border-left-color"):
            cols = [parse_colour(t) for t in COLOUR.findall(v)]
            cols = [c for c in cols if c]
            if len(cols) == 1:
                to = dark_border(cols[0])
                if to:
                    prop = k if k.endswith("-color") else ("border-color" if k == "border" else f"{k}-color")
                    out.append(f"{prop}: {to}{imp}")
    for k, v in decls:
        if k != "color":
            continue
        imp = " !important" if "!important" in v else ""
        c = parse_colour(v.replace("!important", "").strip())
        if c and not bg_kept:                                         # dark text on an accent stays
            to = dark_text(c)
            if to:
                out.append(f"color: {to}{imp}")
    return out


def build(page: Path = PAGE) -> tuple[str, int]:
    """dark.css's text for the page, and its number of rules."""
    html = page.read_text(encoding="utf-8")
    css = strip_comments("\n".join(re.findall(r"<style[^>]*>(.*?)</style>", html, flags=re.S)))
    root = re.search(r":root\s*\{([^}]*)\}", css).group(1)
    redefined = set()
    for _, sel, body in rules(css):
        if "data-theme" in sel:
            redefined.update(re.findall(r"(--[\w-]+)\s*:", body))
    ACCENTS.clear()
    for name, value in re.findall(r"(--[\w-]+)\s*:\s*([^;]+)", root):
        if name not in redefined and parse_colour(value.strip()):
            ACCENTS[name] = value.strip()
    v19_dark = set()
    for _, sel, _ in rules(css):
        if "data-theme" in sel:
            v19_dark.update(s.replace(DARK, "").strip() for s in selectors(sel))
    # a rule touching an element one of V19's dark rules paints must outrank that rule
    simple = sorted({t for s in v19_dark for t in re.findall(r"\.[\w-]+", s)})
    strong = re.compile("|".join(re.escape(t) + r"(?![\w-])" for t in simple)) if simple else None
    out: list[str] = []
    for at_rules, sel, body in rules(css):
        sels = selectors(sel)
        sels = [s for s in sels if not KEEP.search(s) and s not in v19_dark and not s.startswith(("html", ":root"))]
        if not sels:
            continue
        decls = convert(",".join(sels), body)
        if not decls:
            continue
        scoped = ",\n".join(f"{DARK if strong and strong.search(s) else SOFT} {s}" for s in sels)
        rule = f"{scoped} {{ {'; '.join(decls)}; }}"
        # wrapped in its own @media/@supports, in the page's order, so the same rule wins as on the page
        for at in reversed(at_rules):
            rule = f"{at} {{\n  " + rule.replace("\n", "\n  ") + "\n}"
        out.append(rule)
    lines = [
        "/* GENERATED by tools/make_dark_css.py from web/index.html (the designer's V19): do not edit.",
        "   The Lite UI's dark theme, completing V19's own: every page rule that paints a light",
        "   background, a light border or dark text, moved onto V19's dark palette by its hue",
        "   (web/lite/tokens.css). Lite's own additions are in lite.css. */",
        "",
    ]
    return "\n".join(lines + out) + "\n", len(out)


def main() -> None:
    text, count = build()
    OUT.write_text(text, encoding="utf-8")
    print(f"{OUT.relative_to(ROOT)}: {count} rules")


if __name__ == "__main__":
    main()
