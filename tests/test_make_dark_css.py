"""tools/make_dark_css.py on small pages (today's page is checked by tests/test_lite_ui.py):

- a selector list splits only at its top-level commas, not inside (), [] or a string;
- each rule keeps its place in the page's order, inside its own copy of the @media and @supports
  rules around it, so the same rule wins in the dark theme as on the page.
"""
import importlib.util

from alix_studio_lite import vendor_paths

SPEC = importlib.util.spec_from_file_location("make_dark_css", vendor_paths.LITE_ROOT / "tools/make_dark_css.py")
mdc = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(mdc)
S = ':where(html[data-theme="dark"])'


def dark(tmp_path, css: str) -> str:
    """dark.css below its header, for a page whose stylesheet is `css`."""
    page = tmp_path / "index.html"
    page.write_text(f"<style>:root{{}}{css}</style>", encoding="utf-8")
    text, _ = mdc.build(page)
    return text.split("*/\n\n", 1)[1]


def test_selector_lists_split_only_at_top_level_commas():
    assert mdc.selectors(".a:not(.b, .hero-art) .c, .d") == [".a:not(.b, .hero-art) .c", ".d"]
    assert mdc.selectors('.e:is(.f, [g=","]), [title="a], b"], .h\\,i') == [
        '.e:is(.f, [g=","])', '[title="a], b"]', ".h\\,i"]


def test_a_comma_inside_not_leaves_later_rules_intact(tmp_path):
    """Split at every comma, .a:not(.b, .hero-art) .c came out as `.a:not(.b { … }`, whose open
    bracket swallowed every later rule. Whole, it names .hero-art, so it is kept as the designer
    made it; .a:not(.b, .c) .d is converted whole."""
    assert dark(tmp_path, ".a:not(.b, .hero-art) .c{color:#111}"
                          ".a:not(.b, .c) .d{color:#111}"
                          '.e:is(.f, .g), [title="a], b"], .h{background:#fff}'
                          ".k{color:#1d4ed8}") == (
        f"{S} .a:not(.b, .c) .d {{ color: var(--ink); }}\n"
        f"{S} .e:is(.f, .g),\n"
        f'{S} [title="a], b"],\n'
        f"{S} .h {{ background: var(--card); }}\n"
        f"{S} .k {{ color: var(--lite-d-blue); }}\n")


def test_v19s_own_dark_selectors_split_the_same_way(tmp_path):
    """A selector V19's dark theme already paints is left to it, also with a comma inside :not()."""
    assert dark(tmp_path, 'html[data-theme="dark"] .x:not(.y, .z){background:#111}'
                          ".x:not(.y, .w), .x:not(.y, .z){background:#fff}") == (
        'html[data-theme="dark"] .x:not(.y, .w) { background: var(--card); }\n')


def test_media_rules_keep_the_pages_order(tmp_path):
    """Grouped by condition, the .q blocks (1100, 900, 1100 px) came out with the 900 px one last:
    at 900 px and below its red won, where the page shows the last block's green. And .s, which the
    page sets after its @media block, came out before it, so the block won at 700 px and below."""
    assert dark(tmp_path, ".q{color:#111}"
                          "@media(max-width:1100px){.q{color:#1d4ed8}}"
                          "@media(max-width:900px){.q{color:#b91c1c}}"
                          "@media(max-width:1100px){.q{color:#15803d}}"
                          "@media(max-width:700px){.s{color:#1d4ed8}}"
                          ".s{color:#111}") == (
        f"{S} .q {{ color: var(--ink); }}\n"
        "@media(max-width:1100px) {\n"
        f"  {S} .q {{ color: var(--lite-d-blue); }}\n"
        "}\n"
        "@media(max-width:900px) {\n"
        f"  {S} .q {{ color: var(--lite-d-red); }}\n"
        "}\n"
        "@media(max-width:1100px) {\n"
        f"  {S} .q {{ color: var(--lite-d-green); }}\n"
        "}\n"
        "@media(max-width:700px) {\n"
        f"  {S} .s {{ color: var(--lite-d-blue); }}\n"
        "}\n"
        f"{S} .s {{ color: var(--ink); }}\n")


def test_nested_at_rules_keep_their_outer_condition(tmp_path):
    """@supports inside @media stays inside both, between its neighbours. It used to lose the @media,
    so it applied at every width, and come after them."""
    assert dark(tmp_path, "@media(max-width:900px){.m{color:#111}"
                          "@supports (display:grid){.n,.n2{color:#111}}"
                          ".o{color:#111}}") == (
        "@media(max-width:900px) {\n"
        f"  {S} .m {{ color: var(--ink); }}\n"
        "}\n"
        "@media(max-width:900px) {\n"
        "  @supports (display:grid) {\n"
        f"    {S} .n,\n"
        f"    {S} .n2 {{ color: var(--ink); }}\n"
        "  }\n"
        "}\n"
        "@media(max-width:900px) {\n"
        f"  {S} .o {{ color: var(--ink); }}\n"
        "}\n")
