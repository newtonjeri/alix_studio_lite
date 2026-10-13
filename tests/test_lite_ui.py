"""The Lite UI page (``web/index.html``) is the designer's V19 file, unchanged except for one
marked block that loads Lite's bridge (``web/lite/``):

- the page minus that block is byte-identical to ``docs/reference/ui_v19/``;
- every page function the bridge replaces still exists in the page (a new designer version
  that renames one fails here, not on the robot);
- nothing loads from the network, and the bridge uses the page's colour variables only.
"""
import re

from alix_studio_lite import vendor_paths

WEB = vendor_paths.LITE_ROOT / "src/alix_studio_lite/web"
ORIGINAL = vendor_paths.LITE_ROOT / "docs/reference/ui_v19/ALIX_Studio_LITE_Edition_V19_Project_Manager.html"
BLOCK = (b"<!-- ALIX Studio Lite: the designer's page above is unchanged; Lite's link to the backend is web/lite/ -->\n"
         b"<link rel=\"stylesheet\" href=\"lite/lite.css\">\n"
         b"<script type=\"module\" src=\"lite/main.js\"></script>\n")
LITERAL = re.compile(r"(?<![\w&])#[0-9a-fA-F]{3,8}\b|\brgba?\(|\b0x[0-9a-fA-F]{6}\b")


def bridge_files():
    return sorted((WEB / "lite").glob("*.js"))


def test_page_is_the_designers_file_plus_the_bridge():
    page = (WEB / "index.html").read_bytes()
    assert page.count(BLOCK) == 1
    assert page.replace(BLOCK, b"") == ORIGINAL.read_bytes()


def test_replaced_functions_exist_in_the_page():
    page = (WEB / "index.html").read_text(encoding="utf-8")
    declared = set(re.findall(r"\bfunction\s+([A-Za-z_]\w*)\s*\(", page))
    replaced = set()
    for f in bridge_files():
        src = f.read_text(encoding="utf-8")
        replaced |= set(re.findall(r"\bwindow\.([A-Za-z_]\w*)\s*=(?!=)", src))
        for group in re.findall(r"for \(const name of \[([^\]]*)\]\)", src):
            replaced |= set(re.findall(r"'(\w+)'", group))
    assert replaced, "the bridge replaces nothing?"
    missing = sorted(replaced - declared)
    assert not missing, f"page functions the bridge replaces are gone: {missing}"


def test_no_remote_resources():
    page = (WEB / "index.html").read_text(encoding="utf-8")
    assert not re.search(r"""(src|href)=["']https?://""", page)
    for f in [*bridge_files(), *(WEB / "lite").glob("*.css")]:
        text = f.read_text(encoding="utf-8").replace("http://www.w3.org/2000/svg", "")   # a name, never fetched
        assert "http://" not in text and "https://" not in text, f


def test_bridge_uses_the_pages_colours():
    """Colour values live in the page's variables, and the 3D view's palette in lite/tokens.css."""
    bad = []
    for f in [*bridge_files(), *sorted(p for p in (WEB / "lite").glob("*.css") if p.name != "tokens.css")]:
        for i, line in enumerate(f.read_text(encoding="utf-8").splitlines(), 1):
            if LITERAL.search(line):
                bad.append(f"{f.name}:{i}: {line.strip()[:90]}")
    assert not bad, "\n".join(bad)


def test_dark_theme_is_generated_from_the_page():
    """web/lite/dark.css completes the page's dark theme and is tools/make_dark_css.py's output for
    the page as it is: rerun the tool when the designer's page changes."""
    import importlib.util

    tool = WEB.parents[2] / "tools/make_dark_css.py"
    spec = importlib.util.spec_from_file_location("make_dark_css", tool)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    text, count = mod.build(WEB / "index.html")
    assert count > 150
    assert (WEB / "lite/dark.css").read_text(encoding="utf-8") == text, "run: python3 tools/make_dark_css.py"
    assert '@import url("dark.css")' in (WEB / "lite/lite.css").read_text(encoding="utf-8")
