"""NFR-THEME guards for the web UI:

- the ALIX token blocks in tokens.css are ALIX's design/_tokens.css, unchanged;
- no colour literal appears in the UI's CSS or JS outside tokens.css;
- the page loads nothing from the network.
"""
import re
from pathlib import Path

from alix_studio_lite import vendor_paths

WEB = vendor_paths.LITE_ROOT / "src/alix_studio_lite/web"
LITERAL = re.compile(r"(?<![\w&])#[0-9a-fA-F]{3,8}\b|\brgba?\(|\b0x[0-9a-fA-F]{6}\b")


def test_alix_tokens_are_verbatim():
    alix = (vendor_paths.ALIX / "design/_tokens.css").read_text(encoding="utf-8")
    block = alix[alix.index(":root {"):alix.index("*{box-sizing:border-box}")].rstrip()
    assert block in (WEB / "css/tokens.css").read_text(encoding="utf-8")


def test_no_colour_literals_outside_tokens():
    bad = []
    for f in [*sorted((WEB / "css").glob("*.css")), *sorted((WEB / "js").rglob("*.js"))]:
        if f.name == "tokens.css":
            continue
        for i, line in enumerate(f.read_text(encoding="utf-8").splitlines(), 1):
            if LITERAL.search(line):
                bad.append(f"{f.relative_to(WEB)}:{i}: {line.strip()[:90]}")
    assert not bad, "\n".join(bad)


def test_no_remote_resources():
    html = (WEB / "index.html").read_text(encoding="utf-8")
    assert not re.search(r"""(src|href)=["']https?://""", html)
    for f in (WEB / "js").rglob("*.js"):
        assert "http://" not in f.read_text(encoding="utf-8") and "https://" not in f.read_text(encoding="utf-8"), f
