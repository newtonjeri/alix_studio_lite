"""The Lite UI page (``web/index.html``) is the designer's V32 file (ASL), unchanged except for one
marked block that loads Lite's bridge (``web/lite/``):

- the page minus that block is byte-identical to ``docs/reference/ui_v32/``;
- every page function the bridge replaces still exists in the page (a new designer version
  that renames one fails here, not on the robot);
- nothing loads from the network, and the bridge uses the page's colour variables only.
"""
import re

from alix_studio_lite import vendor_paths

WEB = vendor_paths.LITE_ROOT / "src/alix_studio_lite/web"
ORIGINAL = vendor_paths.LITE_ROOT / "docs/reference/ui_v32/ASL_FINAL_Template_V32_AutoHide_Journey.html"
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


def test_the_neighbour_panels_use_the_pages_own_pictures_and_cards():
    """neighbours.js puts the previous and the next task beside the current picture in each arm card
    (the owner, 19 Oct), with the page's own crops (resolveRegionPair) in the page's own cards."""
    page = (WEB / "index.html").read_text(encoding="utf-8")
    for needle in ("function resolveRegionPair(", "const CUSTOM_REGION_IMAGES", "const SLIDES=", 'id="choiceLeft"', 'id="choiceRight"',
                   'class="choice-region-thumb left-thumb"', 'class="choice-region-thumb right-thumb"', 'class="choice-zoom-row"'):
        assert needle in page, needle
    src = (WEB / "lite/neighbours.js").read_text(encoding="utf-8")
    for needle in ("resolveRegionPair(", "CUSTOM_REGION_IMAGES[", "SLIDES[", "'.choice-region-thumb'", "lite:step", "pole_mm", "handPresetName("):
        assert needle in src, needle
    assert "installNeighbours()" in (WEB / "lite/main.js").read_text(encoding="utf-8")
    css = (WEB / "lite/lite.css").read_text(encoding="utf-8")
    assert ".lite-trio" in css and ".task-board.post-record .lite-side { display: none; }" in css


def test_drag_and_teach_uploads_an_existing_project_beside_its_countdown_and_shows_it_in_the_standard_view():
    """The owner, 21 Oct: an "upload existing project" button in Drag and Teach, next to the countdown's Start and Stop,
    and a viewport for an uploaded file "like the other trajectory-visualizing viewports" (vp.js, view3d.js)."""
    page = (WEB / "index.html").read_text(encoding="utf-8")
    for needle in ('class="timer-actions timer-actions-near-clock"', 'id="startTeachBtn"', 'id="stopTeachBtn"', 'id="timerResult"'):
        assert needle in page, needle
    teach = (WEB / "lite/teach.js").read_text(encoding="utf-8")
    for needle in ("'Upload existing project'", "($('stopTeachBtn') || startBtn).after(up, input)", "up.disabled = startBtn.disabled",
                   "source: 'upload'", "stageDialog("):
        assert needle in teach, needle
    view = (WEB / "lite/teachview.js").read_text(encoding="utf-8")
    for needle in ("frame($('liteTeach3dHost'))", "new Tools(", "showFile(host, u, 'raw'", "unitUrl(unit.key, 'collisions')",
                   "r.source !== 'upload'"):
        assert needle in view, needle
    assert "installTeachView()" in (WEB / "lite/main.js").read_text(encoding="utf-8")


def test_any_task_opens_at_any_time_and_the_task_left_resumes_where_it_was():
    """The owner, 21 Oct: no need to finish the current task first (the page's jumpTask refused while the arm was locked
    until Real Test); the task left is kept and resumes; a task left partway says IN PROGRESS."""
    page = (WEB / "index.html").read_text(encoding="utf-8")
    assert "Finish Real Test for the current '+selectedArm.toUpperCase()+' arm before opening another task." in page   # the page's own
    for needle in ("function captureCurrentTaskRecord(", "function taskQueueStatus(", "function armTaskStatusInfo(", "onclick=\"jumpTask(${i})\""):
        assert needle in page, needle
    src = (WEB / "lite/progress.js").read_text(encoding="utf-8")
    for needle in ("window.jumpTask = async function jumpTask(", "armSelectionLocked = false;", "rec.liteResume = {", "await resumeTask(i)",
                   "teachingNow()", "'IN PROGRESS'", "window.openApprovedTask = function"):
        assert needle in src, needle
    assert "installProgress()" in (WEB / "lite/main.js").read_text(encoding="utf-8")
    assert "leaveTask();" in (WEB / "lite/realtest.js").read_text(encoding="utf-8")       # Teach another task keeps it too


def test_a_delete_column_with_no_name_whose_bin_shows_on_the_hovered_row():
    """The owner, 21 Oct: delete anything stored on the controller from Data Management and from S1's merged files: one
    more column after the last, with no name; the icon shows on the row hovered, only there; always asked first."""
    dm = (WEB / "lite/datamgmt.js").read_text(encoding="utf-8")
    assert dm.count("['del', '', null, false]") == 3                     # programs, global points, tracks: the last column
    for needle in ("deleteStored(side, 'program'", "deleteStored(side, 'point'", "deleteStored(side, 'track'", "class: 'lite-del-row'"):
        assert needle in dm, needle
    s1 = (WEB / "lite/s1.js").read_text(encoding="utf-8")
    assert "binCell(side, m)" in s1 and "deleteStored(side, 'program', c.id" in s1 and "lite-del-row" in s1
    assert "() => notOnController(side, m)" in s1                        # every merged file has its bin (the owner, 21 Oct)
    ctx = (WEB / "lite/ctx.js").read_text(encoding="utf-8")
    assert "const yes = await confirmDialog({ title: `Delete ${what} from the ${S} controller?`" in ctx      # asked, every time
    assert "createElementNS(NS, 'svg')" in ctx                           # a drawn bin, not an emoji
    css = (WEB / "lite/lite.css").read_text(encoding="utf-8")
    assert ".lite-del-row:hover .lite-del, .lite-del-row:focus-within .lite-del" in css and "opacity: 0;" in css


def _calls(src, head):
    """Each call ``head(...)`` in JavaScript source: (line, the text of its arguments), strings and templates skipped."""
    def string_end(j):
        q, k = src[j], j + 1
        while k < len(src) and src[k] != q:
            k += 2 if src[k] == "\\" else 1
        return k + 1

    def template_end(j):
        while j < len(src):
            if src[j] == "\\":
                j += 2
            elif src[j] == "`":
                return j + 1
            elif src[j] == "$" and src[j + 1] == "{":
                j = nest_end(j + 2, "{", "}")
            else:
                j += 1
        raise ValueError("template not closed")

    def nest_end(j, opener, closer):
        depth = 1
        while j < len(src):
            c = src[j]
            if c in "'\"":
                j = string_end(j)
            elif c == "`":
                j = template_end(j + 1)
            else:
                if c == opener:
                    depth += 1
                elif c == closer:
                    depth -= 1
                    if depth == 0:
                        return j + 1
                j += 1
        raise ValueError("not closed")

    out, i = [], src.find(head + "(")
    while i >= 0:
        end = nest_end(i + len(head) + 1, "(", ")")
        out.append((src.count("\n", 0, i) + 1, src[i + len(head) + 1:end - 1]))
        i = src.find(head + "(", end)
    return out


def test_every_notification_lite_raises_says_its_level():
    """notify.js colours each notification by its level (the owner, 20 Oct: green good, orange warning, red error):
    Lite's own modules say it with the message, as the last argument, so none falls back to reading the words."""
    level = re.compile(r"(?:^'(?:good|warn|error)'$)|(?:\?\s*'(?:good|warn|error)'\s*:\s*'(?:good|warn|error)'$)")
    bad = []
    for f in bridge_files():
        if f.name in ("notify.js", "fullseq.js"):            # the definition, and a wrapper that passes the level on
            continue
        for line, args in _calls(f.read_text(encoding="utf-8"), "window.toast"):
            last = args.rsplit(",", 1)[-1].strip()
            if not level.search(last):
                bad.append(f"{f.name}:{line}: {args.strip()[:80]}")
    assert not bad, "notifications without a level:\n" + "\n".join(bad)


def test_s2_s_run_card_asks_only_the_two_safety_checks():
    """The owner, 23 Oct: the card shown when RUN starts on the robot listed the whole simulation, "that is not user
    friendly": it shows only the two bullets (REAL_CHECKS); the findings stay in the Simulation card."""
    s2 = (WEB / "lite/s2.js").read_text(encoding="utf-8")
    card = [args for _, args in _calls(s2, "confirmDialog") if "'Start on the robot'" in args]
    assert len(card) == 1, card
    assert "list: REAL_CHECKS" in card[0] and "body" not in card[0], card[0]
    assert "The simulation found:" not in s2 and "Each step is timed on the robot: one far outside" not in s2
    ctx = (WEB / "lite/ctx.js").read_text(encoding="utf-8")
    assert "body ? el('p', { text: body }) : null" in ctx                # no empty paragraph above the bullets


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
