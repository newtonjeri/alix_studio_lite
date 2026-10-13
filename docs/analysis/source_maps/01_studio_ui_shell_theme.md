# 01 — ALIX Studio UI shell, visual theme and UX infrastructure (evidence map for ALIX Studio Lite)

Scope: NFR-THEME, NFR-UX, R2 (5-step undo), R6 (resizable plots/windows), plus the shell pieces that R1/R3/R4/R5/R10/R13 will sit in.
Read-only survey. Line numbers are current as of the files on disk (editor.js 19,325 lines, editor.css 2,166, template.html 953).

## Path abbreviations (all absolute)

| Abbrev | Absolute path |
|---|---|
| `HTML` | `/home/newtonjeri/alix_ws/src/alix/core/python/alix_taskgen/src/alix_taskgen/web_editor/template.html` |
| `CSS` | `/home/newtonjeri/alix_ws/src/alix/core/python/alix_taskgen/src/alix_taskgen/web_editor/editor.css` |
| `JS` | `/home/newtonjeri/alix_ws/src/alix/core/python/alix_taskgen/src/alix_taskgen/web_editor/editor.js` |
| `VEND` | `/home/newtonjeri/alix_ws/src/alix/core/python/alix_taskgen/src/alix_taskgen/web_editor/vendor/` |
| `VIS` | `/home/newtonjeri/alix_ws/src/alix/core/python/alix_taskgen/src/alix_taskgen/scripts/visualize_generated_config.py` |
| `SRV` | `/home/newtonjeri/alix_ws/src/alix/core/python/alix_taskgen/src/alix_taskgen/server.py` |
| `TOK` | `/home/newtonjeri/alix_ws/src/alix/design/_tokens.css` (the design-system source) |
| `DSR` | `/home/newtonjeri/alix_ws/src/alix/design/README.md` |
| `TDT` | `/home/newtonjeri/alix_ws/src/alix/core/python/alix_taskgen/test/test_design_tokens.py` |
| `TST` | `/home/newtonjeri/alix_ws/src/alix/core/python/alix_taskgen/test/test_studio_theme.py` |
| `TIG` | `/home/newtonjeri/alix_ws/src/alix/core/python/alix_taskgen/test/test_studio_interaction_guards.py` |
| `RED` | `/home/newtonjeri/alix_ws/src/alix/docs/STUDIO_REDESIGN.md` (its own editor.js line refs are historical) |
| `RES` | `/home/newtonjeri/alix_ws/src/alix/docs/studio/STUDIO_UIUX_RESEARCH.md` |
| `STI` | `/home/newtonjeri/alix_ws/src/alix/docs/studio/STUDIO_STITCH_PROMPTS.md` |
| `SEAM` | `/home/newtonjeri/alix_ws/src/alix/docs/studio/STUDIO_SEAMS_PLAN.md` |
| `IMP` | `/home/newtonjeri/alix_ws/src/alix/docs/studio/STUDIO_IMPROVEMENT_PLAN.md` |
| `TRAJ` | `/home/newtonjeri/alix_studio_lite_ws/Alix_Studio_Lite_Trajectory_V2.html` (cross-reference only) |
| `LAY` | `/home/newtonjeri/alix_ws/src/alix/tools/layout_audit.py` |

## 0. Headline (read this if nothing else)

- The theme is a clean, tested **CSS custom-property token system**: light default on `:root` (CSS:1-129), dark variant on `:root[data-theme="dark"]` (CSS:134-189), type scale in a second `:root` (CSS:1714-1724). Values are contract-tested against `TOK` (TDT:80-105). Brand blue `#2F96EE` (CSS:22), pinned by TST:40-54.
- Dark mode is **opt-in only**: nothing in JS sets `data-theme`, no `prefers-color-scheme`, no toggle, nothing persisted (grep: no `data-theme` in JS; comment CSS:131-133 "Set `data-theme="dark"` on <html>").
- Layout = app bar (HTML:15-72) / ribbon tabs (HTML:115-125) / context strip (HTML:132-152) / command band (HTML:154-696) / overlay toggles (HTML:698-740) / 4-track grid `Browser dock | viewport | Generate dock | Properties dock` (HTML:742-919, CSS:297-310) / footer status-action bar (HTML:923-949).
- Docks are **drag- and keyboard-resizable, collapsible, auto-hidden when empty, persisted in localStorage** (JS:7475-7641). Dialogs are **draggable + resizable + persisted + clamped** (JS:8389-8523). **Plots are NOT user-resizable or maximisable** — only the 3-D viewport has a Maximize (JS:7538-7559). R6 must be built (mechanisms exist to reuse).
- Undo/redo is a **global snapshot (memento) stack, cap 80, JSON deep-clone of the whole editor state**, Ctrl/Cmd+Z, Ctrl/Cmd+Y or Ctrl/Cmd+Shift+Z (JS:1835-1977, 7112-7114). No labels, no per-entity scoping, no history list. R2 needs a per-region/sub-region, labelled, depth-5 variant.
- Feedback toolkit is rich and reusable: 2-line status + error toast that survives (JS:5431-5462), busy spinner (JS:5464-5467), Promise-based **styled confirm modal** with gate chips (JS:13786-13832), multi-choice modal (JS:14849-14887), 400 ms tooltips with keyboard parity (JS:7833-7937), searchable `?` help built from tooltips (JS:7948-8040), gate chips that are controls (JS:15904-15950), Fixture **Board** table with ✓/·/— pills and progress badge on the tab (JS:14708-14758, 15208-15290). No onboarding/wizard component exists (deliberate: RES:798-801).
- Plotly: only **two** small charts, each with an **inline, hard-coded dark-theme layout** (font `#e6edf3` = 1.18:1 on the default white panel → effectively invisible ticks) (JS:5368, 14483-14492). No shared template, no joint palette, no theme helper. Lazy-loaded (JS:5320-5337).
- Vendored, offline: three.js r128, OrbitControls/TransformControls (examples/js UMD), **Plotly 2.35.2 — the same version TRAJ loads from CDN (TRAJ:7)**, js-yaml 4.x (VEND, VIS:532-568, SRV:6322-6338).
- Separability: tokens are liftable verbatim; base components are mostly generic but **interleaved** with excluded-feature rules; the Fusion-style ribbon/command-group machinery (CSS:1577-1709, 1895-1972; JS:8065-8590) is the most Studio-specific and is a poor fit for non-technical users. Recommended extraction: `alix_tokens.css` + `alix_base.css` + `alix_shell.css` + `alix_shell.js` (line ranges in §8).
- Defects to fix while lifting: accidentally nested `.info-row` rules (CSS:1016-1029), 6 undefined tokens (CSS:388-398, 2096-2166), dark-only Plotly colours, old cyan literals, missing `.danger`/`.hint.error` rules, unloaded webfonts, white-on-brand 3.13:1 contrast.

---

## 1. Theme spec

### 1.1 Where the tokens live

| Block | Lines | Notes |
|---|---|---|
| Light (default) `:root` | CSS:1-129 | brand, neutrals, state, shadows, radius, surfaces, atlas, structural, design-system tokens (spacing, washes, targets) |
| Dark `:root[data-theme="dark"]` | CSS:134-189 | redefines colours/surfaces/shadows/atlas/washes only; non-colour tokens inherited |
| Type scale `:root` (2nd block) | CSS:1714-1724 | `--fs-xs/sm/md/lg`, `--cmdbar-h` (raised 2026-08-15) |
| Short-screen override | CSS:1563-1567 | `@media (max-height: 800px) { :root { --ribbon-h: 168px } }` (dead token, see 1.11) |
| Design-system source of truth | TOK:8-55 | light + dark; app must match numerically (TDT:80-105); card-only rules TOK:56-62 must never reach the app (TDT:129-141) |

### 1.2 Colour tokens (light vs dark)

| Token | Light | Dark | Line L / D | Role |
|---|---|---|---|---|
| `--brand` | `#2F96EE` | `#2F96EE` | 22 / 135 | "measured four independent ways" from butterfli.ai (CSS:2-6; TST:6-8) — selection & primary action only (TOK:9) |
| `--brand-strong` | `#1B7FD4` | `#57A9F0` | 23 / 136 | hover of primary, lower brand facets |
| `--brand-tint` | `#97CBF7` | `#1B4E7A` | 24 / 138 | outlines (e.g. stage-aware ribbon CSS:1692) |
| `--brand-wash` | `rgba(47,150,238,.10)` | `.16` | 25 / 139 | selected fills |
| `--brand-seam` | `#12558F` | `#0B3E6B` | 28 / 137 | facet seams of the logo (HTML:40; TST:335-358) |
| `--bg` | `#FFFFFF` | `#121821` | 30 / 141 | page |
| `--panel` | `#FFFFFF` | `rgba(22,27,34,.86)` | 31 / 142 | panels (dark translucent on purpose, DSR:96-102) |
| `--panel-raised` | `#FAFBFC` | `#161b22` | 32 / 143 | modal head/foot, active segment |
| `--viewport-bg` | `#1B2733` | `#0D1117` | 33 / 144 | **3-D canvas only** (CSS:7-20; TST:430-444); dark in both themes |
| `--border` | `#EEEEEE` | `rgba(139,148,158,.24)` | 34 / 145 | hairlines |
| `--border-strong` | `#DCDFE3` | `#30363d` | 35 / 146 | control/overlay borders |
| `--text` | `#14181D` | `#e6edf3` | 37 / 148 | body text |
| `--muted` | `#6B7480` | `#8b949e` | 38 / 149 | labels, hints (4.74:1 on white — computed) |
| `--accent` / `--accent-soft` | `var(--brand)` / `var(--brand-wash)` | same | 39-40 / 150-151 | aliases used by most components (64 + 39 uses) |
| `--ok` | `#2E9E5B` | `#3fb950` | 42 / 153 | pass |
| `--warning` | `#C77700` | `#ffcc66` | 43 / 154 | degraded |
| `--danger` | `#D93A34` | `#ff7b72` | 44 / 155 | broken / collision |
| `--ok-wash` / `--warn-wash` / `--danger-wash` | `rgba(46,158,91,.14)` / `rgba(199,119,0,.14)` / `rgba(217,58,52,.12)` | `.16` variants | 116-118 / 186-188 | tinted state fills; "every state also carries a glyph or word" (CSS:113-115) |
| `--surface-bar` | `#FFFFFF` | `rgba(13,17,23,.86)` | 65 / 160 | app bar, footer |
| `--surface-ribbon` / `-2` | `#FAFBFC` / `#F4F6F8` | `.55` / `.40` alphas | 66-67 / 161-162 | tab row / band |
| `--surface-dock` | `#FFFFFF` | `rgba(22,27,34,.70)` | 68 / 163 | docks |
| `--surface-inset` | `#F4F6F8` | `rgba(1,4,9,.30)` | 69 / 164 | inputs, recessed areas |
| `--surface-overlay` | `rgba(255,255,255,.97)` | `rgba(10,14,20,.97)` | 70 / 165 | popovers, palette, dialogs |
| `--surface-scrim` | `rgba(16,24,40,.28)` | `rgba(1,4,9,.55)` | 71 / 166 | modal backdrop |
| `--atlas-*` (15 tokens) | CSS:79-93 | CSS:169-183 | — | Redundancy-atlas cells — **Workspace tab only (X2), drop in Lite** |

Typography tokens: `--font: Inter, ui-sans-serif, system-ui, -apple-system, "Segoe UI", sans-serif` (CSS:109); `--mono: "JetBrains Mono", ui-monospace, SFMono-Regular, Menlo, monospace` (CSS:110). Type scale `--fs-xs 14px` (minor: badges, table headers, hints), `--fs-sm 16px` (main UI text), `--fs-md 17px` (control text, table body), `--fs-lg 19px` (headings) (CSS:1718-1721), `--fs-xl 22px` (CSS:111, unused). Hierarchy by weight: labels/dock heads 600, buttons/selects 500, hints 400 (CSS:1675-1680). `h1` 20px/700 (CSS:939), app-bar `h1` 18px (CSS:199). Mono is mandated for numbers/paths/timestamps (CSS:1008-1015; RED §M). Floor: `--fs-xs` is enforced by LAY (LAY:11) and TDT:117-120.

Spacing: `--s1 2px … --s7 24px` (CSS:120-121). Radii: `--radius 4px` (CSS:57), `--radius-sm 2px` (CSS:112), `--radius-pill 999px` (CSS:58) — Stitch's 4px/2px rule (RED:769-777). Shadows: `--shadow-sm/md/lg` are **complete box-shadow values**, never composed or used as colours (CSS:48-55; TST:240-293). Target sizes: `--target-min 24px` (WCAG 2.2 SC 2.5.8) and `--target-primary 44px` (high-frequency) (CSS:127-128); applied to selects/inputs (CSS:975-980), ranges (CSS:924), view-switch (CSS:1367), gate chips (CSS:1419), primary generate actions (CSS:1828).

Structural tokens: `--browser-w 300px`, `--props-w 330px`, `--gen-w 0px` (CSS:96-98); `--cmdbar-max 210px` (CSS:126, read by LAY), `--cmdbar-h 210px` (CSS:1722), `--cmdbar-shared` (set by JS:15117-15118), `--dlg-w/--dlg-h` (set by JS:8418-8423).

### 1.3 How light/dark is switched
- Mechanism: attribute selector `:root[data-theme="dark"]` on `<html>` (CSS:134). Comment: "kept as an opt-in variant … Set `data-theme="dark"` on <html>" (CSS:131-133).
- No JS writes it (grep of JS for `data-theme`/`dataset.theme`/`matchMedia`/`prefers-color-scheme`: zero hits; the 7 "theme" mentions are comments, e.g. JS:184, 10661). No UI control, no persistence, no `prefers-color-scheme` media query in CSS.
- The 3-D palette reads tokens once: `syncColorsFromCss()` (JS:185-201) is called only from `initThree` (JS:311) → a runtime theme flip would leave scene colours stale (inferred). Atlas canvas and its legend read tokens at paint time (JS:10663-10690, 7200-7204).
- Research decision: light default, dark first-class but optional (RES:903-905, RES:1011-1019; TOK:3-5; DSR:32).

### 1.4 Brand colour, app bar and logo
- App bar markup HTML:15-72: `.brand` (inline SVG mark + `<h1>ALIX Studio</h1>` HTML:51 + mode `.badge#modeBadge` "Viewer" → "Generator (server)" JS:9165-9167), `.view-switch` Author|Simulate (HTML:61-64), `.cov-banner#coverageBanner` (HTML:70), `.status-row#appStatusRow` with `#spinner` + `#status` (HTML:71).
- Logo: the butterfli mark as **inline SVG**, 4 facets (2 `fill="var(--brand)"`, 2 `fill="var(--brand-strong)"`), stroked with `var(--brand-seam)` 0.55 (HTML:39-49); 22×22 px (CSS:1354). Rationale comment HTML:17-37 (measured trace; seams instead of gaps so it survives 22 px). Pinned by TST:321-366 (4 facets, seam token in both themes, no external refs) and TST:179-198 (branding, inline SVG, no remote assets/CDN).
- App bar CSS: `.app-bar` flex, `--surface-bar`, `backdrop-filter: blur(16px)` (CSS:197-202); status clamps to 2 lines with full text in `title` (CSS:859-870).

### 1.5 Icon approach
- **Inline SVG strings in JS**, no sprite, no icon font: `RIBBON_ICONS` map (JS:7659-7704) wrapped by `_line` = `<svg viewBox="0 0 16 16" fill="none" stroke="currentColor" stroke-width="1.6" …>` (JS:7660); 2px-stroke frame glyphs from a Claude Design project (JS:7650-7658, 7690-7703). Assigned by button id via `RIBBON_ICON_BY_ID` (JS:7705-7727) and `data-ico` attributes (JS:7795-7799); injected by `applyRibbonIcons()` (JS:7791-7806) / `iconSpan()` (JS:7736-7741). Colour follows `currentColor` → themes automatically. Sizes: 13px inline (CSS:467-469), 21px in icon buttons (CSS:1622-1623), 18px under 2200px wide (CSS:1967-1969), 32px in reference-frame rows (CSS:2063-2066).
- Unicode glyphs carry state and small affordances: ⓘ info (JS:7821), ✓ △ ✗ · ⓘ gate chips (JS:15908), ✓ ⚠ ⛔ ↻ modal chips (JS:16823-16825), ▶ ■ ⟸ ⟹ transport (JS:8813-8823), ❮ ❯ dock chevrons (HTML:745, 869), ⤢ Maximize (HTML:947). Select caret = data-URI SVG with hard-coded `#8b949e` (CSS:966).

### 1.6 Status colours and the state vocabulary
- Chrome states: `--ok/--warning/--danger` + washes (above). ISA-101 rule: ~90% neutral, saturation only for state, red never decorates, every state pairs colour with glyph/word (TOK:6-7; DSR:28-29; RES:773-776, 980-985).
- Reused state patterns: `.gate-chip[data-state=pass|warn|fail|unrun|info]` (CSS:1421-1433), `.axm-chip.g-ok|g-warn|g-bad|g-stale` (CSS:723-730), `.sch-pill.p-ok|p-warn|p-bad|p-stale|p-locked` (CSS:1325-1333), `.pill-ok/.pill-missing/.pill-warn/.pill-na` (CSS:1984-1998), `.val-row[data-state=ok|warn|error]` (CSS:1465-1484), `.cov-banner[data-state]` (CSS:1486-1492), `.sim-dot[data-state=off|ready|live]` (CSS:1389-1395), `.finish-headline.ok|bad|stale` (CSS:610-616).
- 3-D state palette is **JS hex literals**, not tokens: `STATE_COLOR` (JS:165-180: reachable `0x40d959`, collision `0xf85149`, covered `0x2ea043`, armLeft `0x4073ff`, …), `DAC_ROBOT_COLOR 0x9fb3c8`, `DAC_HIT_COLOR 0xff4d4d` (JS:12446). Only `colors.bg/line/point/x/y/z/start/selected` are synced from tokens (JS:193-200). Legend derives from the same constants (JS:7181-7275; TIG:509-543).
- Contrast (computed with the WCAG formula): text `#14181D`/white 17.8:1; muted 4.74:1; **white on `--brand` 3.13:1** (primary buttons CSS:847-852 — below AA for normal text); white on `--brand-strong` 4.17:1; white on `--brand-seam` 7.72:1; `--warning` 3.46:1 and `--ok` 3.41:1 as text on white; `--danger` 4.56:1.

### 1.7 Drift and defects in the theme (fix when lifting)
1. **Undefined tokens** (used, never defined): `--radius-lg` (CSS:389, 394, 398, with fallback 10px), `--line` (CSS:2096, 2113, 2115; **no fallback** → border dropped), `--text-muted` (CSS:2100, 2116), `--text-dim` (CSS:2134, 2147), `--surface-2` (CSS:2143, 2162), `--warn` (CSS:2149). (`--cmdbar-shared`, `--dlg-w`, `--dlg-h` are JS-set — fine.)
2. **Accidental CSS nesting**: `.info-row {` opened at CSS:1016 is not closed until CSS:1029, so `.info-row.is-error code`, `.info-row.is-weak …`, `.info-row code.two-line`, `.info-row.is-error` (CSS:1020-1027) parse as nested rules → under native CSS nesting they become `.info-row .info-row.is-error …` and never match (inferred from nesting semantics; verified by brace scan — the only nesting in the file).
3. Hard-coded colours outside tokens: `#40d959/#f0a020` (CSS:451-452, 657), `#8a63d2` left-arm crumb (CSS:822), `#ff4d4d` slider (CSS:1108), `#ff8c69` (CSS:1265), `rgba(255,204,102,…)` dark-amber fills in light theme (CSS:424, 927, 997), `rgba(255,255,255,.08)` (CSS:1264), dark-default fallbacks `#e6e9ef/#8b93a7` (CSS:2131-2166).
4. Hard-coded fonts: 21 literal `"JetBrains Mono"` declarations vs 14 `var(--mono)`; `html, body` re-states the stack instead of `var(--font)` (CSS:191); inputs use `font: 12px …` (CSS:964-966, 989, 902).
5. **Webfonts are not loaded**: no `@font-face`, no font files in VEND → Inter/JetBrains Mono render only if installed locally, else `system-ui`/`ui-monospace` (CSS:109-110, 191).
6. `.hand-state-warn::before { content: "\\26A0\\FE0E  " }` (CSS:2154) — double backslash renders literal text (inferred).
7. Classes used by JS with no CSS: `button.danger` (JS:19293), `.hint.error` (JS:14725, 14728).
8. Old cyan (`#00d4ff`) survives in JS as `rgba(0,212,255,0.25)` (JS:14486) and `SCH_TYPE_COLOR.paired '#00d4ff'` (JS:13593) — TST:86-91 only greps `#00d4ff` in CSS and `0x00d4ff` in JS.
9. Dead structural tokens/code: `--sidebar-width` (CSS:95; only legacy `initSidebarResize` JS:8723-8744, whose handle no longer exists), `--ribbon-h` (CSS:99, 1564; zero `var()` uses), `.resize-handle` (CSS:1040-1043); `--fs-xl`, `--s7` unused.

### 1.8 Guard tests worth porting to Lite
- TDT:80-105 (tokens equal `TOK` numerically, per theme), TDT:108-126 (structural tokens), TDT:129-141 (no card scaffolding in app CSS).
- TST:40-91 (brand value, tokens defined in default `:root`, dark redefines surfaces, viewport not white, no old cyan), TST:94-125 (3-D palette reads tokens, fallback matches token, axis colours distinct), TST:128-176 (view switch), TST:216-293 (no dark literals in rules; shadow tokens never composed or used as colours), TST:295-313 (surfaces tokenised; app bar/ribbon use tokens), TST:404-428 (scene elements ≥4:1 on the canvas in both themes), TST:430-444 (viewport token only on the canvas), TST:446-456 (JS fallback = token).

---

## 2. Layout system

### 2.1 DOM skeleton (top → bottom)
```
.app (CSS:193: 100vh flex column)
├─ header.app-bar ............ HTML:15-72  brand | view-switch | cov-banner | status
├─ section.sim-panel ......... HTML:79-111 (shown only when body[data-view=simulate], CSS:1379-1381)
├─ nav.ribbon-tabs ........... HTML:115-125 7 tabs: Regions · Board · Workbench ‖ Dual-Arm Collisions · Schedule & Run ‖ Workspace · Gizmo
├─ .task-strip ............... HTML:132-152 context strip: back, session chip, crumb, destination, GATES
├─ .ribbon-body .............. HTML:154-696 one .tab-page[data-page] per tab (regions 156, workbench 204, board 537, workspace 556, collisions 599, schedule 642, gizmo 675)
├─ .visibility-bar ........... HTML:698-740 overlay toggles (now a popover anchored to the viewport nav, CSS:268-283)
├─ .workspace-row ............ HTML:742-919 grid: [Browser dock | main viewport | Generate dock | Properties dock]
│   ├─ aside.browser-dock .... HTML:743-823  (.dock-resize, .dock-head, .dock-body > section.panel[data-tabs])
│   ├─ main .................. HTML:825-850  #threeViewport, #viewCube, Home View, mode overlay, vp-islands, dock-expand buttons
│   ├─ aside.gen-dock ........ HTML:861-865  (X1 — Generate dock)
│   └─ aside.props-dock ...... HTML:867-917  Selected point, Animation, Coverage, Validation, Execution, Collision log, Legend
└─ footer.status-bar ......... HTML:923-949  Save, Save Copy, Mirror, Undo/Redo, dirty dot, stats, Focus, Points form, ? Help, Maximize view
```
Grid definition: `.workspace-row { --col-browser: var(--browser-w); --col-props: var(--props-w); grid-template-columns: var(--col-browser) minmax(0,1fr) var(--gen-w,0px) var(--col-props) }` with every child pinned to row 1 (CSS:297-310).

### 2.2 Visibility idioms (house style)
- `body[data-tab]` (set in `switchTab` JS:7398), `body[data-view]` author|simulate (JS:19204-19216; CSS:1379-1381), `body[data-task-mode]` cleaning|mechanism (CSS:1340-1347), `body[data-stage]` generate|edit (CSS:1682-1693; JS:15124). Per-panel relevance: `section.panel[data-tabs="…"]` filtered by `updateDockPanels()` (JS:7297-7314) with a title fallback registry `DOCK_PANEL_TABS` (JS:7166-7175). RED keeps this as a strength (RED:79); SEAM:275-317 notes the second, imperative idiom (`style.display`) and proposes all-declarative.

### 2.3 Command band (Fusion-style ribbon)
- Dense tabs (>3 panels or a `data-cmd="dialog|dock"` panel) become command **groups** with icon buttons (2-row grid) + a caption button that opens a **flyout** (menu) or a persistent **dialog/dock**; sparse tabs stay inline (`cmd-inline`) (JS:8098-8153, 8169-8189 `adoptRibbonV2`, 8191+ `buildCommandGroup`; CSS:1577-1709, 1895-1972). Band auto-sizes, never scrolls, capped by `--cmdbar-h` and equalised across canvas tabs by measurement (`equaliseCanvasBands` JS:15073-15119; CSS:785-801). Enforced by LAY (LAY:9-14) and TIG:1322-1365.
- Owner requirements behind it: RES:62-74 (Fusion reference, never-scroll band, captions, persistent dialogs, help canvases, one camera, ≥11px text).

### 2.4 Docks — resize, collapse, empty, maximize (R6/R10 building blocks)
| Capability | How | Evidence |
|---|---|---|
| Drag resize | `.dock-resize[data-edge=browser|props]` 8px separator, `pointerdown` → window `pointermove` → `setDockWidth()` writes `--browser-w/--props-w` on `<html>`; clamped `DOCK_W_MIN 180`, `DOCK_W_MAX 620`, and the viewport keeps ≥320px (`dockWidthMax`) | HTML:744, 868; CSS:331-341; JS:7562-7645 |
| Keyboard resize | separator is focusable `role="separator"`; ←/→ 16px, Shift 64px, Home/End limits; `aria-valuenow/min/max` maintained | HTML:744; JS:7617-7630 |
| Persist widths | `localStorage['cleaningPathEditor.dock.browserW'|'propsW']`, written once on pointer-up | JS:7580-7591, 7593-7597, 7636-7645 |
| Collapse / expand | `.app.browser-collapsed` / `.props-collapsed` set the track to 0 and show a `.dock-expand` button in the viewport; focus handed to the expand button | CSS:455-464; JS:7475-7480, 7486-7509 |
| Persist collapse | `localStorage['cleaningPathEditor.dock.browser'|'props']` | JS:7478, 7488 |
| Empty dock stands down | `.app.browser-empty/.props-empty` derived from "no visible panel on this tab"; not persisted | CSS:346-351; JS:7316-7339 |
| Generate dock resize | `--gen-w` 260-760px, pointer + keyboard, not persisted | JS:7510-7537 (X1 dock) |
| Maximize viewport | footer `#maximizeViewBtn` collapses both docks and restores the previous arrangement (`preMaximize`); `aria-pressed` | HTML:947; JS:7484, 7537-7556 |
| Renderer refit | every change calls `onResize()` → camera aspect, `renderer.setSize`, `Plotly.Plots.resize('analysisPlot')`, clamps dialogs | JS:632-645 |
| Responsive | ≤900px wide: single column, page scroll, viewport 60vh (CSS:1069); ≤1460px tighter ribbon (CSS:633-639); ≤800/820/900px tall compact rules (CSS:1563-1567, 2065-2085) |

### 2.5 Dialogs (persistent work panels)
- `.cmd-dialog` fixed, accent border, `--surface-overlay` (CSS:1758-1772); header is the drag handle (`makeDialogDraggable` JS:8389-8405); `[data-frac]` dialogs open at a fraction of the viewport and are **resizable** via a focusable corner grip (`makeDialogResizable` JS:8470-8503; CSS:1783-1798), min 420×320 (JS:8415-8423), box persisted per dialog `cleaningPathEditor.genDialog` / `cleaningPathEditor.dlg.<groupId>` with try/catch (JS:8438-8468), clamped back on-screen on window resize (`clampOpenDialogs` JS:8505-8523). Dismissed only by ×, Esc or its group button — never by an outside click (JS:8119-8133; TIG:1457).

### 2.6 Can plots be resized or maximised? (R6)
- **No.** `#analysisPlot { height: 190px }` inside a collapsed-by-default `<details>` in the Properties dock (CSS:928; HTML:906-909); its width follows the dock width (dock resizable) and `onResize` calls `Plotly.Plots.resize` (JS:644). `#dacPlot` fills the Timeline panel width with a fixed 64px height in the inline band (CSS:1108, 2019, 2023) and is resized on tab entry (JS:7408-7411). No plot splitter, no plot maximize, no height handle. The only "maximize" is for the 3-D viewport (2.4).
- TRAJ cross-check: plots are fixed `.plot { height: 600px }` / `.small 520px` / `480px` under 850px wide (TRAJ:125-129, 312), `responsive: true, scrollZoom: true` (TRAJ:862-867) — zoom/pan only, no window resizing either.
- What to reuse for R6/R10: the dock separator (CSS:331-341 + JS:7562-7645) as a **vertical splitter between the dual-arm viewport (left) and the plot pane (right)**, plus a horizontal variant for stacked plots; the dialog grip (JS:8470-8503) for pop-out plot windows; the `preMaximize` toggle (JS:7538-7559) generalised to "maximize this plot"; after every size change call `Plotly.Plots.resize(div)` (pattern JS:644, 7410) or attach a `ResizeObserver` (inferred design).

### 2.7 localStorage inventory (all under `cleaningPathEditor.*`)
`dock.browser`, `dock.props` (collapsed) JS:7478/7488; `dock.browserW`, `dock.propsW` JS:7588/7595; `genDialog`, `dlg.<id>` JS:8438-8468; `<detailsId>` open state JS:8711-8720; legacy `sidebarWidth` JS:8726/8737. Dock and `<details>` writes are **not** wrapped in try/catch (JS:7478, 7588, 8714-8717) — only dialog persistence is (JS:8447-8453). Research says browser storage must hold only UI prefs/crash buffers, real work on disk (RES:1003-1009).

---

## 3. Undo/redo

| Aspect | Studio implementation | Evidence |
|---|---|---|
| Model | **Snapshot (memento)**, not command objects | `captureEditorState()` JS:1839-1870 |
| Scope | **Global** — one pair of stacks for the whole editor (`state.undoStack/redoStack`) | JS:60-61 |
| What is recorded | deep copies of `config`, `points`, `sequence`, `sequenceDraft`, `selectedIndex`, `dirty`, generator `knots/startPose/selection/dirty`, picked start/pre-start fields, and per-component paint face labels + regions | JS:1839-1870, 1873-1894 |
| Copy method | `cloneData = JSON.parse(JSON.stringify(v))` (paint labels converted to arrays) | JS:1835-1837, 1860 |
| Push | `pushHistory()` before each mutation; skipped when nothing is editable; **cap 80** via `shift()`; clears redo | JS:1896-1902 |
| Call sites | 30+ `pushHistory()` calls (e.g. JS:2598, 2703, 3296, 4341, 4751, 8834, 10272, 16512, 18613); gizmo drags push **once per drag** on `dragging-changed` | JS:351-354, 371-383, 4401 |
| Restore | rebuilds derived data, repaints regions, `updateAll()`; **does not move the camera** | JS:1905-1948 |
| Undo / Redo | push current onto the opposite stack, restore, `markDirty(true,'Undo applied.')` | JS:1950-1964 |
| Buttons | footer `#undoButton/#redoButton` (HTML:931-932) with icons (JS:7713); enabled from stack lengths (`updateHistoryButtons` JS:1966-1977) | |
| Shortcuts | Ctrl/Cmd+Z undo; Ctrl/Cmd+Y **or** Ctrl/Cmd+Shift+Z redo; ignored while typing in INPUT/TEXTAREA/SELECT (`isTypingTarget` JS:276-279) | JS:7108-7114 |
| Cleared | on every file load, edit-session close, generate-session clear | JS:5697-5698, 14910-14911, 14939-14940 |
| Not recorded | camera, tab, visibility (by construction) — matches Blender scope rule | RES:858-861 |
| Missing | labels ("Undo: smooth J3"), history list, per-entity scope, persistence across reload | RES:783-786, 996-1001 |
| Tests | gizmo drags undoable TIG:151; buttons reflect paint sessions TIG:211 | |

R2 fit: the pattern is directly reusable but must be adapted — scope key `regionId[/subRegionId]`, **depth 5** (requirement: "undo option for up to five previous changes", requirements file line 10), labelled entries (show "Undo: Trim 12 s" in the tooltip, as RES:783-786 recommends), one entry per slider *gesture* (copy the once-per-drag rule JS:371-379), and snapshot the *edit parameters* (trim/smooth/scale settings) rather than the 7×N joint arrays, recomputing the curve on restore (inferred design; arrays derive deterministically from raw + params). TRAJ has **no** undo/redo/history today (grep: none).

---

## 4. Feedback and guidance components

| Component | Where | Behaviour | Value for a non-technical operator |
|---|---|---|---|
| Status line | HTML:71; CSS:859-870; `setStatus()` JS:5431-5445 | 2-line clamp, full text on `title`, `.error` red; ~149 callers | Medium: the single place messages appear; Lite must write plain-language sentences (RES:883-886) |
| Busy spinner | HTML:71; CSS:1036-1038; `setLoading()` JS:5464-5467 | spinner + message; paired on/off (TIG:256-280) | High |
| Error toast | CSS:1531-1541; `showErrorToast()` JS:5447-5462 | danger-filled, bottom of viewport, 12 s, click to dismiss; survives the next status write (TIG:594) | High |
| Global error capture | JS:19150-19166 | uncaught errors and promise rejections → status + toast + spinner off | High (no silent failures) |
| API error convention | `api()` JS:9088-9104 | server `{error, hint}` → Error with `.hint` (what to do) and `.payload` | High: the "hint" is the fix sentence |
| **Styled confirm modal** | `showStudioModal(opts)` JS:13786-13832; CSS:683-745 | scrim + card: title, "Writes to" destination, gate chips (glyph+word), severity lines, note, Cancel/primary; Promise<boolean>; Esc/backdrop = cancel; primary focused | Very high: template for "Run on the REAL robot?", "Upload to controller 192.168.1.10?" (deploy example JS:16823-16850) |
| Multi-choice modal | `askSessionChoice(opts)` JS:14849-14887 | 3+ options in the same shell | High ("Discard edits / Save / Keep editing") |
| Residual native confirms | JS:2717, 9324, 10377, 10522 | `window.confirm` | Replace in Lite |
| Persistent dialog | JS:8389-8523; CSS:1753-1798 | draggable, resizable, persisted, clamped, explicit dismiss | Medium |
| Tooltips | `initTooltips()` JS:7833-7937; CSS:679-681, 1882-1893 | `title`→`data-tip`, 400 ms hover delay, immediate on keyboard focus, Esc hides, `aria-describedby`, titled "help canvas" inside commands, placed beside menus | High (TIG:934-1006) |
| ⓘ info icons | `.info-ico[data-tip]` (e.g. HTML:158); CSS:675-676; `initInfoIcons()` JS:7811-7825 | long static hints collapse into an ⓘ | High, but Lite should keep the key hint visible, not hidden |
| Help overlay `?` | `initHelpOverlay()` JS:7948-8040; CSS:747-757; `#helpButton` HTML:946 | searchable list built from every long `data-tip` + `HELP_BINDINGS` (keys/gestures) | High: one place to "read the manual" |
| Command palette `S` | `initCommandPalette()` JS:8600-8709; CSS:683-700 | searches every button/toggle, switches tab and clicks it | Low for non-technical users (accelerator only, RES:1038-1047) |
| Keyboard shortcuts | JS:7108-7148; `kbd` CSS:892; shown in menus `RIBBON_SHORTCUT_BY_ID` JS:7730-7734 | 1-5 modes, F focus, ←/→, Del, Ctrl+S/Z/Y | Low (HELP_BINDINGS JS:7953-7968 still lists key 4 = Rotate, which was removed JS:7134-7137) |
| Tab badge (progress) | HTML:118-119; CSS:832-838; JS:14754-14755 | `present/expected` on the Board tab | **High** — R1 dashboard ("7/12 regions taught") |
| Coverage banner | HTML:70; CSS:1486-1492; `renderCoverageBanner()` JS:15866-15902 | headline verdict in the app bar on every tab, glyph + sentence | High (pattern for "Region 3 of 12 · 04:10 left") |
| Gate chips (as controls) | HTML:148-151; CSS:1409-1433; `renderGates()` JS:15904-15925, `initGates()` JS:15927-15950; model `gateState()` JS:15760+ | pass/warn/fail/unrun/info + `blocking`; click takes you to the fix; stale on edit (`markDirty`→`invalidateValidation` JS:5371-5430) | **Very high** — per-region readiness gates (Optimised → Simulated → Real-tested → Uploaded) |
| Session/mode chip | HTML:138 (`aria-live="polite"`); CSS:806-811; JS:15027 | loud mode indicator (colour + word) | High for SIM vs REAL mode (RES:935-940) |
| Meter with target | `.cov-head/.cov-bar` CSS:1435-1460 | number + pill + bar with a target marker | High (cleaning-time countdown, upload progress) |
| Validation rows | CSS:1462-1484 | glyph + sentence + "where" in mono | High (spike/limit findings, R12) |
| Status pills | CSS:1325-1333, 1984-1998 | glyph + word + wash, never colour alone | High |
| Fixture Board | HTML:537-554; CSS:1507-1528, 1726-1751, 1976-1998; `loadBoard()` JS:14708-14758; `renderBoardRows()` JS:15208-15290 | region × arm table, cells are controls (✓ n pts / · generate / —), summary chips, relative age, sticky header | **Very high** — direct template for R1 "regions with/without a taught .txt" |
| Empty states | e.g. HTML:808, 813, 873-889; JS:14713-14714 | "say what to do next" (RED:82) | High; Studio wording is technical ("Waiting for cleaning path YAML", HTML:71) |
| Dirty indicator + leave guard | HTML:933; CSS:877-878; `markDirty()` JS:5371-5379; `beforeunload` JS:9018-9022 | amber dot; browser prompt only while dirty | High |
| Honest progress | `#genJobRow` HTML:392-396; CSS:1848-1851; `genRenderJob()` JS:17589-17630 | "stage i of N — state", elapsed "m:ss of 64–83 s measured", no fake %, cancel note | High pattern for upload/optimise jobs |
| Sim connection dot + caveat | HTML:80-110; CSS:1383-1405; `simSetDot()` JS:19218-19225 | off/ready/live dot + words; caveat banner | High (robot connection status, R4) |
| Viewport chrome | HTML:826-849; CSS:1044-1068 | mode overlay hint, ViewCube, Home View, Orbit/Pan/Zoom/Fit island, Overlays popover | Medium (sim view) |
| WebGL failure panel | CSS:884-887; `webglHelpHtml()` JS:286-305 | browser-specific remedy text | Medium |
| Per-tab legend | HTML:912-915; CSS:1167-1175; `TAB_LEGENDS` JS:7205-7275 | explains every colour in words | High |
| Focus ring | CSS:1543-1561 | `:focus-visible` brand outline | Medium (TIG:610-633) |
| Wiring self-report | `bind/need/runSteps` JS:8780-8811; `initUi` JS:9054-9077 | missing ids reported once, steps isolated | Developer value |
| Onboarding / tour / wizard | none — deliberately (RES:798-801, 987-994) | — | **Gap**: R1 needs a guided "clean region N for T seconds → next" flow |
| Notification log | rejected (RED:280) | — | — |

---

## 5. Plotly conventions

- **Charts in Studio**: only two Plotly charts render today — `analysisPlot` (segment-length bar chart, JS:5354-5369) and `dacPlot` (dual-arm collision timeline, JS:14478-14497). The Schedule timeline moved from Plotly to a DOM track (JS:13587-13592; TIG:2104), the atlas is a 2-D canvas (JS:10820 area).
- **No shared template/helper**: each call inlines its layout: `paper_bgcolor/plot_bgcolor 'rgba(0,0,0,0)'`, `font {color '#e6edf3', size 10}`, grid `'rgba(139,148,158,0.15)'`, tight margins; config `{displayModeBar: false, responsive: true}` (JS:5368, 14487-14492). `#e6edf3` is the **dark-theme** text colour → 1.18:1 on the default white panel (computed) — tick labels effectively invisible in the light theme.
- **Series colours**: analysis bars use `_hex(colors.point)` (brand, JS:5368); collision bars `'#ff4d4d'` vs `'rgba(0,212,255,0.25)'` (old cyan, JS:14486). No per-joint palette exists in Studio.
- **Interaction**: `plotly_click` → `dacGotoTime(x)` scrubs the robot to that instant (JS:14492-14496, 14501-14513); Prev/Next jump between colliding samples (`dacJumpHit` JS:14548+); `dacGoto` poses the robot, recolours colliding links (`dacHighlight` JS:14531-14546) and prints "COLLIDING: a↔b" in `var(--danger)` (JS:14515-14530). This is the closest existing pattern to R5.
- **Lazy load**: `ensurePlotly()` injects `vendor/plotly.min.js` on first draw, retries on failure, reports "Charts unavailable…" (JS:5320-5337); server whitelists only `plotly.min.js` (SRV:6322-6338); exported standalone HTML inlines it (VIS:537-544). Hidden-panel skip + redraw on open (JS:5339-5365; TIG:871).
- **Resize**: `Plotly.Plots.resize('analysisPlot')` in `onResize` (JS:644) and `'dacPlot'` on tab entry (JS:7410); `#dacPlot`/`.sch-track` forced to 100% width because "Plotly measures its container once" (CSS:2017-2019).
- **Dark-mode helper**: none for Plotly. The token-reading pattern exists for the canvas atlas (`atlasPalette()` JS:10677-10690) and 3-D (`syncColorsFromCss` JS:185-201) — reuse that to build `plotTheme()` for Lite (inferred design).
- **TRAJ (to be integrated)**: Plotly 2.35.2 from CDN (TRAJ:7), white backgrounds hard-coded (TRAJ:857-858), `JOINT_COLORS` = Plotly/D3 category10 (`#1f77b4 #ff7f0e #2ca02c #d62728 #9467bd #8c564b #e377c2`, TRAJ:577-585) — J4 is red and J3 green, which collides with the ISA-101 rule that red/green mean state (RES:773-776); `hovermode 'x unified'`, horizontal legend, select-to-region (TRAJ:849-893); own palette `--accent #175cd3`, dark header `#0b1220` (TRAJ:9-21, 29-31) — must be re-skinned to ALIX tokens.

---

## 6. Vendored libraries and offline capability

| Library | File | Version (evidence) | Notes |
|---|---|---|---|
| three.js | VEND/three.min.js (603,445 B) | **r128** (`const e="128"` in header) | UMD global `THREE`; code uses `renderer.outputEncoding`/`THREE.sRGBEncoding` (JS:328-330), which changed in later releases (inferred upgrade risk) |
| OrbitControls | VEND/OrbitControls.js (26,375 B) | examples/js style, attaches `THREE.OrbitControls` (OrbitControls.js:1043) | matches r128 era |
| TransformControls | VEND/TransformControls.js (45,539 B) | attaches `THREE.TransformControls` (TransformControls.js:1467) | pose gizmo; drag → `pushHistory` once (JS:349-383) |
| Plotly | VEND/plotly.min.js (4,558,696 B) | **v2.35.2** (file header) | Same version TRAJ uses from CDN (TRAJ:7) → no API drift when Lite vendors it |
| js-yaml | VEND/js-yaml.min.js (43,461 B) | **4.x** (contains "is removed in js-yaml 4" shims) | global `jsyaml`; used by `jsyaml.load` (JS:5687) |

Offline: template placeholder `__VENDOR_JS__` (HTML:7-8) is replaced by inlined `<script>` blocks in fixed order (VIS:532-556); CSS/JS are inlined into the page (`render_html` VIS:558-568); the served page omits Plotly and lazy-loads it from `/vendor/` (SRV:6307-6318, 6418-6420; TIG:1147-1190); page cached by asset mtimes (SRV:6297-6318). Tests forbid remote scripts/styles/images (TST:185-198). Gap: webfonts not vendored (1.7 item 5).

---

## 7. UX research principles that matter for a NON-TECHNICAL operator app

Context: Studio was designed for "robot operators and robotics engineers, not CAD professionals" (RES:31-33; DSR:15-17). Lite targets non-technical users, so some Studio decisions must be **reversed** (marked ⟲).

1. **Operator-first simplification; hide frames and quaternions.** "keep coordinate frames OUT of the default workflow … treat 'no code, no frames, no quaternions visible' as the acceptance bar for the operator-facing 80% of flows" (RES:438-440). PolyScope X splits an Operator Screen from the programmer view with progressive disclosure (RES:378-380).
2. ⟲ **Friendly labels.** Studio rejected renaming `base_link`, `start_pose`, `lid_open`, `quarter_length` because its users debug in RViz (RED:268-270). Lite users are not those users → use plain names ("Pole: half height", "Hand: half grip") and keep technical names behind an expander.
3. **Plain-language, constructive errors** close to the source, no jargon, no blame, preserve input, raw detail behind "details" (RES:883-886; RES:971-978 "Arm can't reach these 4 points — move the base closer").
4. **ISA-101 colour discipline**: ~90% neutral, colour only for state, every colour paired with a glyph or word (~8% colour-deficient) (RES:773-776, 980-985; DSR:28-29).
5. **Confirm only what is irreversible**; prefer undo elsewhere; friction ∝ damage × irreversibility; the one real confirmation shows destination + gates (RES:793-796, 971-978). Lite: confirm real-robot runs and controller uploads, not edits.
6. **Simulate-before-run as a gate, real-robot mode visibly separate** (RoboDK RUNMODE_SIMULATE default, colour vocabulary for connection) (RES:353-356, 971-978); MoveIt-style red collision links, plan vs execute separation, trajectory slider (RES:403-406).
7. **Empty states teach and carry the action**; distinguish "nothing yet" from "filtered" (RES:738-741; RED:82).
8. **No forced tutorial; pull help + contextual first-use hints**; an optional retrievable guide (RES:798-801, 987-994). ⟲ Lite's R1 is itself a staged, linear task ("clean region, then the next"), which is exactly where *staged* disclosure is legitimate (RES:478-481) — but keep escape hatches/back navigation (RES:483-485 guideline 3).
9. **Progressive disclosure, max 2 levels**; simple defaults/presets on the surface, expert parameters inside (RES:268-275, 478-481).
10. **Large targets**: ≥24px minimum, 44px for frequent actions, bigger on coarse pointers (RES:1011-1019; RES:808-851) — not yet implemented as `@media (any-pointer: coarse)` in CSS (grep: none).
11. **Light theme default, dark optional** (RES:903-905).
12. **Undo through model mutations only; one gesture = one entry; label entries; history list** (RES:783-786, 858-866, 996-1001).
13. **Async feedback thresholds**: nothing <1 s, spinner 2-10 s, determinate progress >10 s; per-item pills instead of blanking a whole board (RES:728-731).
14. **Board as a dense matrix with pills, sticky header, cells as controls, summary chips** (RES:963-969; RES:503-511; RES:753-756).
15. **One still camera; deliberate reframes only** (RES:68; DSR:31) — relevant to the dual-arm sim view.
16. **Foxglove-style playback keys** (Space, arrows step, loop) and named timeline events (RES:413-416) — relevant to R3/R5 scrubbing.
17. **Curated task layouts beat free docking** (RES:418-421) — supports fixed Lite screens with a few resizable splitters (R6) rather than free-form docking.
18. **Verify layout in a real browser** — static tests missed clipping three times; LAY is the gate (RES:83-108 table; DSR:104-109; LAY:1-40).
19. **Stitch visual language** adopted app-wide: 1px borders, 4px radius, tonal elevation, sentence-case labels, label-left inspectors, mono numbers (RED:751-799, 800-829; STI:21-33 palette & font).

IMP (Studio improvement plan) is server/geometry focused (part registry, transforms, analysis); its only UI item relevant here is the pose gizmo spec (IMP:343-363) — X1-adjacent.

---

## 8. Reuse assessment and concrete extraction

### 8.1 How separable is editor.css?
- **Tokens: cleanly separable** (contiguous blocks, contract-tested).
- **Base elements/components: generic but interleaved.** The main "atoms" block CSS:892-1050 (kbd, details, toggles, panel, buttons, fields, grids, info-list, spinner, viewport overlay) is mixed with excluded-feature rules (paint regions CSS:906-915, IK-frame chips 916-919, twist-row 901-903, sequence-item 925-927, `#analysisPlot` 928, mesh-mode 997, `#threeViewport.mode-*` 888-891/905). Lift with filtering, preserving source order (duplicates such as `.brand` CSS:929 vs 1353 and `.dock` 311 vs 332 rely on cascade order).
- **Shell/overlay layers: mostly contiguous and generic** (CSS:191-213, 263-353, 455-469, 674-757, 832-878, 1052-1068, 1349-1407, 1529-1561, 1753-1798, 1882-1893).
- **Studio-specific machinery**: command groups/flyouts/ribbon v2 (CSS:1577-1709, 1895-1972), canvas-band equalisation (CSS:772-801; JS:15060-15119), task strip specifics (CSS:759-838).
- **Coupled to excluded features (X1 geometric path / X2 C-space & Workspace)** — do not lift: Generate dialog/dock (CSS:356-420, 660-672, 1800-1880; HTML:210-419, 861-865), Regions mesh painting (CSS:503-603; HTML:156-201), Selected Point / Start-pose / Analyze&Finish / Reference frame (CSS:479-502, 605-631, 2034-2085; HTML:421-535), atlas (CSS:72-93, 167-183, 2087-2120; HTML:569-591), selected-point analysis (CSS:438-454; HTML:592-595), task-mode gating (CSS:1340-1347), stage-aware ribbon (CSS:1682-1709), Gizmo tab (HTML:675-694).

### 8.2 How separable is editor.js?
One 19.3k-line script with a global `state` (JS:23-118); shell subsystems are function-level separable but call shared helpers (`setStatus`, `onResize`, `switchTab`, `renderScene`). Liftable as a small `alix_shell.js` (≈1,000 lines, inferred size): `isTypingTarget` 276-279; `setStatus/showErrorToast/setLoading` 5431-5467; `escapeHtml` 8592-8594; `api` 9088-9104; `bind/need/runSteps` 8780-8811; tooltips/info/aria 7811-7937, 8043-8059; help overlay 7948-8040; modals 13786-13832, 14849-14890; docks 7475-7641 (replace `onResize` with a resize bus); dialogs 8389-8523; icons 7659-7741; transport `renderPlaybackBlock` 8749-8766; view switch `setView` 19204-19216; lazy Plotly 5320-5337; history 1835-1977 (generalised); global error capture 19150-19166. `showStudioModal` depends only on `escapeHtml` and the `.cmd-overlay/.cmd-box/.axm-*` CSS — fully standalone.

### 8.3 Recommended extraction (Lite files and the line ranges to lift)

| New Lite file | Lift from | Edits while lifting |
|---|---|---|
| `alix_tokens.css` | CSS:1-71, 94-129 (minus 95 `--sidebar-width`, 98 `--gen-w`, 99 `--ribbon-h`), CSS:134-166, 184-189, CSS:1714-1724 | drop atlas tokens (CSS:72-93, 167-183); add `--radius-lg`, aliases for `--line/--text-muted/--text-dim/--surface-2/--warn` (or rewrite users); add Lite tokens: `--joint-1…7`, `--plot-text/--plot-grid/--plot-bg`, `--mode-sim`/`--mode-real`; keep TOK-named tokens byte-equal so a copy of TDT keeps passing |
| `alix_base.css` | CSS:190-194; 892-900; 924; 929-1037 (filtered: skip 901-921 paint/IK/twist, 925-928, 997); 1039; 1543-1561 (focus) | close `.info-row` at CSS:1016 (move 1028-1029 up); `font-family: var(--font)` on body; replace literal `"JetBrains Mono"` with `var(--mono)`; add `button.danger`, `.hint.error` |
| `alix_shell.css` | app bar 196-202; tabs 204-213 + 931-938; status bar 263-265 + 839-878; workspace grid + docks + resize + empty/collapsed 289-353, 455-464; icons 466-469; info-ico/tooltip/palette/modal/help 674-757; tab badge 832-838; viewport chrome 1044-1068; playback/timeline 642-645, 1116-1127, 1167-1175; view switch + sim dot/caveat 1349-1407; gates/meters/validation/banner 1409-1505; board + pills 1507-1528, 1976-1998; label-role/toast 1529-1541; dialogs 1753-1798; help canvas 1882-1893 | rename coverage-specific classes (`cov-*`) to generic (`meter-*`, `banner-*`) or alias; replace `rgba(255,204,102,…)` with `--warn-wash` |
| `alix_shell.js` | the JS ranges in 8.2 | generalise `onResize` to a resize bus (`ResizeObserver` per plot); wrap all localStorage in try/catch; namespace keys `alixLite.*`; add `syncColorsFromCss()` + `plotTheme()` re-run on theme change |
| `alix_plot.js` (new) | patterns from JS:5320-5337, 14478-14513, 10677-10690 | `plotTheme()` from tokens (font `--text`, ticks `--muted`, grid from `--border-strong`, transparent bg); joint colourway that avoids state red/green or is documented as identity; R5 "stop at collision": vertical marker shape + red marker at the colliding sample (TRAJ already uses `Plotly.relayout(... {shapes})` for cursors, TRAJ:1141, 1302-1303) |
| Brand header | HTML:38-52 | title "ALIX Studio Lite"; badge shows mode ("Simulation" / "Real robot 192.168.1.10") |
| Vendor | VEND/plotly.min.js, three.min.js, OrbitControls.js, TransformControls.js (only if the sim view needs gizmos), js-yaml.min.js | add Inter + JetBrains Mono woff2 + `@font-face` (inferred need) |
| Tests | TDT, TST:40-198 & 216-456 subset, TIG dock tests (TIG:2388-2525), TIG focus/tooltip/toast/busy tests (TIG:256-280, 594-633, 934-1006) | retarget paths |

### 8.4 What NOT to lift for Lite (and why)
- The Fusion ribbon (command groups, flyouts, palette as a primary path) — dense by design for expert throughput (RES:62-74); non-technical users need visible, labelled, large primary actions and a guided flow (RES:378-380, 438-440, 268-275).
- Seven-tab IA (Regions/Board/Workbench/Collisions/Schedule/Workspace/Gizmo) — Lite has its own flow: Regions (PPT + dashboard) → Teach → Optimise → Simulate → Real test → Upload (inferred from requirements lines 8-34).
- Task strip deploy-destination UI and `edit destination…` (HTML:132-152) — Lite uploads to a controller program slot, not a repo path (inferred).
- Overlay toggles popover with 25+ technical switches (HTML:698-740) — expose 3-5 (robot, bin, commode, trail) at most (inferred).

---

## Requirement relevance

- **NFR-THEME** — Reuse the token system verbatim (CSS:1-129, 134-189, 1714-1724; TOK:8-55), the inline-SVG brand mark (HTML:38-52), the Stitch visual language (RED:751-829), the state vocabulary (glyph + word + wash, CSS:1409-1505), and port TDT/TST guards. Fix the drift list in §1.7. Re-skin TRAJ (its own palette TRAJ:9-21) onto ALIX tokens.
- **NFR-UX** — Reuse the feedback toolkit (§4): plain-language status + toast that survives, `showStudioModal` for real-robot/upload confirmation, gate chips as clickable readiness steps, Board-style dashboard, tab progress badge, honest progress, legend. Reverse Studio's "keep technical names" decision (RED:268-270) per RES:438-440; raise targets (44px primary) and add `@media (any-pointer: coarse)` (RES:1011-1019). Add the missing guided-flow component (none exists).
- **R1** — Board table (JS:14708-14758, 15208-15290; CSS:1507-1528, 1976-1998) + tab badge (JS:14754-14755) = dashboard of regions with/without a raw .txt; banner + meter (CSS:1435-1460, 1486-1492) for "clean region N for T s"; no countdown/timer component exists (gap); auto-advance is new logic.
- **R2** — Snapshot undo pattern (JS:1835-1977) reusable; adapt to per-region/sub-region scope, depth 5, labelled entries, one entry per slider gesture (JS:371-379 rule). TRAJ has no undo.
- **R3** — Author|Simulate segmented switch + `body[data-view]` idiom (HTML:61-64; CSS:1361-1381; JS:19204-19216); transport block (JS:8749-8766; CSS:642-645, 1116-1127); sim dot/caveat (CSS:1383-1405).
- **R4** — Same switch reused as "Simulation | Real robot"; sim-dot states for connection (JS:19218-19225); `showStudioModal` before real motion (JS:13786-13832); RoboDK mode-split principle (RES:353-356). IP entry has no Studio precedent in the shell (inferred: belongs to INT-CTRL).
- **R5** — DAC timeline: click-to-scrub (`plotly_click`→`dacGotoTime` JS:14492-14513), Prev/Next collision jump (JS:14548+), colliding-link recolour (JS:14531-14546), danger-coloured frame readout (JS:14515-14530). Needs a theme-correct Plotly layout (JS:14487-14491 is dark-only).
- **R6** — Plots not resizable today (CSS:928, 2023; TRAJ:125-129); reuse dock separator (CSS:331-341; JS:7562-7645), dialog grip (JS:8470-8503), maximize-with-restore (JS:7538-7559), `Plotly.Plots.resize` hooks (JS:644, 7410).
- **R7** — UI precedent only: per-region properties beside the region list (Paintable parts HTML:796-799, Standoff HTML:800-803 inside the Regions dock panel HTML:783-805; rationale comment HTML:789-795) and the Tool Frame & Pole panel with derived hand state (HTML:747-757); technical value names need friendly labels.
- **R10** — 4-track grid with token-driven widths (CSS:297-310) adapts to "viewport left | plots right" with a draggable splitter; one-camera stillness rule (RES:68).
- **R11** — Live validation pattern: `markDirty` → `invalidateValidation` marks verdicts stale on every edit (JS:5371-5430) — reuse so a smoothness change marks "Simulated ✓" stale.
- **R12** — Validation rows with severity glyph + sentence + "where" (CSS:1462-1484) suit spike findings; plain-language advice (RES:883-886).
- **R13** — Honest progress pattern (JS:17589-17630) and per-item result list + retry-failed (RES:673-676, 723-726; STI:231-233) for uploads.
- **X1 / X2** — Excluded UI is isolated enough to omit (§8.1 list); atlas tokens are X2-only.
- **INT-TRAJ** — Same Plotly version (2.35.2) as vendored; restyle TRAJ to tokens; add plot theming + resizing; replace category10 joint colours or document them as identity.
- **INT-CTRL** — The Tkinter control app must be re-expressed in this web shell's components (modal confirm, connection dot, progress, status/toast) to keep one theme (inferred).

## Reuse verdicts

| Feature | Verdict | What to lift | Evidence |
|---|---|---|---|
| Colour/type/spacing/radius/shadow/target tokens (light + dark) | reuse-as-is | CSS:1-71, 94-129, 134-166, 184-189, 1714-1724 (drop atlas) | TDT:80-126; TOK:8-55 |
| Theme switching | reuse-with-adaptation | keep `[data-theme]`; add toggle + persistence + re-sync of 3-D/Plotly colours | CSS:131-134; JS:185-201, 311 |
| Brand mark, app bar | reuse-with-adaptation | HTML:38-52; CSS:196-202, 1353-1354; rename to "ALIX Studio Lite" | TST:321-366 |
| Inline SVG icon set | reuse-as-is | JS:7659-7741; CSS:466-469 | TIG:1498 |
| Base controls (buttons, primary, fields, toggles, panels, info rows, kbd, spinner) | reuse-with-adaptation | CSS:839-878, 892-1039 (filtered) | fix CSS:1016-1029 nesting |
| Workspace grid + docks (resize/collapse/empty/persist) | reuse-with-adaptation | CSS:289-353, 455-464; JS:7475-7641 | TIG:2388-2525 |
| Plot resize / maximize | not-available (build from parts) | dock separator + dialog grip + `preMaximize` pattern + `Plotly.Plots.resize` | CSS:928, 2023; JS:644, 7537-7556, 8470-8503 |
| Persistent draggable/resizable dialogs | reuse-as-is | CSS:1753-1798; JS:8389-8523 | TIG:1457 |
| Undo/redo | reuse-with-adaptation | JS:1835-1977, 7112-7114; make scoped, depth 5, labelled | TIG:151, 211; RES:996-1001 |
| Status line + toast + busy + global error capture | reuse-as-is | JS:5431-5467, 19150-19166; CSS:859-878, 1531-1541, 1036-1038 | TIG:256-280, 594 |
| Confirmation modal (gates + destination) | reuse-as-is | JS:13786-13832, 14849-14890; CSS:683-745 | TIG:2074 |
| Tooltips, ⓘ, help overlay | reuse-with-adaptation | JS:7811-8063; CSS:674-681, 746-757, 1882-1893 (keep key hints visible) | TIG:934-1006 |
| Command palette (S) | reference-only | JS:8600-8709 | RES:1038-1047 |
| Fusion ribbon / command groups / flyouts | not-applicable (too dense for Lite) | — | CSS:1577-1709; JS:8098-8590; RES:438-440 |
| View switch (segmented) + `body[data-view]` | reuse-as-is | HTML:61-64; CSS:1356-1381; JS:19204-19216 | TST:128-176 |
| Sim connection dot / caveat | reuse-with-adaptation | CSS:1383-1405; JS:19218-19225 | — |
| Gate chips + gateState + stale-on-edit | reuse-with-adaptation | CSS:1409-1433; JS:15760+, 15904-15950, 5371-5430 | TIG:1838-1927 |
| Meter / banner / validation rows / pills | reuse-with-adaptation | CSS:1435-1505, 1325-1333, 1984-1998 (rename `cov-*`) | TIG:1893-1950 |
| Fixture Board table + tab badge | reuse-with-adaptation | CSS:1507-1528, 1726-1751, 1976-1998; JS:14708-14758, 15208-15290 | TIG:387-437, 1621-1679 |
| Legend | reuse-with-adaptation | CSS:1167-1175; JS:7181-7290 | TIG:509-543 |
| Transport / playback block | reuse-as-is | JS:8749-8766; CSS:642-645, 1116-1127 | — |
| Collision timeline scrub + link highlight | reuse-with-adaptation | JS:14478-14560 (retheme the layout) | TIG:2120-2156 |
| Plotly theme helper | not-available | build `plotTheme()` from tokens (pattern JS:10677-10690) | JS:5368, 14487-14491 |
| Plotly lazy loader + vendored Plotly 2.35.2 | reuse-as-is | JS:5320-5337; VEND/plotly.min.js; SRV:6322-6338 | TIG:1147-1190; TRAJ:7 |
| three r128 + controls (offline) | reuse-as-is | VEND/*; VIS:532-556 | TST:185-198 |
| Webfonts | not-available | vendor Inter + JetBrains Mono (inferred) | CSS:109-110, 191 |
| Onboarding / guided flow / countdown | not-available | design new (staged disclosure for R1) | RES:478-485, 798-801 |
| Layout audit gate | reference-only | adapt LAY to Lite screens | LAY:1-40; DSR:104-131 |
| Regions painting, Generate dock, pose editing, atlas, gizmo tab | not-applicable (X1/X2) | — | §8.1 |

## Gaps, risks, open questions (for the planner)
- Risk: copying CSS blocks verbatim also copies the `.info-row` nesting bug and six undefined tokens (§1.7).
- Risk: Plotly charts copied from Studio are unreadable in the default light theme (1.18:1).
- Risk: primary buttons (white on `#2F96EE`) are 3.13:1; for a non-technical audience consider `--brand-strong` fill (4.17:1) or larger bold labels — a token-contract change if `TOK` is to stay in sync (TDT:80-105).
- Risk: fonts silently fall back to system fonts on field laptops without Inter/JetBrains Mono (CSS:191).
- Open: should Lite keep `TOK` as its design source (and port TDT) or fork its own tokens file?
- Open: dark mode in Lite — ship the toggle (and persist it) or keep light-only like Studio's effective default?
- Open: one web app for both trajectory editing and the control app (Tkinter today) — the shell supports it, but INT-CTRL's runtime/server split decides.
