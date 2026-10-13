# 04 — ALIX Studio server architecture → what ALIX Studio Lite should do with it

Key: `04_studio_server_architecture`. Read-only survey; nothing in either repo was modified.
All claims cite `path:line`. "(inferred)" marks conclusions drawn from code that were not executed/observed.

## Path legend (used in every citation below)

| Alias | Absolute path |
|---|---|
| `ALIX/` | `/home/newtonjeri/alix_ws/src/alix/` |
| `TG/` | `ALIX/core/python/alix_taskgen/src/alix_taskgen/` (Studio package; `server.py`, `web_editor/`) |
| `TGP/` | `ALIX/core/python/alix_taskgen/` (project dir: `pyproject.toml`, `README.md`, `test/`, `test_generate/`) |
| `WS/` | `ALIX/core/python/alix_workspace/src/alix_workspace/` |
| `DP/` | `ALIX/core/python/alix_dispatch/src/alix_dispatch/` |
| `EMU/` | `ALIX/core/python/src/alix_emulator/` |
| `LITE/` | `/home/newtonjeri/alix_studio_lite_ws/` |

Sizes measured with `wc -l`: `TG/server.py` 6,566 lines; `TG/web_editor/editor.js` 19,325; `editor.css` 2,166; `template.html` 953; `TG/studio_generate.py` 5,846; `EMU/rm_emulator.py` 2,616; `DP/track.py` 4,099; `DP/emulated_program.py` 547.

## TL;DR

* The Studio is a **stdlib `ThreadingHTTPServer` + one hand-dispatched `BaseHTTPRequestHandler`** with two dict route tables (24 GET + 40 POST = 64 `/api/*` routes), JSON in/out, gzip, a single global `_GENERATION_LOCK`, a per-route "needs" table enforced by `_dispatch`, and a job/poll model for multi-minute generation (`TG/server.py:5961-6253`, `TG/studio_generate.py:537-1060`).
* The page is **one HTML file assembled at request time** by string-replacing four placeholders (`__VENDOR_JS__`, `__EDITOR_CSS__`, `__VIEWER_MANIFEST__`, `__EDITOR_JS__`) in `template.html` (`TG/web_editor/template.html:8,9,12,951`; `TG/scripts/visualize_generated_config.py:558-568`). Plotly is deferred and served from a whitelisted `/vendor/` route (`TG/server.py:6325-6338`).
* There is **no `--emulator` or fixture flag in `server.main`** (only `--host/--port/--no-browser`, `TG/server.py:6512-6514`). `--emulator` lives in the launcher `ALIX/tools/run_studio.py:167`, which installs the in-process RealMan SDK emulator before importing the server (`run_studio.py:142-158,188-205`).
* Everything resolves **relative to the repo** (marker scan for `ARCHITECTURE.md`+`core/`, `.git`-rooted `package.xml` index for meshes, SRDF search for `*_moveit_config`, emulator `sys.path` insert of `external/butterfli-realman-alix/Python`). Installed outside the repo, most of it degrades **silently** to empty answers (details in Q2).
* The emulator **can** accept uploaded `.txt` programs and run program IDs — but only through `alix_dispatch.connect.open_arm()`, which wraps the emulated handle with `emulated_program.ProgramSurface` (`DP/connect.py:174-180`). The Studio's own `--emulator` path does **not** wrap (`run_studio.py:154-157`) and the Studio has **no execution channel at all** (pinned by `TGP/test/test_studio_interaction_guards.py:2204-2211`).
* **Recommendation: (c)+(b) hybrid — a standalone `alix_studio_lite` package** that (b) imports `alix_workspace` as a pinned library (FK/IK/collision/plan frames, ROS-free, small dep set), (c) copies a thin server scaffold, the theme tokens, vendored JS libs and a few front-end modules from `alix_taskgen`, and (c) vendors `alix_dispatch.track/connect/emulated_program` + `alix_emulator` behind a real/sim controller seam. Do **not** make Lite a mode of the Studio (Q7).

---

## 1. Server technology

### 1.1 Stack and process model
* `from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer` (`TG/server.py:30`); server constructed in `main()` as `ThreadingHTTPServer((args.host, args.port), StudioHandler)` (`TG/server.py:6548`). One thread per request (stdlib `ThreadingHTTPServer` uses daemon threads — stdlib behaviour, not repo code).
* No `protocol_version` is set → every response is HTTP/1.0 with connection close, and there is **no request timeout**; this is why long work must be a job (`TG/server.py:5814-5819`, docstring of `do_auto_path`).
* Access log suppressed: `log_message` is a no-op (`TG/server.py:6340-6341`).
* Process-global state in module dicts: caches (`_SCENE_MANIFEST_CACHE`, `_FIXTURE_COLL_CACHE`, `_ROBOT_COLL_CACHE` `TG/server.py:886-888`; `_REACH_CACHE` `:695`; `_ROBOT_GEOM_CACHE` `:1864`), external sim feed `_SIM_EXTERNAL` (`:5309`), job registries in `studio_generate._RUNS` (`TG/studio_generate.py:537`) and `studio_bulk._JOBS` (`TG/studio_bulk.py:54`). Same-key cache builds are serialized with `_serialized`/`_key_lock` (`TG/cache_locks.py:22-41`); robot IK contexts are an LRU `_BoundedCache` of 8 (`TG/robot_model.py:51-87`, ~20-50 MB per context `:57-62`).
* A CPU-bound reachability sweep runs in a 1-worker `ProcessPoolExecutor` with `spawn` context so the server stays responsive (`TG/server.py:700-725`); it is shut down in `main()`'s `finally` (`:6562`).
* Startup also starts a `prewarm` daemon thread (`TG/server.py:6546`, body `:6472-6490`) building IK context, scene manifests + fixture BVHs for every fixture with regions, reachability analyzer and robot collision BVHs; failures are swallowed ("cold first click").
* `sys.addaudithook` is installed once to record routes that `open()` a task-config YAML without declaring it (`TG/server.py:6187-6225`, called at `:6547`). An audit hook cannot be removed — it observes every `open` in the process.

### 1.2 Dispatch tables
* `ROUTES_GET` (`TG/server.py:5961-5987`, 24 entries) and `ROUTES_POST` (`:5988-6029`, 40 entries); counted by AST parse. Handler signatures: GET handlers receive `parse_qs(parsed.query)` → `dict[str, list[str]]` (`:6426`; e.g. `q["fixture"][0]` at `:5970`); POST handlers receive the parsed JSON body (`:6448`).
* `do_GET` (`TG/server.py:6402-6432`): `/` and `/index.html` → assembled page; `/vendor/<name>` → `_send_vendor`; else table lookup; unknown → `404 {"error":"not found"}`.
* `do_POST` (`TG/server.py:6434-6456`): table lookup; body capped at `MAX_POST_BYTES = 64 MiB` → `413` (`:5178`, `:6444-6447`); `json.loads(self.rfile.read(length) or b"{}")` (no content-type check).
* Lite-relevant routes that exist today (all "free" = no config/lock needs, per the AST tally of `ROUTE_NEEDS`): `/api/robot-geometry`, `/api/joint-presets`, `/api/sim/status|joints|pose`, `/api/plans`, `/api/plan-frames` (reads configs only for mechanism context), `/api/dual-arm-collisions`, `/api/analyze-plan`, `/api/ik`, `/api/session`. Tally: 43 free, 7 read-only, 7 read+lock, 3 lock+job, 1 lock+write, 1 lock, 1 read+lock+job, 1 write (`TG/server.py:6064-6132`).

### 1.3 JSON conventions and error handling
* Success: handler returns a dict → `_send_json` → `json.dumps` → `_write_json` (`TG/server.py:6347-6375`). Responses > 8 KiB are gzip-compressed (level 5) when the client sends `Accept-Encoding: gzip` (`:6369-6373`).
* **Expected refusal = HTTP 200 with `{"error": ..., "hint": ...}`** — the "house convention" read by the client's `api()` (`TG/server.py:6162-6163`; client `TG/web_editor/editor.js:9088-9104`, which throws on `!response.ok || data.error` and keeps `hint`/`payload`/`busy`).
* Uncaught handler exception → `_error_payload` → `400 {"error": str(exc)}`; a `ReflectionRefused` becomes `422` with the verdict (`TG/server.py:6459-6469`). POST prints a traceback except for reflection refusals (`:6452-6456`).
* Client disconnects (`BrokenPipeError`, `ConnectionResetError`) are swallowed and the connection closed (`TG/server.py:6345`, `:6357-6361`, `:6396-6400`, `:6427-6431`).
* No auth, no CORS, no cookies. Binding a non-loopback host prints a warning that `/api/save` can write files (`TG/server.py:6539-6544`).

### 1.4 Busy/lock contract
* `_GENERATION_LOCK = _OwnedLock()` — a `threading.Lock` that also records its holder (route label or thread name, `since`) (`TG/server.py:343-389`).
* Every route declares needs in `ROUTE_NEEDS` from `{read_configs, write_configs, generation_lock, long_running}` (`TG/server.py:6040-6062` rationale, `:6064-6132` table); `ROUTE_BUSY_POLICY` marks routes that handle busy themselves: `degrade` (`/api/path-ik`, `/api/redundancy-atlas`) or `job` (`/api/auto-path`, `/api/auto-path-reauthor`, `/api/bulk-generate`) (`:6134-6140`).
* `_dispatch` (`TG/server.py:6228-6252`): if the route is busy-sensitive and the lock is held, poll up to `BUSY_WAIT_S = 8.0` s (`:6146`), then return `_busy_contract` — HTTP 200 `{"error","hint","generationBusy":true,"retryable":true,"route","owner","pollUrl":"/api/session","waitedS"}` (`:6159-6184`). A `TaskConfigReadError` raised by an undeclared route is converted to the same contract and the route recorded in `_UNDECLARED_CONFIG_ROUTES` (`:6243-6248`).
* Why the lock exists: while a generation runs, `path_generator.forbid_config_reads` **replaces `builtins.open`/`io.open`/`Path.open`/`read_text`/`read_bytes` for the whole interpreter** and refuses task-config reads (`TG/path_generator.py:278-303`; matcher `is_task_config_path` `:231-253`, which flags only YAML outputs / `*_points.yaml` / `*_task.yaml` / YAML under `hand_authored|generated|archive|auto-generated_config` — a Lite `.txt` track is not matched).
* Client side: `watchGenerationBusy` shows one status line and polls `/api/session` every 3 s until `generation.busy` is false (`TG/web_editor/editor.js:9115-9134`).

### 1.5 Long jobs and polling
* Job starters: `/api/auto-path` → `studio_generate.start_run` (`TG/server.py:5810-5834`), `/api/auto-path-reauthor` (`:5837-5858`), `/api/bulk-generate` → `studio_bulk.start_bulk` (`:5899-5908`). Status: `GET /api/auto-path-status?runId=` (`:5861-5868`), `GET /api/bulk-generate-status?jobId=` (`:5911-5914`). Cancel: POST routes (`:5889-5896`, `:5917-5920`).
* Job record lifecycle (`TG/studio_generate.py:861-1060`): validate → mint `runId = uuid4().hex[:12]` → state `queued` → daemon thread `auto-path-<runId>` → `running` stages → `done|failed|cancelled`. Only one live run (`_reserve_run`, `:597-…`; refusal `:916-920`). Registry guarded by `_RUNS_GUARD`, which **never nests the generation lock** so status always answers (`:537-541`). Constants: `POLL_MS = 2000` (`:94`), `ETA_SECONDS_RANGE = (40, 350)` (`:86`), `CANCEL_MAY_TAKE_S = 314` (`:91`), `MAX_KEPT_RUNS = 8` (`:99`). A RUNNING payload never carries `error` (it would kill the client poll loop, `:1021-1023`). `sync: true` exists for tests (`:926-928`).
* `/api/session` (`TG/server.py:6255-6294`) is the single global-state answer: generation busy/owner, live auto-path run, live bulk job, undeclared routes, `busyWaitS`. It opens no file and takes no lock. **There is no per-user session**; "session" means process state.

### 1.6 Static assets and template
* Page: `do_GET("/")` builds `index_manifest = {"refFrame": ref_frame_payload(), "regions": {}, "meshes": {}}` and serves `_cached_index_html(...)` with `Cache-Control: no-store, must-revalidate` (`TG/server.py:6404-6418`).
* `_cached_index_html` memoises on `_web_asset_stamp()` = (name, mtime_ns, size) of `template.html`, `editor.css`, `editor.js`, `vendor/*.js` (`TG/server.py:6297-6317`); calls `render_html(index_manifest, inline_plotly=False)`.
* `render_html` (`TG/scripts/visualize_generated_config.py:558-568`) reads assets via `importlib.resources.files("alix_taskgen") / "web_editor"` and replaces `__VENDOR_JS__` (inlined `three.min.js`, `OrbitControls.js`, `TransformControls.js`, `js-yaml.min.js`, plus `plotly.min.js` only when `inline_plotly=True`; `:534`, `:543`, `:546-555`), `__EDITOR_CSS__`, `__EDITOR_JS__`, `__VIEWER_MANIFEST__` (JSON). Placeholders sit at `TG/web_editor/template.html:8,9,12,951`; the manifest is read client-side from `<script id="viewerManifest" type="application/json">`.
* Measured sizes recorded in code: page 5,781,424 B → ~1.66 MB gzip (`TG/server.py:6379-6383`); Plotly 4.56 MB = 79 % of the page, so deferred (`TG/scripts/visualize_generated_config.py:536-541`); client loads it on demand via `ensurePlotly()` → `<script src="vendor/plotly.min.js">` (`TG/web_editor/editor.js:5322-5337`).
* `/vendor/` is **not** a file server: `VENDOR_WHITELIST = {"plotly.min.js"}` (`TG/server.py:6325`), cached `public, max-age=86400` (`:6338`).
* Vendored versions: plotly.js **v2.35.2** (header of `TG/web_editor/vendor/plotly.min.js`), three.js **r128** (`const e="128"` in `three.min.js`), js-yaml 4.x.

---

## 2. Startup and configuration

### 2.1 Entry points and CLI
* Console script `alix-studio = "alix_taskgen.server:main"` (`TGP/pyproject.toml:45`). README still says `cleaning_path_studio` (`TGP/README.md:13,26`) — doc drift.
* `server.main` flags: `--host` (default `127.0.0.1`), `--port` (default `8765`), `--no-browser` (`TG/server.py:6512-6514`). It enables `faulthandler` (guarded; `dump_traceback_later` is forbidden because it segfaults CPython 3.12 here, `:6516-6537`), starts prewarm, installs the audit hook, binds, opens a browser after 0.6 s preferring Microsoft Edge (`_open_studio_tab`, `:6493-6507`, `:6550-6552`).
* Launcher `ALIX/tools/run_studio.py` (cross-platform replacement for bash; `run_studio.sh:18-19` just `exec`s it): flags `-e/--emulator`, `-p/--port` (8765), `--no-browser` (`run_studio.py:167-171`); **no `--host` pass-through** and **no fixture flag**. It `chdir`s to the temp dir before importing `alix_taskgen` (F30: running from `core/python/` shadows the installed package, `run_studio.py:16-20,182`), preflights the emulator (`:92-139`), starts it in-process (`:142-158`) and then calls `server.main(["--port", …, "--no-browser"?])` in the same interpreter (`:200-205`) — required because `sim_bridge` reads the emulator's in-memory registry (`run_studio.py:22-25`).
* Fixture choice is UI-driven: fixtures are discovered under `CONFIG_ROOT/*_cleaning/<name>/scene.yaml` (`TG/server.py:97-101,158-182`); the default is simply the first one (`currentfixture_default`, `:5515-5520`).

### 2.2 How resources are located (env var vs relative path)

| Resource | Resolution | Override | Evidence |
|---|---|---|---|
| Repo root (server) | walk up for `ARCHITECTURE.md` + `core/`; fallback `parents[2]` | `ALIX_REPO_ROOT` | `TG/server.py:56-79` |
| Repo root (paths/robot_model) | same marker; fallback `parents[3]` | — | `TG/paths.py:48-54`; `TG/robot_model.py:37-42` |
| Task configs, fixtures, scenes, meshes | `REPO_ROOT/ros/orchestration/alix_tasks/config` | `ALIX_TASKS_CONFIG` | `TG/server.py:84-87` |
| Task package (`package://alix_tasks/…`) | `repo_root()/ros/orchestration/alix_tasks` | `ALIX_TASKS_PKG` | `TG/paths.py:57-85,104-107` |
| Plan JSON roots | `Resource/plans`, `core/python/src/alix_core/plans`, `<tasks>/plans` | `ALIX_PLANS_DIR` (replaces all) | `TG/paths.py:110-138`; gate `TG/server.py:4187-4197` |
| Generated output | `<project>/auto-generated_config` (`parents[2]` of `paths.py`) | `ALIX_TASKGEN_OUTPUT_ROOT` | `TG/paths.py:141-161`; `TG/server.py:53,89` |
| IK frames / gizmos | package-relative `TG/config/ik_frames.yaml`, `TG/config/gizmos` | — | `TG/server.py:93-94` |
| Web assets | `importlib.resources` (render) and `Path(__file__)` (vendor route, stamp) | — | `TG/scripts/visualize_generated_config.py:547,560`; `TG/server.py:6301,6331` |
| Workspace config (chains, URDF) | `$BUTTERFLI_WORKSPACE_CONFIG` else package data `WS/config/alix_workspace_config.yaml`; URDF `../urdf/alix.urdf` relative to it | `BUTTERFLI_WORKSPACE_CONFIG` | `WS/config_loader.py:23-56`; `WS/config/alix_workspace_config.yaml:41-50` |
| Robot meshes (`package://rm_description/…` etc.) | `PackageIndex.discover(urdf)`: nearest `.git` ancestor (else a `src` dir, else `parents[2]`), scan every `package.xml`, plus sibling dirs of the repo under `src/` | **none** | `WS/package_index.py:27-37,40-74,80-121`; `WS/collision.py:54-61` |
| SRDF (pole/hand/bin states, disabled pairs) | upward glob for `*_moveit_config/config/*.srdf`, bounded; then from `REPO_ROOT` | `ALIX_SRDF` | `WS/srdf_states.py:120-154`; `TG/robot_model.py:217` |
| Home pose | every indexed package's `config/initial_positions.yaml` | — | `WS/srdf_states.py:61-84` |
| Reach cache | per-user `$ALIX_WORKSPACE_CACHE` → `$XDG_CACHE_HOME/alix_workspace` → `~/.cache/alix_workspace` | `ALIX_WORKSPACE_CACHE` | `TG/server.py:750-757`; `WS/reachability.py:81-84` |
| Dual-arm sweep / plan-frame disk cache | `REPO_ROOT/.cache/dual_arm_collisions` | — | `TG/server.py:4154,4303-4316,4659-4672` |
| Emulator's real SDK (Algo + ctypes structs) | `sys.path.insert(parents[4]/external/butterfli-realman-alix/Python)` | — | `EMU/rm_emulator.py:86-110` |
| SDK-parity reach oracle | loads `ALIX/tools/verify_task_motion.py` **by file path** | — | `TG/sdk_reach.py:47-64` |
| Arm IPs | `alix_core.sdk.endpoints` / `RM_LEFT_IP`, `RM_RIGHT_IP` (defaults .10 / .103) | env | `EMU/rm_emulator.py:476-477`; `DP/connect.py:25-33`; `TG/sim_bridge.py:56-85` |

Data facts behind the table: URDF mesh URIs are all `package://` for `rm_description`, `inspire_{left,right}_hand_description`, `pole_description`, `ugv_description`, `realsense2_description` (counted from `WS/urdf/alix.urdf`). Those packages live at `ALIX/ros/description/*` except `realsense2_description`, which is a sibling vendor tree `alix_ws/src/vendor/realsense-ros/…` (found via `package.xml` search). The SRDF is `ALIX/ros/manipulation/alix_moveit_config/config/alix.srdf`.

### 2.3 What breaks if installed outside the repo (no ROS, no checkout)
* Fixtures/regions/configs: `CONFIG_ROOT` does not exist → `list_fixtures()` returns `[]` with no error — the exact failure shape the code warns about (`TG/server.py:60-69`, `:97-101`).
* Robot meshes: `PackageIndex` finds no `package.xml` → `resolve_uri` logs `package '…' not found in source index` and returns `None` (`WS/package_index.py:113-121`); `_load_mesh` caches `None` (`WS/collision.py:117-136`). Consequence (inferred): empty robot overlay and **collision models without meshes → collisions silently under-reported**.
* SRDF not found → `do_joint_presets` returns `{"poleStates": []}` (`TG/server.py:1776-1778`); dual-arm sweep has no disabled pairs (`WS/dual_arm_plan_collision.py:628-629`) and cannot seed bin/pole/hand states (`:427-453`) (inferred: more false-positive self pairs and a wrong home pose).
* Plans: `_PLAN_ROOTS` missing → `/api/plans` empty and `_resolve_plan` refuses every path unless `ALIX_PLANS_DIR` is set (`TG/server.py:4175-4197`).
* Disk caches attempt `REPO_ROOT/.cache/...` writes; failures are swallowed (`TG/server.py:4360-4365`) (inferred: cache never persists if unwritable).
* Emulator: SDK path missing → `_ALGO = None` and `install()` then raises `NameError` because it references `_RealAlgo`, `_arm_model_e`, `_force_e`, `_ik_params_t` unconditionally (`EMU/rm_emulator.py:87-110`, `:2604-2607`). `run_studio.py` preflights the submodule path (`:123-138`).
* `_arm_base_frames` (controller Base/World frames per pole state) returns `{}` because `sdk_reach._oracle()` cannot load `tools/verify_task_motion.py` (`TG/server.py:1822-1831`; `TG/sdk_reach.py:54-64`).
* `alix_dispatch` has **no `pyproject.toml`** and is reached by `sys.path` (`ALIX/LAYERS.md:25`; `ALIX/tools/alix-dispatch:16-23` inserts four repo-relative paths).

---

## 3. Dependencies and weight

### 3.1 Python dependencies
* `alix_taskgen` runtime: `trimesh`, `python-fcl`, `pycollada` (+lxml), `networkx`, `numpy`, `scipy`, `shapely`, `pyyaml`, `ruamel.yaml` (`TGP/pyproject.toml:13-28`; same list `TGP/package.yaml:16-25`). Optional `analysis = ["matplotlib"]` (`:35-36`). `alix_workspace` is an **optional sibling, not on PyPI**, imported lazily (`:29-33`).
* `alix_workspace`: `numpy`, `scipy`, `pyyaml`, `trimesh`, `python-fcl` (**required**: the dual-arm sweep calls `trimesh.collision.CollisionManager` with no fallback), `pycollada`, `networkx`; optional `viz = ["fast-simplification"]` (`ALIX/core/python/alix_workspace/pyproject.toml:12-22,34-35`). Package data includes `urdf/*.urdf`, `config/*.yaml`, `workspace_data/*.npz` (`:44-45`) — i.e. the URDF and the baked visual overlay (`WS/workspace_data/visual_overlay_baked.npz`, 2.9 MB) ship inside the wheel, the meshes do **not**.
* `alix-core` (+ `alix_emulator` in the same distribution): **zero** dependencies by policy (`ALIX/core/python/pyproject.toml:10-13,26-34`); the SDK is the pinned submodule `external/butterfli-realman-alix`, not package data (`:36-45`).
* SDK: RealMan RM_API2 Python v1.1.6 with `libs/linux_x86`, `linux_arm`, `win_64`, `win_32` (`ALIX/DEPENDENCIES.md:35-60`); **licence unknown — no LICENSE file anywhere in RM_API2** (`ALIX/DEPENDENCIES.md:63-70`). `tools/rm_upload.py` also accepts an installed `pip install Robotic_Arm` (`ALIX/tools/rm_upload.py:9-11,54-66`).
* Test-only: `quickjs` (Python module) for executing `editor.js` (`TGP/test/test_editor_js_executes.py:41-42`); pytest-xdist `-n 12` (`TGP/pyproject.toml:70`).
* Install policy in the repo: editable, user-site, `--break-system-packages`, Python 3.10–3.13; `python-fcl` publishes **wheels only, no sdist**, so a missing wheel is unrecoverable (`ALIX/SETUP.md:426-475`).

### 3.2 Import/startup weight
* `import alix_taskgen.server` pulls the generator stack at module import: `generate_task` (→ `trimesh`, `scipy`, `path_planners`, coverage, YAML writer…) and the visualizer (→ `trimesh`, `numpy`) (`TG/server.py:34-44`; `TG/scripts/generate_cleaning_config.py:8-28`; `TG/scripts/visualize_generated_config.py:9-20`).
* Documented readiness time of a live server: **"startup is 30–60 s"** (`ALIX/docs/studio/STUDIO_GENERATE_TAB_PLAN.md:2586-2591`).
* Cost centres on first use: IK context = 2 `UrdfModel` + collision & visual `RobotCollisionModel` (each `from_file` re-runs `PackageIndex.discover` unless an index is passed — `WS/collision.py:54-61`) ~20–50 MB each (`TG/robot_model.py:57-62,169-228`); scene manifest ~1 s per fixture (`TG/server.py:916-917`); fixture FCL BVHs ~2–4 s (`:934-936`); reach sweep ~17 s once per URDF change (`TG/robot_model.py:305-306`); full-res robot visual payload ~12 MB gzipped unless the baked `.npz` is present (`TG/server.py:1866-1871,1954-1966`). `compute_dual_arm_plan_collisions` / `compute_plan_frames` rebuild `UrdfModel` + `RobotCollisionModel` on **every call** (`WS/dual_arm_plan_collision.py:549-552,624-631`); the server compensates with content-hash disk caches (`TG/server.py:4303-4316,4659-4672`).
* `PackageIndex._SKIP` excludes `.venv`/`venv` but not `.venv-taught` (present at `ALIX/.venv-taught`) — (inferred) extra scan time on this machine (`WS/package_index.py:24-25`).

### 3.3 OS support
* Authoring path loads **no RealMan native library** (pinned by `TGP/test/test_studio_launcher.py:103-148`) → Linux/Windows/macOS.
* `--emulator`: Linux x86_64/aarch64 and Windows only; macOS fails inside vendor code (`NameError: libname`) and is refused by the preflight (`ALIX/tools/run_studio.py:27-50,92-139`; `ALIX/docs/EMULATOR.md:14-44`; `TGP/README.md:33-45`).

---

## 4. Emulator

### 4.1 What it is
* `alix_emulator.rm_emulator`: an in-process emulator of the RealMan RM_API2 Python SDK surface for two RM75-6FB arms, Gen-3 controllers, pole lifts; `install()` plants fake `Robotic_Arm`, `Robotic_Arm.rm_robot_interface`, `Robotic_Arm.rm_ctypes_wrap` modules in `sys.modules` and must run **before** code imports the SDK (`EMU/rm_emulator.py:1-35,2541-2616`; `ALIX/docs/EMULATOR.md:1-12,56-66`).
* It is **not vendor code**; it is "Newton's", verbatim-pinned by sha256 (`ALIX/core/python/src/alix_core/tests/test_emulator_verbatim.py:64,157-161`) — decision D4: no edits; extensions go at seams (`TG/sim_bridge.py:8-20`; `DP/emulated_program.py:1-12`).
* It still needs the **real vendored SDK**: FK/IK via RealMan's offline `Algo` (`RM_MODEL_RM_75_E` + `RM_MODEL_RM_ISF_E`) and real ctypes structs (`EMU/rm_emulator.py:82-110`).

### 4.2 What it emulates (fidelity table `ALIX/docs/EMULATOR.md:68-112`)
* Arm joints: `rm_movej` synchronized interpolation at v % of 180 °/s, mid-motion interpolated state, blocking/non-blocking, arrival events; joint-limit rejection (per-side table `EMU/rm_emulator.py:416-436`).
* `rm_movej_canfd` passthrough: state follows the command immediately, no arrival event, per-joint limit check (`EMU/rm_emulator.py:1728-1742,2339-2353`) — this is what program playback uses.
* Lift: queued moves with measured speed map + 0.38 s latency; lift-aborts-planned-move hardware defect with latched errors (`ALIX/docs/EMULATOR.md:75,114-136`).
* Inspire hand: `rm_set_hand_angle` (6 × 0–1000), measured stroke law (`:84`).
* UDP push subset, wrong-host trap, fault injection (`reject_next_dispatch`, `fail_next_motion`, `drop_next_event`, `lift_locked`, `emu_power_off/on`) (`:82,85,138-167`). Time scaling `RM_EMU_TIME_SCALE` / `set_time_scale` (`EMU/rm_emulator.py:31-32,472,518-524`).
* Not emulated (doc): Cartesian moves partly, CANFD passthrough (doc says not emulated, but code now has it — `EMU/rm_emulator.py:1728,2339`; the doc is stale), online programming, fences (`ALIX/docs/EMULATOR.md:94-100`).

### 4.3 How the Studio talks to it
* Same process only. `sim_bridge` reads the emulator's private `_by_ip` registry (never `emu_controller()`, which would fabricate arms) and converts `joints_deg` → radians `L_/R_joint1..7`, `lift_hw` × `lift_hw_to_phys` → metres `*_sliding_plate_joint`, `hand_hw` 0–1000 → finger radians via `HAND_CHANNELS` (`TG/sim_bridge.py:22-38,93-105,112-186`).
* Endpoints: `GET /api/sim/status` (fleet + external-feed age), `POST /api/sim/joints` (push any `{jointName: value}` map — "future: the real arms"), `POST /api/sim/pose` (FK of the joint map layered over the home pose through `_ik_context`, returns `{link: 4x4}`) (`TG/server.py:5309-5393`).
* Client: the **Simulate** view is a top-level `view-switch` peer of Author (not a ribbon tab), polls `/api/sim/pose` at 2/5/10 Hz and applies poses with `applyRobotPose` (`TG/web_editor/template.html:61-64,79-111`; `TG/web_editor/editor.js:19200-19325`). It carries the caveat "Command-level, not dynamics… A movel stroke shows as a hold (F20)" (`template.html:105-110`).

### 4.4 Can it take uploaded `.txt` programs / run program IDs? — **Yes, via `alix_dispatch` only**
* `DP/emulated_program.py` fakes the five program verbs at the connect seam: `rm_send_project` (parses the controller file format: CRLF mandatory, line 1 `{"file":DOF}`, line 2 `{"name":"Folder",…,"type":N}` with N = point count, then `{"point":[7 ints]}` rows; slot 1–100; stored name `f"{id}_{stem}.txt"` cut to 32 bytes), `rm_get_program_trajectory_list` (vendor dict shape, key `trajectory_name`), `rm_set_plan_speed`, `rm_get_program_run_state`, `rm_set_program_id_run` (implicit blocking movej to the first point at run speed, 0.35 s dwell, then streams every point via `movej_canfd` at `REPLAY_CLOCK_S = 0.0055545` s scaled by the emulator time scale; persistent global speed ratio; stop policy `aborts|ignores`) (`DP/emulated_program.py:1-58,77-125,128-174,285-547`).
* Observers: `ProgramStore.subscribe(ip, cb)` receives `start`, `transit_done`, `point` (`{"i", "deg", "ret"}`), `end`, `aborted` events (`DP/emulated_program.py:216,252-266`; emitted in `_play` `:430-498`, the `point` event at `:468`) — a ready hook for "where in the track was the arm when it stopped" (R5).
* Wiring: `alix_dispatch.connect.open_arm(side, ip)` opens the SDK handle (real or emulated, whichever `Robotic_Arm` is installed), sets the ack timeout (`RM_ACK_TIMEOUT_MS`, default 1500 ms, F122), and **wraps with `emulated_program.attach` only when `sdk_provenance()` says "emulated"** (`DP/connect.py:37-49,121-138,141-182`). `attach()` refuses a real handle (`DP/emulated_program.py:527-547`).
* The Studio's `--emulator` launcher creates bare `RoboticArm` handles (`ALIX/tools/run_studio.py:148-157`) — **no program verbs** — and the Studio has no upload/run route (`TGP/test/test_studio_interaction_guards.py:2204-2211`; grep of `TG/server.py` finds no `alix_dispatch`/`rm_send_project`).
* Gap vs the Lite runner: `LITE/upload_channel/rm_upload_fast_v14_safe_transit.py` (+ `pole.py`, `hand.py`) uses SDK verbs absent from emulator **even after** `attach`: `rm_start_drag_teach`, `rm_stop_drag_teach`, `rm_save_trajectory`, `rm_run_drag_trajectory`, `rm_drag_trajectory_origin`, `rm_stop_drag_trajectory`, `rm_update_program_trajectory`, `rm_get_global_waypoints_list`, `rm_get_given_global_waypoint`, `rm_change_work_frame`, `rm_get_current_work_frame`, `rm_set_timeout` (diff of `grep -o "rm_[a-z_]*("` over the runner vs `def rm_*` in `EMU/rm_emulator.py`). A Lite sim backend needs an extra seam wrapper (like `ProgramSurface`) for these; drag-teach itself cannot be simulated meaningfully (inferred).

---

## 5. Front-end test guards (what they enforce)

* `TGP/test/test_editor_js_integrity.py` (177 lines) — no JS engine; strips comments/strings/regex literals then: balanced `{}`/`()`/`[]` (`:113-126`); no duplicate column-0 `const|let|function|class` (`:128-140`); a named set of project helpers (`isTypingTarget`, `currentFixture`, `setFixture`, …, `initGates`) defined and not dangling (`:143-160`); `COVERAGE_TARGET_PCT` and `CLICK_DRAG_THRESHOLD_PX` declared exactly once (`:163-169`); file > 7,000 lines (`:172-177`).
* `TGP/test/test_editor_js_executes.py` (222 lines) — evaluates the whole `editor.js` top level in `quickjs` against a Proxy stub DOM/`THREE`/`Plotly` and fails on any top-level throw (`:41-42,55-147,161-181`); pins `const STATE_COLOR` before `const IK_STATUS_COLORS` (TDZ regression, `:184-203`; actual lines `TG/web_editor/editor.js:165,2047`); guard-the-guard test (`:206-222`).
* `TGP/test/test_studio_theme.py` (456 lines) — brand `--brand: #2F96EE` (`:40-44`); 17 tokens on bare `:root` (`:47-65`); dark variant redefines surfaces (`:68-72`); viewport not pure white (`:75-83`); old cyan `#00d4ff`/`0x00d4ff` gone (`:86-91`); 3-D palette synced from CSS (`syncColorsFromCss`, `:94-100`); view switch Author/Simulate exists, **simulate is not a ribbon tab** (`:128-146`); views hide each other via `body[data-view]`, a11y roles, leaving Simulate stops polling, movel/F20 caveat (`:149-176`); `<title>ALIX Studio</title>`/`<h1>ALIX Studio</h1>` (`:179-182`); inline-SVG brand mark, **no remote `<link>`/`<script>`/`<img>`** (`:185-195`); no dark colour literals, shadow tokens never composed/used as colours, structural `--surface-*` tokens in both themes, app-bar/ribbon use tokens, brand-mark facet geometry, canvas contrast (`:216-456`).
* `TGP/test/test_studio_interaction_guards.py` (2,525 lines, ~139 tests) — static "shape of the fix" checks per redesign-backlog row, with `_code_only` stripping comments because mutation testing showed comment text satisfying guards (`:36-50`). Highlights relevant to extension: wiring must be `initUi → runSteps([...['label', wireX]...])`, every `wire*` listed, `initUi` < 40 lines, `bind()` must null-check and collect `missingIds`, every bound id must exist (≥ 60 bindings) (`:1008-1081`); page gzip/memo/no-store, Plotly deferred with `ensurePlotly().then(ok => { if (ok) <self>(…) })`, vendor whitelist, robot geometry fetched via `requestIdleCallback` (`:1109-1202`); numbered tabs forbidden, exactly one `tab-page` per `tab-button`, no duplicate ids (`:1207-1243`); command band never scrolls, target sizes (`:1322-1835`); every tab has a `TAB_LEGENDS` entry, dock rules (`:2353-2525`); **"the studio has no execution channel… arms not connected"** (`:2204-2216`).
* Related guards: `TGP/test/test_design_tokens.py` (every token in `ALIX/design/_tokens.css` must exist in `editor.css` with the same numeric value per theme; card-only `.ds*` rules must not reach the app, `:1-22,81-141`); `TGP/test/test_constants_contract.py` (client literals pinned to `TG/constants.py`); `TGP/test_generate/test_route_needs.py` (every route has a `ROUTE_NEEDS` row, policies valid, busy contract behaviour, `/api/session` answers with `open` patched, `:31-197`); `TGP/test/test_studio_launcher.py` (launcher contract, no native lib on authoring import, emulator platform matrix, `:68-256`).
* Implication for Lite: these guards make editor.js/template edits expensive (option a), but the **kit is copyable** for a separate Lite front end (integrity + quickjs execution + token parity + no-remote-assets + route-needs tests).

---

## 6. Extensibility — adding a tab + endpoints

### 6.1 Server side (recipe)
1. Write `do_x(body)` (POST) or `do_x(query)` (GET, dict-of-lists) returning a dict; expected refusals as `{"error","hint"}` with HTTP 200.
2. Register in `ROUTES_GET`/`ROUTES_POST` (`TG/server.py:5961-6029`).
3. Add a `ROUTE_NEEDS` row — mandatory, `test_route_needs.py:31-42` fails otherwise (`TG/server.py:6064-6132`); add `ROUTE_BUSY_POLICY` if the handler handles busy itself (`:6134-6140`).
4. Multi-second work → job: copy the `start_run`/`run_status`/`cancel_run` pattern (`TG/studio_generate.py:537-600,861-1060`), inject collaborators (lock, path helpers) rather than importing the server (the documented one-way rule, `TG/server.py:5800-5808`; `TG/robot_model.py:1-25`).
5. A new page (not a tab) would need `do_GET` edits — only `/`, `/index.html` and `/vendor/` are served (`TG/server.py:6404-6421`) — and a second renderer, because `render_html` hard-codes `template.html`/`editor.css`/`editor.js` (`TG/scripts/visualize_generated_config.py:558-568`).

### 6.2 Client side (recipe)
1. `<button class="tab-button" data-tab="x">` in `<nav class="ribbon-tabs">` and exactly one `<div class="tab-page" data-page="x">` (`TG/web_editor/template.html:115-125,156-675`; guard `test_studio_interaction_guards.py:1226-1243`).
2. Per-tab hooks go in `switchTab(tab)` (`TG/web_editor/editor.js:7369-…`; buttons wired at `:7151-7152`).
3. Bindings in a new `wireX()` listed in `initUi`'s `runSteps` (`editor.js:9054-9078`), using `bind()`/`need()` (`:8784-8801`).
4. Add `TAB_LEGENDS.x` (`editor.js:7205`) and dock panel declarations (`data-tabs` or `DOCK_PANEL_TABS`, `:7166,7297-7314`).
5. Use tokens only (no literals), deferred Plotly via `ensurePlotly`, no numbered labels, nothing named `simulate`/`sim` as a tab.

### 6.3 Coupling to task-config YAML and fixtures
* 21 of 64 routes declare `read_configs`/`write_configs`/`generation_lock` (AST tally). The whole data model is **fixture → region (painted OBJ groups + `regions.yaml` + family CSV) → arm → one YAML task config** (`TG/server.py:195-311` region offering; `:5396-5486` board reads `<tasks>/config/<family>_cleaning/<fixture>/<region>_<arm>_cleaning_points.yaml`; `_save_roots` `:104-113`).
* The "free" plan/sim routes are still coupled to **MoveIt-style plan JSON** (`sub_trajectories[].joint_names/waypoints[].positions(rad)/time_from_start(s)`, sample: `ALIX/Resource/plans/commode_c/moveit2_seeded_free/seat_ring_bottom_right_ruckig_on_s.json`; parser `WS/dual_arm_plan_collision.py:46-87`) and to repo plan roots (`_resolve_plan` only accepts `.json` under `_PLAN_ROOTS`, `TG/server.py:4187-4197`). Lite's NDJSON millidegree `.txt` cannot be posted as-is.
* `/api/path-ik` (closest thing to R9 IK) is C-space-coupled: builds a `stroke_gate` oracle keyed by fixture/region/partner schedule, defaults fixture `commode_c` (`TG/studio_generate.py:4243-4323`) — X2-excluded territory.
* Hard-coded choices Lite must change: bin always `BIN_LOWERED` (`TG/server.py:1135,1529,1725,1991`; `WS/dual_arm_plan_collision.py:434`; docstring "not user-selectable" `TG/server.py:1767-1768`); sweep body set `kBodyCheckLinks = ("bin_link", "glove_wearing_unit_arm")` (`WS/dual_arm_plan_collision.py:644-648`); sweep home pole `dual_poles full_length` = 0.300 m (`:435`, trap explained `:405-421`); sweep hand default fully open unless a state is named (`:394-404`); **the server's sweep never passes the commode** (`extra_managers` has no caller in `TG/*.py`; parameter exists `WS/dual_arm_plan_collision.py:596,610-611,699`).

---

## 7. Reuse assessment and recommendation

### 7.1 Option (a) — Lite as a mode/page of the existing Studio
Pros: zero packaging work on the dev box; robot overlay, Simulate view, DAC playback, theme are already there; a `/lite` page plus `/api/lite/*` routes is mechanically possible (§6).
Cons (decisive):
* **Weight**: every launch imports the generator stack and spends 30–60 s to readiness (§3.2); 5.8 MB page; trimesh/shapely/ruamel/FCL mandatory.
* **Repo-bound**: marker/`.git`/SRDF/SDK path resolution (§2.2–2.3) — an operator laptop would need the full checkout (+ `vendor/realsense-ros` sibling) to get meshes and states.
* **Process-global hazards** Lite does not need: whole-interpreter `open` patching during generation, audit hook, one global generation lock with busy contracts (§1.4).
* **Pinned UI decisions contradict Lite's needs**: "no execution channel" (`test_studio_interaction_guards.py:2204-2211`), "simulate is not a tab" (`test_studio_theme.py:134-146`), "numbered tabs are gone" (`test_studio_interaction_guards.py:1207-1224`) vs Lite's guided region-by-region wizard and real-arm runs; ~140 structural guards on a 19k-line single-scope `editor.js`.
* **Wrong audience/data model**: Studio is a CAD-like authoring tool over fixture/region/arm YAML (§6.3); Lite is a guided operator flow over taught `.txt` tracks keyed by the PPT tasks 1A–15 (requirements file `LITE/alix_studio_lite/docs/reference/ALIX_Studio_Lite_requirement_inputs.txt:8`).
* Release coupling to a fast-moving tree (editor.js modified 2026-10-05; full suite ~475 s, `TGP/pyproject.toml:53-70`).
Verdict: acceptable only as a **throw-away dev prototype**, not as the product.

### 7.2 Option (b) — import `alix_taskgen` / `alix_workspace` as libraries
* `alix_workspace`: **yes**. ROS-free, pip-installable, small deps, clean APIs: `UrdfModel` (`fk`, `fk_batch`, `link_world_transforms`, `WS/urdf_kinematics.py:130-349`), `IKSolver` (DLS, `WS/ik.py:150-334`), `SrsArm` closed-form 7-DOF with `branches_at`, `wrap_into_limits`, `within_limits`, `limit_margin`, `jacobian` (`WS/srs.py:296-900`), `RobotCollisionModel`/`build_fixture_manager` (`WS/collision.py`), `compute_plan_frames`, `compute_dual_arm_plan_collisions(..., extra_managers=…, padding_m=…, left/right_hand_state=…)` (`WS/dual_arm_plan_collision.py:487-760`), `analyze_plan` (`WS/plan_analysis.py:1-25`), `SrdfStates` (`WS/srdf_states.py`). Caveats: mesh/SRDF/home-pose discovery must be made explicit (pass a `PackageIndex`, set `ALIX_SRDF`, ship `initial_positions.yaml`), models are rebuilt per sweep call (cache in Lite), the bin/pole/hand defaults above must be parameterised (small upstream patch or Lite-side subclass).
* `alix_taskgen`: **mostly no**. Its pip install drags the full generator deps and a 6.9 MB web bundle; `server` is not importable without the generator stack. The few Lite-useful modules are tiny and import-light (`robot_model` imports only `cache_locks` + `paths`, `TG/robot_model.py:27-42`; `sim_bridge` imports only `math`/`typing`, `TG/sim_bridge.py:51-54`) but carry repo-root coupling (`TG/robot_model.py:37-42,217`) and a latent bug (`_config_signature` calls `yaml.safe_load` but `robot_model.py` never imports `yaml`; the `NameError` is swallowed, so URDF/SRDF edits do not invalidate the cache — `TG/robot_model.py:110-137`, grep shows no `import yaml`). Better to copy (c) than import.

### 7.3 Option (c) — copy selected modules into a standalone package
Copy (small, stable, high value): server scaffold (`TG/server.py:6320-6469` handler/gzip/vendor whitelist/error payload, `:6228-6294` dispatch + session pattern, simplified), job registry pattern (`TG/studio_generate.py:537-600,861-1060`), page assembly (`TG/scripts/visualize_generated_config.py:530-568`, `TG/server.py:6297-6317`), theme (`ALIX/design/_tokens.css:8-62`, `TG/web_editor/editor.css:1-175`), brand mark (`TG/web_editor/template.html:15-51`), vendor libs (`TG/web_editor/vendor/*`), robot overlay JS (`TG/web_editor/editor.js:667,12352-12437`), DAC playback/highlight JS (`:12439-12448,14478-14600`), Simulate polling JS (`:19200-19325`), `sim_bridge` (`TG/sim_bridge.py`), launcher preflight (`ALIX/tools/run_studio.py:92-205`). Vendor (unchanged copies + seam wrappers): `alix_emulator` (verbatim), `DP/connect.py`, `DP/emulated_program.py`, the track I/O part of `DP/track.py` (`:51-160` parse/build/send/list; `:3706-3862` CANFD limits/preflight).

### 7.4 Recommendation — (c)+(b) hybrid, standalone `alix_studio_lite`
1. **Own process, own package, own port** (e.g. 8766 so it can run beside the Studio — inferred convenience). Stdlib `ThreadingHTTPServer` like the Studio (keeps Windows/Linux portability and zero web-framework deps), routes `/api/lite/*` with a **needs table** reused in spirit: replace `generation_lock` with a per-controller **robot-I/O lock** (one owner per controller is already the `alix_dispatch` rule, `DP/track.py:29`) and a `long_running` job model for upload/run/sweep/IK-offset jobs, polled like `/api/auto-path-status`.
2. **Front end**: new `template.html` with the same four-placeholder assembly; ALIX tokens copied verbatim from `design/_tokens.css` and kept honest by a copy of `test_design_tokens.py`; brand mark + app bar copied; vendor `three r128`, `OrbitControls`, `plotly 2.35.2` (identical version to the trajectory HTML's CDN tag `LITE/Alix_Studio_Lite_Trajectory_V2.html:7`, so its chart code ports unchanged once switched to the local copy). Non-technical UX: a guided wizard (Regions → Teach → Edit → Simulate → Real test → Upload) — numbered steps are fine here because Lite *is* a linear flow (contrast with the Studio's ruling).
3. **Kinematics/collision via `alix_workspace` (b)**, with a Lite **asset bundle** shipped explicitly: `WS/urdf/alix.urdf` (hardware-proven reference, FK-gated against the xacro by `ALIX/core/python/src/alix_core/tests/test_kinematic_gate.py:34-44,111-117`), collision meshes (~6.5 MB: rm 432K, hands 1.5M+1.5M, ugv 3.1M, pole 24K measured with `du`), the baked visual overlay (2.9 MB), `alix.srdf`, the five `initial_positions.yaml`, the commode scene (`ALIX/ros/orchestration/alix_tasks/config/commode_cleaning/commode_c/scene.yaml` + `models/*.stl`, 5.0 MB). Lay the bundle out with `package.xml` stubs (or rewrite URDF URIs to relative file paths — `resolve_uri` accepts plain paths, `WS/package_index.py:120-121`) and pass a prebuilt `PackageIndex` so no repo scan happens.
4. **Track ↔ plan adapter** (new, small): NDJSON millideg rows → one `execute_path` sub-trajectory (`L_/R_joint1..7` radians, `time_from_start = i × period / speed`), plus pole (`*_sliding_plate_joint` from the region's SRDF pole state) and hand state, plus finite-difference `velocities`/`accelerations` (`analyze_plan` reads declared values only, `WS/plan_analysis.py:114-115`). Unlocks `compute_plan_frames`, the dual-arm sweep (with `extra_managers` = commode managers built like `TG/server.py:931-953`) and `analyze_plan` without touching the Studio.
5. **Controller seam with two backends** (R4 toggle): *real* = SDK handle via `connect.open_arm(side, ip)` with operator-entered IPs; *sim* = `rm_emulator.install()` + the same `open_arm` (auto-wrapped by `emulated_program`) + a Lite seam wrapper for the runner verbs the emulator lacks (§4.4). Keep the field-proven v14 runner as the real-arm executor initially (run as a subprocess job, mirroring the Tkinter UI) and converge on `DP/track.py` semantics over time (inferred plan).
6. **Deployment to an operator laptop (no ROS, no repo)**: Python 3.10–3.13 (python-fcl wheels only, `ALIX/SETUP.md:469-475`), Linux x86_64 or Windows 64 (emulator support matrix), a wheelhouse for numpy/scipy/pyyaml/trimesh/python-fcl/pycollada/networkx/lxml + `alix_workspace` wheel, the asset bundle, the RealMan SDK for the target OS (licence question open, `ALIX/DEPENDENCIES.md:63-70`), env defaults set by the launcher (`ALIX_SRDF`, `BUTTERFLI_WORKSPACE_CONFIG`, `ALIX_WORKSPACE_CACHE`, `RM_LEFT_IP`/`RM_RIGHT_IP`). Startup target: seconds (no generator import; lazy model build; baked overlay). The existing control app is pure stdlib Tkinter (`LITE/upload_channel/rm_trajectory_ui_v53_safe_lid_checkpoint.py` imports only stdlib) and the runner needs only `Robotic_Arm` + local `pole.py`/`hand.py` — Lite's real-arm path stays that light; only the sim/collision path adds the scientific stack.

---

## Requirement relevance

* **R1 Region-wise data collection** — Studio's Board is the right *pattern* (region × arm cells, exists/missing, summary present/missing, per-cell async metrics) but reads task-config YAML under the tasks package (`TG/server.py:5396-5486`); design card `ALIX/design/components/board-table.html`. Lite: same payload/UI shape over `<project>/<task 1A..15>/<arm>/raw.txt`. Region identity comes from the PPT, not `regions.yaml` (`TG/server.py:195-311` not applicable). Timed "clean this region now" prompts are new.
* **R2 Trajectory editing** — no server-side editing/undo in the Studio; client undo is Studio-specific (`test_studio_interaction_guards.py:151-165,211-217`). Port the trajectory HTML logic; keep undo client-side (5-deep stack). Rate/step limits to respect when speeding up: `CANFD_MAX_RATE_DEG_S = 180`, `CANFD_MAX_STEP_DEG = 10`, per-joint rating `(180,180,225,225,225,225,225)` and 4105 evidence (`DP/track.py:3700-3729`).
* **R3 Simulation playback** — reuse robot overlay (`/api/robot-geometry` `TG/server.py:1918-2001`; `editor.js:12352-12437`) + frame playback (`compute_plan_frames` `WS/dual_arm_plan_collision.py:487-583`; `/api/plan-frames` `TG/server.py:4590-4674`) through the track→plan adapter; add the commode mesh (`build_scene_manifest`, `TG/scripts/visualize_generated_config.py:219`).
* **R4 Real-arm testing + real/sim toggle** — Studio has no execution channel (`test_studio_interaction_guards.py:2204-2211`); use `DP/connect.open_arm` (IP args, ack timeout) for real and the emulator + `emulated_program` for sim (§4.4); live joint feed for both can reuse `/api/sim/joints`-style push + `/api/sim/pose` FK (`TG/server.py:5326-5393`).
* **R5 Collision detection & graph highlighting** — reuse the DAC sweep (arm-vs-arm, self, body, `extra_managers` for the commode) and the DAC UI: bar timeline of hit samples, click-to-scrub (`dacGotoTime`), red link highlight (`dacHighlight`) (`TG/web_editor/editor.js:14478-14560`); Lite must add "stop at the joint value" on its joint plots by mapping sample index → track index (sim) or `ProgramStore` `point` events / UDP frames (real) (`DP/emulated_program.py:252-266,468`). Note DAC plot colours are hard-coded dark/cyan (`editor.js:14485-14488`) — retheme when lifting.
* **R6 Adjustable plots** — Plotly `responsive: true` (`editor.js:14492`); Studio docks have keyboard-operable separators, persisted width, maximize/restore (`test_studio_interaction_guards.py:2446-2511`) — reference-only pattern; the trajectory HTML is the base.
* **R7 Pole height & hand position per region** — SRDF named states: poles `full/three_quarter/half/quarter/minimum_length` = 0.300/0.225/0.150/0.075/0.010 m (`ALIX/ros/manipulation/alix_moveit_config/config/alix.srdf:271-285`), hands `release, thumb_grasp, tenth_grasp, half_grasp, open_tenth, grasp, quarter_grasp, pointing` (`alix.srdf:659-715`); exposed today only as pole presets (`/api/joint-presets`, `TG/server.py:1763-1785`). Unit mapping SDK↔URDF for hand/lift in `TG/sim_bridge.py:22-29,93-105,166-184`. Controller Base/World frame per pole state: `_arm_base_frames` (`TG/server.py:1788-1861`).
* **R8 TXT merging with commode collision scene** — sweep supports fixture managers but the Studio never passes them (§6.3); reuse `_fixture_collision_for` pattern (`TG/server.py:931-953`) + `compute_dual_arm_plan_collisions(extra_managers=…)`; merged output must obey the controller file contract (`DP/track.py:66-99`).
* **R9 Offset via FK→offset→IK** — library-level reuse: `UrdfModel.fk_batch`, `SrsArm` (closed-form, branch continuity, limits) or `IKSolver`; the Studio already has a "seed each IK from the previous solution for a continuous, flip-free motion" loop with pole pinning in `/api/animate-sequence` (`TG/server.py:1645-1760`) — the closest existing pattern to "no adjacent-point jumps"; SDK-parity alternative: vendor `Algo` via the emulator with FK re-verification at 2.0 mm / 0.6°, joint margin 5°, **joint jump 45°**, multi-seed (`ALIX/tools/verify_task_motion.py:88-125,395-407`; `EMU/rm_emulator.py:231-240,364-411`). Frame trap: controller Base vs World differ by the 90° mount; mixing them failed 3406/3469 IK samples (`TG/server.py:1806-1812`). Joint-limit tables disagree (see Open questions).
* **R10 Dual-arm speed optimization view** — reuse overlay + DAC + plan frames; "correct URDF" = `WS/urdf/alix.urdf` (FK-gated vs xacro, `test_kinematic_gate.py:34-44`); **bin toggle** requires parameterising `BIN_LOWERED` (5 sites) and `kBodyCheckLinks` (§6.3); SRDF offers `BIN_STOWED|BIN_EXTENDED|BIN_LOWERED` (`alix.srdf:286-297`).
* **R11 Smoothness with immediate sim validation** — needs warm in-memory models (the per-call rebuild in `WS/dual_arm_plan_collision.py:549-552,624-631` is too slow for "immediate"; the Studio's answer is disk caching keyed by content hash, `TG/server.py:4303-4316`); consider client-side FK from server-sent link frames (pattern `applyRobotPose`, `editor.js:12427-12437`).
* **R12 Plot spike analysis** — `analyze_plan` gives velocity/acc limit violations, joint-limit margin, manipulability per waypoint (`WS/plan_analysis.py:1-25`); `canfd_preflight(rows_millideg, period_s, rate_gate)` gives per-joint max step and peak rate utilisation against the controller's channel limits (`DP/track.py:3792-3866`).
* **R13 Faster upload** — not in the Studio. Findings live in `DP/track.py` (CRLF, Folder `type` = point count, `PROGRAM_LIST_PAGE = 10` to avoid SDK buffer overrun, global speed ratio survives power cycles → always restore, `:1-31,85-160`), `DP/connect.py:37-49` (ack timeout 1500 ms, F122) and the standalone `ALIX/tools/rm_upload.py:1-70` (upload + size verify + run). Alternative: CANFD streaming at the 5.0 ms controller cycle avoids the upload step and runs ~11 % faster than the player's 5.5545 ms average (`DP/track.py:3708-3719`) — higher-risk channel with stall/blind rules (`:3740-3764`).
* **R14 Offset test in sim before real** — same pipeline as R9 + R3 + emulator playback of the adjusted `.txt` (`DP/emulated_program.py:128-174,313-354,396-485`); emulator rejects out-of-limit canfd targets (`EMU/rm_emulator.py:1734-1736`).
* **X1 / X2 exclusions** — everything behind `ROUTE_NEEDS` long-running/generation-lock routes (`/api/generate`, `/api/auto-path*`, `/api/bulk-*`, `/api/auto-generate`, coverage, reachability, manipulability, redundancy atlas, path-compare, path-ik) is C-space/geometric-path or Workspace-tab machinery and is not needed (`TG/server.py:5988-6029,6064-6132`).
* **NFR-UX** — Studio design rules are written for "robot operators, not CAD professionals": ≥24 px targets (44 px primary), colour + glyph for every state, persistent dialogs, never-scrolling command band (`ALIX/design/README.md:9-37`; tokens `--target-min`, `--target-primary` `TG/web_editor/editor.css:127-128`). Reuse the rules, not the CAD layout.
* **NFR-THEME** — lift `ALIX/design/_tokens.css:8-62` (light default, dark via `:root[data-theme="dark"]`), brand `#2F96EE`, `--viewport-bg #1B2733`, Inter font stack, spacing `--s1..--s7`, radius 4 px, type scale 14/16/17/19/22 px (`TG/web_editor/editor.css:1-128,1714-1724`). The trajectory HTML uses a different palette (accent `#175cd3`, dark header `#0b1220`, radius 12 px, CDN Plotly) (`LITE/Alix_Studio_Lite_Trajectory_V2.html:7-40`) — remap to ALIX tokens and vendor Plotly to satisfy the "no remote assets" rule (`test_studio_theme.py:192-195`).
* **INT-CTRL** — Studio provides nothing for control; integrate the upload-channel runner as a server-side job (start/poll/cancel like `studio_generate`) and vendor `DP/connect` + `DP/track` for the controller seam; emulator coverage gap for the runner's verbs (§4.4).
* **INT-TRAJ** — port the HTML's processing/plots into Lite's front end; Plotly version already matches the Studio's vendored copy (v2.35.2).

---

## Reuse verdicts

| Feature | Verdict | What to lift | Evidence |
|---|---|---|---|
| HTTP scaffold (handler, gzip, 413 cap, disconnect handling, vendor whitelist, error payload) | reuse-with-adaptation | copy ~200 lines into Lite `server.py` | `TG/server.py:5178,6320-6469` |
| Route tables + needs/busy contract + `_dispatch` | reuse-with-adaptation | keep tables + needs; swap generation lock for per-controller robot-I/O lock | `TG/server.py:5961-6253` |
| Job registry (start/status/cancel, one live job, poll ms) | reuse-with-adaptation | pattern for upload/run/sweep/IK jobs | `TG/studio_generate.py:537-600,861-1060` |
| `/api/session` global-state answer | reference-only | a Lite `/api/lite/state` (controller owner, live job) | `TG/server.py:6255-6294` |
| Page assembly (placeholders, mtime-stamped cache, no-store) | reuse-with-adaptation | own template, same mechanism | `TG/scripts/visualize_generated_config.py:530-568`; `TG/server.py:6297-6317,6402-6418` |
| Theme tokens (light/dark) | reuse-as-is | `design/_tokens.css` minus `.ds*`; token-parity test | `ALIX/design/_tokens.css:8-62`; `TGP/test/test_design_tokens.py:81-141` |
| Brand mark / app bar | reuse-as-is | inline SVG + header markup | `TG/web_editor/template.html:15-51` |
| Vendored JS (three r128, Orbit/TransformControls, Plotly 2.35.2, js-yaml) | reuse-as-is | copy `web_editor/vendor/` | `TG/web_editor/vendor/`; `visualize_generated_config.py:534` |
| Robot overlay (geometry endpoint + `loadRobotGeometry`/`applyRobotPose` + baked npz) | reuse-with-adaptation | endpoint logic + JS; serve baked overlay | `TG/server.py:1918-2001`; `TG/web_editor/editor.js:667,12352-12437`; `WS/workspace_data/visual_overlay_baked.npz` |
| Dual-arm FCL sweep + DAC timeline/highlight UI | reuse-with-adaptation | add commode `extra_managers`, bin toggle, hand/pole states; retheme plot | `WS/dual_arm_plan_collision.py:586-760`; `TG/server.py:4335-4366`; `editor.js:14478-14600` |
| Plan-frame FK playback | reuse-with-adaptation | via track→plan adapter; in-memory model cache | `WS/dual_arm_plan_collision.py:487-583`; `TG/server.py:4590-4674` |
| Plan analysis (vel/acc/limits/manipulability) | reuse-with-adaptation | synthesize velocities/accelerations | `WS/plan_analysis.py:1-25,114-115`; `TG/server.py:4272-4293` |
| Simulate view + `sim_bridge` + `/api/sim/*` | reuse-with-adaptation | live joint feed for sim and real | `TG/sim_bridge.py`; `TG/server.py:5309-5393`; `editor.js:19200-19325` |
| RealMan SDK emulator | reuse-as-is (vendored, Linux/Windows) | `alix_emulator` verbatim + time scaling | `EMU/rm_emulator.py`; `ALIX/docs/EMULATOR.md` |
| Program-channel fake (`emulated_program`) + `connect.open_arm` | reuse-with-adaptation | add seam fakes for drag-teach / work-frame / `rm_update_program_trajectory` | `DP/emulated_program.py`; `DP/connect.py:121-182` |
| Track I/O, upload, program list, CANFD limits/preflight | reuse-with-adaptation | vendor `DP/track.py` subset (no pyproject) | `DP/track.py:51-160,3706-3866`; `ALIX/LAYERS.md:25` |
| Standalone uploader CLI | reference-only | upload+verify+run flow, SDK discovery | `ALIX/tools/rm_upload.py:1-70` |
| SRDF named states (pole/hand/bin) | reuse-with-adaptation | `SrdfStates` + bundled SRDF; expose per region | `WS/srdf_states.py:61-154`; `TG/server.py:1763-1785`; `alix.srdf:271-297,659-715` |
| FK/IK primitives | reuse-as-is (library) | `UrdfModel`, `IKSolver`, `SrsArm` | `WS/urdf_kinematics.py:130-349`; `WS/ik.py:150-334`; `WS/srs.py:296-900` |
| SDK-parity IK verification (Algo, 2 mm/0.6°, 45° jump) | reference-only | thresholds + multi-seed logic for R9/R14 | `ALIX/tools/verify_task_motion.py:88-125,395-407` |
| Controller Base/World frames per pole state | reference-only | frame semantics for offset UI | `TG/server.py:1788-1861` |
| Commode collision world + scene manifest | reuse-with-adaptation | `build_fixture_manager` over bundled commode meshes | `TG/server.py:914-953`; `TG/scripts/visualize_generated_config.py:219` |
| Fixture Board (region × arm present/missing) | reference-only | dashboard shape for R1 | `TG/server.py:5396-5486`; `ALIX/design/components/board-table.html` |
| Region model (painted OBJ, regions.yaml, family CSV) | not-applicable | Lite regions = PPT tasks 1A–15 | `TG/server.py:195-311` |
| Generation / C-space / Workspace endpoints | not-applicable (X1/X2) | — | `TG/server.py:5810-5960,6064-6132` |
| Front-end guard tests | reuse-with-adaptation | integrity, quickjs execution, theme, route-needs kit | `TGP/test/test_editor_js_integrity.py`; `test_editor_js_executes.py`; `test_studio_theme.py`; `TGP/test_generate/test_route_needs.py` |
| Launcher (preflight, neutral cwd, browser open) | reuse-with-adaptation | Lite launcher with asset/SDK preflight | `ALIX/tools/run_studio.py:92-205`; `TG/server.py:6493-6507` |
| Studio as host for Lite (option a) | not recommended | only for a dev prototype | §7.1 |

---

## Gaps and risks found while mapping

* `TG/robot_model.py:117` uses `yaml.safe_load` with no `import yaml` in the module; the `NameError` is swallowed (`:129-130`), so `_config_signature` never includes URDF/SRDF mtimes → stale IK/geometry caches until restart (the docstring at `:110-114` says the opposite).
* `ALIX/docs/EMULATOR.md:94-100` says CANFD passthrough is not emulated, but `EMU/rm_emulator.py:1728,2339` implements it — doc drift.
* `TGP/README.md:13` documents `cleaning_path_studio`; the entry point is `alix-studio` (`TGP/pyproject.toml:45`).
* The Studio's dual-arm sweep ignores the commode (no `extra_managers` caller) — R5/R8 would be falsely "clear" against fixture contact if reused naively.
* Bin is hard-wired `BIN_LOWERED`; sweep pole default 0.300 m and hand default fully open silently change verdicts (`WS/dual_arm_plan_collision.py:388-421`).
* DAC plot uses hard-coded dark-theme colours (`rgba(0,212,255,.25)`, `#e6edf3`) that escape the theme test's `0x00d4ff` check (`TG/web_editor/editor.js:14485-14488`; `TGP/test/test_studio_theme.py:86-91`).
* Joint-limit sources conflict: `ALIX/tools/verify_task_motion.py:268-274` (`[178,130,178,135,178,128,360]`, "document wins") vs emulator per-side measured table (`[183,135,183,169|140,360,133,365]`, `EMU/rm_emulator.py:416-436`). R9/R14 must pick one.
* Emulator is verbatim-pinned; any missing verb must be added at a seam, never inside `rm_emulator.py`.
* RealMan SDK licence is unknown — redistribution on operator laptops is an open legal question (`ALIX/DEPENDENCIES.md:63-70`).
* No auth on the Studio server; a Lite server with robot-motion endpoints must stay on loopback or add a confirmation/auth layer (`TG/server.py:6539-6544`).
* `python-fcl` has no sdist → Python version outside 3.10–3.13 is a hard stop (`ALIX/SETUP.md:469-475`).
* HTTP/1.0 with no request timeout — long robot runs must be jobs, never held requests (`TG/server.py:5814-5819`).

## Open questions

1. Which joint-limit table is authoritative for Lite's IK/offset checks (ROBOT_MODEL.md list vs the per-side table read from the arms on 2026-08-11)?
2. Operator laptop OS: Linux or Windows (emulator works) — or macOS (sim backend unavailable)?
3. Is redistributing the RealMan SDK binaries with Lite acceptable given the missing licence?
4. In which frame should operators enter offsets (controller Base, controller World, or `base_link`/`alix_ref_frame`)?
5. Should the sim backend replay through `emulated_program` (movej transit + canfd stream at 5.5545 ms) or through Lite's own FK playback only? (Drag-teach verbs cannot be emulated.)
6. Will Lite own a separate port (e.g. 8766) and data folder, or must it share the Studio's `Resource/plans` / tasks tree?
7. Is the v14 runner kept as the real-arm executor (subprocess) or replaced by `alix_dispatch.track` calls in-process?

---

## Appendix A — Full route catalogue (64 routes, AST-extracted from `TG/server.py:5961-6140`)

Needs: R = read_configs, W = write_configs, L = generation_lock, J = long_running, `-` = none. Busy policy is `wait` unless noted. "Lite" column: KEEP = lift the endpoint logic, ADAPT = lift with changes, PATTERN = lift the mechanism only, REF = read for ideas, DROP = excluded (X1/X2) or Studio-only.

| Method | Route | Handler (line) | Needs | Lite | Why / requirement |
|---|---|---|---|---|---|
| GET | `/api/sim/status` | `do_sim_status` (5312) | - | KEEP | emulator/external feed health (R3, R4) |
| GET | `/api/fixture-board` | `do_fixture_board` (5396) | R | PATTERN | region × arm present/missing dashboard (R1) |
| GET | `/api/board-cell` | `do_board_cell` (5562) | R,L | DROP | coverage per cell (X1) |
| GET | `/api/deploy-path` | `do_deploy_path` (5776) | - | DROP | YAML deploy destination |
| GET | `/api/schedule-pairs` | `do_schedule_pairs` (5180) | - | REF | dual-arm pairing from schedule CSV (R8, R10) |
| GET | `/api/fixtures` | lambda → `list_fixtures` (158) | - | ADAPT | Lite has one commode scene bundle |
| GET | `/api/schedule` | `do_schedule` (4695) | - | REF | orchestrator step order/pairing (R8, R10) |
| GET | `/api/plans` | `list_plans` (4175) | - | ADAPT | list Lite tracks instead of plan JSON |
| GET | `/api/regions` | lambda → `list_regions` (259) | - | DROP | painted-mesh regions; Lite uses PPT tasks |
| GET | `/api/manifest` | `get_manifest` (4132) | R | ADAPT | commode meshes for the 3-D view (R3) |
| GET | `/api/ik-frames` | lambda → `load_ik_frames` (3453) | - | REF | tool-frame names for R9 |
| GET | `/api/glove-frames` | lambda → `_glove_frames_payload` (3368) | - | REF | glove/pad frames for R9 |
| GET | `/api/gizmos` | lambda → `list_gizmos` (3467) | - | DROP | calibration gizmos |
| GET | `/api/hand-gizmo` | `do_hand_gizmo` (3376) | - | REF | hand posed at an SRDF hand state (R7 preview) |
| GET | `/api/gizmo` | `get_gizmo` (3479) | - | DROP | — |
| GET | `/api/robot-geometry` | `do_robot_geometry` (1918) | - | KEEP | robot meshes + home pose (R3, R10) |
| GET | `/api/joint-presets` | `do_joint_presets` (1763) | - | KEEP | SRDF pole states + controller frames (R7, R9) |
| GET | `/api/mechanism-ops` | `do_mechanism_ops` (4200) | R | DROP | mechanism authoring |
| GET | `/api/mechanism-exemplar` | `do_mechanism_exemplar` (4208) | R | DROP | mechanism authoring |
| GET | `/api/atlas-environment` | `do_atlas_environment` (2509) | - | DROP | redundancy atlas (X2) |
| GET | `/api/auto-path-status` | `do_auto_path_status` (5861) | - | PATTERN | job polling contract |
| GET | `/api/session` | lambda → `do_session` (6255) | - | PATTERN | global state answer |
| GET | `/api/bulk-generate-status` | `do_bulk_generate_status` (5911) | - | DROP | X1 |
| GET | `/api/bulk-generate-config` | `do_bulk_generate_config` (5923) | - | DROP | X1 |
| POST | `/api/sim/joints` | `do_sim_joints` (5326) | - | KEEP | push a live joint map (real-arm feed, R4/R5) |
| POST | `/api/sim/pose` | `do_sim_pose` (5344) | - | KEEP | FK joint map → link poses (R3, R4) |
| POST | `/api/generate` | `do_generate` (437) | R,L | DROP | legacy raster generator (X1) |
| POST | `/api/mirror` | `do_mirror` (3131) | - | DROP | YAML mirroring |
| POST | `/api/retarget` | `do_retarget` (2964) | - | DROP | YAML retarget |
| POST | `/api/validate` | `do_validate` (3190) | R,L | DROP | YAML gate |
| POST | `/api/verify-save` | `do_verify_save` (3221) | R | DROP | YAML gate |
| POST | `/api/save` | `do_save` (3291) | W,L | PATTERN | whitelisted-root save + mtime conflict guard idea |
| POST | `/api/save-ik-frames` | `save_ik_frames` (3460) | - | DROP | — |
| POST | `/api/save-gizmo` | `save_gizmo` (3471) | - | DROP | — |
| POST | `/api/save-regions-obj` | `save_regions_obj` (3520) | - | DROP | region painting |
| POST | `/api/update-region` | `update_region` (4030) | - | DROP | region painting |
| POST | `/api/coverage` | `do_coverage` (595) | R,L | DROP | X1 |
| POST | `/api/reachability` | `do_reachability` (800) | - | DROP | Workspace tab (X2) |
| POST | `/api/region-reachability` | `do_region_reachability` (850) | - | DROP | X2 |
| POST | `/api/reachable-mask` | `do_reachable_mask` (866) | - | DROP | X2 |
| POST | `/api/ik` | `do_ik` (1095) | - | ADAPT | per-point IK + fixture contact; batch it for R9 |
| POST | `/api/animate-sequence` | `do_animate_sequence` (1645) | - | REF | IK seeded from previous solution = flip-free continuation (R9, R14) |
| POST | `/api/manipulability` | `do_manipulability` (2060) | - | DROP | X2 |
| POST | `/api/redundancy-atlas` | `do_redundancy_atlas` (2568) | L (degrade) | DROP | X2 |
| POST | `/api/atlas-pose` | `do_atlas_pose` (2412) | - | DROP | X2 |
| POST | `/api/atlas-sweep` | `do_atlas_sweep` (2182) | R,L | DROP | X2 |
| POST | `/api/analyze-points` | `do_analyze_points` (1404) | - | REF | per-point reach/singularity verdicts |
| POST | `/api/analyze-config` | `do_analyze_config` (1563) | - | DROP | YAML analysis |
| POST | `/api/analyze-plan` | `do_analyze_plan` (4272) | - | ADAPT | per-waypoint limits/velocity (R12) |
| POST | `/api/generate-mechanism` | `do_generate_mechanism` (4246) | R,L | DROP | — |
| POST | `/api/dual-arm-collisions` | `do_dual_arm_collisions` (4335) | - | ADAPT | FCL sweep (R5, R8, R10) — add commode + bin toggle |
| POST | `/api/plan-frames` | `do_plan_frames` (4590) | R | ADAPT | FK frames for playback (R3, R10, R11) |
| POST | `/api/schedule-preview` | `do_schedule_preview` (5043) | - | REF | ordering + guard tiers (R8) |
| POST | `/api/schedule-save` | `do_schedule_save` (5080) | - | DROP | orchestrator CSV |
| POST | `/api/auto-generate` | `do_auto_generate` (4369) | R,L,J | DROP | X1 |
| POST | `/api/auto-path` | `do_auto_path` (5810) | L,J (job) | PATTERN | job starter shape |
| POST | `/api/auto-path-reauthor` | `do_auto_path_reauthor` (5837) | L,J (job) | DROP | X1 |
| POST | `/api/start-pose-check` | `do_start_pose_check` (5871) | R,L | DROP | X1 |
| POST | `/api/auto-path-cancel` | `do_auto_path_cancel` (5889) | - | PATTERN | cooperative cancel |
| POST | `/api/path-compare` | `do_path_compare` (5936) | - | DROP | X1 |
| POST | `/api/path-ik` | `do_path_ik` (5946) | R (degrade) | DROP | C-space oracle (X2) |
| POST | `/api/bulk-generate` | `do_bulk_generate` (5899) | L,J (job) | DROP | X1 |
| POST | `/api/bulk-generate-cancel` | `do_bulk_generate_cancel` (5917) | - | DROP | X1 |
| POST | `/api/bulk-generate-apply` | `do_bulk_generate_apply` (5930) | W | DROP | X1 |

Count: KEEP 5, ADAPT 7, PATTERN 6, REF 8, DROP 38 (= 64). Only 12 routes (~19 %) carry code Lite would reuse (KEEP + ADAPT), and most of those need the track→plan adapter or new parameters.

## Appendix B — Payload shapes of the endpoints Lite would lift

* `GET /api/robot-geometry?set=visual|collision[&config=]` → `{"meshes": [{"link", "vertices": [[x,y,z]…], "faces": [[i,j,k]…], "color"?}], "homePose": {link: 16 floats row-major}, "count", "set"}`; link-frame vertices, client applies per-link 4×4; cached on `(config, set, _config_signature, baked_stat)` (`TG/server.py:1918-2001`). Client: `meshFromPayload` (`TG/web_editor/editor.js:667`), `loadRobotGeometry` (`:12352-12392`), `applyRobotPose(poseMap)` uses `THREE.Matrix4.set(...16)` row-major (`:12427-12437`).
* `GET /api/joint-presets[?config=]` → `{"poleStates": [{"name", "value"}] (longest first, `_length` states only), "default", "armBaseFrames": {state: {side: {"base": 16, "world": 16}}}}` (`TG/server.py:1763-1785,1788-1861`).
* `POST /api/sim/joints {"joints": {name: number}, "source"?}` → `{"ok": true, "jointCount"}`; `POST /api/sim/pose {"source": "auto|emulator|external", "arm"?, "config"?, "tipFrame"?}` → `{"source", "arm", "jointCount", "pose": {link: 16 floats}}`; `GET /api/sim/status` → `{"running", "arms": [{"ip","side","joints_deg","lift_hw_mm","hand_hw","powered"}], "external": {"present","source","ageSeconds","jointCount"}}` (`TG/server.py:5312-5393`; `TG/sim_bridge.py:189-208`).
* `POST /api/dual-arm-collisions {"leftPlan", "rightPlan", "dt"=0.05, "window"=100}` → `{"left_plan","right_plan","urdf","dt","num_samples","left_links","right_links","body_links","checks":[…],"disabled_pairs","sample_times","collisions":[{"index","time","pairs":[[a,b]…]}],"windows":[{"center","start","end"}],"frames":{sample:{link:16}},"mesh_links","leftPlan","rightPlan","cached"}` (server drops `joint_timeline`) (`WS/dual_arm_plan_collision.py:739-760`; `TG/server.py:4335-4366`).
* `POST /api/plan-frames {"plans": [path…]} | {"tracks": [{"plans": [path…], "start": s}]}, "dt"=0.05, "initial"?: {joint: value}, "mech"?` → `{"tracks","urdf","dt","num_samples","sample_times","frames","mesh_links","final_joints","cached"}` (+ `hinge_angles` for mechanisms) (`WS/dual_arm_plan_collision.py:487-583`; `TG/server.py:4590-4674`).
* `POST /api/analyze-plan {"plan", "chain"?, "config"?, "tipFrame"?, "includeEnvelope"=true}` → per-waypoint FK tip, manipulability, joint-limit margin, velocity/acc violations (`WS/plan_analysis.py:1-25`; `TG/server.py:4272-4293`).
* Job contract (to copy for Lite upload/run/sweep jobs): start → `{"runId", "state": "queued", "pollMs": 2000, "etaSecondsRange", "cancelMayTakeUntilS"}`; status → `{"runId","state","phase","startedAt","elapsedS","progress":{"stage","stageIndex","of","stages","stageElapsedS","stageSecondsSoFar","sub"},"partials","notes","request", "result"|"error"+"hint"+"failure"|"cancelled"}`; cancel → `{"runId","state","cancelPending","mayTakeUntilS","note"}` (`TG/studio_generate.py:934-937,973-1029,1032-1060`).
* Plan JSON the WS functions consume (what Lite's track→plan adapter must emit): top-level `task_name`, `num_sub_trajectories`, `sub_trajectories[]` each with `index`, `stage_name`, `hold_before_s`, `trajectory_mode`, `joint_names` (e.g. `R_joint1..7`, `R_sliding_plate_joint`, `IRH_right_*_joint`), `num_waypoints`, `waypoints[]` of `{time_from_start (s), positions (rad | m), velocities, accelerations}` (sampled from `ALIX/Resource/plans/commode_c/moveit2_seeded_free/seat_ring_bottom_right_ruckig_on_s.json`; parser `WS/dual_arm_plan_collision.py:46-87`).
