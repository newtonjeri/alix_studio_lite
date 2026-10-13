# ALIX Studio Lite

A guided application for **non-technical operators** to teach, tune, check and run drag-taught
cleaning trajectories on the dual-arm RealMan RM75 robot (commode C). It reuses ALIX's proven code:
its kinematics and 3D robot, its optimisers, its robot I/O library and its control program.

It runs in a web browser on the operator's computer. It has two front ends on one backend:
- the **Lite UI**, at `/`: the designer's *ALIX Studio LITE Edition V19* page, connected to the backend;
- **Legacy Lite**, at `/legacy/`: the earlier ALIX-themed UI, kept working.

**The repository is self-contained.** Everything Lite needs is in this folder: no ALIX checkout and
no ROS.

---

## Get it running (one command)

You need **Python 3.11 or newer** and **git**. The installer says how to get either if one is
missing. The repository is **private**, so sign in to GitHub once first. Any one of these works:

- GitHub CLI: `gh auth login`
- an SSH key on your GitHub account
- a personal access token, typed as the password when git asks

Then, on Linux or macOS:

```bash
git clone -b lite-ui https://github.com/newtonjeri/alix_studio_lite.git
python3 alix_studio_lite/install_lite.py
```

On Windows (PowerShell):

```powershell
git clone -b lite-ui https://github.com/newtonjeri/alix_studio_lite.git
py alix_studio_lite\install_lite.py
```

Have only the file [`install_lite.py`](install_lite.py), for example from a USB stick? Run it on its
own. It clones the repository into `~/alix_studio_lite`, or into `--dir`, and installs it.

**What the installer does.** The first run takes about 5 minutes, most of it downloading packages.
1. It checks Python (with `venv`) and git.
2. It clones the code, or updates an existing copy. It never pulls over changes you made.
3. It builds a private environment, `.venv`, inside the copy, and installs the pinned packages. Nothing is installed system-wide.
4. It checks that everything works by running the test suite, which takes about 2 minutes. Use `--no-tests` to skip this.

| Option | What it does |
|---|---|
| `--dir PATH` | Installs somewhere else (default: the copy it sits in, else `~/alix_studio_lite`). |
| `--shortcut` | Adds an "ALIX Studio Lite" launcher to the desktop and the applications menu. |
| `--start` | Starts Lite when it is installed. |
| `--no-tests` | Skips the test suite at the end. |
| `--no-optional` | Skips the optional packages (toppra, meshoptimizer). |

**To update later,** run the installer again: `python3 install_lite.py` inside the copy.

---

## Start Lite

```bash
alix_studio_lite/tools/run_lite.sh                # Windows: py alix_studio_lite\tools\run_lite.py
```

A browser tab opens at **http://127.0.0.1:8766/**.
- Log in with the operator PIN. Advanced Robot Controls ask for the engineer PIN.
- Both PINs are **`1234`** until changed: `tools/run_lite.sh --set-pin operator`, or `--set-pin engineer`.
- STOP BOTH always works, with or without a log-in.
- Projects and settings live in `~/ALIX-Lite/`, outside the code, so updating never touches them.

Other options:

| Option | What it does |
|---|---|
| `--no-browser` | Does not open a browser tab. |
| `--port 8800` | Uses another port. The default 8766 runs beside ALIX Studio (8765). |
| `--data DIR` | Keeps the projects and settings somewhere other than `~/ALIX-Lite/`. |
| `--ui legacy` | Opens Legacy Lite instead. |
| `--robot realman` | Connects to the real arms: see below. |

---

## Connect to the robot

```bash
alix_studio_lite/tools/run_lite.sh --robot realman
```

1. Put the computer on the arms' network, **192.168.1.x**. The arms are **192.168.1.10** (LEFT) and **192.168.1.103** (RIGHT), on port 8080.
2. In Lite, click the status pill at the top. It opens **Robot & mode**.
3. Check each arm's address; it can be edited before connecting. Then press **Connect**.

Lite connects through ALIX's own robot code (`alix_dispatch`). It then shows the live feedback of
the three devices on each side, refreshed about five times a second:

| Device | What Lite reads |
|---|---|
| **Arm** (RM75) | Its seven joints, its error codes, and the controller's run mode (SIM or REAL). |
| **Pole** (lift) | Its height in mm. |
| **Hand** (Inspire RH56) | Where its six fingers actually are (0–1000, 1000 = open). |
| **E-stop** (IO3) | Clear, pressed, unknown, or *not configured*. IO3 must be in the controller's E-stop mode (5); RealMan's demo maps it to "continue program". |

The connection is **read-only**: Lite commands nothing on the robot yet.
- **Robot actions stay with the stand-in.** That covers runs, moves, the pole and the hand. **Real robot** is not available, so use the control program for real runs meanwhile.
- **The live E-stop still guards Lite.** While either arm's E-stop is not clear, every robot action in Lite is refused on both arms.

If an arm cannot be reached, Lite says so, with its address. Check that the controller is on, the
cable, and that this computer has a 192.168.1.x address (`ip addr`).

**No robot at hand?** `tools/run_lite.sh --robot emulated` connects to ALIX's emulated arms instead.
Without `--robot`, Lite uses a stand-in, so every robot screen works and nothing is connected.

### The three modes

The status pill at the top chooses where the region steps run. Lite always starts in Visual sim.

| Mode | What moves |
|---|---|
| **Visual sim** | Only the 3D robot on screen. Nothing is sent to the robot. |
| **Hardware sim** | **Nothing physical.** The controllers run programs in their own SIM mode, so the arms stay still. The pole and the hand are **emulated**: Lite takes their measured times, but sends them nothing. |
| **Real robot** | The arms, the pole and the hand. Every move asks for confirmation first. Not available yet (see above). |

Whatever the mode, every execution stages the file's own pole height and hand pose first. If
either does not arrive, nothing else runs. A trajectory runs only on the arm it was taught on, and
only after tuning has passed every limit. [`docs/UI_V19_CONTRACT.md`](docs/UI_V19_CONTRACT.md),
"The gates on every execution", explains each gate.

---

## When something goes wrong

| Message or symptom | What to do |
|---|---|
| `this Python has no venv module` | Ubuntu or Debian: `sudo apt install python3.12-venv` (use your Python version). Then run the installer again. |
| `the repository could not be cloned` | Sign in first (see above), then run the installer again. |
| `Cannot use port 8766` | Lite is probably already running: open http://127.0.0.1:8766/. Or start it with `--port 8800`. |
| `Lite cannot reach the LEFT arm's controller` | Check the power, the cable, the address in Robot & mode, and that the computer is on 192.168.1.x. |
| E-stop chip reads *not configured* | IO3 is not in the controller's E-stop mode. An engineer sets it at the pendant; Lite never writes IO modes. |
| Time-optimal optimiser missing | `toppra` has no ready-made package for this computer. The Default optimiser still works. |
| A ROS workspace is sourced | Nothing to do: Lite clears `PYTHONPATH` and uses its own environment. |

---

## For developers

### Run the tests

```bash
cd alix_studio_lite
.venv/bin/python -m pytest -q          # Windows: .venv\Scripts\python -m pytest -q   (about 2 minutes; no robot, no network)
```

The tests cover:
- the vendored ALIX code and its manifest;
- the cubicles and the robot model;
- ALIX's TOPP-RA chain against its reference result;
- the Lite UI page;
- the API;
- the region loop against the stand-in, including the gates on every execution;
- staging, and L3 against ALIX's own reading;
- the Trajectory Viewer;
- the full sequence;
- the dark theme;
- the read-only robot link, against ALIX's emulated arms.

### Where things are

| Folder | What it holds |
|---|---|
| `src/alix_studio_lite/` | Lite's code. See the rows below. |
| `src/alix_studio_lite/motion/` | Tracks, the 5 ms clock, limits and checks. |
| `src/alix_studio_lite/optimise/` | The Default and Time-optimal optimisers. |
| `src/alix_studio_lite/server/` | The local web server: projects, log-in, the robot stand-in and the read-only robot link (`robot_link.py`). |
| `src/alix_studio_lite/web/` | The Lite UI: the designer's page plus `lite/`, its bridge to the backend. |
| `src/alix_studio_lite/web_legacy/` | Legacy Lite. |
| `vendor/alix/` | The parts of ALIX that Lite reuses, in ALIX's own layout. `vendor/MANIFEST.json` lists every file with its sha256. |
| `vendor/control_app/` | The control program: runner v14, `hand.py`, `pole.py`. |
| `assets/` | Cubicle walls, the regions deck and catalogue, the robot overlay, ALIX's brand mark. |
| `docs/` | Requirements, plan, build status, the UI contract, analysis. |
| `tests/` | The tests and their data (taught recordings, the legacy V6 package). |
| `tools/` | Setup, the launcher and the maintenance tools below. |
| `install_lite.py` | The clone-and-install script. |

The maintenance tools in `tools/`:

| Tool | What it does |
|---|---|
| `setup_lite.py` | Installs the environment; `install_lite.py` calls it. |
| `run_lite.sh` / `run_lite.py` | Starts Lite. |
| `vendor_from_alix.py` | Refreshes `vendor/` from an ALIX checkout: `python3 tools/vendor_from_alix.py --alix ~/alix_ws/src/alix`. |
| `make_dark_css.py` | Regenerates the Lite UI's dark theme when the designer's page changes. |
| `make_cubicle.py` | Builds a cubicle wall model, for example `python3 tools/make_cubicle.py 1400`. |
| `build_catalogue.py` | Turns the region sign-off CSV into the catalogue. |
| `bake_robot_overlay.py` | Re-bakes the 3D robot overlay. It needs the optional packages. |

### Read more

- [`docs/STATUS.md`](docs/STATUS.md): what works and what comes next.
- [`docs/UI_V19_CONTRACT.md`](docs/UI_V19_CONTRACT.md): each screen, what it does, and the gates on every execution.
- [`docs/ALIX_Studio_Lite_Requirements_Revised.md`](docs/ALIX_Studio_Lite_Requirements_Revised.md): the requirements (final, v1.6).
- [`docs/ALIX_Studio_Lite_Implementation_Plan.md`](docs/ALIX_Studio_Lite_Implementation_Plan.md): the implementation plan.
