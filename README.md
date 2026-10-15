# ALIX Studio Lite

A guided application for **non-technical operators** to teach, tune, check and run drag-taught
cleaning trajectories on the dual-arm RealMan RM75 robot (commode C). It reuses ALIX's proven code:
its kinematics and 3D robot, its optimisers, its robot I/O library and its control program.

It runs in a web browser on the operator's computer. It has two front ends on one backend:
- the **Lite UI**, at `/`: the designer's *ASL* page (V32, which replaced V19 on 14 October), connected to the backend;
- **Legacy Lite**, at `/legacy/`: the earlier ALIX-themed UI, kept working. It asks for the same operator PIN.

**The repository is self-contained.** Everything Lite needs is in this folder: no ALIX checkout and
no ROS.

---

## Get it running (one command)

You need **Python 3.12, 3.13 or 3.14** and **git**. If another Python runs the installer, it finds
one of these by itself; if there is none, it says how to get one. The repository is **private**, so
sign in to GitHub once first. Any one of these works:

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
2. It clones the code, or updates the branch an existing copy is on. It never updates over changes or commits you made; when it does not update, it says *NOT updated* and why.
3. It builds a private environment, `.venv`, inside the copy, and installs the pinned packages. Nothing is installed system-wide.
4. It checks that everything works by running the test suite, which takes about 4 minutes. Use `--no-tests` to skip this.

| Option | What it does |
|---|---|
| `--dir PATH` | Installs somewhere else (default: the copy it sits in, else `~/alix_studio_lite`). |
| `--shortcut` | Adds an "ALIX Studio Lite" launcher: on Linux to the desktop and the applications menu; on Windows and macOS to the desktop. |
| `--start` | Starts Lite when it is installed. |
| `--no-tests` | Skips the test suite at the end. |
| `--no-optional` | Skips the optional packages (toppra, meshoptimizer). |
| `--branch NAME` | Uses another branch. A new copy gets `lite-ui`; an existing copy stays on its own branch unless this is given. |

**To update later,** run the installer again: `python3 install_lite.py` inside the copy (Windows:
`py install_lite.py`). It prints this command, with the full path, when it finishes.

---

## Start Lite

```bash
alix_studio_lite/tools/run_lite.sh                # Windows: py alix_studio_lite\tools\run_lite.py
```

Lite opens at **http://127.0.0.1:8766/** in a Chrome window of its own (its own profile, with Chrome's GPU blocklist ignored), so the 3D shows even on a computer whose graphics run in software. Without Chrome, the default browser opens a tab.
- Log in with the operator PIN. Advanced Robot Controls ask for the engineer PIN.
- Both PINs are **`1234`** until changed: `tools/run_lite.sh --set-pin operator`, or `--set-pin engineer`.
- STOP BOTH always works, with or without a log-in, and acts like the E-stop: afterwards nothing moves until **Recover** (Robot & mode). When a window covers the top bar (the log-in screen, the Project Manager, a dialog), a second STOP BOTH stays at the bottom left.
- Projects and settings live in `~/ALIX-Lite/`, outside the code, so updating never touches them.
- A project file (`.alixproj`) from another computer brings its recordings and settings, but not its kept versions or test results: tune and keep again here before running on this robot.

Other options:

| Option | What it does |
|---|---|
| `--no-browser` | Opens no browser: open the address yourself. |
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
| **E-stop** (IO3) | Clear, pressed, unknown, or *not configured*. IO3 must be in the controller's E-stop mode (5), with the button wired normally-closed to ground (RealMan's circuit-type IO E-stop wiring): it reads 1 released, 0 pressed. RealMan's demo maps IO3 to "continue program". |

With the arms connected, Lite **commands** them through ALIX's own robot code:

- **Real robot** (after a confirmation; every move asks again):
  - **Prepare → Apply** stages the region's setup. The hand opens if the pole must move, the arm is checked at rest, then the pole and then the hand move at **50 % speed**, each read back. A hand that stops on the tool counts as done, as the control program's `hand.py` takes it (arrived or settled).
  - **Drag teaching** runs on the controller, as in the control program. STOP DRAG saves the controller's recording into the project, byte for byte as the control program saves it, with the arm and the stage it was taught with.
  - **Upload** and **Run** use the control program's own tool, `rm_upload.py` (in `vendor/control_app/`): the upload stores the kept version in its Program ID and reads the stored size back; the run stages the pole and the hand, then the controller moves the arm to the first point at 20 % and plays the program, its error codes watched. The controller is set to its REAL mode for it (read back) and put back after.
  - **Move to Recorded Start** moves the arm to the file's first line at 30 % speed. Start poses by name are not stored on the controller yet.
- **Hardware sim**: the controller runs the program in its SIM mode, so the arm stays still. The pole and the hand, which the controller cannot simulate, are taken as arrived at their commanded positions.
- **STOP BOTH stops the real arms, like the E-stop.** Each controller gets ALIX's own stop, both at once: the arm stops the fastest way (`rm_set_arm_stop`, which drops its trajectory), the pole stops, and the hand freezes where it is. ALIX's check then proves the arm is still, including a running program. One arm's STOP (SLOW STOP in Advanced) stops that arm on its path. Drag teaching ends first. If a controller does not confirm, the page says so: press the red E-stop button. After any STOP, nothing moves until **Recover**, as after the E-stop. The red E-stop buttons remain the safety stop.
- **The live E-stop guards Lite.** While either arm's E-stop is not clear, every robot action in Lite is refused on both arms. After a press, motion stays blocked until **Recover**.
- **Errors show at the bottom right** of the screen until closed: a controller's error codes (in words, from ALIX's RealMan tables), a command refused or failed with its return code, an E-stop press, STOP not confirmed, a link that stopped reading.
- **The link needs Linux** (x86-64 or ARM): the RealMan SDK ships only for Linux. Elsewhere, Connect says so.

**The first time on the robot**, keep the area clear and a hand on the red E-stop button: ALIX ran
the pole and the hand at 100 % speed, so their 50 % speeds, and the stop of a running program, are
new on the hardware.

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
| **Real robot** | The arms, the pole and the hand (the pole and the hand at 50 % speed). Every move asks for confirmation first. |

Whatever the mode, every execution stages the file's own pole height and hand pose first. If
either does not arrive, nothing else runs. A trajectory runs only on the arm it was taught on, and
only after tuning has passed every limit. [`docs/UI_CONTRACT.md`](docs/UI_CONTRACT.md),
"The gates on every execution", explains each gate.

### Logs

Every activity is logged, one folder per day in **`Resources/Logs/<date>/`** in this folder (committed, so the lab's runs reach every copy; `--logs DIR` puts them elsewhere):

| File | What it holds |
|---|---|
| `activity.jsonl` | What the operator did: every change and every refusal, with who (the session) and the outcome, and the page's own steps (view, stage, task, arm). PINs are never written. |
| `hardware.jsonl` | Every command to a controller, with its arguments, its return code, the time it took and what it means, and the lines ALIX's own functions log while they run. |
| `status.jsonl` | Twice a second while an arm is connected: each arm's joints, error codes and run mode, its pole, its hand, its E-stop input, and the jobs running. |
| `lite.log` | Activity and hardware, as plain text: `tail -f Resources/Logs/$(date +%F)/lite.log` follows Lite live. |

---

## When something goes wrong

| Message or symptom | What to do |
|---|---|
| `Lite needs Python 3.12 to 3.14` | Install one (Windows: `py install 3.14`, or Python 3.14 from python.org; Ubuntu 22.04: `sudo add-apt-repository ppa:deadsnakes/ppa`, then `sudo apt install python3.12 python3.12-venv`; macOS: `brew install python@3.14`). Then run the installer again. |
| `this Python has no venv module` | Ubuntu or Debian: `sudo apt install python3.12-venv` (use your Python version). Then run the installer again. |
| `NOT updated: could not reach GitHub (sign-in or network)` | Lite is installed as it was. Check the internet connection and the GitHub sign-in, then run the installer again. |
| `the repository could not be cloned` | Sign in first (see above), then run the installer again. |
| `Cannot use port 8766: another program …` | Something other than Lite holds the port. On Linux, `ss -ltnp 'sport = :8766'` shows which program; close it, or start Lite with `--port 8800`. (If this Lite is already running there, starting it again just opens it.) |
| `An older ALIX Studio Lite is still running` | The Lite started before an update still runs the old code. Press Ctrl+C in its terminal, or run the `kill` command it shows, then start Lite again. |
| `Lite cannot reach the LEFT arm's controller` | Check the power, the cable, the address in Robot & mode, and that the computer is on 192.168.1.x. |
| `The RealMan SDK does not load on this computer` | The robot link needs Linux. Run Lite on the Linux computer that runs the control program. |
| E-stop chip reads *not configured* | IO3 is not in the controller's E-stop mode. An engineer sets it at the pendant; Lite never writes IO modes. |
| Time-optimal optimiser missing | `toppra` has no ready-made package for this computer. Manual tuning (the default) still works. |
| A ROS workspace is sourced | Nothing to do: Lite clears `PYTHONPATH` and uses its own environment. |
| *3-D view unavailable* | The browser gives the page no WebGL. `chrome://gpu` names the reason. *WebGL: Disabled* with an *llvmpipe* GPU means the computer's graphics run in software (the GPU driver is not in use: `nvidia-smi` tells), and Chrome then turns WebGL off for every page, ALIX Studio's too. Open Lite with `tools/run_lite.sh`: its own Chrome window ignores that block. In another Chrome: `chrome://flags/#ignore-gpu-blocklist` → Enabled. |

---

## For developers

### Run the tests

```bash
cd alix_studio_lite
.venv/bin/python -m pytest -q          # Windows: .venv\Scripts\python -m pytest -q   (about 4 minutes; no robot, no network)
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
- the robot link, its STOP, and Lite commanding the arms (staging, drag teaching, upload, Real and Hardware sim runs), against ALIX's emulated arms;
- the logs and the hardware alerts;
- the fixes from the final review: sessions, the E-stop latch, kept versions used only as kept, imports, stopped tunes, the dark-theme tool.

### Where things are

| Folder | What it holds |
|---|---|
| `src/alix_studio_lite/` | Lite's code. See the rows below. |
| `src/alix_studio_lite/motion/` | Tracks, the 5 ms clock, limits and checks. |
| `src/alix_studio_lite/optimise/` | The Default and Time-optimal optimisers. |
| `src/alix_studio_lite/server/` | The local web server: projects, log-in, the robot stand-in, the live robot link (`robot_link.py`), Lite's commands to the arms through ALIX's code (`robot_real.py`), and the logs (`logbook.py`). |
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
- [`docs/UI_CONTRACT.md`](docs/UI_CONTRACT.md): each screen, what it does, and the gates on every execution.
- [`docs/ALIX_Studio_Lite_Requirements_Revised.md`](docs/ALIX_Studio_Lite_Requirements_Revised.md): the requirements (final, v1.6).
- [`docs/ALIX_Studio_Lite_Implementation_Plan.md`](docs/ALIX_Studio_Lite_Implementation_Plan.md): the implementation plan.
