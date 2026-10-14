# ASL
## Developer Handoff / Functional Specification

**Reference frontend:** `ALIX_Studio_LITE_Edition_V22_Transition_StartPoints_TuneLayout.html`  
**Document purpose:** This file explains the current HTML prototype, intended operator workflow, state model, backend integration points, and recommended next-development architecture.

> **Important:** The current HTML is a functional frontend prototype. Some features are real browser-side behavior, while robot control, trajectory recording, trajectory processing, simulation execution, controller upload/run, telemetry, and hardware safety must be connected to the Python/RealMan backend before production use.

---

## 1. Product Goal

ASL is intended to let a **non-technical operator** teach and validate a complete dual-arm toilet-cleaning workflow with clear step-by-step guidance.

The operator should not need to understand RealMan APIs, trajectory file formats, inverse kinematics, robot program upload commands, or low-level controller details.

The software should guide the operator through:

1. Physical setup and safety checks.
2. Choosing one arm and one cleaning region.
3. Drag & Teach recording.
4. Saving and reviewing the trajectory.
5. Tuning and simulation.
6. Real robot validation.
7. Repeating for all required regions.
8. Building the complete LEFT / RIGHT cleaning sequence.
9. Running the complete sequence in simulation.
10. Running the final sequence on the physical dual-arm robot.
11. Stopping at any point and resuming later from a saved project.

---

## 2. Intended User Types

### Cleaning Operator
Primary user.

The operator should see the simplified guided workflow and should normally stay inside:

- Setup & Safety
- Region Teaching
- Merge + Full Sequence
- Project Summary
- Project Manager

### Engineer / Service User
Advanced user.

The **Advanced Robot Controls** window is separated from the normal operator workflow and currently has its own numeric PIN gate.

Current template credentials:

- **Operator login PIN:** `1234`
- **Advanced Robot Controls PIN:** `1234`

These PINs are currently implemented in frontend JavaScript and therefore **must not be considered secure**. Production authentication must be moved to Python/backend storage.

---

## 3. Main Application Navigation

The current top-level workflow is:

| No. | Section | Purpose |
|---|---|---|
| 01 | Setup & Safety | Physical preparation, robot positioning, start poses, preparation checklist |
| 02 | Region Teaching | Teach and validate individual LEFT / RIGHT cleaning tasks |
| 03 | Merge + Full Sequence | Define, merge, simulate, and validate the complete dual-arm flow |
| 04 | Project Summary | Review overall completion and approved task information |

There is also a separate **Project Manager** screen after login.

---

## 4. Login and First-Time Tour

After operator login, the software opens the **Project Manager**.

For a first-time browser/user, a guided tour is available and currently covers:

- Welcome
- Setup & Safety
- Arm selection
- Drag & Teach
- Tune + Simulation
- Merge + Full Sequence
- Project Summary

The tour includes:

- Previous
- Next
- Finish
- Skip Tour
- Restart Tour

The frontend currently remembers tour completion using browser `localStorage`.

Current key:

```text
alixLiteTourCompleted_v1
```

---

# 5. Project Manager

The operator is not expected to complete all 22 tasks in a single session.

The Project Manager therefore allows the operator to stop anywhere and later continue from the same task, arm, and workflow stage.

## 5.1 Main Project Actions

The current UI provides:

- **Continue Last Project**
- **New Project**
- **Open / Manage Projects**
- **Load Project File**
- Rename
- Duplicate
- Archive
- Unarchive
- Delete
- Save Project File
- Manual Save
- Auto Save

The top bar shows:

- Current project name
- Saved / Unsaved state
- Manual Save button
- Projects button

Example resume text:

```text
Resume: TASK 5A · RIGHT ARM · Tune + Simulation
```

## 5.2 Current Browser Storage Keys

```text
alixLiteProjects_v1
alixLiteLastProjectId_v1
alixLiteTourCompleted_v1
```

Other legacy/current frontend state keys are also used by the HTML for task completion and sequence state.

## 5.3 Current Project Schema Name

```text
alix_studio_lite_project_v1
```

## 5.4 Project State Currently Captured

The HTML currently attempts to save/resume:

- Current top-level page
- Current task index
- Selected arm
- Current Region Teaching stage
- LEFT / RIGHT approval state
- Arm-selection lock state
- Completed tasks
- Per-task records
- Setup checklist
- Target teaching time
- Pole height
- Hand preset
- Six hand values
- LEFT start pose
- RIGHT start pose
- Program ID
- Trajectory filename
- Active trajectory source
- Active trajectory display name
- Trajectory row count
- Manual scale regions
- Manual trim regions
- Full sequence configuration
- Merge selections
- Engineering sequence steps
- Overall sequence speed
- Waiting-arm confirmation
- Full-sequence visual confirmation
- Full-simulation approval
- Elapsed Drag & Teach time
- Saved timestamp

### Important backend limitation

The browser project currently saves **state and file metadata**, but it should not be relied on to persist the actual trajectory file bytes.

For production, a project must own the real files on disk.

Recommended project layout:

```text
projects/
  <project_id>/
    project.json
    trajectories/
      left/
      right/
    merged/
      left/
      right/
    sequence/
      full_sequence.json
    simulation/
      reports/
      logs/
    controller_exports/
    telemetry/
```

A production `.alixproj` export should ideally be a ZIP-compatible package containing `project.json` plus all required trajectory and sequence files.

---

# 6. Setup & Safety

The Setup & Safety stage is intended to be completed before Region Teaching is unlocked.

The UI guidance includes:

- Power ON robot
- Move ALIX robot close to the toilet
- Approximate robot-to-toilet positioning guidance: **250 mm**
- Manual preparation checks
- Move LEFT and RIGHT arms to selected start poses
- Pole and hand setup
- Preparation confirmation

The Region Teaching page is intentionally blocked if required preparation is not confirmed.

---

# 7. Region Teaching Workflow

The Region Teaching pipeline contains five stages:

```text
1. Prepare
2. Drag & Teach
3. Save & Review
4. Tune + Simulation
5. Real Test
```

The **Merge + Full Sequence** workflow is intentionally separate and is not treated as a sixth per-region stage.

---

## 7.1 Stage 1 — Prepare

In Prepare:

- Operator chooses either LEFT or RIGHT arm.
- LEFT and RIGHT region cards show separate visual references.
- Each arm card has a zoom icon.
- The chosen arm becomes the active teaching arm.
- Operator can select / apply start pose.
- Operator can set pole state.
- Operator can set hand state.
- The opposite arm remains available only until Drag & Teach begins.

### Arm-lock rule

Once Drag & Teach starts:

> The selected arm remains locked through Save & Review, Tune + Simulation, and Real Test.

The operator cannot accidentally switch to the opposite arm halfway through the workflow.

After the selected arm passes Real Test, the opposite arm becomes available for the same task.

After both arms pass, the software advances to the next task automatically.

---

## 7.2 Stage 2 — Drag & Teach

Main behavior:

- Editable target time, default **30 seconds**
- English spoken timer
- START DRAG + RECORD
- STOP DRAG
- Reset timer
- Elapsed-time display
- Progress bar
- Current selected-arm region image is copied into the Drag & Teach section
- Current region name is displayed near the copied reference

The current concept is:

```text
START
→ follow highlighted region
→ finish near target time
→ STOP
```

After STOP, the intended operator flow supports:

```text
Redo Again
or
Proceed to Save & Review
```

When Redo is selected, the workflow should let the operator move back to a selected start pose before recording again.

---

## 7.3 Stage 3 — Save & Review

The current design supports:

- Trajectory filename
- Save TXT
- Move to recorded start
- Playback
- Record Again
- Browse an older / previous trajectory from the computer

The design intentionally distinguishes:

- **New Recording**
- **Loaded Previous File**

Loading an old trajectory should not silently overwrite the current recording.

### Backend requirement

The HTML cannot create a valid RealMan controller trajectory by itself.

The backend must provide:

- Receive drag-record samples
- Save exact controller-compatible TXT format
- Validate format
- Return duration / row count / metadata
- Load previous TXT
- Move to recorded start
- Playback trajectory

---

## 7.4 Stage 4 — Tune + Simulation

The current tuning concept uses explicit processing stages rather than hiding everything in advanced dropdowns.

Planned / represented processing controls:

- Trim idle-region removal
- Mean Smooth
- Scale selected-region resampling

Manual region editor includes:

- Start row
- End row
- Scale %
- Meaning
- Delete
- Clear regions

Example:

```text
Rows 1–500     = 100%   unchanged
Rows 501–1000  = 50%    50% shorter
```

### Plot interaction

The intended plot behavior is:

- Operator click-drags over a trajectory range.
- Selected range becomes highlighted.
- Selected Start Row / End Row are populated.
- Operator can tune only the selected range.
- Apply scale / smoothing / trim to the selected range.
- Re-run simulation after tuning.

### Simulation behavior

The simulation area currently includes:

- Start
- Stop
- Reset
- Expanded 90% view
- Timer
- Plot playhead
- Current simulation image/reference

When simulation starts:

> The timer and the vertical plot playhead should advance together from zero to the end.

Re-run Simulation should scroll the operator back to the simulation view.

### Backend requirement

Production backend should:

1. Parse trajectory samples.
2. Perform real trim/smoothing/resampling.
3. Return processed data.
4. Return original and optimized duration.
5. Return joint-speed/acceleration statistics.
6. Run the processed trajectory in PyBullet or the chosen simulator.
7. Stream simulation state/progress back to the HTML.

---

## 7.5 Stage 5 — Real Test

The current Real Test workflow includes:

- Controller Program ID
- Program ID range: **1–100**
- Upload trajectory
- Move to trajectory start
- Run Real Test
- PASS
- Needs Tuning
- Record Again

Behavior:

- `Needs Tuning` returns to Tune + Simulation.
- `Record Again` returns to Drag & Teach.
- PASS approves the current arm for the current task.
- After one arm passes, Prepare opens for the opposite arm.
- After both arms pass, the task becomes complete and the next task is opened automatically.

For the last task, the intended workflow proceeds to the standalone Merge + Full Sequence stage.

---

# 8. Cleaning Task List

The current HTML defines **22 tasks**.

| # | Source Task | LEFT Arm | RIGHT Arm | Type | Note |
|---:|---|---|---|---|---|
| 1 | TASK 1A | L-R01 BASE LEFT | R-R01 LID TOP PART 01 | clean | — |
| 2 | TASK 2A | L-R02 SIDE EXTERIOR LEFT | R-R01 LID TOP PART 02 | clean | — |
| 3 | TASK 1C | L-R03 PEDESTAL UPPER LEFT | R-R02 RIGHT HINGE | clean | — |
| 4 | TASK 2 | LEFT HINGE | SIDE EXTERIOR RIGHT | clean | — |
| 5 | TASK 3 | OPEN LID + SEAT (MASTER) | OPEN LID + SEAT (SUPPORT) | action | — |
| 6 | TASK 4 | CLOSE SEAT ONLY (MASTER) | CLOSE SEAT ONLY SUPPORT | action | — |
| 7 | TASK 5A | SEAT TOP PART 1 | LID UNDERSIDE RIGHT HALF | clean | — |
| 8 | TASK 5B | SEAT TOP PART 2 | PEDESTAL UPPER RIGHT | clean | — |
| 9 | TASK 6 | LID UNDERSIDE LEFT HALF | BASE RIGHT | clean | — |
| 10 | TASK 7 | L-T03 OPEN SEAT (MASTER) | R-T03 OPEN SEAT (SUPPORT) | action | — |
| 11 | TASK 8A | RIM TOP PART 1 | SEAT UNDERSIDE RIGHT HALF | clean | — |
| 12 | TASK 8B | RIM TOP PART 2 | FLOOR SPILLAGE PART 1 | clean | — |
| 13 | TASK 9 | FLOOR SPILLAGE PART 2 | SEAT UNDERSIDE LEFT HALF | clean | — |
| 14 | TASK 10A | FL-R09 INNER RIM PART 1 | R-T04 PICK DRAIN BRUSH | mixed | — |
| 15 | TASK 10B | FL-R09 INNER RIM PART 2 | R-T04 PICK DRAIN BRUSH | mixed | — |
| 16 | TASK 11A | L-R10 BOWL INSIDE PART 1 | HOLD BRUSH OUTSIDE | mixed | — |
| 17 | TASK 11B | L-R10 BOWL INSIDE PART 2 | HOLD BRUSH OUTSIDE | mixed | — |
| 18 | TASK 12A | L-R11 FLOOR SPILLAGE LEFT PART 1 | R-T05 CLEAN DRAIN HOLE PART1 | clean | — |
| 19 | TASK 12B | L-R11 FLOOR SPILLAGE LEFT PART 2 | R-T05 CLEAN DRAIN HOLE PART 2 | clean | — |
| 20 | TASK 13 | DROP GLOVE | R-T06 RETURN DRAIN BRUSH AND DROP GLOVE | procedure | Procedure to be added later |
| 21 | TASK 14 | L-T04 LEFT GLOVE CHANGE | R-T07 RIGHT GLOVE CHANGE | procedure | Procedure to be added later |
| 22 | TASK 15 | L-T05 CLOSE SEAT + LID (MASTER) | R-T09 CLOSE SEAT + LID (SUPPORT) | action | — |

---

# 9. Visual Reference System

The UI includes visual references for operator guidance.

Current behavior includes:

- LEFT / RIGHT task-specific reference cards
- Arm-specific image zoom
- Drag & Teach copied current-region reference
- Visual toilet reference slideshow
- Automatic visual-reference rotation every **5 seconds**
- State labels presented as UI text rather than being permanently baked into the latest reference images

The developer should preserve:

```text
object-fit: contain
```

for operator reference images so the entire region remains visible.

Zoom dialogs must fit inside the browser viewport and keep the Close button visible.

---

# 10. Merge + Full Sequence

This is a standalone project-level workflow.

Its purpose is to combine all approved regional trajectories into a complete dual-arm cleaning cycle.

Main flow:

```text
Define Sequence
→ Visual Confirm
→ Full Simulation
→ Operator Confirm
→ Real Machine
```

## 10.1 Editable LEFT / RIGHT Flow

The full flow table is intended to be editable.

Fields include:

- Source Task name
- LEFT Program ID
- LEFT Arm Flow
- RIGHT Program ID
- RIGHT Arm Flow
- Execution Rule
- Checkpoint / Wait
- Include / Exclude

Execution rules represented by the frontend include:

```text
TOGETHER
LEFT RUN FIRST
RIGHT RUN FIRST
ONE BY ONE
PROCEDURE
```

## 10.2 Merge Trajectory

LEFT and RIGHT merges are independent.

Intended per-arm process:

```text
Choose files
→ reorder files
→ Merge
→ test merged LEFT or RIGHT arm alone in simulation
→ optional real-arm test
```

Merge tools should support:

- Multiple custom trajectory files
- Explicit order
- Move ↑ / ↓
- Merge LEFT
- Merge RIGHT
- Merge BOTH where required
- Merge-only
- Merge + Simulation
- Merge + Real Test

### Backend requirement

Actual merge must happen in Python.

The backend must handle:

- TXT parsing
- Continuity between file boundaries
- Start/end pose comparison
- Optional bridge generation
- Timing
- Validation
- Final merged trajectory creation

---

# 11. Full Sequence Simulation

The full-sequence simulation opens in a large simulation view.

Controls represented in the current UI:

- Start
- Pause
- Stop
- Reset
- Overall Speed %
- Exit

Live display includes:

- Current LEFT status
- Current LEFT task
- LEFT Program ID
- Current RIGHT status
- Current RIGHT task
- RIGHT Program ID
- Current sequence step
- Elapsed simulation time

The Real full-sequence run remains locked until the full simulation is completed and confirmed.

---

# 12. Dual-Arm Safety Interlock

A key requirement is:

> A serious fault on either arm must immediately stop both arms.

The frontend includes a conceptual **Dual Safety Interlock**.

Fault examples:

- Collision detected
- Controller fault
- Power failure
- Communication loss
- Emergency stop
- Sequence execution failure

Expected behavior:

```text
LEFT fault
→ latch fault
→ STOP LEFT
→ STOP RIGHT immediately
→ cancel full-sequence approval
→ prevent restart
→ require reset / acknowledgement
```

and the same applies when RIGHT is the source of the fault.

### Critical production note

This must **not rely on browser JavaScript**.

The Python/backend/controller layer must own the real interlock.

Recommended implementation:

```text
robot status monitor thread / callback
        ↓
shared dual-arm safety state
        ↓
any critical fault
        ↓
send stop command to BOTH controllers
        ↓
latch FAULT state
        ↓
WebSocket event to frontend
```

Hardware/controller safety and physical E-stop remain the final safety layer.

---

# 13. Project Summary

The Project Summary shows:

- Current task
- Number of approved tasks
- Target-time information
- Complete task sequence
- Task status
- LEFT / RIGHT task names

For approved tasks the intended stored setup includes:

### LEFT
- Pole height
- Hand preset/state
- Six hand-channel values
- Start pose
- Program ID

### RIGHT
- Pole height
- Hand preset/state
- Six hand-channel values
- Start pose
- Program ID

The six hand values are:

```text
Little
Ring
Middle
Index
Thumb Bend
Thumb Rotate
```

---

# 14. Advanced Robot Controls

The engineering window is intentionally separated from the normal guided workflow.

Current controller addresses represented in the HTML:

```text
LEFT RM75:  192.168.1.10
RIGHT RM75: 192.168.1.103
```

The current frontend shows independent LEFT and RIGHT tabs for:

- RUN
- GLOBAL POINTS
- DRAG TEACH
- HAND / POLE
- TELEMETRY

## 14.1 RUN

Per-arm concepts include:

- SIM
- REAL
- CHECK
- SLOW STOP
- Trajectory file
- Program ID / Upload Slot
- Upload Only
- Upload + Run
- Run Only
- Multi Upload
- Run Stored
- Multi Run
- Load Sequence
- Refresh controller program list

## 14.2 Global Points

Intended actions:

- Refresh saved points
- Search/select point
- MoveJ
- MoveL
- Speed %
- Wait after arrival
- Add to point sequence
- Run point sequence

## 14.3 Drag Teach

Intended actions:

- Start Drag + Record
- Stop Drag
- Save Local
- Move to Start + Playback

The guided single-arm lock must continue to take priority over engineering commands while an operator region-teaching session is active.

## 14.4 Hand / Pole

Hand:

```text
6 channels
range: 0–1000
```

Pole:

```text
range: 0–315 mm
```

The UI includes numeric entry + slider behavior.

## 14.5 Telemetry

There are independent LEFT and RIGHT telemetry areas.

Production telemetry should ideally stream through WebSocket/SSE rather than periodic page polling.

---

# 15. Existing Python / RealMan Integration Direction

The HTML contains references to the existing Python/RealMan ecosystem including:

```text
rm_upload.py
```

and an SDK path reference under:

```text
/home/alai-labs/alix_ws/src/alix/external/butterfli-realman-alix/Python
```

The recommended approach is to **reuse the proven Python robot-control layer** rather than reimplement low-level RealMan control in JavaScript.

---

# 16. Recommended Desktop Architecture

The current HTML should remain the operator frontend.

A practical architecture is:

```text
HTML / CSS / JavaScript UI
        │
        │ local API + WebSocket
        ▼
Python ALIX backend
        │
        ├── Project Manager service
        ├── Trajectory recorder
        ├── Trajectory parser / processor
        ├── Merge / bridge generator
        ├── PyBullet simulation manager
        ├── Dual sequence scheduler
        ├── Safety interlock manager
        ├── Robot telemetry manager
        └── RealMan SDK / existing rm_upload logic
                  │
                  ├── LEFT  192.168.1.10
                  └── RIGHT 192.168.1.103
```

For a Windows/Linux desktop application, a local Python-hosted web UI is suitable.

Possible packaging approaches:

- **PyWebView + Python backend** — recommended for a lightweight desktop application.
- Local FastAPI/Flask server + browser window.
- Electron only if the team specifically wants a heavier JS desktop shell.

The backend should be the authority for:

- robot state
- project files
- authentication
- trajectory files
- simulation state
- safety state
- upload/run commands

---

# 17. Recommended Backend API Boundary

This is a suggested contract for the next development phase.

## Project API

```text
GET    /api/projects
POST   /api/projects
GET    /api/projects/{id}
PUT    /api/projects/{id}
DELETE /api/projects/{id}

POST   /api/projects/{id}/export
POST   /api/projects/import
```

## Robot API

```text
GET  /api/robots/status
POST /api/robots/{left|right}/mode
POST /api/robots/{left|right}/stop
POST /api/robots/stop-both

POST /api/robots/{arm}/move-start
POST /api/robots/{arm}/movej
POST /api/robots/{arm}/movel
```

## Drag Teach / Trajectory API

```text
POST /api/drag/{arm}/start
POST /api/drag/{arm}/stop
POST /api/drag/{arm}/save

POST /api/trajectory/load
POST /api/trajectory/process
POST /api/trajectory/playback

POST /api/trajectory/{arm}/upload
POST /api/trajectory/{arm}/run
```

## Merge API

```text
POST /api/merge/{arm}
POST /api/merge/{arm}/simulate
POST /api/merge/{arm}/real-test
```

## Sequence API

```text
POST /api/sequence/validate
POST /api/sequence/simulate
POST /api/sequence/run
POST /api/sequence/stop
```

## Streaming

```text
WS /ws/telemetry
WS /ws/simulation
WS /ws/sequence
WS /ws/safety
```

These endpoint names are recommendations, not existing APIs.

---

# 18. Frontend-to-Backend Events

A useful backend event model would include:

```json
{
  "type": "robot_status",
  "arm": "left",
  "state": "RUNNING",
  "program_id": 71
}
```

```json
{
  "type": "trajectory_progress",
  "arm": "left",
  "elapsed_s": 3.2,
  "duration_s": 8.0,
  "row": 4120
}
```

```json
{
  "type": "safety_fault",
  "source_arm": "right",
  "reason": "communication_loss",
  "dual_stop": true,
  "latched": true
}
```

```json
{
  "type": "sequence_step",
  "step": 8,
  "left_task": "RIM TOP PART 1",
  "right_task": "SEAT UNDERSIDE RIGHT HALF",
  "rule": "TOGETHER"
}
```

---

# 19. Security Notes

The current HTML prototype contains frontend PIN values.

Do not use that model in production.

Production should:

- validate users in Python/backend
- store hashed credentials
- define Operator vs Engineer permissions
- prevent operator access to engineering endpoints
- require real-hardware confirmation server-side
- log real robot commands
- log stop/fault events
- not trust client-side button-disabled states for safety

---

# 20. Browser Storage vs Production Storage

Current HTML uses `localStorage` so the prototype can demonstrate:

- project creation
- autosave
- resume
- sequence persistence
- tour completion
- theme/preferences

This is suitable for UI prototyping only.

Production should use filesystem/database persistence managed by Python.

Recommended minimum project record:

```json
{
  "id": "project_uuid",
  "name": "Toilet 01 Training",
  "created_at": "ISO-8601",
  "updated_at": "ISO-8601",
  "current_task": 6,
  "current_arm": "right",
  "current_stage": "optimize",
  "task_records": {},
  "sequence": {},
  "files": {},
  "safety_log": []
}
```

---

# 21. Important UI Rules to Preserve

The following behavior is intentional and should not be removed accidentally during backend work:

1. **One arm at a time** during Region Teaching.
2. Arm can only be chosen in Prepare.
3. Once Drag & Teach starts, that arm stays locked until Real Test completes.
4. Both task images may be visible, but only the active arm is emphasized.
5. Current region image is repeated inside Drag & Teach.
6. Real Test requires a valid Program ID before upload.
7. After one arm passes, the opposite arm becomes available.
8. After both arms pass, move automatically to the next task.
9. Merge + Full Sequence remains a separate project-level section.
10. Full real sequence remains locked until simulation is approved.
11. Any critical single-arm fault must result in a dual-arm stop.
12. Operator can stop the project at any time and resume later.
13. Auto Save should never block robot STOP.
14. STOP controls must remain visually prominent.
15. Reference images should use `contain`, not cropped fill.

---

# 22. Current Template-Only / Placeholder Areas

The developer should assume the following still require real implementation:

- Real controller connection
- Controller authentication / mode switching
- Drag recording from RM75
- Real TXT trajectory generation
- Real playback
- Program upload
- Program run
- Program-list refresh
- Live program completion detection
- Global Point fetch/run
- Hand control
- Pole control
- Real telemetry
- PyBullet simulation
- Plot from real trajectory data
- Real smoothing/resampling
- Merge and bridge generation
- Runtime calculation
- Collision detection integration
- Communication-failure detection
- Dual-arm hardware interlock
- Real sequence scheduler
- Real project file management
- Secure operator / engineering authentication

---

# 23. Suggested Development Order

A safe implementation order is:

### Phase 1 — Application shell and project persistence
- Convert browser project storage to Python project folders.
- Implement project CRUD.
- Implement `.alixproj` import/export.
- Move login/PIN validation to backend.

### Phase 2 — Read-only robot integration
- Connect both controllers.
- Read mode/state.
- Stream telemetry.
- Read program lists.
- Read Global Points.

### Phase 3 — Single-arm commands
- Move to start pose.
- Hand control.
- Pole control.
- Global Point MoveJ/MoveL.
- Stop.

### Phase 4 — Drag & Teach
- Start drag.
- Record.
- Stop.
- Save TXT.
- Playback.

### Phase 5 — Trajectory processing
- Parse trajectory.
- Plot.
- Manual row selection.
- Trim.
- Smooth.
- Scale/resample.
- Save optimized trajectory.

### Phase 6 — Simulation
- Integrate dual RM75 + hand + pole + toilet scene.
- Run one-arm simulation.
- Run full-sequence simulation.
- Stream progress.

### Phase 7 — Real Test
- Upload Program ID 1–100.
- Move to start.
- Run.
- Detect completion/fault.
- PASS/Tune/Redo flow.

### Phase 8 — Merge + full sequence
- Merge/bridge trajectories.
- Validate continuity.
- Full dual-arm sequence scheduler.
- Checkpoints and execution rules.

### Phase 9 — Safety hardening
- Dual-stop interlock.
- Fault monitoring.
- Communication timeout.
- Collision event handling.
- Recovery/reset state machine.
- Logging.

---

# 24. Acceptance Checklist for Developer

Before calling the application production-ready, verify:

- [ ] Project can be stopped and resumed after application restart.
- [ ] Real trajectory files are stored inside the project.
- [ ] Project import/export transfers all required files.
- [ ] LEFT and RIGHT robot telemetry is live.
- [ ] Stop Both works independently of UI page/state.
- [ ] A LEFT critical fault stops RIGHT.
- [ ] A RIGHT critical fault stops LEFT.
- [ ] Arm-selection lock cannot be bypassed by page navigation.
- [ ] Drag Teach saves a controller-compatible trajectory.
- [ ] Previous trajectory can be loaded without overwriting new recording.
- [ ] Plot uses real trajectory samples.
- [ ] Selected plot region maps exactly to sample/row numbers.
- [ ] Optimized trajectory can be simulated before hardware test.
- [ ] Program ID validation is enforced backend-side.
- [ ] Real Test reports actual completion/failure.
- [ ] Both-arm PASS advances to next task correctly.
- [ ] Full-sequence simulation is required before full real run.
- [ ] Project Summary shows approved hand/pole/start-pose/program information.
- [ ] Operator and Engineer permissions are enforced server-side.
- [ ] Project autosave never delays emergency stop.

---

# 25. Files to Share With Developer

Primary files:

```text
ALIX_Studio_LITE_Edition_V22_Transition_StartPoints_TuneLayout.html
ALIX_Studio_LITE_Edition_V22_Developer_Handoff.md
```

The HTML contains embedded frontend assets and represents the current UI/UX prototype.

The Markdown file should be treated as the functional/technical handoff document explaining how the frontend is expected to behave when connected to the real backend.

---

## Final Development Principle

The intended product separation is:

> **HTML = guided operator experience**  
> **Python/backend = authority for robots, files, projects, simulation, processing, authentication and safety**

Keep the normal operator path simple. Engineering capabilities can remain available behind Advanced Robot Controls, but they should not complicate the main teaching workflow.


---

# V20 Additions — Visual Region Summary and Project Folder

## Project Summary Images

Project Summary now displays separate LEFT and RIGHT region-reference images for every task.

The image resolver follows this rule:

- When the task source slide contains two LEFT / RIGHT visuals, the frontend resolves separate LEFT and RIGHT images.
- When the current supplied task slide contains one central task image, the same complete image is shown for both arms and is not split/cropped.

For the current embedded source presentation, the task slides treated as single-image sources are:

```text
Slide 9
Slide 10
Slide 14
Slide 25
```

This list is a frontend metadata rule and should ideally come from the future source-content importer rather than remain hard-coded.

## Per-Arm Task Status

Project Summary now exposes arm-level progress independently:

```text
LEFT Status
RIGHT Status
Overall Status
```

Expected arm values are:

```text
PENDING
CURRENT
APPROVED
```

Overall task state remains independent and may be:

```text
PENDING
CURRENT
COMPLETE
PROCEDURE
```

## Preview Region Workflow

The Home page Preview Region Workflow control now opens a full visual gallery of every task.

Each task card contains:

- Task number / task label
- Overall task status
- LEFT image
- LEFT region name
- LEFT arm status
- RIGHT image
- RIGHT region name
- RIGHT arm status

Task images are clickable for larger viewing.

## Project Folder Export

V20 adds a browser prototype of **Save Project Folder** using the File System Access API available in current Chromium-based browsers such as Chrome and Edge.

Recommended generated structure:

```text
<Project Name>/
  project.json
  project_manifest.json
  README.txt

  trajectories/
    left/
      trajectory_manifest.json
      <LEFT trajectory TXT files>
    right/
      trajectory_manifest.json
      <RIGHT trajectory TXT files>

  merged/
    left/
      merge_manifest.json
    right/
      merge_manifest.json

  sequence/
    full_sequence.json

  tuning/
    manual_tuning.json

  simulation/
    reports/
      simulation_state.json

  controller_exports/

  telemetry/

  logs/
    save_log.json

  media/
    region_references/
```

### Critical Browser Limitation

The standalone HTML currently owns project metadata but **does not reliably own every historical trajectory file byte** after the browser has downloaded or released a file.

Therefore V20 can create the folder tree and LEFT / RIGHT trajectory manifests, but the production Python backend must save the real trajectory `.txt` files into the project directory at the moment they are:

- recorded
- loaded
- optimized
- approved
- merged
- uploaded to the controller

The intended production behavior is:

```text
Start / open project
        ↓
Python creates/opens project directory
        ↓
LEFT recordings → trajectories/left/
RIGHT recordings → trajectories/right/
        ↓
optimized versions remain associated with source task/arm
        ↓
approved version marked in project.json + trajectory manifest
        ↓
merge output → merged/left or merged/right
        ↓
full sequence → sequence/full_sequence.json
```

`Save .alixproj` remains useful as a portable project package/export, while the project folder should be the normal working format during development and production.



---

# V22 Additions — Transition Trajectories, Task Start Points, and Tune Layout

## Region image zoom behavior

The large region-reference modal must not upscale an image beyond its intrinsic pixel dimensions.

Frontend rule:

```css
width: auto;
height: auto;
max-width: 100%;
max-height: 100%;
object-fit: contain;
```

The modal can scale a large image down to fit the viewport, but must not enlarge a smaller source image.

## Drag & Teach control placement

`START DRAG + RECORD`, `STOP DRAG`, and `Reset` are intentionally positioned immediately below the timer so the operator does not need to scan the page while hand-guiding the arm.

## Tune + Simulation layout

The simulation pane is a flex column and the simulation viewport expands vertically to match the overall Plot Analysis pane height.

Manual Scaling Regions are displayed **below** the side-by-side Simulation / Plot Analysis workspace.

The intended visual sequence is now:

```text
Processing Stages
        ↓
Simulation            Plot Analysis
        ↓
Manual Scaling Regions
        ↓
Re-run / Real Test
```

## Optional Task Transition Trajectory Runner

Prepare now includes an optional **Task Transition & Start Point Tools** section.

Each arm is independent.

LEFT:

```text
Browse LEFT transition .txt
Run LEFT in Simulation
Run LEFT on Real Arm
Use trajectory final pose as a task start point
```

RIGHT:

```text
Browse RIGHT transition .txt
Run RIGHT in Simulation
Run RIGHT on Real Arm
Use trajectory final pose as a task start point
```

The opposite arm must remain stopped for an individual real-arm transition run.

### Why this exists

A later cleaning task may need to begin at the previous task's stop location.

Rather than forcing every task to return to a common home pose, a transition trajectory or the previous task endpoint can become the next task start reference.

This supports smooth workflow continuity and reduces unnecessary robot travel.

## Per-Task Start Points

Every task can maintain independent named start-point records for LEFT and RIGHT.

The frontend provides:

```text
Capture Current Pose
Previous Task End
Trajectory End
Move to Start Point
Delete Start Point
```

A start-point metadata record has the conceptual structure:

```json
{
  "id": "sp_unique_id",
  "name": "LEFT · TASK 1A End",
  "taskIndex": 1,
  "taskLabel": "TASK 2A",
  "arm": "left",
  "source": "previous-task-end",
  "pose": null,
  "joints": null,
  "createdAt": "ISO-8601"
}
```

Valid `source` values currently include:

```text
captured-current-pose
previous-task-end
trajectory-end
```

In the standalone HTML, `pose` and `joints` remain null because the browser has no robot-state connection.

### Backend implementation requirement

When the operator clicks **Capture Current Pose**, Python must read and save:

```text
J1 ... J7
X, Y, Z, RX, RY, RZ
pole height
hand state
timestamp
robot/controller identity
```

When **Previous Task End** is used, the backend should resolve the approved previous trajectory's actual final sample / final robot pose.

When **Trajectory End** is used, the backend should parse the selected transition trajectory and record the final pose/joints.

## Transition trajectory file persistence

The HTML can remember the filename and metadata during project Auto Save, but normal browser security means the file handle may not be reusable after application reload.

Production Python behavior should instead copy/import the selected file into the current project, for example:

```text
<Project>/
  trajectories/
    transitions/
      left/
      right/
```

and update `project.json` / the task record to reference that stored project-relative file.

## Project Folder Manifest

LEFT and RIGHT trajectory manifests should now also include:

```text
start_points
selected_start_point_id
transition_trajectory
```

This allows the project to reconstruct not only approved cleaning trajectories but also how each task is reached from the previous task.


---

# V25 FINAL Template Changes

## Final operator behavior

- Journey/sidebar is hidden by default.
- Top-bar Tour button is removed.
- First project creation lands on **Home**.
- Home headline is simplified to **“Teach every toilet region with simple guided steps.”**
- Visible product branding is **ASL**.
- The top emergency **STOP BOTH** control is intentionally shorter while retaining high visual contrast.

## Real Test → Record Again

`Record Again` no longer jumps directly to Drag & Teach.

New behavior:

```text
Real Test
→ Record Again
→ Prepare
→ adjust pole / hand / UGV / start pose
→ Drag & Teach
```

The same arm remains locked during this re-preparation so the operator does not accidentally switch sides.

## UGV Actual Position Inputs

Prepare now contains task- and arm-specific operator inputs:

```text
Front (mm)
Rear (mm)
Left (mm)
Right (mm)
```

These values are stored with the selected arm setup alongside:

```text
pole height
hand preset
six hand channels
start pose
```

The task record should contain:

```json
{
  "prepareSetupByArm": {
    "left": {
      "poleHeightMm": 150,
      "handPreset": "Grip / Cleaning",
      "handChannels": {},
      "startPose": "Start Pose A",
      "ugvPositionMm": {
        "frontMm": 250,
        "rearMm": 0,
        "leftMm": 100,
        "rightMm": 100
      }
    },
    "right": {}
  }
}
```

When an arm passes Real Test, the approved snapshot also retains the UGV values.

The Project Summary approved setup displays the UGV position values.

The project trajectory manifest should include:

```text
ugv_position_mm
```

## Backend recommendation for UGV values

The current values are operator-entered measurements.

Future backend work may additionally save:

- UGV odometry pose
- UGV map coordinate
- toilet-relative transform
- timestamp
- measurement source (`manual`, `sensor`, `localization`)
- robot base-to-toilet distance

Do not overwrite the operator-entered values without preserving the original measurement record.

---

## V26 Branding Update

The temporary ASL letter monogram was replaced with the blue Butterfli butterfly brand mark shown by the user from `https://butterfli.ai`.

The brand mark is embedded directly into the HTML as a PNG data URI so the final frontend remains a single self-contained HTML file.

Branding behavior:

- Product/application name remains **ASL**.
- Top-left application mark uses the blue Butterfli butterfly.
- Operator Login uses the same blue Butterfli butterfly.
- Browser favicon also uses the same mark.
- The logo asset is presentation-only; authentication and product naming remain unchanged.


---

## V27 Mandatory Workflow Indicators

V27 adds a red `*` marker to operator-facing items that are required to continue the guided workflow.

A global legend is shown in the main workspace:

```text
* Mandatory to continue
```

Mandatory indicators are intentionally limited to required operator flow items. Optional engineering or convenience tools remain unmarked.

Examples marked mandatory include:

```text
Operator PIN
Project name
01 Setup & Safety
02 Region Teaching
03 Merge + Full Sequence
Physical-preparation checklist
Arm selection / Region Teaching workflow
Prepare
Pole height
Hand preset
UGV Front / Rear / Left / Right inputs
Region start pose
Drag & Teach
Target time
START / STOP recording
Save & Review
Trajectory filename / Save TXT
Tune + Simulation
Real-Arm Test
Controller Program ID
Upload / Move to Start / Test Run
PASS approval
Complete LEFT / RIGHT Flow
Visual sequence confirmation
Full sequence simulation confirmation
```

The following remain intentionally optional and therefore are not marked with `*`:

```text
Visual-reference viewing
Voice timer
Previous trajectory browsing
Advanced hand channels
Task Transition & Start Point Tools
Manual scaling/tuning utilities
Custom trajectory merge / single-arm engineering tests
Advanced Sequence Step Editor
Project export/archive/duplicate operations
Advanced Robot Controls
Project Summary viewing
```

The frontend asterisk is a usability indicator only. Production backend validation must still enforce all required state before robot commands or workflow progression are accepted.


---

## V28 Full-Sequence Simplification

The legacy direct dual-sequence quick-run panel was removed from the operator-facing `03 · Merge + Full Sequence` page.

Removed controls:

```text
Run Both · Safe Timeline
Stop Both Sequences
Confirm before waiting arm
Overall speed %
Programs refresh
Global Points refresh
LEFT LIVE
RIGHT LIVE
DUAL RUNTIME
```

Reason:

The final guided operator workflow already provides the safer and clearer project-level sequence:

```text
Define Sequence
→ Visual Confirm
→ Full Simulation
→ Operator Confirm
→ Real Machine
```

The operator should not have a second direct-run entry point before completing simulation validation.

Engineering-level program/global-point operations remain available under Advanced Robot Controls, where they belong.

The production backend may still implement a dual sequence scheduler internally, but the normal operator UI should expose only the validated full-sequence workflow.


---

## V29 Tuning File Identity and Tuned Save-As

The Tune + Simulation workspace now explicitly shows the active input trajectory being tuned.

Displayed information:

```text
CURRENTLY TUNING
<active trajectory filename>
<New recording / Loaded previous file / Saved tuned trajectory>
<row count>
```

The intended tuning workflow is now:

```text
Choose / record source trajectory
        ↓
Tune + Simulation
        ↓
Apply selected processing
        ↓
Apply Tuning
        ↓
Save Tuned File As...
        ↓
new tuned trajectory becomes the active trajectory
```

The original source filename is retained separately from the tuned output filename.

The frontend currently records the intended tuned output filename and tuning metadata. It does not generate the real processed robot trajectory samples. The Python trajectory processor must create the actual new TXT file without overwriting the source trajectory.

Recommended backend behavior:

```text
input:  L_TASK_1A.txt
output: L_TASK_1A_TUNED.txt
```

Project/task metadata should retain:

```text
tuningSourceFileName
tunedOutputFileName
processing stages
manual scale regions
trim regions
smooth window
saved timestamp
```

## V29 Full Sequence Column Order

The editable full sequence table now follows a symmetric LEFT / RIGHT layout:

```text
Step
Source Task

LEFT Arm Flow
LEFT Program ID
LEFT Checkpoint / Wait (s)

RIGHT Arm Flow
RIGHT Program ID
RIGHT Checkpoint / Wait (s)

Execution Rule
Include
```

LEFT and RIGHT Checkpoint / Wait values are independent.

For backward compatibility, when an older saved sequence contains the previous single `checkpointWait` value, V29 imports that value into both:

```text
leftCheckpointWait
rightCheckpointWait
```

The production sequence scheduler should apply the LEFT and RIGHT checkpoint/wait values independently according to the execution rule.


---

## V30 — 05 Activity Logs / Audit Trail

V30 adds a fifth sidebar page:

```text
05 · Activity Logs
```

The purpose is to provide a chronological audit trail covering the operator session from login to session lock/logout and all important workflow actions in between.

### Log fields

Each audit event contains:

```text
timestamp
user
sessionId
projectId
projectName
page
stage
task
arm
category
event
details
result
source
```

### Frontend categories

```text
AUTH
PROJECT
NAVIGATION
SETUP
TEACH
TRAJECTORY
TUNING
SIMULATION
ROBOT
SEQUENCE
SAFETY
SYSTEM
UI
```

### Result states

```text
INFO
SUCCESS
WARNING
ERROR
REQUESTED
STOPPED
```

### Activity Log screen functions

The UI provides:

```text
From date & time
To date & time
Category filter
Result filter
Free-text search
Today shortcut
Last 24 Hours shortcut
Clear Filters
Refresh
Export CSV
Export JSON
```

The log screen is intentionally append-only from the normal operator interface. No regular Clear/Delete Logs control is provided.

### Sensitive values

Operator/engineering PIN and password input values must never be stored in the audit log.

The frontend redacts fields whose type/id indicates:

```text
password
PIN
```

Authentication events record only:

```text
login successful
incorrect PIN
session lock/logout
```

not the credential value.

### Frontend automatic capture

The standalone frontend records:

```text
operator login attempts
login success
session lock/logout
screen navigation
button actions
setup/select/input changes on change/commit
project create/open/save actions
drag teach start/stop
tuning application
tuned trajectory Save As requests
single-arm transition requests
full simulation requests
full real sequence requests
dual safety fault events
browser close/refresh
```

This is useful for prototype traceability but is **not sufficient for production hardware auditing**.

### Production requirement — backend is the audit authority

The Python backend must append all actual robot/backend events to an append-only log/database, including:

```text
command issued
command accepted/rejected
robot arm
controller IP / controller identity
Program ID
trajectory path/name
MoveJ / MoveL target
start pose
joint position
Cartesian position
pole command / result
hand command / result
drag-record state
upload result
run start
run completion
run interruption
stop command
fault code
communication loss
collision/interlock event
telemetry state changes
simulation start/stop/result
file creation/update
project folder file operations
operator approval
sequence step transition
checkpoint/wait events
```

For actual robot movement, the audit record should ideally store both:

```text
requested motion/command
actual controller result/state
```

### Backend integration hook

The frontend exposes the conceptual function:

```javascript
auditBackendEvent(payload)
```

The future WebSocket/API layer can call this for backend-originated audit events.

Example:

```json
{
  "category": "ROBOT",
  "event": "MoveJ completed",
  "arm": "left",
  "details": "Start pose TASK 3",
  "result": "SUCCESS"
}
```

### Storage

The browser prototype stores audit entries in:

```text
localStorage key: aslAuditLog_v1
```

The prototype keeps a bounded local history to avoid browser quota failure.

Production should use an append-only backend database or rotating JSONL/log files with backups.

### Project Folder export

`Save Project Folder` now also writes:

```text
logs/
  save_log.json
  audit_log.json
  audit_log.csv
```

The exported audit files contain events associated with that project.

### Important definition of “100% logging”

For the production application, “100%” must mean all **software state changes, backend commands, robot command/results, faults, file operations, and operator approvals** are written by the backend.

It should not mean raw mouse-pointer movement or every timer repaint, which would generate high-volume noise without improving traceability.


---

## V31 — Live Activity Logging Fix

V30 audit logging was event-driven. If the operator remained active without triggering one of the specific captured controls, no new timestamp was written. The Activity Logs page also did not independently prove that the session was still active.

V31 adds live audit behavior.

### Session heartbeat

While an operator is logged in and the ASL window is visible:

```text
every 30 seconds
→ write "Session active"
→ include seconds since last user interaction
→ include current page/stage
```

This means a long-running operator session continues to produce fresh audit timestamps even when the operator is observing simulation, waiting for a process, or not changing a field.

### Live log-screen refresh

When `05 · Activity Logs` is open:

```text
refresh table every 1 second
```

The screen now shows:

```text
● LIVE
Last event: <time>
<event count>
```

### Stronger UI capture

V31 uses capture-phase listeners so button/change activity is recorded before workflow handlers can navigate or modify the DOM.

Recorded frontend activity now includes:

```text
button actions
select / checkbox / file changes
text / number / range inputs (debounced)
task changes
arm changes
teaching-stage changes
scroll/navigation activity (throttled)
application hidden / visible
login / logout
session heartbeat
```

### Sensitive input rule

Individual keyboard keys are **not** logged.

PIN/password values remain redacted.

The operator keypad digits are no longer written as generic audit events. Authentication is logged as:

```text
Operator login — SUCCESS
Operator login attempt — ERROR
Operator logout / session lock — SUCCESS
```

### Continuous use versus raw pointer movement

V31 records meaningful operator/software activity plus a 30-second live heartbeat. It intentionally does not record every mouse-coordinate pixel because that creates large amounts of low-value data.

For production robot traceability, Python remains responsible for logging every real backend/robot command and result.


---

## V32 — Journey Auto-Hide on Left Navigation

When the operator opens the Journey panel and then selects any page from the left navigation, the Journey panel now closes automatically.

Applies to left-navigation selections such as:

```text
Home
01 · Setup & Safety
02 · Region Teaching
03 · Merge + Full Sequence
04 · Project Summary
05 · Activity Logs
```

Behavior:

```text
Journey open
→ operator clicks a left-navigation item
→ requested page opens
→ Journey automatically hides
→ full workspace is available
```

This rule only applies when `showView()` is triggered by a `.navbtn` left-navigation control. Programmatic page changes inside guided workflows do not unnecessarily toggle the Journey panel.
