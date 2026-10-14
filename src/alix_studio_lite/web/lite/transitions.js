// V32's Task Transition & Start Point Tools (Prepare, optional; handoff V22), with the robot
// behind them. Each task's arm step can have:
// - a transition trajectory: a .txt that takes the arm from where the previous task left it
//   toward this task's start. It is kept in the project (trajectories/transitions/<arm>/<step>/,
//   the handoff's folder) with the arm and the pole and hand it was taught with, like every
//   recording, so it still runs after a reload. ▶ Simulation plays it in 3D (staged first, on the
//   5 ms clock); ▶ <ARM> Real Arm sends it to a Program ID and runs it on that arm alone, in
//   Hardware sim or Real: its limits are checked on its bytes first, and the other arm stays stopped
//   while it runs (the server refuses either otherwise).
// - named start points (the page keeps them in the task's record, saved with the project):
//   Capture Current Pose reads the linked controller (J1-J7, X Y Z RX RY RZ, the pole, the hand,
//   the time, the controller); Previous Task End and Use Trajectory End take the last line of the
//   previous task's file (its kept version, else its active recording) or of the transition; Move to
//   Start Point moves the arm to its joints (asked first on the real robot).

import { get, post, say } from './api.js';
import { ctx, el, jobLine, robotCall, stageDialog, step, armFromName, tell, unitUrl, watchJob } from './ctx.js';
import * as sim from './sim.js';

const ARM = (side) => side.toUpperCase();
const views = {};                       // "<project>/<unit key>" -> the server's view of its transition (or null)

const unitOf = (side, i = currentTask) => step(i, side);

async function transitionOf(side, refresh = false) {
  const u = unitOf(side);
  if (!u || !ctx.pid) return null;
  const k = `${ctx.pid}/${u.key}`;
  if (refresh || !(k in views)) {
    try { views[k] = (await get(unitUrl(u.key, 'transition'))).transition; } catch (_) { views[k] = null; }
  }
  return views[k];
}

const deg = (j) => (Array.isArray(j) ? j.map(v => Number(v).toFixed(1)).join(', ') : '—');

function noUnit(side) {
  const t = TASKS[currentTask];
  window.toast(`${t ? t.label : 'This task'} has no ${ARM(side)}-arm step, so it has no ${ARM(side)} transition or start point.`);
}

// The card's lines: the file kept in the project, and what the chosen start point holds.
async function paint() {
  for (const side of ['left', 'right']) {
    const S = side === 'left' ? 'Left' : 'Right';
    const fs = document.getElementById(`transitionFile${S}Status`);
    const tv = await transitionOf(side);
    if (fs) {
      fs.textContent = tv
        ? `${tv.name} · kept in the project · ${tv.lines} lines, ${tv.seconds.toFixed(2)} s · staged first: ${tv.stage.text}`
          + ` · ends at J1–J7 ${deg(tv.end_joints_deg)}°` + (tv.limits_ok ? ' · passes L1–L4' : ` · ${tv.limits_text}`)
        : `No ${ARM(side)} transition trajectory selected.`;
    }
    const rec = window.taskRecordForStartTools();
    const p = (rec.startPoints[side] || []).find(x => x.id === rec.selectedStartPoint?.[side]);
    const st = document.getElementById(`taskStartPoint${S}Status`);
    if (st && p) {
      const bits = [p.name, window.startPointSourceText(p)];
      bits.push(p.joints ? `J1–J7 ${deg(p.joints)}°` : 'no joints: it cannot be moved to');
      if (p.pose) bits.push(`X Y Z ${p.pose.slice(0, 3).map(v => (v * 1000).toFixed(1)).join(', ')} mm`);
      if (p.pole_mm != null) bits.push(`pole ${Number(p.pole_mm).toFixed(0)} mm`);
      if (p.controller) bits.push(`${p.controller.ip}`);
      st.textContent = bits.join(' · ');
    }
  }
}

function pointName(side, name, fallback) {
  const S = side === 'left' ? 'Left' : 'Right';
  const input = document.getElementById(`newStartPoint${S}`);
  const typed = input?.value?.trim();
  if (input) input.value = '';
  return typed || name || fallback;
}

// ── the 3D for ▶ Simulation (the page's one 3D view, in a window of its own)

let box = null;
async function simulate(side) {
  const u = unitOf(side);
  const tv = await transitionOf(side, true);
  if (!tv) { window.toast(`Choose a ${ARM(side)} transition trajectory .txt file first.`); return; }
  box?.remove();
  const host = el('div', { class: 'lite-transition-3d', 'aria-label': '3D simulation of the transition' });
  const line = el('div', { class: 'mini-note', text: 'Loading the 3D view…' });
  const play = el('button', { class: 'btn green', text: '▶ Play' });
  const again = el('button', { class: 'btn light', text: '↺ Restart' });
  const close = el('button', { class: 'btn outline', text: 'Close' });
  play.disabled = again.disabled = true;                 // until the 3D has the file
  const card = el('div', { class: 'modal-card accessible-modal lite-transition-card', role: 'dialog', 'aria-modal': 'true' },
    el('h3', { text: `${ARM(side)} transition · ${tv.name}` }),
    el('p', { class: 'mini-note', text: `${u.display}: the pole and the hand staged first (${tv.stage.text}), then the move to its first line, then the file at 5 ms per line (${tv.seconds.toFixed(2)} s). The other arm stays where it is.` }),
    host, line, el('div', { class: 'modal-actions' }, again, play, close));
  box = el('div', { class: 'modal show', id: 'liteTransitionSim' }, card);
  document.body.append(box);
  const end = () => { sim.release(); box?.remove(); box = null; document.dispatchEvent(new CustomEvent('lite:viewer-closed')); };
  close.onclick = end;
  sim.invalidate();
  try {
    await sim.showFile(host, { key: u.key, arm: side, phase: u.phase, display: u.display }, 'transition');
  } catch (err) { line.textContent = say(err); play.disabled = again.disabled = true; return; }
  if (!box) return;                                        // closed while it loaded
  line.textContent = 'Ready: ▶ Play runs it once, as the robot would.';
  play.disabled = again.disabled = false;
  let playing = false;
  const run = async () => {
    if (playing) return;
    playing = true;
    play.disabled = true;
    const ok = await sim.playOnce((t) => { line.textContent = t; });
    if (ok) line.textContent = `Done: the ${ARM(side)} arm is at the transition's end.`;
    playing = false;
    play.disabled = false;
  };
  play.onclick = run;
  again.onclick = () => { sim.rewind(); playing = false; play.disabled = false; line.textContent = 'At the start: ▶ Play.'; };
}

// ── ▶ <ARM> Real Arm: send it to a Program ID, then run it on that arm alone

function programInput(side) {
  const S = side === 'left' ? 'Left' : 'Right';
  let inp = document.getElementById(`liteTransitionProgram${S}`);
  if (!inp) {
    const actions = document.querySelector(`#transitionFile${S}`)?.closest('.transition-arm-card')?.querySelector('.transition-run-actions');
    inp = el('input', { id: `liteTransitionProgram${S}`, type: 'number', min: '1', max: '100', step: '1', placeholder: 'Program ID',
      'aria-label': `${ARM(side)} transition Program ID`, class: 'lite-transition-program' });
    actions?.prepend(inp);
  }
  return inp;
}

async function runReal(side) {
  const u = unitOf(side);
  const tv = await transitionOf(side, true);
  if (!tv) { window.toast(`Choose a ${ARM(side)} transition trajectory .txt file first.`); return; }
  const inp = programInput(side);
  const pid = Number(inp.value);
  if (!(pid >= 1 && pid <= 100)) { inp.focus(); window.toast('Enter the Program ID the transition is sent to (1 to 100).'); return; }
  const S = side === 'left' ? 'Left' : 'Right';
  const fs = document.getElementById(`transitionFile${S}Status`);
  const show = (j) => { if (fs) fs.textContent = `${ARM(side)} transition: ${jobLine(j)}`; };
  try {
    const up = await robotCall(unitUrl(u.key, 'transition/upload'), { program_id: pid }, { what: `Sending the ${ARM(side)} transition` });
    if (up && up.id) {
      const j = await watchJob(up, show);
      if (j.state !== 'done') throw Object.assign(new Error(j.error || 'The upload did not finish.'), { hint: j.hint });
    }
    const job = await robotCall(unitUrl(u.key, 'transition/run'), { program_id: pid },
      { what: `The ${ARM(side)} transition (${tv.name}), with the ${side === 'left' ? 'RIGHT' : 'LEFT'} arm stopped,` });
    const j = await watchJob(job, show);
    window.toast(j.state === 'done' ? `${ARM(side)} transition finished: the arm is at its end.` : `${ARM(side)} transition stopped: ${j.error || j.state}`);
  } catch (err) { tell(err); }
  paint();
}

export function installTransitions() {
  window.transitionTrajectorySelected = async function transitionTrajectorySelected(side, event) {
    const file = event.target.files?.[0];
    event.target.value = '';
    if (!file) return;
    const u = unitOf(side);
    if (!u) { noUnit(side); return; }
    const named = armFromName(file.name);
    if (named && named !== side) {
      window.toast(`${file.name}'s name says the ${ARM(named)} arm: a ${ARM(side)} transition must be a ${ARM(side)}-arm file.`);
      return;
    }
    const said = await stageDialog({ title: `${ARM(side)} transition: ${file.name}`, arm: named,
      body: `The pole height and hand pose ${file.name} was taught with: every run stages them first.` + (named ? '' : ' Its name does not say the arm.'),
      setup: u.setup, askArm: !named });
    if (!said) return;
    if (said.arm !== side) { window.toast(`A ${ARM(said.arm)}-arm file cannot be the ${ARM(side)} transition.`); return; }
    try {
      const out = await post(unitUrl(u.key, 'transition'), { name: file.name, content: await file.text(), arm: said.arm,
        pole_mm: said.pole_mm, hand: said.hand });
      views[`${ctx.pid}/${u.key}`] = out.transition;
      const rec = window.taskRecordForStartTools();
      rec.transitionTrajectories[side] = { name: out.transition.name, size: file.size, lastModified: file.lastModified,
        selectedAt: new Date().toISOString(), file: out.transition.file, sha256: out.transition.sha256, lines: out.transition.lines,
        project_path: out.transition.project_path };
      window.safeSaveTaskRecords();
      window.renderTaskStartPointTools();
      window.toast(`${out.transition.name} kept in the project as the ${ARM(side)} transition.`);
    } catch (err) { tell(err); }
  };

  window.runTransitionTrajectory = function runTransitionTrajectory(side, mode) {
    window.auditLog(mode === 'real' ? 'ROBOT' : 'SIMULATION', 'Single-arm transition trajectory requested', `${ARM(side)} · ${mode}`, 'REQUESTED', { arm: side });
    if (!unitOf(side)) { noUnit(side); return; }
    (mode === 'real' ? runReal(side) : simulate(side)).catch(tell);
  };

  window.captureTaskStartPoint = async function captureTaskStartPoint(side) {
    if (!unitOf(side)) { noUnit(side); return; }
    try {
      const c = await get(`/api/robots/${side}/capture`);
      const n = (window.taskRecordForStartTools().startPoints[side]?.length || 0) + 1;
      const point = window.addTaskStartPointRecord(side, pointName(side, null, `${ARM(side)} Start ${n}`), 'captured-current-pose',
        { joints: c.joints_deg, pose: c.pose, pole_mm: c.pole_mm, hand: c.hand, capturedAt: c.at, controller: c.controller });
      window.toast(`${point.name}: the ${ARM(side)} arm's pose read from its controller.`);
    } catch (err) { tell(err); }
  };

  window.addPreviousTaskEndStartPoint = async function addPreviousTaskEndStartPoint(side) {
    if (currentTask <= 0) { window.toast('Task 1 has no previous task end position. Use Capture Current Pose or a transition trajectory.'); return; }
    const prev = TASKS[currentTask - 1], pu = unitOf(side, currentTask - 1);
    if (!unitOf(side)) { noUnit(side); return; }
    if (!pu) { window.toast(`${prev.label} has no ${ARM(side)}-arm step, so it has no ${ARM(side)} end.`); return; }
    try {
      const e = await get(`/api/projects/${ctx.pid}/units/${pu.key}/end`);
      const point = window.addTaskStartPointRecord(side, pointName(side, `${ARM(side)} · ${prev.label} End`), 'previous-task-end',
        { joints: e.joints_deg, referenceTaskIndex: currentTask - 1, referenceTaskLabel: prev.label, trajectoryFile: e.name,
          trajectorySha256: e.sha256, from: e.which, pole_mm: e.stage.pole_mm, hand: e.stage.hand });
      window.toast(`${point.name}: the last line of ${e.name} (${e.which === 'version' ? 'its kept version' : 'the recording as taught'}).`);
    } catch (err) { tell(err); }
  };

  window.saveTrajectoryEndAsStartPoint = async function saveTrajectoryEndAsStartPoint(side) {
    if (!unitOf(side)) { noUnit(side); return; }
    const tv = await transitionOf(side, true);
    if (!tv) { window.toast(`Choose a ${ARM(side)} transition trajectory first.`); return; }
    const point = window.addTaskStartPointRecord(side, pointName(side, `${ARM(side)} · ${tv.name.replace(/\.txt$/i, '')} End`), 'trajectory-end',
      { joints: tv.end_joints_deg, trajectoryFile: tv.name, trajectorySha256: tv.sha256, pole_mm: tv.stage.pole_mm, hand: tv.stage.hand });
    window.toast(`${point.name}: the transition's last line.`);
  };

  window.moveToTaskStartPoint = async function moveToTaskStartPoint(side) {
    const rec = window.taskRecordForStartTools(), id = rec.selectedStartPoint?.[side];
    const point = (rec.startPoints[side] || []).find(p => p.id === id);
    if (!point) { window.toast(`Select a ${ARM(side)} start point first.`); return; }
    if (!Array.isArray(point.joints)) {
      window.toast(`${point.name} has no joint values (it was saved before Lite read them): capture it again.`);
      return;
    }
    const S = side === 'left' ? 'Left' : 'Right';
    const st = document.getElementById(`taskStartPoint${S}Status`);
    try {
      const job = await robotCall(`/api/robots/${side}/move-joints`, { joints_deg: point.joints, name: point.name },
        { what: `Moving the ${ARM(side)} arm to “${point.name}”` });
      const j = await watchJob(job, (v) => { if (st) st.textContent = jobLine(v); });
      window.toast(j.state === 'done' ? `${ARM(side)} arm at “${point.name}”.` : `${ARM(side)} move stopped: ${j.error || j.state}`);
    } catch (err) { tell(err); }
    paint();
  };

  const pageRender = window.renderTaskStartPointTools;
  window.renderTaskStartPointTools = function renderTaskStartPointTools() {
    pageRender();
    programInput('left'); programInput('right');
    paint();
  };
  const note = document.querySelector('.transition-backend-note');
  if (note) note.replaceChildren(el('b', { text: 'With Lite: ' }), 'a transition is kept in the project with the pole and hand it '
    + 'was taught with. ▶ Simulation plays it in 3D; ▶ Real Arm sends it to the Program ID and runs it on that arm alone '
    + '(Hardware sim or Real, after its limits are checked; the other arm stays stopped). Capture Current Pose reads the '
    + 'connected controller; Previous Task End and Trajectory End take the file\'s last line; Move to Start Point goes to its joints.');
}
