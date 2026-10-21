// Advanced Robot Controls (engineer PIN, server-checked). Connected: the controller's run mode per arm
// (SIM / REAL / CHECK), the hand and the pole, the controller's program list, its global points (the list
// read from the controller, found by name, MoveJ / MoveL and a sequence of them, by the control program's
// runner v14), and the arm moves of the RUN tab (the owner, 17 Oct: "running a saved program, meaning an
// uploaded .txt file (a project in a certain slot) ... in all three run modes"): Run Stored and Run Only
// (a slot, as the control program runs it), Upload Only and Upload + Run (a .txt from this computer),
// Run Both Stored. In Visual sim nothing is sent: those moves, and the global points, are shown in 3D in the
// panel (server/api_advanced.py). Multi Upload, Multi Run, Load Sequence, Run Both .TXT and the arm's own
// drag teach are not connected: they say so (they used to report COMPLETE with nothing sent).

import { get, post, say } from './api.js';
import { ctx, el, fmtT, jobLine, robotCall, stageDialog, tell, watchJob } from './ctx.js';
import { loadPoints, matches, movePoint } from './points.js';
import { borrow, Player } from './seq3d.js';
import { liteMode, speedPct, standIn } from './status.js';
import { frame } from './vp.js';

const S = (side) => (side === 'left' ? 'LEFT' : 'RIGHT');
const readBack = () => (standIn() ? 'stand-in robot: nothing reached a controller' : 'read back');

async function job(side, url, body, what) {
  const j = await robotCall(url, body, { what });
  const done = await watchJob(j, (v) => window.rcSetStatus(side, jobLine(v), true));
  window.rcSetStatus(side, done.state === 'done' ? `${what}: done ✓` : (done.error || `${what} stopped`), false);
  window.rcAppend(side, `${what} // ${done.state.toUpperCase()}${done.error ? ' // ' + done.error : ''}`);
}

// Visual sim: the arm moves shown in the panel in 3D, nothing sent (the owner, 17 Oct: "in all three run
// modes"); each from where the arm stands (a linked arm), else its standing pose.
const simView = {
  player: null, three: null, sides: [], total: 0, done: null,
  build() {
    let card = document.getElementById('liteRcSim');
    if (card) return card;
    const after = document.querySelector('#robotControlModal .rc-dual-toolbar');
    if (!after) return null;
    card = el('div', { id: 'liteRcSim', class: 'lite-rc-sim' },
      el('div', { class: 'lite-section-title' }, el('h3', { text: 'Simulation' }),
        el('span', { class: 'mini-note', text: 'Visual sim: the arm moves are shown here in 3D, and nothing is sent to the robot.' })),
      el('div', { id: 'liteRcSimHost', class: 'lite-rev3d-host lite-rc-sim-host' }),
      el('input', { id: 'liteRcSimTime', type: 'range', min: '0', max: '0', step: '10', value: '0', 'aria-label': 'Simulation time',
        oninput: (e) => { this.player?.pause(); this.player?.seek(Number(e.target.value) / 1000); } }),
      el('div', { id: 'liteRcSimSays', class: 'mini-note' }));
    after.after(card);
    this.player = new Player({ onTime: (t) => this.show(t), onEnd: () => { const f = this.done; this.done = null; if (f) f(); } });
    frame(document.getElementById('liteRcSimHost')).title('Simulation', 'Visual sim: nothing is sent');
    return card;
  },
  async play(timelines, says) {                 // {side: timeline}, on one clock
    if (!this.build()) return;
    const host = document.getElementById('liteRcSimHost');
    this.three = await borrow(host);
    this.three.robot.home();
    for (const s of ['left', 'right']) if (!timelines[s]) this.three.robot.clearTrack(s);
    this.sides = Object.keys(timelines);
    this.total = Math.max(0, ...Object.entries(timelines).map(([s, r]) => this.three.robot.loadTimeline(s, r).total_s));
    document.getElementById('liteRcSimTime').max = String(Math.round(this.total * 1000));
    document.getElementById('liteRcSimSays').textContent = says;
    frame(host).title(`Simulation · ${says}`, 'Visual sim: nothing is sent');
    host.scrollIntoView({ block: 'nearest', behavior: 'smooth' });
    this.player.load(this.total);
    await new Promise((resolve) => { this.done = resolve; this.player.play(); });
  },
  show(t) {
    const host = document.getElementById('liteRcSimHost');
    if (this.three && this.three.viewer.host === host) {
      for (const s of this.sides) this.three.robot.showTime(t, s);
      this.three.viewer.dirty = true;
    }
    const bar = document.getElementById('liteRcSimTime');
    if (bar && document.activeElement !== bar) bar.value = String(Math.round(t * 1000));
    frame(host).time(`${t.toFixed(2)} s / ${this.total.toFixed(2)} s`);
  },
};

const visualSim = async () => (await liteMode()) === 'visual';
const shape = (side, body) => post(`/api/robots/${side}/simulate-move`, { ...body, pid: ctx.pid || null });

// One arm's move shown in 3D (Visual sim).
async function showMove(side, body, says) {
  const r = await shape(side, body);
  window.rcSetStatus(side, `SIMULATION // ${says}`, true);
  window.rcAppend(side, `SIMULATION // ${says} // from ${r.start_from === 'now' ? 'where the arm stands' : 'its standing pose'} // nothing sent`);
  await simView.play({ [side]: r }, says);
  window.rcSetStatus(side, `SIMULATION // ${says}: shown ✓ (nothing sent)`, false);
  return r;
}

// A program run from its slot (Hardware sim: the controller's SIM mode; Real: the arm), with its own pole and hand staged
// first (the owner, 21 Oct): Lite's record of the program it sent; for one it did not send, the operator states them.
// ``status(text, busy)``: where its progress is said (the Trajectory Viewer's own line), else the arm's Advanced status.
export async function runSlot(side, program, stated = null, status = null) {
  const what = `RUN // program ${program}`;
  const set = status || ((text, busy) => window.rcSetStatus(side, text, busy));
  let j;
  try {
    j = await robotCall(`/api/robots/${side}/programs/run`, { program, ...(stated || {}) },
      { what: `Program ${program} runs from its slot with its own pole and hand, staged first; the controller then moves `
        + 'the arm to its first point and plays it' });
  } catch (err) {
    if (err.code !== 'stage' || stated) throw err;
    const st = await stageDialog({ title: `Program ${program}: the pole and hand it runs with`, arm: side,
      body: `${say(err)} Lite stages them before the program runs.` });
    if (!st) { window.toast('Not run: nothing was sent to the robot.', 'warn'); return null; }
    return runSlot(side, program, { pole_mm: st.pole_mm, hand: st.hand }, status);
  }
  const done = await watchJob(j, (v) => set(jobLine(v), true));
  set(done.state === 'done' ? `${what}: done ✓` : (done.error || `${what} stopped`), false);
  if (!status) window.rcAppend(side, `${what} // ${done.state.toUpperCase()}${done.error ? ' // ' + done.error : ''}`);
  if (done.state !== 'done') window.toast(`${done.error || 'The program stopped.'} ${done.hint || ''}`, 'error');
  return done;
}

const fileOf = (side) => document.getElementById(`rc${S(side)}Track`)?.files?.[0] || null;
const slotOf = (side) => {
  const f = document.getElementById(`rc${S(side)}Slot`), id = Number(f?.value);
  if (!Number.isInteger(id) || id < 1 || id > 100) { window.toast(`${S(side)} Program ID must be 1 to 100.`, 'warn'); f?.focus(); return null; }
  return id;
};
const notConnected = (what) => window.toast(`${what} is not connected yet: nothing was sent. Run each program with Run Stored.`, 'warn');

export function installAdvanced() {
  window.rcSetMode = async function rcSetMode(side, mode) {
    try {
      const a = await robotCall(`/api/robots/${side}/controller-mode`, { mode: mode === 'real' ? 'REAL' : 'SIM' },
        { what: `Switching the ${S(side)} controller to REAL` });
      robotControlState[side].mode = a.controller_mode;
      window.rcSetStatus(side, `MODE // ${a.controller_mode} (${readBack()})`, false);
      window.rcAppend(side, `MODE SWITCH // ${a.controller_mode} (${readBack()})`);
    } catch (err) { tell(err); window.rcAppend(side, `MODE SWITCH refused // ${err.message}`); }
  };

  window.rcCheckMode = async function rcCheckMode(side) {
    try {
      const st = await get('/api/robots/status');
      const link = st.arms[side] && st.arms[side].link;
      if (link) {                                     // the live controller, as read (Lite only reads it)
        const mode = (link.feedback && link.feedback.run_mode) || 'not read';
        window.rcAppend(side, `MODE CHECK // live controller: ${mode} (read only) // E-stop ${st.arms[side].estop}`);
        window.toast(`${S(side)} controller: run mode ${mode}, E-stop ${st.arms[side].estop}. Lite only reads it.`, 'good');
        return;
      }
      const a = await robotCall(`/api/robots/${side}/controller-mode`, { mode: 'CHECK' });
      robotControlState[side].mode = a.controller_mode;
      window.rcAppend(side, `MODE CHECK // ${a.connected ? a.controller_mode : 'not connected'} // E-stop ${a.estop}`);
      window.toast(`${S(side)}: ${a.connected ? a.controller_mode : 'not connected'}, E-stop ${a.estop_text}.`, 'good');
    } catch (err) { tell(err); }
  };

  // the Cleaning Pose preset beside Grip and All Open (the owner, 15 Oct): fingers open, thumb bent
  const pagePreset = window.rcHandPreset;
  window.rcHandPreset = function rcHandPreset(side, type) {
    if (type !== 'cleaning') { pagePreset(side, type); return; }
    [1000, 1000, 1000, 1000, 0, 1000].forEach((v, i) => window.rcHandSync(side, i, v, 'num'));
  };
  for (const side of ['left', 'right']) {
    const open = document.querySelector(`#rcArmPane-${side}-hand button[onclick="rcHandPreset('${side}','open')"]`);
    if (open && !open.nextElementSibling?.dataset.liteCleaning) {
      const b = Object.assign(document.createElement('button'), { className: 'btn green', textContent: 'Cleaning Pose' });
      b.dataset.liteCleaning = '1';
      b.addEventListener('click', () => window.rcHandPreset(side, 'cleaning'));
      open.after(b);
    }
  }

  window.rcApplyHand = async function rcApplyHand(side) {
    try { await job(side, `/api/robots/${side}/hand`, { counts: window.rcHandValues(side) }, 'Hand'); } catch (err) { tell(err); }
  };

  window.rcApplyPole = async function rcApplyPole(side) {
    const mm = Number(document.getElementById(`rc${S(side)}PoleNum`)?.value || 0);
    try { await job(side, `/api/robots/${side}/pole`, { mm }, `Pole to ${mm} mm`); } catch (err) { tell(err); }
  };

  // RUN: the controller's programs as a list (the owner, 14 Oct: the run-on line was unreadable); a
  // row fills "Program ID or name".
  window.rcRefreshPrograms = async function rcRefreshPrograms(side) {
    const count = document.getElementById(`rc${S(side)}ProgramCount`);
    try {
      const d = await get(`/api/robots/${side}/programs`);
      const where = standIn() ? ' on the stand-in robot' : '';
      if (count) count.textContent = d.programs.length ? `${d.programs.length} programs${where}. Choose one to fill Program ID.` : `No programs stored yet${where}.`;
      listBox(count, `rc${S(side)}ProgramList`).replaceChildren(table(['ID', 'Name', 'Time', 'Size'], d.programs.map(p => ({
        cells: [String(p.id), p.name || '', p.seconds != null ? fmtT(p.seconds) : '', p.bytes || p.size ? `${Math.round((p.bytes || p.size) / 1000)} kB` : ''],
        pick: () => { const f = document.getElementById(`rc${S(side)}StoredProgram`); if (f) f.value = String(p.id); },
      }))));
      window.rcAppend(side, `PROGRAM LIST // ${d.programs.length} stored`);
    } catch (err) { tell(err); }
  };

  // GLOBAL POINTS: the controller's list, filtered by "Point name / Search" as it is typed; a row
  // fills the name. MoveJ / MoveL at the tab's speed and wait.
  const shown = { left: [], right: [] };
  const renderPoints = (side) => {
    const q = document.getElementById(`rc${S(side)}PointSearch`)?.value || '';
    const rows = shown[side].filter(p => matches(p.name, q));
    listBox(document.getElementById(`rc${S(side)}PointCount`), `rc${S(side)}PointList`).replaceChildren(rows.length
      ? table(['Point', 'Joints (deg)', 'Frames', 'Saved'], rows.map(p => ({
        cells: [p.name, p.joints.map(v => v.toFixed(1)).join(', '), [p.work_frame, p.tool_frame].filter(Boolean).join(' / ') || '—', p.time || ''],
        pick: () => { const f = document.getElementById(`rc${S(side)}PointSearch`); if (f) { f.value = p.name; renderPoints(side); } },
      })))
      : el('div', { class: 'mini-note', text: shown[side].length ? 'No point matches the search.' : '' }));
  };
  window.rcRefreshPoints = async function rcRefreshPoints(side) {
    const count = document.getElementById(`rc${S(side)}PointCount`);
    try {
      const d = await loadPoints(side, true);
      shown[side] = d.points;
      if (count) count.textContent = d.note || `${d.points.length} global points on the ${S(side)} controller. Type to search; choose one to use it.`;
      renderPoints(side);
      window.rcAppend(side, `GLOBAL POINTS // ${d.points.length} read from the controller`);
    } catch (err) { if (count) count.textContent = say(err); tell(err); }
  };
  for (const side of ['left', 'right']) {
    document.getElementById(`rc${S(side)}PointSearch`)?.addEventListener('input', () => renderPoints(side));
  }
  // the panel's Speed % shows the top bar's speed slider: every MoveJ / MoveL goes at it (the owner, 15 Oct)
  const speedOf = () => speedPct();
  const showSpeed = () => {
    for (const side of ['left', 'right']) {
      const f = document.getElementById(`rc${S(side)}PointSpeed`);
      if (!f) continue;
      f.readOnly = true;
      f.title = 'The speed slider in the top bar: every MoveJ / MoveL of both arms';
      if (document.activeElement !== f) f.value = String(speedPct());
    }
  };
  showSpeed();
  document.addEventListener('lite:status', showSpeed);
  const waitOf = (side) => Math.max(0, Number(document.getElementById(`rc${S(side)}PointWait`)?.value) || 0);

  window.rcPointMove = async function rcPointMove(side, motion) {
    const name = (document.getElementById(`rc${S(side)}PointSearch`)?.value || '').trim();
    if (!name) { window.toast('Enter or choose a Global Point name first.', 'warn'); return; }
    const move = String(motion).toLowerCase() === 'movel' ? 'movel' : 'movej';
    try {
      const done = await movePoint(side, name, move, waitOf(side), {}, (v) => window.rcSetStatus(side, jobLine(v), true));
      const what = `GLOBAL POINT // ${motion} // ${name} // ${speedOf()} %`;
      window.rcSetStatus(side, done.state === 'done' ? `${what}: done ✓` : (done.error || `${what} stopped`), false);
      window.rcAppend(side, `${what} // ${done.state.toUpperCase()}${done.error ? ' // ' + done.error : ''}`);
      if (done.state !== 'done') window.toast(`${done.error || 'The move stopped.'} ${done.hint || ''}`, 'error');
    } catch (err) { tell(err); }
  };

  // The page's point sequence, step by step; one confirmation for all of it, and it stops at the
  // first step that does not finish.
  window.rcRunPointSequence = async function rcRunPointSequence(side) {
    const steps = robotControlState[side].pointSequence;
    if (!steps.length) { window.toast('Add one or more Global Points first.', 'warn'); return; }
    let extra = {};
    for (const [i, st] of steps.entries()) {
      const move = String(st.motion).toLowerCase() === 'movel' ? 'movel' : 'movej';
      try {
        const done = await movePoint(side, st.name, move, Number(st.wait) || 0, extra,
          (v) => window.rcSetStatus(side, `Step ${i + 1} of ${steps.length}: ${jobLine(v)}`, true));
        extra = { confirmed: true };
        window.rcAppend(side, `SEQUENCE ${i + 1}/${steps.length} // ${st.motion} // ${st.name} // ${done.state.toUpperCase()}`);
        if (done.state !== 'done') {
          window.rcSetStatus(side, `Sequence stopped at step ${i + 1}: ${done.error || 'the move did not finish'}`, false);
          window.toast(`Sequence stopped at step ${i + 1}: ${done.error || 'the move did not finish.'}`, 'error');
          return;
        }
      } catch (err) { tell(err); return; }
    }
    window.rcSetStatus(side, `Global point sequence done ✓ (${steps.length} steps)`, false);
  };
}

// A list under a count line, made once.
function listBox(after, id) {
  let box = document.getElementById(id);
  if (!box && after) { box = el('div', { id, class: 'lite-rc-list' }); after.after(box); }
  return box || el('div');
}

function table(heads, rows) {
  return el('table', { class: 'lite-table lite-rc-table' },
    el('thead', {}, el('tr', {}, heads.map(h => el('th', { text: h })))),
    el('tbody', {}, rows.map(r => el('tr', { tabindex: '0', title: 'Use this one', onclick: r.pick,
      onkeydown: (e) => { if (e.key === 'Enter') r.pick(); } }, r.cells.map(c => el('td', { text: c }))))));
}

// The RUN tab's arm moves (the page's template said COMPLETE after a timer and sent nothing).
export function installAdvancedRuns() {
  window.rcProgramAction = async function rcProgramAction(side, action) {
    if (action !== 'RUN STORED') { notConnected(action === 'MULTI RUN' ? 'Multi Run' : 'Load Sequence'); return; }
    const program = (document.getElementById(`rc${S(side)}StoredProgram`)?.value || '').trim();
    if (!program) { window.toast('Enter a Program ID or a program name first.', 'warn'); return; }
    try {
      if (await visualSim()) await showMove(side, { program }, `program ${program}`);
      else await runSlot(side, program);
    } catch (err) { tell(err); window.rcAppend(side, `RUN // program ${program} // ${say(err)}`); }
  };

  window.rcTrajectoryAction = async function rcTrajectoryAction(side, action) {
    const id = slotOf(side);
    if (!id) return;
    const file = fileOf(side);
    if (action !== 'RUN ONLY' && !file) { window.toast(`Choose a ${S(side)} trajectory .txt file first.`, 'warn'); return; }
    try {
      if (await visualSim()) {                  // nothing is sent: the file, or the slot's program, shown in 3D
        if (action === 'UPLOAD ONLY') { window.toast('Visual sim sends nothing: choose Hardware sim or Real robot to upload.', 'warn'); return; }
        await (action === 'RUN ONLY' ? showMove(side, { program: String(id) }, `program ${id}`)
          : showMove(side, { name: file.name, content: await file.text() }, file.name));
        return;
      }
      if (action !== 'RUN ONLY') {
        // its pole and hand, recorded with it: every run from its slot stages them first (the owner, 21 Oct)
        const st = await stageDialog({ title: `${file.name}: the pole and hand it was taught with`, arm: side,
          body: `Program ${id} will run from its slot with these: Lite stages them before every run of it.` });
        if (!st) { window.toast('Not uploaded: nothing was sent to the robot.', 'warn'); return; }
        const j = await robotCall(`/api/robots/${side}/programs/upload`, { program_id: id, name: file.name, content: await file.text(),
          pole_mm: st.pole_mm, hand: st.hand }, { what: `Upload of ${file.name}`, replacing: `program ${id}` });
        const note = document.getElementById(`rc${S(side)}UploadNote`);
        if (j.unchanged) {
          if (note) note.textContent = `Program ${id} already holds ${file.name} ✓`;
        } else {
          const done = await watchJob(j, (v) => window.rcSetStatus(side, jobLine(v), true));
          if (note) note.textContent = done.state === 'done' ? `${file.name} is program ${id} on the ${S(side)} controller ✓ (read back)`
            : (done.error || 'The upload did not finish.');
          window.rcAppend(side, `UPLOAD // ${file.name} → program ${id} // ${done.state.toUpperCase()}${done.error ? ' // ' + done.error : ''}`);
          if (done.state !== 'done') { window.rcSetStatus(side, done.error || 'Upload stopped', false); return; }
        }
        window.rcSetStatus(side, `UPLOAD // ${file.name} → program ${id}: done ✓`, false);
        if (action === 'UPLOAD ONLY') return;
      }
      await runSlot(side, String(id));
    } catch (err) { tell(err); window.rcAppend(side, `${action} // ${say(err)}`); }
  };

  window.rcRunDualStored = async function rcRunDualStored() {
    const l = (document.getElementById('rcDualLeftProgram')?.value || '').trim();
    const r = (document.getElementById('rcDualRightProgram')?.value || '').trim();
    if (!l || !r) { window.toast('Enter both LEFT and RIGHT stored programs.', 'warn'); return; }
    try {
      if (await visualSim()) {
        const [tl, tr] = await Promise.all([shape('left', { program: l }), shape('right', { program: r })]);
        for (const [side, p] of [['left', l], ['right', r]]) window.rcSetStatus(side, `SIMULATION // program ${p}`, true);
        await simView.play({ left: tl, right: tr }, `LEFT program ${l} and RIGHT program ${r}, together`);
        for (const [side, p] of [['left', l], ['right', r]]) window.rcSetStatus(side, `SIMULATION // program ${p}: shown ✓ (nothing sent)`, false);
        return;
      }
      await Promise.all([runSlot('left', l), runSlot('right', r)]);
    } catch (err) { tell(err); }
  };

  window.rcMultiUpload = function rcMultiUpload() { notConnected('Multi Upload'); };
  window.rcOpenDualTxt = function rcOpenDualTxt() { notConnected('Run Both .TXT'); };

  // the global points: in Visual sim the move is shown in 3D, from where the arm stands
  const pointMove = window.rcPointMove;
  window.rcPointMove = async function rcPointMove(side, motion) {
    if (!(await visualSim())) { await pointMove(side, motion); return; }
    const name = (document.getElementById(`rc${S(side)}PointSearch`)?.value || '').trim();
    if (!name) { window.toast('Enter or choose a Global Point name first.', 'warn'); return; }
    try { await showMove(side, { point: name, move: String(motion).toLowerCase() }, `${motion} to ${name}`); } catch (err) { tell(err); }
  };
  const pointSequence = window.rcRunPointSequence;
  window.rcRunPointSequence = async function rcRunPointSequence(side) {
    if (!(await visualSim())) { await pointSequence(side); return; }
    const steps = robotControlState[side].pointSequence;
    if (!steps.length) { window.toast('Add one or more Global Points first.', 'warn'); return; }
    let start = null;
    try {
      for (const [i, st] of steps.entries()) {          // each step from where the one before left the arm
        const r = await shape(side, { point: st.name, move: String(st.motion).toLowerCase(), start });
        window.rcSetStatus(side, `SIMULATION // step ${i + 1} of ${steps.length}: ${st.motion} to ${st.name}`, true);
        await simView.play({ [side]: r }, `step ${i + 1} of ${steps.length}: ${st.motion} to ${st.name}`);
        start = r.end;
      }
      window.rcSetStatus(side, `SIMULATION // global point sequence shown ✓ (${steps.length} steps, nothing sent)`, false);
    } catch (err) { tell(err); }
  };
}
