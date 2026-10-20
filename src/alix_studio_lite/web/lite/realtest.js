// Real Test (R4, R13; decision D30). Upload sends the kept tuned version to the Program ID (1–100,
// checked by the server; a slot holding another program needs a confirmation). Move to Start and
// Run go to the robot: in Hardware sim nothing moves (the controller runs the program in its SIM
// mode, the pole and the hand are emulated), in Real the arm, the pole and the hand move after a
// confirmation, and in Visual sim nothing is sent. The run's real outcome is
// shown, and PASS is accepted only after a completed run (tasks.js). Until Lite's robot commands
// exist the robot is the stand-in, and the screens say that nothing reached a controller.

import { get, post, say } from './api.js';
import { REAL_CHECKS, confirmDialog, ctx, el, fmtS, jobLine, loadUnit, robotCall, step, tell, unitUrl, watchJob } from './ctx.js';
import { fillPointGroup, filterOptions, hasController, movePoint, pointControls } from './points.js';
import { borrow, Player } from './seq3d.js';
import { scene } from './sim.js';
import { liteMode, standIn } from './status.js';
import { refreshTasks } from './tasks.js';
import { gateChips } from './tune.js';
import { frame } from './vp.js';

const $ = (id) => document.getElementById(id);

const MODE_NOTE = {
  visual: ['notice', 'Visual sim: nothing is sent to the robot. Step 3 switches to Hardware sim, step 4 to Real robot, by themselves.'],
  hwsim: ['notice', 'Hardware sim: nothing moves. The controller runs the program in its SIM mode (the arm stays still); the pole and the hand are emulated.'],
  real: ['notice warn', 'REAL ROBOT: the arm, the pole and the hand move. Keep the area clear and the red E-stop buttons within reach. Each move asks for confirmation.'],
};

// ── the test run in two steps (the owner, 17 Oct): 3, the program in Hardware sim (the controller's SIM mode),
// passed once it has run to its end, whatever the arm reported on the way (each report a pop-up, bottom right,
// as the vendor's sim mode reports it); 4, the program on the real arm, greyed out until step 3 has passed for
// this version. Each step switches the mode first (Hardware sim; Real robot, which asks to confirm). Below the
// steps, the run as the robot runs it (runView).

async function switchMode(mode) {
  if ((await liteMode()) === mode) return true;
  try {
    await post('/api/robots/mode', { mode });
  } catch (err) {
    if (err.code !== 'confirm') { tell(err); return false; }
    const yes = await confirmDialog({ title: 'Switch to the real robot?', ok: 'Yes, use the real robot', list: REAL_CHECKS,
      body: 'Step 4 runs the trajectory on the physical arm: Lite switches to Real robot, and the controller to its REAL mode.' });
    if (!yes) { window.toast('Not switched: nothing was run.', 'warn'); return false; }
    try { await post('/api/robots/mode', { mode, confirmed: true }); } catch (e2) { tell(e2); return false; }
  }
  document.dispatchEvent(new CustomEvent('lite:mode'));
  return true;
}

function stepCards() {
  const grid = $('stage-real')?.querySelector('.grid.grid-3, .grid.lite-real-steps');
  if (!grid) return {};
  grid.classList.remove('grid-3');
  grid.classList.add('lite-real-steps');
  const three = grid.querySelector('button[onclick*="simulateRealRun"]')?.closest('.card');
  if (three && !three.dataset.lite) {
    three.dataset.lite = 'hwsim';
    const h = three.querySelector('h3');
    if (h) h.firstChild.textContent = 'Test run on Hardware sim ';
    const b = three.querySelector('button');
    b.textContent = '▶ Run on Hardware sim';
    b.className = 'btn';
    $('realRunStatus').textContent = 'Switches to Hardware sim and runs the program in the controller’s SIM mode.';
  }
  let four = $('liteRealStep4');
  if (!four) {
    four = el('div', { id: 'liteRealStep4', class: 'card' },
      el('span', { class: 'badge', text: '4' }),
      el('h3', { style: 'margin-top:8px' }, 'Run on real ', el('span', { class: 'mandatory-star', text: '*' })),
      el('button', { id: 'liteRealRunBtn', class: 'btn orange', type: 'button', text: '▶ Run on the real robot', disabled: true,
        onclick: () => runStep('real') }),
      el('div', { id: 'liteRealRunStatus', class: 'mini-note', style: 'margin-top:8px', text: 'Opens once step 3 has passed.' }));
    grid.append(four);
  }
  return { three, four };
}

async function runStep(kind) {
  const s = step();
  if (!s || !ctx.pid) return;
  const hw = kind === 'hwsim';
  const status = hw ? $('realRunStatus') : $('liteRealRunStatus');
  if (!hw && $('liteRealRunBtn')?.disabled) { window.toast('Step 4 opens once step 3 has run this version to its end in Hardware sim.', 'warn'); return; }
  if (!(await switchMode(hw ? 'hwsim' : 'real'))) { status.textContent = 'Not run: the mode was not switched.'; return; }
  try {
    const u = await loadUnit(s.key);
    await runView.load(u, hw);
    const job = await robotCall(unitUrl(s.key, 'run'), {}, { what: hw ? 'The Hardware sim run' : 'The real run' });
    status.className = 'mini-note';
    runView.start(job);
    const done = await watchJob(job, (j) => { status.textContent = jobLine(j); runView.follow(j); });
    runView.end(done);
    const said = (done.result && done.result.reported) || [];
    const how = `${hw ? 'Hardware sim' : 'real robot'}${standIn() ? ', stand-in robot: nothing reached a controller' : ''}`;
    status.textContent = done.state === 'done'
      ? (hw ? `Passed ✓ (${how}): it ran to its end${said.length ? `, the arm reported ${said.map(r => r.codes).join('; ')}` : ''}. Step 4 is open.`
        : `Test complete ✓ (${how}) — choose PASS, Needs Tuning or Record Again.`)
      : `${hw ? 'Not passed' : 'Stopped'}: ${done.error || 'the run did not finish.'} ${done.hint || ''}`;
    status.className = done.state === 'done' ? 'mini-note lite-ok' : 'mini-note lite-bad';
  } catch (err) { status.textContent = say(err); status.className = 'mini-note lite-bad'; tell(err); }
  refresh();
}

// The run as the robot runs it, below the steps (the owner, 17 Oct: "a viewport that visualizes both stage 3 and
// stage 4"): the version's execution (the pole and the hand staged first, the move to its first line, then the
// file on the 5 ms clock) on the run's own clock, set back on it when the control program starts the program;
// what the arm reported marked where it came. While a run holds the controller Lite cannot read the arm, so this
// is the run as planned, kept in step with the controller's own lines.
const runView = {
  three: null, tl: null, key: null, total: 0, start: 0, marks: [], seen: -1, player: null,

  build() {
    let card = $('liteRealView');
    if (card) return card;
    const grid = $('stage-real')?.querySelector('.lite-real-steps');
    if (!grid) return null;
    card = el('div', { id: 'liteRealView', class: 'card lite-real-view' },
      el('div', { class: 'lite-section-title' }, el('h3', { text: 'The run' }),
        el('span', { class: 'mini-note', text: 'Steps 3 and 4 as the robot runs them: staged first, the move to the first line, then the file.' })),
      el('div', { class: 'lite-s1-sim-grid' },
        el('div', { class: 'lite-rev3d-main' }, el('div', { id: 'liteRealHost', class: 'lite-rev3d-host' })),
        el('div', { class: 'lite-rev3d-side' },
          el('div', { id: 'liteRealTrack', class: 'lite-rev3d-track' },
            el('input', { id: 'liteRealTimeline', type: 'range', min: '0', max: '0', step: '10', value: '0', 'aria-label': 'Run time',
              oninput: (e) => { this.player?.pause(); this.player?.seek(Number(e.target.value) / 1000); } })),
          el('div', { id: 'liteRealPhases', class: 'mini-note' }),
          el('div', { id: 'liteRealReports', class: 'mini-note' }))));
    grid.after(card);
    this.player = new Player({ onTime: (t) => this.show(t) });
    frame($('liteRealHost')).title('The run', 'Step 3 or 4 plays it here');
    return card;
  },

  async load(u, hw) {
    if (!this.build()) return;
    const host = $('liteRealHost');
    const v = u.versions.find(x => x.version === u.version) || u.versions[u.versions.length - 1];
    frame(host).title(`${hw ? 'Hardware sim' : 'Real'} run · ${u.display || u.key} ${v ? v.version : ''}`,
      hw ? 'The controller’s SIM mode: the arm stays still' : 'The physical arm');
    this.three = await borrow(host);
    const r = await get(`${unitUrl(u.key, 'timeline')}?which=version`);
    this.three.robot.home();
    for (const side of ['left', 'right']) if (side !== u.arm) this.three.robot.clearTrack(side);
    this.tl = this.three.robot.loadTimeline(u.arm, r);
    this.key = u.key; this.arm = u.arm; this.total = this.tl.total_s;
    await scene(u.phase || 'lid_closed', undefined, r.scene || null).catch(() => {});
    $('liteRealTimeline').max = String(Math.round(this.total * 1000));
    $('liteRealPhases').textContent = (this.tl.phases || []).map(p => `${fmtS(p.t0)}: ${p.text}`).join(' · ');
    this.marks = []; this.paintMarks();
    $('liteRealReports').textContent = '';
    this.player.load(this.total);
  },

  // the moment the control program starts the program: the end of the staging (the move to the first line)
  runStart() {
    const ph = (this.tl && this.tl.phases) || [];
    const move = ph.find(p => /start|first/i.test(p.text) && !/trajectory:/i.test(p.text));
    return move ? move.t0 : (this.tl ? Math.max(0, this.tl.traj_t0 - 1) : 0);
  },

  start(job) { this.seen = -1; this.player?.seek(0); this.player?.play(); this.follow(job); },

  follow(j) {
    for (const e of j.events || []) {
      if (e.seq <= this.seen) continue;
      this.seen = e.seq;
      const text = String(e.text || '');
      if (/^run slot /.test(text)) this.player.seek(this.runStart());                 // the program starts now
      const rep = /^!! the arm reports (.+) at ([\d.]+) s$/.exec(text);
      if (rep) {                                       // its time is the control program's, from the program's start
        this.marks.push({ t: this.runStart() + Number(rep[2]), at: Number(rep[2]), warn: /a warning/.test(rep[1]), text: rep[1] });
        this.paintMarks();
        $('liteRealReports').textContent = `The arm reported: ${this.marks.map(m => `${m.text}, ${fmtS(m.at)} into the program `
          + `(${fmtS(m.t)} on this timeline)`).join('; ')}.`;
      }
    }
  },

  end(done) {
    this.player?.pause();
    if (done.state === 'done') this.player?.seek(this.total);
  },

  paintMarks() {
    const box = $('liteRealTrack');
    if (!box) return;
    let layer = box.querySelector('.lite-vtools-marks');
    if (!layer) { layer = el('div', { class: 'lite-vtools-marks', 'aria-hidden': 'true' }); box.append(layer); }
    const T = Math.max(1e-6, this.total);
    layer.replaceChildren(...this.marks.map(m => el('span', { class: `lite-vtools-mark${m.warn ? ' warn' : ''}`, title: m.text,
      style: `left:${Math.min(100, (100 * m.t) / T)}%;width:0.6%` })));
  },

  show(t) {
    const three = this.three;
    if (three && this.arm && three.viewer.host === $('liteRealHost')) {
      three.robot.showTime(t, this.arm);
      three.viewer.dirty = true;
    }
    const tl = $('liteRealTimeline');
    if (tl && document.activeElement !== tl) tl.value = String(Math.round(t * 1000));
    frame($('liteRealHost')).time(`${fmtS(t)} / ${fmtS(this.total)}`);
  },
};

// Move to start (feedback v2, slide 7): the tuned trajectory's first line, as before, or one of the
// arm's global points from its controller, by MoveJ or MoveL.
const moveCard = () => $('stage-real')?.querySelector('button[onclick*="trajectory start pose"]')?.closest('.card');

async function startPoints() {
  const card = moveCard();
  if (!card) return;
  let sel = $('liteRealStart');
  if (!sel) {
    sel = el('select', { id: 'liteRealStart', 'aria-label': 'Where Move to Start goes' },
      el('option', { value: 'trajectory', text: 'The tuned trajectory’s first line' }));
    card.querySelector('button')?.before(el('div', { class: 'inline lite-real-start' }, sel));
  }
  const side = selectedArm, linkedArm = hasController(side);
  pointControls(sel, 'lite_realstart', linkedArm);
  await fillPointGroup(sel, side).catch(() => {});
  filterOptions(sel, $('lite_realstartSearch')?.value);
}

async function refresh() {
  startPoints();
  const s = step();
  if (!s || !ctx.pid || s.later) return;
  let u;
  try { u = await loadUnit(s.key); } catch (err) { tell(err); return; }
  const st = await get('/api/robots/status').catch(() => null);
  const note = $('stage-real').querySelector('.notice');
  if (note && st) {
    const [cls, text] = MODE_NOTE[st.mode];
    const linked = Object.values(st.arms || {}).some((a) => a.link);
    note.className = cls;
    note.textContent = st.mode === 'visual' || st.backend !== 'stand-in' || linked ? text
      : `${text} The robot is the stand-in: nothing is sent to a controller.`;
  }
  const v = u.versions.find((x) => x.version === u.version) || u.versions[u.versions.length - 1];     // the version it runs
  $('realTaskLabel').textContent = `${TASKS[currentTask].label} · ${s.display}`;
  const pid = u.settings_view.settings.program_id;
  if (pid && !$('programId').value) $('programId').value = pid;
  window.validateProgramId();
  let box = $('liteVersion');
  if (!box) { box = el('div', { id: 'liteVersion', class: 'lite-version mini-note' }); $('stage-real').querySelector('.real-program-row')?.after(box); }
  const file = v ? (v.name || `${s.key}_${v.version}.txt`) : null;    // the file this section sends (the owner, 15 Oct)
  const limits = !v ? '' : v.limits_ok === false
    ? ` It breaks ${v.limits_text || 'a limit'}: shown, not enforced (the hardware decides); the upload asks once.`
    : ' Every limit passes.';
  box.replaceChildren(
    v ? el('div', { class: 'lite-real-file' }, el('span', { text: 'File' }), el('b', { text: file }), el('small', { text: `version ${v.version}` })) : '',
    gateChips(u), el('div', { text: v
      ? `Tuned version ${v.version} (${file}): ${fmtS(v.achieved_s)}, ${Number(v.lines).toLocaleString()} rows.${limits}`
        + (v.stage ? ` Every run stages ${v.stage.text} first (read back in Real, emulated in Hardware sim); if either fails, nothing runs.` : '')
        + (u.last_run ? ` Last run: ${u.last_run.ok ? 'completed ✓' : 'did not finish'} (${u.last_run.at}).` : ' Not run yet.')
      : 'No tuned version yet: finish Tune + Simulation first.' }));
  const btn = $('uploadTrajectoryBtn');
  let up = $('liteUploadFile');
  if (!up && btn) { up = el('div', { id: 'liteUploadFile', class: 'lite-upload-file mini-note' }); btn.before(up); }
  // the controller keeps a program under a short name of its own (api_units: <region>_<version>, 32 characters)
  if (up) up.textContent = file ? `Sends ${file} (${v.version}), named “${`${s.key}_${v.version}`.slice(0, 32)}” on the controller` : 'No tuned file yet';
  stepCards();
  runView.build();
  const four = $('liteRealRunBtn');                    // step 4: open once step 3 has passed for this version
  if (four) {
    const ok = !!(v && u.hwsim_passed);
    four.disabled = !ok;
    four.title = ok ? `${v.version} ran to its end in Hardware sim (${u.hwsim_passed.at || 'passed'}): run it on the real arm.`
      : 'Opens once step 3 (Hardware sim) has run this version to its end.';
    const st = $('liteRealRunStatus');
    if (st && !st.classList.contains('lite-ok') && !st.classList.contains('lite-bad')) {
      st.textContent = ok ? `Open: ${v.version} passed in Hardware sim.` : 'Opens once step 3 has passed.';
    }
  }
  const next = teachAnotherButton();
  if (next) {                                          // open once the file is on the controller (or its run completed)
    const ready = !!v && ['uploaded', 'dryrun_ok', 'real_ok'].includes(u.status);
    const n = nextTask(currentTask, selectedArm);
    next.disabled = !ready;
    next.title = ready ? `${file} is on the controller: let go of the ${s.arm.toUpperCase()} arm and go on to `
      + `${n ? `${TASKS[n.i].label}, ${n.arm.toUpperCase()} arm` : 'another task'} at Prepare.`
      : 'Opens once this region’s file is uploaded to the controller (or its test run has completed).';
  }
}

// Teach another task (the owner, 15 Oct; 16 Oct: "should progress to the next task in the sequence"): greyed
// out until this region's file is on the controller, uploaded or tested; then the arm is let go and the next
// task of the queue opens at Prepare, on the same arm when that task uses it (else on its other arm); a task
// with nothing to teach in Release 1 is passed. After the queue's last task, Prepare of this task, to choose.
// The file stays on its program; PASS still approves it.
function nextTask(from, arm) {
  const other = arm === 'left' ? 'right' : 'left';
  for (let i = from + 1; i < TASKS.length; i += 1) {
    const t = TASKS[i], can = (a) => !!(t && t.lite && t.lite[a] && !t.lite[a].later);
    if (can(arm)) return { i, arm };
    if (can(other)) return { i, arm: other };
  }
  return null;
}

function teachAnotherButton() {
  let b = $('liteTeachAnother');
  const foot = $('stage-real')?.querySelector('.workspace-footer');
  if (!b && foot) {
    b = el('button', { class: 'btn green', id: 'liteTeachAnother', type: 'button', disabled: true, text: 'Teach another task →',
      onclick: teachAnother });
    foot.append(b);
  }
  return b;
}

async function teachAnother() {
  const s = step();
  if (!s || !ctx.pid || $('liteTeachAnother')?.disabled) return;
  try {                                               // the page's own record of this task, as it stands
    const rec = window.captureCurrentTaskRecord();
    taskRecords[currentTask] = rec;
    window.safeSaveTaskRecords();
  } catch (_) { /* the page still works */ }
  const next = nextTask(currentTask, selectedArm);
  window.auditLog?.('TEACH', 'Teach another task', `${TASKS[currentTask]?.label || ''} · ${selectedArm.toUpperCase()} ARM · `
    + `left Real Test with its file on the controller${next ? ` · next: ${TASKS[next.i].label} · ${next.arm.toUpperCase()} ARM` : ''}`,
  'INFO', { arm: selectedArm });
  armSelectionLocked = false;
  window.updateArmLockUI();
  if (next) {
    selectedArm = next.arm;                            // the task opens on this arm (loadTask selects it)
    window.loadTask(next.i);
    await refreshTasks().catch(() => {});            // the cards and the queue say where each file is now
    if (currentStage !== 'prepare') await window.setStage('prepare');
    document.getElementById('taskBoard')?.scrollIntoView({ behavior: 'smooth', block: 'start' });
    window.toast(`${s.display} stays on the controller. Next: ${TASKS[next.i].label}, ${next.arm.toUpperCase()} arm: prepare it.`, 'good');
    return;
  }
  window.resetWorkingTrajectoryState?.();
  await refreshTasks().catch(() => {});
  await window.setStage('prepare');
  window.setJourneyVisible?.(true, false);
  document.getElementById('taskBoard')?.scrollIntoView({ behavior: 'smooth', block: 'start' });
  window.toast(`${s.display} stays on the controller. ${TASKS[currentTask]?.label || 'This'} is the last task in the queue: `
    + 'choose a task in the Operator Journey, or the other arm, then prepare it.', 'good');
}

// The upload as a horizontal bar with its percentage (the owner, 14 Oct), under the page's status
// line. On the robot the SDK sends in one call, so the server's percentage is estimated until the
// stored size is read back; the bar is red when the upload did not finish.
function uploadBar(status) {
  let box = $('liteUploadBar');
  if (!box && status) {
    box = el('div', { id: 'liteUploadBar', class: 'lite-progress', role: 'progressbar', 'aria-valuemin': '0', 'aria-valuemax': '100' },
      el('div', { class: 'lite-progress-fill' }), el('span', { class: 'lite-progress-text' }));
    status.after(box);
  }
  return {
    show(pct, failed = false) {
      if (!box) return;
      box.hidden = false;
      box.classList.toggle('failed', !!failed);
      if (pct == null) return;
      const v = Math.max(0, Math.min(100, Math.round(pct)));
      box.querySelector('.lite-progress-fill').style.width = `${v}%`;
      box.querySelector('.lite-progress-text').textContent = `${v} %`;
      box.setAttribute('aria-valuenow', String(v));
    },
  };
}

export function installRealTest() {
  window.simulateUpload = async function simulateUpload() {
    if (!window.validateProgramId()) { window.toast('Enter a valid Program ID from 1 to 100 before Upload.', 'warn'); $('programId')?.focus(); return; }
    const s = step();
    if (!s || !ctx.pid) return;
    const pid = Number($('programId').value);
    const status = $('uploadStatus');
    const bar = uploadBar(status);
    try {
      const out = await robotCall(unitUrl(s.key, 'upload'), { program_id: pid }, { what: 'Upload' });
      if (out.unchanged) { status.textContent = `Program ${pid} already holds this version ✓`; bar.show(100); return; }
      bar.show(0);
      const done = await watchJob(out, (j) => { status.textContent = jobLine(j); bar.show(j.n ? (100 * j.i) / j.n : 0); });
      bar.show(done.state === 'done' ? 100 : null, done.state !== 'done');
      status.textContent = done.state !== 'done' ? (done.error || 'Upload stopped.')
        : standIn() ? `Uploaded to Program ID ${pid} on the stand-in robot ✓ (nothing reached a controller)`
          : `Uploaded to Program ID ${pid} ✓ (name and size read back)`;
      if (done.state === 'done') {                          // the page's own record of the Program ID
        const rec = window.captureCurrentTaskRecord();
        rec.programId = pid;
        rec[`${selectedArm}ProgramId`] = pid;
        taskRecords[currentTask] = rec;
        window.safeSaveTaskRecords();
      }
    } catch (err) { status.textContent = err.message; bar.show(null, true); tell(err); }
    refresh();
  };

  // the page's Test run button is step 3: the program in Hardware sim
  window.simulateRealRun = function simulateRealRun() { return runStep('hwsim'); };

  const pageAction = window.demoAction;
  window.demoAction = async function demoAction(x) {
    const v = $('liteRealStart')?.value || '';
    if (currentStage !== 'real' || !/start pose/i.test(String(x)) || !v.startsWith('gp:')) return pageAction(x);
    const side = selectedArm, name = v.slice(3), motion = $('lite_realstartMotion')?.value || 'movej';
    const status = moveCard()?.querySelector('.mini-note');
    try {
      const done = await movePoint(side, name, motion, 0, {}, (j) => { if (status) status.textContent = jobLine(j); });
      const real = (await liteMode()) === 'real';
      const said = done.state === 'done'
        ? `${side.toUpperCase()} at “${name}” (${motion === 'movel' ? 'MoveL' : 'MoveJ'})${real ? ', read back' : ': the controller’s SIM mode, nothing moved'}.`
        : `Stopped: ${done.error || 'the move did not finish.'} ${done.hint || ''}`;
      if (status) status.textContent = said;
      window.toast(said, done.state === 'done' ? 'good' : 'error');
    } catch (err) { if (status) status.textContent = say(err); tell(err); }
    return undefined;
  };
  document.addEventListener('lite:points', (e) => { if (currentStage === 'real' && e.detail === selectedArm) startPoints(); });

  const pageSync = window.syncRealTestTask;
  window.syncRealTestTask = function syncRealTestTask() {
    pageSync();
    refresh();
  };

  document.addEventListener('lite:stage', (e) => { if (e.detail === 'real') refresh(); });
  document.addEventListener('lite:mode', () => { if (currentStage === 'real') refresh(); });
}
