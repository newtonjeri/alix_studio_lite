// Merge + Full Sequence: the Full Cleaning Sequence Simulation in 3D (R8, R10), in place of the
// page's reference picture, in the page itself (feedback v2, slide 12: what to run at the top, the
// simulation in the middle at full width, the real machine below; run.js makes the top and the
// bottom). It plays what is chosen there: the full sequence, one arm's full sequence alone, single
// files of the project, the controller's global points, or a program Lite sent. The included rows of the flow table run as the robot would run them:
// each arm step's file (the region's kept version; or, as a preview, its recording as taught),
// each arm continuing from where its previous step left it: the pole, then the hand, then the move
// to the file's first line, then the trajectory on the 5 ms clock, with the row's rule (TOGETHER,
// LEFT/RIGHT RUN FIRST, ONE BY ONE, PROCEDURE) and checkpoint wait. The server makes the schedule
// and refuses any arm step of the other arm or without its stage (server/sequence.py); the 3D
// frames of each step are fetched as the clock reaches it. Only a run of kept versions with every
// row ready, played from the start without skipping, can approve the sequence.

import { get, post, say } from './api.js';
import { ctx, el, fmtS, tell, unitUrl } from './ctx.js';
import { ensure3D, release, say3d, scene } from './sim.js';

const $ = (id) => document.getElementById(id);
const ARMS = ['left', 'right'];
const SOURCES = [['version', 'Kept versions · what the robot runs'], ['recording', 'Recordings as taught · preview']];
// clean: the run started at 0 and nothing was skipped (no slider move, no step that could not be shown)
// req: what is played (run.js): mode both | left | right (the flow table's included rows), file (items),
// points (a side's global points), program (a program Lite sent, as its file)
export const seq = { plan: null, source: 'version', acts: { left: [], right: [] }, cache: new Map(), shown: { left: null, right: null },
  phase: null, sceneKey: '', loading: false, token: 0, ui: null, clean: false, req: { mode: 'both' } };

export function setRequest(req) {
  seq.req = req;
  if (seq.ui) { window.fullSequenceSimulationControl('reset', false); loadPlan(); }
}
export const planOf = () => seq.plan;

// ── the flow table's files (feedback v2, slide 8): under each row's LEFT and RIGHT flow, the kept
// version that arm step runs, chosen from its versions, with its time and limits, and the controller
// programs that hold it

let flowCall = 0;
async function decorateFlow() {
  if (!ctx.pid || !$('sequenceTableBody')) return;
  const call = ++flowCall;
  let flow;
  try { flow = await get(`/api/projects/${ctx.pid}/flow-files`); } catch (_) { return; }
  if (call !== flowCall) return;
  const byLabel = new Map(flow.tasks.map((t) => [t.label, t]));
  [...document.querySelectorAll('#sequenceTableBody tr')].forEach((tr, i) => {
    const t = byLabel.get(sequenceConfig[i]?.label) || flow.tasks[i];
    for (const side of ARMS) {
      const cell = tr.querySelector(`.${side}-edit-flow`)?.closest('td');
      if (!cell) continue;
      cell.querySelector('.lite-flow-file')?.remove();
      cell.append(fileBox(side, t && t[side]));
    }
  });
}

function fileBox(side, a) {
  if (!a || a.later) return el('div', { class: 'lite-flow-file mini-note', text: a ? 'Later: not in Release 1.' : `No ${side.toUpperCase()} region in this task.` });
  if (!a.versions.length) {
    return el('div', { class: 'lite-flow-file lite-bad', text: a.recording ? `No kept version yet · recording ${a.recording}` : 'No recording yet' });
  }
  const sel = el('select', { class: 'lite-flow-version', 'aria-label': `${side.toUpperCase()} file of ${a.display}`,
    onchange: (e) => chooseVersion(a, e.target.value) },
  a.versions.map((v) => el('option', { value: v.version, selected: v.version === a.version,
    text: `${v.version} · ${v.name} · ${fmtS(v.achieved_s)}${v.uploadable ? '' : ' · fails a limit'}` })));
  const cur = a.versions.find((v) => v.version === a.version);
  const up = cur && cur.uploaded.length;
  return el('div', { class: 'lite-flow-file' }, sel, el('small', { class: up ? 'lite-ok' : 'mini-note',
    text: up ? `Uploaded: ${cur.uploaded.map((u) => `program ${u.program_id}${u.name ? ` (“${u.name}”)` : ''}`).join(', ')}` : 'Not uploaded yet' }));
}

async function chooseVersion(a, version) {
  try {
    ctx.units[a.key] = await post(unitUrl(a.key, 'version'), { version });
    window.toast(`${a.display} now runs ${version} (Real Test and the full sequence).`);
    if (seq.plan) loadPlan();
  } catch (err) { tell(err); }
  decorateFlow();
}

// ── the plan

export async function loadPlan() {
  const token = ++seq.token;
  seq.plan = null;
  seq.acts = { left: [], right: [] };                         // nothing of the previous plan is shown meanwhile
  seq.clean = false;
  seq.cache.clear();
  seq.shown = { left: null, right: null };
  if (seq.ui) ensure3D().then(({ robot }) => robot.home()).catch(() => {});     // the plan never waits for the 3D
  showPlan(null, 'Working out what to play…');
  const steps = window.fullSequenceIncludedSteps();
  if (!ctx.pid) { showPlan(null, 'Open a project first.'); return; }
  const r = seq.req;
  let plan;
  try {
    if (r.mode === 'points') {
      if (!r.points || !r.points.length) { showPlan(null, r.why || 'Add the global points to play.'); return; }
      plan = await post(`/api/projects/${ctx.pid}/points-plan`, { side: r.side, points: r.points.map(x => x.name), task: TASKS[currentTask]?.label });
    } else if (r.mode === 'file' || r.mode === 'program') {
      if (!r.items || !r.items.length) { showPlan(null, r.why || 'Add the files to play.'); return; }
      plan = await post(`/api/projects/${ctx.pid}/sequence`, { items: r.items, only: r.side, source: 'version' });
    } else {
      plan = await post(`/api/projects/${ctx.pid}/sequence`, { steps, source: seq.source, ...(r.mode !== 'both' ? { only: r.mode } : {}) });
    }
  } catch (err) { if (token === seq.token) showPlan(null, say(err)); return; }
  if (token !== seq.token) return;
  seq.plan = plan;
  seq.acts = { left: [], right: [] };
  for (const row of plan.rows) {
    for (const side of ARMS) {
      const a = row.arms[side];
      if (a && a.ready) seq.acts[side].push(Object.assign(a, { row }));
    }
  }
  showPlan(plan);
  const total = $('fullSimOverlayTotal');
  if (total) total.textContent = fmt(plan.total_s);
  syncSlider();
  await show(0);
  paintPanels(0);
  document.dispatchEvent(new CustomEvent('lite:run-plan'));
}

const flowMode = () => ['both', 'left', 'right'].includes(seq.req.mode);

export function canRun() {
  const p = seq.plan;
  if (!p) return false;
  return flowMode() ? (seq.source === 'version' ? p.ready : p.playable) : p.playable;
}

function armLine(side, a) {
  if (!a) return el('span', { class: 'mini-note', text: `${side.toUpperCase()}: nothing` });
  const what = a.skip ? `skipped · ${a.why}` : a.ready ? `${a.file} · ${fmtS(a.traj_s)} · ${a.stage.text}` : `✕ ${a.why}`;
  return el('span', { class: a.ready ? 'lite-ok' : a.skip ? 'mini-note' : 'lite-bad' }, el('b', { text: `${side.toUpperCase()} ${a.display}` }), ` · ${what}`);
}

function showPlan(plan, message = '') {
  const box = seq.ui && seq.ui.plan;
  if (!box) return;
  const head = box.querySelector('summary');
  const body = box.querySelector('.lite-seq-rows');
  if (!plan) { head.textContent = message; body.replaceChildren(); return; }
  const blocked = plan.blocking.length;
  head.textContent = `${plan.rows.length} rows · ${fmt(plan.total_s)}`
    + (blocked ? ` · ${blocked} arm step${blocked === 1 ? '' : 's'} not ready` : ' · every arm step ready ✓');
  const what = seq.req.mode === 'both' ? (seq.source === 'version'
    ? (plan.ready ? 'Every included arm step runs its kept version, staged first: this run can approve the sequence.'
      : 'The full simulation runs the kept versions, every one of them: tune the regions listed ✕, or exclude their rows from the flow, then open it again.')
    : 'Preview: each region\'s recording as taught. Arm steps without one stand still. A preview never approves the sequence.')
    : flowMode() ? `The ${seq.req.mode.toUpperCase()} arm's full sequence alone: the other arm stands still. Only the full sequence of both arms approves the sequence.`
      : seq.req.mode === 'points' ? `The ${seq.req.side.toUpperCase()} arm through its global points.`
        : `Single files on the ${seq.req.side.toUpperCase()} arm, one after another, each staged first.`;
  body.replaceChildren(
    el('p', { class: `mini-note ${canRun() ? '' : 'lite-bad'}`, text: what }),
    ...plan.rows.map(r => el('div', { class: 'lite-seq-row', 'data-row': r.index },
      el('div', {}, el('b', { text: `${r.index + 1} · ${r.label}` }), el('small', { text: ` · ${r.rule}${r.wait_s ? ` · then wait ${r.wait_s} s` : ''} · ${fmt(r.t0)}` })),
      r.note ? el('small', { class: 'mini-note', text: r.note }) : null,
      ...ARMS.filter(s => r.arms[s]).map(s => armLine(s, r.arms[s])))));
  box.open = !canRun() || box.open;
  setStatus(canRun() ? (seq.source === 'version' ? 'Ready: press ▶ Start. Each step stages the pole and the hand first.' : 'Preview ready: press ▶ Start.')
    : 'Not ready: the steps marked ✕ in the list need a kept version.', canRun() ? '' : 'warn');
}

function setStatus(text, cls = '') {
  const st = $('fullSimStatus');
  if (st) { st.textContent = text; st.className = `statusbar ${cls}`.trim(); }
  const line = seq.ui && seq.ui.status;
  if (line) { line.textContent = text; line.className = `lite-seq-status ${cls}`.trim(); }
}

// ── the 3D at sequence time g

function actAt(side, g) {
  const list = seq.acts[side];
  let a = null;
  for (const x of list) { if (x.stage_at <= g) a = x; else break; }
  return a;
}

const nextAct = (side, a) => { const list = seq.acts[side]; return list[list.indexOf(a) + 1] || null; };

function local(a, g) {
  const t = g < a.stage_at + a.traj_t0 ? g - a.stage_at : a.traj_t0 + Math.max(0, g - a.traj_at);
  return Math.max(0, Math.min(a.traj_t0 + a.traj_s, t));
}

// An arm step's 3D frames, fetched once: {result} when they arrived, {error} when refused.
function timeline(a) {
  const id = `${a.side}:${a.row.index}`;
  let e = seq.cache.get(id);
  if (!e) {
    e = { result: null, error: null };
    (a.timeline ? post(a.timeline.url, a.timeline.body) : post(unitUrl(a.key, 'timeline'), { which: a.which, start: a.start, task: a.row.task }))
      .then((r) => { e.result = r; }, (err) => { e.error = err; });
    seq.cache.set(id, e);
  }
  return e;
}

function prune(g) {
  const keep = new Set();
  for (const side of ARMS) {
    const a = actAt(side, g) || seq.acts[side][0];
    if (!a) continue;
    keep.add(`${side}:${a.row.index}`);
    const n = nextAct(side, a);
    if (n) keep.add(`${side}:${n.row.index}`);
  }
  for (const id of [...seq.cache.keys()]) if (!keep.has(id)) seq.cache.delete(id);
}

// Shows sequence time g. Resolves true when shown, false while a step's 3D frames are still on
// their way, or the server's refusal (an Error) for a step that cannot be shown.
async function show(g) {
  if (!seq.plan) return false;
  const { robot, viewer } = await ensure3D();
  let ready = true;
  for (const side of ARMS) {
    const a = actAt(side, g);
    const n = a ? nextAct(side, a) : seq.acts[side][0];
    if (n) timeline(n);                                       // the next step, early
    if (!a) {
      if (seq.shown[side]) { robot.clearTrack(side); seq.shown[side] = null; }
      continue;
    }
    const e = timeline(a);
    if (e.error) return e.error;
    if (!e.result) { ready = false; continue; }
    if (seq.shown[side] !== a) { robot.loadTimeline(side, e.result); seq.shown[side] = a; }
    robot.showTime(local(a, g), side);
  }
  const row = rowAt(g);                                       // the toilet as the row's region needs it
  const phase = (row && (Object.values(row.arms).find(x => x.ready && x.phase) || {}).phase) || 'lid_closed';
  const shownRes = row && Object.values(row.arms).filter(x => x.ready).map(x => (seq.cache.get(`${x.side}:${row.index}`) || {}).result).find(Boolean);
  const place = shownRes ? shownRes.scene || null : undefined;            // the UGV where its task places it
  const key = JSON.stringify([phase, place && place.offset_m, place && place.walls]);
  if (key !== seq.sceneKey) { seq.sceneKey = key; seq.phase = phase; scene(phase, undefined, place).catch(() => {}); }
  viewer.dirty = true;
  prune(g);
  return ready;
}

function rowAt(g) {
  if (!seq.plan) return null;
  let r = null;
  for (const x of seq.plan.rows) { if (x.t0 <= g) r = x; else break; }
  return r;
}

// What each arm is doing at g, for the arm panels.
function armState(side, g) {
  const row = rowAt(g);
  const a = row && row.arms[side];
  if (!a) return { badge: 'IDLE', task: row ? `No ${side.toUpperCase()} action in ${row.label}` : 'Standing pose', line: '' };
  if (!a.ready) return { badge: a.skip ? 'IDLE' : 'NOT READY', task: a.display, line: a.why };
  const t = g - a.stage_at;
  let line;
  let badge;
  if (g < a.stage_at) { badge = 'WAITING'; line = 'Waiting for the other arm'; }
  else if (t < a.t_pole) { badge = 'STAGING'; line = `Pole → ${a.stage.pole_mm} mm`; }
  else if (t < a.t_pole + a.t_hand) { badge = 'STAGING'; line = `Hand → ${a.stage.hand_text}`; }
  else if (t < a.traj_t0) { badge = 'STAGING'; line = 'Moving to the first line'; }
  else if (g < a.traj_at) { badge = 'WAITING'; line = `At its start, waiting (${row.rule})`; }
  else if (g < a.end_at) { badge = 'RUNNING'; line = `${fmtS(g - a.traj_at)} of ${fmtS(a.traj_s)} · ${a.file}`; }
  else { badge = 'DONE'; line = `Done · ${a.file}`; }
  return { badge, task: a.display, line };
}

function paintPanels(g) {
  const row = rowAt(g);
  const cur = $('fullSimCurrentStep');
  if (cur) cur.textContent = row ? `${row.index + 1} · ${row.label} · ${row.rule}` : 'Ready';
  for (const side of ARMS) {
    const s = armState(side, g);
    const L = side === 'left' ? 'Left' : 'Right';
    if ($(`fullSim${L}Task`)) $(`fullSim${L}Task`).textContent = s.task;
    if ($(`fullSim${L}Program`)) $(`fullSim${L}Program`).textContent = s.line;
    window.setFullSimStateBadge(side, s.badge);
  }
}

function syncSlider() {
  const sl = seq.ui && seq.ui.slider;
  if (!sl || document.activeElement === sl) return;
  sl.max = String(Math.round((seq.plan ? seq.plan.total_s : 0) * 1000));
  sl.value = String(Math.round(fullSimOverlayElapsed * 1000));
}

// ── the simulation, in the page (run.js puts the modal's view and controls in its middle section)

export function setSource(v) { seq.source = v; if (seq.ui) { window.fullSequenceSimulationControl('reset', false); loadPlan(); } }
export const SOURCE_CHOICES = SOURCES;

function build() {
  const root = $('liteRunSim');
  const host = root && root.querySelector('.full-sim-scene');
  if (!host) return null;
  host.classList.add('lite-3d', 'lite-seq-scene');
  root.classList.add('lite-seq');                           // rows instead of overlays (lite.css)
  const status = el('div', { class: 'lite-seq-status', role: 'status' });
  const slider = el('input', { type: 'range', min: '0', max: '0', step: '5', value: '0', 'aria-label': 'Sequence time',
    oninput: (e) => {
      seq.clean = false;                                    // a run that skipped part of the sequence never approves it
      if (fullSimOverlayRunning) window.fullSequenceSimulationControl('pause', false);
      fullSimOverlayElapsed = Number(e.target.value) / 1000;
      if ($('fullSimOverlayTimer')) $('fullSimOverlayTimer').textContent = fmt(fullSimOverlayElapsed);
      show(fullSimOverlayElapsed).then(() => paintPanels(fullSimOverlayElapsed));
    } });
  const controls = root.querySelector('.full-sim-overlay-controls');
  controls.prepend(status);
  controls.append(el('div', { class: 'lite-seq-slider' }, slider));
  controls.querySelectorAll('button[onclick*="exit"]').forEach((b) => { b.hidden = true; });
  const plan = el('details', { class: 'lite-seq-plan' }, el('summary', { text: 'Sequence' }), el('div', { class: 'lite-seq-rows' }));
  root.querySelector('.full-sim-viewport').append(plan);
  return { host, status, slider, plan };
}

// The 3D in the page's simulation section, and what is chosen loaded into it.
export async function openRun() {
  if (!$('liteRunSim')) return;
  if (!seq.ui) seq.ui = build();
  if (!seq.ui) return;
  release();                                                // the region's simulation stops; it reloads after this
  try {
    const { viewer, robot } = await ensure3D();
    await viewer.attach(seq.ui.host);
    seq.ui.host.dataset.lite3d = 'ready';
    seq.phase = null;
    seq.sceneKey = '';
    robot.home();
    seq.shown = { left: null, right: null };
  } catch (err) {
    if (err.code === 'webgl') say3d(seq.ui.host, say(err), err);       // ALIX Studio's notice
    else tell(err);
  }
  await loadPlan();
}

export function installSequence() {
  const pageRender = window.renderSequenceBuilder;
  window.renderSequenceBuilder = function renderSequenceBuilder(...args) {
    const out = pageRender.apply(this, args);
    decorateFlow();
    return out;
  };

  // the page's Open Full Simulation: the full sequence of both arms, in the page's own view
  window.runFullSequenceSimulation = async function runFullSequenceSimulation() {
    window.auditLog('SIMULATION', 'Full sequence simulation requested', 'Complete LEFT / RIGHT flow simulation requested', 'REQUESTED');
    document.dispatchEvent(new CustomEvent('lite:run-mode', { detail: 'both' }));
    seq.req = { mode: 'both' };
    await openRun();
    $('liteRunSim')?.scrollIntoView({ behavior: 'smooth', block: 'start' });
  };

  const pageDuration = window.fullSequenceSimulationDuration;
  window.fullSequenceSimulationDuration = function fullSequenceSimulationDuration() {
    return seq.plan ? Math.max(0.005, seq.plan.total_s) : pageDuration();
  };

  window.updateFullSimArmPanels = function updateFullSimArmPanels() { paintPanels(fullSimOverlayElapsed || 0); };

  const pageControl = window.fullSequenceSimulationControl;
  let waiting = false;
  window.updateFullSequenceOverlayFrame = async function updateFullSequenceOverlayFrame() {
    if (!fullSimOverlayRunning || dualSafetyFaultLatched || !seq.plan || waiting) return;
    const total = seq.plan.total_s;
    const g = Math.min(total, ((performance.now() - fullSimOverlayStart) / 1000) * (fullSimSpeedPct / 100));
    waiting = true;
    let ready;
    try { ready = await show(g); } catch (err) { ready = err instanceof Error ? err : new Error(String(err)); } finally { waiting = false; }
    if (!fullSimOverlayRunning) return;
    if (ready instanceof Error) {
      seq.clean = false;
      pageControl('stop', false);
      setStatus(`Stopped: ${say(ready)}`, 'warn');
      return;
    }
    if (!ready) {                                         // hold the clock until the step's frames arrive
      fullSimOverlayStart = performance.now() - (fullSimOverlayElapsed / (fullSimSpeedPct / 100)) * 1000;
      setStatus('Loading the next step…');
      fullSimOverlayRaf = requestAnimationFrame(window.updateFullSequenceOverlayFrame);
      return;
    }
    fullSimOverlayElapsed = g;
    if ($('fullSimOverlayTimer')) $('fullSimOverlayTimer').textContent = fmt(g);
    paintPanels(g);
    syncSlider();
    if (g >= total) { finish(); return; }
    const row = rowAt(g);
    setStatus(row ? `Running ${row.index + 1} of ${seq.plan.rows.length}: ${row.label} (${row.rule})` : 'Running…', 'ok');
    fullSimOverlayRaf = requestAnimationFrame(window.updateFullSequenceOverlayFrame);
  };

  function finish() {
    fullSimOverlayRunning = false;
    fullSimOverlayPaused = false;
    const approve = seq.req.mode === 'both' && seq.source === 'version' && seq.plan.ready && seq.clean;
    fullSimCompleted = approve;
    const c = $('fullSimulationApproved');
    if (c) c.disabled = !approve;
    if ($('fullSimCurrentStep')) $('fullSimCurrentStep').textContent = 'COMPLETE ✓';
    window.setFullSimStateBadge('left', 'COMPLETE');
    window.setFullSimStateBadge('right', 'COMPLETE');
    const why = seq.req.mode !== 'both' ? 'Done. Only the full sequence of both arms, run from the start, approves the sequence.'
      : seq.source !== 'version' ? 'Preview complete. The full simulation that approves the sequence runs the kept versions.'
        : 'Run complete, but not played from the start without skipping: press ↺ Reset, then ▶ Start to approve the sequence.';
    setStatus(approve ? 'Full sequence simulation complete ✓ · Review the result and confirm below.' : why, approve ? 'ok' : 'warn');
    window.updateSequenceGate();
    window.toast(approve ? 'Full sequence simulation complete.' : 'This run does not approve the sequence.');
  }

  window.fullSequenceSimulationControl = function fullSequenceSimulationControl(action, announce = true) {
    if (action === 'exit') return;                         // the view is in the page now
    if (action === 'start' && seq.req.mode === 'both' && !$('sequenceVisualConfirmed')?.checked) {
      window.toast('Confirm the sequence order first (Visual Confirmation, above).');
      return;
    }
    if (action === 'start' && !canRun()) {
      if (seq.ui && seq.ui.plan) seq.ui.plan.open = true;
      window.toast(seq.plan ? 'Not ready: the steps marked ✕ need a kept version (or exclude their rows).' : 'The sequence is still loading.');
      return;
    }
    // a run from 0 (or again after the end) can approve; a resumed run keeps what it was
    if (action === 'start' && !fullSimOverlayRunning && (fullSimOverlayElapsed <= 0 || fullSimOverlayElapsed >= seq.plan.total_s)) seq.clean = true;
    if (action === 'start' && seq.req.mode !== 'both') {  // one arm, files, points: no two-arm order to confirm first
      if (dualSafetyFaultLatched) { window.toast('Safety fault is latched. Press Reset before restarting.'); return; }
      if (fullSimOverlayRunning) return;
      window.fullSimSpeedChanged();
      if (fullSimOverlayElapsed >= seq.plan.total_s) fullSimOverlayElapsed = 0;
      fullSimOverlayRunning = true;
      fullSimOverlayPaused = false;
      fullSimOverlayStart = performance.now() - (fullSimOverlayElapsed / (fullSimSpeedPct / 100)) * 1000;
      setStatus('Running…', 'ok');
      if (fullSimOverlayRaf) cancelAnimationFrame(fullSimOverlayRaf);
      fullSimOverlayRaf = requestAnimationFrame(window.updateFullSequenceOverlayFrame);
      return;
    }
    pageControl(action, announce);
    $('fullSequenceSimModal')?.classList.remove('show');   // the page shows its window; the view is in the page now
    if (action === 'reset') {
      fullSimOverlayElapsed = 0;
      syncSlider();
      if (seq.plan) { show(0).then(() => paintPanels(0)); showPlan(seq.plan); }
    }
  };
}
