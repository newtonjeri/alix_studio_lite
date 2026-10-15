// Merge + Full Sequence: what to run, the simulation and the real machine, top to bottom (feedback
// v2, slides 11 and 12; the owner, 15 Oct). Under the flow table and its Visual Confirmation:
// - at the top, what to run and its files: the full sequence of both arms, one arm's full sequence
//   alone, single files of the project (kept versions, the transitions, recordings in 3D only), the
//   controller's global points, or a program stored on a controller;
// - in the middle, the simulation at the page's full width (the page's full-sequence view, moved
//   out of its window; sequence.js plays what is chosen in it);
// - at the bottom, the real machine: the page's two-arm card, and the run of what is chosen on its
//   arm alone (the other arm stays stopped; the server refuses otherwise), in Hardware sim or Real.
// The robot runs go through the routes every robot step uses, so the gates hold: the arm a file was
// taught on, its pole and hand staged first, a kept version or a transition within the limits (a
// recording as taught never runs on the real arms), one confirmation for a run on the real robot.

import { get, say } from './api.js';
import { REAL_CHECKS, confirmDialog, ctx, el, jobLine, robotCall, stageDialog, tell, unitUrl, watchJob } from './ctx.js';
import { POINT_SPEED, hasController, loadPoints, movePoint } from './points.js';
import { SOURCE_CHOICES, canRun, openRun, planOf, seq, setRequest, setSource } from './sequence.js';
import { liteMode } from './status.js';

const $ = (id) => document.getElementById(id);
const ARMS = ['left', 'right'];
const MODES = [
  ['both', 'Full sequence · both arms'], ['left', 'LEFT arm · full sequence alone'], ['right', 'RIGHT arm · full sequence alone'],
  ['file', 'Single trajectory files'], ['points', 'Global points'], ['program', 'A program on the controller'],
];
const pick = { mode: 'both', side: 'left', files: [], points: [] };
let flow = null;                                  // the project's files (flow-files), read when the page opens
let programs = { left: [], right: [] };
let running = null;                               // {side, stop}: a robot run in progress

const ARM = (s) => s.toUpperCase();

// ── the top: what to run and its files

function files(side) {
  const seen = new Set(), out = [];
  for (const t of (flow && flow.tasks) || []) {
    const a = t[side];
    if (!a || a.later || seen.has(a.key)) continue;
    seen.add(a.key);
    const opts = a.versions.map((v) => ({ value: `${a.key}|version:${v.version}`, text: `Kept ${v.version} · ${v.name}${v.version === a.version ? ' (runs)' : ''}${v.uploadable ? '' : ' · fails a limit'}`, sha: v.sha256 }));
    if (a.transition) opts.push({ value: `${a.key}|transition`, text: `Transition · ${a.transition.name}${a.transition.limits_ok ? '' : ' · fails a limit'}`, sha: a.transition.sha256 });
    if (a.recording) opts.push({ value: `${a.key}|raw`, text: `Recording as taught · ${a.recording} (3D only)` });
    if (opts.length) out.push({ key: a.key, display: `${t.label} · ${a.display}`, opts });
  }
  return out;
}

function chips(list, onRemove) {
  return el('div', { class: 'lite-run-chips' }, list.length
    ? list.map((x, i) => el('span', { class: 'lite-chip' }, `${i + 1}. ${x.label}`,
      el('button', { type: 'button', class: 'lite-chip-x', 'aria-label': `Remove ${x.label}`, text: '✕', onclick: () => onRemove(i) })))
    : el('span', { class: 'mini-note', text: 'Nothing added yet.' }));
}

function request() {
  const m = pick.mode;
  if (['both', 'left', 'right'].includes(m)) return { mode: m };
  if (m === 'file') return { mode: 'file', side: pick.side, items: pick.files.map((f) => ({ key: f.key, which: f.which })) };
  if (m === 'points') return { mode: 'points', side: pick.side, points: pick.points };
  const prog = programs[pick.side].find((p) => String(p.id) === String(pick.program));
  if (!prog) return { mode: 'program', side: pick.side, items: [], why: 'Choose a program on the controller.' };
  const hit = programFile(pick.side, prog);
  return hit ? { mode: 'program', side: pick.side, items: [hit] }
    : { mode: 'program', side: pick.side, items: [], why: `Lite did not send program ${prog.id} (“${prog.name}”), so it cannot show it in 3D. It can still run it on the robot below.` };
}

// The project's file a program holds, by its sha256 (a program Lite sent).
function programFile(side, prog) {
  if (!prog.sha256) return null;
  for (const t of (flow && flow.tasks) || []) {
    const a = t[side];
    if (!a) continue;
    const v = a.versions.find((x) => x.sha256 === prog.sha256);
    if (v) return { key: a.key, which: `version:${v.version}`, label: `${a.display} ${v.version}` };
    if (a.transition && a.transition.sha256 === prog.sha256) return { key: a.key, which: 'transition', label: `${a.display} transition` };
  }
  return null;
}

function paintTop() {
  const box = $('liteRunPick');
  if (!box) return;
  const m = pick.mode, side = pick.side;
  const parts = [];
  if (['both', 'left', 'right'].includes(m)) {
    parts.push(el('label', { class: 'field' }, 'Files ', el('select', { 'aria-label': 'Files', onchange: (e) => setSource(e.target.value) },
      SOURCE_CHOICES.map(([v, n]) => el('option', { value: v, text: n, selected: v === seq.source })))));
    parts.push(el('div', { class: 'mini-note', text: m === 'both'
      ? 'The flow table’s included rows, each arm step’s chosen kept version (above), with each row’s rule and checkpoint wait.'
      : `The flow table’s included rows, the ${ARM(m)} arm’s steps only, one after another; the other arm stands still.` }));
  } else {
    parts.push(el('label', { class: 'field' }, 'Arm ', el('select', { 'aria-label': 'Arm', onchange: (e) => { pick.side = e.target.value; pick.files = []; pick.points = []; pick.program = null; changed(); } },
      ARMS.map((s) => el('option', { value: s, text: `${ARM(s)} arm`, selected: s === side })))));
  }
  if (m === 'file') {
    const groups = files(side);
    const sel = el('select', { 'aria-label': 'File', class: 'lite-run-file' }, groups.length
      ? groups.map((g) => el('optgroup', { label: g.display }, g.opts.map((o) => el('option', { value: o.value, text: o.text }))))
      : el('option', { value: '', text: `No ${ARM(side)}-arm files in this project yet` }));
    parts.push(el('label', { class: 'field lite-run-grow' }, 'File ', sel),
      el('button', { class: 'btn light', type: 'button', text: '＋ Add', disabled: !groups.length, onclick: () => {
        const [key, which] = sel.value.split('|');
        const o = groups.flatMap((g) => g.opts.map((x) => ({ ...x, g }))).find((x) => x.value === sel.value);
        pick.files.push({ key, which, label: `${o.g.display} · ${o.text.split(' · ')[0]}` });
        apply();
      } }),
      chips(pick.files, (i) => { pick.files.splice(i, 1); apply(); }));
  }
  if (m === 'points') {
    const sel = el('select', { 'aria-label': 'Global point', class: 'lite-run-file' }, el('option', { value: '', text: 'Reading the controller…' }));
    const motion = el('select', { 'aria-label': 'Motion' }, el('option', { value: 'movej', text: 'MoveJ' }), el('option', { value: 'movel', text: 'MoveL' }));
    if (!hasController(side)) sel.replaceChildren(el('option', { value: '', text: `Connect the ${ARM(side)} arm with the live link to read its global points` }));
    else loadPoints(side).then((d) => sel.replaceChildren(...(d.points.length ? d.points.map((p) => el('option', { value: p.name, text: p.name }))
      : [el('option', { value: '', text: d.note || 'No global points on this controller' })])), (err) => sel.replaceChildren(el('option', { value: '', text: say(err) })));
    parts.push(el('label', { class: 'field lite-run-grow' }, 'Global point ', sel), el('label', { class: 'field' }, 'Motion ', motion),
      el('button', { class: 'btn light', type: 'button', text: '＋ Add', onclick: () => {
        if (!sel.value) return;
        pick.points.push({ name: sel.value, move: motion.value, label: `${sel.value} (${motion.value === 'movel' ? 'MoveL' : 'MoveJ'})` });
        apply();
      } }),
      chips(pick.points, (i) => { pick.points.splice(i, 1); apply(); }),
      el('div', { class: 'mini-note', text: `In order, at ${POINT_SPEED} % (the control program’s speed for a global point).` }));
  }
  if (m === 'program') {
    const list = programs[side].filter((p) => p.name);
    const sel = el('select', { 'aria-label': 'Program', class: 'lite-run-file', onchange: (e) => { pick.program = e.target.value; apply(); } },
      el('option', { value: '', text: list.length ? 'Choose a program…' : `No programs read from the ${ARM(side)} controller yet` }),
      list.map((p) => el('option', { value: String(p.id), selected: String(p.id) === String(pick.program),
        text: `${p.id} · ${p.name}${p.sha256 ? ' · sent by Lite' : ' · not sent by Lite'}` })));
    parts.push(el('label', { class: 'field lite-run-grow' }, 'Program ', sel),
      el('button', { class: 'btn light', type: 'button', text: '↻ Read the programs', onclick: () => readPrograms(side) }));
  }
  box.replaceChildren(...parts);
}

async function readPrograms(side) {
  try { programs[side] = (await get(`/api/robots/${side}/programs`)).programs || []; } catch (err) { tell(err); }
  paintTop();
}

function apply() {
  paintTop();
  paintReal();
  setRequest(request());
}

// A new mode or arm: shown and played at once; a controller's programs are read for it.
function changed() {
  apply();
  if (pick.mode === 'program') readPrograms(pick.side);
}

// The project's files, read again (when the page opens): the pickers follow, the plan is not reloaded.
async function refreshFlow() {
  if (!ctx.pid) return;
  try { flow = await get(`/api/projects/${ctx.pid}/flow-files`); } catch (_) { /* the picker says there are no files */ }
  paintTop();
  paintReal();
}

// ── the bottom: the real machine

function paintReal() {
  const box = $('liteRunRealControls');
  if (!box) return;
  const m = pick.mode;
  const pageCard = document.querySelector('#liteRunReal .real-full-run');
  if (pageCard) pageCard.hidden = m !== 'both';
  if (m === 'both') {
    box.replaceChildren(el('div', { class: 'mini-note', text: 'The full sequence of both arms on the robot: the page’s card below, once the full simulation was run and confirmed.' }));
    return;
  }
  const what = m === 'left' || m === 'right' ? `the ${ARM(m)} arm’s full sequence` : m === 'file' ? 'the files above, one after another'
    : m === 'points' ? 'the global points above, in order' : 'the program chosen above';
  const side = ['left', 'right'].includes(m) ? m : pick.side;
  box.replaceChildren(
    el('p', { class: 'mini-note', text: `Runs ${what} on the ${ARM(side)} arm alone, in Hardware sim or Real (asked first): the other arm stays stopped while it runs. `
      + (m === 'file' ? 'Each file is sent to the Program ID below, then run, its limits checked first.' : m === 'left' || m === 'right'
        ? 'Each step is sent to its Program ID in the flow table (or the one its Real Test used), then run, staged first.' : '') }),
    el('div', { class: 'cta-row' },
      m === 'file' ? el('input', { id: 'liteRunProgram', type: 'number', min: '1', max: '100', step: '1', placeholder: 'Program ID', 'aria-label': 'Program ID for the files' }) : null,
      el('button', { class: 'btn red', id: 'liteRunRealBtn', type: 'button', text: `▶ Run on the ${ARM(side)} arm`, onclick: () => runReal(side) }),
      el('button', { class: 'btn outline', id: 'liteRunStopBtn', type: 'button', text: '■ Stop', onclick: stopReal })),
    el('div', { id: 'liteRunRealStatus', class: 'statusbar', text: 'Not running.' }));
}

const realStatus = (text, cls = '') => { const s = $('liteRunRealStatus'); if (s) { s.textContent = text; s.className = `statusbar ${cls}`.trim(); } };
const pause = (s) => new Promise((r) => setTimeout(r, s * 1000));

async function watchOrThrow(job, label) {
  if (!job || !job.id) return job;
  const done = await watchJob(job, (j) => realStatus(`${label}: ${jobLine(j)}`));
  if (done.state !== 'done') throw Object.assign(new Error(done.error || `${label} did not finish.`), { hint: done.hint });
  return done;
}

async function runReal(side) {
  if (running) { window.toast('A run is in progress: wait, or press Stop.'); return; }
  const mode = await liteMode();
  if (mode === 'visual') { tell(Object.assign(new Error('Lite is in Visual sim, so it does not use the robot.'), { code: 'mode', hint: 'Choose Hardware sim or Real robot.' })); return; }
  const m = pick.mode;
  let extra = {};
  if (mode === 'real') {
    const yes = await confirmDialog({ title: `Real robot: the ${ARM(side)} arm will move`, ok: `Yes, run it on the ${ARM(side)} arm`, list: REAL_CHECKS,
      body: `The ${ARM(side)} arm runs ${m === 'file' ? 'the files chosen' : m === 'points' ? 'through the global points chosen' : m === 'program' ? 'the program chosen' : 'its full sequence'} on the real robot. The other arm stays stopped.` });
    if (!yes) { window.toast('Cancelled: nothing was sent to the robot.'); return; }
    extra = { confirmed: true };
  }
  running = { side, stop: false };
  try {
    if (m === 'left' || m === 'right') await runArmSequence(side, extra);
    else if (m === 'file') await runFiles(side, extra);
    else if (m === 'points') await runPoints(side, extra);
    else await runProgram(side, extra);
    if (!running.stop) { realStatus(`Done on the ${ARM(side)} arm${mode === 'real' ? '' : ' (Hardware sim: nothing moved)'}.`, 'ok'); window.toast(`Done on the ${ARM(side)} arm.`); }
  } catch (err) {
    realStatus(`Stopped: ${say(err)}`, 'warn');
    tell(err);
  } finally { running = null; }
}

async function runArmSequence(side, extra) {
  const plan = planOf();
  if (!plan || !canRun() || plan.only !== side) throw new Error(`Load the ${ARM(side)} arm's full sequence in the simulation first: every step needs a kept version.`);
  const steps = window.fullSequenceIncludedSteps();
  const rows = plan.rows.filter((r) => r.arms[side] && r.arms[side].ready);
  for (const [n, r] of rows.entries()) {
    if (running.stop) return;
    const a = r.arms[side];
    const given = Number(steps[r.index] && steps[r.index][`${side}ProgramId`]) || null;
    const label = `Step ${n + 1} of ${rows.length} · ${r.label} · ${a.display}`;
    let up = given ? await robotCall(unitUrl(a.key, 'upload'), { program_id: given }, { what: `Sending ${a.display}` }) : null;
    if (!given) {
      const u = await get(unitUrl(a.key));
      const pid = u.settings_view.settings.program_id;
      if (!pid) throw new Error(`Give ${r.label} a ${ARM(side)} Program ID in the flow table (or upload it in its Real Test).`);
      up = await robotCall(unitUrl(a.key, 'upload'), { program_id: pid }, { what: `Sending ${a.display}` });
    }
    await watchOrThrow(up, `${label} · sending`);
    if (running.stop) return;
    await watchOrThrow(await robotCall(unitUrl(a.key, 'run'), { alone: true, ...extra }, { what: `Running ${a.display}` }), `${label} · running`);
    if (r.wait_s && !running.stop) { realStatus(`${label} · checkpoint wait ${r.wait_s} s`); await pause(r.wait_s); }
  }
}

async function runFiles(side, extra) {
  const pid = Number($('liteRunProgram')?.value);
  if (!(pid >= 1 && pid <= 100)) { $('liteRunProgram')?.focus(); throw new Error('Enter the Program ID the files are sent to (1 to 100).'); }
  if (!pick.files.length) throw new Error('Add the files to run first.');
  for (const [n, f] of pick.files.entries()) {
    if (running.stop) return;
    const label = `File ${n + 1} of ${pick.files.length} · ${f.label}`;
    await watchOrThrow(await robotCall(unitUrl(f.key, 'file/upload'), { which: f.which, program_id: pid }, { what: `Sending ${f.label}` }), `${label} · sending`);
    if (running.stop) return;
    await watchOrThrow(await robotCall(unitUrl(f.key, 'file/run'), { which: f.which, program_id: pid, ...extra }, { what: `Running ${f.label}` }), `${label} · running`);
  }
}

async function runPoints(side, extra) {
  if (!pick.points.length) throw new Error('Add the global points to run first.');
  for (const [n, p] of pick.points.entries()) {
    if (running.stop) return;
    const label = `Point ${n + 1} of ${pick.points.length} · ${p.label}`;
    const done = await movePoint(side, p.name, p.move, POINT_SPEED, 0, extra, (j) => realStatus(`${label}: ${jobLine(j)}`));
    if (done.state !== 'done') throw Object.assign(new Error(done.error || `${label} did not finish.`), { hint: done.hint });
  }
}

async function runProgram(side, extra) {
  const pid = Number(pick.program);
  if (!pid) throw new Error('Choose a program on the controller first.');
  const url = `/api/robots/${side}/programs/run`;
  let job;
  try {
    job = await robotCall(url, { program_id: pid, ...extra }, { what: `Running program ${pid}` });
  } catch (err) {
    if (err.code !== 'stage') throw err;
    const said = await stageDialog({ title: `Program ${pid}: its pole and hand`, arm: side,
      body: `${err.message} ${err.hint || ''} Both are staged first, read back in Real.` });
    if (!said) throw Object.assign(new Error('Cancelled: nothing was sent to the robot.'), { code: 'cancelled' });
    job = await robotCall(url, { program_id: pid, pole_mm: said.pole_mm, hand: said.hand, ...extra }, { what: `Running program ${pid}` });
  }
  await watchOrThrow(job, `Program ${pid}`);
}

async function stopReal() {
  const side = running ? running.side : (['left', 'right'].includes(pick.mode) ? pick.mode : pick.side);
  if (running) running.stop = true;
  try { await robotCall(`/api/robots/${side}/stop`, {}); } catch (err) { tell(err); }
  realStatus(`STOP sent to the ${ARM(side)} arm: it holds until Recover (Robot & mode).`, 'warn');
}

// ── the page's sections, rearranged

function build() {
  if ($('liteRun')) return true;
  const grid = document.querySelector('#view-sequence .full-run-grid');
  const modal = $('fullSequenceSimModal');
  const viewport = modal && modal.querySelector('.full-sim-viewport');
  const controls = modal && modal.querySelector('.full-sim-overlay-controls');
  if (!grid || !viewport || !controls) return false;
  const section = el('div', { id: 'liteRun', class: 'lite-run' },
    el('div', { class: 'lite-run-top card' },
      el('div', { class: 'section-head compact-head' }, el('div', {}, el('h3', { text: 'What to run' }),
        el('p', { text: 'Choose what to simulate and run, and its files. The simulation below plays it; the real machine runs it at the bottom.' }))),
      el('div', { class: 'lite-run-row' },
        el('label', { class: 'field' }, 'Run ', el('select', { id: 'liteRunMode', 'aria-label': 'What to run', onchange: (e) => { pick.mode = e.target.value; changed(); } },
          MODES.map(([v, n]) => el('option', { value: v, text: n })))),
        el('div', { id: 'liteRunPick', class: 'lite-run-pick' }))),
    el('div', { id: 'liteRunSim', class: 'lite-run-sim card' },
      el('div', { class: 'section-head compact-head' }, el('div', {}, el('h3', { text: 'Simulation' }),
        el('p', { text: 'The robot in 3D, each step staged first and played on the 5 ms clock, in the room and at each task’s UGV placement.' })))),
    el('div', { id: 'liteRunReal', class: 'lite-run-real card' },
      el('div', { class: 'section-head compact-head' }, el('div', {}, el('h3', { text: 'Real machine' }),
        el('p', { text: 'On the robot, in Hardware sim (nothing moves) or Real. STOP BOTH at the top always works.' }))),
      el('div', { id: 'liteRunRealControls' })));
  grid.before(section);
  $('liteRunSim').append(viewport);
  if (!viewport.contains(controls)) viewport.append(controls);
  const real = grid.querySelector('.real-full-run');
  if (real) $('liteRunReal').append(real);
  grid.hidden = true;                             // the page's simulation card: its view is in the page now
  return true;
}

export function installRun() {
  const pageShow = window.showView;
  window.showView = function showView(v, btn) {
    pageShow(v, btn);
    if (v === 'sequence' && build()) { paintTop(); paintReal(); refreshFlow().finally(openRun); }   // the files first: the 3D's requests are slow
  };
  document.addEventListener('lite:run-mode', (e) => { pick.mode = e.detail; if ($('liteRunMode')) $('liteRunMode').value = e.detail; paintTop(); paintReal(); });
  document.addEventListener('lite:run-plan', paintReal);
  document.addEventListener('lite:points', () => { if (pick.mode === 'points') paintTop(); });
}
