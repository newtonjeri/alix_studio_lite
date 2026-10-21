// The Trajectory Viewer: taught .txt files, uploaded from this computer (files or a whole folder)
// or from the built-in sets, replayed in 3D the way the robot executes a file: the pole to the
// file's stage height, the hand to its pose, the arm to the first line, then the trajectory on the
// 5 ms clock (a file of N lines lasts (N - 1) x 5 ms). A file replays only on the arm it was taught
// on and with its stage (pole height, hand pose) known: the server refuses anything else. A LEFT
// and a RIGHT file stage together; the arm ready first holds at its start, and both trajectories
// start on the same tick. The joints run on a plot with a playhead, and every limit is checked on
// each file as taught. A file can become the active recording of a region of its own arm only.
// The programs on the arms' controllers are listed too, found by a search (the owner, 21 Oct: "The
// tab can search the uploaded tasks"): one Lite sent there replays like a file, with its own pole
// and hand (Lite's record of what it sent), and any of them runs from its slot on the robot ("run a
// saved project, from the slot number where it is stored, on the real hardware"), its own pole and
// hand staged first (advanced.js runSlot: Advanced's Run Stored). Nothing else here is sent.

import { runSlot } from './advanced.js';
import { del, get, post, say } from './api.js';
import { ctx, el, fitPlot, fmtS, stageDialog, step, tell } from './ctx.js';
import { ensure3D, release, say3d, scene } from './sim.js';
import { liteMode } from './status.js';
import { checkJob, spansOf, Tools } from './view3d.js';
import { frame } from './vp.js';

const CYCLE = 0.005;
const ARMS = ['left', 'right'];
const slots = { left: null, right: null };          // {meta, r (timeline as sent), tl, track, checks}
const asked = { left: 0, right: 0 };                 // the newest ▶ View per arm: an older answer is dropped
let box = null, lib = null, tools = null;
const phase = () => (tools && tools.phase) || 'lid_closed';       // the commode as the view's tools show it
let t = 0, playing = false, speed = 1, raf = 0, last = 0, said = '';   // t: the run clock, 0 = both arms standing
let plotP = null;
let progs = null, progSearch = null, progBox = null;   // the controllers' programs ({arms: {left, right}}) and their search
const running = { left: false, right: false };         // a program run from its slot, per arm

const $ = (id) => document.getElementById(id);
const loaded = () => ARMS.filter(a => slots[a]);
const trajS = (a) => (slots[a] ? (slots[a].tl.lines - 1) * CYCLE : 0);
const start = () => Math.max(0, ...loaded().map(a => slots[a].tl.traj_t0));     // both trajectories start here
const total = () => (loaded().length ? start() + Math.max(...loaded().map(trajS)) : 0);
const fmtT = (s) => { const m = Math.floor(s / 60); return `${String(m).padStart(2, '0')}:${(s - m * 60).toFixed(1).padStart(4, '0')}`; };
const ARM_BADGE = { left: 'badge blue', right: 'badge orange' };
const pidQ = () => (ctx.pid ? `?pid=${encodeURIComponent(ctx.pid)}` : '');
const isProgram = (f) => f && f.source === 'program';
const urlOf = (f) => (isProgram(f) ? `/api/library/programs/${f.arm}/${f.program}` : `/api/library/${f.id}`);
const titleOf = (f) => (isProgram(f) ? `Program ${f.program} · ${f.name}` : f.name);

// Arm a's own timeline at run time t: it stages from 0, holds at its start, starts with the other arm.
function armTime(a, at) {
  const t0 = slots[a].tl.traj_t0, T = start();
  return at < t0 ? at : at < T ? t0 : t0 + (at - T);
}

function loadPlotly() {
  if (!plotP) {
    plotP = window.Plotly ? Promise.resolve() : new Promise((res, rej) => {
      const s = document.createElement('script');
      s.src = '/vendor/plotly.min.js'; s.onload = res; s.onerror = () => rej(new Error('could not load the plot library'));
      document.head.append(s);
    });
  }
  return plotP;
}

function status(text, cls = '') {
  const e = $('liteViewerStatus');
  if (e) { e.textContent = text; e.className = `statusbar ${cls}`.trim(); }
}

// ── the file list

function fileRow(f) {
  const arm = f.arm
    ? el('span', { class: ARM_BADGE[f.arm], text: f.arm.toUpperCase() })
    : el('span', { class: 'lite-arm-pick' }, el('small', { text: 'Arm?' }),
      ...ARMS.map(a => el('button', { class: 'btn light', text: a.toUpperCase(), onclick: () => setArm(f, a) })));
  const stage = f.stage
    ? el('small', { class: 'lite-file-stage ok', text: `✓ ${f.stage.text}` })
    : el('small', { class: 'lite-file-stage bad', text: '✕ Pole height and hand pose not known: it cannot be viewed' });
  const own = f.source === 'upload';
  return el('div', { class: `lite-file-row${ARMS.some(a => slots[a] && slots[a].meta.id === f.id) ? ' on' : ''}${f.ready ? '' : ' todo'}` },
    arm,
    el('div', { class: 'lite-file-name' }, el('b', { text: f.name, title: f.folder ? `${f.folder}/${f.name}` : f.name }),
      el('small', { text: [fmtS(f.seconds), `${Number(f.lines).toLocaleString()} rows`, f.task || f.folder || ''].filter(Boolean).join(' · ') }),
      stage),
    el('div', { class: 'lite-file-actions' },
      f.ready ? el('button', { class: 'btn green', text: '▶ View', onclick: () => view(f) })
        : own ? el('button', { class: 'btn orange', text: 'Pole & hand…', onclick: () => setStage(f) }) : null,
      own && f.ready ? el('button', { class: 'btn light', text: '✎', title: 'Change the pole height and hand pose',
        'aria-label': `Change the pole and hand of ${f.name}`, onclick: () => setStage(f) }) : null,
      own ? el('button', { class: 'btn light', text: '✕', 'aria-label': `Remove ${f.name}`, onclick: () => remove(f) }) : null));
}

function renderList() {
  const box2 = $('liteViewerFiles');
  if (!box2 || !lib) return;
  const mine = el('div', { class: 'lite-file-group' }, el('h4', { text: `Your files (${lib.files.length})` }),
    lib.files.length ? lib.files.map(fileRow)
      : el('p', { class: 'mini-note', text: 'Upload taught .txt files above. They are kept on this computer for every project.' }));
  const sets = lib.sets.map(s => el('details', { class: 'lite-file-group' },
    el('summary', {}, el('b', { text: s.name }), el('small', { text: ` · ${s.files.length} files` })),
    el('p', { class: 'mini-note', text: s.note }), s.files.map(fileRow)));
  box2.replaceChildren(programsBox(), mine, ...sets);
  renderPrograms();
}

async function refreshList() {
  try { lib = await get('/api/library'); renderList(); } catch (err) { tell(err); }
}

// ── the programs on the controllers: found by a search, replayed like a file, run from their slots

const words = (q) => String(q || '').toLowerCase().split(/\s+/).filter(Boolean);
const programOf = (side, p) => ({ id: `p_${side}_${p.id}`, source: 'program', program: p.id, name: p.name || `program ${p.id}`,
  arm: side, stage: p.stage, ready: p.viewable, seconds: p.seconds, lines: p.lines });

async function refreshPrograms(again = false) {
  const note = $('liteViewerProgNote');
  if (note && again) note.textContent = 'Reading the controllers…';
  try {
    progs = await get(`/api/library/programs${pidQ()}${again ? `${pidQ() ? '&' : '?'}refresh=1` : ''}`);
  } catch (err) {
    progs = null;
    if (note) note.textContent = `The controllers' programs are not listed: ${say(err)}`;
    return;
  }
  renderPrograms();
}

// The group, made once: its search field keeps what is typed while the rows change.
function programsBox() {
  if (progBox) return progBox;
  progSearch = el('input', { type: 'search', id: 'liteViewerProgSearch', class: 'lite-picker-search lite-prog-search',
    placeholder: 'Number, name, LEFT or RIGHT', 'aria-label': 'Search the programs on the controllers: a number, a name, LEFT or RIGHT',
    autocomplete: 'off', oninput: () => renderPrograms() });
  progBox = el('details', { class: 'lite-file-group lite-prog-group', open: true },
    el('summary', {}, el('b', { text: 'On the controllers' }), el('small', { id: 'liteViewerProgCount', text: '' })),
    el('div', { class: 'lite-prog-tools' }, progSearch,
      el('button', { class: 'btn light', text: 'Read again', title: "Read the arms' controllers again", onclick: () => refreshPrograms(true) })),
    el('p', { id: 'liteViewerProgNote', class: 'mini-note' }),
    el('div', { id: 'liteViewerProgRows' }));
  return progBox;
}

function renderPrograms() {
  const rows = $('liteViewerProgRows'), note = $('liteViewerProgNote'), count = $('liteViewerProgCount');
  if (!rows) return;
  if (!progs) { rows.replaceChildren(); return; }
  const all = ARMS.flatMap(a => (progs.arms[a]?.programs || []).map(p => [a, p]));
  const ws = words(progSearch?.value);
  const hit = all.filter(([a, p]) => {
    const t = `${p.id} ${p.name || ''} ${a} ${p.file_name || ''} ${p.stage ? p.stage.text : ''}`.toLowerCase();
    return ws.every(w => t.includes(w));
  });
  if (count) count.textContent = ` · ${ws.length ? `${hit.length} of ${all.length}` : all.length} program${all.length === 1 ? '' : 's'}`;
  if (note) {
    const read = progs.backend === 'stand-in' ? ['The stand-in robot: the programs Lite sent to it.'] : ARMS.map(a => {
      const x = progs.arms[a];
      return `${a.toUpperCase()}: ${!x.connected ? 'not connected, so the programs Lite sent to it are listed'
        : x.note || (x.read ? `read at ${x.read.slice(11, 16)}` : 'not read yet')}.`;
    });
    note.textContent = `${read.join(' ')} Run sends a program to its arm: its own pole and hand first, then the program from its slot.`;
  }
  rows.replaceChildren(...(hit.length ? hit.map(([a, p]) => programRow(a, p))
    : [el('p', { class: 'mini-note', text: all.length ? 'No program matches the search.' : 'No programs on the controllers.' })]));
}

function programRow(side, p) {
  const f = programOf(side, p);
  const on = slots[side] && slots[side].meta.id === f.id;
  const facts = [p.seconds != null ? fmtS(p.seconds) : null, p.lines ? `${Number(p.lines).toLocaleString()} rows` : null].filter(Boolean);
  return el('div', { class: `lite-file-row lite-prog-row${on ? ' on' : ''}` },
    el('span', { class: ARM_BADGE[side], text: side.toUpperCase() }),
    el('div', { class: 'lite-file-name' }, el('b', { text: `${p.id} · ${p.name || ''}`,
      title: `Program ${p.id} on the ${side.toUpperCase()} controller${p.file_name ? `: Lite's file ${p.file_name}` : ''}` }),
      facts.length ? el('small', { text: facts.join(' · ') }) : null,
      p.stage ? el('small', { class: 'lite-file-stage ok', text: `Its own: ${p.stage.text}` }) : null,
      p.why ? el('small', { class: 'lite-file-stage lite-warn', text: p.why }) : null),
    el('div', { class: 'lite-file-actions' },
      p.viewable ? el('button', { class: 'btn green', text: 'View', 'aria-label': `View program ${p.id} on the ${side} arm`, onclick: () => view(f) }) : null,
      el('button', { class: 'btn', text: running[side] ? 'Running' : 'Run', disabled: running[side],
        title: `Run program ${p.id} from its slot on the ${side.toUpperCase()} arm: its own pole and hand first`,
        'aria-label': `Run program ${p.id} on the ${side} arm`, onclick: () => runProgram(side, p) })));
}

// From its slot (the owner, 21 Oct), as Advanced's Run Stored runs it: its own pole and hand staged first (a
// program Lite did not send asks for them), then the controller's move to its first point, then the program.
// Hardware sim: the controller runs it in its SIM mode; Real robot: the arm moves, confirmed first.
async function runProgram(side, p) {
  const S = side.toUpperCase();
  if (running[side]) return;
  if ((await liteMode()) === 'visual') {
    status(`Visual sim sends nothing to the robot: View shows how program ${p.id} runs. To run it, choose Hardware sim or `
      + 'Real robot in the status pill at the top.', 'warn');
    return;
  }
  running[side] = true;
  renderPrograms(); renderChecks();
  status(`${S} program ${p.id}: the pole and the hand first, then the program from its slot…`);
  try {
    const done = await runSlot(side, String(p.id), null, (text) => status(`${S} program ${p.id}: ${text}`));
    if (!done) status(`${S} program ${p.id} was not run: nothing was sent to the robot.`, 'warn');
    else if (done.state === 'done') status(`${S} program ${p.id} ran from its slot, its pole and hand staged first.`, 'ok');
    else status(`${S} program ${p.id} stopped: ${done.error || done.state}`, 'warn');
  } catch (err) {
    status(`${S} program ${p.id} was not run: ${say(err)}`, 'warn');
    window.toast(say(err), err.code === 'cancelled' ? 'warn' : 'error');
  } finally {
    running[side] = false;
    renderPrograms(); renderChecks();
  }
}

const byId = (id) => lib.files.find(f => f.id === id);

async function upload(fileList) {
  const files = [...fileList].filter(f => /\.txt$/i.test(f.name));
  if (!files.length) { status('Choose .txt trajectory files.', 'warn'); return; }
  const refused = [], added = [];
  for (const [i, f] of files.entries()) {
    status(`Checking ${i + 1} of ${files.length}: ${f.name}…`);
    try {
      lib = await post('/api/library', { name: f.name, content: await f.text(), path: f.webkitRelativePath || f.name });
      added.push(lib.added);
    } catch (err) { refused.push(`${f.name}: ${err.message}`); }
  }
  renderList();
  status(`${added.length} file${added.length === 1 ? '' : 's'} added${refused.length ? `, ${refused.length} refused` : ''}.`
    + (refused.length ? ` ${refused.join(' · ')}` : ''), refused.length ? 'warn' : 'ok');
  const pending = added.map(byId).filter(f => f && !f.stage);
  if (pending.length) await askStage(pending);
  const noArm = lib.files.filter(f => !f.arm).length;
  if (noArm) status(`${noArm} file${noArm === 1 ? '' : 's'} need${noArm === 1 ? 's' : ''} an arm: choose LEFT or RIGHT on ${noArm === 1 ? 'its' : 'their'} row.`, 'warn');
}

// The pole height and hand pose files were taught with: the replay stages them first, as the robot does.
async function askStage(files, current = null) {
  const one = files.length === 1 ? files[0] : null;
  const v = await stageDialog({
    title: one ? `How was ${one.name} taught?` : `How were these ${files.length} files taught?`,
    body: 'Before a file runs, the robot moves the pole to the height it was taught at and the hand to the pose it was '
      + 'taught with. The replay does the same, so say them here.'
      + (one ? '' : ' These values go to every file just added; Cancel to set each one on its own row.'),
    arm: one?.arm || null, askArm: !!one && !one.arm, setup: current });
  if (!v) {
    status('A file without its pole height and hand pose cannot be viewed: set them on its row.', 'warn');
    return false;
  }
  for (const f of files) {
    try {
      lib = await post(`/api/library/${f.id}`, { pole_mm: v.pole_mm, hand: v.hand, ...(f.arm ? {} : { arm: v.arm || undefined }) });
    } catch (err) { tell(err); return false; }
  }
  renderList();
  return true;
}

async function setStage(f) {
  if (!(await askStage([f], f.stage))) return;
  const now = byId(f.id);
  if (now && ARMS.some(a => slots[a] && slots[a].meta.id === f.id)) await view(now);   // replay with the new stage
}

async function setArm(f, arm) {
  try { lib = await post(`/api/library/${f.id}`, { arm }); renderList(); } catch (err) { tell(err); }
}

async function remove(f) {
  if (!confirm(`Remove ${f.name} from your files?\n\nIt moves to the library's trash folder on this computer.`)) return;
  for (const a of ARMS) if (slots[a] && slots[a].meta.id === f.id) await clearSlot(a);
  try { lib = await del(`/api/library/${f.id}`); renderList(); } catch (err) { tell(err); }
}

// ── the player

async function view(f) {
  if (!f.ready && !isProgram(f)) { await setStage(f); return; }
  const side = f.arm;
  const tok = ++asked[side];
  status(`Loading ${titleOf(f)} on the ${side.toUpperCase()} arm…`);
  try {
    const { viewer, robot } = await ensure3D();
    await viewer.attach($('liteViewer3d'));
    await scene(phase());
    const q = isProgram(f) ? pidQ() : '';
    const [r, track, checks] = await Promise.all([
      get(`${urlOf(f)}/timeline${isProgram(f) ? q : `?side=${side}`}`),   // refused unless the file is this arm's, with its stage
      get(`${urlOf(f)}/track${q}`),
      get(`${urlOf(f)}/checks${q}`)]);
    if (tok !== asked[side]) return;
    slots[side] = { meta: f, r, tl: robot.loadTimeline(side, r), track, checks };
    pause();
    t = 0; said = '';
    retool();
    show();
    renderSlots(); renderChecks(); renderList();
    await drawPlot();
    status(`${titleOf(f)} on the ${side.toUpperCase()} arm: staged first (${slots[side].tl.stage.text}), `
      + `then ${fmtS(trajS(side))} of trajectory. Press ▶ Play.`, 'ok');
  } catch (err) {
    if (tok === asked[side]) status(`${titleOf(f)} is not shown: ${say(err)}`, 'warn');
  }
}

async function clearSlot(side) {
  asked[side]++;                                      // a ▶ View still loading for this arm is dropped
  const { robot, viewer } = await ensure3D();
  robot.clearTrack(side);
  viewer.dirty = true;
  slots[side] = null;
  t = Math.min(t, total());
  said = '';
  retool();
  show();
  renderSlots(); renderChecks(); renderList(); drawPlot();
}

// What each arm is doing while they stage, or null once both trajectories run.
function stagingLine() {
  if (t >= start()) return null;
  return loaded().map(a => {
    const s = slots[a], ta = armTime(a, t);
    const ph = ta < s.tl.traj_t0 ? s.tl.phases.find(p => ta >= p.t0 && ta < p.t1) : null;
    return `${a.toUpperCase()}: ${ph ? ph.text : 'at its start, waiting for the other arm'}`;
  }).join(' · ');
}

// The files loaded: the view's tools check them as they replay (both arms on the viewer's clock).
function retool() {
  if (!tools) return;
  tools.load(JSON.stringify(ARMS.map(a => slots[a] && [slots[a].meta.id, slots[a].meta.stage])), {
    phase: 'lid_closed', text: 'no region', enabled: loaded().length > 0 });
  tools.setLength(total());
}

function show() {
  ensure3D().catch(() => null).then((v3) => {
    if (!v3 || v3.viewer.host !== $('liteViewer3d')) return;   // no 3D in this browser, or another view has it: the times still show
    const { robot, viewer } = v3;
    for (const a of loaded()) robot.showTime(armTime(a, t), a);
    viewer.dirty = true;
  });
  if (tools) tools.at(t);
  const T = start(), L = total() - T;
  frame($('liteViewer3d')).time(!loaded().length ? '' : t < T ? `Staging ${t.toFixed(1)} / ${T.toFixed(1)} s` : `${fmtT(t - T)} / ${fmtT(L)}`);
  frame($('liteViewer3d')).title(loaded().length ? loaded().map(a => `${a.toUpperCase()} ${titleOf(slots[a].meta)}`).join(' · ') : 'The robot at home',
    loaded().length ? 'As the robot runs them: staged first, then the 5 ms clock' : 'Choose a file with View');
  const sl = $('liteViewerSlider');
  if (sl && document.activeElement !== sl) { sl.max = String(Math.round(total() * 1000)); sl.value = String(Math.round(t * 1000)); }
  placePlayhead();
}

function sayPhase() {
  const line = stagingLine();
  const text = line ? `Staging · ${line}` : 'Playing at the 5 ms clock' + (speed !== 1 ? ` × ${speed}` : '') + '.';
  if (text !== said) { said = text; status(text, line ? 'warn' : 'ok'); }
}

function tick(now) {
  if (!playing) return;
  t = Math.min(total(), t + ((now - last) / 1000) * speed);
  last = now;
  show();
  sayPhase();
  if (t >= total()) { playing = false; said = ''; status('Finished. Press ↺ Reset or ▶ Play to watch again.', 'ok'); return; }
  raf = requestAnimationFrame(tick);
}

function play() {
  if (!total()) { status('Choose a file with ▶ View first.', 'warn'); return; }
  if (t >= total()) t = 0;
  playing = true;
  said = '';
  last = performance.now();
  cancelAnimationFrame(raf);
  raf = requestAnimationFrame(tick);
  sayPhase();
}
function pause() { playing = false; cancelAnimationFrame(raf); }
function reset() { pause(); t = 0; said = ''; show(); if (loaded().length) status('Back to the start: both arms standing. Press ▶ Play.'); }

function renderSlots() {
  const strip = $('liteViewerSlots');
  if (!strip) return;
  strip.replaceChildren(...ARMS.map(a => {
    const s = slots[a];
    return el('div', { class: `lite-slot ${a}` },
      el('span', { class: ARM_BADGE[a], text: `${a.toUpperCase()} ARM` }),
      s ? el('div', { class: 'lite-slot-file' }, el('b', { text: `${titleOf(s.meta)} · ${fmtS(trajS(a))}` }),
        el('small', { text: `staged first: ${s.tl.stage.text}` }))
        : el('span', { class: 'mini-note', text: ' standing pose' }),
      s ? el('button', { class: 'btn light', text: '✕', 'aria-label': `Clear the ${a} arm`, onclick: () => clearSlot(a) }) : null);
  }));
}

const CHECK_IDS = ['L1', 'L2', 'L3', 'L4'];

function renderChecks() {
  const host = $('liteViewerChecks');
  if (!host) return;
  const here = ctx.pid ? step() : null;
  host.replaceChildren(...loaded().map(a => {
    const { meta, checks, tl } = slots[a];
    const prog = isProgram(meta);
    const fails = checks.checks.filter(c => c.hard && c.passed === false);
    const gates = el('div', { class: 'lite-gates' },
      el('span', { class: 'lite-limit ok', title: 'arm' }, prog ? `✓ Program ${meta.program} on the ${a.toUpperCase()} controller`
        : `✓ ${a.toUpperCase()} file on the ${a.toUpperCase()} arm only`),
      el('span', { class: 'lite-limit ok', title: 'stage' }, `✓ ${prog ? 'Its own pole and hand first' : 'Staged first'}: ${tl.stage.text}`));
    const chips = el('div', { class: 'lite-limits' }, CHECK_IDS.map(id => {
      const c = checks.checks.find(x => x.id === id);
      if (!c) return null;
      return el('span', { class: `lite-limit ${c.passed === false ? 'bad' : 'ok'}`, title: c.detail || '' },
        `${c.passed === false ? '✕' : '✓'} ${id} ${c.name}${c.detail && c.passed === false ? `: ${c.detail}` : ''}`);
    }));
    let use = null;
    if (prog) {                                       // the program as the controller holds it: run from its slot
      use = el('button', { class: 'btn', text: running[a] ? `Program ${meta.program} is running` : `Run program ${meta.program} on the ${a.toUpperCase()} arm`,
        disabled: running[a], onclick: () => runProgram(a, { id: meta.program, name: meta.name }) });
    } else if (here && !here.later) {
      use = here.arm === a
        ? el('button', { class: 'btn', text: `Use for ${TASKS[currentTask].label} · ${here.display}`, onclick: () => useFor(a, here) })
        : el('p', { class: 'mini-note', text: `The step on screen, ${here.display}, is a ${here.arm.toUpperCase()}-arm step: `
          + `this ${a.toUpperCase()} file is never used there.` });
    }
    return el('div', { class: 'lite-viewer-check' },
      el('div', {}, el('span', { class: ARM_BADGE[a], text: a.toUpperCase() }), el('b', { text: ` ${meta.name}` }),
        el('small', { text: ` · ${fmtS(checks.seconds)} · ${checks.lines.toLocaleString()} rows` })),
      gates, chips,
      el('p', { class: 'mini-note', text: prog
        ? (fails.length ? `As the controller holds it, this program breaks ${fails.map(c => c.id).join(', ')}.`
          : 'As the controller holds it, this program stays within L1 to L4.')
          + ` Run moves the pole and the hand to its own (${tl.stage.text}) first; the controller then moves the arm to its `
          + 'first point and plays it (Hardware sim: in its SIM mode, the arm stays still).'
        : (fails.length
          ? `As taught, this file breaks ${fails.map(c => c.id).join(', ')}.`
          : 'As taught, this file stays within L1 to L4.')
        + ' Lite never runs a file as taught: it runs only after tuning in a region (manual tuning shows the limits L1 to L6, '
        + 'Time-optimal keeps them).' }),
      use);
  }));
}

async function useFor(side, target) {
  const f = slots[side].meta;
  try {
    const r = await post(`/api/library/${f.id}/use`, { pid: ctx.pid, key: target.key });
    ctx.units[target.key] = r.unit;
    close();
    window.showView('teach');
    armSelectionLocked = true;
    window.updateArmLockUI();
    window.setStage('review');
    window.toast(`${f.name} is now the active recording of ${target.display}.`, 'good');
  } catch (err) { tell(err); }
}

// ── the plot: LEFT solid, RIGHT dashed, on the trajectory's 5 ms time axis

async function drawPlot() {
  const div = $('liteViewerPlot');
  if (!div) return;
  if (!loaded().length) { if (window.Plotly) window.Plotly.purge(div); return; }
  await loadPlotly();
  const colours = ['--blue', '--orange', '--green', '--violet', '--cyan', '--red', '--muted']
    .map(v => getComputedStyle(document.documentElement).getPropertyValue(v).trim());
  const ink = getComputedStyle(document.documentElement).getPropertyValue('--muted').trim();
  const traces = [];
  for (const a of loaded()) {
    const tr = slots[a].track;
    const stride = Math.max(1, Math.ceil(tr.lines / 2000));
    const idx = [];
    for (let i = 0; i < tr.lines; i += stride) idx.push(i);
    tr.joints.forEach((q, j) => traces.push({
      x: idx.map(i => i * CYCLE), y: idx.map(i => q[i]), name: `J${j + 1}`, legendgroup: a, mode: 'lines', hoverinfo: 'skip',
      legendgrouptitle: j === 0 ? { text: a === 'left' ? 'LEFT (solid)' : 'RIGHT (dashed)' } : undefined,
      line: { width: 1.4, color: colours[j], dash: a === 'right' ? 'dash' : 'solid' },
    }));
  }
  await window.Plotly.react(div, traces, {
    margin: { l: 44, r: 8, t: 8, b: 30 }, paper_bgcolor: 'transparent', plot_bgcolor: 'transparent',
    xaxis: { range: [0, Math.max(CYCLE, total() - start())], ticksuffix: ' s', fixedrange: true, color: ink, zeroline: false },
    yaxis: { ticksuffix: '°', fixedrange: true, color: ink, gridcolor: ink, gridwidth: 0.3, zeroline: false },
    legend: { orientation: 'v', x: 1.01, y: 1, font: { size: 11, color: ink } }, showlegend: true,
  }, { staticPlot: true, responsive: true, displayModeBar: false });
  fitPlot(div);
  if (!div.dataset.liteClick) {                       // a click on the plot is that moment (the owner, 16 Oct)
    div.dataset.liteClick = '1';
    div.addEventListener('click', (e) => {
      const xa = div._fullLayout && div._fullLayout.xaxis;
      if (!xa || !loaded().length) return;
      const x = xa.p2l(e.clientX - div.getBoundingClientRect().left - xa._offset);
      if (!Number.isFinite(x)) return;
      pause();
      t = Math.max(0, Math.min(total(), start() + x));
      said = '';
      show();
      sayPhase();
    });
  }
  placePlayhead();
}

// The playhead on the trajectory clock; faint at 0 while the arms stage.
function placePlayhead() {
  const div = $('liteViewerPlot'), ph = $('liteViewerPlayhead');
  if (!div || !ph || !div._fullLayout || !div._fullLayout.xaxis || !loaded().length) { if (ph) ph.style.display = 'none'; return; }
  const xa = div._fullLayout.xaxis, T = start();
  ph.style.display = 'block';
  ph.style.opacity = t < T ? '0.35' : '1';
  ph.style.left = `${xa._offset + xa.l2p(Math.max(0, Math.min(t, total()) - T))}px`;
  ph.style.top = `${div._fullLayout.margin.t}px`;
  ph.style.height = `${xa._length ? div._fullLayout.height - div._fullLayout.margin.t - div._fullLayout.margin.b : 0}px`;
}

// ── the window

function build() {
  const files = el('input', { type: 'file', accept: '.txt,text/plain', multiple: true, hidden: true, onchange: (e) => { upload(e.target.files); e.target.value = ''; } });
  const folder = el('input', { type: 'file', multiple: true, hidden: true, onchange: (e) => { upload(e.target.files); e.target.value = ''; } });
  folder.setAttribute('webkitdirectory', '');
  const speedSel = el('select', { id: 'liteViewerSpeed', 'aria-label': 'Speed', onchange: (e) => { speed = Number(e.target.value); if (playing) play(); } },
    [['0.25', '25 %'], ['0.5', '50 %'], ['1', '100 % (real time)'], ['2', '200 %']].map(([v, n]) => el('option', { value: v, text: n, selected: v === '1' })));
  const slider = el('input', { id: 'liteViewerSlider', type: 'range', min: '0', max: '0', step: '5', value: '0', 'aria-label': 'Replay time',
    oninput: (e) => { pause(); t = Number(e.target.value) / 1000; show(); said = ''; sayPhase(); } });
  box = el('div', { class: 'modal lite-viewer', id: 'liteViewer' },
    el('div', { class: 'modal-card lite-viewer-card', role: 'dialog', 'aria-modal': 'true', 'aria-label': 'Trajectory Viewer' },
      el('div', { class: 'section-head' },
        el('div', {}, el('div', { class: 'eyebrow', text: "Taught files and the controllers' programs" }),
          el('h3', { text: 'Trajectory Viewer' }),
          el('p', { class: 'mini-note', text: 'Upload taught .txt files, or find a program on the controllers, then replay them as the '
            + 'robot runs them: the pole and the hand first, then the move to the start, then the trajectory at the 5 ms clock. '
            + 'Each plays on its own arm only. Only Run sends anything: a program, from its slot, with its own pole and hand.' })),
        el('div', { class: 'cta-row' },
          el('button', { class: 'btn light', text: '✕ Close', onclick: () => close() }))),
      el('div', { class: 'lite-viewer-body' },
        el('aside', { class: 'lite-viewer-side' },
          el('div', { class: 'cta-row' },
            el('button', { class: 'btn', text: '⇧ Upload .txt files', onclick: () => files.click() }),
            el('button', { class: 'btn outline', text: '⇧ Upload a folder', onclick: () => folder.click() }), files, folder),
          el('div', { id: 'liteViewerStatus', class: 'statusbar', text: 'A LEFT or RIGHT folder, or a name starting L_ or R_, sets the arm. You say the pole height and hand pose.' }),
          el('div', { id: 'liteViewerFiles', class: 'lite-viewer-files' })),
        el('section', { class: 'lite-viewer-main' },
          el('div', { id: 'liteViewerSlots', class: 'lite-viewer-slots' }),
          el('div', { id: 'liteViewer3d', class: 'lite-viewer-3d lite-3d' }),
          el('div', { class: 'lite-viewer-transport' },
            el('button', { class: 'btn green', text: '▶ Play', onclick: play }),
            el('button', { class: 'btn orange', text: 'Ⅱ Pause', onclick: () => { pause(); said = ''; status('Paused.'); } }),
            el('button', { class: 'btn light', text: '↺ Reset', onclick: reset }),
            speedSel, el('div', { id: 'liteViewerTrack', class: 'lite-rev3d-track' }, slider)),
          el('div', { class: 'lite-viewer-plot-wrap' }, el('div', { id: 'liteViewerPlot', class: 'lite-viewer-plot' }),
            el('div', { id: 'liteViewerPlayhead', class: 'lite-viewer-playhead' })),
          el('div', { id: 'liteViewerChecks', class: 'lite-viewer-checks' })))));
  document.body.append(box);
  document.addEventListener('keydown', (e) => { if (e.key === 'Escape' && box.classList.contains('show') && !document.querySelector('.lite-confirm')) close(); });
  // the view's tools (view3d.js): the commode's state, the collision check of both files as they replay
  // (each arm against the walls and the commode, and the two arms against each other), the links in red
  tools = new Tools({
    what: 'the files',
    time: (at) => (at < start() ? `${at.toFixed(2)} s of the staging` : fmtT(at - start())),
    check: async (ph, progress) => {
      const r = await checkJob(post, '/api/library/collisions', { left: slots.left?.meta.id || null, right: slots.right?.meta.id || null,
        phase: ph || 'lid_closed', pid: ctx.pid || null }, progress);
      return { result: r, spans: spansOf(r) };
    },
    seek: (at) => { pause(); t = Math.max(0, Math.min(total(), at)); said = ''; show(); sayPhase(); },
    changed: (ph) => { scene(ph || 'lid_closed').catch(() => {}); },
  }).mount($('liteViewer3d'));
  tools.track($('liteViewerTrack'));
  retool();
}

export async function openViewer() {
  if (!box) build();
  box.classList.add('show');
  document.body.style.overflow = 'hidden';
  release();                                          // the region's simulation stops; it reloads after this
  renderSlots(); renderChecks();
  refreshList();
  refreshPrograms();
  let v3;
  try {
    v3 = await ensure3D();
  } catch (err) {
    if (err.code !== 'webgl') throw err;
    say3d($('liteViewer3d'), say(err), err);            // ALIX Studio's notice; the file list still works
    return;
  }
  const { viewer, robot } = v3;
  await viewer.attach($('liteViewer3d'));
  await scene(phase());
  robot.home();
  for (const a of loaded()) slots[a].tl = robot.loadTimeline(a, slots[a].r);
  show();
  drawPlot().catch(() => {});
}

function close() {
  pause();
  box?.classList.remove('show');
  document.body.style.overflow = '';
  document.dispatchEvent(new CustomEvent('lite:viewer-closed'));
}

export function installViewer() {
  const open = () => openViewer().catch(tell);
  // the Operator Journey, Home, and Save & Review's previous-file card
  const nav = document.querySelector('.navbtn[data-view="project"]');
  nav?.after(el('button', { class: 'navbtn lite-viewer-nav', onclick: open },
    el('span', { class: 'navicon', text: '▶' }), el('span', { text: 'Trajectory Viewer' })));
  document.querySelector('#view-home .hero .cta-row')
    ?.append(el('button', { class: 'btn light big', text: '▶ View Taught Trajectories', onclick: open }));
  document.querySelector('#stage-review .browse-source-card')
    ?.append(el('button', { class: 'btn outline wide-btn lite-viewer-link', text: '▶ Open the Trajectory Viewer', onclick: open }));
}
