// Merge + Full Sequence, S2: Full & Partial Sequence (feedback v5, slide 9; the owner, 16 Oct): the
// control program's Page 02 (V53 with runner v14) in Lite's style, its layout and its working as
// they are (server/workflow.py):
// - each arm's steps: PROGRAM (a stored program, its Start %: the controller's move to its first
//   point, its Dual Rule), GLOBAL POINT (MoveJ / MoveL, speed, wait), HAND / POLE / WAIT; the table
//   with Up, Down, Duplicate, Remove, Clear, Save and Load (V53's JSON; another arm's file is
//   refused); RUN and STOP per arm;
// - RUN BOTH // SAFE TIMELINE (V53's plan), STOP BOTH, Confirm before waiting arm, the overall
//   speed (the speed slider at the top), the status and the live status of each arm.
// What Lite adds: Simulation Mode or Real Machine Mode; the full sequence or rows of it (Partial,
// a range per arm); Simulation Mode plays the run in 3D with the collision check and says what would
// not be right on the robot; Real Machine Mode runs only what completed in Simulation as it is.

import { call, get, post, put, say } from './api.js';
import { REAL_CHECKS, confirmDialog, ctx, el, tell } from './ctx.js';
import { borrow, Player } from './seq3d.js';
import { scene } from './sim.js';
import { speedPct } from './status.js';
import { saveBlobAs } from './tune.js';

const $ = (id) => document.getElementById(id);
const ARMS = ['left', 'right'];
const ARM = (s) => s.toUpperCase();
const RULES = ['TOGETHER', 'RUN FIRST', 'WAIT OTHER'];
const GRIP = [0, 0, 0, 0, 150, 1000], OPEN = [1000, 1000, 1000, 1000, 1000, 1000];
const S = {
  steps: { left: [], right: [] }, sources: { left: { programs: [], points: [] }, right: { programs: [], points: [] } },
  sel: { left: null, right: null }, mode: 'sim', scope: 'full', confirmEach: true,
  sim: null, job: null, live: { left: 'IDLE', right: 'IDLE' }, status: 'PAGE 02 READY // Build LEFT and RIGHT sequences', runtime: 0,
  rows: { left: null, right: null }, asked: 0,
};
const view = { frames: new Map(), shown: { left: null, right: null }, phase: null, three: null, clean: false, furthest: 0, token: 0 };
let panel = null, built = false, player = null, saving = { left: null, right: null };

const base = () => `/api/projects/${ctx.pid}/workflow`;
const fmt = (s) => `${Number(s || 0).toFixed(1)} s`;

// ── the steps as V53 shows them (WorkflowSequencePanel._row)

function row(step) {
  const k = step.type;
  if (k === 'program') {
    const known = (S.sources[step._side]?.programs || []).find(p => p.id === step.id);
    const name = step.name || (known && known.name) || '';
    return [k.toUpperCase(), `Program ${step.id}${name ? ` // ${name}` : ''}`,
      `Start ${step.transit_speed ?? step.start_speed ?? 30}% // Stored timing // ${String(step.dual_rule || 'TOGETHER').toUpperCase()}`];
  }
  if (k === 'pole') return ['POLE', `${Number(step.mm)} mm`, ''];
  if (k === 'hand') return ['HAND', step.values.join(','), 'L,R,M,I,TB,TR'];
  if (k === 'wait') return ['WAIT', `${Number(step.seconds)} s`, step.note || ''];
  return ['GLOBAL', step.name, `${String(step.move).toUpperCase()} // Speed ${step.speed}% // Wait ${Number(step.wait_s)}s`];
}

// ── the panel

function btn(text, onclick, cls = 'btn light', attrs = {}) {
  return el('button', { class: cls, type: 'button', text, onclick, ...attrs });
}

function armPanel(side) {
  const tab = (id, text) => el('button', { id: `liteS2Tab_${side}_${id}`, class: 'lite-s2-subtab', type: 'button', role: 'tab', text,
    onclick: () => subtab(side, id) });
  const num = (id, value, attrs = {}) => el('input', { id: `liteS2_${side}_${id}`, type: 'number', value: String(value), ...attrs });
  return el('div', { class: `card lite-s2-arm ${side}` },
    el('div', { class: 'lite-s2-armhead' }, el('b', { text: `${ARM(side)} ARM // SEQUENCE` })),
    el('div', { class: 'lite-s2-subtabs', role: 'tablist' }, tab('program', 'PROGRAM'), tab('global', 'GLOBAL POINT'), tab('aux', 'HAND / POLE / WAIT')),
    el('div', { id: `liteS2Add_${side}_program`, class: 'lite-s2-add' },
      el('div', { class: 'lite-row' },
        el('div', { class: 'field lite-grow' }, el('label', { for: `liteS2_${side}_prog`, text: 'Stored Program' }),
          el('input', { id: `liteS2_${side}_prog`, type: 'text', list: `liteS2Progs_${side}`, autocomplete: 'off', placeholder: 'ID, e.g. 71' }),
          el('datalist', { id: `liteS2Progs_${side}` })),
        el('div', { class: 'field' }, el('label', { for: `liteS2_${side}_start`, text: 'Start %' }), num('start', 30, { min: '1', max: '100', step: '1' })),
        el('div', { class: 'field' }, el('label', { for: `liteS2_${side}_rule`, text: 'Dual Rule' }),
          el('select', { id: `liteS2_${side}_rule` }, RULES.map(r => el('option', { value: r, text: r })))),
        btn('+ ADD PROGRAM', () => addProgram(side), 'btn green')),
      el('div', { class: 'mini-note', text: 'Start % = speed used only to reach this Program\'s first pose (default 30%). Stored trajectory timing is '
        + 'unchanged. Dual Rule controls TOGETHER / RUN FIRST / WAIT OTHER.' })),
    el('div', { id: `liteS2Add_${side}_global`, class: 'lite-s2-add', hidden: true },
      el('div', { class: 'lite-row' },
        el('div', { class: 'field lite-grow' }, el('label', { for: `liteS2_${side}_point`, text: 'Global Point' }),
          el('input', { id: `liteS2_${side}_point`, type: 'text', list: `liteS2Points_${side}`, autocomplete: 'off' }),
          el('datalist', { id: `liteS2Points_${side}` })),
        el('div', { class: 'field' }, el('label', { for: `liteS2_${side}_move`, text: 'Move' }),
          el('select', { id: `liteS2_${side}_move` }, el('option', { value: 'movej', text: 'MoveJ' }), el('option', { value: 'movel', text: 'MoveL' }))),
        el('div', { class: 'field' }, el('label', { for: `liteS2_${side}_gspeed`, text: 'Speed' }), num('gspeed', 20, { min: '1', max: '100', step: '1' })),
        el('div', { class: 'field' }, el('label', { for: `liteS2_${side}_gwait`, text: 'Wait' }), num('gwait', 0, { min: '0', max: '3600', step: '0.1' })),
        btn('+ ADD GLOBAL', () => addGlobal(side), 'btn'))),
    el('div', { id: `liteS2Add_${side}_aux`, class: 'lite-s2-add', hidden: true },
      el('div', { class: 'lite-row' },
        el('div', { class: 'field' }, el('label', { for: `liteS2_${side}_pole`, text: 'Pole mm' }), num('pole', 0, { min: '0', max: '315', step: '1' })),
        btn('+ POLE', () => addPole(side), 'btn'),
        el('div', { class: 'field' }, el('label', { for: `liteS2_${side}_wait`, text: 'Wait s' }), num('wait', 1, { min: '0', max: '3600', step: '0.1' })),
        btn('+ WAIT', () => addWait(side), 'btn light')),
      el('div', { class: 'lite-row lite-s2-hand' },
        ['L', 'R', 'M', 'I', 'TB', 'TR'].map((n, i) => el('div', { class: 'field' }, el('label', { for: `liteS2_${side}_h${i}`, text: n }),
          num(`h${i}`, 1000, { min: '0', max: '1000', step: '1' }))),
        btn('GRIP', () => setHand(side, GRIP), 'btn orange'), btn('OPEN', () => setHand(side, OPEN), 'btn green'),
        btn('+ ADD HAND', () => addHand(side), 'btn green'))),
    el('div', { class: 'lite-table-scroll lite-s2-table' }, el('table', { class: 'lite-table' },
      el('thead', {}, el('tr', {}, ['#', 'TYPE', 'TARGET', 'OPTIONS'].map(h => el('th', { text: h })))),
      el('tbody', { id: `liteS2Rows_${side}` }))),
    el('div', { class: 'lite-row lite-s2-tools' },
      btn('UP', () => moveSel(side, -1)), btn('DOWN', () => moveSel(side, 1)), btn('DUPLICATE', () => duplicate(side)),
      btn('REMOVE', () => remove(side)), btn('CLEAR', () => clear(side)),
      el('span', { class: 'lite-grow' }),
      btn('SAVE', () => saveFile(side), 'btn'), btn('LOAD', () => $(`liteS2File_${side}`).click(), 'btn outline'),
      el('input', { id: `liteS2File_${side}`, type: 'file', accept: '.json,application/json', hidden: true, onchange: (e) => loadFile(side, e) })),
    el('div', { class: 'lite-row lite-s2-runbar' },
      btn(`RUN ${ARM(side)} SEQUENCE`, () => run(side), 'btn green lite-grow', { id: `liteS2Run_${side}` }),
      btn(`STOP ${ARM(side)}`, () => stop(side), 'btn red', { id: `liteS2Stop_${side}` })),
    el('div', { id: `liteS2ArmStatus_${side}`, class: 'mini-note' }));
}

function build() {
  if (built) return;
  built = true;
  panel.replaceChildren(
    el('div', { class: 'lite-s2-header' },
      el('div', {},
        el('div', { class: 'eyebrow', text: 'PAGE 02 // DUAL ARM SEQUENCE BUILDER // SAFE START + LIVE STATUS' }),
        el('h3', { text: 'Build independent LEFT and RIGHT cleaning workflows' }),
        el('p', { class: 'mini-note', text: 'Mix stored Programs, Pole, 6-channel Hand, Wait, and Global Points (MoveJ / MoveL + speed) in any order.' })),
      el('div', { class: 'lite-row' }, btn('Refresh Programs', () => refreshSources(true)), btn('Refresh Global Points', () => refreshSources(true)))),
    el('div', { class: 'card lite-s2-exec' },
      el('div', { class: 'lite-s2-modes', role: 'radiogroup', 'aria-label': 'Execution mode' },
        el('b', { text: 'Execution mode' }),
        btn('Simulation Mode', () => setMode('sim'), 'btn light lite-s2-mode', { id: 'liteS2Mode_sim', role: 'radio' }),
        btn('Real Machine Mode', () => setMode('real'), 'btn light lite-s2-mode', { id: 'liteS2Mode_real', role: 'radio' }),
        el('span', { id: 'liteS2Gate', class: 'lite-status' })),
      el('div', { class: 'lite-s2-modes', role: 'radiogroup', 'aria-label': 'Sequence' },
        el('b', { text: 'Sequence' }),
        btn('Full Sequence', () => setScope('full'), 'btn light lite-s2-mode', { id: 'liteS2Scope_full', role: 'radio' }),
        btn('Partial Sequence', () => setScope('partial'), 'btn light lite-s2-mode', { id: 'liteS2Scope_partial', role: 'radio' }),
        el('span', { id: 'liteS2Ranges', class: 'lite-s2-ranges', hidden: true },
          ARMS.map(side => el('span', { class: 'lite-s2-range' }, el('b', { text: `${ARM(side)} rows` }),
            el('input', { id: `liteS2From_${side}`, type: 'number', min: '1', step: '1', 'aria-label': `${ARM(side)} first row`, onchange: gate }),
            ' to ',
            el('input', { id: `liteS2To_${side}`, type: 'number', min: '1', step: '1', 'aria-label': `${ARM(side)} last row`, onchange: gate })))))),
    el('div', { class: 'card lite-s2-control' },
      el('div', { class: 'lite-s2-kicker', text: 'DUAL SEQUENCE CONTROL' }),
      el('div', { class: 'lite-row' },
        btn('RUN BOTH // SAFE TIMELINE', () => run('both'), 'btn green', { id: 'liteS2RunBoth' }),
        btn('STOP BOTH SEQUENCES', () => stop('both'), 'btn red', { id: 'liteS2StopBoth' }),
        el('label', { class: 'lite-s2-check' }, el('input', { id: 'liteS2Confirm', type: 'checkbox', checked: true,
          onchange: (e) => { S.confirmEach = e.target.checked; } }), ' CONFIRM BEFORE WAITING ARM'),
        el('span', { class: 'lite-s2-speed' }, 'OVERALL SPEED % ', el('b', { id: 'liteS2Speed', text: '50' }),
          el('small', { class: 'mini-note', text: ' (the speed slider at the top)' })),
        el('span', { id: 'liteS2StatusText', class: 'lite-s2-statustext lite-grow' }),
        el('b', { id: 'liteS2Runtime', class: 'lite-s2-runtime', text: '0.0 s' }))),
    el('div', { class: 'lite-s2-kicker', text: 'LIVE ARM STATUS' }),
    el('div', { class: 'lite-s2-live' }, ARMS.map(side => el('div', { id: `liteS2Live_${side}`, class: 'lite-s2-livearm', text: `${ARM(side)} // IDLE` }))),
    el('div', { class: 'lite-s2-arms' }, ARMS.map(armPanel)),
    el('div', { id: 'liteS2Sim', class: 'card lite-s2-sim' },
      el('div', { class: 'lite-section-title' },
        el('h3', {}, 'Simulation · ', el('span', { id: 'liteS2SimName', text: 'press RUN in Simulation Mode' })),
        el('div', { class: 'lite-rev3d-views', role: 'group', 'aria-label': 'View' },
          [['default', 'Default'], ['front', 'Front'], ['side', 'Side'], ['top', 'Top']].map(([v, n]) => btn(n, () => setView(v), 'btn light lite-rev3d-view')))),
      el('div', { class: 'lite-s2-simgrid' },
        el('div', { id: 'liteS2Host', class: 'lite-rev3d-host' }),
        el('div', { class: 'lite-rev3d-side' },
          el('div', { class: 'lite-row' }, btn('Play', () => playSim(), 'btn'), btn('Pause', () => player.pause(), 'btn outline'),
            btn('Restart', () => restartSim()),
            el('select', { id: 'liteS2Speedup', 'aria-label': 'Replay speed', onchange: (e) => player.setSpeed(Number(e.target.value)) },
              [1, 2, 4].map(x => el('option', { value: String(x), text: `${x}x` }))),
            el('b', { id: 'liteS2SimTime', class: 'lite-rev3d-time', text: '0.0 s / 0.0 s' })),
          el('div', { class: 'lite-rev3d-track' },
            el('input', { id: 'liteS2Timeline', type: 'range', min: '0', max: '0', step: '10', value: '0', 'aria-label': 'Sequence time',
              oninput: (e) => { player.pause(); seekSim(Number(e.target.value) / 1000); } }),
            el('div', { id: 'liteS2Marks', class: 'lite-rev3d-marks', 'aria-hidden': 'true' })),
          el('div', { id: 'liteS2SimState', class: 'lite-status' }),
          el('div', { id: 'liteS2Stages', class: 'lite-s2-stages' }),
          el('div', { id: 'liteS2Findings', class: 'lite-s2-findings' })))));
  player = new Player({ onTime: showAt, onEnd: simEnded });
  for (const side of ARMS) subtab(side, 'program');
}

function subtab(side, id) {
  for (const t of ['program', 'global', 'aux']) {
    $(`liteS2Add_${side}_${t}`).hidden = t !== id;
    $(`liteS2Tab_${side}_${t}`).classList.toggle('active', t === id);
    $(`liteS2Tab_${side}_${t}`).setAttribute('aria-selected', String(t === id));
  }
}

// ── data

async function load() {
  try {
    const d = await get(base());
    S.steps.left = d.left;
    S.steps.right = d.right;
    if (d.sim && (!S.sim || S.sim.hash !== d.sim.hash)) setSim(d.sim, false);
    else if (d.sim && S.sim) S.sim.completed = d.sim.completed;
    if (d.running && (!S.job || S.job.id !== d.running.id)) follow(d.running);
  } catch (err) { tell(err); }
  paintAll();
}

async function refreshSources(said = false) {
  await Promise.all(ARMS.map(async (side) => {
    try { S.sources[side] = await get(`${base()}/${side}/sources`); } catch (_) { /* the lists stay */ }
    const pl = $(`liteS2Progs_${side}`);
    if (pl) pl.replaceChildren(...S.sources[side].programs.map(p => el('option', { value: `${p.id}${p.name ? ` // ${p.name}` : ''}`,
      text: p.sent ? 'sent by Lite' : (p.gone ? 'no longer on the controller' : 'on the controller') })));
    const pp = $(`liteS2Points_${side}`);
    if (pp) pp.replaceChildren(...S.sources[side].points.map(n => el('option', { value: n })));
  }));
  paintAll();
  if (said) {
    window.toast(`Programs: LEFT ${S.sources.left.programs.length}, RIGHT ${S.sources.right.programs.length}. Global points: `
      + ARMS.map(s => (S.sources[s].points_read ? `${ARM(s)} ${S.sources[s].points.length}` : `${ARM(s)} not read (connect the arm)`)).join(', ') + '.');
  }
}

async function saveSteps(side) {
  clearTimeout(saving[side]);
  saving[side] = setTimeout(async () => {
    try {
      const d = await put(`${base()}/${side}`, { steps: S.steps[side] });
      S.steps[side] = d[side];
    } catch (err) {
      tell(err);
      try { S.steps[side] = (await get(base()))[side]; } catch (_) { /* as it was */ }
    }
    paintAll();
  }, 250);
}

// ── editing (V53's WorkflowSequencePanel)

const val = (side, id) => $(`liteS2_${side}_${id}`)?.value?.trim() ?? '';

function append(side, step) {
  S.steps[side].push(step);
  S.sel[side] = S.steps[side].length - 1;
  paintArm(side);
  saveSteps(side);
}

function addProgram(side) {
  const m = /^\s*(\d+)/.exec(val(side, 'prog'));
  if (!m) { window.toast(`${ARM(side)}: choose or enter a Program ID.`); return; }
  const id = Number(m[1]);
  if (!(id >= 1 && id <= 100)) { window.toast('Program ID must be 1-100.'); return; }
  let start = Math.trunc(Number(val(side, 'start')));
  if (!Number.isFinite(start)) start = 30;
  if (!(start >= 1 && start <= 100)) { window.toast('Start / Transit speed must be 1-100%.'); return; }
  const known = S.sources[side].programs.find(p => p.id === id);
  const rule = RULES.includes(val(side, 'rule')) ? val(side, 'rule') : 'TOGETHER';
  append(side, { type: 'program', id, name: (known && known.name) || '', speed: 100, transit_speed: start, dual_rule: rule });
}

function addGlobal(side) {
  const name = val(side, 'point');
  if (!name) { window.toast(`${ARM(side)}: choose or enter a global point name.`); return; }
  const speed = Math.trunc(Number(val(side, 'gspeed'))), wait = Number(val(side, 'gwait') || 0);
  if (!Number.isFinite(speed) || !Number.isFinite(wait)) { window.toast('Speed and Wait must be numeric.'); return; }
  if (!(speed >= 1 && speed <= 100)) { window.toast('Speed must be 1-100%.'); return; }
  if (!(wait >= 0 && wait <= 3600)) { window.toast('Wait must be 0-3600 seconds.'); return; }
  append(side, { type: 'global', name, move: val(side, 'move') === 'movel' ? 'movel' : 'movej', speed, wait_s: wait });
}

function addPole(side) {
  const mm = Number(val(side, 'pole'));
  if (!(mm >= 0 && mm <= 315) || val(side, 'pole') === '') { window.toast('Pole height must be 0-315 mm.'); return; }
  append(side, { type: 'pole', mm });
}

function addWait(side) {
  const s = Number(val(side, 'wait'));
  if (!(s >= 0 && s <= 3600) || val(side, 'wait') === '') { window.toast('Wait must be 0-3600 seconds.'); return; }
  append(side, { type: 'wait', seconds: s });
}

function setHand(side, values) { values.forEach((v, i) => { $(`liteS2_${side}_h${i}`).value = String(v); }); }

function addHand(side) {
  const values = [0, 1, 2, 3, 4, 5].map(i => Math.trunc(Number(val(side, `h${i}`))));
  if (values.some(v => !Number.isFinite(v) || v < 0 || v > 1000)) { window.toast('All six Hand values must be 0-1000.'); return; }
  append(side, { type: 'hand', values });
}

function moveSel(side, d) {
  const i = S.sel[side], list = S.steps[side];
  if (i == null || i + d < 0 || i + d >= list.length) return;
  [list[i], list[i + d]] = [list[i + d], list[i]];
  S.sel[side] = i + d;
  paintArm(side);
  saveSteps(side);
}

function duplicate(side) {
  const i = S.sel[side];
  if (i == null) return;
  S.steps[side].splice(i + 1, 0, JSON.parse(JSON.stringify(S.steps[side][i])));
  S.sel[side] = i + 1;
  paintArm(side);
  saveSteps(side);
}

function remove(side) {
  const i = S.sel[side];
  if (i == null) return;
  S.steps[side].splice(i, 1);
  S.sel[side] = S.steps[side].length ? Math.min(i, S.steps[side].length - 1) : null;
  paintArm(side);
  saveSteps(side);
}

async function clear(side) {
  if (!S.steps[side].length) return;
  const yes = await confirmDialog({ title: 'Clear sequence', body: `Clear all ${ARM(side)} sequence steps?`, ok: 'Clear', danger: true });
  if (!yes) return;
  S.steps[side] = [];
  S.sel[side] = null;
  paintArm(side);
  saveSteps(side);
}

async function saveFile(side) {
  if (!S.steps[side].length) { window.toast(`${ARM(side)} sequence is empty.`); return; }
  try {
    const blob = await call('GET', `${base()}/${side}/download`, undefined, { raw: true });
    const saved = await saveBlobAs(blob, `${side}_workflow_sequence.json`, [{ description: 'JSON sequence', accept: { 'application/json': ['.json'] } }]);
    if (saved) window.toast(`Saved ${saved}.`);
  } catch (err) { tell(err); }
}

async function loadFile(side, e) {
  const f = e.target.files && e.target.files[0];
  e.target.value = '';
  if (!f) return;
  try {
    const d = await post(`${base()}/${side}/load`, { document: await f.text() });
    S.steps[side] = d[side];
    S.sel[side] = S.steps[side].length ? 0 : null;
    window.toast(`${ARM(side)}: ${S.steps[side].length} step(s) loaded from ${f.name}.`);
  } catch (err) { window.toast(`Could not load sequence: ${say(err)}`); }
  paintAll();
}

// ── painting

function paintArm(side) {
  const body = $(`liteS2Rows_${side}`);
  if (!body) return;
  const list = S.steps[side];
  const now = S.rows[side];
  body.replaceChildren(...list.map((step, i) => {
    const [type, target, options] = row({ ...step, _side: side });
    const tr = el('tr', { class: `${S.sel[side] === i ? 'lite-picked' : ''}${now === i + 1 ? ' lite-s2-now' : ''}`, tabindex: '0',
      onclick: () => { S.sel[side] = i; paintArm(side); }, onkeydown: (e) => { if (e.key === 'Enter' || e.key === ' ') { S.sel[side] = i; paintArm(side); } } },
    el('td', { text: String(i + 1) }), el('td', { text: type }), el('td', { text: target }), el('td', { text: options }));
    return tr;
  }));
  const cur = now ? body.querySelector('.lite-s2-now') : null;          // the row in view inside its table, the page left where it is
  const box = body.closest('.lite-s2-table');
  if (cur && box) {
    const top = cur.offsetTop - box.querySelector('thead').offsetHeight, bottom = cur.offsetTop + cur.offsetHeight;
    if (top < box.scrollTop) box.scrollTop = top;
    else if (bottom > box.scrollTop + box.clientHeight) box.scrollTop = bottom - box.clientHeight;
  }
  const st = $(`liteS2ArmStatus_${side}`);
  if (st) st.textContent = `${ARM(side)} // ${list.length} sequence step(s)`;
  for (const id of ['From', 'To']) {
    const inp = $(`liteS2${id}_${side}`);
    if (!inp) continue;
    inp.max = String(Math.max(1, list.length));
    if (!inp.value || Number(inp.value) > list.length) inp.value = String(id === 'From' ? 1 : Math.max(1, list.length));
  }
}

function paintAll() {
  if (!built) return;
  for (const side of ARMS) paintArm(side);
  for (const m of ['sim', 'real']) $(`liteS2Mode_${m}`).classList.toggle('active', S.mode === m);
  for (const m of ['full', 'partial']) $(`liteS2Scope_${m}`).classList.toggle('active', S.scope === m);
  for (const m of ['sim', 'real']) $(`liteS2Mode_${m}`).setAttribute('aria-checked', String(S.mode === m));
  for (const m of ['full', 'partial']) $(`liteS2Scope_${m}`).setAttribute('aria-checked', String(S.scope === m));
  $('liteS2Ranges').hidden = S.scope !== 'partial';
  $('liteS2Speed').textContent = String(speedPct());
  $('liteS2StatusText').textContent = S.status;
  $('liteS2Runtime').textContent = fmt(S.runtime);
  for (const side of ARMS) $(`liteS2Live_${side}`).textContent = `${ARM(side)} // ${S.live[side]}`;
  const busy = !!S.job;
  for (const id of ['liteS2RunBoth', 'liteS2Run_left', 'liteS2Run_right']) $(id).disabled = busy;
  $('liteS2Sim').hidden = S.mode !== 'sim' && !S.sim;
  gate();
}

function setMode(m) { S.mode = m; paintAll(); }
function setScope(s) { S.scope = s; paintAll(); }

function ranges(mode) {
  if (S.scope !== 'partial') return null;
  const out = {};
  for (const side of ARMS) {
    if (mode !== 'both' && mode !== side) continue;
    out[side] = [Number($(`liteS2From_${side}`).value), Number($(`liteS2To_${side}`).value)];
  }
  return out;
}

// Whether Real Machine Mode can run what is chosen: the last simulation is this run, played to its end
// (the server checks it again, with the programs and the points on the controllers, when it starts).
function simulatedNow(mode) {
  const sim = S.sim;
  if (!sim || !sim.completed || sim.mode !== mode) return false;
  const r = ranges(mode) || {};
  for (const side of Object.keys(sim.steps || {})) {
    const want = (sim.ranges || {})[side] || null, have = r[side] || null;
    if (JSON.stringify(want) !== JSON.stringify(have)) return false;
    const strip = (x) => JSON.stringify(x.map(({ _row, ...rest }) => rest));
    const lo = have ? have[0] - 1 : 0, hi = have ? have[1] : S.steps[side].length;
    if (strip(S.steps[side].slice(lo, hi)) !== strip(sim.steps[side])) return false;
  }
  return sim.overall === speedPct();
}

function gate() {
  const g = $('liteS2Gate');
  if (!g) return;
  if (S.mode === 'sim') { g.className = 'lite-status'; g.textContent = 'RUN plays the sequence in 3D with the collision check. Nothing moves.'; return; }
  const ok = ['both', 'left', 'right'].filter(simulatedNow);
  g.className = `lite-status ${ok.length ? 'ok' : 'warn'}`;
  g.textContent = ok.length ? `Simulated as it is, played to its end: ${ok.map(m => (m === 'both' ? 'RUN BOTH' : `RUN ${ARM(m)}`)).join(', ')} can run on the robot.`
    : 'Real Machine Mode runs only a sequence that completed in Simulation Mode as it is now: run it there first.';
}

// ── running: Simulation Mode or Real Machine Mode

async function run(mode) {
  if (!ctx.pid) { window.toast('Open a project first.'); return; }
  if (S.job) { window.toast('A sequence is already running.'); return; }
  if (mode === 'both' && (!S.steps.left.length || !S.steps.right.length)) { window.toast('Build or load BOTH LEFT and RIGHT sequences first.'); return; }
  if (mode !== 'both' && !S.steps[mode].length) { window.toast(`${ARM(mode)} sequence is empty.`); return; }
  if (S.mode === 'sim') simulate(mode);
  else runReal(mode);
}

async function simulate(mode) {
  player.pause();
  S.status = `PAGE 02 SIMULATION // ${mode === 'both' ? 'LEFT + RIGHT' : ARM(mode)} // timing the stages and checking collisions`;
  try {
    const job = await post(`${base()}/simulate`, { mode, ranges: ranges(mode) });
    S.job = job;
    paintAll();
    const done = await watch(job);
    if (done.state !== 'done') { S.status = `PAGE 02 SIMULATION STOPPED // ${done.error || done.state}`; tell(Object.assign(new Error(done.error || 'Stopped.'), { hint: done.hint })); return; }
    const sim = (await get(base())).sim;                     // the page's view of it: its steps, without the heavy parts
    S.job = null;                                            // over: the player paints the arms from here
    setSim(sim, true);
  } catch (err) { S.status = `PAGE 02 // ${say(err)}`; tell(err); } finally { S.job = null; paintAll(); }
}

async function runReal(mode) {
  const body = { mode, ranges: ranges(mode), confirm_each: S.confirmEach };
  let job;
  try {
    job = await post(`${base()}/run`, body);
  } catch (err) {
    if (err.code === 'simulate') { tell(err); setMode('sim'); return; }
    if (err.code !== 'warnings' && err.code !== 'confirm') { tell(err); return; }
    let warns = [];
    if (err.code === 'warnings') warns = [err.message.replace(/^Before it runs on the robot: /, '')];
    let plan = null;
    try { plan = await post(`${base()}/plan`, { mode, ranges: ranges(mode) }); } catch (_) { /* the server says it again */ }
    const c = plan && plan.counts;
    const what = mode === 'both' ? 'DUAL SAFE-TIMELINE EXECUTION' : `${ARM(mode)} ROBOT ARM WILL MOVE`;
    const yes = await confirmDialog({ title: mode === 'both' ? 'Run Dual Sequences' : `Run ${ARM(mode)} Sequence`, ok: 'Start on the robot',
      body: `${what}. ${c ? (mode === 'both' ? `Coordinated timeline/serial stages: ${c.coordinated}; TOGETHER timeline blocks: ${c.together}; `
        + `one-arm-at-a-time stages: ${c.one_arm}; manual safety checkpoints: ${c.checkpoints}. ` : '') : ''}`
        + `Overall controller speed: ${speedPct()}%. ${warns.length ? `The simulation found: ${warns.join(' ')} ` : ''}`
        + 'Confirm both work areas are clear and both hardware E-stops are accessible.',
      list: REAL_CHECKS });
    if (!yes) { window.toast('Cancelled: nothing was sent to the robot.'); return; }
    try { job = await post(`${base()}/run`, { ...body, confirmed: true, warnings_seen: true }); } catch (e2) { tell(e2); return; }
  }
  follow(job);
}

async function follow(job) {
  S.job = job;
  paintAll();
  try {
    const done = await watch(job);
    if (done.state === 'done') window.toast(`Sequence complete (${fmt(done.elapsed_s)}).`);
    else if (done.state === 'cancelled') window.toast('The sequence was stopped.');
    else tell(Object.assign(new Error(done.error || 'The sequence did not finish.'), { hint: done.hint }));
    if (done.state !== 'done' && done.error) S.status = `PROGRAM RULES STOPPED // ${done.error}`;
  } catch (err) { tell(err); } finally {
    S.job = null;
    S.rows = { left: null, right: null };
    paintAll();
  }
}

// Polls a job, painting its live status and asking its questions (the lid checkpoint, the waiting arm).
async function watch(job) {
  let j = job;
  for (;;) {
    paintJob(j);
    if (j.state !== 'running') return j;
    const q = j.extra && j.extra.question;
    if (q && q.id > S.asked) {
      S.asked = q.id;
      const yes = await confirmDialog({ title: q.title, body: q.text, ok: q.ok, danger: true });
      try { j = await post(`/api/jobs/${j.id}/answer`, { question: q.id, yes }); continue; } catch (err) { tell(err); }
    }
    await new Promise(r => setTimeout(r, 300));
    j = await get(`/api/jobs/${j.id}`);
  }
}

function paintJob(j) {
  const x = j.extra || {};
  if (x.live) S.live = { ...S.live, ...x.live };
  if (x.status) S.status = x.status;
  else if (j.kind === 'simulate') S.status = `PAGE 02 SIMULATION // ${j.stage || 'starting'}`;
  const rowsChanged = x.rows && JSON.stringify(x.rows) !== JSON.stringify(S.rows);
  if (x.rows) S.rows = { ...x.rows };
  S.runtime = j.elapsed_s || 0;
  if (rowsChanged) paintAll();
  else paintBar();
}

// The control bar and the live status only (a job's poll; the tables stay as they are).
function paintBar() {
  if (!built) return;
  $('liteS2StatusText').textContent = S.status;
  $('liteS2Runtime').textContent = fmt(S.runtime);
  for (const side of ARMS) $(`liteS2Live_${side}`).textContent = `${ARM(side)} // ${S.live[side]}`;
  const busy = !!S.job;
  for (const id of ['liteS2RunBoth', 'liteS2Run_left', 'liteS2Run_right']) $(id).disabled = busy;
}

async function stop(which) {
  if (S.mode === 'sim' || (S.job && S.job.kind === 'simulate')) {
    player.pause();
    if (S.job && S.job.kind === 'simulate') { try { await post(`/api/jobs/${S.job.id}/cancel`, {}); } catch (_) { /* it ends by itself */ } }
    S.status = 'PAGE 02 SIMULATION // stopped';
    paintAll();
    if (S.mode === 'sim') return;
  }
  S.status = which === 'both' ? 'PAGE 02 STOP REQUESTED // LEFT + RIGHT' : `${ARM(which)} // STOP REQUESTED`;
  for (const side of (which === 'both' ? ARMS : [which])) S.live[side] = 'STOP REQUESTED';
  paintAll();
  try { await post(which === 'both' ? '/api/robots/stop-both' : `/api/robots/${which}/stop`, {}); } catch (err) { tell(err); }
}

// ── Simulation Mode's 3D: each arm's part of each stage as frames, on the run's own clock

function setSim(sim, fresh) {
  S.sim = sim;
  view.token += 1;
  view.frames.clear();
  view.shown = { left: null, right: null };
  view.phase = null;
  view.clean = false;
  view.furthest = 0;
  $('liteS2SimName').textContent = `${sim.mode === 'both' ? 'both arms' : `${ARM(sim.mode)} arm`}`
    + `${Object.values(sim.ranges || {}).some(Boolean) ? ', partial' : ''} · ${sim.counts.stages} stage(s) · ${fmt(sim.total_s)}`
    + (sim.completed ? ' · played to its end' : '');
  $('liteS2Timeline').max = String(Math.round(sim.total_s * 1000));
  paintStages();
  paintFindings();
  player.load(sim.total_s);
  if (fresh) {
    S.status = `PAGE 02 SIMULATION READY // ${sim.counts.stages} stage(s), ${fmt(sim.total_s)}`;
    prefetch().then((ok) => { if (ok) playSim(true); });
  }
  paintAll();
}

const chunks = (side) => {
  const out = [{ stage: -1, t0: 0 }];
  for (const a of (S.sim.acts[side] || [])) if (out[out.length - 1].stage !== a.stage) out.push({ stage: a.stage, t0: a.t0, t1: a.t1 });
  return out;
};
const chunkAt = (side, g) => { let c = null; for (const x of chunks(side)) { if (x.t0 <= g + 1e-9) c = x; else break; } return c; };

async function frames(side, stage) {
  const k = `${side}:${stage}`;
  if (!view.frames.has(k)) {
    const p = post(`${base()}/frames`, { side, stage });
    p.catch(() => view.frames.delete(k));
    view.frames.set(k, p);
  }
  return view.frames.get(k);
}

async function prefetch() {
  const my = view.token;
  const host = $('liteS2Host');
  try { view.three = await borrow(host); } catch (err) { $('liteS2SimState').textContent = `No 3D in this browser: ${say(err)}`; return false; }
  const all = ARMS.flatMap(side => chunks(side).map(c => [side, c.stage]));
  let n = 0;
  for (const [side, stage] of all) {
    if (my !== view.token) return false;
    $('liteS2SimState').className = 'lite-status';
    $('liteS2SimState').textContent = `Loading the 3D frames: ${n + 1} of ${all.length}…`;
    try { await frames(side, stage); } catch (err) { $('liteS2SimState').textContent = `The 3D frames could not load: ${say(err)}`; return false; }
    n += 1;
  }
  $('liteS2SimState').textContent = '';
  view.three.robot.home();
  return my === view.token;
}

function playSim(fromStart = false) {
  if (!S.sim) { window.toast('Run the sequence in Simulation Mode first.'); return; }
  if (!(S.sim.total_s > 0)) { view.clean = true; simEnded(); return; }   // nothing that takes time: it is at its end
  if (fromStart || player.t >= player.total - 1e-3) restartSim(false);
  if (player.t === 0) { view.clean = true; view.furthest = 0; }
  player.play();
}

function restartSim(paused = true) {
  if (paused) player.pause();
  view.clean = true;
  view.furthest = 0;
  player.seek(0);
}

function seekSim(t) {
  if (t > view.furthest + 0.5) view.clean = false;          // skipped ahead: not played through
  player.seek(t);
}

async function showAt(g, playing = false) {
  if (!S.sim) return;
  if (playing && g > view.furthest) view.furthest = g;
  const tl = $('liteS2Timeline');
  if (tl && document.activeElement !== tl) tl.value = String(Math.round(g * 1000));
  $('liteS2SimTime').textContent = `${fmt(g)} / ${fmt(S.sim.total_s)}`;
  paintMoment(g);
  const three = view.three;
  if (!three) return;
  const my = view.token;
  for (const side of ARMS) {
    const c = chunkAt(side, g);
    if (!c) continue;
    const k = `${side}:${c.stage}`;
    if (view.shown[side] !== k) {
      let r;
      try { r = await frames(side, c.stage); } catch (_) { continue; }
      if (my !== view.token) return;
      three.robot.loadTimeline(side, r);
      view.shown[side] = k;
    }
    three.robot.showTime(g, side);
  }
  const ph = phaseAt(g);
  if (ph !== view.phase) { view.phase = ph; scene(ph, undefined, S.sim.scene || null).catch(() => {}); }
  three.viewer.dirty = true;
}

function phaseAt(g) {
  let ph = 'lid_closed';
  for (const [t, p] of (S.sim.phases || [])) { if (t <= g + 1e-9) ph = p; else break; }
  if ((S.sim.phases || []).length && g < S.sim.phases[0][0]) ph = S.sim.phases[0][1];
  return ph;
}

// What each arm does at g, as the live status and the row in its table.
function paintMoment(g) {
  const sim = S.sim;
  if (S.job) return;                                           // a run on the robot paints its own
  const stage = sim.stages.find(s => s.t0 <= g && g < s.t1) || sim.stages.filter(s => s.t0 <= g).pop();
  for (const side of ARMS) {
    const acts = sim.acts[side] || [];
    const a = acts.find(x => x.t0 <= g && g < x.t1);
    let text = 'IDLE';
    if (a) text = `${a.text} // ${a.drawn === false ? 'NOT DRAWN' : 'RUNNING'} (row ${a.row})`;
    else if (stage && stage.kind === 'serial' && stage.second === side && g < stage.handoff_t) text = `Program ${stage.label} // WAITING FOR ${ARM(stage.first)}`;
    else if (stage && stage.kind === 'checkpoint') text = 'PAUSED // MANUAL LID CHECK';
    else if (acts.length && g >= acts[acts.length - 1].t1) text = 'SEQUENCE COMPLETE';
    S.live[side] = text;
    if ((a ? a.row : null) !== S.rows[side]) { S.rows[side] = a ? a.row : null; paintArm(side); }
  }
  S.status = stage ? `SIMULATION // ${stage.kind === 'serial' ? `Program ${stage.label} // ${ARM(stage.first)} RUN FIRST` : stage.label || stage.kind.toUpperCase()}`
    + `${stage.kind === 'parallel' ? ' // LEFT + RIGHT TOGETHER' : ''}` : S.status;
  document.querySelectorAll('.lite-s2-stage').forEach((n) => n.classList.toggle('active', stage && Number(n.dataset.i) === stage.index));
  for (const side of ARMS) $(`liteS2Live_${side}`).textContent = `${ARM(side)} // ${S.live[side]}`;
  $('liteS2StatusText').textContent = S.status;
}

async function simEnded() {
  const sim = S.sim;
  if (!sim || !view.clean) { $('liteS2SimState').className = 'lite-status warn'; $('liteS2SimState').textContent = 'Played with a skip: play it from the start to the end for Real Machine Mode.'; return; }
  try {
    const out = await post(`${base()}/played`, { hash: sim.hash });
    sim.completed = out.completed;
    $('liteS2SimState').className = 'lite-status ok';
    $('liteS2SimState').textContent = 'Played to its end: Real Machine Mode can run this sequence as it is.';
    window.toast('Simulation complete: Real Machine Mode can run this sequence as it is.');
  } catch (err) { tell(err); }
  paintAll();
}

function paintStages() {
  const sim = S.sim, box = $('liteS2Stages');
  const words = { parallel: 'LEFT + RIGHT together', serial: 'one arm at a time', checkpoint: 'manual checkpoint', tail: 'final steps', single: 'one arm' };
  box.replaceChildren(el('h4', { text: 'Stages (V53\'s SAFE TIMELINE plan)' }), ...sim.stages.map(s => el('button', {
    class: 'lite-s2-stage', type: 'button', 'data-i': String(s.index), onclick: () => { player.pause(); seekSim(s.t0); },
    text: `${s.index + 1}. ${s.kind === 'serial' ? `Program ${s.label}` : s.label || s.kind.toUpperCase()} · ${words[s.kind] || s.kind}${s.kind === 'serial' ? ` (${ARM(s.first)} first)` : ''} · ${fmt(s.t0)}–${fmt(s.t1)}` })));
}

function paintFindings() {
  const sim = S.sim, box = $('liteS2Findings'), col = sim.collisions;
  const pair = (p) => `${p.link} – ${p.object}${p.depth_mm ? ` (${p.depth_mm} mm)` : ''}`;
  const where = (e) => (e.at || []).filter(Boolean).map(w => `${ARM(w.side)} row ${w.row} (${w.text}${w.line ? `, line ${w.line.toLocaleString()}` : ''})`).join('; ');
  const c = Object.entries(col.contact || {}).filter(([, v]) => v.samples).map(([s, v]) => `${ARM(s)} hand on the commode from ${fmt(v.first_t)} (deepest ${v.deepest_mm} mm)`);
  box.replaceChildren(...[
    el('h4', { text: 'Collision check' }),
    el('div', { class: `lite-status ${col.ok ? 'ok' : 'err'}`, text: col.ok
      ? `No collision: ${col.samples.toLocaleString()} moments checked, both arms against ${col.checked.walls}, the commode and each other.`
      : `Collision at ${fmt(col.first.first_t)}: ${col.first.pairs.map(pair).join('; ') || col.first.what.join(', ')}. ${where(col.first)}. `
        + `${col.events.length + col.more} event(s).` }),
    c.length ? el('div', { class: 'mini-note', text: `Contact (the cleaning; up to ${col.checked.contact_depth_mm} mm is the pad's give): ${c.join('; ')}.` }) : null,
    ...col.events.slice(0, 6).map(e => el('div', { class: 'lite-rev3d-event' },
      el('span', { text: `${fmt(e.first_t)}–${fmt(e.last_t)}: ${e.pairs.map(pair).join('; ') || e.what.join(', ')} · ${where(e)}` }),
      el('button', { class: 'btn light', type: 'button', text: 'Show', onclick: () => { player.pause(); seekSim(e.first_t); } }))),
    el('h4', { text: 'Before it runs on the robot' }),
    sim.findings.length ? el('ul', { class: 'lite-s2-findlist' }, sim.findings.map(f => el('li', { class: f.level === 'warn' ? 'lite-warn' : 'mini-note', text: f.text })))
      : el('div', { class: 'lite-status ok', text: 'Every Program is drawn, and its pole and hand are the ones it was taught with.' }),
    el('div', { class: 'mini-note', text: `Times are estimated: the moves at their speed % of each joint's rating, the programs on their stored 5 ms timing. `
      + `The arms start ${Object.values(sim.start_from || {}).includes('now') ? 'where they stand now' : 'from the standing pose'}. Nothing is blocked by this check: `
      + 'Real Machine Mode asks about what it found, once.' })].filter(Boolean));
  const marks = $('liteS2Marks'), T = Math.max(1e-6, sim.total_s);
  marks.replaceChildren(...col.events.map(e => el('span', { class: 'lite-rev3d-mark', title: `${fmt(e.first_t)}`,
    style: `left:${(100 * e.first_t) / T}%;width:${Math.max(0.4, (100 * (e.last_t - e.first_t)) / T)}%` })),
  ...sim.stages.filter(s => s.kind === 'checkpoint').map(s => el('span', { class: 'lite-s2-checkmark', title: s.label, style: `left:${(100 * s.t0) / T}%` })));
}

async function setView(name) {
  try { const { ensure3D } = await import('./sim.js'); const { viewer } = await ensure3D(); viewer.setView(name); } catch (_) { /* no 3D */ }
}

export function openS2(host) {
  if (panel !== host) { panel = host; built = false; }
  build();
  if (!ctx.pid) { S.status = 'PAGE 02 // open a project first'; paintAll(); return; }
  load().then(() => refreshSources(false)).then(async () => {
    if (S.sim && S.mode === 'sim' && !view.three) { if (await prefetch()) showAt(player.t); }
  });
}

document.addEventListener('lite:speed', () => { if (built) paintAll(); });
