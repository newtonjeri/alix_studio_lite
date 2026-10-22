// Merge + Sequence, S2: Sequence (feedback v5, slide 9; the owner, 16 Oct; was Full & Partial Sequence): the
// control program's Page 02 (V53 with runner v14) in Lite's style, its layout as it is (server/workflow.py):
// - each arm's steps: PROGRAM (a stored program, its Start %: the controller's move to its first
//   point), GLOBAL POINT (MoveJ / MoveL, speed, wait), HAND / POLE / WAIT; the table with Up, Down,
//   Duplicate, Remove, Clear, Save and Load (another arm's file is refused); RUN and STOP per arm;
// - RUN BOTH, STOP BOTH, the overall speed (the speed slider at the top), the status and the live
//   status of each arm.
// How the arms go together (the owner, 22 Oct; server/flow.py): each arm's rows run top to bottom, both arms
// starting together, and an arm waits only where its list says: at a MEET (both arms meet there, the first to
// arrive waits; the Meets pair up in order), at a WAIT FOR (until the other arm has finished the row it names)
// and at a lid or seat task. V53's Dual Rule and its SAFE TIMELINE are gone; a V53 file loads with its handoffs as
// Meets and a Wait-for.
// What Lite adds: Simulation Mode or Real Machine Mode; what to run (the owner, 16 Oct: "options to play
// a single task arm, dual arm task and sequences in both simulation and real"; Full and Partial are gone):
// the sequences, a single task (a Program row on its arm, with the Pole and Hand in force before it) or a
// dual-arm task (one part between two Meets with both arms' Programs, or a lid or seat task); Simulation Mode
// plays the run in 3D with the collision check, checks the two arms 50 mm apart at every timing the robot's
// ranges allow and says what would not be right on the robot; Real Machine Mode runs only what completed in
// Simulation as it is.

import { call, get, post, put, say } from './api.js';
import { REAL_CHECKS, confirmDialog, ctx, el, fmtT, jobLine, robotCall, tell, watchJob } from './ctx.js';
import { borrow, home3d, Player } from './seq3d.js';
import { scene } from './sim.js';
import { speedPct } from './status.js';
import { saveBlobAs } from './tune.js';
import { checkJob, spansOf, Tools } from './view3d.js';
import { choose } from './picker.js';
import { frame } from './vp.js';

const $ = (id) => document.getElementById(id);
const ARMS = ['left', 'right'];
const ARM = (s) => s.toUpperCase();
const OTHER = { left: 'right', right: 'left' };
const SCOPES = [['sequence', 'Sequences'], ['task', 'Single task'], ['dual', 'Dual-arm task']];
const GRIP = [0, 0, 0, 0, 150, 1000], OPEN = [1000, 1000, 1000, 1000, 1000, 1000];
const CLEANING = [1000, 1000, 1000, 1000, 0, 1000];          // the Cleaning Pose (the owner, 15 Oct; Prepare's preset)
const S = {
  steps: { left: [], right: [] }, sources: { left: { programs: [], points: [] }, right: { programs: [], points: [] } },
  sel: { left: null, right: null }, mode: 'sim', scope: 'sequence', pick: null, edited: false,
  sim: null, job: null, live: { left: 'IDLE', right: 'IDLE' }, status: 'PAGE 02 READY // Build LEFT and RIGHT sequences', runtime: 0,
  rows: { left: null, right: null }, mechanisms: {},
};
const view = { frames: new Map(), shown: { left: null, right: null }, phase: null, three: null, clean: false, furthest: 0, token: 0 };
let panel = null, built = false, player = null, saving = { left: null, right: null }, tools = null;

const base = () => `/api/projects/${ctx.pid}/workflow`;
const fmt = (s) => `${Number(s || 0).toFixed(1)} s`;

// ── the steps as V53 shows them (WorkflowSequencePanel._row), with Lite's Meet and Wait-for rows

// A row's own id (flow.with_uids): a Wait-for names the row it waits for by it, so that it follows the row when rows move.
function newUid() {
  return Array.from(crypto.getRandomValues(new Uint8Array(4)), b => b.toString(16).padStart(2, '0')).join('');
}

// A row in a few words (a Wait-for's target, its picker).
function brief(st) {
  const k = st.type;
  if (k === 'program') return `Program ${st.id}${st.name ? ` // ${st.name}` : ''}`;
  if (k === 'pole') return `Pole ${Number(st.mm)} mm`;
  if (k === 'hand') return `Hand ${st.values.join(',')}`;
  if (k === 'wait') return `Wait ${Number(st.seconds)} s`;
  if (k === 'mechanism') return (S.mechanisms[st.task] || {}).title || st.task;
  if (k === 'meet') return 'Meet';
  if (k === 'wait_for') return 'Wait for';
  return `${String(st.move).toUpperCase()} ${st.name}`;
}

// The row of the other arm a Wait-for waits for (its uid; a V53 file's: the other arm's first row of that Program); -1:
// not there (removed).
function targetOf(side, st) {
  const list = S.steps[OTHER[side]], tg = st.target || {};
  if (tg.uid) return list.findIndex(x => x.uid === tg.uid);
  if (tg.program != null) return list.findIndex(x => x.type === 'program' && Number(x.id) === Number(tg.program));
  return -1;
}

// A Meet's partner: the other arm's Meet of the same number (the Meets pair up in order); -1: none.
function partnerOf(side, i) {
  const n = S.steps[side].slice(0, i + 1).filter(x => x.type === 'meet').length;
  let seen = 0;
  return S.steps[OTHER[side]].findIndex(x => x.type === 'meet' && ++seen === n);
}

// Why a Meet or Wait-for row cannot run as it is (marked in its table), or ''.
function gateProblem(side, i) {
  const st = S.steps[side][i], other = ARM(OTHER[side]);
  if (st.type === 'wait_for' && targetOf(side, st) < 0) return `The ${other} row it waited for is no longer there: choose the row again (remove this one, add WAIT FOR).`;
  if (st.type === 'meet' && partnerOf(side, i) < 0) return `No Meet of the same number on the ${other} arm: add one there, or remove this one.`;
  return '';
}

function row(step) {
  const k = step.type;
  if (k === 'program') {
    const known = (S.sources[step._side]?.programs || []).find(p => p.id === step.id);
    const name = step.name || (known && known.name) || '';
    const t = known && known.seconds != null ? ` (${fmtT(known.seconds)})` : '';     // its execution time, after its name
    return [k.toUpperCase(), `Program ${step.id}${name ? ` // ${name}` : ''}${t}`,
      `Start ${step.transit_speed ?? step.start_speed ?? 30}% // Stored timing`];
  }
  if (k === 'meet') {
    const side = step._side, n = S.steps[side].slice(0, step._i + 1).filter(x => x.type === 'meet').length, j = partnerOf(side, step._i);
    return ['MEET', `Meet ${n}${j >= 0 ? ` // with ${ARM(OTHER[side])} row ${j + 1}` : ` // no Meet ${n} on the ${ARM(OTHER[side])} arm`}`,
      'Both arms meet here: the first to arrive waits'];
  }
  if (k === 'wait_for') {
    const side = step._side, j = targetOf(side, step), other = OTHER[side];
    return ['WAIT FOR', j >= 0 ? `${ARM(other)} row ${j + 1} // ${brief(S.steps[other][j])}` : `${ARM(other)} row no longer there`,
      `Until the ${ARM(other)} arm has finished it${step.note ? ` // ${step.note}` : ''}`];
  }
  if (k === 'pole') return ['POLE', `${Number(step.mm)} mm`, ''];
  if (k === 'hand') return ['HAND', step.values.join(','), 'L,R,M,I,TB,TR'];
  if (k === 'wait') return ['WAIT', `${Number(step.seconds)} s`, step.note || ''];
  if (k === 'mechanism') {                                   // a lid or seat task of ALIX's (mechanism.py), on both arms
    const t = S.mechanisms[step.task] || {};
    return ['LID / SEAT', `${t.short || step.task} // ${t.title || ''}${t.seconds != null ? ` (${fmtT(t.seconds)})` : ''}`,
      'Pre-recorded (ALIX) // both arms // camera checks'];
  }
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
    el('div', { class: 'lite-s2-subtabs', role: 'tablist' }, tab('program', 'PROGRAM'), tab('global', 'GLOBAL POINT'), tab('aux', 'HAND / POLE / WAIT'),
      tab('gate', 'MEET / WAIT FOR'), tab('lid', 'LID / SEAT')),
    el('div', { id: `liteS2Add_${side}_program`, class: 'lite-s2-add' },
      el('div', { class: 'lite-row' },
        el('div', { class: 'field lite-grow' }, el('label', { for: `liteS2_${side}_prog`, text: 'Stored Program' }),
          el('div', { class: 'inline' },
            el('input', { id: `liteS2_${side}_prog`, type: 'text', list: `liteS2Progs_${side}`, autocomplete: 'off', placeholder: 'ID, e.g. 71' }),
            btn('Choose', (e) => pickInto(side, 'prog', e.currentTarget), 'btn light')),
          el('datalist', { id: `liteS2Progs_${side}` })),
        el('div', { class: 'field' }, el('label', { for: `liteS2_${side}_start`, text: 'Start %' }), num('start', 30, { min: '1', max: '100', step: '1' })),
        btn('+ ADD PROGRAM', () => addProgram(side), 'btn green')),
      el('div', { class: 'mini-note', text: 'Start % = speed used only to reach this Program\'s first pose (default 30%). Stored trajectory timing is '
        + 'unchanged. Where this arm waits for the other: the MEET / WAIT FOR tab.' })),
    el('div', { id: `liteS2Add_${side}_global`, class: 'lite-s2-add', hidden: true },
      el('div', { class: 'lite-row' },
        el('div', { class: 'field lite-grow' }, el('label', { for: `liteS2_${side}_point`, text: 'Global Point' }),
          el('div', { class: 'inline' },
            el('input', { id: `liteS2_${side}_point`, type: 'text', list: `liteS2Points_${side}`, autocomplete: 'off' }),
            btn('Choose', (e) => pickInto(side, 'point', e.currentTarget), 'btn light')),
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
        btn('CLEANING POSE', () => setHand(side, CLEANING), 'btn violet'),
        btn('+ ADD HAND', () => addHand(side), 'btn green'))),
    el('div', { id: `liteS2Add_${side}_gate`, class: 'lite-s2-add', hidden: true },
      el('div', { class: 'lite-row' },
        btn('+ MEET (BOTH ARMS)', () => addMeet(), 'btn green'),
        el('div', { class: 'field lite-grow' }, el('label', { for: `liteS2_${side}_target`, text: `Wait for the ${ARM(OTHER[side])} arm's row` }),
          el('select', { id: `liteS2_${side}_target` })),
        btn('+ WAIT FOR', () => addWaitFor(side), 'btn')),
      el('div', { class: 'mini-note', text: 'MEET: both arms meet there; the first to arrive waits for the other, then both go on. It goes on both '
        + 'arms, after the row chosen in each table (at the end of a table with none chosen); the Meets pair up in order, each arm\'s first '
        + `with the other's first. WAIT FOR: this arm waits there until the ${ARM(OTHER[side])} arm has finished the row chosen above, `
        + `while the ${ARM(OTHER[side])} arm goes on; it goes after the row chosen in this table. Anywhere else each arm goes on with its `
        + 'own rows.' })),
    el('div', { id: `liteS2Add_${side}_lid`, class: 'lite-s2-add', hidden: true },
      el('div', { class: 'lite-row' },
        el('div', { class: 'field lite-grow' }, el('label', { for: `liteS2_${side}_task`, text: 'Lid or seat task (pre-recorded)' }),
          el('select', { id: `liteS2_${side}_task`, onchange: () => paintTask(side) })),
        btn('+ ADD TO BOTH ARMS', () => addMechanism(side), 'btn green')),
      el('div', { id: `liteS2TaskNote_${side}`, class: 'mini-note' }),
      el('div', { class: 'lite-row' },
        el('div', { class: 'field lite-grow' }, el('label', { for: `liteS2_${side}_pose`, text: "ALIX's inspection pose (the right arm carries the camera)" }),
          el('select', { id: `liteS2_${side}_pose` })),
        btn('MOVE RIGHT ARM', () => movePose(side), 'btn light'),
        btn('CHECK THE CAMERA', () => cameraCheck(side), 'btn light')),
      el('div', { id: `liteS2Camera_${side}`, class: 'lite-s2-camera' })),
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
        el('p', { class: 'mini-note', text: 'Mix stored Programs, Pole, 6-channel Hand, Wait, and Global Points (MoveJ / MoveL + speed) in any order. '
          + 'Both arms start together and each goes through its own rows; an arm waits for the other only at a Meet, a Wait-for or a lid '
          + 'or seat task. After a lid or seat task (or several in a row) each arm\'s pole and hand go back to what they were before '
          + 'it, and the rows after start on both arms together. A Wait is a task of its own: its arm idles for its seconds, on one arm or both. '
          + 'Before a Global Point or a lid or seat task that follows a Program, Lite clears the 4105 warning (a joint over its speed '
          + 'limit) when the arm holds one: the controller refuses that move\'s arrival while it is latched.' })),
      el('div', { class: 'lite-row' }, btn('Refresh Programs', () => refreshSources(true)), btn('Refresh Global Points', () => refreshSources(true)))),
    el('div', { class: 'card lite-s2-exec' },
      el('div', { class: 'lite-s2-modes', role: 'radiogroup', 'aria-label': 'Execution mode' },
        el('b', { text: 'Execution mode' }),
        btn('Simulation Mode', () => setMode('sim'), 'btn light lite-s2-mode', { id: 'liteS2Mode_sim', role: 'radio' }),
        btn('Real Machine Mode', () => setMode('real'), 'btn light lite-s2-mode', { id: 'liteS2Mode_real', role: 'radio' }),
        el('span', { id: 'liteS2Gate', class: 'lite-status' })),
      el('div', { class: 'lite-s2-modes', role: 'radiogroup', 'aria-label': 'What to run' },
        el('b', { text: 'What to run' }),
        ...SCOPES.map(([id, name]) => btn(name, () => setScope(id), 'btn light lite-s2-mode', { id: `liteS2Scope_${id}`, role: 'radio' })),
        el('button', { id: 'liteS2Pick', class: 'lite-picker-btn lite-s2-pick', type: 'button', hidden: true, 'aria-haspopup': 'listbox',
          'aria-expanded': 'false', onclick: (e) => chooseTask(e.currentTarget) }, el('span', { id: 'liteS2PickText', class: 'lite-picker-value' })))),
    el('div', { class: 'card lite-s2-control' },
      el('div', { class: 'lite-s2-kicker', text: 'DUAL SEQUENCE CONTROL' }),
      el('div', { class: 'lite-row' },
        btn('RUN BOTH', () => run('both'), 'btn green', { id: 'liteS2RunBoth' }),
        btn('STOP BOTH SEQUENCES', () => stop('both'), 'btn red', { id: 'liteS2StopBoth' }),
        el('span', { class: 'lite-s2-speed' }, 'OVERALL SPEED % ', el('b', { id: 'liteS2Speed', text: '50' }),
          el('small', { class: 'mini-note', text: ' (the speed slider at the top)' })),
        el('span', { id: 'liteS2StatusText', class: 'lite-s2-statustext lite-grow' }),
        el('b', { id: 'liteS2Runtime', class: 'lite-s2-runtime', text: '0.0 s' }))),
    el('div', { class: 'lite-s2-kicker', text: 'LIVE ARM STATUS' }),
    el('div', { class: 'lite-s2-live' }, ARMS.map(side => el('div', { id: `liteS2Live_${side}`, class: 'lite-s2-livearm', text: `${ARM(side)} // IDLE` }))),
    el('div', { class: 'lite-s2-arms' }, ARMS.map(armPanel)),
    el('div', { id: 'liteS2Sim', class: 'card lite-s2-sim' },
      el('div', { class: 'lite-section-title' },
        el('h3', { text: 'Simulation' })),
      el('div', { class: 'lite-s2-simgrid' },
        el('div', { class: 'lite-rev3d-main' }, el('div', { id: 'liteS2Host', class: 'lite-rev3d-host' })),
        el('div', { class: 'lite-rev3d-side' },
          el('div', { class: 'lite-row' }, btn('Play', () => playSim(), 'btn'), btn('Pause', () => player.pause(), 'btn outline'),
            btn('Restart', () => restartSim()),
            el('select', { id: 'liteS2Speedup', 'aria-label': 'Replay speed', onchange: (e) => player.setSpeed(Number(e.target.value)) },
              [1, 2, 4].map(x => el('option', { value: String(x), text: `${x}x` })))),
          el('div', { class: 'lite-rev3d-track' },
            el('input', { id: 'liteS2Timeline', type: 'range', min: '0', max: '0', step: '10', value: '0', 'aria-label': 'Sequence time',
              oninput: (e) => { player.pause(); seekSim(Number(e.target.value) / 1000); } }),
            el('div', { id: 'liteS2Marks', class: 'lite-rev3d-marks', 'aria-hidden': 'true' })),
          el('div', { id: 'liteS2SimState', class: 'lite-status' }),
          el('div', { id: 'liteS2Stages', class: 'lite-s2-stages' }),
          el('div', { id: 'liteS2Findings', class: 'lite-s2-findings' })))));
  player = new Player({ onTime: showAt, onEnd: simEnded });
  for (const side of ARMS) subtab(side, 'program');
  // the view's tools (view3d.js): the commode as each Program's region has it, or one state chosen here
  // (the run's own check stays the run's; another state is checked on demand), the links in red
  tools = new Tools({
    what: 'the sequence',
    check: async (phase, progress) => {
      if (!S.sim) throw new Error('Run the sequence in Simulation Mode first.');
      if (!phase) return { result: S.sim.collisions, spans: spansOf(S.sim.collisions) };
      const r = await checkJob(post, `${base()}/collisions`, { hash: S.sim.hash, phase }, progress);
      return { result: r, spans: spansOf(r) };
    },
    seek: (t) => { player.pause(); seekSim(t); },
    changed: () => { view.phase = null; showAt(player.t); },
  }).mount($('liteS2Host'));
  tools.track($('liteS2Timeline').parentElement);
  frame($('liteS2Host')).title('The robot at home', 'Run the sequence in Simulation Mode to play it here');
  tools.load(null, { phase: null, text: "each Program's region", enabled: false });
}

function subtab(side, id) {
  for (const t of ['program', 'global', 'aux', 'gate', 'lid']) {
    $(`liteS2Add_${side}_${t}`).hidden = t !== id;
    $(`liteS2Tab_${side}_${t}`).classList.toggle('active', t === id);
    $(`liteS2Tab_${side}_${t}`).setAttribute('aria-selected', String(t === id));
  }
}

// ── data

// The lid and seat tasks (ALIX's, pre-recorded: the owner, 18 Oct, "not subjected to teaching procedure ... tasks
// that are available to be injected at a point in the sequence"): what each does, for the LID / SEAT tab.
async function loadMechanisms() {
  if (Object.keys(S.mechanisms).length) return;
  try {
    const d = await get('/api/mechanism');
    S.mechanisms = Object.fromEntries(d.tasks.map(t => [t.name, t]));
    S.poses = d.poses || [];                       // ALIX's inspection poses of the right arm (1 colour, 4 depth)
    for (const side of ARMS) {
      const sel = $(`liteS2_${side}_task`);
      if (sel) sel.replaceChildren(...d.tasks.map(t => el('option', { value: t.name, text: `${t.short} // ${t.title}` })));
      const ps = $(`liteS2_${side}_pose`);
      if (ps) {
        ps.replaceChildren(...S.poses.map(p => el('option', { value: p.name, text: `${p.title} // ${p.short}` })));
        if (S.poses.some(p => p.name === 'inspection_pose_4')) ps.value = 'inspection_pose_4';
      }
      paintTask(side);
    }
    paintAll();
  } catch (err) { tell(err); }
}

function paintTask(side) {
  const t = S.mechanisms[$(`liteS2_${side}_task`)?.value];
  const note = $(`liteS2TaskNote_${side}`);
  if (!note || !t) return;
  note.textContent = `${t.does} From ${t.start} to ${t.end}; about ${Math.round(t.seconds)} s of motion. ALIX's plan, made with the UGV at `
    + `${t.standoff_mm} mm from the commode and no offset. It uses both arms: it is added to both lists, and both arms finish what is `
    + 'before it, then it runs. In Real the wrist camera checks the lid and the seat; in Simulation the checks are assumed.';
}

async function load() {
  loadMechanisms();
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

// The arm's stored programs and its controller's global points as a searchable list (picker.js, the
// owner, 16 Oct): the choice fills the field, which can still be typed in.
function pickInto(side, what, anchor) {
  const input = $(`liteS2_${side}_${what}`), src = S.sources[side];
  const label = (p) => `${p.id}${p.name ? ` // ${p.name}` : ''}`;
  const groups = what === 'point'
    ? [{ label: src.points_read ? `${ARM(side)} controller · ${src.points.length} global points (read ${src.points_read})`
      : `${ARM(side)}: global points not read (connect the arm, then refresh)`, rows: src.points.map(n => ({ value: n, text: n })) }]
    : [{ label: `${ARM(side)} · ${src.programs.length} stored programs`, rows: src.programs.map(p => ({ value: label(p), text: label(p),
      detail: (p.sent ? `sent by Lite${p.file_name ? `: ${p.file_name}` : ''}` : p.gone ? 'no longer on the controller' : 'on the controller')
        + (p.seconds != null ? ` · ${fmtT(p.seconds)}` : '') })) }];
  choose(anchor, groups, input.value, (v) => { input.value = v; input.focus(); },
    { placeholder: what === 'point' ? 'Search the global points' : 'Search the programs: ID or name', files: what !== 'point' });
}

// The programs and the global points to choose from are the controller's, as Data Management lists them (the
// owner, 17 Oct): a program or a point deleted on the controller is gone from them at the next read. Refresh
// (`said`) reads the controllers again, completely; a field that names something no longer on its controller is
// cleared, and a step that uses one is marked in its table (it is kept: the sequence is the operator's).
async function refreshSources(said = false) {
  const cleared = [];
  await Promise.all(ARMS.map(async (side) => {
    try { S.sources[side] = await get(`${base()}/${side}/sources${said ? '?refresh=1' : ''}`); } catch (_) { /* the lists stay */ }
    const src = S.sources[side];
    const pl = $(`liteS2Progs_${side}`);
    if (pl) pl.replaceChildren(...src.programs.map(p => el('option', { value: `${p.id}${p.name ? ` // ${p.name}` : ''}`,
      text: (p.sent ? `sent by Lite${p.file_name ? `: ${p.file_name}` : ''}` : 'on the controller') + (p.seconds != null ? ` · ${fmtT(p.seconds)}` : '') })));
    const pp = $(`liteS2Points_${side}`);
    if (pp) pp.replaceChildren(...src.points.map(n => el('option', { value: n })));
    if (!known(side)) return;                                  // nothing read yet: nothing is judged
    const prog = $(`liteS2_${side}_prog`), pid = /^\s*(\d+)/.exec(prog?.value || '');
    if (pid && !src.programs.some(p => p.id === Number(pid[1]))) { cleared.push(`${ARM(side)} Program ${pid[1]}`); prog.value = ''; }
    const point = $(`liteS2_${side}_point`), name = (point?.value || '').trim();
    if (name && !src.points.includes(name)) { cleared.push(`${ARM(side)} global point ${name}`); point.value = ''; }
  }));
  paintAll();
  if (said) {
    for (const side of ARMS) document.dispatchEvent(new CustomEvent('lite:controller', { detail: { side, from: 's2' } }));
    const gone = ARMS.flatMap(side => missing(side).map(m => `${ARM(side)} row ${m.row} (${m.what})`));
    window.toast(`Read from the controllers: programs LEFT ${S.sources.left.programs.length}, RIGHT ${S.sources.right.programs.length}; global points `
      + ARMS.map(s => (S.sources[s].points_read ? `${ARM(s)} ${S.sources[s].points.length}` : `${ARM(s)} not read (connect the arm)`)).join(', ') + '.'
      + (cleared.length ? ` Cleared (no longer on the controller): ${cleared.join(', ')}.` : '')
      + (gone.length ? ` Not on the controller: ${gone.join(', ')}.` : ''), cleared.length || gone.length ? 'warn' : 'good');
  }
}

// Whether the arm's controller list was read (live, or as last read): only then is a step judged missing.
const known = (side) => !!(S.sources[side] && (S.sources[side].read || S.sources[side].standin));

// The steps of an arm that use a program or a global point its controller no longer holds.
function missing(side) {
  if (!known(side)) return [];
  const src = S.sources[side], out = [];
  S.steps[side].forEach((st, i) => {
    if (st.type === 'program' && !src.programs.some(p => p.id === Number(st.id))) out.push({ row: i + 1, what: `Program ${st.id}` });
    if (st.type === 'global' && src.points_read && !src.points.includes(st.name)) out.push({ row: i + 1, what: `global point ${st.name}` });
  });
  return out;
}

// Each arm's steps go to the server as they are edited, one save at a time; the server's reply (its checked
// form) is taken only when nothing was edited since, so a slow reply never undoes a newer edit.
const edits = { left: 0, right: 0 };
const queue = { left: Promise.resolve(), right: Promise.resolve() };

// The task chosen (Single task, Dual-arm task) after an edit: a row or a stage that no longer holds the Programs
// it was chosen with is cleared, and said, so the run and its one confirmation never name another Program.
function recheckPick() {
  const k = S.pick;
  if (!k) return;
  let still = true;
  if (S.scope === 'task') {
    const st = S.steps[k.side][k.row - 1];
    still = !!st && st.type === 'program' && String(st.id) === String(k.id);
  } else if (S.scope === 'dual') still = false;            // its stages are counted again: chosen again
  if (still) return;
  S.pick = null;
  window.toast(S.scope === 'task' ? 'The steps changed: choose the task again (its row no longer holds that Program).'
    : 'The steps changed: choose the dual-arm task again.', 'warn');
}

function saveSteps(side) {
  edits[side] += 1;
  S.edited = true;                                           // not as simulated any more (the server says what changed)
  recheckPick();
  clearTimeout(saving[side]);
  saving[side] = setTimeout(() => {
    saving[side] = null;
    const mine = edits[side];
    queue[side] = queue[side].then(async () => {
      try {
        const d = await put(`${base()}/${side}`, { steps: S.steps[side] });
        if (edits[side] === mine) S.steps[side] = d[side];
      } catch (err) {
        tell(err);
        try { if (edits[side] === mine) S.steps[side] = (await get(base()))[side]; } catch (_) { /* as it was */ }
      }
      paintAll();
    });
  }, 250);
}

// Every edit on its way to the server, there (a run uses the steps the server has).
async function saved() {
  const failed = [];
  for (const side of ARMS) {
    if (saving[side]) {
      clearTimeout(saving[side]);
      saving[side] = null;
      const mine = edits[side];
      queue[side] = queue[side].then(async () => {
        const d = await put(`${base()}/${side}`, { steps: S.steps[side] });
        if (edits[side] === mine) S.steps[side] = d[side];
      }).catch((err) => { failed.push(err); });           // said by the caller; the next save still goes
    }
  }
  await Promise.all(ARMS.map(side => queue[side]));
  if (failed.length) throw failed[0];
}

// ── editing (V53's WorkflowSequencePanel)

const val = (side, id) => $(`liteS2_${side}_${id}`)?.value?.trim() ?? '';

function append(side, step) {
  S.steps[side].push({ ...step, uid: newUid() });
  S.sel[side] = S.steps[side].length - 1;
  paintAll();
  saveSteps(side);
}

// A Meet or a Wait-for row after the arm's chosen row (at its end when none is chosen): where it is placed matters
// against the other arm's rows. Its row number, from 1.
function insertGate(side, step) {
  const i = S.sel[side] == null ? S.steps[side].length : S.sel[side] + 1;
  S.steps[side].splice(i, 0, { ...step, uid: newUid() });
  S.sel[side] = i;
  paintAll();
  saveSteps(side);
  return i + 1;
}

function addMeet() {
  const rows = ARMS.map(side => insertGate(side, { type: 'meet' }));
  window.toast(`Meet added: LEFT row ${rows[0]}, RIGHT row ${rows[1]}. Both arms meet there (UP / DOWN move it in each list).`, 'good');
}

function addWaitFor(side) {
  const other = OTHER[side], uid = val(side, 'target'), j = S.steps[other].findIndex(x => x.uid === uid);
  if (!uid || j < 0) { window.toast(`${ARM(side)}: choose the ${ARM(other)} row to wait for.`, 'warn'); return; }
  const row = insertGate(side, { type: 'wait_for', target: { uid } });
  window.toast(`${ARM(side)} row ${row}: waits until the ${ARM(other)} arm has finished its row ${j + 1} (${brief(S.steps[other][j])}).`, 'good');
}

// Each arm's WAIT FOR picker: the other arm's rows (not its Meet and Wait-for rows), the choice kept while its row is.
function paintTargets() {
  for (const side of ARMS) {
    const sel = $(`liteS2_${side}_target`);
    if (!sel) continue;
    const was = sel.value, other = OTHER[side];
    const opts = S.steps[other].map((st, i) => (st.uid && st.type !== 'meet' && st.type !== 'wait_for'
      ? el('option', { value: st.uid, text: `Row ${i + 1} // ${brief(st)}` }) : null)).filter(Boolean);
    sel.replaceChildren(...(opts.length ? opts : [el('option', { value: '', text: `The ${ARM(other)} list has no rows yet` })]));
    if (opts.some(o => o.value === was)) sel.value = was;
  }
}

function addProgram(side) {
  const m = /^\s*(\d+)/.exec(val(side, 'prog'));
  if (!m) { window.toast(`${ARM(side)}: choose or enter a Program ID.`, 'warn'); return; }
  const id = Number(m[1]);
  if (!(id >= 1 && id <= 100)) { window.toast('Program ID must be 1-100.', 'warn'); return; }
  let start = Math.trunc(Number(val(side, 'start')));
  if (!Number.isFinite(start)) start = 30;
  if (!(start >= 1 && start <= 100)) { window.toast('Start / Transit speed must be 1-100%.', 'warn'); return; }
  const known = S.sources[side].programs.find(p => p.id === id);
  append(side, { type: 'program', id, name: (known && known.name) || '', speed: 100, transit_speed: start });
}

function addGlobal(side) {
  const name = val(side, 'point');
  if (!name) { window.toast(`${ARM(side)}: choose or enter a global point name.`, 'warn'); return; }
  const speed = Math.trunc(Number(val(side, 'gspeed'))), wait = Number(val(side, 'gwait') || 0);
  if (!Number.isFinite(speed) || !Number.isFinite(wait)) { window.toast('Speed and Wait must be numeric.', 'warn'); return; }
  if (!(speed >= 1 && speed <= 100)) { window.toast('Speed must be 1-100%.', 'warn'); return; }
  if (!(wait >= 0 && wait <= 3600)) { window.toast('Wait must be 0-3600 seconds.', 'warn'); return; }
  append(side, { type: 'global', name, move: val(side, 'move') === 'movel' ? 'movel' : 'movej', speed, wait_s: wait });
}

function addPole(side) {
  const mm = Number(val(side, 'pole'));
  if (!(mm >= 0 && mm <= 315) || val(side, 'pole') === '') { window.toast('Pole height must be 0-315 mm.', 'warn'); return; }
  append(side, { type: 'pole', mm });
}

function addWait(side) {
  const s = Number(val(side, 'wait'));
  if (!(s >= 0 && s <= 3600) || val(side, 'wait') === '') { window.toast('Wait must be 0-3600 seconds.', 'warn'); return; }
  append(side, { type: 'wait', seconds: s });
}

function setHand(side, values) { values.forEach((v, i) => { $(`liteS2_${side}_h${i}`).value = String(v); }); }

function addHand(side) {
  const values = [0, 1, 2, 3, 4, 5].map(i => Math.trunc(Number(val(side, `h${i}`))));
  if (values.some(v => !Number.isFinite(v) || v < 0 || v > 1000)) { window.toast('All six Hand values must be 0-1000.', 'warn'); return; }
  append(side, { type: 'hand', values });
}

// What the wrist camera sees now, as ALIX's perception node's status_check answers it (the colour model of the
// commode's state, the depth state deciding when its margin is clear): nothing moves (mechanism.camera_check).
async function cameraCheck(side) {
  const box = $(`liteS2Camera_${side}`);
  box.replaceChildren(el('div', { class: 'mini-note', text: 'Asking the wrist camera...' }));
  try {
    const r = await post('/api/mechanism/camera/check', {});
    const words = (x) => (x ? String(x).replace(/_/g, ' ') : 'none');
    const said = r.ok
      ? `The camera sees: ${words(r.state)} (${Math.round(r.confidence * 100)} %, decided by ${r.source || 'nothing'}; `
        + `colour model: ${words(r.colour_state)}, depth state: ${words(r.depth_state)}).`
      : `The camera could not tell: ${r.message}`;
    const where = !r.posed ? ' The right arm is not linked, so the colour model answers alone (no depth state).'
      : r.pose ? ` The right arm is at ALIX's ${words(r.pose)}.` : ' The right arm is not at one of ALIX\'s named poses.';
    const gate = r.gate && r.gate !== 'depth_state_poses empty' ? ` Depth state gate: ${r.gate}.` : '';
    box.replaceChildren(el('div', { class: `lite-status ${r.ok ? 'ok' : 'warn'}`, text: said + where + gate }),
    r.picture ? el('img', { src: r.picture, alt: 'What the wrist camera sees', class: 'lite-s2-camera-img' }) : '');
  } catch (err) { box.replaceChildren(); tell(err); }
}

// The right arm to one of ALIX's inspection poses (its MoveIt group states, vendored), as Lite moves to any joint
// pose: the speed slider's speed, the controller's SIM mode in Hardware sim, the robot's confirmation in Real.
async function movePose(side) {
  const name = $(`liteS2_${side}_pose`)?.value, p = (S.poses || []).find(x => x.name === name);
  if (!p) { window.toast('Choose an inspection pose.', 'warn'); return; }
  const box = $(`liteS2Camera_${side}`);
  box.replaceChildren(el('div', { class: 'mini-note', text: `Moving the RIGHT arm to ${p.title}...` }));
  try {
    const job = await robotCall('/api/robots/right/move-joints', { joints_deg: p.joints_deg, name: p.title },
      { what: `Moving the RIGHT arm to ALIX's ${p.title} (${p.what})` });
    const j = await watchJob(job, (v) => box.replaceChildren(el('div', { class: 'mini-note', text: jobLine(v) })));
    box.replaceChildren(el('div', { class: `lite-status ${j.state === 'done' ? 'ok' : 'warn'}`,
      text: j.state === 'done' ? `RIGHT arm at ${p.title} (${p.what}). CHECK THE CAMERA shows what it sees from here.`
        : `The move stopped: ${j.error || j.state}` }));
  } catch (err) { box.replaceChildren(); tell(err); }
}

function addMechanism(side) {
  const task = $(`liteS2_${side}_task`)?.value, t = S.mechanisms[task];
  if (!t) { window.toast('Choose a lid or seat task.', 'warn'); return; }
  for (const s of ARMS) {
    S.steps[s].push({ type: 'mechanism', task, uid: newUid() });
    S.sel[s] = S.steps[s].length - 1;
    paintArm(s);
    saveSteps(s);
  }
  window.toast(`${t.title}: added at the end of both arms' lists (UP / DOWN place it in each).`, 'good');
}

// A lid or seat task is on both arms: the k-th of one arm's is the k-th of the other's.
const nthTask = (side, i) => S.steps[side].slice(0, i + 1).filter(x => x.type === 'mechanism').length - 1;
const taskAt = (side, k) => S.steps[side].reduce((at, x, i) => (x.type === 'mechanism' && k-- === 0 ? i : at), -1);

function moveSel(side, d) {
  const i = S.sel[side], list = S.steps[side];
  if (i == null || i + d < 0 || i + d >= list.length) return;
  [list[i], list[i + d]] = [list[i + d], list[i]];
  S.sel[side] = i + d;
  paintAll();
  saveSteps(side);
}

function duplicate(side) {
  const i = S.sel[side];
  if (i == null) return;
  if (S.steps[side][i].type === 'mechanism') { window.toast('A lid or seat task is added to both arms from the LID / SEAT tab.', 'warn'); return; }
  if (S.steps[side][i].type === 'meet') { window.toast('A Meet is added to both arms from the MEET / WAIT FOR tab.', 'warn'); return; }
  S.steps[side].splice(i + 1, 0, { ...JSON.parse(JSON.stringify(S.steps[side][i])), uid: newUid() });    // a copy: a row of its own
  S.sel[side] = i + 1;
  paintAll();
  saveSteps(side);
}

function remove(side) {
  const i = S.sel[side];
  if (i == null) return;
  if (S.steps[side][i].type === 'mechanism') {                // and its pair on the other arm
    const other = side === 'left' ? 'right' : 'left', j = taskAt(other, nthTask(side, i));
    if (j >= 0 && S.steps[other][j].task === S.steps[side][i].task) {
      S.steps[other].splice(j, 1);
      S.sel[other] = S.steps[other].length ? Math.min(j, S.steps[other].length - 1) : null;
      paintArm(other);
      saveSteps(other);
      window.toast(`${(S.mechanisms[S.steps[side][i].task] || {}).title || 'The task'}: removed from both arms.`, 'good');
    }
  }
  if (S.steps[side][i].type === 'meet') {                     // and its partner: the Meets pair up in order
    const other = OTHER[side], j = partnerOf(side, i);
    if (j >= 0) {
      S.steps[other].splice(j, 1);
      S.sel[other] = S.steps[other].length ? Math.min(j, S.steps[other].length - 1) : null;
      paintArm(other);
      saveSteps(other);
      window.toast(`Meet removed from both arms (${ARM(side)} row ${i + 1}, ${ARM(other)} row ${j + 1}).`, 'good');
    }
  }
  const waiting = S.steps[OTHER[side]].map((x, j) => (x.type === 'wait_for' && targetOf(OTHER[side], x) === i ? j + 1 : null)).filter(Boolean);
  if (waiting.length) {
    window.toast(`${ARM(side)} row ${i + 1} was the row ${ARM(OTHER[side])} row ${waiting.join(', ')} waited for: choose the row to wait for again `
      + '(marked in red).', 'warn');
  }
  S.steps[side].splice(i, 1);
  S.sel[side] = S.steps[side].length ? Math.min(i, S.steps[side].length - 1) : null;
  paintAll();
  saveSteps(side);
}

async function clear(side) {
  if (!S.steps[side].length) return;
  const yes = await confirmDialog({ title: 'Clear sequence', body: `Clear all ${ARM(side)} sequence steps?`, ok: 'Clear', danger: true });
  if (!yes) return;
  S.steps[side] = [];
  S.sel[side] = null;
  paintAll();
  saveSteps(side);
}

async function saveFile(side) {
  if (!S.steps[side].length) { window.toast(`${ARM(side)} sequence is empty.`, 'warn'); return; }
  try {
    const blob = await call('GET', `${base()}/${side}/download`, undefined, { raw: true });
    const saved = await saveBlobAs(blob, `${side}_workflow_sequence.json`, [{ description: 'JSON sequence', accept: { 'application/json': ['.json'] } }]);
    if (saved) window.toast(`${ARM(side)} sequence saved: ${saved.name}.`, 'good');
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
    S.edited = true;                                         // not as simulated any more
    recheckPick();
    window.toast(`${ARM(side)}: ${S.steps[side].length} step(s) loaded from ${f.name}.`, 'good');
  } catch (err) { window.toast(`Could not load sequence: ${say(err)}`, 'error'); }
  paintAll();
}

// ── painting

function paintArm(side) {
  const body = $(`liteS2Rows_${side}`);
  if (!body) return;
  const list = S.steps[side];
  const now = S.rows[side];
  const pick = (i, focus) => {
    S.sel[side] = i;
    paintArm(side);
    if (focus) body.querySelector(`tr[data-i="${i}"]`)?.focus();          // the row keeps the keyboard's focus
  };
  const gone = new Set(missing(side).map(m => m.row));
  body.replaceChildren(...list.map((step, i) => {
    const [type, target, options] = row({ ...step, _side: side, _i: i });
    const problem = gateProblem(side, i);
    const off = gone.has(i + 1) || !!problem;
    const tr = el('tr', { class: `${S.sel[side] === i ? 'lite-picked' : ''}${now === i + 1 ? ' lite-s2-now' : ''}${off ? ' lite-s2-gone' : ''}`, tabindex: '0',
      title: problem || (off ? `Not on the ${ARM(side)} controller (as read ${S.sources[side].read || ''}): replace or remove this step.` : ''),
      'data-i': String(i), 'aria-selected': String(S.sel[side] === i),
      onclick: () => pick(i, false),
      onkeydown: (e) => {
        if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); pick(i, true); }
        if (e.key === 'ArrowDown' && i + 1 < list.length) { e.preventDefault(); pick(i + 1, true); }
        if (e.key === 'ArrowUp' && i > 0) { e.preventDefault(); pick(i - 1, true); }
      } },
    el('td', { text: String(i + 1) }), el('td', { text: type }), el('td', { text: target }),
    el('td', { text: problem ? `${options} · cannot run as it is` : off ? `${options}${options ? ' · ' : ''}not on the controller` : options }));
    return tr;
  }));
  if (list.length) {                     // after the last row: the trajectories' own times added (the owner, 20 Oct), no transitions
    const secsOf = (st) => st.type === 'program' ? (S.sources[side]?.programs || []).find(p => p.id === st.id)?.seconds
      : st.type === 'mechanism' ? (S.mechanisms[st.task] || {}).seconds : null;
    const withTime = list.filter(st => secsOf(st) != null);
    const unknown = list.filter(st => (st.type === 'program' || st.type === 'mechanism') && secsOf(st) == null).length;
    body.append(el('tr', { class: 'lite-s2-total' }, el('td', { colspan: '4',
      text: `Total ${fmtT(withTime.reduce((a, st) => a + Number(secsOf(st)), 0))} · ${withTime.length} trajector${withTime.length === 1 ? 'y' : 'ies'}, `
        + `their own times added, no transition time between them${unknown ? ` · ${unknown} with no known time` : ''}` })));
  }
  const cur = now ? body.querySelector('.lite-s2-now') : null;          // the row in view inside its table, the page left where it is
  const box = body.closest('.lite-s2-table');
  if (cur && box) {
    const top = cur.offsetTop - box.querySelector('thead').offsetHeight, bottom = cur.offsetTop + cur.offsetHeight;
    if (top < box.scrollTop) box.scrollTop = top;
    else if (bottom > box.scrollTop + box.clientHeight) box.scrollTop = bottom - box.clientHeight;
  }
  const st = $(`liteS2ArmStatus_${side}`);
  if (st) st.textContent = `${ARM(side)} // ${list.length} sequence step(s)`;
}

function paintAll() {
  if (!built) return;
  for (const side of ARMS) paintArm(side);
  paintTargets();
  for (const m of ['sim', 'real']) $(`liteS2Mode_${m}`).classList.toggle('active', S.mode === m);
  for (const [m] of SCOPES) $(`liteS2Scope_${m}`).classList.toggle('active', S.scope === m);
  for (const m of ['sim', 'real']) $(`liteS2Mode_${m}`).setAttribute('aria-checked', String(S.mode === m));
  for (const [m] of SCOPES) $(`liteS2Scope_${m}`).setAttribute('aria-checked', String(S.scope === m));
  $('liteS2Pick').hidden = S.scope === 'sequence';
  $('liteS2PickText').textContent = pickText() || (S.scope === 'task' ? 'Choose the task: a Program row…' : 'Choose the dual-arm task: a part…');
  $('liteS2RunBoth').textContent = S.scope === 'task' ? 'RUN TASK' : S.scope === 'dual' ? 'RUN DUAL-ARM TASK' : 'RUN BOTH';
  for (const side of ARMS) $(`liteS2Run_${side}`).hidden = S.scope !== 'sequence';     // an arm's own sequence: Sequences only
  $('liteS2Speed').textContent = String(speedPct());
  $('liteS2StatusText').textContent = S.status;
  $('liteS2Runtime').textContent = fmt(S.runtime);
  for (const side of ARMS) $(`liteS2Live_${side}`).textContent = `${ARM(side)} // ${S.live[side]}`;
  const busy = !!S.job;
  for (const id of ['liteS2RunBoth', 'liteS2Run_left', 'liteS2Run_right', 'liteS2Mode_sim', 'liteS2Mode_real', 'liteS2Pick',
    ...SCOPES.map(([m]) => `liteS2Scope_${m}`)]) $(id).disabled = busy;
  $('liteS2Sim').hidden = S.mode !== 'sim' && !S.sim;
  gate();
}

function setMode(m) {
  if (S.job) { window.toast('The mode stays as it is while a sequence or a simulation runs.', 'warn'); return; }
  S.mode = m;
  paintAll();
}
function setScope(s) { S.scope = s; S.pick = null; paintAll(); }

// What is run (workflow.plan_for): {mode, pick}. A sequence's mode is the button's (both, left, right).
function what(mode) {
  if (S.scope === 'task') return S.pick ? { mode: S.pick.side, pick: { scope: 'task', side: S.pick.side, row: S.pick.row, id: S.pick.id } } : null;
  if (S.scope === 'dual') return S.pick ? { mode: 'both', pick: { scope: 'dual', stage: S.pick.stage, ids: S.pick.ids,
    ...(S.pick.task ? { task: S.pick.task } : {}) } } : null;
  return { mode, pick: null };
}

function pickText() {
  const k = S.pick;
  if (!k) return '';
  if (S.scope === 'task') return `${ARM(k.side)} row ${k.row} · ${k.text}`;
  return `Stage ${k.stage + 1} · ${k.text}`;
}

// The task as a searchable list (picker.js): each arm's Program rows (Single task), or the parts of both arms'
// plan (their rows between two Meets) with both arms' Programs and the lid and seat tasks (Dual-arm task).
async function chooseTask(anchor) {
  try { await saved(); } catch (err) { tell(err); return; }
  const secs = (side, id) => (S.sources[side]?.programs || []).find(p => p.id === id)?.seconds;      // its execution time
  const label = (st, side) => `Program ${st.id}${st.name ? ` // ${st.name}` : ''}${secs(side, st.id) != null ? ` (${fmtT(secs(side, st.id))})` : ''}`;
  if (S.scope === 'task') {
    const groups = ARMS.map(side => ({ label: `${ARM(side)} arm`, rows: S.steps[side].map((st, i) => (st.type !== 'program' ? null
      : { value: `${side}:${i + 1}`, text: `Row ${i + 1} · ${label(st, side)}`, search: ARM(side),
        detail: `Start ${st.transit_speed || 30} % · with the Pole and Hand in force before it` })).filter(Boolean) }));
    choose(anchor, groups, S.pick ? `${S.pick.side}:${S.pick.row}` : null, (v) => {
      const [side, row] = v.split(':');
      const st = S.steps[side][Number(row) - 1];
      S.pick = { side, row: Number(row), id: st.id, text: label(st, side) };
      paintAll();
    }, { placeholder: 'Search the Programs: row, ID or name' });
    return;
  }
  let plan;
  try { plan = await post(`${base()}/plan`, { mode: 'both' }); } catch (err) { tell(err); return; }
  const names = (side, s) => (s.programs[side] || []).map(x => `${x.id}${x.name ? ` ${x.name}` : ''}`
    + (secs(side, x.id) != null ? ` (${fmtT(secs(side, x.id))})` : '')).join(', ');
  const tasks = plan.stages.filter(s => s.kind === 'mechanism').map(s => ({ value: String(s.index),
    text: `Stage ${s.index + 1} · ${s.label} · ${s.title} (both arms, pre-recorded)`,
    detail: `ALIX's plan · LEFT row ${(s.rows || {}).left} · RIGHT row ${(s.rows || {}).right}` }));
  const rows = plan.stages.filter(s => s.kind === 'parallel' && (s.programs.left || []).length && (s.programs.right || []).length)
    .map(s => ({ value: String(s.index),
      text: `Stage ${s.index + 1} · ${s.label} · each arm through its own rows`
        + (s.meet_rows ? ` · up to LEFT row ${s.meet_rows.left} and RIGHT row ${s.meet_rows.right}` : ''),
      detail: `LEFT ${names('left', s)} · RIGHT ${names('right', s)}` }));
  const groups = [{ label: `Both arms' parts (between the Meets) · ${rows.length} dual-arm task${rows.length === 1 ? '' : 's'}`, rows }];
  if (tasks.length) groups.push({ label: `Lid and seat · ${tasks.length} task${tasks.length === 1 ? '' : 's'}`, rows: tasks });
  choose(anchor, groups, S.pick ? String(S.pick.stage) : null, (v) => {
    const r = [...rows, ...tasks].find(x => x.value === v), s = plan.stages.find(x => String(x.index) === v);
    S.pick = { stage: Number(v), ids: { left: (s.programs.left || []).map(x => x.id), right: (s.programs.right || []).map(x => x.id) },
      text: r.text.replace(/^Stage \d+ · /, ''), ...(s.kind === 'mechanism' ? { task: s.task } : {}) };
    paintAll();
  }, { placeholder: 'Search the parts: Program ID or name' });
}

// Whether Real Machine Mode can run what is chosen: the last simulation is this run, played to its end, and
// nothing edited since (the server checks it again, with the programs and the points on the controllers).
function simulatedNow(mode) {
  const sim = S.sim, w = what(mode);
  if (!sim || !sim.completed || !w || sim.mode !== w.mode || S.edited) return false;
  if (JSON.stringify(sim.pick || { scope: 'sequence' }) !== JSON.stringify(w.pick || { scope: 'sequence' })
    && !(w.pick && sim.pick && ['scope', 'side', 'row', 'stage'].every(k => sim.pick[k] === w.pick[k]))) return false;
  return sim.overall === speedPct();
}

function gate() {
  const g = $('liteS2Gate');
  if (!g) return;
  if (S.mode === 'sim') { g.className = 'lite-status'; g.textContent = 'RUN plays it in 3D with the collision check. Nothing moves.'; return; }
  const modes = S.scope === 'sequence' ? ['both', 'left', 'right'] : [null];
  const ok = modes.filter(simulatedNow);
  const says = (m) => (S.scope === 'task' ? 'RUN TASK' : S.scope === 'dual' ? 'RUN DUAL-ARM TASK' : m === 'both' ? 'RUN BOTH' : `RUN ${ARM(m)}`);
  g.className = `lite-status ${ok.length ? 'ok' : 'warn'}`;
  g.textContent = ok.length ? `Simulated as it is, played to its end: ${ok.map(says).join(', ')} can run on the robot.`
    : 'Not simulated as it is now: it can still run on the robot, after a confirmation that it was not simulated first.';
}

// ── running: Simulation Mode or Real Machine Mode

async function run(mode) {
  if (!ctx.pid) { window.toast('Open a project first.', 'warn'); return; }
  if (S.job) { window.toast('A sequence is already running.', 'warn'); return; }
  try { await saved(); } catch (err) { tell(err); return; }
  const w = what(mode);
  if (!w) { window.toast(S.scope === 'task' ? 'Choose the task first: a Program row.' : 'Choose the dual-arm task first: a stage of the plan.', 'warn'); return; }
  if (w.mode === 'both' && (!S.steps.left.length || !S.steps.right.length)) { window.toast('Build or load BOTH LEFT and RIGHT sequences first.', 'warn'); return; }
  if (w.mode !== 'both' && !S.steps[w.mode].length) { window.toast(`${ARM(w.mode)} sequence is empty.`, 'warn'); return; }
  if (S.mode === 'sim') simulate(w);
  else runReal(w);
}

async function simulate(w) {
  const mode = w.mode;
  player.pause();
  S.status = `PAGE 02 SIMULATION // ${pickText() || (mode === 'both' ? 'LEFT + RIGHT' : ARM(mode))} // timing the stages and checking collisions`;
  try {
    const job = await post(`${base()}/simulate`, { mode, pick: w.pick });
    S.job = job;
    paintAll();
    const done = await watch(job);
    if (done.state !== 'done') { S.status = `PAGE 02 SIMULATION STOPPED // ${done.error || done.state}`; tell(Object.assign(new Error(done.error || 'Stopped.'), { hint: done.hint })); return; }
    const sim = (await get(base())).sim;                     // the page's view of it: its steps, without the heavy parts
    S.job = null;                                            // over: the player paints the arms from here
    setSim(sim, true);
  } catch (err) { S.status = `PAGE 02 // ${say(err)}`; tell(err); } finally { S.job = null; paintAll(); }
}

async function runReal(w) {
  const mode = w.mode;
  const body = { mode, pick: w.pick };
  let job;
  try {
    try {
      job = await post(`${base()}/run`, body);
    } catch (err) {
      // not run in Simulation Mode as it is now (the owner, 17 Oct): it may run, once the operator says so,
      // and the real-robot confirmation follows
      if (err.code !== 'unsimulated') throw err;
      const go = await confirmDialog({ title: 'Run it without simulating it first?', ok: 'Yes, run it on the real robot',
        body: `${err.message} It has not been played in Simulation Mode, so its collision check has not been made. `
          + 'Do you want to run it on the real hardware without simulating it first?' });
      if (!go) { window.toast('Not run: nothing was sent to the robot. Simulation Mode plays it in 3D first.', 'warn'); return; }
      body.unsimulated_ok = true;
      job = await post(`${base()}/run`, body);
    }
  } catch (err) {
    if (err.code !== 'warnings' && err.code !== 'confirm') { tell(err); return; }
    let warns = [];
    if (err.code === 'warnings') warns = [err.message.replace(/^Before it runs on the robot: /, '')];
    let plan = null;
    try { plan = await post(`${base()}/plan`, { mode, pick: w.pick }); } catch (_) { /* the server says it again */ }
    const c = plan && plan.counts;
    const moves = mode === 'both' ? 'BOTH ROBOT ARMS WILL MOVE' : `${ARM(mode)} ROBOT ARM WILL MOVE`;
    const title = S.scope === 'task' ? `Run the task: ${pickText()}` : S.scope === 'dual' ? `Run the dual-arm task: ${pickText()}`
      : mode === 'both' ? 'Run Dual Sequences' : `Run ${ARM(mode)} Sequence`;
    const n = (k, one, many) => `${k} ${k === 1 ? one : many}`;
    const yes = await confirmDialog({ title, ok: 'Start on the robot',
      body: `${moves}. ${c && mode === 'both' ? `Each arm goes through its own rows: ${n(c.parts, 'part', 'parts')}, `
        + `${n(c.meets, 'Meet', 'Meets')} where both arms meet, ${n(c.waits_for, 'Wait-for row', 'Wait-for rows')}, `
        + `${n(c.mechanisms, 'lid or seat task', 'lid and seat tasks')}. ` : ''}`
        + `${(plan && plan.said || []).join(' ')}${plan && (plan.said || []).length ? ' ' : ''}`
        + `Overall controller speed: ${speedPct()}%. ${warns.length ? `The simulation found: ${warns.join(' ')} ` : ''}`
        + 'Each step is timed on the robot: one far outside its range stops both arms. '
        + 'Confirm both work areas are clear and both hardware E-stops are accessible.',
      list: REAL_CHECKS });
    if (!yes) { window.toast('Cancelled: nothing was sent to the robot.', 'warn'); return; }
    try { job = await post(`${base()}/run`, { ...body, confirmed: true, warnings_seen: S.sim && S.sim.hash }); } catch (e2) { tell(e2); return; }
  }
  follow(job);
}

async function follow(job) {
  if (job.kind !== 'simulate' && job.kind !== 'run') return;   // S2's run or simulation only (a check is the view's)
  S.job = job;
  if (job.kind === 'run') S.mode = 'real';
  paintAll();
  try {
    const done = await watch(job);
    if (job.kind === 'simulate') {                           // a simulation followed after a reload
      if (done.state === 'done') { const sim = (await get(base())).sim; S.job = null; setSim(sim, true); }
      return;
    }
    if (done.state === 'done') window.toast(`Sequence complete (${fmt(done.elapsed_s)}).`, 'good');
    else if (done.state === 'cancelled') window.toast('The sequence was stopped.', 'warn');
    else tell(Object.assign(new Error(done.error || 'The sequence did not finish.'), { hint: done.hint }));
    if (done.state !== 'done' && done.error) S.status = `PAGE 02 SEQUENCE STOPPED // ${done.error}`;
  } catch (err) { tell(err); } finally {
    S.job = null;
    S.rows = { left: null, right: null };
    paintAll();
  }
}

// Polls a job, painting its live status and asking its questions, if it has any (jobs.py: ask).
async function watch(job) {
  let j = job, misses = 0, asked = 0;                       // a job's questions are numbered from 1: per job
  for (;;) {
    paintJob(j);
    if (j.state !== 'running') return j;
    const q = j.extra && j.extra.question;
    if (q && q.id > asked) {
      asked = q.id;
      const yes = await confirmDialog({ title: q.title, body: q.text, ok: q.ok, danger: true });
      try { j = await post(`/api/jobs/${j.id}/answer`, { question: q.id, yes }); continue; } catch (err) { tell(err); }
    }
    await new Promise(r => setTimeout(r, 300));
    try {
      j = await get(`/api/jobs/${j.id}`);
      misses = 0;
    } catch (err) {                                          // a network blip: the run goes on, keep following it
      misses += 1;
      if (misses >= 20 || err.code === 'login') throw err;
      await new Promise(r => setTimeout(r, 700));
    }
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

// STOP goes by what is running, never by the mode on screen: a run on the robot always gets the robot's
// STOP (Lite's: it holds the arms until Recover); a simulation is cancelled and its playback paused.
async function stop(which) {
  player.pause();
  if (S.job && S.job.kind === 'simulate') {
    try { await post(`/api/jobs/${S.job.id}/cancel`, {}); } catch (_) { /* it ends by itself */ }
    S.status = 'PAGE 02 SIMULATION // stopped';
    paintAll();
    return;
  }
  if (!S.job && S.mode === 'sim') { S.status = 'PAGE 02 SIMULATION // paused'; paintAll(); return; }
  S.status = which === 'both' ? 'PAGE 02 STOP REQUESTED // LEFT + RIGHT' : `${ARM(which)} // STOP REQUESTED`;
  for (const side of (which === 'both' ? ARMS : [which])) S.live[side] = 'STOP REQUESTED';
  paintAll();
  try { await post(which === 'both' ? '/api/robots/stop-both' : `/api/robots/${which}/stop`, {}); } catch (err) { tell(err); }
}

// ── Simulation Mode's 3D: each arm's part of each stage as frames, on the run's own clock

// S2 on screen: its 3D can be watched (a run plays itself only then, and counts as played only then).
const onScreen = () => { const h = $('liteS2Host'); return !!h && h.getClientRects().length > 0; };

function setSim(sim, fresh) {
  S.sim = sim;
  view.token += 1;
  view.frames.clear();
  view.shown = { left: null, right: null };
  view.phase = null;
  view.clean = false;
  view.furthest = 0;
  const k = sim.pick || {};
  frame($('liteS2Host')).title(k.scope === 'task' ? `${ARM(k.side)} task, row ${k.row}` : k.scope === 'dual' ? `Dual-arm task, stage ${k.stage + 1}`
    : sim.mode === 'both' ? 'Both arms' : `${ARM(sim.mode)} arm`, `${sim.counts.stages} stage(s) · ${fmt(sim.total_s)}`
    + (sim.completed ? ' · played to its end' : ''));
  if (fresh) S.edited = false;                               // this simulation is of the steps as they are
  $('liteS2Timeline').max = String(Math.round(sim.total_s * 1000));
  tools.load(sim.hash, { phase: null, text: "each Program's region" });
  tools.setLength(sim.total_s);
  tools.provide(null, { result: sim.collisions, spans: spansOf(sim.collisions) });
  paintStages();
  paintFindings();
  player.load(sim.total_s);
  if (fresh) {
    S.status = `PAGE 02 SIMULATION READY // ${sim.counts.stages} stage(s), ${fmt(sim.total_s)}`;
    // it plays itself only on screen; ended while S2 was not shown, it waits there for Play (never played unseen)
    if (onScreen()) prefetch().then((ok) => { if (ok && onScreen()) playSim(true); });
    else S.status += ' // press Play in S2 to watch it';
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
    const p = post(`${base()}/frames`, { side, stage, hash: S.sim && S.sim.hash });   // this simulation's, no other's
    p.catch((err) => {                                     // asked again later, unless its files are not on this computer
      if (err && (err.code === 'files' || err.code === 'plans')) return;    // (made elsewhere, or with other lid and seat plans):
                                                             // a new simulation is needed
      setTimeout(() => { if (view.frames.get(k) === p) view.frames.delete(k); }, 2000);
    });
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
  if (!S.sim) { window.toast('Run the sequence in Simulation Mode first.', 'warn'); return; }
  if (!(S.sim.total_s > 0)) { view.clean = true; simEnded(); return; }   // nothing that takes time: it is at its end
  if (!view.three || view.three.viewer.host !== $('liteS2Host')) {     // the 3D first: a run with nothing shown is not played
    prefetch().then((ok) => { if (ok) playSim(fromStart); });
    return;
  }
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
  frame($('liteS2Host')).time(`${fmt(g)} / ${fmt(S.sim.total_s)}`);
  paintMoment(g);
  const three = view.three;
  tools.at(g);
  if (!three || three.viewer.host !== $('liteS2Host') || !onScreen()) {   // only while S2 has the 3D, on screen
    if (playing) { view.clean = false; player.pause(); }          // not watched: it does not count as played
    return;
  }
  const my = view.token;
  for (const side of ARMS) {
    const c = chunkAt(side, g);
    if (!c) continue;
    const k = `${side}:${c.stage}`;
    if (view.shown[side] !== k) {
      let r;
      try { r = await frames(side, c.stage); } catch (err) {
        view.clean = false;                                    // not shown: the run does not count as played
        $('liteS2SimState').className = 'lite-status warn';
        $('liteS2SimState').textContent = `The 3D could not show the ${ARM(side)} arm's part of stage ${c.stage + 1}: ${say(err)}`;
        continue;
      }
      if (my !== view.token) return;
      three.robot.loadTimeline(side, r);
      view.shown[side] = k;
    }
    three.robot.showTime(g, side);
  }
  const ph = tools.phase || phaseAt(g);                        // the commode chosen under the view, else each Program's
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
    if (a) text = `${a.text} // ${a.gate ? 'WAITING' : a.drawn === false ? 'NOT DRAWN' : 'RUNNING'} (row ${a.row})`;
    else if (acts.length && g >= acts[acts.length - 1].t1) text = 'SEQUENCE COMPLETE';
    S.live[side] = text;
    if ((a ? a.row : null) !== S.rows[side]) { S.rows[side] = a ? a.row : null; paintArm(side); }
  }
  S.status = stage ? `SIMULATION // ${stage.label || stage.kind.toUpperCase()}${stage.kind === 'parallel' ? ' // EACH ARM THROUGH ITS OWN ROWS' : ''}`
    : S.status;
  document.querySelectorAll('.lite-s2-stage').forEach((n) => n.classList.toggle('active', stage && Number(n.dataset.i) === stage.index));
  for (const side of ARMS) $(`liteS2Live_${side}`).textContent = `${ARM(side)} // ${S.live[side]}`;
  $('liteS2StatusText').textContent = S.status;
}

async function simEnded() {
  const sim = S.sim;
  if (!sim || !view.clean) {
    $('liteS2SimState').className = 'lite-status warn';
    $('liteS2SimState').textContent = /could not show/.test($('liteS2SimState').textContent)
      ? `${$('liteS2SimState').textContent} Not counted as played: play it again from the start once the 3D shows it.`
      : 'Played with a skip: play it from the start to the end for Real Machine Mode.';
    return;
  }
  // what the 3D could not draw is said, never "complete" over it (the owner, 16 Oct: a run reported
  // complete while nothing moved on the view)
  const missing = ARMS.flatMap(side => (sim.acts[side] || []).filter(a => (a.kind === 'program' || a.kind === 'global') && a.drawn === false)
    .map(a => `${ARM(side)} row ${a.row} (${a.kind === 'program' ? `Program ${a.program}` : a.text || 'Global Point'})`));
  try {
    const out = await post(`${base()}/played`, { hash: sim.hash });
    sim.completed = out.completed;
    if (missing.length) {
      const text = `Played to its end, but ${missing.length} step${missing.length === 1 ? ' was' : 's were'} not drawn (a Program with no file `
        + `Lite has, or a Global Point not read from its controller): ${missing.join(', ')}. Real Machine Mode checks `
        + `${missing.length === 1 ? 'it' : 'them'} on the controllers, and asks, before it runs.`;
      $('liteS2SimState').className = 'lite-status warn';
      $('liteS2SimState').textContent = text;
      window.toast(text, 'warn');
    } else {
      $('liteS2SimState').className = 'lite-status ok';
      $('liteS2SimState').textContent = 'Played to its end with every step drawn: Real Machine Mode can run it as it is.';
      window.toast('Simulation complete, every step drawn: Real Machine Mode can run it as it is.', 'good');
    }
  } catch (err) { tell(err); }
  paintAll();
}

function paintStages() {
  const sim = S.sim, box = $('liteS2Stages');
  const words = { parallel: 'each arm through its own rows', single: 'one arm', mechanism: 'lid / seat, both arms (ALIX\'s plan)' };
  const ends = (s) => (s.t1_range && s.t1_range[1] - s.t1_range[0] >= 0.05 ? ` (ends ${fmt(s.t1_range[0])}–${fmt(s.t1_range[1])} on the robot)` : '');
  box.replaceChildren(el('h4', { text: 'Stages (each ends where both arms meet)' }), ...sim.stages.map(s => el('button', {
    class: 'lite-s2-stage', type: 'button', 'data-i': String(s.index), onclick: () => { player.pause(); seekSim(s.t0); },
    text: `${s.index + 1}. ${s.title || s.label || s.kind.toUpperCase()} · ${words[s.kind] || s.kind} · ${fmt(s.t0)}–${fmt(s.t1)}${ends(s)}` })));
}

function paintFindings() {
  const sim = S.sim, box = $('liteS2Findings'), col = sim.collisions, drift = sim.drift;
  const pair = (p) => `${p.link} – ${p.object}${p.depth_mm ? ` (${p.depth_mm} mm)` : ''}`;
  const where = (e) => (e.at || []).filter(Boolean).map(w => `${ARM(w.side)} row ${w.row} (${w.text}${w.line ? `, line ${w.line.toLocaleString()}` : ''})`).join('; ');
  const c = Object.entries(col.contact || {}).filter(([, v]) => v.samples).map(([s, v]) => `${ARM(s)} hand on the commode from ${fmt(v.first_t)} (deepest ${v.deepest_mm} mm)`);
  const revised = sim.plans_changed || [];                  // made with ALIX's plans of before (server: workflow.plans_changed)
  box.replaceChildren(...[
    revised.length ? el('div', { class: 'lite-status err', text: `This simulation was made with the plan of ${revised.join(' and ')} as it `
      + 'was before ALIX revised it: it no longer plays. Run the sequence again in Simulation Mode.' }) : null,
    el('h4', { text: 'Collision check' }),
    el('div', { class: `lite-status ${col.ok ? 'ok' : 'err'}`, text: col.ok
      ? `No collision: ${col.samples.toLocaleString()} moments checked, both arms against ${col.checked.walls}, the commode and each other.`
      : `Collision at ${fmt(col.first.first_t)}: ${col.first.pairs.map(pair).join('; ') || col.first.what.join(', ')}. ${where(col.first)}. `
        + `${col.events.length + col.more} event(s).` }),
    c.length ? el('div', { class: 'mini-note', text: `Contact (the cleaning; up to ${col.checked.contact_depth_mm} mm is the pad's give): ${c.join('; ')}.` }) : null,
    ...col.events.slice(0, 6).map(e => el('div', { class: 'lite-rev3d-event' },
      el('span', { text: `${fmt(e.first_t)}–${fmt(e.last_t)}: ${e.pairs.map(pair).join('; ') || e.what.join(', ')} · ${where(e)}` }),
      el('button', { class: 'btn light', type: 'button', text: 'Show', onclick: () => { player.pause(); seekSim(e.first_t); } }))),
    drift && drift.timings ? el('h4', { text: `The two arms ${drift.clearance_mm} mm apart, at every timing` }) : null,
    drift && drift.timings ? el('div', { class: `lite-status ${drift.events.length ? 'warn' : 'ok'}`, text: (drift.events.length
      ? `Closer than ${drift.clearance_mm} mm in ${drift.events.length} place${drift.events.length === 1 ? '' : 's'} (said below)`
      : `Never closer than ${drift.clearance_mm} mm`) + `: ${drift.timings} timing${drift.timings === 1 ? '' : 's'} checked, each arm's steps from as fast `
      + `as the robot runs them to as slow, against the other's (up to ${fmt(drift.band_s)} apart).` }) : null,
    // (no Show: the 3D plays the usual timing, and these are other timings)
    ...(drift && drift.events || []).slice(0, 6).map(e => el('div', { class: 'lite-rev3d-event' },
      el('span', { text: `${fmt(e.first_t)}–${fmt(e.last_t)} in its timing: ${e.exact_mm > 0 ? `${e.exact_mm} mm` : 'touching'} · ${(e.links || []).join(', ')} · ${where(e)}` }))),
    el('h4', { text: 'Before it runs on the robot' }),
    sim.findings.length ? el('ul', { class: 'lite-s2-findlist' }, sim.findings.map(f => el('li', { class: f.level === 'stop' ? 'lite-bad' : f.level === 'warn' ? 'lite-warn' : 'mini-note', text: f.text })))
      : el('div', { class: 'lite-status ok', text: 'Every Program is drawn, and its pole and hand are the ones it was taught with.' }),
    el('div', { class: 'mini-note', text: 'Times are as the robot runs each step as a rule (measured in the lab): the programs on their stored 5 ms '
      + 'timing, the moves at their speed % of each joint\'s rating; each step also has its range, which the timing check above goes through. '
      + `The arms start ${Object.values(sim.start_from || {}).includes('now') ? 'where they stand now' : 'from the standing pose'}. A Program set at another `
      + 'pole or hand than its own stops Real Machine Mode (red); the rest it asks about, once. On the robot each step is timed: a '
      + 'Program whose lines run past 105 % of their time (or end before 90 %), or another step more than 10 % outside its range, '
      + 'stops both arms.' })].filter(Boolean));
  $('liteS2Marks').replaceChildren();                       // (the collisions are marked by the view's tools)
}

// Data Management read a controller again (or S1, or a start-pose list): the lists here follow it.
document.addEventListener('lite:controller', (e) => { if (built && ctx.pid && (e.detail || {}).from !== 's2') refreshSources(false); });

export function openS2(host) {
  if (panel !== host) { panel = host; built = false; }
  build();
  if (!ctx.pid) { S.status = 'PAGE 02 // open a project first'; paintAll(); return; }
  load().then(() => refreshSources(false)).then(async () => {
    if (S.sim) {                                            // the 3D back from wherever it was (S1, Tune, Review)
      view.three = null;
      view.shown = { left: null, right: null };
      view.phase = null;
      if (await prefetch()) showAt(player.t);
    } else {                                                // nothing simulated yet: the robot at home
      try { view.three = await home3d($('liteS2Host')); } catch (_) { /* no 3D in this browser */ }
    }
  });
}

// Leaving S2 (the S1 tab, another view): its playback stops and takes the robot no further; a played run
// is "played to its end" only when watched through, so a stop here also counts as a skip.
export function stopS2() {
  if (!player) return;
  if (player.playing) view.clean = false;
  player.pause();
  view.token += 1;
}

document.addEventListener('lite:speed', () => { if (built) paintAll(); });
