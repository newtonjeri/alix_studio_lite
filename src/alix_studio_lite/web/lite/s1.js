// Merge + Sequence, S1: Merge & Upload, LEFT and RIGHT side by side (feedback v5, slide 9; the
// owner, 16 Oct). For each arm:
// 1. the files of that arm (every region's recordings and kept versions, and the arm's merges), of this
//    project or of all projects (the owner, 16 Oct: "current project (default), and all projects taught
//    files"), picked in order from a searchable list (picker.js, as the global points are). Only files
//    with the same pole, hand, UGV placement and room (each measurement within ±10 mm; the standoff
//    ±5 mm) merge: once one is picked, the files that differ are greyed out in the list with what
//    differs (server/merges.py says it, and refuses a merge that differs);
// 2. merged and saved under a name, the files joined where one ends away from where the next
//    begins (merges.py), the merge keeping their pole, hand, placement and phase;
// 3. each merge renamed, downloaded, sent to a Program ID on the arm's controller, reviewed in 3D
//    (below: staged first, then the file on the 5 ms clock, in the room and at the UGV placement its
//    files were taught at), or sent to Tune + Simulation: Tune opens it as its source, and what Tune
//    keeps comes back here as a new merge.
// The three steps are set apart as the teaching flow sets its own apart (the owner, 21 Oct: "Use the same
// step-separation format that the app already uses for steps in other sub-tasks (such as Prepare, Drag and Teach, and
// Save and Review)"): the numbered step bar (the page's pipeline: the step to do now, the steps done) and each step a
// card of its own with its heading and what it does; a click on the bar goes to that step.

import { call, get, post, say } from './api.js';
import { armFromName, confirmDialog, ctx, deleteButton, deleteStored, el, fmtT, jobLine, robotCall, stageDialog, tell, watchJob } from './ctx.js';
import { borrow, home3d, Player } from './seq3d.js';
import { scene } from './sim.js';
import { saveBlobAs } from './tune.js';
import { checkJob, spansOf, Tools } from './view3d.js';
import { choose } from './picker.js';
import { frame } from './vp.js';

const $ = (id) => document.getElementById(id);
const ARMS = ['left', 'right'];
const ARM = (s) => s.toUpperCase();
const KIND = { rec: 'Recording', ver: 'Kept version', merge: 'Merge', file: 'Loaded file' };
// what Lite says before a file it did not record joins a merge (the owner, 16 Oct: "warn the user that we do not know
// the pole and hand states for those tasks that are not taught in the lite app")
const LOAD_WARNING = 'Lite does not know the pole height and hand pose these files were taught with: they were not taught in Lite. '
  + 'Say them as the robot had them when they were taught. A merge stages them before it runs, so a wrong value runs it at the wrong '
  + 'height or with the wrong hand. Lite has no UGV placement or room for them either: they merge only with files that have none.';
const PHASE_WORDS = { lid_closed: 'lid closed', lid_open: 'lid open', seat_open: 'seat open' };
const fresh = () => ({ items: [], picked: [], merges: [], sel: null, busy: false, scope: 'project', show: 'controller', controller: null });
// the steps, as the teaching flow's step bar names its own: [title, what it holds]
const STEPS = [['Files', 'Pick them in order'], ['Merge order', 'Order · name · merge'], ['Merged files', 'Upload · review · tune']];
const st = { left: fresh(), right: fresh() };
const review = { side: null, file: null, phase: null, scene: null, t0: 0, total: 0, token: 0 };
let panel = null, player = null, built = false, tools = null;

const base = () => `/api/projects/${ctx.pid}`;
const murl = (side, file, verb) => `${base()}/merges/${side}/${encodeURIComponent(file)}/${verb}`;
const fmt = (s) => `${Number(s || 0).toFixed(2)} s`;

function status(side, text, cls = '') {
  const s = $(`liteS1Status_${side}`);
  if (s) { s.className = `lite-status${cls ? ` ${cls}` : ''}`; s.textContent = text; }
}

// ── the panel

// One step's card: its heading and what it does, as the teaching flow's stage cards have them, then its content.
function stepCard(side, n, does, ...kids) {
  return el('section', { id: `liteS1Step${n}_${side}`, class: 'card lite-s1-step', 'aria-labelledby': `liteS1StepTitle${n}_${side}` },
    el('div', { class: 'section-head' },
      el('div', {}, el('h2', { id: `liteS1StepTitle${n}_${side}`, text: `${n}. ${STEPS[n - 1][0]}` }), el('p', { text: does })),
      el('span', { id: `liteS1StepBadge${n}_${side}`, class: 'badge' })),
    ...kids);
}

function build() {
  if (built) return;
  built = true;
  const btn = (text, onclick, cls = 'btn light') => el('button', { class: cls, type: 'button', text, onclick });
  panel.replaceChildren(
    el('div', { class: 'notice lite-s1-hint' }, el('b', { text: 'Which files merge: ' }),
      'one version per task, all raw (recordings) or all processed (tuned in Lite, or auto optimized: TOPP-RA), with the same pole, hand, UGV placement and room '
      + '(each measurement within ±10 mm, the standoff ±5 mm). Pick the first file: the files that cannot join it are greyed out, with why. '
      + 'The merged program runs from one UGV position, with one pole and hand.'),
    el('div', { class: 'lite-s1-arms' }, ARMS.map((side) => el('div', { class: `card lite-s1-arm ${side}` },
      el('div', { class: 'lite-s1-head' }, el('span', { class: `badge ${side === 'left' ? 'blue' : 'orange'}`, text: `${ARM(side)} ARM` }),
        el('b', { text: `Merge the ${ARM(side)} arm's files` })),
      el('div', { class: 'pipeline lite-s1-pipeline', role: 'list', 'aria-label': `${ARM(side)} arm: the merge's steps` },
        STEPS.map(([title, sub], i) => el('button', { id: `liteS1Pipe${i + 1}_${side}`, class: 'pipe', type: 'button', role: 'listitem',
          onclick: () => $(`liteS1Step${i + 1}_${side}`)?.scrollIntoView({ behavior: 'smooth', block: 'start' }) },
        `${i + 1}. ${title}`, el('span', { text: sub })))),
      stepCard(side, 1, 'Pick the files to merge, in the order they run: from the list (by default the files already on the controller), '
        + 'or loaded from this computer.',
      el('div', { class: 'lite-row lite-s1-pickrow' },
        el('div', { class: 'field' }, el('label', { for: `liteS1Scope_${side}`, text: 'Files from' }),
          el('select', { id: `liteS1Scope_${side}`, onchange: (e) => { st[side].scope = e.target.value; load(side); } },
            el('option', { value: 'project', text: 'This project' }), el('option', { value: 'all', text: 'All projects' }))),
        // the list shows, by default, the files already on the arm's controller (the owner, 21 Oct: "only the uploaded
        // tasks ... the ones that already exist in the controller"); All files shows every one
        el('div', { class: 'field' }, el('label', { for: `liteS1Show_${side}`, text: 'Show' }),
          el('select', { id: `liteS1Show_${side}`, onchange: (e) => { st[side].show = e.target.value; paint(side); } },
            el('option', { value: 'controller', text: 'On the controller' }), el('option', { value: 'all', text: 'All files' }))),
        el('div', { class: 'field lite-grow' }, el('label', { for: `liteS1Add_${side}`, text: 'Add a file' }),
          el('button', { id: `liteS1Add_${side}`, class: 'lite-picker-btn', type: 'button', 'aria-haspopup': 'listbox', 'aria-expanded': 'false',
            onclick: (e) => chooseFile(side, e.currentTarget) }, el('span', { class: 'lite-picker-value', text: 'Choose a file to add…' })))),
      el('div', { id: `liteS1Count_${side}`, class: 'mini-note' }),
      el('div', { class: 'lite-row lite-s1-loadrow' },
        btn('Load files from this computer…', () => $(`liteS1Load_${side}`).click()),
        btn('Load a folder…', () => $(`liteS1LoadDir_${side}`).click()),
        el('input', { id: `liteS1Load_${side}`, type: 'file', accept: '.txt,text/plain', multiple: true, hidden: true,
          onchange: (e) => loadFiles(side, e.target) }),
        el('input', { id: `liteS1LoadDir_${side}`, type: 'file', multiple: true, hidden: true, webkitdirectory: '',
          onchange: (e) => loadFiles(side, e.target) }))),
      stepCard(side, 2, 'Put the files in the order they run, name the merged file, then merge and save it.',
        el('ol', { id: `liteS1Order_${side}`, class: 'lite-s1-order' }),
        el('div', { class: 'lite-row' },
          el('div', { class: 'field lite-grow' }, el('label', { for: `liteS1Name_${side}`, text: 'Merged file name' }),
            el('input', { id: `liteS1Name_${side}`, type: 'text', autocomplete: 'off', maxlength: '120' })),
          btn('Merge and save', () => merge(side), 'btn'))),
      stepCard(side, 3, 'Choose a merged file: rename or download it, upload it to a Program ID on the controller, review it in 3D, '
        + 'or send it to Tune + Simulation.',
      el('div', { class: 'lite-table-scroll lite-s1-merges' }, el('table', { class: 'lite-table' },
        el('thead', {}, el('tr', {}, ['', 'Merge', 'Time', 'Pole and hand', 'UGV placement', 'Limits'].map(h => el('th', { text: h })),
          el('th', { class: 'lite-del-cell', 'aria-label': 'Delete from the controller' }))),
        el('tbody', { id: `liteS1Merges_${side}` }))),
      el('div', { id: `liteS1Tools_${side}`, class: 'lite-s1-tools', hidden: true },
        el('div', { class: 'lite-row' },
          el('div', { class: 'field lite-grow' }, el('label', { for: `liteS1Rename_${side}`, text: 'Name' }),
            el('input', { id: `liteS1Rename_${side}`, type: 'text', autocomplete: 'off', maxlength: '120' })),
          btn('Rename', () => rename(side)), btn('Download', () => download(side), 'btn outline')),
        el('div', { class: 'lite-row' },
          el('div', { class: 'field' }, el('label', { for: `liteS1Pid_${side}`, text: 'Program ID' }),
            el('input', { id: `liteS1Pid_${side}`, type: 'number', min: '1', max: '100', step: '1', placeholder: '1–100',
              list: `liteS1Slots_${side}`, oninput: () => slotLine(side) }),
            el('datalist', { id: `liteS1Slots_${side}` })),
          btn('Read the controller', () => readSlots(side, true), 'btn light'),
          btn('Upload to controller', () => upload(side), 'btn'),
          btn('Review in 3D', () => show3d(side), 'btn green'),
          btn('Send to Tune + Simulation', () => toTune(side), 'btn outline')),
        el('div', { id: `liteS1Slot_${side}`, class: 'mini-note' }))),
      el('div', { id: `liteS1Status_${side}`, class: 'lite-status' })))),
    el('div', { class: 'card lite-s1-sim' },
      el('div', { class: 'lite-section-title' },
        el('h3', { text: 'Simulation' })),
      el('div', { class: 'lite-s1-sim-grid' },
        el('div', { class: 'lite-rev3d-main' }, el('div', { id: 'liteS1Host', class: 'lite-rev3d-host' })),
        el('div', { class: 'lite-rev3d-side' },
          el('div', { class: 'lite-row' }, btn('Play', () => player.play(), 'btn'), btn('Pause', () => player.pause(), 'btn outline'),
            btn('Restart', () => { player.pause(); player.seek(0); })),
          el('div', { id: 'liteS1Track', class: 'lite-rev3d-track' },
            el('input', { id: 'liteS1Timeline', type: 'range', min: '0', max: '0', step: '5', value: '0', 'aria-label': 'Merge time',
              oninput: (e) => { player.pause(); player.seek(Number(e.target.value) / 1000); } })),
          el('div', { id: 'liteS1SimNote', class: 'mini-note', text: 'The merged program as the robot runs it: the pole and the hand staged first, '
            + 'the move to its first line, then the file on the 5 ms clock, in the room and at the UGV placement its files were taught at.' }),
          el('div', { id: 'liteS1Phases', class: 'mini-note' })))));
  player = new Player({ onTime: showAt });
  // the view's tools (view3d.js): the commode's state, the merge's collision check, its links in red
  tools = new Tools({
    what: 'the merge',
    check: async (phase, progress) => {
      const r = await checkJob(post, murl(review.side, review.file, 'collisions'), { phase }, progress);
      return { result: r, spans: spansOf(r, review.t0) };
    },
    seek: (t) => { player.pause(); player.seek(t); },
    changed: (phase) => { if (review.side) scene(phase || review.phase, undefined, review.scene).catch(() => {}); },
  }).mount($('liteS1Host'));
  tools.track($('liteS1Track'));
  tools.load(null, { enabled: false, text: "the merge's" });
  frame($('liteS1Host')).title('The robot at home', 'Choose a merged file, then Review in 3D');
}

// ── data

async function load(side) {
  try {
    const [src, all] = await Promise.all([post(`${base()}/merge-sources/${side}`, { picked: st[side].picked, scope: st[side].scope }),
      get(`${base()}/merges`)]);
    const was = new Map(st[side].items.map(it => [it.ref, it]));
    st[side].items = src.items;
    st[side].controller = src.controller || null;
    const gone = st[side].picked.filter(r => !src.items.some(it => it.ref === r));
    st[side].picked = st[side].picked.filter(r => src.items.some(it => it.ref === r));
    if (gone.length) {                                       // e.g. another project's files, with Files from: This project
      status(side, `Taken out of the merge order (not in the list now: ${st[side].scope === 'all' ? 'gone' : 'Files from: This project'}): `
        + `${gone.map(r => (was.get(r) || { name: r }).name).join(', ')}.`, 'warn');
    }
    st[side].merges = all[side] || [];
    if (st[side].sel && !st[side].merges.some(m => m.file === st[side].sel)) st[side].sel = null;
  } catch (err) { status(side, `The files could not be read: ${say(err)}`, 'warn'); }
  paint(side);
}

async function repick(side) {
  try {
    const src = await post(`${base()}/merge-sources/${side}`, { picked: st[side].picked, scope: st[side].scope });
    st[side].items = src.items;
    st[side].controller = src.controller || null;
  } catch (err) { tell(err); }
  paint(side);
}

// Files from this computer (a folder too) kept in the project to merge, with the pole and hand the operator says
// they were taught with, after the warning; a file named for the other arm is left out (server/merges.py). The
// files loaded go into the merge order (step 2: the owner, 16 Oct, loaded a file and did not find it there), after
// the files in it, in name order (a folder's 2 before its 10), each that can join the merge; one that cannot stays
// in the list, greyed with why, and the line under the card says why.
const MAX_PARTS = 50;                                      // a merge's files at most (server/merges.py)

async function loadFiles(side, input) {
  const files = [...(input.files || [])].filter(f => /\.txt$/i.test(f.name));
  input.value = '';
  if (!files.length) { window.toast('Choose .txt trajectory files.', 'warn'); return; }
  const where = (f) => f.webkitRelativePath || f.name;
  files.sort((a, b) => where(a).localeCompare(where(b), undefined, { numeric: true, sensitivity: 'base' }));
  const other = files.filter(f => { const a = armFromName(where(f)); return a && a !== side; });
  const mine = files.filter(f => !other.includes(f));
  const otherArm = ARM(side === 'left' ? 'right' : 'left');
  if (!mine.length) { window.toast(`${other.length === 1 ? `${other[0].name} is` : 'These files are'} named for the ${otherArm} arm: never merged on the ${ARM(side)} arm.`, 'warn'); return; }
  const said = await stageDialog({ title: `Load ${mine.length === 1 ? mine[0].name : `${mine.length} files`} into the ${ARM(side)} merges`,
    body: LOAD_WARNING, arm: side });
  if (!said) { status(side, 'Nothing loaded: without their pole and hand, files cannot be checked against the others.', 'warn'); return; }
  const refused = [], loadedRefs = [], restated = [];
  for (const f of mine) {
    try {
      const out = await post(`${base()}/merge-files/${side}`, { name: f.name, path: where(f), content: await f.text(), pole_mm: said.pole_mm, hand: said.hand });
      loadedRefs.push(`file:${out.file}`);
      if (out.was) restated.push(`${out.name}: loaded before with ${out.was}, now ${out.stage.text}`);
    } catch (err) { refused.push(`${f.name}: ${say(err)}`); }
  }
  const ok = loadedRefs.length;
  const { added, left } = await intoOrder(side, loadedRefs);
  const otherWords = other.length ? ` ${other.length} named for the ${otherArm} arm left out.` : '';
  const refusedWords = refused.length ? ` Refused: ${refused.join('; ')}` : '';
  const restatedWords = restated.length ? ` Pole and hand changed: ${restated.join('; ')} (a merge made from it before keeps its own).` : '';
  const addedWords = !ok ? '' : added === ok ? ` ${ok === 1 ? 'It is' : 'They are'} in the merge order (2. Merge order).`
    : `${added ? ` ${added} added to the merge order (2. Merge order).` : ''} Not added to the merge order (in the list under Loaded from this computer, `
      + `greyed with why): ${left.join('; ')}.`;
  status(side, `${ok} file${ok === 1 ? '' : 's'} loaded.${restatedWords}${addedWords}${otherWords}${refusedWords}`,
    refused.length || other.length || added < ok ? 'warn' : 'ok');
  await load(side);
}

// `refs` (files just loaded) after the merge order's files, each that can join it; returns how many were added
// and why the others were not ([name: why]).
async function intoOrder(side, refs) {
  const s = st[side];
  const news = refs.filter((r, i) => !s.picked.includes(r) && refs.indexOf(r) === i);
  const left = [];
  if (!news.length) return { added: 0, left };
  let src;
  try {
    src = await post(`${base()}/merge-sources/${side}`, { picked: [...s.picked, ...news], scope: s.scope });
  } catch (err) { return { added: 0, left: [`the merge order could not be checked: ${say(err)}`] }; }
  const by = new Map(src.items.map(it => [it.ref, it]));
  let added = 0;
  for (const r of news) {
    const it = by.get(r);
    if (!it) continue;
    const why = it.why ? `cannot be merged: ${it.why}` : it.differs && it.differs.length ? `cannot join this merge: ${it.differs.join('; ')}`
      : s.picked.length >= MAX_PARTS ? `a merge takes ${MAX_PARTS} files at most` : '';
    if (why) { left.push(`${it.name}: ${why}`); continue; }
    s.picked.push(r);
    added += 1;
  }
  return { added, left };
}

// The files to add as a searchable list (picker.js): by project and region; a file that cannot join is
// greyed, with why (another pole, hand, placement or room than the first file; its arm or stage unknown).
// Where a file is on its arm's controller, in the list's words.
const onCtl = (it) => (it.on_controller ? `on the controller as program ${it.on_controller.id}${it.on_controller.by === 'name' ? ', found by its name' : ''}` : '');

function chooseFile(side, anchor) {
  const s = st[side];
  const groups = new Map();
  for (const it of s.items) {
    if (s.picked.includes(it.ref)) continue;                 // in the order already
    if (s.show === 'controller' && !it.on_controller) continue;     // by default, the files on the controller only
    const label = `${it.project ? `${it.project} · ` : s.scope === 'all' ? 'This project · ' : ''}${it.region}`;
    if (!groups.has(label)) groups.set(label, []);
    const differs = it.differs && it.differs.length ? `Cannot join this merge: ${it.differs.join('; ')}` : '';
    groups.get(label).push({ value: it.ref, text: it.name, search: `${KIND[it.kind]} ${it.grade} ${it.grade_text || ''} ${it.region} ${it.project || ''}`,
      detail: `${KIND[it.kind]} · ${it.grade_text || it.grade} · ${fmtT(it.seconds)} · ${it.stage_text} · ${it.placement_text}`
        + (it.on_controller ? ` · ${onCtl(it)}` : ''),
      disabled: !!(it.why || differs), note: it.why ? `Cannot be merged: ${it.why}` : differs });
  }
  const lists = [...groups].map(([label, rows]) => ({ label, rows }));
  if (!lists.length && s.show === 'controller') {
    lists.push({ label: `${ARM(side)} controller`, rows: [{ value: '', text: 'No file here is on the controller', disabled: true,
      note: `${ctlWords(side)} Choose Show: All files to see every file.` }] });
  }
  choose(anchor, lists, null, (ref) => pick(side, ref), { placeholder: 'Search the files: name, region or project', files: true });
}

// What Lite knows of the arm's controller, for the list's count and its empty list.
function ctlWords(side) {
  const c = st[side].controller;
  if (!c) return `The ${ARM(side)} controller's programs are not known.`;
  if (c.standin) return `The stand-in robot holds the programs Lite sent it (${c.programs}).`;
  if (!c.read) return `The ${ARM(side)} controller has not been read: connect the arm.`;
  return `The ${ARM(side)} controller holds ${c.programs} program${c.programs === 1 ? '' : 's'} (read ${c.read}${c.live ? '' : ', not connected now'}).`;
}

// ── painting

function paint(side) {
  const s = st[side];
  const count = $(`liteS1Count_${side}`);
  if (!count) return;
  $(`liteS1Scope_${side}`).value = s.scope;
  $(`liteS1Show_${side}`).value = s.show;
  const listed = s.show === 'controller' ? s.items.filter(it => it.on_controller) : s.items;
  const free = listed.filter(it => !s.picked.includes(it.ref));
  const can = free.filter(it => !it.why && !(it.differs && it.differs.length)).length;
  const where = s.scope === 'all' ? 'in all projects' : 'in this project';
  count.textContent = !s.items.length
    ? `${s.scope === 'all' ? 'No project has' : 'The project has no'} ${ARM(side)} files yet: teach or load them in the region loop`
      + `${s.scope === 'all' ? '.' : ', or choose All projects.'}`
    : s.show === 'controller' && !listed.length
      ? `None of the ${s.items.length} ${ARM(side)} files ${where} is on the controller. ${ctlWords(side)} Choose Show: All files to see every file.`
      : `${s.show === 'controller' ? `${listed.length} of the ${s.items.length} ${ARM(side)} files ${where} ${listed.length === 1 ? 'is' : 'are'} on the controller` : `${s.items.length} ${ARM(side)} file${s.items.length === 1 ? '' : 's'} ${where}`}: `
        + `${can} can be added now${free.length > can ? `, ${free.length - can} greyed in the list with why` : ''}.`;

  const order = $(`liteS1Order_${side}`);
  const byRef = new Map(s.items.map(it => [it.ref, it]));
  order.replaceChildren(...(s.picked.length ? s.picked.map((ref, i) => {
    const it = byRef.get(ref) || { name: ref, seconds: 0 };
    const clash = i > 0 && it.differs && it.differs.length ? `Cannot join this merge: ${it.differs.join('; ')}` : '';   // the order changed
    return el('li', {}, el('span', {}, el('b', { text: it.name }), ` · ${fmtT(it.seconds)}`,
      el('small', { class: 'mini-note', text: ` ${KIND[it.kind] || ''}${it.grade_text ? ` · ${it.grade_text}` : ''} · ${it.project ? `${it.project} · ` : ''}${it.region || ''}` }),
      clash ? el('div', { class: 'lite-warn lite-s1-why', text: clash }) : null),
      el('span', { class: 'lite-s1-order-tools' },
        el('button', { class: 'btn light', type: 'button', text: 'Up', disabled: i === 0, onclick: () => move(side, i, -1) }),
        el('button', { class: 'btn light', type: 'button', text: 'Down', disabled: i === s.picked.length - 1, onclick: () => move(side, i, 1) }),
        el('button', { class: 'btn outline', type: 'button', text: 'Remove', onclick: () => unpick(side, ref) })));
  }) : [el('li', { class: 'mini-note lite-s1-empty', text: 'Add files above (Choose a file to add, or load them from this computer), in the order they run.' })]));
  if (s.picked.length) {                 // after the last row: the files' own times added (the owner, 20 Oct), no transitions between them
    const total = s.picked.reduce((a, ref) => a + Number((byRef.get(ref) || {}).seconds || 0), 0);
    order.append(el('li', { class: 'lite-s1-total' }, el('span', {}, el('b', { text: `Total ${fmtT(total)}` }),
      el('small', { class: 'mini-note', text: ` · ${s.picked.length} file${s.picked.length === 1 ? '' : 's'}, their own times added, no transition time between them` }))));
  }
  const name = $(`liteS1Name_${side}`);
  if (name && !name.value) name.value = `${ARM(side)[0]}_merge_${s.merges.length + 1}.txt`;

  const mb = $(`liteS1Merges_${side}`);
  mb.replaceChildren(...(s.merges.length ? s.merges.slice().reverse().map((m) => el('tr', { class: `lite-del-row${m.file === s.sel ? ' lite-picked' : ''}` },
    el('td', {}, el('input', { type: 'radio', name: `liteS1Sel_${side}`, checked: m.file === s.sel, 'aria-label': `Select ${m.name}`,
      onchange: () => select(side, m.file) })),
    el('td', {}, el('b', { text: m.name }), el('div', { class: 'mini-note', text: (m.made_from ? `Tuned from ${m.made_from.merge} · ` : '')
      + `${m.parts.length} file${m.parts.length === 1 ? '' : 's'}: ${m.parts.map(p => p.name).join(' → ')}`
      + (m.joins.length ? ` · ${m.joins.length} join${m.joins.length === 1 ? '' : 's'}` : '') }),
      onControllerOf(side, m) ? el('div', { class: 'lite-ok lite-s1-onctl', text: `On the ${ARM(side)} controller as program ${onControllerOf(side, m).id}`
        + (onControllerOf(side, m).by === 'name' ? ' (found by its name)' : '') }) : null),
    el('td', { text: `${fmtT(m.seconds)} · ${Number(m.lines).toLocaleString()} lines` }),
    el('td', { text: m.stage ? m.stage.text || stageText(m.stage) : '—' }),
    el('td', { text: m.placement ? placeText(m.placement) : 'not recorded' }),
    el('td', {}, el('span', { class: m.limits_ok ? 'lite-ok' : 'lite-warn', text: m.limits_ok ? 'L1–L4 pass' : `${m.limits_text} (shown, not enforced)` })),
    binCell(side, m)))
    : [el('tr', {}, el('td', { colspan: '7', class: 'mini-note', text: `No ${ARM(side)} merge yet.` }))]));
  steps(side);
  const sel = s.merges.find(m => m.file === s.sel);
  $(`liteS1Tools_${side}`).hidden = !sel;
  if (sel && $(`liteS1Rename_${side}`).dataset.for !== sel.file) {
    $(`liteS1Rename_${side}`).value = sel.name;
    $(`liteS1Rename_${side}`).dataset.for = sel.file;
  }
}

// Each merged file's bin (the owner, 21 Oct: "It lets the user delete any of those uploaded files from the controller";
// every row has it, as in Data Management): a merge on the arm's controller (S1's list knows which program holds it:
// merges.mark_on_controller) is deleted from there after the question, and the merge stays in the project; for one the
// controller does not hold as Lite reads it, the bin says why there is nothing to delete there.
const onControllerOf = (side, m) => (st[side].items.find(it => it.ref === `merge:${m.file}`) || {}).on_controller || null;

function notOnController(side, m) {
  const c = st[side].controller, A = ARM(side);
  const why = !c ? `the ${A} controller's programs are not known yet`
    : c.standin ? `Lite runs the stand-in robot here (no hardware): it holds only the programs Lite sent to it (${c.programs}), `
      + 'and this file is not one of them. A file sent to the robot is deleted with Lite started for the robot (--robot realman) '
      + 'and the arm connected'
      : !c.live ? `the ${A} arm is not connected (its programs as last read${c.read ? `, ${c.read}` : ''}): connect it, then read the controller`
        : `the ${A} controller (read ${c.read}) holds no program of its bytes or its name`;
  const text = `${m.name} is not on the ${A} controller, so there is nothing to delete there: ${why}.`;
  status(side, text, 'warn');
  window.toast(text, 'warn');
}

function binCell(side, m) {
  const c = onControllerOf(side, m);
  if (!c) return el('td', { class: 'lite-del-cell' }, deleteButton(`Delete ${m.name} from the ${ARM(side)} controller`, () => notOnController(side, m)));
  const what = `program ${c.id}${c.name ? ` (${c.name})` : ''}`;
  return el('td', { class: 'lite-del-cell' }, deleteButton(`Delete ${what}, ${m.name}, from the ${ARM(side)} controller`, async () => {
    const done = await deleteStored(side, 'program', c.id, { what,
      more: `It is ${m.name}${c.by === 'name' ? ' (found on the controller by its name)' : ''}; the merge stays in this project, to upload again. `
        + 'A sequence (S2) that runs it will not find it.' });
    if (!done) return;
    status(side, `${m.name}: program ${c.id} is deleted from the ${ARM(side)} controller; the merge stays here.`, 'ok');
    readSlots(side, true);
    await load(side);
  }));
}

// The step bar as the teaching flow's: the step to do now (blue), the steps done (green); each card's badge says how far.
function steps(side) {
  const s = st[side];
  const now = s.picked.length ? 2 : s.merges.length ? 3 : 1;
  for (let n = 1; n <= 3; n += 1) {
    const pipe = $(`liteS1Pipe${n}_${side}`);
    if (!pipe) continue;
    pipe.classList.toggle('active', n === now);
    pipe.classList.toggle('done', n < now);
    pipe.setAttribute('aria-current', n === now ? 'step' : 'false');
  }
  const badge = (n, text, cls) => { const b = $(`liteS1StepBadge${n}_${side}`); if (b) { b.textContent = text; b.className = `badge ${cls}`.trim(); } };
  badge(1, s.picked.length ? `${s.picked.length} PICKED` : 'NO FILE PICKED', s.picked.length ? 'green' : now === 1 ? 'blue' : '');
  badge(2, s.picked.length ? 'READY TO MERGE' : 'NOTHING TO MERGE', s.picked.length ? 'blue' : '');
  badge(3, s.merges.length ? `${s.merges.length} MERGED` : 'NONE YET', s.merges.length ? (now === 3 ? 'blue' : 'green') : '');
}

const stageText = (s) => `pole ${s.pole_mm} mm, hand ${(s.hand || []).join(', ')}`;
function placeText(p) {
  const moves = ['front_mm', 'rear_mm', 'left_mm', 'right_mm'].filter(k => p[k]).map(k => `${k.split('_')[0]} ${p[k]}`);
  return `standoff ${p.standoff_mm} mm${moves.length ? ` (moves: ${moves.join(', ')} mm)` : ''}`
    + (p.room ? `; room ${p.room.width_mm} × ${p.room.length_mm} mm` : '; room not measured');
}

// ── picking and merging

function pick(side, ref) {
  if (!st[side].picked.includes(ref)) st[side].picked.push(ref);
  repick(side);
}
function unpick(side, ref) {
  st[side].picked = st[side].picked.filter(r => r !== ref);
  repick(side);
}
function move(side, i, d) {
  const p = st[side].picked, j = i + d;
  if (j < 0 || j >= p.length) return;
  [p[i], p[j]] = [p[j], p[i]];
  repick(side);
}

async function merge(side) {
  const s = st[side];
  if (s.busy) return;                                      // one merge at a time (a double click makes one)
  if (!s.picked.length) { window.toast(`Add the ${ARM(side)} files to merge first.`, 'warn'); return; }
  const off = s.picked.slice(1).map(r => s.items.find(it => it.ref === r)).filter(it => it && it.differs && it.differs.length);
  if (off.length) { window.toast(`${off[0].name} cannot join this merge: ${off[0].differs.join('; ')}.`, 'warn'); return; }
  const name = ($(`liteS1Name_${side}`)?.value || '').trim();
  s.busy = true;
  status(side, 'Merging…');
  try {
    const m = await post(`${base()}/merges/${side}/from-project`, { name, files: s.picked });
    s.picked = [];
    s.sel = m.file;
    $(`liteS1Name_${side}`).value = '';
    status(side, `${m.name}: ${m.parts.length} file${m.parts.length === 1 ? '' : 's'}, ${m.joins.length} join${m.joins.length === 1 ? '' : 's'}, `
      + `${fmtT(m.seconds)}${m.limits_ok ? ', within L1–L4.' : `; it breaks ${m.limits_text}: shown, not enforced (sending it asks once).`}`,
    m.limits_ok ? 'ok' : 'warn');
  } catch (err) { status(side, say(err), 'warn'); tell(err); }
  s.busy = false;
  await load(side);
}

function select(side, file) {
  st[side].sel = file;
  paint(side);
  if (!slots[side]) readSlots(side, false);                // the controller's programs, for its Program ID
}

const selected = (side) => st[side].merges.find(m => m.file === st[side].sel) || null;

async function rename(side) {
  const m = selected(side);
  if (!m) return;
  try {
    const out = await post(murl(side, m.file, 'name'), { name: $(`liteS1Rename_${side}`).value });
    status(side, `Renamed: ${out.name}.`, 'ok');
    $(`liteS1Rename_${side}`).dataset.for = '';
  } catch (err) { status(side, say(err), 'warn'); tell(err); }
  await load(side);
}

async function download(side) {
  const m = selected(side);
  if (!m) return;
  try {
    const blob = await call('GET', murl(side, m.file, 'download'), undefined, { raw: true });
    const saved = await saveBlobAs(blob, m.name, [{ description: 'Trajectory', accept: { 'text/plain': ['.txt'] } }]);
    if (saved) window.toast(`Saved ${saved}.`, 'good');
  } catch (err) { tell(err); }
}

async function upload(side) {
  const m = selected(side);
  if (!m) return;
  if (st[side].sending) { window.toast(`${ARM(side)}: a file is being sent. Send the next one once it is on the controller.`, 'warn'); return; }
  const inp = $(`liteS1Pid_${side}`), pid = Number(inp.value);
  if (!(Number.isInteger(pid) && pid >= 1 && pid <= 100)) { inp.focus(); window.toast(`Enter the ${ARM(side)} Program ID (1 to 100).`, 'warn'); return; }
  st[side].sending = true;                                   // one send at a time (a double click sends once; the server holds it too)
  const btn = [...($(`liteS1Tools_${side}`)?.querySelectorAll('button') || [])].find(b => b.textContent === 'Upload to controller');
  if (btn) btn.disabled = true;
  try {
    const job = await robotCall(murl(side, m.file, 'upload'), { program_id: pid }, { what: `Sending ${m.name}`, replacing: `program ${pid}` });
    if (job && job.id) {
      const done = await watchJob(job, (j) => status(side, `${m.name}: ${jobLine(j)}`));
      if (done.state !== 'done') throw Object.assign(new Error(done.error || 'The upload did not finish.'), { hint: done.hint });
    }
    status(side, `${m.name} is on the ${ARM(side)} controller as program ${pid} (its name and size read back).`, 'ok');
    readSlots(side, false);                                  // its slot, as the controller holds it now
  } catch (err) { status(side, say(err), 'warn'); tell(err); } finally {
    st[side].sending = false;
    if (btn) btn.disabled = false;
  }
}

// The controller's programs, as Data Management lists them (the owner, 17 Oct: the Program ID's choices come from
// the controller, a program deleted there gone from them; "Read the controller" reads it again, completely, and
// every other list follows): offered under the Program ID, and what the slot typed holds said below it.
const slots = { left: null, right: null };

async function readSlots(side, refresh = false) {
  if (!ctx.pid) return;
  try {
    slots[side] = await get(`/api/robots/${side}/content?pid=${encodeURIComponent(ctx.pid)}${refresh ? '&refresh=1' : ''}`);
  } catch (err) { if (refresh) tell(err); return; }
  const dl = $(`liteS1Slots_${side}`);
  if (dl) dl.replaceChildren(...slots[side].programs.map(p => el('option', { value: String(p.id),
    label: `${p.name}${p.sent ? ` (sent by Lite${p.file_name ? `: ${p.file_name}` : ''})` : ''}` })));
  slotLine(side);
  if (refresh) {
    window.toast(`${ARM(side)} controller: ${slots[side].programs.length} programs${slots[side].read ? `, read ${slots[side].read}` : ''}.`
      + (slots[side].note ? ` ${slots[side].note}` : ''), 'good');
    document.dispatchEvent(new CustomEvent('lite:controller', { detail: { side, from: 's1' } }));
  }
}

function slotLine(side) {
  const line = $(`liteS1Slot_${side}`), c = slots[side];
  if (!line) return;
  const id = Number($(`liteS1Pid_${side}`)?.value);
  if (!c) { line.textContent = ''; return; }
  if (!c.read && !c.standin) { line.textContent = c.note || `The ${ARM(side)} controller has not been read.`; return; }
  if (!(Number.isInteger(id) && id >= 1 && id <= 100)) {
    line.textContent = `The ${ARM(side)} controller holds ${c.programs.length} programs${c.read ? ` (read ${c.read})` : ''}: type a Program ID, or choose one.`;
    return;
  }
  const p = c.programs.find(x => x.id === id);
  line.textContent = p ? `Program ${id} holds “${p.name}”${p.sent ? ', sent by Lite' : ''}: Upload replaces it (it asks first).`
    : `Program ${id} is free on the ${ARM(side)} controller${c.read ? ` (read ${c.read})` : ''}.`;
}

document.addEventListener('lite:controller', (e) => {
  const d = e.detail || {};
  if (d.from === 's1' || !built) return;
  for (const side of d.side ? [d.side] : ARMS) readSlots(side, false);
});

// The files of a merge that were tuned already, as the server reads them (merges.py: kept versions, merges kept
// from Tune, at any depth and from any project), or the merge itself when Tune kept it.
const tunedParts = (side, m) => m.tuned || [];

async function toTune(side) {
  const m = selected(side);
  if (!m) return;
  // parts tuned already (the owner, 16 Oct: "the user should be made aware, and decide how to proceed")
  const tuned = tunedParts(side, m);
  if (tuned.length) {
    const all = tuned.length === (m.sources || []).length;
    const yes = await confirmDialog({ title: `${m.name} has tuned files in it`, danger: false, ok: 'Tune it anyway',
      body: `${all ? 'Every file' : `${tuned.length} of its ${(m.sources || []).length} files`} in ${m.name} ${tuned.length === 1 ? 'was' : 'were'} `
        + 'tuned already. Tuning the merge times them again. Cancel keeps the merge as it is (it can be uploaded from S1 as it is).',
      list: tuned });
    if (!yes) return;
  }
  stopS1();
  try {
    const u = await post(murl(side, m.file, 'tune'), {});
    ctx.units[u.key] = u;
    ctx.tuneKey = u.key;
    ctx.tuneMerge = { side, file: m.file, name: m.name, parts: m.parts, joins: m.joins, sources: m.sources };
    window.showView('teach');
    await window.setStage('optimize');
    if (currentStage !== 'optimize') {                     // the page did not open Tune (Setup & Safety first)
      ctx.tuneKey = null;
      ctx.tuneMerge = null;
      return;
    }
    window.toast(`${m.name} is open in Tune + Simulation. What you keep there comes back to S1 as a new merge.`, 'good');
  } catch (err) { ctx.tuneKey = null; ctx.tuneMerge = null; tell(err); }
}

// ── the 3D review of a merge

let three = null;
// `again`: the merge reviewed before another view had the 3D, back at the moment it was at.
async function show3d(side, again = false) {
  const m = again ? st[side].merges.find(x => x.file === review.file) : selected(side);
  if (!m) return;
  const at = again ? player.t : 0;
  const my = ++review.token;
  const host = $('liteS1Host');
  frame(host).title(m.name, `${ARM(side)} arm · the merge as the robot runs it, staged first`);
  try {
    three = await borrow(host);
    const r = await post(murl(side, m.file, 'timeline'), {});
    if (my !== review.token) return;
    three.robot.home();
    for (const s of ARMS) if (s !== side) three.robot.clearTrack(s);
    const tl = three.robot.loadTimeline(side, r);
    Object.assign(review, { side, file: m.file, phase: m.phase || 'lid_closed', scene: r.scene || null, t0: tl.traj_t0, total: tl.total_s });
    tools.load(`${side}|${m.file}|${m.sha256 || ''}`, { phase: review.phase, text: "the merge's" });
    await scene(tools.phase || review.phase, undefined, review.scene);
    $('liteS1Timeline').max = String(Math.round(tl.total_s * 1000));
    tools.setLength(tl.total_s);
    $('liteS1Phases').textContent = (tl.phases || []).map(p => `${fmt(p.t0)}: ${p.text}`).join(' · ')
      + ` · the commode ${PHASE_WORDS[m.phase] || m.phase || 'lid closed'}`;
    player.load(tl.total_s);
    if (again) player.seek(Math.min(at, tl.total_s)); else player.play();
  } catch (err) {
    if (my === review.token) { window.toast(`Nothing plays: ${say(err)}`, 'error'); frame(host).title(m.name, `Nothing plays: ${say(err)}`); }
  }
}

function showAt(t) {
  if (three && review.side && three.viewer.host === $('liteS1Host')) {      // only while S1 has the 3D
    three.robot.showTime(t, review.side);
    three.viewer.dirty = true;
  }
  if (tools) tools.at(t);
  const tl = $('liteS1Timeline');
  if (tl && document.activeElement !== tl) tl.value = String(Math.round(t * 1000));
  frame($('liteS1Host')).time(`${fmt(t)} / ${fmt(review.total)}`);
}

let shownFor = null;

export function openS1(host, opts = {}) {
  if (panel !== host) { panel = host; built = false; }
  build();
  if (shownFor !== ctx.pid) {                              // another project: its own files, nothing picked
    shownFor = ctx.pid;
    for (const side of ARMS) Object.assign(st[side], fresh());
    for (const side of ARMS) { const n = $(`liteS1Name_${side}`); if (n) n.value = ''; }
    Object.assign(review, { side: null, file: null });
    tools.load(null, { enabled: false, text: "the merge's" });
  }
  if (!ctx.pid) {
    for (const side of ARMS) status(side, 'Open a project first.', 'warn');
    return;
  }
  if (opts.side && opts.file) st[opts.side].sel = opts.file;
  Promise.all(ARMS.map(load)).then(() => {
    // the merge in review before another view had the 3D comes back with the tab (each view loads when shown);
    // with none, the robot at home
    if (three && three.viewer.host === $('liteS1Host')) return;
    if (review.side && review.file) show3d(review.side, true);
    else home3d($('liteS1Host')).then((x) => { three = x; }).catch(() => {});
  });
}

// Leaving the page or the project: nothing of a merge keeps playing.
export function stopS1() { if (player) player.pause(); review.token += 1; }
