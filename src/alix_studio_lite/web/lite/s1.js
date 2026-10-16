// Merge + Full Sequence, S1: Merge & Upload, LEFT and RIGHT side by side (feedback v5, slide 9; the
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

import { call, get, post, say } from './api.js';
import { armFromName, confirmDialog, ctx, el, jobLine, robotCall, stageDialog, tell, watchJob } from './ctx.js';
import { borrow, home3d, Player } from './seq3d.js';
import { scene } from './sim.js';
import { saveBlobAs } from './tune.js';
import { checkJob, spansOf, Tools } from './view3d.js';
import { choose } from './picker.js';

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
const fresh = () => ({ items: [], picked: [], merges: [], sel: null, busy: false, scope: 'project' });
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

function build() {
  if (built) return;
  built = true;
  const btn = (text, onclick, cls = 'btn light') => el('button', { class: cls, type: 'button', text, onclick });
  panel.replaceChildren(
    el('div', { class: 'notice lite-s1-hint' }, el('b', { text: 'Which files merge: ' }),
      'one version per task, all raw (recordings) or all processed (tuned versions), with the same pole, hand, UGV placement and room '
      + '(each measurement within ±10 mm, the standoff ±5 mm). Pick the first file: the files that cannot join it are greyed out, with why. '
      + 'The merged program runs from one UGV position, with one pole and hand.'),
    el('div', { class: 'lite-s1-arms' }, ARMS.map((side) => el('div', { class: `card lite-s1-arm ${side}` },
      el('div', { class: 'lite-s1-head' }, el('span', { class: `badge ${side === 'left' ? 'blue' : 'orange'}`, text: `${ARM(side)} ARM` }),
        el('b', { text: `Merge the ${ARM(side)} arm's files` })),
      el('h4', { text: '1 · Files, picked in order' }),
      el('div', { class: 'lite-row lite-s1-pickrow' },
        el('div', { class: 'field' }, el('label', { for: `liteS1Scope_${side}`, text: 'Files from' }),
          el('select', { id: `liteS1Scope_${side}`, onchange: (e) => { st[side].scope = e.target.value; load(side); } },
            el('option', { value: 'project', text: 'This project' }), el('option', { value: 'all', text: 'All projects' }))),
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
          onchange: (e) => loadFiles(side, e.target) })),
      el('h4', { text: '2 · Merge order' }),
      el('ol', { id: `liteS1Order_${side}`, class: 'lite-s1-order' }),
      el('div', { class: 'lite-row' },
        el('div', { class: 'field lite-grow' }, el('label', { for: `liteS1Name_${side}`, text: 'Merged file name' }),
          el('input', { id: `liteS1Name_${side}`, type: 'text', autocomplete: 'off', maxlength: '120' })),
        btn('Merge and save', () => merge(side), 'btn')),
      el('h4', { text: '3 · Merged files' }),
      el('div', { class: 'lite-table-scroll lite-s1-merges' }, el('table', { class: 'lite-table' },
        el('thead', {}, el('tr', {}, ['', 'Merge', 'Time', 'Pole and hand', 'UGV placement', 'Limits'].map(h => el('th', { text: h })))),
        el('tbody', { id: `liteS1Merges_${side}` }))),
      el('div', { id: `liteS1Tools_${side}`, class: 'lite-s1-tools', hidden: true },
        el('div', { class: 'lite-row' },
          el('div', { class: 'field lite-grow' }, el('label', { for: `liteS1Rename_${side}`, text: 'Name' }),
            el('input', { id: `liteS1Rename_${side}`, type: 'text', autocomplete: 'off', maxlength: '120' })),
          btn('Rename', () => rename(side)), btn('Download', () => download(side), 'btn outline')),
        el('div', { class: 'lite-row' },
          el('div', { class: 'field' }, el('label', { for: `liteS1Pid_${side}`, text: 'Program ID' }),
            el('input', { id: `liteS1Pid_${side}`, type: 'number', min: '1', max: '100', step: '1', placeholder: '1–100' })),
          btn('Upload to controller', () => upload(side), 'btn'),
          btn('Review in 3D', () => show3d(side), 'btn green'),
          btn('Send to Tune + Simulation', () => toTune(side), 'btn outline'))),
      el('div', { id: `liteS1Status_${side}`, class: 'lite-status' })))),
    el('div', { class: 'card lite-s1-sim' },
      el('div', { class: 'lite-section-title' },
        el('h3', {}, 'Simulation · ', el('span', { id: 'liteS1SimName', text: 'choose a merged file, then Review in 3D' }))),
      el('div', { class: 'lite-s1-sim-grid' },
        el('div', { class: 'lite-rev3d-main' }, el('div', { id: 'liteS1Host', class: 'lite-rev3d-host' })),
        el('div', { class: 'lite-rev3d-side' },
          el('div', { class: 'lite-row' }, btn('Play', () => player.play(), 'btn'), btn('Pause', () => player.pause(), 'btn outline'),
            btn('Restart', () => { player.pause(); player.seek(0); }), el('b', { id: 'liteS1Time', class: 'lite-rev3d-time', text: '0.00 s / 0.00 s' })),
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
}

// ── data

async function load(side) {
  try {
    const [src, all] = await Promise.all([post(`${base()}/merge-sources/${side}`, { picked: st[side].picked, scope: st[side].scope }),
      get(`${base()}/merges`)]);
    st[side].items = src.items;
    st[side].picked = st[side].picked.filter(r => src.items.some(it => it.ref === r));
    st[side].merges = all[side] || [];
    if (st[side].sel && !st[side].merges.some(m => m.file === st[side].sel)) st[side].sel = null;
  } catch (err) { status(side, `The files could not be read: ${say(err)}`, 'warn'); }
  paint(side);
}

async function repick(side) {
  try {
    const src = await post(`${base()}/merge-sources/${side}`, { picked: st[side].picked, scope: st[side].scope });
    st[side].items = src.items;
  } catch (err) { tell(err); }
  paint(side);
}

// Files from this computer (a folder too) kept in the project to merge, with the pole and hand the operator says
// they were taught with, after the warning; a file named for the other arm is left out (server/merges.py).
async function loadFiles(side, input) {
  const files = [...(input.files || [])].filter(f => /\.txt$/i.test(f.name));
  input.value = '';
  if (!files.length) { window.toast('Choose .txt trajectory files.'); return; }
  const where = (f) => f.webkitRelativePath || f.name;
  const other = files.filter(f => { const a = armFromName(where(f)); return a && a !== side; });
  const mine = files.filter(f => !other.includes(f));
  const otherArm = ARM(side === 'left' ? 'right' : 'left');
  if (!mine.length) { window.toast(`${other.length === 1 ? `${other[0].name} is` : 'These files are'} named for the ${otherArm} arm: never merged on the ${ARM(side)} arm.`); return; }
  const said = await stageDialog({ title: `Load ${mine.length === 1 ? mine[0].name : `${mine.length} files`} into the ${ARM(side)} merges`,
    body: LOAD_WARNING, arm: side });
  if (!said) { status(side, 'Nothing loaded: without their pole and hand, files cannot be checked against the others.', 'warn'); return; }
  let ok = 0;
  const refused = [];
  for (const f of mine) {
    try {
      await post(`${base()}/merge-files/${side}`, { name: f.name, path: where(f), content: await f.text(), pole_mm: said.pole_mm, hand: said.hand });
      ok += 1;
    } catch (err) { refused.push(`${f.name}: ${say(err)}`); }
  }
  status(side, `${ok} file${ok === 1 ? '' : 's'} loaded: in the list under Loaded from this computer.`
    + (other.length ? ` ${other.length} named for the ${otherArm} arm left out.` : '') + (refused.length ? ` Refused: ${refused.join('; ')}` : ''),
  refused.length || other.length ? 'warn' : 'ok');
  await load(side);
}

// The files to add as a searchable list (picker.js): by project and region; a file that cannot join is
// greyed, with why (another pole, hand, placement or room than the first file; its arm or stage unknown).
function chooseFile(side, anchor) {
  const s = st[side];
  const groups = new Map();
  for (const it of s.items) {
    if (s.picked.includes(it.ref)) continue;                 // in the order already
    const label = `${it.project ? `${it.project} · ` : s.scope === 'all' ? 'This project · ' : ''}${it.region}`;
    if (!groups.has(label)) groups.set(label, []);
    const differs = it.differs && it.differs.length ? `Differs from the first file: ${it.differs.join('; ')}` : '';
    groups.get(label).push({ value: it.ref, text: it.name, search: `${KIND[it.kind]} ${it.grade} ${it.region} ${it.project || ''}`,
      detail: `${KIND[it.kind]} (${it.grade}) · ${fmt(it.seconds)} · ${it.stage_text} · ${it.placement_text}`,
      disabled: !!(it.why || differs), note: it.why ? `Cannot be merged: ${it.why}` : differs });
  }
  choose(anchor, [...groups].map(([label, rows]) => ({ label, rows })), null, (ref) => pick(side, ref),
    { placeholder: 'Search the files: name, region or project' });
}

// ── painting

function paint(side) {
  const s = st[side];
  const count = $(`liteS1Count_${side}`);
  if (!count) return;
  $(`liteS1Scope_${side}`).value = s.scope;
  const free = s.items.filter(it => !s.picked.includes(it.ref));
  const can = free.filter(it => !it.why && !(it.differs && it.differs.length)).length;
  count.textContent = s.items.length
    ? `${s.items.length} ${ARM(side)} file${s.items.length === 1 ? '' : 's'} ${s.scope === 'all' ? 'in all projects' : 'in this project'}: `
      + `${can} can be added now${free.length > can ? `, ${free.length - can} greyed in the list with why` : ''}.`
    : `${s.scope === 'all' ? 'No project has' : 'The project has no'} ${ARM(side)} files yet: teach or load them in the region loop`
      + `${s.scope === 'all' ? '.' : ', or choose All projects.'}`;

  const order = $(`liteS1Order_${side}`);
  const byRef = new Map(s.items.map(it => [it.ref, it]));
  order.replaceChildren(...(s.picked.length ? s.picked.map((ref, i) => {
    const it = byRef.get(ref) || { name: ref, seconds: 0 };
    const clash = i > 0 && it.differs && it.differs.length ? `Differs from the first file: ${it.differs.join('; ')}` : '';   // the order changed
    return el('li', {}, el('span', {}, el('b', { text: it.name }), ` · ${fmt(it.seconds)}`,
      el('small', { class: 'mini-note', text: ` ${KIND[it.kind] || ''} · ${it.project ? `${it.project} · ` : ''}${it.region || ''}` }),
      clash ? el('div', { class: 'lite-warn lite-s1-why', text: clash }) : null),
      el('span', { class: 'lite-s1-order-tools' },
        el('button', { class: 'btn light', type: 'button', text: 'Up', disabled: i === 0, onclick: () => move(side, i, -1) }),
        el('button', { class: 'btn light', type: 'button', text: 'Down', disabled: i === s.picked.length - 1, onclick: () => move(side, i, 1) }),
        el('button', { class: 'btn outline', type: 'button', text: 'Remove', onclick: () => unpick(side, ref) })));
  }) : [el('li', { class: 'mini-note lite-s1-empty', text: 'Add files above (Choose a file to add), in the order they run.' })]));
  const name = $(`liteS1Name_${side}`);
  if (name && !name.value) name.value = `${ARM(side)[0]}_merge_${s.merges.length + 1}.txt`;

  const mb = $(`liteS1Merges_${side}`);
  mb.replaceChildren(...(s.merges.length ? s.merges.slice().reverse().map((m) => el('tr', { class: m.file === s.sel ? 'lite-picked' : '' },
    el('td', {}, el('input', { type: 'radio', name: `liteS1Sel_${side}`, checked: m.file === s.sel, 'aria-label': `Select ${m.name}`,
      onchange: () => select(side, m.file) })),
    el('td', {}, el('b', { text: m.name }), el('div', { class: 'mini-note', text: (m.made_from ? `Tuned from ${m.made_from.merge} · ` : '')
      + `${m.parts.length} file${m.parts.length === 1 ? '' : 's'}: ${m.parts.map(p => p.name).join(' → ')}`
      + (m.joins.length ? ` · ${m.joins.length} join${m.joins.length === 1 ? '' : 's'}` : '') })),
    el('td', { text: `${fmt(m.seconds)} · ${Number(m.lines).toLocaleString()} lines` }),
    el('td', { text: m.stage ? m.stage.text || stageText(m.stage) : '—' }),
    el('td', { text: m.placement ? placeText(m.placement) : 'not recorded' }),
    el('td', {}, el('span', { class: m.limits_ok ? 'lite-ok' : 'lite-warn', text: m.limits_ok ? 'L1–L4 pass' : `${m.limits_text} (shown, not enforced)` }))))
    : [el('tr', {}, el('td', { colspan: '6', class: 'mini-note', text: `No ${ARM(side)} merge yet.` }))]));
  const sel = s.merges.find(m => m.file === s.sel);
  $(`liteS1Tools_${side}`).hidden = !sel;
  if (sel && $(`liteS1Rename_${side}`).dataset.for !== sel.file) {
    $(`liteS1Rename_${side}`).value = sel.name;
    $(`liteS1Rename_${side}`).dataset.for = sel.file;
  }
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
  if (!s.picked.length) { window.toast(`Add the ${ARM(side)} files to merge first.`); return; }
  const off = s.picked.slice(1).map(r => s.items.find(it => it.ref === r)).filter(it => it && it.differs && it.differs.length);
  if (off.length) { window.toast(`${off[0].name} differs from the first file: ${off[0].differs.join('; ')}.`); return; }
  const name = ($(`liteS1Name_${side}`)?.value || '').trim();
  s.busy = true;
  status(side, 'Merging…');
  try {
    const m = await post(`${base()}/merges/${side}/from-project`, { name, files: s.picked });
    s.picked = [];
    s.sel = m.file;
    $(`liteS1Name_${side}`).value = '';
    status(side, `${m.name}: ${m.parts.length} file${m.parts.length === 1 ? '' : 's'}, ${m.joins.length} join${m.joins.length === 1 ? '' : 's'}, `
      + `${fmt(m.seconds)}${m.limits_ok ? ', within L1–L4.' : `; it breaks ${m.limits_text}: shown, not enforced (sending it asks once).`}`,
    m.limits_ok ? 'ok' : 'warn');
  } catch (err) { status(side, say(err), 'warn'); tell(err); }
  s.busy = false;
  await load(side);
}

function select(side, file) {
  st[side].sel = file;
  paint(side);
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
    if (saved) window.toast(`Saved ${saved}.`);
  } catch (err) { tell(err); }
}

async function upload(side) {
  const m = selected(side);
  if (!m) return;
  const inp = $(`liteS1Pid_${side}`), pid = Number(inp.value);
  if (!(Number.isInteger(pid) && pid >= 1 && pid <= 100)) { inp.focus(); window.toast(`Enter the ${ARM(side)} Program ID (1 to 100).`); return; }
  try {
    const job = await robotCall(murl(side, m.file, 'upload'), { program_id: pid }, { what: `Sending ${m.name}`, replacing: `program ${pid}` });
    if (job && job.id) {
      const done = await watchJob(job, (j) => status(side, `${m.name}: ${jobLine(j)}`));
      if (done.state !== 'done') throw Object.assign(new Error(done.error || 'The upload did not finish.'), { hint: done.hint });
    }
    status(side, `${m.name} is on the ${ARM(side)} controller as program ${pid} (its name and size read back).`, 'ok');
  } catch (err) { status(side, say(err), 'warn'); tell(err); }
}

// The files of a merge that were tuned already: kept versions, and merges kept from Tune (each in any project).
function tunedParts(side, m) {
  const merges = new Map(st[side].merges.map(x => [x.file, x]));
  return (m.sources || []).filter((src) => {
    const ref = String(src.ref || '').split('|').pop();
    if (ref.startsWith('ver:')) return true;
    if (ref.startsWith('merge:')) { const x = merges.get(ref.slice(6)); return !!(x && x.made_from); }
    return false;
  }).map(src => src.name);
}

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
    window.toast(`${m.name} is open in Tune + Simulation. What you keep there comes back to S1 as a new merge.`);
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
  $('liteS1SimName').textContent = `${m.name} (${ARM(side)} arm)`;
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
    if (my === review.token) { window.toast(`Nothing plays: ${say(err)}`); $('liteS1SimName').textContent = `${m.name}: nothing plays (${say(err)})`; }
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
  if ($('liteS1Time')) $('liteS1Time').textContent = `${fmt(t)} / ${fmt(review.total)}`;
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
