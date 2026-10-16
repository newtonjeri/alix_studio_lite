// Merge + Full Sequence, S1: Merge & Upload, LEFT and RIGHT side by side (feedback v5, slide 9; the
// owner, 16 Oct). For each arm:
// 1. the project's files of that arm (every region's recordings and kept versions, and the arm's
//    merges), picked in order. Only files with the same pole, hand, UGV placement and room (each
//    measurement within ±10 mm) merge: once one is picked, the files that differ are greyed out
//    with what differs (server/merges.py says it, and refuses a merge that differs);
// 2. merged and saved under a name, the files joined where one ends away from where the next
//    begins (merges.py), the merge keeping their pole, hand, placement and phase;
// 3. each merge renamed, downloaded, sent to a Program ID on the arm's controller, reviewed in 3D
//    (below: staged first, then the file on the 5 ms clock, in the room and at the UGV placement its
//    files were taught at), or sent to Tune + Simulation: Tune opens it as its source, and what Tune
//    keeps comes back here as a new merge.

import { call, get, post, say } from './api.js';
import { ctx, el, jobLine, robotCall, tell, watchJob } from './ctx.js';
import { borrow, Player } from './seq3d.js';
import { scene } from './sim.js';
import { saveBlobAs } from './tune.js';

const $ = (id) => document.getElementById(id);
const ARMS = ['left', 'right'];
const ARM = (s) => s.toUpperCase();
const KIND = { rec: 'Recording', ver: 'Kept version', merge: 'Merge' };
const PHASE_WORDS = { lid_closed: 'lid closed', lid_open: 'lid open', seat_open: 'seat open' };
const st = { left: { items: [], picked: [], merges: [], sel: null, busy: false }, right: { items: [], picked: [], merges: [], sel: null, busy: false } };
const review = { side: null, file: null, total: 0, token: 0 };
let panel = null, player = null, built = false;

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
      'only files with the same pole, hand, UGV placement and room (each measurement within ±10 mm). Pick the first file: the files that '
      + 'differ from it are greyed out, with what differs. The merged program runs from one UGV position, with one pole and hand.'),
    el('div', { class: 'lite-s1-arms' }, ARMS.map((side) => el('div', { class: `card lite-s1-arm ${side}` },
      el('div', { class: 'lite-s1-head' }, el('span', { class: `badge ${side === 'left' ? 'blue' : 'orange'}`, text: `${ARM(side)} ARM` }),
        el('b', { text: `Merge the ${ARM(side)} arm's files` })),
      el('h4', { text: '1 · Project files, picked in order' }),
      el('div', { class: 'lite-table-scroll lite-s1-files' }, el('table', { class: 'lite-table' },
        el('thead', {}, el('tr', {}, ['', 'File', 'Time', 'Pole and hand', 'UGV placement and room'].map(h => el('th', { text: h })))),
        el('tbody', { id: `liteS1Files_${side}` }))),
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
        el('h3', {}, 'Simulation · ', el('span', { id: 'liteS1SimName', text: 'choose a merged file, then Review in 3D' })),
        el('div', { class: 'lite-rev3d-views', role: 'group', 'aria-label': 'View' },
          [['default', 'Default'], ['front', 'Front'], ['side', 'Side'], ['top', 'Top']].map(([v, n]) => btn(n, () => setView(v), 'btn light lite-rev3d-view')))),
      el('div', { class: 'lite-s1-sim-grid' },
        el('div', { id: 'liteS1Host', class: 'lite-rev3d-host' }),
        el('div', { class: 'lite-rev3d-side' },
          el('div', { class: 'lite-row' }, btn('Play', () => player.play(), 'btn'), btn('Pause', () => player.pause(), 'btn outline'),
            btn('Restart', () => { player.pause(); player.seek(0); }), el('b', { id: 'liteS1Time', class: 'lite-rev3d-time', text: '0.00 s / 0.00 s' })),
          el('input', { id: 'liteS1Timeline', type: 'range', min: '0', max: '0', step: '5', value: '0', 'aria-label': 'Merge time',
            oninput: (e) => { player.pause(); player.seek(Number(e.target.value) / 1000); } }),
          el('div', { id: 'liteS1SimNote', class: 'mini-note', text: 'The merged program as the robot runs it: the pole and the hand staged first, '
            + 'the move to its first line, then the file on the 5 ms clock, in the room and at the UGV placement its files were taught at.' }),
          el('div', { id: 'liteS1Phases', class: 'mini-note' })))));
  player = new Player({ onTime: showAt });
}

// ── data

async function load(side) {
  try {
    const [src, all] = await Promise.all([post(`${base()}/merge-sources/${side}`, { picked: st[side].picked }), get(`${base()}/merges`)]);
    st[side].items = src.items;
    st[side].picked = st[side].picked.filter(r => src.items.some(it => it.ref === r));
    st[side].merges = all[side] || [];
    if (st[side].sel && !st[side].merges.some(m => m.file === st[side].sel)) st[side].sel = null;
  } catch (err) { status(side, `The files could not be read: ${say(err)}`, 'warn'); }
  paint(side);
}

async function repick(side) {
  try {
    const src = await post(`${base()}/merge-sources/${side}`, { picked: st[side].picked });
    st[side].items = src.items;
  } catch (err) { tell(err); }
  paint(side);
}

// ── painting

function paint(side) {
  const s = st[side];
  const body = $(`liteS1Files_${side}`);
  if (!body) return;
  const pickedSet = new Set(s.picked);
  body.replaceChildren(...(s.items.length ? s.items.map((it) => {
    const n = s.picked.indexOf(it.ref) + 1;
    const off = it.why || (!pickedSet.has(it.ref) && it.differs && it.differs.length ? `Differs: ${it.differs.join('; ')}` : '');
    return el('tr', { class: `${off ? 'lite-off' : ''}${n ? ' lite-picked' : ''}`, title: off || '' },
      el('td', {}, n ? el('button', { class: 'btn outline', type: 'button', text: `Remove (${n})`, onclick: () => unpick(side, it.ref) })
        : el('button', { class: 'btn light', type: 'button', text: 'Add', disabled: !!off, onclick: () => pick(side, it.ref) })),
      el('td', {}, el('b', { text: it.name }), el('div', { class: 'mini-note', text: `${KIND[it.kind]} · ${it.region}` }),
        off ? el('div', { class: 'lite-warn lite-s1-why', text: off }) : null),
      el('td', { text: fmt(it.seconds) }),
      el('td', { text: it.stage_text }),
      el('td', { text: it.placement_text }));
  }) : [el('tr', {}, el('td', { colspan: '5', class: 'mini-note', text: `The project has no ${ARM(side)} files yet: teach or load them in the region loop.` }))]));

  const order = $(`liteS1Order_${side}`);
  const byRef = new Map(s.items.map(it => [it.ref, it]));
  order.replaceChildren(...(s.picked.length ? s.picked.map((ref, i) => {
    const it = byRef.get(ref) || { name: ref, seconds: 0 };
    return el('li', {}, el('span', { text: `${it.name} · ${fmt(it.seconds)}` }),
      el('span', { class: 'lite-s1-order-tools' },
        el('button', { class: 'btn light', type: 'button', text: 'Up', disabled: i === 0, onclick: () => move(side, i, -1) }),
        el('button', { class: 'btn light', type: 'button', text: 'Down', disabled: i === s.picked.length - 1, onclick: () => move(side, i, 1) }),
        el('button', { class: 'btn outline', type: 'button', text: 'Remove', onclick: () => unpick(side, ref) })));
  }) : [el('li', { class: 'mini-note lite-s1-empty', text: 'Add files above, in the order they run.' })]));
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
  if (!s.picked.length) { window.toast(`Add the ${ARM(side)} files to merge first.`); return; }
  const name = ($(`liteS1Name_${side}`)?.value || '').trim();
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

async function toTune(side) {
  const m = selected(side);
  if (!m) return;
  try {
    const u = await post(murl(side, m.file, 'tune'), {});
    ctx.units[u.key] = u;
    ctx.tuneKey = u.key;
    ctx.tuneMerge = { side, file: m.file, name: m.name };
    window.showView('teach');
    await window.setStage('optimize');
    window.toast(`${m.name} is open in Tune + Simulation. What you keep there comes back to S1 as a new merge.`);
  } catch (err) { tell(err); }
}

// ── the 3D review of a merge

async function setView(name) {
  try { const { ensure3D } = await import('./sim.js'); const { viewer } = await ensure3D(); viewer.setView(name); } catch (_) { /* no 3D */ }
}

let three = null;
async function show3d(side) {
  const m = selected(side);
  if (!m) return;
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
    await scene(m.phase || 'lid_closed', undefined, r.scene || null);
    Object.assign(review, { side, file: m.file, total: tl.total_s });
    $('liteS1Timeline').max = String(Math.round(tl.total_s * 1000));
    $('liteS1Phases').textContent = (tl.phases || []).map(p => `${fmt(p.t0)}: ${p.text}`).join(' · ')
      + ` · the commode ${PHASE_WORDS[m.phase] || m.phase || 'lid closed'}`;
    player.load(tl.total_s);
    player.play();
  } catch (err) {
    if (my === review.token) { window.toast(`Nothing plays: ${say(err)}`); $('liteS1SimName').textContent = `${m.name}: nothing plays (${say(err)})`; }
  }
}

function showAt(t) {
  if (three && review.side) { three.robot.showTime(t, review.side); three.viewer.dirty = true; }
  const tl = $('liteS1Timeline');
  if (tl && document.activeElement !== tl) tl.value = String(Math.round(t * 1000));
  if ($('liteS1Time')) $('liteS1Time').textContent = `${fmt(t)} / ${fmt(review.total)}`;
}

export function openS1(host, opts = {}) {
  if (panel !== host) { panel = host; built = false; }
  build();
  if (!ctx.pid) {
    for (const side of ARMS) status(side, 'Open a project first.', 'warn');
    return;
  }
  if (opts.side && opts.file) st[opts.side].sel = opts.file;
  for (const side of ARMS) load(side);
}

// Leaving the page or the project: nothing of a merge keeps playing.
export function stopS1() { if (player) player.pause(); review.token += 1; }
