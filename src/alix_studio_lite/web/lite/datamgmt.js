// Data Management (the owner, 16 Oct: "a data management tab similar to what we have the controller to
// help us display, the saved projects ... with selecter buttons for left and right arms ... accessible via
// a button at the top ribbon"; "will list all that information in the attached screenshot per arm, with the
// same columns sorting features"): what each arm's controller stores, laid out as RealMan's own web page
// lays it out (its Data Management: Tracks List, Graphical Program, Script Program, Model Warehouse,
// Global Points; the Graphical Program's columns Name, Num, Create Time, Edit Time, File Size, Default
// Speed, sorted by Num, Create Time or Edit Time; a search and pages). Display only: nothing here changes
// a controller. Read from the arm's controller when the window opens, when the other arm is chosen and on
// Refresh (GET /api/robots/<arm>/data), never while the arm moves. What RealMan's API cannot give is said:
// the programs' own create and edit times (Lite's record of when it sent a program stands in), the script
// programs and the model warehouse, and the tracks list on these 3rd-generation controllers.

import { get, say } from './api.js';
import { ctx, el } from './ctx.js';

const $ = (id) => document.getElementById(id);
const ARMS = ['left', 'right'];
const TABS = [['tracks', 'Tracks List'], ['programs', 'Graphical Program'], ['script', 'Script Program'], ['models', 'Model Warehouse'],
  ['points', 'Global Points']];
const data = { left: null, right: null };               // the last read of each arm: the server's answer
const failed = { left: null, right: null };             // the last refusal (busy, not connected)
let box = null, side = 'left', tab = 'programs', q = '', page = 1, size = 20, loading = 0;
const sorts = { programs: { key: 'num', dir: 1 }, points: { key: null, dir: 1 }, tracks: { key: null, dir: 1 } };

const dash = '—';
const kb = (b) => (b == null ? dash : `${(Number(b) / 1024).toFixed(3)} KB`);
const deg = (r) => (Number(r) * 180) / Math.PI;
const words = (s) => String(s || '').toLowerCase().split(/\s+/).filter(Boolean);
const fits = (text, ws) => { const t = String(text || '').toLowerCase(); return ws.every(w => t.includes(w)); };

// Each tab's columns: [key, title, value(row) for its cell and its sort, sortable].
const COLUMNS = {
  programs: [
    ['name', 'Name', (r) => r.name || '', false],
    ['num', 'Num', (r) => r.num, true],
    ['created', 'Create Time', (r) => (r.lite && r.lite.sent) || '', true],
    ['edited', 'Edit Time', (r) => (r.lite && r.lite.sent) || '', true],
    ['size', 'File Size', (r) => r.bytes, false],
    ['speed', 'Default Speed', (r) => r.speed, false]],
  points: [
    ['name', 'Name', (r) => r.name, true],
    ['joints', 'Joints J1–J7 (deg)', (r) => (r.joints || []).join(','), false],
    ['pose', 'Pose X Y Z (mm) · RX RY RZ (deg)', (r) => (r.pose || []).join(','), false],
    ['work', 'Work Frame', (r) => r.work_frame, false],
    ['tool', 'Tool Frame', (r) => r.tool_frame, false],
    ['time', 'Time', (r) => r.time || '', true]],
  tracks: [
    ['name', 'Name', (r) => r.name, true],
    ['points', 'Points', (r) => r.points, false],
    ['created', 'Create Time', (r) => r.created || '', true]],
};

function cell(t, r, key) {
  if (t === 'programs') {
    if (key === 'name') {
      const d = data[side];
      return el('td', { class: 'lite-dm-name' }, el('b', { text: r.name || dash }),
        d && d.default_program === r.num ? el('span', { class: 'lite-dm-badge', text: 'IO default' }) : null,
        r.lite ? el('small', { text: `Sent by Lite${r.lite.project_file ? `: ${r.lite.project_file}` : r.lite.file ? `: ${r.lite.file}` : ''}`
          + `${r.lite.region ? ` (${r.lite.region})` : ''}${r.lite.lines ? ` · ${Number(r.lite.lines).toLocaleString()} lines` : ''}` }) : null);
    }
    if (key === 'num') return el('td', { text: String(r.num) });
    if (key === 'created' || key === 'edited') return el('td', { text: (r.lite && r.lite.sent) || dash, title: r.lite ? 'When Lite sent it (Lite’s record)' : '' });
    if (key === 'size') return el('td', { text: kb(r.bytes) });
    if (key === 'speed') return el('td', { text: r.speed == null ? dash : String(r.speed) });
  }
  if (t === 'points') {
    if (key === 'name') return el('td', {}, el('b', { text: r.name }));
    if (key === 'joints') return el('td', { class: 'lite-dm-num', text: (r.joints || []).map(v => Number(v).toFixed(2)).join(', ') });
    if (key === 'pose') {
      const p = r.pose || [];
      return el('td', { class: 'lite-dm-num', text: p.length < 6 ? dash
        : `${p.slice(0, 3).map(v => (Number(v) * 1000).toFixed(1)).join(', ')} · ${p.slice(3, 6).map(v => deg(v).toFixed(2)).join(', ')}` });
    }
    if (key === 'work') return el('td', { text: r.work_frame || dash });
    if (key === 'tool') return el('td', { text: r.tool_frame || dash });
    if (key === 'time') return el('td', { text: r.time || dash });
  }
  if (t === 'tracks') {
    if (key === 'name') return el('td', {}, el('b', { text: r.name }));
    if (key === 'points') return el('td', { text: r.points == null ? dash : Number(r.points).toLocaleString() });
    if (key === 'created') return el('td', { text: r.created || dash });
  }
  return el('td', { text: dash });
}

function rowsOf(t) {
  const d = data[side];
  if (!d) return null;
  if (t === 'programs') return d.programs || [];
  if (t === 'points') return d.points || [];
  if (t === 'tracks') return d.tracks && d.tracks.available ? d.tracks.list : null;
  return null;
}

function sorted(t, rows) {
  const s = sorts[t];
  if (!s || !s.key) return rows;
  const col = COLUMNS[t].find(c => c[0] === s.key);
  const v = col[2];
  return [...rows].sort((a, b) => {
    const x = v(a), y = v(b);
    const blank = (z) => z === '' || z == null;
    if (blank(x) !== blank(y)) return blank(x) ? 1 : -1;          // what is not known goes last, either way
    return (typeof x === 'number' && typeof y === 'number' ? x - y : String(x).localeCompare(String(y), undefined, { numeric: true })) * s.dir;
  });
}

// ── painting

function paintStatus() {
  const d = data[side], why = failed[side];
  const s = $('liteDmStatus');
  if (!s) return;
  const ARM = side.toUpperCase();
  if (loading) { s.className = 'lite-status'; s.textContent = `Reading the ${ARM} controller…`; return; }
  if (why) {
    s.className = 'lite-status warn';
    s.textContent = `${say(why)}${d ? ` Shown: as read at ${d.read}.` : ''}`;
    return;
  }
  if (!d) { s.className = 'lite-status'; s.textContent = ''; return; }
  s.className = 'lite-status ok';
  s.textContent = d.linked
    ? `${ARM} controller${d.ip ? ` (${d.ip})` : ''} · read at ${d.read} · ${d.programs.length} programs, ${(d.points || []).length} global points`
      + `${d.default_program ? ` · its IO starts program ${d.default_program}` : ''}.`
    : d.note;
}

function paintTabs() {
  for (const [id] of TABS) {
    const b = $(`liteDmTab_${id}`);
    b.classList.toggle('active', id === tab);
    b.setAttribute('aria-selected', String(id === tab));
  }
  for (const s of ARMS) {
    const b = $(`liteDmArm_${s}`);
    b.className = `btn lite-dm-arm ${s}${s === side ? '' : ' light'}`;          // the arm shown: the page's main button
    b.setAttribute('aria-pressed', String(s === side));
  }
}

function paintTable() {
  const area = $('liteDmTable'), pager = $('liteDmPager'), note = $('liteDmNote');
  const d = data[side];
  const rows = rowsOf(tab);
  $('liteDmSearch').disabled = rows == null;
  note.textContent = '';
  if (rows == null) {
    pager.replaceChildren();
    let text;
    if (!d) text = loading ? '' : 'Nothing read yet: Refresh reads the controller.';
    else if (tab === 'tracks') text = d.tracks ? d.tracks.why : 'Not read.';
    else if (tab === 'script' || tab === 'models') text = `${d.not_in_api[tab]}${d.ip ? ` Its address: ${d.ip}.` : ''}`;
    else if (tab === 'points') text = d.linked ? 'Not read.' : 'The stand-in robot has no controller, so no global points.';
    area.replaceChildren(el('div', { class: 'lite-dm-empty', text }));
    return;
  }
  const ws = words(q);
  const shown = sorted(tab, rows.filter(r => fits(r.name, ws)));
  const pages = Math.max(1, Math.ceil(shown.length / size));
  page = Math.min(Math.max(1, page), pages);
  const part = shown.slice((page - 1) * size, page * size);
  const cols = COLUMNS[tab];
  const head = el('tr', {}, cols.map(([key, title, , sortable]) => {
    if (!sortable) return el('th', { scope: 'col', text: title });
    const s = sorts[tab];
    const dir = s.key === key ? (s.dir > 0 ? 'ascending' : 'descending') : 'none';
    return el('th', { scope: 'col', 'aria-sort': dir },
      el('button', { type: 'button', class: `lite-dm-sort${s.key === key ? (s.dir > 0 ? ' asc' : ' desc') : ''}`, text: title,
        title: `Sort by ${title}`, onclick: () => { sorts[tab] = { key, dir: s.key === key ? -s.dir : 1 }; page = 1; paintTable(); } }));
  }));
  area.replaceChildren(el('table', { class: 'lite-table lite-dm-table' }, el('thead', {}, head),
    el('tbody', {}, part.length ? part.map(r => el('tr', {}, cols.map(([key]) => cell(tab, r, key))))
      : el('tr', {}, el('td', { colspan: String(cols.length), class: 'lite-dm-none', text: rows.length ? 'Nothing matches the search.' : 'None stored.' })))));
  if (tab === 'programs') {
    note.textContent = 'Create Time and Edit Time: when Lite sent the program (Lite’s record); RealMan’s API does not give the '
      + 'controller’s own times, so a program Lite did not send shows —. Names as RealMan’s API gives them (31 characters at most).';
  }
  // pages, as the controller's web page has them
  const go = (n) => { page = n; paintTable(); };
  const btn = (text, n, on = false, dis = false) => el('button', { type: 'button', class: `btn light lite-dm-page${on ? ' active' : ''}`, text,
    disabled: dis, 'aria-current': on ? 'page' : false, onclick: () => go(n) });
  const nums = [];
  for (let n = 1; n <= pages; n += 1) {
    if (n === 1 || n === pages || Math.abs(n - page) <= 2) nums.push(btn(String(n), n, n === page));
    else if (nums.length && nums[nums.length - 1].tagName !== 'SPAN') nums.push(el('span', { class: 'lite-dm-gap', text: '…' }));
  }
  const goto = el('input', { type: 'number', min: '1', max: String(pages), value: String(page), 'aria-label': 'Go to page',
    onchange: (e) => go(Math.round(Number(e.target.value) || 1)) });
  pager.replaceChildren(el('span', { text: `Total ${shown.length}` }),
    el('select', { 'aria-label': 'Rows per page', onchange: (e) => { size = Number(e.target.value); page = 1; paintTable(); } },
      [10, 20, 50, 100].map(n => el('option', { value: String(n), text: `${n}/page`, selected: n === size }))),
    btn('Previous', page - 1, false, page <= 1), ...nums, btn('Next', page + 1, false, page >= pages),
    el('label', { class: 'lite-dm-goto' }, 'Go to ', goto));
}

function paint() { paintTabs(); paintStatus(); paintTable(); }

// ── reading

async function read(s = side) {
  const my = ++loading;
  failed[s] = null;
  paint();
  try {
    const d = await get(`/api/robots/${s}/data${ctx.pid ? `?pid=${encodeURIComponent(ctx.pid)}` : ''}`);
    data[s] = d;
  } catch (err) {
    failed[s] = err;
  } finally {
    if (my === loading) loading = 0;
  }
  if (s === side) paint();
}

function build() {
  const close = () => { box.classList.remove('show'); document.body.style.overflow = ''; };
  box = el('div', { class: 'modal lite-viewer lite-dm', id: 'liteDataMgmt' },
    el('div', { class: 'modal-card lite-viewer-card lite-dm-card', role: 'dialog', 'aria-modal': 'true', 'aria-label': 'Data Management' },
      el('div', { class: 'section-head' },
        el('div', {}, el('div', { class: 'eyebrow', text: 'Each arm’s controller · display only' }),
          el('h3', { text: 'Data Management' }),
          el('p', { class: 'mini-note', text: 'What each arm’s controller stores, as RealMan’s own web page lists it. Read when this '
            + 'window opens, when the other arm is chosen and on Refresh, never while the arm moves. Nothing here changes a controller.' })),
        el('div', { class: 'cta-row' }, el('button', { class: 'btn light', type: 'button', text: 'Close', onclick: close }))),
      el('div', { class: 'lite-dm-bar' },
        el('div', { class: 'lite-dm-arms', role: 'group', 'aria-label': 'Arm' },
          ARMS.map(s => el('button', { id: `liteDmArm_${s}`, type: 'button', class: `btn lite-dm-arm ${s}`, text: `${s.toUpperCase()} ARM`,
            onclick: () => { side = s; page = 1; paint(); if (!data[s]) read(s); } }))),
        el('button', { class: 'btn outline', type: 'button', text: 'Refresh', onclick: () => read() }),
        el('div', { id: 'liteDmStatus', class: 'lite-status lite-grow' })),
      el('div', { class: 'lite-dm-tabsrow' },
        el('div', { class: 'lite-tabs lite-dm-tabs', role: 'tablist', 'aria-label': 'Stored data' },
          TABS.map(([id, name]) => el('button', { id: `liteDmTab_${id}`, class: 'lite-tab', type: 'button', role: 'tab', text: name,
            onclick: () => { tab = id; page = 1; paint(); } }))),
        el('input', { id: 'liteDmSearch', type: 'search', class: 'lite-dm-search', placeholder: 'Search by name', 'aria-label': 'Search by name',
          oninput: (e) => { q = e.target.value; page = 1; paintTable(); } })),
      el('div', { id: 'liteDmTable', class: 'lite-dm-tablebox', role: 'tabpanel' }),
      el('div', { id: 'liteDmNote', class: 'mini-note' }),
      el('div', { id: 'liteDmPager', class: 'lite-dm-pager' })));
  document.body.append(box);
  document.addEventListener('keydown', (e) => {
    if (e.key === 'Escape' && box.classList.contains('show') && !document.querySelector('.lite-confirm, .lite-picker-panel')) close();
  });
}

export function openDataManagement() {
  if (!box) build();
  box.classList.add('show');
  document.body.style.overflow = 'hidden';
  paint();
  read(side);
}

// The top bar's button (beside Projects; its label gives way to "Data" when the bar is narrow) and the
// Operator Journey's entry.
export function installDataManagement() {
  const projects = document.querySelector('.topbar .project-manager-btn');
  if (projects && !$('liteDataBtn')) {
    projects.after(el('button', { id: 'liteDataBtn', class: 'lite-data-btn', type: 'button', onclick: openDataManagement,
      title: 'What each arm’s controller stores: programs, global points, tracks (display only)', 'aria-label': 'Data Management' },
    el('span', { class: 'lite-label' }, 'Data', el('span', { class: 'lite-long', text: ' Management' }))));
  }
  const logs = document.querySelector('.navbtn[data-view="logs"]');
  if (logs && !document.querySelector('.lite-data-nav')) {
    logs.after(el('button', { class: 'navbtn lite-data-nav', type: 'button', onclick: openDataManagement },
      el('span', { class: 'navicon', text: '≡' }), el('span', { text: 'Data Management' })));
  }
}
