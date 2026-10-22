// Open / Manage Projects (the owner, 22 Oct: "add a feature to open and view existing projects, and to manage them,
// including initiating (creating) a project, renaming a project, any other management actions ... The Open / Manage
// project button already exists. The feature goes behind it"). The Project Manager's card of that name only scrolled
// to the library under it; it now opens this view in the Project Manager, in place of its cards and library:
// - every project the server finds in the projects folder, read again each time the view opens and on Refresh (a
//   project copied there, or brought by git, shows without logging in again), with a search and Active / Archived / All;
// - the project chosen, with what it holds, read from its folder without opening it (GET /api/projects/<id>/overview):
//   its folder, fixture, times, each task's step on each arm, the merges, the S2 sequence and its last simulation;
// - every action on it: Open (or a double click), Rename (asked here, not in the browser's prompt; its folder keeps its
//   name), Duplicate, Archive, Save Project Folder, Save .alixproj, Delete; and New Project and Load Project File.
// The library under the cards is read from the server too, when the Project Manager opens and on its Refresh.

import { get, say } from './api.js';
import { el } from './ctx.js';
import { flush, loadProjects } from './projects.js';

const $ = (id) => document.getElementById(id);
const clean = (v) => String(v || '').trim().replace(/\s+/g, ' ');
let box = null, sel = null, filter = 'active', q = '', root = '';
let fixtureNames = {};
let detail = { id: null, stamp: null, data: null, error: null }, reading = 0;

const shell = () => document.querySelector('#projectManagerScreen .project-manager-shell');
const records = () => Object.values(window.loadProjectDatabase())
  .sort((a, b) => String(b.updatedAt || '').localeCompare(String(a.updatedAt || '')));
const isOpen = (id) => typeof currentProjectId !== 'undefined' && currentProjectId === id;

// A time as this computer shows it: an ISO time (the page's records), or the server's local "YYYY-MM-DD HH:MM:SS".
function when(t) {
  if (!t) return '—';
  const s = String(t), d = new Date(/[TZ]/.test(s) ? s : s.replace(' ', 'T'));
  return Number.isNaN(d.getTime()) ? s : d.toLocaleString();
}

const size = (b) => (b == null ? '—' : b >= 1e9 ? `${(b / 1e9).toFixed(2)} GB` : `${(b / 1e6).toFixed(1)} MB`);
const fixtureOf = (p) => fixtureNames[p?.lite?.fixture] || p?.lite?.fixture || '';
const resumeOf = (p) => (window.currentResumeDescription ? window.currentResumeDescription(p.state) : '');
const doneOf = (p) => Object.values((p.state && p.state.completed) || {}).filter(Boolean).length;

// The projects folder read again (after the page's own saves have reached the server), then every list drawn.
export async function refreshProjects() {
  try {
    await flush();
    const d = await loadProjects();
    root = d.folder || root;
  } catch (err) {
    window.toast(`Could not read the projects folder: ${say(err)}`, 'error');
  }
  window.renderProjectManager();
}

async function readFixtures() {
  try {
    const d = await get('/api/fixtures');
    fixtureNames = Object.fromEntries((d.fixtures || []).map((f) => [f.id, f.name]));
  } catch (_) { /* the ids are shown instead */ }
}

function build() {
  if (box) return;
  const search = el('input', { id: 'litePmSearch', type: 'search', placeholder: 'Search by name, folder or fixture',
    'aria-label': 'Search the projects', oninput: (e) => { q = e.target.value; paint(); } });
  const seg = el('div', { class: 'lite-pm-seg', role: 'group', 'aria-label': 'Which projects' },
    [['active', 'Active'], ['archived', 'Archived'], ['all', 'All']].map(([v, t]) => el('button', { type: 'button', 'data-filter': v, text: t,
      onclick: () => { filter = v; paint(); } })));
  box = el('section', { id: 'liteManage', class: 'lite-pm', hidden: true, 'aria-label': 'Open / Manage Projects' },
    el('div', { class: 'lite-pm-bar' },
      el('div', { class: 'lite-pm-title' },
        el('button', { class: 'btn light', type: 'button', text: 'Back', onclick: close, title: 'Back to the Project Manager' }),
        el('div', {}, el('h3', { text: 'Open / Manage Projects' }), el('p', { id: 'litePmCount', class: 'mini-note' }))),
      el('div', { class: 'lite-pm-tools' }, search, seg,
        el('button', { class: 'btn light', type: 'button', text: 'Refresh', title: 'Read the projects folder again', onclick: refreshProjects }),
        el('button', { class: 'btn', type: 'button', text: 'New Project', onclick: newProject }),
        el('button', { class: 'btn light', type: 'button', text: 'Load Project File', title: 'Import a saved .alixproj project file',
          onclick: () => $('projectImportInput')?.click() }))),
    el('div', { class: 'lite-pm-body' },
      el('div', { id: 'litePmList', class: 'lite-pm-list', role: 'listbox', 'aria-label': 'Projects' }),
      el('div', { id: 'litePmDetail', class: 'lite-pm-detail', 'aria-live': 'polite' })));
  const head = shell()?.querySelector('.project-manager-head');
  if (head) head.after(box); else shell()?.append(box);
}

function open() {
  build();
  shell()?.classList.add('lite-pm-on');
  box.hidden = false;
  if (!Object.keys(fixtureNames).length) readFixtures().then(paint);
  paint();
  refreshProjects();
  setTimeout(() => $('litePmSearch')?.focus(), 50);
}

function close() {
  shell()?.classList.remove('lite-pm-on');
  if (box) box.hidden = true;
}

function newProject() {
  close();
  window.showNewProjectPanel();
  $('newProjectPanel')?.scrollIntoView({ behavior: 'smooth', block: 'center' });
}

function shown() {
  const words = q.toLowerCase().split(/\s+/).filter(Boolean);
  return records().filter((p) => (filter === 'all' || (filter === 'archived') === !!p.archived)
    && words.every((w) => `${p.name} ${p.lite?.folder || ''} ${fixtureOf(p)}`.toLowerCase().includes(w)));
}

function paint() {
  if (!box || box.hidden) return;
  const all = records(), list = shown();
  const active = all.filter((p) => !p.archived).length;
  $('litePmCount').textContent = `${active} active, ${all.length - active} archived${root ? ` · ${root}` : ''}`;
  for (const b of box.querySelectorAll('[data-filter]')) {
    b.classList.toggle('active', b.dataset.filter === filter);
    b.setAttribute('aria-pressed', String(b.dataset.filter === filter));
  }
  if (!list.some((p) => p.id === sel)) sel = list[0]?.id || null;
  const total = typeof TASKS !== 'undefined' ? TASKS.length : 0;
  $('litePmList').replaceChildren(...(list.length ? list.map((p) => el('button', {
    type: 'button', role: 'option', 'aria-selected': String(p.id === sel), 'data-id': p.id,
    class: `lite-pm-row${p.id === sel ? ' sel' : ''}${p.archived ? ' archived' : ''}`,
    onclick: () => { sel = p.id; paint(); }, ondblclick: () => openIt(p.id),
  },
  el('span', { class: 'lite-pm-name' }, el('b', { text: p.name }),
    isOpen(p.id) ? el('span', { class: 'badge blue', text: 'OPEN' }) : null,
    p.archived ? el('span', { class: 'badge', text: 'ARCHIVED' }) : null),
  el('span', { class: 'lite-pm-meta', text: [fixtureOf(p), `updated ${when(p.updatedAt)}`].filter(Boolean).join(' · ') }),
  el('span', { class: 'lite-pm-meta', text: `${doneOf(p)}${total ? ` / ${total}` : ''} tasks done${resumeOf(p) ? ` · resume: ${resumeOf(p)}` : ''}` })))
    : [el('div', { class: 'notice', text: all.length ? 'No project matches.' : 'No projects yet: create one with New Project.' })]));
  paintDetail();
}

async function readDetail(id, stamp) {
  const mine = ++reading;
  detail = { id, stamp, data: null, error: null };
  try {
    const d = await get(`/api/projects/${id}/overview`);
    if (mine === reading) detail = { id, stamp, data: d, error: null };
  } catch (err) {
    if (mine === reading) detail = { id, stamp, data: null, error: say(err) };
  }
  if (mine === reading) paintDetail();
}

const fact = (k, v) => [el('dt', { text: k }), el('dd', {}, v)];

function armCell(u) {
  if (!u) return el('td', { class: 'sub', text: '—' });
  const what = `${u.code}${u.part ? ` part ${u.part}` : ''} ${u.name || ''}`.trim();
  const how = [u.status_label, u.recordings != null ? `${u.recordings} recording${u.recordings === 1 ? '' : 's'}` : null,
    u.version ? `kept ${u.version}` : null].filter(Boolean).join(' · ');
  return el('td', {}, el('div', { text: what }), how ? el('div', { class: 'sub', text: how }) : null);
}

function paintDetail() {
  const pane = $('litePmDetail');
  if (!pane) return;
  const p = sel ? window.loadProjectDatabase()[sel] : null;
  if (!p) { pane.replaceChildren(el('div', { class: 'mini-note', text: 'Choose a project to see what it holds.' })); return; }
  if (detail.id !== p.id || detail.stamp !== p.updatedAt) readDetail(p.id, p.updatedAt);
  const d = detail.id === p.id ? detail.data : null;
  const busy = typeof currentProjectId !== 'undefined' && currentProjectId === p.id;
  const actions = el('div', { class: 'lite-pm-actions' },
    el('button', { class: 'btn green', type: 'button', text: isOpen(p.id) ? 'Open (it is open)' : 'Open', onclick: () => openIt(p.id) }),
    el('button', { class: 'btn light', type: 'button', text: 'Rename', onclick: () => renameDialog(p.id) }),
    el('button', { class: 'btn light', type: 'button', text: 'Duplicate', onclick: () => window.duplicateProject(p.id) }),
    el('button', { class: 'btn outline', type: 'button', text: p.archived ? 'Unarchive' : 'Archive', disabled: busy && !p.archived,
      title: busy && !p.archived ? 'Open another project before archiving this one' : '', onclick: () => window.toggleArchiveProject(p.id) }),
    el('button', { class: 'btn folder-export-btn', type: 'button', text: 'Save Project Folder', onclick: () => window.saveProjectFolder(p.id) }),
    el('button', { class: 'btn export-btn', type: 'button', text: 'Save .alixproj', onclick: () => window.exportProjectFile(p.id) }),
    el('button', { class: 'btn red', type: 'button', text: 'Delete', onclick: () => window.deleteProject(p.id) }));
  const facts = el('dl', { class: 'lite-pm-facts' },
    fact('Folder', d ? d.folder : (p.lite?.folder || '—')),
    fact('Fixture', d?.fixture_name || fixtureOf(p) || '—'),
    fact('Created', when(p.createdAt)),
    fact('Last saved', `${when(p.updatedAt)}${p.lastSaveReason ? ` (${p.lastSaveReason})` : ''}`),
    fact('Resume at', resumeOf(p) || '—'),
    d ? fact('Files', `${d.files.count} files, ${size(d.files.bytes)}`) : null);
  const parts = [el('div', { class: 'lite-pm-dhead' }, el('h3', { text: p.name }),
    isOpen(p.id) ? el('span', { class: 'badge blue', text: 'OPEN' }) : null, p.archived ? el('span', { class: 'badge', text: 'ARCHIVED' }) : null),
  actions, facts];
  if (detail.id === p.id && detail.error) parts.push(el('div', { class: 'lite-status err', text: `What it holds could not be read: ${detail.error}` }));
  else if (!d) parts.push(el('div', { class: 'mini-note', text: 'Reading what it holds…' }));
  else {
    const sim = d.simulation;
    parts.push(el('dl', { class: 'lite-pm-facts' },
      fact('S2 sequence', d.sequence.left || d.sequence.right
        ? `LEFT ${d.sequence.left} rows, RIGHT ${d.sequence.right} rows${d.sequence.updated ? ` · changed ${when(d.sequence.updated)}` : ''}` : 'none yet'),
      fact('Last simulation', !sim ? 'none' : sim.completed
        ? `played to its end ${when(sim.completed)}${sim.total_s != null ? ` · ${Number(sim.total_s).toFixed(1)} s of motion` : ''}`
        : `made ${when(sim.made)}, not played to its end`),
      fact('Merges (S1)', `LEFT ${d.merges.left}, RIGHT ${d.merges.right}`)));
    const done = new Set(d.done || []);
    parts.push(el('table', { class: 'lite-pm-tasks' },
      el('thead', {}, el('tr', {}, el('th', { text: 'Task' }), el('th', { text: 'LEFT arm' }), el('th', { text: 'RIGHT arm' }), el('th', { text: 'Done' }))),
      el('tbody', {}, d.tasks.map((t, i) => el('tr', {},
        el('td', { text: t.label || `Task ${i + 1}` }), armCell(t.left), armCell(t.right),
        el('td', { text: done.has(i) ? 'Done' : '' }))))));
  }
  pane.replaceChildren(...parts);
}

async function openIt(id) {
  await window.openProjectById(id);
  if (typeof currentProjectId !== 'undefined' && currentProjectId === id) close();
}

// Rename, asked in the page (the template asked in the browser's prompt): the project's name only; its folder keeps
// the name it was created with (Resources/Projects/README.md). A name another project has is refused.
function renameDialog(id) {
  const p = window.loadProjectDatabase()[id];
  if (!p) return;
  const input = el('input', { type: 'text', maxlength: '120', value: p.name, 'aria-label': 'Project name' });
  const msg = el('div', { class: 'project-name-message', text: 'Its folder keeps its name; only the project is renamed.' });
  const cancel = el('button', { class: 'btn outline', type: 'button', text: 'Cancel' });
  const ok = el('button', { class: 'btn green', type: 'button', text: 'Rename' });
  const card = el('div', { class: 'modal-card accessible-modal', role: 'dialog', 'aria-modal': 'true', 'aria-label': 'Rename project' },
    el('h3', { text: 'Rename project' }),
    el('p', { text: `“${p.name}”, in the folder ${p.lite?.folder || p.name}.` }),
    el('div', { class: 'field' }, el('label', { text: 'New name' }), input), msg,
    el('div', { class: 'modal-actions' }, cancel, ok));
  const dlg = el('div', { class: 'modal show lite-confirm' }, card);
  const check = () => {
    const name = clean(input.value);
    const other = Object.values(window.loadProjectDatabase()).find((r) => r.id !== id && clean(r.name).toLocaleLowerCase() === name.toLocaleLowerCase());
    const why = !name ? 'A project needs a name.' : other ? `Another project is already named “${other.name}”.` : '';
    msg.textContent = why || 'Its folder keeps its name; only the project is renamed.';
    msg.className = `project-name-message${why ? ' error' : ''}`;
    ok.disabled = !!why || name === p.name;
    return why ? null : name;
  };
  const done = () => { dlg.remove(); document.removeEventListener('keydown', key, true); };
  const apply = () => {
    const name = check();
    if (!name || name === p.name) return;
    const db = window.loadProjectDatabase(), rec = db[id];
    if (!rec) { done(); return; }
    rec.name = name;
    rec.updatedAt = window.projectNow();
    db[id] = rec;
    window.saveProjectDatabase(db);
    window.renderProjectManager();
    window.updateCurrentProjectHeader();
    if (isOpen(id)) window.saveCurrentProjectNow('Project renamed');
    window.toast(`Project renamed: ${name}`, 'good');
    done();
  };
  const key = (e) => {
    if (e.key === 'Escape') { e.stopImmediatePropagation(); done(); }
    if (e.key === 'Enter' && document.activeElement === input) { e.preventDefault(); apply(); }
  };
  input.addEventListener('input', check);
  cancel.onclick = done;
  ok.onclick = apply;
  document.addEventListener('keydown', key, true);
  document.body.append(dlg);
  check();
  input.focus();
  input.select();
}

export function installManage() {
  window.focusProjectLibrary = open;                 // the card "Open / Manage Projects"
  window.renameProject = renameDialog;               // the library's Rename too

  const pageShow = window.showProjectManager;
  window.showProjectManager = function showProjectManager() {
    close();                                          // the Project Manager opens on its cards
    pageShow();
    refreshProjects();
  };
  const pageClose = window.closeProjectManager;
  window.closeProjectManager = function closeProjectManager(force = false) {
    pageClose(force);
    if (!$('projectManagerScreen')?.classList.contains('show')) close();
  };
  const pageRender = window.renderProjectManager;
  window.renderProjectManager = function renderProjectManager() {
    pageRender();
    paint();
  };
  // the library's Refresh reads the projects folder again (it redrew the copy read at log-in)
  const refresh = document.querySelector('#projectLibrary .project-library-actions button[onclick^="renderProjectManager"]');
  if (refresh) {
    refresh.removeAttribute('onclick');
    refresh.title = 'Read the projects folder again';
    refresh.addEventListener('click', () => refreshProjects());
  }
  document.addEventListener('lite:logged-in', () => readFixtures());
}
