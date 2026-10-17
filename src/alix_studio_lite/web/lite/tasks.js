// The task queue (decision D35) and the fixture (D29).
//
// The page's 22 tasks stay as designed (order, pictures, kinds); each arm step is tied to its unit
// of the project's region catalogue: its code and name (TASK 9 as the deck has it), its status,
// and whether it is in Release 1. Arm steps outside Release 1 show as LATER and count as done, so
// the other arm's PASS completes the task. A project of another fixture gets that fixture's task
// list. PASS is recorded on the server, which accepts it only after a completed test run.

import { get, post, say } from './api.js';
import { ctx, el, step, tell, unitUrl } from './ctx.js';
import { flush, setNextFixture } from './projects.js';

const ORIGINAL = JSON.parse(JSON.stringify(TASKS));       // the designer's list, restored per project
let quiet = false;
let fixtures = null;

const SOURCE = { library: 'existing recording', upload: 'loaded file', teach: 'taught', trim: 'trimmed' };

export function applyTasks(d) {
  Object.assign(ctx, { fixture: d.fixture, fixtureName: d.fixture_name || d.fixture, hasDeck: d.has_deck });
  if (d.has_deck) {
    const byLabel = new Map(d.tasks.map(t => [t.label, t]));
    TASKS.splice(0, TASKS.length, ...ORIGINAL.map(t => ({ ...t })));
    for (const t of TASKS) {
      const s = byLabel.get(t.label);
      t.lite = s ? { left: s.left, right: s.right } : { left: null, right: null };
      if (!s) continue;
      t.left = s.left ? s.left.display : '—';
      t.right = s.right ? s.right.display : '—';
      if (s.note && t.kind !== 'procedure') t.note = s.note;
    }
  } else {
    TASKS.splice(0, TASKS.length, ...d.tasks.map(s => ({
      label: s.label, slide: null, kind: s.kind, note: s.note,
      left: s.left ? s.left.display : '—', right: s.right ? s.right.display : '—',
      lite: { left: s.left, right: s.right },
    })));
  }
}

export async function refreshTasks() {
  if (!ctx.pid) return;
  applyTasks(await get(`/api/projects/${ctx.pid}/tasks`));
  window.renderTaskList();
  relabel();
}

const later = (t, arm) => !t || !t.lite || !t.lite[arm] || t.lite[arm].later;
const bothLater = (t) => later(t, 'left') && later(t, 'right');
// A lid or seat task (mechanism.py): ALIX's plan, pre-recorded, not taught (the owner, 18 Oct: "not subjected to
// teaching procedure ... tasks that are available to be injected at a point in the sequence").
const preRecorded = (t) => (t && t.lite && ((t.lite.left || {}).mechanism || (t.lite.right || {}).mechanism)) || null;

// The Teach page of a pre-recorded task says what it is and where it goes, instead of the teaching it does not need.
function mechanismNotice() {
  const t = TASKS[currentTask], m = preRecorded(t);
  let box = document.getElementById('liteMechNotice');
  const anchor = document.getElementById('procedureNotice');
  if (!box && anchor && m) {
    box = el('div', { id: 'liteMechNotice', class: 'notice lite-mech-notice' });
    anchor.after(box);
  }
  document.getElementById('view-teach')?.classList.toggle('lite-prerecorded', !!m);   // no teaching stages: it is not taught
  if (!box) return;
  box.hidden = !m;
  if (!m) return;
  const next = TASKS.findIndex((x, j) => j > currentTask && !completed[j] && !preRecorded(x));
  box.replaceChildren(el('b', { text: `Pre-recorded: ${m.title} (${m.short}).` }),
    ` ALIX's plan for both arms, with the wrist camera's checks: it is not taught. Put it in the sequence in Merge + Sequence, S2, `
    + 'its LID / SEAT tab (added to both arms\' lists at once); it runs there in Simulation and on the robot.',
    el('button', { class: 'btn light', type: 'button', text: 'Open S2',
      onclick: () => { window.showView('sequence'); setTimeout(() => document.getElementById('liteSeqTab_s2')?.click(), 300); } }),
    next >= 0 ? el('button', { class: 'btn', type: 'button', text: `Go on to ${TASKS[next].label}`, onclick: () => window.jumpTask(next) }) : '');
}

// The arm cards: LATER / NOT USED, and each step's status under its name.
function relabel() {
  const t = TASKS[currentTask];
  if (!t || !t.lite) return;
  for (const arm of ['left', 'right']) {
    const card = document.getElementById(arm === 'left' ? 'choiceLeft' : 'choiceRight');
    const state = document.getElementById(`${arm}State`);
    const name = document.getElementById(`${arm}TaskName`);
    const s = t.lite[arm];
    const isLater = later(t, arm) && !bothLater(t);
    card?.classList.toggle('lite-later', isLater);
    if (isLater && state) state.textContent = s ? 'LATER · NOT IN RELEASE 1' : 'NOT USED';
    let line = document.getElementById(`lite${arm}Status`);
    if (!line && name) { line = el('div', { id: `lite${arm}Status`, class: 'lite-step-status' }); name.after(line); }
    if (line) {
      line.textContent = !s ? '' : s.later ? 'Not in Release 1: added later.'
        : s.mechanism ? `Pre-recorded: ALIX's ${s.mechanism.title} plan, run in S2 (not taught).`
          : [s.status_label, s.recordings ? `${s.recordings} recording${s.recordings > 1 ? 's' : ''}` : 'no recording yet',
            s.version ? `tuned ${s.version}` : ''].filter(Boolean).join(' · ');
    }
    if (s && s.mechanism && state && !isLater) state.textContent = 'PRE-RECORDED';
  }
  mechanismNotice();
}

function markLater() {
  const t = TASKS[currentTask];
  if (!t || !t.lite || bothLater(t)) { relabel(); return; }
  for (const arm of ['left', 'right']) if (later(t, arm)) armDone[arm] = true;
  if (later(t, selectedArm)) window.selectArm(selectedArm === 'left' ? 'right' : 'left');
  relabel();
}

async function fixtureChoices() {
  if (!fixtures) fixtures = (await get('/api/fixtures')).fixtures;
  return fixtures;
}

async function addFixtureField() {
  const row = document.querySelector('#newProjectPanel .new-project-row');
  if (!row || document.getElementById('liteNewProjectFixture')) return;
  const sel = el('select', { id: 'liteNewProjectFixture', 'aria-label': 'Fixture' });
  row.querySelector('.field')?.after(el('div', { class: 'field lite-fixture-field' }, el('label', { text: 'Fixture' }), sel));
  try {
    for (const f of await fixtureChoices()) {
      sel.append(el('option', { value: f.id, text: f.has_deck ? f.name : `${f.name} (provisional regions)`, selected: f.default }));
    }
  } catch (err) { tell(err); }
}

export function installTasks() {
  // the queue and the Summary say PRE-RECORDED for a lid or seat task (an approval or the current task first)
  const pageQueueStatus = window.taskQueueStatus;
  window.taskQueueStatus = function taskQueueStatus(i, t) {
    const own = pageQueueStatus(i, t);
    return own.cls === 'pending' && preRecorded(t) ? { label: 'PRE-RECORDED', cls: 'lite-prerecorded' } : own;
  };
  const pageArmStatus = window.armTaskStatusInfo;
  window.armTaskStatusInfo = function armTaskStatusInfo(i, side) {
    const own = pageArmStatus(i, side), s = TASKS[i] && TASKS[i].lite && TASKS[i].lite[side];
    return own.label === 'PENDING' && s && s.mechanism ? { label: 'PRE-RECORDED', cls: 'violet' } : own;
  };

  // the Summary's count: the pre-recorded tasks are not taught, so they are said apart
  const pageSummary = window.renderSummary;
  window.renderSummary = function renderSummary() {
    pageSummary();
    const pre = TASKS.filter((t, i) => preRecorded(t) && !completed[i]).length;
    const badge = document.getElementById('projectBadge');
    if (pre && badge) {
      const done = Object.keys(completed).filter(k => completed[k]).length;
      badge.textContent = `${done} / ${TASKS.length - pre} tasks complete · ${pre} pre-recorded`;
    }
  };

  const pageLoadTask = window.loadTask;
  window.loadTask = function loadTask(i) {
    quiet = true;
    try { pageLoadTask(i); } finally { quiet = false; }
    markLater();
    document.dispatchEvent(new CustomEvent('lite:step'));
  };

  const pageSelectArm = window.selectArm;
  window.selectArm = function selectArm(a) {
    const t = TASKS[currentTask];
    if (t && t.lite && later(t, a) && !bothLater(t)) {
      if (!quiet) window.toast(t.lite[a] ? `${t.lite[a].display} is not in Release 1: it will be added later.` : `This task does not use the ${a.toUpperCase()} arm.`);
      return;
    }
    pageSelectArm(a);
    relabel();
    if (!quiet) document.dispatchEvent(new CustomEvent('lite:step'));
  };

  const pageDecision = window.decision;
  window.decision = async function decision(d) {
    const s = step();
    if (s && !s.later && ctx.pid) {
      try { await post(unitUrl(s.key, 'verdict'), { verdict: d }); }
      catch (err) { tell(err); return; }
    }
    pageDecision(d);
    refreshTasks().catch(() => {});
  };

  // Opens a project once the server has its task list (the newest Open wins). The project being
  // left is saved first, with its own task list, which this one's then replaces.
  const pageOpen = window.openProjectById;
  let opening = 0;
  window.openProjectById = async function openProjectById(id) {
    const mine = ++opening;
    let d;
    try { d = await get(`/api/projects/${id}/tasks`); } catch (err) { tell(err); return; }
    if (mine !== opening) return;
    if (currentProjectId && currentProjectId !== id) {
      window.saveCurrentProjectNow('Switched project');
      currentProjectId = null;                          // saved: the page does not save it again
    }
    ctx.pid = id;
    applyTasks(d);
    pageOpen(id);
    if (currentProjectId !== id) { ctx.pid = null; return; }       // the page did not open it
    relabel();
    document.dispatchEvent(new CustomEvent('lite:step'));
  };

  const pageCreate = window.createProject;
  window.createProject = async function createProject(name) {
    const fixture = document.getElementById('liteNewProjectFixture')?.value || 'commode_c';
    try { applyTasks(await get(`/api/fixtures/${fixture}/tasks`)); } catch (err) { tell(err); return null; }
    setNextFixture(fixture);
    const id = pageCreate(name);
    ctx.pid = id;
    await flush();                                        // the project now exists on the server
    try { await refreshTasks(); } catch (err) { window.toast(say(err)); }
    document.dispatchEvent(new CustomEvent('lite:step'));
    return id;
  };

  const pageShowNew = window.showNewProjectPanel;
  window.showNewProjectPanel = function showNewProjectPanel() {
    pageShowNew();
    addFixtureField();
  };

  // the fixture of each project in the Project Library
  const pageRender = window.renderProjectManager;
  window.renderProjectManager = function renderProjectManager() {
    pageRender();
    const db = window.loadProjectDatabase();
    document.querySelectorAll('#projectList .project-row').forEach((row) => {
      const id = (row.querySelector('button[onclick^="openProjectById"]')?.getAttribute('onclick') || '').match(/'([^']+)'/)?.[1];
      const fx = db[id]?.lite?.fixture;
      const name = (fixtures || []).find(f => f.id === fx)?.name || fx;
      const where = row.querySelector('.project-row-name > span');
      if (fx && where && !row.querySelector('.lite-fixture-badge')) where.before(el('span', { class: 'badge lite-fixture-badge', text: name }));
    });
  };
  document.addEventListener('lite:logged-in', () => fixtureChoices().catch(() => {}));
}

export { SOURCE };
