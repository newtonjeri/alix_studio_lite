// What the region loop shares: the open project, its task list, the arm step on screen, jobs,
// the confirmation dialog, and robot calls that ask before a real move.
//
// The page's own globals are read directly: TASKS, currentTask, selectedArm, currentStage.

import { get, post, say } from './api.js';

export const ctx = { pid: null, fixture: null, fixtureName: '', hasDeck: true, units: {} };

// The arm step on screen (server view of the task's LEFT or RIGHT unit), or null.
export function step(i = currentTask, arm = selectedArm) {
  const t = TASKS[i];
  return t && t.lite ? t.lite[arm] : null;
}

export const unitUrl = (key, verb = '') => `/api/projects/${ctx.pid}/units/${key}${verb ? '/' + verb : ''}`;

export async function loadUnit(key) {
  const u = await get(unitUrl(key));
  ctx.units[key] = u;
  return u;
}

export const el = (tag, attrs = {}, ...kids) => {
  const e = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (k === 'class') e.className = v;
    else if (k === 'text') e.textContent = v;
    else if (k.startsWith('on')) e.addEventListener(k.slice(2), v);
    else if (v !== false && v != null) e.setAttribute(k, v === true ? '' : v);
  }
  for (const c of kids.flat()) if (c != null) e.append(c instanceof Node ? c : document.createTextNode(String(c)));
  return e;
};

export const fmtS = (s) => (s == null ? '—' : `${Number(s).toFixed(2)} s`);

// Polls a job until it ends; onProgress(view) on every poll. Resolves to the final view.
export async function watchJob(job, onProgress = null) {
  let j = job;
  while (j.state === 'running') {
    if (onProgress) onProgress(j);
    await new Promise(r => setTimeout(r, 300));
    j = await get(`/api/jobs/${j.id}`);
  }
  if (onProgress) onProgress(j);
  return j;
}

export const jobLine = (j) => (j.n ? `${j.stage} (${Math.round(100 * j.i / Math.max(1, j.n))} %)` : j.stage || j.title);

// A confirmation in the page's modal style. Cancel has the focus, so Enter never moves a robot.
export function confirmDialog({ title, body, ok = 'Continue', danger = true, list = [] }) {
  return new Promise((resolve) => {
    const cancel = el('button', { class: 'btn outline', text: 'Cancel' });
    const yes = el('button', { class: danger ? 'btn red' : 'btn green', text: ok });
    const card = el('div', { class: 'modal-card accessible-modal', role: 'alertdialog', 'aria-modal': 'true' },
      el('h3', { text: title }), el('p', { text: body }),
      list.length ? el('ul', { class: 'lite-confirm-list' }, list.map(x => el('li', { text: x }))) : null,
      el('div', { class: 'modal-actions' }, cancel, yes));
    const box = el('div', { class: 'modal show lite-confirm' }, card);
    const done = (v) => { box.remove(); document.removeEventListener('keydown', key, true); resolve(v); };
    const key = (e) => { if (e.key === 'Escape') { e.stopPropagation(); done(false); } };
    cancel.onclick = () => done(false);
    yes.onclick = () => done(true);
    document.addEventListener('keydown', key, true);
    document.body.append(box);
    cancel.focus();
  });
}

const REAL_CHECKS = ['The work area around the arm is clear of people.', 'The red E-stop buttons are within reach.'];

// A robot request. The server answers code "confirm" (Real needs a yes), "replace" (a program
// slot holds another file) or "mode" (Visual sim uses no robot); this asks and retries once.
export async function robotCall(url, body = {}, { what = 'This move' } = {}) {
  try {
    return await post(url, body);
  } catch (err) {
    if (err.code === 'confirm') {
      const yes = await confirmDialog({ title: 'Real robot: the arm will move', body: `${what} on the real robot. ${err.message}`,
        ok: 'Yes, move the robot', list: REAL_CHECKS });
      if (!yes) throw Object.assign(new Error('Cancelled: nothing was sent to the robot.'), { code: 'cancelled' });
      return robotCall(url, { ...body, confirmed: true }, { what });
    }
    if (err.code === 'replace') {
      const yes = await confirmDialog({ title: 'Replace the program?', body: `${err.message} ${err.hint || ''}`.trim(),
        ok: 'Replace it', danger: true });
      if (!yes) throw Object.assign(new Error('Cancelled: the program on the robot was kept.'), { code: 'cancelled' });
      return robotCall(url, { ...body, replace: true }, { what });
    }
    throw err;
  }
}

export function tell(err) {
  if (err && err.code === 'cancelled') { window.toast(err.message); return; }
  window.toast(say(err));
  if (err && (err.code === 'mode' || err.code === 'connect')) document.dispatchEvent(new CustomEvent('lite:open-robot-panel'));
}
