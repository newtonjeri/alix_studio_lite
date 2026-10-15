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

// Seconds to the hundredth, rounded half up (48.175 s is 48.18 s; toFixed alone gives 48.17).
export const fmtS = (s) => (s == null ? '—' : `${(Math.round(Number(s) * 100 + 1e-6) / 100).toFixed(2)} s`);

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

export const REAL_CHECKS = ['The work area around the arm is clear of people.', 'The red E-stop buttons are within reach.'];

// A robot request. The server answers code "confirm" (Real needs a yes), "replace" (a program
// slot holds another file) or "mode" (Visual sim uses no robot); this asks and retries once.
export async function robotCall(url, body = {}, { what = 'This move', replacing = 'the program' } = {}) {
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
      const yes = await confirmDialog({ title: `Replace ${replacing}?`, body: `${err.message} ${err.hint || ''}`.trim(),
        ok: 'Replace it', danger: true });
      if (!yes) throw Object.assign(new Error(`Cancelled: ${replacing} on the robot was kept.`), { code: 'cancelled' });
      return robotCall(url, { ...body, replace: true }, { what, replacing });
    }
    if (err.code === 'limits') {               // manual tuning's limits: shown, never enforced (the owner, 15 Oct)
      const yes = await confirmDialog({ title: 'This file breaks a limit', body: `${err.message} Send it anyway?`,
        ok: 'Send it anyway', danger: true });
      if (!yes) throw Object.assign(new Error('Cancelled: nothing was sent to the robot.'), { code: 'cancelled' });
      return robotCall(url, { ...body, limits_seen: true }, { what, replacing });
    }
    throw err;
  }
}

export function tell(err) {
  if (err && err.code === 'cancelled') { window.toast(err.message); return; }
  window.toast(say(err));
  if (err && (err.code === 'mode' || err.code === 'connect')) document.dispatchEvent(new CustomEvent('lite:open-robot-panel'));
}

const HAND_CHOICES = [
  ['GRIP', 'Grip (0, 0, 0, 0, 150, 1000)'],
  ['OPEN', 'All open (1000 × 6)'],
  ['CLEANING', 'Cleaning pose (1000, 1000, 1000, 1000, 0, 1000)'],
  ['100% open', 'Fingers 100 % open, thumb held'],
  ['50% open', 'Fingers 50 % open, thumb held'],
];

// Asks what a file was taught with: its arm (when not known) and its stage, the pole height and
// hand pose. Prefilled with `setup` ({pole_mm, hand}): a hand that is none of the presets is offered
// with its exact values; with no setup, nothing is preselected. Resolves {arm, pole_mm, hand} or null.
export function stageDialog({ title, body, arm = null, setup = null, askArm = false }) {
  return new Promise((resolve) => {
    const armSel = el('select', { 'aria-label': 'Arm' },
      el('option', { value: '', text: 'Choose…' }), el('option', { value: 'left', text: 'LEFT arm' }),
      el('option', { value: 'right', text: 'RIGHT arm' }));
    if (arm) armSel.value = arm;
    const pole = el('input', { type: 'number', min: '0', max: '315', step: '1', value: setup ? setup.pole_mm : '' });
    const preset = (h) => {
      const k = JSON.stringify(h);
      if (k === JSON.stringify([0, 0, 0, 0, 150, 1000])) return 'GRIP';
      if (k === JSON.stringify([1000, 1000, 1000, 1000, 0, 1000])) return 'CLEANING';
      if (k === JSON.stringify([1000, 1000, 1000, 1000, 1000, 1000])) return 'OPEN';
      if (k === JSON.stringify([1000, 1000, 1000, 1000, null, null])) return '100% open';
      if (k === JSON.stringify([500, 500, 500, 500, null, null])) return '50% open';
      return null;
    };
    const exact = setup && Array.isArray(setup.hand) && !preset(setup.hand) ? setup.hand : null;
    const hand = el('select', { 'aria-label': 'Hand' }, el('option', { value: '', text: 'Choose…' }),
      exact ? el('option', { value: 'SETUP', text: `As set up (${exact.map(c => (c == null ? 'held' : c)).join(', ')})` }) : null,
      HAND_CHOICES.map(([v, n]) => el('option', { value: v, text: n })));
    hand.value = exact ? 'SETUP' : (setup && preset(setup.hand)) || '';
    const cancel = el('button', { class: 'btn outline', text: 'Cancel' });
    const ok = el('button', { class: 'btn green', text: 'Use these values' });
    const msg = el('div', { class: 'mini-note' });
    const card = el('div', { class: 'modal-card accessible-modal', role: 'dialog', 'aria-modal': 'true' },
      el('h3', { text: title }), el('p', { text: body }),
      el('div', { class: 'lite-stage-form' },
        askArm ? el('div', { class: 'field' }, el('label', { text: 'Taught on' }), armSel) : null,
        el('div', { class: 'field' }, el('label', { text: 'Pole height it was taught at (mm, 0 to 315)' }), pole),
        el('div', { class: 'field' }, el('label', { text: 'Hand pose it was taught with' }), hand)),
      msg, el('div', { class: 'modal-actions' }, cancel, ok));
    const box = el('div', { class: 'modal show lite-confirm' }, card);
    const done = (v) => { box.remove(); resolve(v); };
    cancel.onclick = () => done(null);
    ok.onclick = () => {
      const p = Number(pole.value);
      if (askArm && !armSel.value) { msg.textContent = 'Choose the arm it was taught on.'; return; }
      if (pole.value === '' || !(p >= 0 && p <= 315)) { msg.textContent = 'The pole height is 0 to 315 mm.'; return; }
      if (!hand.value) { msg.textContent = 'Choose the hand pose it was taught with.'; return; }
      done({ arm: askArm ? armSel.value : arm, pole_mm: p, hand: hand.value === 'SETUP' ? exact : hand.value });
    };
    document.body.append(box);
    pole.focus();
  });
}

// LEFT / RIGHT from a file's name or folder, as the server reads it (R15_... is a region, not RIGHT).
export function armFromName(name) {
  const parts = String(name || '').split(/[\\/]/).filter(Boolean).reverse();
  for (const p of parts) {
    const u = p.toUpperCase();
    if (/^(L[_-]|LEFT\b|LEFT[_-])/.test(u) || u === 'LEFT' || u === 'L') return 'left';
    if (/^(R[_-]|RIGHT\b|RIGHT[_-])/.test(u) || u === 'RIGHT' || u === 'R') return 'right';
  }
  for (const p of parts) { const u = p.toUpperCase(); if (u.includes('LEFT')) return 'left'; if (u.includes('RIGHT')) return 'right'; }
  return null;
}
