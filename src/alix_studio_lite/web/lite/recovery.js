// Arm Drag Mode and Normal Mode (feedback v5, slide 8; the owner, 16 Oct), beside STOP BOTH: an arm made
// free to be moved by hand, one or both, for recovering it after a collision or an unexpected stop,
// never for recording (the controller's drag with recording off). While an arm is in it, the whole page
// is locked: only Normal Mode and STOP BOTH can be pressed. Normal Mode leaves it, and the server checks
// the arm is safe (at rest, no program, no error latched, every joint enabled) before the page is let
// go; until then the lock stays and says why, with Clear errors when an error is what holds it. The
// lock follows the server's state, so a reload keeps it.

import { get, post } from './api.js';
import { el, robotCall, tell } from './ctx.js';

const $ = (id) => document.getElementById(id);
const ARMS = ['left', 'right'];
let last = null;                       // the latest /api/robots/status
let menu = null;
let busy = false;

function buttons() {
  const stop = document.querySelector('.topbar .emstop');
  if (!stop || $('liteDragBtn')) return;
  const drag = el('button', { id: 'liteDragBtn', class: 'lite-drag-btn', type: 'button', title: 'Free an arm to move it by hand, for recovery (nothing is recorded)',
    onclick: (e) => { e.stopPropagation(); toggleMenu(); } }, el('span', { class: 'lite-long', text: 'Arm ' }), 'Drag Mode');
  const normal = el('button', { id: 'liteNormalBtn', class: 'lite-normal-btn', type: 'button', disabled: true,
    title: 'Leave Arm Drag Mode: the arm is checked before the page is unlocked', onclick: normalMode }, 'Normal', el('span', { class: 'lite-long', text: ' Mode' }));
  stop.before(drag, normal);
  document.addEventListener('click', (e) => { if (menu && !menu.contains(e.target)) closeMenu(); });
}

function toggleMenu() {
  if (menu) { closeMenu(); return; }
  const st = last;
  const real = st && st.mode === 'real';
  menu = el('div', { class: 'lite-drag-menu', role: 'menu' },
    el('div', { class: 'lite-drag-menu-head', text: 'Arm Drag Mode: recovery only, nothing is recorded' }),
    ARMS.map((side) => {
      const a = st && st.arms[side];
      const why = !st ? 'status not read yet' : !real ? 'needs Real robot' : !a.link ? 'not connected to its controller'
        : a.recovery === 'drag' ? 'already in drag mode' : a.teaching ? 'drag teaching runs' : '';
      return el('button', { class: 'btn light', type: 'button', role: 'menuitem', disabled: !!why,
        onclick: () => { closeMenu(); dragMode(side); } }, `${side.toUpperCase()} arm`, why ? el('small', { text: ` (${why})` }) : null);
    }));
  document.body.append(menu);
  const r = $('liteDragBtn').getBoundingClientRect();
  menu.style.top = `${Math.round(r.bottom + 6)}px`;
  menu.style.left = `${Math.round(Math.max(8, Math.min(window.innerWidth - 300, r.left)))}px`;
}

function closeMenu() { menu?.remove(); menu = null; }

async function dragMode(side) {
  try {
    await robotCall(`/api/robots/${side}/drag-mode`, { on: true }, {
      what: `Arm Drag Mode on the ${side.toUpperCase()} arm: it becomes free to be moved by hand, for recovery only (nothing is `
        + 'recorded). Support the arm: it can sag if its load is not set right. Everything else is locked until Normal Mode' });
    window.toast(`${side.toUpperCase()} arm in Arm Drag Mode: move it by hand, then press Normal Mode.`, 'good');
  } catch (err) { tell(err); }
  refresh();
}

async function normalMode() {
  if (busy) return;
  busy = true;
  paintLock(last, 'Leaving drag mode and checking the arm…');
  const out = [];
  try {
    for (const side of ARMS) {
      const a = last && last.arms[side];
      if (a && a.recovery) out.push(await post(`/api/robots/${side}/drag-mode`, { on: false }));
    }
    const held = out.filter(a => a.recovery);
    window.toast(held.length ? `Still held: ${held.map(a => `${a.side.toUpperCase()}: ${a.recovery_note}`).join('; ')}.`
      : 'Normal Mode: the arm is at rest with no error. The page is unlocked.', held.length ? 'warn' : 'good');
  } catch (err) { tell(err); }
  busy = false;
  refresh();
}

async function clearErrors(side) {
  try {
    const out = await post(`/api/robots/${side}/clear-errors`, {});
    window.toast(out.clean ? `${side.toUpperCase()} controller: errors cleared.` : `${side.toUpperCase()} controller: still ${out.text}.`, out.clean ? 'good' : 'warn');
  } catch (err) { tell(err); }
  refresh();
}

async function refresh() {
  try { paintLock(await get('/api/robots/status')); } catch (_) { /* the status poll paints it next */ }
}

// The lock: over the whole page; Normal Mode and STOP BOTH stay above it and nothing else can be reached.
function paintLock(st, working = null) {
  if (st) last = st;
  const arms = last ? ARMS.filter(s => last.arms[s] && last.arms[s].recovery) : [];
  const on = arms.length > 0;
  if ($('liteNormalBtn')) $('liteNormalBtn').disabled = !on || busy;
  if ($('liteDragBtn')) $('liteDragBtn').classList.toggle('lite-active', arms.some(s => last.arms[s].recovery === 'drag'));
  let lock = $('liteDragLock');
  document.body.classList.toggle('lite-drag-locked', on);
  for (const n of [$('appShell'), ...document.querySelectorAll('.topbar > *, body > .modal, body > .lite-confirm')]) {
    if (!n || n.classList.contains('emstop') || n.id === 'liteNormalBtn') continue;
    if (on) n.setAttribute('inert', ''); else n.removeAttribute('inert');
  }
  if (on) closeMenu();
  if (!on) { lock?.remove(); return; }
  if (!lock) {
    lock = el('div', { id: 'liteDragLock', class: 'lite-drag-lock', role: 'alertdialog', 'aria-modal': 'true', 'aria-labelledby': 'liteDragLockTitle' });
    document.body.append(lock);
  }
  const rows = arms.map((side) => {
    const a = last.arms[side];
    const drag = a.recovery === 'drag';
    return el('div', { class: `lite-drag-lock-arm ${drag ? 'drag' : 'held'}` },
      el('b', { text: `${side.toUpperCase()} arm: ${drag ? 'in Arm Drag Mode' : 'held'}` }),
      el('p', { text: drag ? 'Free to be moved by hand, for recovery only: nothing is recorded. Move it to a safe place, supporting it, '
        + 'then press Normal Mode.' : `Not yet safe to use: ${a.recovery_note}. Fix it, then press Normal Mode again.` }),
      !drag && /error|reports/i.test(a.recovery_note || '')
        ? el('button', { class: 'btn', type: 'button', text: `Clear errors (${side.toUpperCase()})`, onclick: () => clearErrors(side) }) : null);
  });
  lock.replaceChildren(el('div', { class: 'lite-drag-lock-card' },
    el('h3', { id: 'liteDragLockTitle', text: 'Arm Drag Mode' }),
    el('p', { class: 'mini-note', text: 'Everything else is locked. Only Normal Mode and STOP BOTH (top right) can be pressed.' }),
    ...rows,
    working ? el('div', { class: 'lite-status', text: working }) : null,
    el('div', { class: 'cta-row' }, el('button', { class: 'btn green', type: 'button', text: 'Normal Mode', disabled: busy, onclick: normalMode }))));
}

export function installRecovery() {
  buttons();
  document.addEventListener('lite:status', (e) => { if (!busy) paintLock(e.detail); });
}
