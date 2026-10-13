// Shared UI: status line, toasts, dialogs. Motion dialogs focus Cancel and Enter never confirms.
import { esc, icon } from './core.js';

export function setStatus(text, busy = false) {
  const el = document.getElementById('statusText');
  el.textContent = text;
  el.dataset.idle = 'no';                      // keep this message until the next one
  clearTimeout(setStatus._t);
  setStatus._t = setTimeout(() => { el.dataset.idle = ''; }, busy ? 60000 : 8000);
  document.getElementById('statusSpin').hidden = !busy;
}

export function toast(message, kind = 'info', hint = '', ms = 6000) {
  const root = document.getElementById('toastRoot');
  const el = document.createElement('div');
  el.className = `toast ${kind}`;
  el.setAttribute('role', kind === 'bad' ? 'alert' : 'status');
  el.innerHTML = `<div><div>${esc(message)}</div>${hint ? `<div class="hint">${esc(hint)}</div>` : ''}</div>
    <button class="icon-btn x" aria-label="Close">${icon('x', 14)}</button>`;
  el.querySelector('.x').onclick = () => el.remove();
  root.append(el);
  if (kind === 'bad') ms = Math.max(ms, 12000);       // errors stay long enough to read
  setTimeout(() => el.remove(), ms);
  return el;
}

export function showError(e) {
  toast(e.message || String(e), 'bad', e.hint || '');
}

// A dialog. actions: [{label, kind:'primary'|'danger'|'quiet', value, focus:true}]
export function modal({ title, body = '', actions = [{ label: 'Close', value: null }], motion = false, wide = false }) {
  return new Promise((resolve) => {
    const root = document.getElementById('modalRoot');
    const scrim = document.createElement('div');
    scrim.className = 'scrim';
    const m = document.createElement('div');
    m.className = 'modal' + (motion ? ' motion' : '');
    if (wide) m.style.width = 'min(760px, 100%)';
    m.setAttribute('role', 'dialog');
    m.setAttribute('aria-modal', 'true');
    m.innerHTML = `<header><h3>${esc(title)}</h3></header><div class="content"></div><footer></footer>`;
    const content = m.querySelector('.content');
    if (body instanceof Node) content.append(body); else content.innerHTML = body;
    const foot = m.querySelector('footer');
    let focusEl = null;
    const close = (v) => { scrim.remove(); document.removeEventListener('keydown', onKey); resolve(v); };
    for (const a of actions) {
      const b = document.createElement('button');
      b.className = 'btn' + (a.kind ? ' ' + a.kind : '');
      b.textContent = a.label;
      b.onclick = () => close(a.value);
      if (a.disabledUntil) { b.disabled = true; a.disabledUntil(b); }
      foot.append(b);
      if (a.focus) focusEl = b;
    }
    const onKey = (ev) => {
      if (ev.key === 'Escape') close(null);
      if (ev.key === 'Enter' && motion) ev.preventDefault();   // Enter never confirms motion
    };
    document.addEventListener('keydown', onKey);
    scrim.append(m);
    root.append(scrim);
    (focusEl || foot.querySelector('button'))?.focus();
  });
}

// Robot motion or a controller overwrite: a plain question, the facts, a checklist, and the
// action named on its button. Cancel has the focus; the action unlocks when every box is ticked.
export function confirmMotion({ title, facts = [], checklist = [], action = 'Confirm', danger = true, note = '' }) {
  const body = document.createElement('div');
  body.className = 'stack';
  body.innerHTML = `
    <dl class="facts">${facts.map(([k, v]) => `<dt>${esc(k)}</dt><dd>${v}</dd>`).join('')}</dl>
    ${checklist.length ? `<div>${checklist.map((c, i) => `<label class="check"><input type="checkbox" data-i="${i}"><span>${esc(c)}</span></label>`).join('')}</div>` : ''}
    ${note ? `<p class="muted small">${esc(note)}</p>` : ''}
    <p class="muted small">The hardware E-stop buttons are the safety stop. STOP stays available at the top of the screen.</p>`;
  const boxes = () => [...body.querySelectorAll('input[type=checkbox]')];
  return modal({
    title, body, motion: true,
    actions: [
      { label: 'Cancel', value: false, focus: true },
      { label: action, value: true, kind: danger ? 'danger' : 'primary',
        disabledUntil: (btn) => {
          const upd = () => { btn.disabled = !boxes().every(b => b.checked); };
          body.addEventListener('change', upd); upd();
        } },
    ],
  });
}

export async function askPin() {
  const body = document.createElement('div');
  body.className = 'stack';
  body.innerHTML = `<p>Engineer mode shows the advanced settings, program numbers and the robot console.</p>
    <label class="field">Engineer PIN<input type="password" inputmode="numeric" autocomplete="off" id="pinInput"></label>`;
  setTimeout(() => body.querySelector('#pinInput')?.focus(), 30);
  body.addEventListener('keydown', (e) => { if (e.key === 'Enter') body.closest('.modal')?.querySelector('footer .primary')?.click(); });
  const ok = await modal({ title: 'Switch to Engineer mode?', body, actions: [
    { label: 'Cancel', value: false }, { label: 'Switch', value: true, kind: 'primary' }] });
  return ok ? body.querySelector('#pinInput').value : null;
}
