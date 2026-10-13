// Core helpers: API calls, shared state, formatting, small DOM helpers and icons.

// Lite's session: every request but STOP carries it. When the server asks for a log-in (code
// "login") Legacy Lite asks for the operator PIN, once for every request waiting, and keeps the
// session for this browser tab. A real move the server wants confirmed (code "confirm") is asked
// here too, then sent again with the yes. app.js provides both questions (ui.js).
export const asks = { pin: null, confirm: null };
const TOKEN_KEY = 'alix-lite-legacy-session';
let token = null;
try { token = sessionStorage.getItem(TOKEN_KEY); } catch (e) { /* no storage: log in after a reload */ }
let loggingIn = null;

async function send(path, body) {
  const headers = {};
  if (token) headers['X-Lite-Token'] = token;
  const opt = body === undefined ? { headers } : {
    method: 'POST', headers: { ...headers, 'Content-Type': 'application/json' }, body: JSON.stringify(body),
  };
  let res;
  try {
    res = await fetch(path, opt);
  } catch (e) {
    throw new LiteError('Lost contact with Lite.', 'Check that Lite is still running in its terminal.');
  }
  try { return await res.json(); } catch (e) { throw new LiteError('Lite sent an unreadable reply.', 'Try again.'); }
}

function logIn() {
  if (!loggingIn) {
    loggingIn = (async () => {
      let said = '';
      for (;;) {
        const pin = asks.pin ? await asks.pin(said) : null;
        if (pin === null) throw new LiteError('Not logged in.', 'Lite needs the operator PIN: reload the page to log in.');
        const r = await send('/api/auth/login', { pin });
        if (r && r.token) {
          token = r.token;
          try { sessionStorage.setItem(TOKEN_KEY, token); } catch (e) { /* kept for this page only */ }
          return;
        }
        said = (r && r.error) || 'Lite did not accept the PIN.';
      }
    })().finally(() => { loggingIn = null; });
  }
  return loggingIn;
}

export async function api(path, body, again = 2) {
  const data = await send('/api/lite/' + path, body);
  if (data && data.error) {
    if (data.code === 'login' && again > 0) { await logIn(); return api(path, body, again - 1); }
    if (data.code === 'confirm' && body !== undefined && !body.confirmed && asks.confirm) {
      if (!(await asks.confirm(data.error))) throw new LiteError('Cancelled: nothing was sent to the robot.');
      return api(path, { ...body, confirmed: true }, again);
    }
    throw Object.assign(new LiteError(data.error, data.hint), { code: data.code });
  }
  return data;
}

export class LiteError extends Error {
  constructor(message, hint) { super(message); this.hint = hint || ''; }
}

// ── shared state, refreshed from /api/lite/state ─────────────────────────
export const store = {
  state: null,
  listeners: new Set(),
  on(fn) { this.listeners.add(fn); return () => this.listeners.delete(fn); },
  set(s) { this.state = s; for (const fn of this.listeners) { try { fn(s); } catch (e) { console.error(e); } } },
};

export async function refreshState() {
  const s = await api('state');
  store.set(s);
  return s;
}

// ── formatting: operators see seconds and degrees, never lines or millidegrees ──
export function fmtS(s, digits = 1) {
  if (s === null || s === undefined || Number.isNaN(s)) return '–';
  const v = Number(s);
  if (v >= 60) {                                     // m:ss, as the UX rules ask
    const m = Math.floor(v / 60), r = Math.round(v - 60 * m);
    return r === 60 ? `${m + 1}:00 min` : `${m}:${String(r).padStart(2, '0')} min`;
  }
  return `${v.toFixed(digits)} s`;
}
export function pct(a, b) { return b ? Math.round(100 * (a - b) / b) : 0; }
// a file of N lines lasts (N - 1) x 5 ms: one line per 5 ms controller cycle (D26)
export function durS(lines) { return Math.max((Number(lines) || 0) - 1, 0) * 0.005; }
export const ARM = { left: 'LEFT', right: 'RIGHT' };
// a unit's code in the deck's own format: L-R05 PART 1
export function unitCode(u) { return `${u.code}${u.part ? ' PART ' + u.part : ''}`; }

export function esc(s) {
  return String(s ?? '').replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
}

// tiny element builder: h('div.card.pad', {onclick}, [children or html string])
export function h(sel, attrs = {}, children = []) {
  const [tag, ...cls] = sel.split('.');
  const el = document.createElement(tag || 'div');
  if (cls.length) el.className = cls.join(' ');
  for (const [k, v] of Object.entries(attrs || {})) {
    if (v === undefined || v === null || v === false) continue;
    if (k.startsWith('on') && typeof v === 'function') el.addEventListener(k.slice(2), v);
    else if (k === 'html') el.innerHTML = v;
    else if (k === 'text') el.textContent = v;
    else if (k === 'style' && typeof v === 'object') Object.assign(el.style, v);
    else el.setAttribute(k, v === true ? '' : v);
  }
  for (const c of [].concat(children)) {
    if (c === null || c === undefined || c === false) continue;
    el.append(c instanceof Node ? c : document.createTextNode(String(c)));
  }
  return el;
}

// ── status chips: always a glyph AND a word, never colour alone ───────────
const PIPELINE = {
  not_taught: ['todo', '○', 'Not taught'],
  taught: ['info', '●', 'Taught'],
  optimised: ['info', '◆', 'Optimised'],
  check_ok: ['ok', '✓', '3D check OK'],
  uploaded: ['ok', '✓', 'On the robot'],
  dryrun_ok: ['ok', '✓', 'Hardware sim OK'],
  real_ok: ['ok', '✓', 'Tested on robot'],
};
export function statusChip(status) {
  const [k, g, w] = PIPELINE[status] || ['todo', '·', status];
  return `<span class="chip ${k}"><span class="g">${g}</span>${w}</span>`;
}
export function chip(kind, glyph, word, title = '') {
  return `<span class="chip ${kind}" ${title ? `title="${esc(title)}"` : ''}><span class="g">${glyph}</span>${esc(word)}</span>`;
}
export function armBadge(side) {
  return `<span class="arm-badge ${side}">${ARM[side] || side}</span>`;
}
export const PIPE_ORDER = ['not_taught', 'taught', 'optimised', 'check_ok', 'uploaded', 'dryrun_ok', 'real_ok'];

// ── icons: ALIX style (16 px grid, 1.6 stroke, currentColor) ──────────────
const P = {
  home: '<path d="M2.5 7.5 8 3l5.5 4.5V13a.5.5 0 0 1-.5.5H10V10H6v3.5H3a.5.5 0 0 1-.5-.5z"/>',
  region: '<rect x="2.5" y="2.5" width="11" height="11" rx="1"/><path d="M5 9.5l2-2 2 2 2-3"/>',
  robot: '<rect x="4" y="6" width="8" height="6" rx="1"/><path d="M8 6V3.5M6.5 9h.01M9.5 9h.01M2.5 8.5v2M13.5 8.5v2"/>',
  settings: '<circle cx="8" cy="8" r="2"/><path d="M8 1.8v1.7M8 12.5v1.7M1.8 8h1.7M12.5 8h1.7M3.6 3.6l1.2 1.2M11.2 11.2l1.2 1.2M3.6 12.4l1.2-1.2M11.2 4.8l1.2-1.2"/>',
  help: '<circle cx="8" cy="8" r="6"/><path d="M6.3 6.3a1.8 1.8 0 1 1 2.4 1.7c-.5.2-.7.5-.7 1v.5M8 11.5h.01"/>',
  sun: '<circle cx="8" cy="8" r="2.8"/><path d="M8 1.5v1.6M8 12.9v1.6M1.5 8h1.6M12.9 8h1.6M3.4 3.4l1.1 1.1M11.5 11.5l1.1 1.1M3.4 12.6l1.1-1.1M11.5 4.5l1.1-1.1"/>',
  moon: '<path d="M12.8 9.8A5.3 5.3 0 0 1 6.2 3.2a5.3 5.3 0 1 0 6.6 6.6z"/>',
  teach: '<path d="M5 8V3.8a1 1 0 0 1 2 0V7M7 7V2.8a1 1 0 0 1 2 0V7M9 7V3.5a1 1 0 0 1 2 0v5.2c0 2.6-1.7 4.8-4 4.8-1.6 0-2.6-.8-3.5-2.2L2.3 9.1a1 1 0 0 1 1.6-1.1L5 9.3"/>',
  optimise: '<path d="M2 12c2-6 4-6 6-3s4 3 6-3"/><path d="M2 14h12"/>',
  cube: '<path d="M8 1.8 13.5 5v6L8 14.2 2.5 11V5z"/><path d="M2.5 5 8 8.2 13.5 5M8 8.2v6"/>',
  play: '<path d="M5 3.5v9l7-4.5z"/>',
  pause: '<path d="M5.5 3.5v9M10.5 3.5v9"/>',
  upload: '<path d="M8 10.5V2.5M5 5.5l3-3 3 3M3 10.5v2.5h10v-2.5"/>',
  undo: '<path d="M6 4 3 7l3 3"/><path d="M3 7h6.5a3.5 3.5 0 0 1 0 7H7"/>',
  redo: '<path d="m10 4 3 3-3 3"/><path d="M13 7H6.5a3.5 3.5 0 0 0 0 7H9"/>',
  check: '<path d="m3 8.5 3 3 7-7"/>',
  x: '<path d="M4 4l8 8M12 4l-8 8"/>',
  plug: '<path d="M6 2.5v3M10 2.5v3M4.5 5.5h7v2a3.5 3.5 0 0 1-7 0zM8 11v2.5"/>',
  list: '<path d="M5.5 4h8M5.5 8h8M5.5 12h8M2.5 4h.01M2.5 8h.01M2.5 12h.01"/>',
};
export function icon(name, size = 18) {
  return `<svg viewBox="0 0 16 16" width="${size}" height="${size}" fill="none" stroke="currentColor" stroke-width="1.6" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">${P[name] || ''}</svg>`;
}

// ── job polling: long work reports stage i of n, never a made-up percentage ──
export function watchJob(job, onUpdate, intervalMs = 250) {
  let since = 0, stopped = false;
  return new Promise((resolve) => {
    async function tick() {
      if (stopped) return;
      let v;
      try { v = await api(`jobs/${job.id}?since=${since}`); } catch (e) { setTimeout(tick, 600); return; }
      since += (v.events || []).length;
      onUpdate && onUpdate(v);
      if (v.state === 'running') setTimeout(tick, intervalMs);
      else resolve(v);
    }
    tick();
  });
}

export function debounce(fn, ms) {
  let t;
  return (...a) => { clearTimeout(t); t = setTimeout(() => fn(...a), ms); };
}
