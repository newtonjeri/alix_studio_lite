// Hardware errors and failures, at the bottom right, until the operator closes them (the owner's
// request, 14 October): a controller's error codes, a robot command refused or failed with its
// return code, STOP not confirmed, an E-stop press, a link that stopped reading. They come with
// the robot status (status.js polls it; server/logbook.py keeps them), newest last, and every one
// is also in the hardware log. Closed ones stay closed in this browser tab. Each is coloured by its
// level (the owner, 20 Oct): red an error, orange a warning, green a good status (an arm connected again).

import { el } from './ctx.js';

const KEY = 'alix-lite-closed-alerts';
let box = null;
const shown = new Map();                       // id -> card
let closed = new Set();
try { closed = new Set(JSON.parse(sessionStorage.getItem(KEY) || '[]')); } catch (_) { /* a fresh set */ }

function remember() {
  try { sessionStorage.setItem(KEY, JSON.stringify([...closed].slice(-300))); } catch (_) { /* this page only */ }
}

function close(id) {
  closed.add(id);
  remember();
  shown.get(id)?.remove();
  shown.delete(id);
  tidy();
}

function tidy() {
  if (!box) return;
  const all = box.querySelector('.lite-alerts-all');
  if (all) all.hidden = shown.size < 2;
  box.hidden = shown.size === 0;
}

function card(a) {
  const when = new Date(a.at * 1000).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit', second: '2-digit' });
  const head = { error: 'Hardware error', warn: 'Hardware warning', good: 'Hardware status' }[a.level] || 'Hardware status';
  return el('div', { class: `lite-alert ${a.level}`, role: a.level === 'error' ? 'alert' : 'status' },
    el('div', { class: 'lite-alert-head' },
      el('b', { text: `${a.side ? a.side.toUpperCase() + ' · ' : ''}${head}` }),
      el('small', { text: when }),
      el('button', { class: 'lite-alert-x', type: 'button', 'aria-label': 'Close this message', text: '✕', onclick: () => close(a.key) })),
    el('div', { text: a.text }),
    a.hint ? el('small', { class: 'lite-alert-hint', text: a.hint }) : null);
}

function ensureBox() {
  if (box) return box;
  box = el('div', { class: 'lite-alerts', 'aria-live': 'polite', hidden: true },
    el('button', { class: 'btn light lite-alerts-all', type: 'button', text: 'Close all', hidden: true,
      onclick: () => { for (const id of [...shown.keys()]) close(id); } }));
  document.body.append(box);
  return box;
}

export function installAlerts() {
  document.addEventListener('lite:status', (e) => {
    const st = e.detail || {};
    for (const a of st.alerts || []) {
      const id = `${st.instance || ''}:${a.id}`;        // ids restart with the server
      if (shown.has(id) || closed.has(id)) continue;
      const c = card({ ...a, key: id });
      ensureBox().insertBefore(c, box.querySelector('.lite-alerts-all'));
      shown.set(id, c);
    }
    while (shown.size > 6) close(shown.keys().next().value);      // the oldest goes first
    tidy();
  });
}
