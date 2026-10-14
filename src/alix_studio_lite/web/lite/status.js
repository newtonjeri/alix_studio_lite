// The live state in the top bar: the page's "FRONTEND READY · Controller backend not connected"
// pill shows what the backend reports instead: the mode (Visual sim, Hardware sim or Real),
// which robot is behind it (the stand-in, or the live link to the arms) and any E-stop.

import { get, session } from './api.js';

const MODE = { visual: 'VISUAL SIM', hwsim: 'HARDWARE SIM', real: 'REAL ROBOT' };
let timer = null;
let last = null;                                     // the last /api/robots/status

// The mode the server runs region steps in (visual | hwsim | real), read now when not known yet;
// null when the server cannot be reached. The pill's data-lite also shows E-stop and offline.
export async function liteMode() {
  if (!last) await poll();
  return last ? last.mode : null;
}
// Whether robot actions go to the stand-in (until Lite's robot commands exist), and whether an
// arm has the live link: then Lite commands its controller.
export const standIn = () => !last || last.backend === 'stand-in';
export const linked = () => !!last && Object.values(last.arms || {}).some((a) => a.link);

function pillText(st) {
  if (st.motion_block) return { state: 'estop', head: 'E-STOP', text: st.motion_block };
  const arms = Object.values(st.arms || {});
  const linked = arms.filter((a) => a.connected && a.link).map((a) => a.side.toUpperCase());
  const robot = linked.length ? `${linked.join(' + ')} live`
    : st.backend === 'stand-in' ? 'stand-in robot, no hardware' : 'robot not connected';
  return { state: st.mode, head: MODE[st.mode] || st.mode, text: robot };
}

const el = (tag, className, text) => Object.assign(document.createElement(tag), { className, textContent: text });

function show(state, head, text, title) {
  const pill = document.querySelector('.topbar .mode-pill');
  if (!pill) return;
  pill.dataset.lite = state;
  pill.title = title ? `${head}: ${title}` : `${head} · ${text}`;
  pill.replaceChildren(el('span', 'dot', ''), el('b', 'lite-head', head), el('span', 'lite-label', ` · ${text}`));
  const rc = document.querySelector('.rc-backend-state');
  if (rc) {
    rc.dataset.lite = state;
    const b = rc.querySelector('b'), small = rc.querySelector('small');
    if (b) b.textContent = state === 'offline' ? 'Backend not reachable' : 'Backend connected';
    if (small) small.textContent = `${head} · ${text}`;
  }
}

async function poll() {
  if (!session.token) return;
  try {
    const st = await get('/api/robots/status');
    last = st;
    document.dispatchEvent(new CustomEvent('lite:status', { detail: st }));      // alerts.js, and anyone else
    const p = pillText(st);
    show(p.state, p.head, p.text, st.mode_text);
  } catch (err) {
    last = null;
    if (err.code === 'offline') show('offline', 'NO SERVER', 'restart ALIX Studio Lite');
  }
}

export function installStatus() {
  show('offline', 'CONNECTING', 'to the Lite server');
  if (timer) clearInterval(timer);
  timer = setInterval(poll, 1500);
  poll();
  document.addEventListener('lite:mode', poll);
}
