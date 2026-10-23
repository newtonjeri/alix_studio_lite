// The live state in the top bar: the page's "FRONTEND READY · Controller backend not connected"
// pill shows what the backend reports instead: the mode (Visual sim, Hardware sim or Real),
// which robot is behind it (the stand-in, or the live link to the arms) and any E-stop.
// Beside it, the speed slider (the owner, 15 Oct): the % every MoveJ / MoveL Lite sends goes at,
// the same for both arms; each controller gets back the speed it had when it was connected.

import { get, post, session } from './api.js';

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
// The speed slider's %: MoveJ / MoveL, both arms.
export const speedPct = () => (last && last.speed ? last.speed.pct : 15);
// S2's record of the controllers' feed (run_lite --feed 20 | off): its period in ms, null when off or not read yet.
export const feedMs = () => (last && last.feed_ms ? last.feed_ms : null);

// Each linked controller's own run mode as it reads (the owner, 17 Oct: Hardware sim puts the controllers in their
// SIM mode, Real in their REAL mode); one that differs from the mode chosen (changed on its pendant) is named.
function pillText(st) {
  if (st.motion_block) return { state: 'estop', head: 'E-STOP', text: st.motion_block };
  const arms = Object.values(st.arms || {}).filter((a) => a.connected && a.link);
  const linked = arms.map((a) => a.side.toUpperCase());
  const run = (a) => (a.link && a.link.feedback && a.link.feedback.run_mode) || null;
  const want = st.mode === 'hwsim' ? 'SIM' : st.mode === 'real' ? 'REAL' : null;
  const modes = [...new Set(arms.map(run).filter(Boolean))];
  const off = want ? arms.filter((a) => run(a) && run(a) !== want).map((a) => `${a.side.toUpperCase()} controller ${run(a)}`) : [];
  const robot = linked.length ? `${linked.join(' + ')} live${off.length ? ` · ${off.join(', ')}` : modes.length === 1 ? ` · ${modes[0]}` : ''}`
    : st.backend === 'stand-in' ? 'stand-in robot, no hardware' : 'robot not connected';
  return { state: off.length ? 'estop' : st.mode, head: MODE[st.mode] || st.mode, text: robot };
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

// The speed slider: MoveJ / MoveL for both arms; what each controller had is put back after every
// move, on disconnecting and when Lite closes.
let speedBusy = false;
function speedSlider() {
  let box = document.getElementById('liteSpeed');
  if (box) return box;
  const pill = document.querySelector('.topbar .mode-pill');
  if (!pill) return null;
  const input = Object.assign(document.createElement('input'), { type: 'range', min: '1', max: '100', step: '1', value: '15', id: 'liteSpeedRange' });
  input.setAttribute('aria-label', 'MoveJ / MoveL speed, both arms (%)');
  const val = el('b', 'lite-speed-val', '15 %');
  box = Object.assign(document.createElement('label'), { id: 'liteSpeed', className: 'lite-speed' });
  box.append(el('span', 'lite-speed-head', 'Speed'), input, val);
  input.addEventListener('input', () => { speedBusy = true; val.textContent = `${input.value} %`; });
  input.addEventListener('change', async () => {
    try {
      const out = await post('/api/robots/speed', { pct: Number(input.value) });
      if (last) last.speed = out;
      window.toast(`Speed ${out.pct} %: every MoveJ / MoveL of both arms goes at it.`, 'good');
      document.dispatchEvent(new CustomEvent('lite:speed', { detail: out }));   // the 3D's moves to a start follow it
    } catch (err) { window.toast(err.message || String(err), 'error'); } finally { speedBusy = false; poll(); }
  });
  pill.before(box);
  return box;
}

function showSpeed(sp) {
  const box = speedSlider();
  if (!box || !sp) return;
  const input = box.querySelector('input'), val = box.querySelector('.lite-speed-val');
  if (!speedBusy && document.activeElement !== input) { input.value = String(sp.pct); val.textContent = `${sp.pct} %`; }
  const arms = Object.entries(sp.found || {}).filter(([, f]) => f.connected)
    .map(([side, f]) => `${side.toUpperCase()} ${f.pct} %${f.read ? ' (read)' : ' (the controller does not report it: taken as ' + sp.assumed + ' %)'}`);
  box.title = `MoveJ / MoveL speed, both arms: ${sp.pct} %. `
    + (arms.length ? `Each controller gets back the speed it had: ${arms.join(', ')}, after every move and when Lite disconnects or closes.`
      : 'Connect an arm: Lite reads its speed and gives it back after every move.');
}

// The arm indicators, top right (the owner, 17 Oct): one circle per arm, green a good connection, yellow a
// weak one, red none (a cable out, the controller off or not answering); the server grades each link
// (robot_link.ArmLink.quality) and says why in the circle's title. After them the wrist camera's (the owner,
// 21 Oct): green connected (a D435 plugged in, or open and its frames coming), red not (camera_d435.status).
const LEVEL = { good: 'Good connection', weak: 'Weak connection', none: 'No connection' };
const CAMERA = { good: 'Connected', none: 'Not connected' };
function armDots(arms, offline, camera) {
  let box = document.getElementById('liteArmDots');
  if (!box) {
    const stop = document.querySelector('.topbar .emstop');
    if (!stop) return;
    box = el('div', 'lite-arm-dots', '');
    box.id = 'liteArmDots';
    box.setAttribute('role', 'status');
    for (const side of ['left', 'right']) {
      const dot = el('span', 'lite-arm-dot', '');
      dot.dataset.side = side;
      dot.append(el('i', 'lite-arm-dot-led', ''), el('b', '', side === 'left' ? 'L' : 'R'));
      box.append(dot);
    }
    const cam = el('span', 'lite-arm-dot lite-cam-dot', '');
    cam.id = 'liteCamDot';
    cam.append(el('i', 'lite-arm-dot-led', ''), el('b', '', 'CAM'));
    box.append(cam);
    stop.before(box);
  }
  const cam = document.getElementById('liteCamDot');
  if (cam) {
    const c = offline ? { level: 'none', text: 'the Lite server is not answering' } : camera || { level: 'none', text: 'not looked at yet' };
    cam.dataset.level = c.level;
    cam.title = `Wrist camera: ${CAMERA[c.level] || c.level} (${c.text})`;
    cam.setAttribute('aria-label', cam.title);
  }
  for (const dot of box.querySelectorAll('.lite-arm-dot[data-side]')) {
    const side = dot.dataset.side, q = offline ? { level: 'none', text: 'the Lite server is not answering' }
      : ((arms || {})[side] || {}).quality || { level: 'none', text: 'not connected' };
    dot.dataset.level = q.level;
    dot.title = `${side.toUpperCase()} arm: ${LEVEL[q.level] || q.level} (${q.text})`;
    dot.setAttribute('aria-label', dot.title);
  }
}

async function poll() {
  if (!session.token) return;
  try {
    const st = await get('/api/robots/status');
    last = st;
    showSpeed(st.speed);
    document.dispatchEvent(new CustomEvent('lite:status', { detail: st }));      // alerts.js, and anyone else
    const p = pillText(st);
    show(p.state, p.head, p.text, st.mode_text);
    armDots(st.arms, false, st.camera);
  } catch (err) {
    last = null;
    if (err.code === 'offline') { show('offline', 'NO SERVER', 'restart ALIX Studio Lite'); armDots(null, true); }
  }
}

// A view of an earlier start (the owner, 21 Oct: "Starting the app should automatically close any existing front end
// views"): this start of the server (`boot`, from /api/lite/hello, which needs no log-in) is read every few seconds;
// once another start answers, this view belongs to the Lite before. It says so over the page and closes itself where
// the browser lets a page close (a new start closes Lite's own Chrome windows itself: server/__init__.py). STOP BOTH
// stays above it wherever it shows (never on the log-in screen, where such a page usually is by then).
let boot = null, bootTimer = null;
async function watchBoot() {
  let d;
  try {
    const r = await fetch('/api/lite/hello', { cache: 'no-store' });
    d = await r.json();
  } catch (_) { return; }                             // restarting, or gone: nothing to compare yet
  if (!d || !d.boot) return;
  if (boot == null) { boot = d.boot; return; }
  if (d.boot !== boot) staleView();
}

function staleView() {
  clearInterval(bootTimer); clearInterval(timer);
  if (document.getElementById('liteStale')) return;
  const reload = Object.assign(document.createElement('button'), { className: 'btn', type: 'button', textContent: 'Reload this window' });
  reload.addEventListener('click', () => window.location.reload());
  const card = el('div', 'lite-stale-card', '');
  card.append(el('h2', '', 'ALIX Studio Lite was started again'),
    el('p', '', 'This window belongs to the run before it, so it is closed. Use the window the new start opened, '
      + 'or reload this one to use the new start here.'), reload);
  const box = el('div', 'lite-stale', '');
  box.id = 'liteStale';
  box.setAttribute('role', 'alertdialog');
  box.setAttribute('aria-modal', 'true');
  box.append(card);
  document.body.append(box);
  reload.focus();
  window.close();                                     // where the browser lets a page close itself
}

export function installStatus() {
  show('offline', 'CONNECTING', 'to the Lite server');
  if (timer) clearInterval(timer);
  timer = setInterval(poll, 1500);
  poll();
  document.addEventListener('lite:mode', poll);
  clearInterval(bootTimer);
  bootTimer = setInterval(watchBoot, 3000);
  watchBoot();
}
