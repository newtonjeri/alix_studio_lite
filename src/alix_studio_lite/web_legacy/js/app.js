// The shell: app bar (fixture, mode, arms, STOP, role, theme), the tab bar (current region + tabs,
// as ALIX Studio's ribbon tabs), banners and the router.
import { api, asks, store, refreshState, icon, esc, ARM, unitCode, h, PIPE_ORDER } from './core.js';
import { setStatus, toast, showError, confirmMotion, askPin, modal } from './ui.js';

// What core.api asks: the operator PIN for Lite's session, and a yes before a real move.
asks.pin = async (message = '') => {
  const body = document.createElement('div');
  body.className = 'stack';
  body.innerHTML = `<p>Log in with the operator PIN. STOP works without it.</p>${message ? `<p class="muted small">${esc(message)}</p>` : ''}
    <label class="field">Operator PIN<input type="password" inputmode="numeric" autocomplete="off" id="loginPinInput"></label>`;
  setTimeout(() => body.querySelector('#loginPinInput')?.focus(), 30);
  body.addEventListener('keydown', (e) => { if (e.key === 'Enter') body.closest('.modal')?.querySelector('footer .primary')?.click(); });
  const ok = await modal({ title: 'ALIX Studio Lite', body, actions: [
    { label: 'Cancel', value: false }, { label: 'Log in', value: true, kind: 'primary' }] });
  return ok ? body.querySelector('#loginPinInput').value : null;
};
asks.confirm = (message) => confirmMotion({ title: 'Real robot: the arm will move', facts: [['What', esc(message)]],
  checklist: ['Nobody is inside the cell.', 'I can reach an E-stop button.'], action: 'Move' });

const SCREENS = {
  regions: () => import('./screens/home.js'),
  teach: () => import('./screens/unit.js'),
  optimise: () => import('./screens/unit.js'),
  view3d: () => import('./screens/view3d.js'),
  run: () => import('./screens/unit.js'),
  robot: () => import('./screens/robot.js'),
  settings: () => import('./screens/settings.js'),
  help: () => import('./screens/help.js'),
};
const STEP_TABS = [
  { id: 'teach', title: 'Teach', icon: 'teach' },
  { id: 'optimise', title: 'Optimise', icon: 'optimise' },
  { id: 'view3d', title: '3D view', icon: 'cube' },
  { id: 'run', title: 'Run', icon: 'play' },
];

let cleanup = null;
let current = null;            // the current region: the full unit from /api/lite/unit/<key>, or a stub
let routeSeq = 0;

// ── theme ────────────────────────────────────────────────────────────────
function applyTheme(t) {
  document.documentElement.dataset.theme = t;
  document.getElementById('themeBtn').innerHTML = icon(t === 'dark' ? 'sun' : 'moon');
  try { localStorage.setItem('alixLite.theme', t); } catch (e) { /* private window */ }
  window.dispatchEvent(new CustomEvent('lite-theme', { detail: t }));
}
function initialTheme() {
  const q = new URLSearchParams(location.search).get('theme');
  if (q === 'dark' || q === 'light') return q;
  try { return localStorage.getItem('alixLite.theme') || 'light'; } catch (e) { return 'light'; }
}

// ── app bar ──────────────────────────────────────────────────────────────
let barSig = '';
function renderBar(s) {
  const fx = document.getElementById('fixtureBtn');
  fx.textContent = s.project?.name || 'Choose a fixture';
  document.getElementById('statusVersion').textContent = `Lite ${s.version}`;
  const connected = Object.values(s.arms).some(a => a.connected);
  for (const b of document.querySelectorAll('#modeSwitch button')) {
    const m = b.dataset.mode;
    b.classList.toggle('on', s.mode === m);
    b.classList.toggle('real', m === 'real');
    b.setAttribute('aria-checked', String(s.mode === m));
    b.disabled = m !== 'visual' && !connected;
    b.title = b.disabled ? 'Connect an arm first (the LEFT / RIGHT chips).' : b.dataset.tip;
  }
  const sig = JSON.stringify(s.arms);
  if (sig !== barSig) {
    barSig = sig;
    const chips = document.getElementById('armChips');
    chips.innerHTML = '';
    for (const [side, a] of Object.entries(s.arms)) {
      const glyph = { CLEAR: '✓', ENGAGED: '✗', RECOVERING: '△', UNKNOWN: '△' }[a.estop] || '';
      const word = { CLEAR: 'E-stop clear', ENGAGED: 'E-STOP', RECOVERING: 'Recover', UNKNOWN: 'E-stop ?', NOT_CONNECTED: 'Connect' }[a.estop];
      chips.append(h(`button.arm-chip.${side}`, {
        title: a.connected ? `${ARM[side]} arm at ${a.ip}. E-stop input ${a.estop_io}: ${a.estop_text}. Click for options.`
                           : `${ARM[side]} arm (${a.ip}) is not connected. Click to connect.`,
        onclick: () => armMenu(side, a),
        html: `<span class="dot ${a.connected ? 'on' : ''}"></span><span class="arm-name">${ARM[side]}</span>
          <span class="estop ${a.estop}">${glyph}${glyph ? ' ' : ''}<span class="${a.connected ? 'estop-word' : ''}">${esc(word)}</span></span>`,
      }));
    }
  }
  document.getElementById('stopGroup').hidden = !connected;
  const role = document.getElementById('roleChip');
  role.textContent = s.role === 'engineer' ? 'Engineer' : 'Operator';
  role.classList.toggle('engineer', s.role === 'engineer');
  renderBanners(s);
  if (s.role !== renderTabs.role) renderTabs();
  const running = s.jobs || [];
  document.getElementById('statusJobs').textContent = running.length ? `${running.length} task(s) running` : '';
  const st = document.getElementById('statusText');
  if (!running.length && st.dataset.idle !== 'no') {
    st.textContent = { visual: 'Ready. Visual sim: the robot moves in the 3D view; no hardware is used.',
      hwsim: 'Ready. Hardware sim: the controller runs programs in SIM mode; the arms stay still.',
      real: 'Ready. REAL ROBOT: runs move the arms.' }[s.mode];
  }
}

let bannerSig = '';
function renderBanners(s) {
  const el = document.getElementById('banners');
  const parts = [];
  for (const [side, a] of Object.entries(s.arms).filter(([, a]) => a.connected && a.estop !== 'CLEAR')) {
    const rec = a.estop === 'RECOVERING';
    parts.push(`<div class="banner estop" role="alert"><span>✗</span><span class="grow">
      ${rec ? `${ARM[side]} arm: E-stop released. Recover before any motion.` :
              `E-STOP PRESSED on the ${ARM[side]} arm. All robot motion is blocked on both arms. Make the cell safe, then release the button.`}</span>
      ${rec ? `<button class="btn" data-recover="${side}">Recover ${ARM[side]} arm</button>` : ''}</div>`);
  }
  if (s.mode === 'real') parts.push(`<div class="banner real" role="status"><span>⚠</span><span class="grow">REAL ROBOT — the arms will move.</span></div>`);
  if (s.robot_backend === 'stand-in') parts.push(`<div class="banner info"><span class="grow">${esc(s.robot_label)}: robot screens work for review; nothing moves. The robot connection arrives with the backend.</span></div>`);
  const html = parts.join('');
  if (html === bannerSig) return;
  bannerSig = html;
  el.innerHTML = html;
  el.querySelectorAll('[data-recover]').forEach(b => b.onclick = () => recoverWizard(b.dataset.recover));
}

// ── tab bar ──────────────────────────────────────────────────────────────
export function stepStates(u) {
  if (!u || !u.recordings) return null;
  const i = PIPE_ORDER.indexOf(u.status);
  return {
    teach: u.recordings.length ? 'done' : 'todo',
    optimise: !u.recordings.length ? 'locked' : (u.versions.length ? 'done' : 'todo'),
    view3d: i >= 3 ? 'done' : 'todo',
    run: i < 3 ? 'locked' : (i >= 6 ? 'done' : 'todo'),
  };
}

function renderTabs() {
  const s = store.state || {};
  renderTabs.role = s.role;
  const bar = document.getElementById('tabbar');
  const r = parseRoute();
  const st = stepStates(current);
  bar.innerHTML = '';
  bar.append(h('button.region-pick', { title: 'Choose the region to work on', onclick: pickRegion,
    html: current ? `<span class="lbl">Region</span><b>${esc(unitCode(current))} · ${esc(current.name)}</b> ▾`
                  : `<span class="lbl">Region</span><b>Choose…</b> ▾` }), h('span.sep'));
  const tab = (id, title, ic, { href, state, disabled } = {}) => {
    const on = r.screen === id;
    const glyph = state === 'done' ? '<span class="st done">✓</span>' : state === 'locked' ? '<span class="st">🔒</span>' : '';
    bar.append(h('a.tab' + (on ? '.on' : ''), { href: disabled ? null : href, role: 'tab', 'aria-selected': String(on),
      'aria-disabled': disabled ? 'true' : null, title: disabled ? 'Finish the earlier steps first.' : title,
      html: `${icon(ic)}<span class="tl">${esc(title)}</span>${glyph}` }));
  };
  tab('regions', 'Regions', 'home', { href: '#/regions' });
  for (const t of STEP_TABS) {
    const state = st ? st[t.id] : null;
    tab(t.id, t.title, t.icon, { href: current ? `#/${t.id}/${current.key}` : (t.id === 'view3d' ? '#/view3d' : '#/regions'),
      state, disabled: state === 'locked' });
  }
  bar.append(h('span.spacer'));
  if (s.role === 'engineer') {
    tab('robot', 'Robot', 'robot', { href: '#/robot' });
    tab('settings', 'Settings', 'settings', { href: '#/settings' });
  }
  tab('help', 'Help', 'help', { href: '#/help' });
}

export function setCurrentUnit(u) {
  current = u || null;
  try { sessionStorage.setItem('alixLite.unit', JSON.stringify(u ? { key: u.key, code: u.code, part: u.part, name: u.name } : null)); } catch (e) { /* ignore */ }
  renderTabs();
}
export function currentUnit() { return current; }

async function pickRegion() {
  let cat;
  try { cat = await api('catalogue'); } catch (e) { showError(e); return; }
  const body = h('div.stack');
  const q = h('input', { type: 'text', placeholder: 'Search: name or code', 'aria-label': 'Search regions' });
  const list = h('div.picker-list');
  let resolve;
  const done = new Promise(r => { resolve = r; });
  const choose = (u) => { document.querySelector('#modalRoot .scrim')?.remove(); resolve(u); };
  const draw = () => {
    const t = q.value.trim().toLowerCase();
    list.innerHTML = '';
    for (const u of cat.units.filter(u => u.first_release)) {
      if (t && !(`${unitCode(u)} ${u.name} ${u.arm}`.toLowerCase().includes(t))) continue;
      list.append(h(`button${current?.key === u.key ? '.cur' : ''}`, { onclick: () => choose(u), html:
        `<span class="arm-badge ${u.arm}">${ARM[u.arm]}</span><span><b>${esc(u.name)}</b><br><span class="code">${esc(unitCode(u))}</span></span>
         <span class="small muted">${esc(u.label || '')}</span>` }));
    }
  };
  q.addEventListener('input', draw);
  body.append(q, list);
  draw();
  setTimeout(() => q.focus(), 30);
  modal({ title: 'Choose a region', body, actions: [{ label: 'Close', value: null }] }).then(() => resolve(null));
  const u = await done;
  if (!u) return;
  current = { key: u.key, code: u.code, part: u.part, name: u.name };
  const r = parseRoute();
  const step = ['teach', 'optimise', 'view3d', 'run'].includes(r.screen) ? r.screen : 'teach';
  location.hash = `#/${step}/${u.key}`;
}

async function pickFixture() {
  let d;
  try { d = await api('fixtures'); } catch (e) { showError(e); return; }
  const body = h('div.picker-list');
  let resolve;
  const done = new Promise(r => { resolve = r; });
  for (const f of d.fixtures) {
    body.append(h(`button${f.id === d.current ? '.cur' : ''}`, {
      onclick: () => { document.querySelector('#modalRoot .scrim')?.remove(); resolve(f.id); },
      html: `<span class="chip info">${esc(f.type_text)}</span><span><b>${esc(f.name)}</b><br><span class="code">${esc(f.id)}</span></span>
        <span class="small muted">${f.has_deck ? 'regions deck' : 'provisional regions'}${f.default ? ' · default' : ''}</span>` }));
  }
  modal({ title: 'Choose the fixture', body, actions: [{ label: 'Close', value: null }] }).then(() => resolve(null));
  const id = await done;
  if (!id || id === d.current) return;
  try {
    setStatus('Opening the fixture…', true);
    await api('fixture', { id });
    current = null;
    try { sessionStorage.removeItem('alixLite.unit'); } catch (e) { /* ignore */ }
    await refreshState();
    setStatus(`${store.state.project?.name} opened.`);
    if (location.hash === '#/regions') route(); else location.hash = '#/regions';
  } catch (e) { setStatus('The fixture did not change.'); showError(e); }
}

// ── arms, modes, STOP, role ──────────────────────────────────────────────
async function armMenu(side, a) {
  const s = store.state;
  if (!a.connected) {
    try {
      setStatus(`Connecting to the ${ARM[side]} arm at ${a.ip}…`, true);
      await api('robot/connect', { arm: side });
      await refreshState();
      setStatus(`${ARM[side]} arm connected. Its E-stop input (IO3) reads released.`);
      toast(`${ARM[side]} arm connected.`, 'ok');
    } catch (e) { setStatus('Not connected.'); showError(e); }
    return;
  }
  const body = `<dl class="facts">
      <dt>Address</dt><dd class="mono">${esc(a.ip)}</dd>
      <dt>E-stop input</dt><dd>${esc(a.estop_io)} · ${esc(a.estop_text)}</dd>
      <dt>Controller mode</dt><dd>${a.controller_mode === 'SIM' ? 'SIM (hardware sim)' : 'REAL'}</dd>
      <dt>Pole</dt><dd>${a.pole_mm} mm</dd></dl>`;
  const actions = [{ label: 'Close', value: null, focus: true }, { label: 'Disconnect', value: 'disconnect' }];
  if (s.role === 'engineer') actions.splice(1, 0, { label: a.estop === 'ENGAGED' ? 'Test: release E-stop' : 'Test: press E-stop', value: 'estop' });
  const v = await modal({ title: `${ARM[side]} arm`, body, actions });
  try {
    if (v === 'disconnect') { await api('robot/disconnect', { arm: side }); toast(`${ARM[side]} arm disconnected.`); }
    if (v === 'estop') await api('robot/estop-test', { arm: side, pressed: a.estop !== 'ENGAGED' });
    await refreshState();
  } catch (e) { showError(e); }
}

async function recoverWizard(side) {
  const body = h('div.stack', { html: `<ol class="stack" style="margin:0;padding-left:20px">
      <li>The cell is safe: nobody is inside, nothing is trapped.</li>
      <li>The E-stop button is released (pulled out).</li>
      <li>Lite clears the controller's errors and reads them back.</li>
      <li>Before the next run, Lite checks the move back to the program's start in 3D.</li></ol>
    <p class="muted">Nothing restarts by itself. You start the region again on purpose.</p>` });
  const ok = await modal({ title: `Recover the ${ARM[side]} arm?`, body, actions: [
    { label: 'Cancel', value: false, focus: true }, { label: 'Recover', value: true, kind: 'primary' }] });
  if (!ok) return;
  try { await api('robot/recover', { arm: side }); await refreshState(); toast(`${ARM[side]} arm recovered.`, 'ok'); }
  catch (e) { showError(e); }
}

async function setMode(mode) {
  const s = store.state;
  if (mode === s.mode) return;
  let confirmed = false;
  if (mode === 'real') {
    const arms = Object.entries(s.arms).filter(([, a]) => a.connected);
    confirmed = await confirmMotion({
      title: 'Switch to the real robot?',
      facts: [['Arms', arms.map(([k, a]) => `${ARM[k]} (${esc(a.ip)})`).join(', ')],
              ['E-stop', arms.map(([k, a]) => `${ARM[k]}: ${esc(a.estop_text)}`).join(', ')],
              ['What changes', 'Runs and tests move the arms.']],
      checklist: ['Nobody is inside the cell.', 'I can reach an E-stop button.'],
      action: 'Switch to real robot',
    });
    if (!confirmed) return;
  }
  try { await api('mode', { mode, confirmed }); await refreshState(); }
  catch (e) { showError(e); }
}

async function stop(side) {
  window.dispatchEvent(new CustomEvent('lite-stop', { detail: side }));     // also stops a 3D playback
  try {
    const r = await api('robot/stop', { arm: side });
    toast(side === 'all' ? 'STOP sent to both arms.' : `STOP sent to the ${ARM[side]} arm.`, 'warn',
      r.stopped ? `${r.stopped} task(s) stopped.` : 'Nothing was running.');
    await refreshState();
  } catch (e) { showError(e); }
}

async function toggleRole() {
  const s = store.state;
  try {
    if (s.role === 'engineer') await api('role', { role: 'operator' });
    else {
      const pin = await askPin();
      if (pin === null) return;
      await api('role', { role: 'engineer', pin });
    }
    await refreshState();
  } catch (e) { showError(e); }
}

// ── router ───────────────────────────────────────────────────────────────
const LEGACY = { home: 'regions', check: 'view3d', test: 'run' };
export function parseRoute() {
  const raw = (location.hash || '#/regions').replace(/^#\/?/, '').split('?')[0];
  let parts = raw.split('/').filter(Boolean);
  if (parts[0] === 'unit') parts = [LEGACY[parts[2]] || parts[2] || 'teach', parts[1]];
  const screen = LEGACY[parts[0]] || parts[0] || 'regions';
  return { screen: SCREENS[screen] ? screen : 'regions', params: parts.slice(1) };
}
export function routeQuery() {
  return new URLSearchParams((location.hash.split('?')[1]) || '');
}

async function route() {
  const seq = ++routeSeq;
  const { screen, params } = parseRoute();
  if (cleanup) { try { cleanup(); } catch (e) { console.error(e); } cleanup = null; }
  const main = document.getElementById('main');
  main.innerHTML = '';
  main.scrollTop = 0;
  if (['teach', 'optimise', 'run'].includes(screen) && !params[0]) {
    location.replace(current ? `#/${screen}/${current.key}` : '#/regions');
    return;
  }
  renderTabs();
  try {
    const mod = await SCREENS[screen]();
    if (seq !== routeSeq) return;
    const args = ['teach', 'optimise', 'run'].includes(screen) ? [params[0], screen] : params;
    cleanup = (await mod.render(main, args)) || null;
  } catch (e) {
    console.error(e);
    main.innerHTML = `<div class="page"><div class="callout bad"><span class="g">✗</span><div class="body"><strong>This screen could not open.</strong><span>${esc(e.message)}</span></div></div></div>`;
  }
  renderTabs();
}

// ── start ────────────────────────────────────────────────────────────────
async function start() {
  applyTheme(initialTheme());
  try { const c = JSON.parse(sessionStorage.getItem('alixLite.unit') || 'null'); if (c) current = c; } catch (e) { /* ignore */ }
  document.querySelectorAll('#modeSwitch button').forEach(b => { b.dataset.tip = b.title; b.onclick = () => setMode(b.dataset.mode); });
  document.querySelectorAll('[data-stop]').forEach(b => b.onclick = () => stop(b.dataset.stop));
  document.getElementById('roleChip').onclick = toggleRole;
  document.getElementById('fixtureBtn').onclick = pickFixture;
  document.getElementById('themeBtn').onclick = () => applyTheme(document.documentElement.dataset.theme === 'dark' ? 'light' : 'dark');
  // the dialog backdrop starts below the app bar, whatever its height, so STOP stays reachable
  const bar = document.querySelector('.app-bar');
  new ResizeObserver(() => document.documentElement.style.setProperty('--appbar-h', `${bar.offsetHeight}px`)).observe(bar);
  store.on(renderBar);
  try { await refreshState(); }
  catch (e) { setStatus('Lite is not reachable.'); showError(e); }
  window.addEventListener('hashchange', route);
  await route();
  setInterval(() => refreshState().catch(() => setStatus('Lost contact with Lite. Is it still running?')), 1000);
}

start();
