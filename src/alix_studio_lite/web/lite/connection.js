// Connection (the Operator Journey's entry below Camera Calibration; the user, 6 Nov): each arm's connection details,
// entered once for a new setup and kept on this computer (server/connection.py): the controller's address and port, the
// UDP port its feed is recorded on, and the input the E-stop is wired to with the level that means pressed. Anyone may
// change them; they apply at the arm's next Connect. Test connection asks each device on its own (the controller, the
// E-stop input, the pole, the hand) and says which one does not answer and why; beside the fields, what the controller
// reports of its own settings, as the vendor's app shows them (Configuration, Communication). The hand's Modbus settings
// are shown, not edited: runner v14's hand.py fixes them.

import { get, post, say } from './api.js';
import { el, tell } from './ctx.js';

const $ = (id) => document.getElementById(id);
const ARMS = ['left', 'right'];
const DEVICES = { controller: 'Controller', estop: 'E-stop input', pole: 'Lift pole', hand: 'Hand', feed: 'Feed (UDP)' };
const LEVELS = { 0: '0 (normally closed)', 1: '1 (normally open)' };   // the input's level while the button is pressed
let box = null;
let page = null;
let busy = false;
const said = { left: '', right: '' };

const fmtTime = (t) => (t ? new Date(t * 1000).toLocaleTimeString() : '');

function field(side, key, label, value, attrs = {}) {
  return el('label', { class: 'field' }, el('span', { text: label }),
    el('input', { id: `liteConn-${side}-${key}`, value: value ?? '', ...attrs }));
}

function select(side, key, label, value, options) {
  return el('label', { class: 'field' }, el('span', { text: label }),
    el('select', { id: `liteConn-${side}-${key}` },
      Object.entries(options).map(([v, t]) => el('option', { value: v, text: t, selected: String(v) === String(value) }))));
}

function values(side) {
  const v = (k) => $(`liteConn-${side}-${k}`)?.value.trim();
  return { arm: side, ip: v('ip'), port: v('port'), feed_port: v('feed_port'), estop_io: v('estop_io'), estop_level: v('estop_level') };
}

function deviceRows(check, live) {
  const rows = (check?.rows || []).map((r) => {
    const state = r.level === 'ok' ? 'ok' : r.level === 'warn' ? 'warn' : 'fail';
    const word = r.device === 'feed' ? (state === 'ok' ? 'Set' : 'Look at it')
      : state === 'fail' ? 'Not answering' : state === 'warn' ? 'Answers (see note)' : 'Answers';
    return el('tr', { class: state },
      el('td', { text: DEVICES[r.device] || r.device }),
      el('td', {}, el('span', { class: `lite-conn-state ${state}`, text: word })),
      el('td', {}, el('div', { text: r.text }), r.hint ? el('div', { class: 'mini-note', text: r.hint }) : null));
  });
  // the readings' own judgement since (the hand and the pole, every few seconds)
  for (const [name, d] of Object.entries(live || {})) {
    if (d && d.ok === false && !(check?.rows || []).some((r) => r.device === name && !r.ok)) {
      rows.push(el('tr', { class: 'fail' }, el('td', { text: DEVICES[name] }),
        el('td', {}, el('span', { class: 'lite-conn-state fail', text: 'Not answering' })),
        el('td', { text: `${d.why || 'no reply'} (since ${fmtTime(d.since)})` })));
    }
  }
  return rows;
}

function reported(read) {
  if (!read) return null;
  const line = (k, v) => (v === undefined || v === null || v === '' ? null : el('div', {}, el('b', { text: `${k}: ` }), String(v)));
  const w = read.wired || {}, f = read.wifi || {}, p = read.push || {}, r = read.tool_rs485 || {}, i = read.info || {};
  return el('div', { class: 'lite-conn-read' },
    el('div', { class: 'eyebrow', text: 'As the controller reports its settings' }),
    line('Wired network (IP Mode Setting)', w.ip ? `${w.ip}, mask ${w.mask}, MAC ${w.mac}` : null),
    line('WiFi', f.ip ? `${f.mode === 'ap' ? 'access point' : f.mode || ''} ${f.ssid || ''}, ${f.ip}, channel ${f.channel ?? '?'}` : null),
    line('Feed push', p.port !== undefined ? `${p.enable ? 'on' : 'off'}, to ${p.ip || '?'}:${p.port}, every ${(p.cycle || 0) * 5} ms` : null),
    line('End power', read.tool_v !== undefined ? `${read.tool_v} V` : null),
    line('End RS485', r.baudrate !== undefined ? `${r.mode === 1 ? 'Modbus RTU' : `mode ${r.mode}`}, ${r.baudrate} baud` : null),
    line('Arm', i.arm_model !== undefined ? `${i.arm_model}, ${i.arm_dof} joints, controller generation ${i.robot_controller_version}` : null));
}

function card(side) {
  const a = page.arms[side], link = page.linked[side], d = page.defaults[side];
  const check = link.check, S = side.toUpperCase();
  const opened = link.opened_with;
  const differs = opened && ['ip', 'port', 'feed_port', 'estop_io', 'estop_level'].some((k) => String(opened[k]) !== String(a[k]));
  const wired = check?.read?.wired?.ip;
  return el('section', { class: `lite-conn-card ${side}` },
    el('div', { class: 'lite-conn-head' },
      el('span', { class: `badge ${side === 'left' ? 'blue' : 'orange'}`, text: `${S} RM75` }),
      el('span', { class: `lite-dot ${link.link ? 'on' : ''}` }),
      el('span', { text: link.link ? `Connected at ${opened?.ip}:${opened?.port}` : link.connected ? 'The stand-in robot' : 'Not connected' })),
    el('div', { class: 'lite-conn-fields' },
      field(side, 'ip', 'Controller address (IP)', a.ip, { inputmode: 'decimal', placeholder: d.ip }),
      field(side, 'port', 'Port', a.port, { inputmode: 'numeric', placeholder: d.port }),
      field(side, 'feed_port', 'Feed UDP port (this computer)', a.feed_port, { inputmode: 'numeric', placeholder: d.feed_port }),
      select(side, 'estop_io', 'E-stop input', a.estop_io, { 1: 'IO1', 2: 'IO2', 3: 'IO3', 4: 'IO4' }),
      select(side, 'estop_level', 'Level when pressed', a.estop_level, LEVELS)),
    el('p', { class: 'mini-note', text: `Hand: ${page.hand.port_name}, Modbus ID ${page.hand.device}, ${page.hand.baud} baud `
      + '(fixed by runner v14 and the hand itself; set the hand to them).' }),
    differs ? el('p', { class: 'lite-conn-note warn', text: 'Saved details differ from the ones this connection uses: they apply at its next Connect.' }) : null,
    wired && wired !== a.ip ? el('p', { class: 'lite-conn-note', text: `The controller reports its wired address as ${wired}.` }) : null,
    el('div', { class: 'cta-row' },
      el('button', { class: 'btn', type: 'button', text: 'Save', onclick: () => act(() => save(side)) }),
      el('button', { class: 'btn light', type: 'button', text: 'Defaults', title: `${d.ip}:${d.port}, feed ${d.feed_port}, IO${d.estop_io}`,
        onclick: () => fill(side, d) }),
      wired && wired !== a.ip ? el('button', { class: 'btn light', type: 'button', text: 'Use the reported address',
        onclick: () => fill(side, { ip: wired }) }) : null,
      link.link
        ? el('button', { class: 'btn primary', type: 'button', text: 'Test connection', onclick: () => act(() => test(side)) })
        : el('button', { class: 'btn primary', type: 'button', text: 'Save and connect', onclick: () => act(() => connect(side)) }),
      link.link && differs ? el('button', { class: 'btn', type: 'button', text: 'Connect again', onclick: () => act(() => reconnect(side)) }) : null),
    said[side] ? el('div', { class: 'lite-status', text: said[side] }) : null,
    check || Object.values(link.devices || {}).some((x) => x?.ok === false)
      ? el('div', { class: 'lite-conn-tablebox' },
        el('table', { class: 'lite-conn-devices' },
          el('thead', {}, el('tr', {}, el('th', { text: 'Device' }), el('th', { text: 'Status' }),
            el('th', { text: check ? `Checked at ${fmtTime(check.at)}` : '' }))),
          el('tbody', {}, deviceRows(check, link.devices))))
      : el('p', { class: 'mini-note', text: link.link ? 'Press Test connection to ask each device.' : 'Connect the arm to check its devices.' }),
    reported(check?.read));
}

function fill(side, v) {
  for (const [k, x] of Object.entries(v)) { const i = $(`liteConn-${side}-${k}`); if (i) i.value = x; }
}

async function save(side) {
  const out = await post('/api/robots/connection', values(side));
  said[side] = out.pending ? 'Saved. It applies at this arm\'s next Connect.' : 'Saved.';
}

async function test(side) {
  said[side] = 'Asking each device…';
  render();
  const r = await post(`/api/robots/${side}/check`, {});
  const bad = (r.rows || []).filter((x) => !x.ok).map((x) => DEVICES[x.device] || x.device);
  said[side] = r.ok ? 'Every device answers.' : `Not answering: ${bad.join(', ')}. Robot motion stays blocked until it answers.`;
}

async function connect(side) {
  await save(side);
  said[side] = 'Connecting…';
  render();
  await post(`/api/robots/${side}/connect`, {});
  const st = (await get('/api/robots/status')).arms[side];
  const bad = (st.check?.rows || []).filter((x) => !x.ok).map((x) => DEVICES[x.device] || x.device);
  said[side] = !st.check ? 'Connected.' : st.check.ok ? 'Connected; every device answers.' : `Connected. Not answering: ${bad.join(', ')}.`;
}

async function reconnect(side) {
  await post(`/api/robots/${side}/disconnect`, {});
  await connect(side);
}

async function act(fn) {
  if (busy) return;
  busy = true;
  box.querySelectorAll('button').forEach((b) => { if (!b.classList.contains('lite-conn-close')) b.disabled = true; });
  try {
    await fn();
  } catch (err) {                                     // said on the card it came from (a Connect or a test under way), and told
    for (const s of ARMS) if (said[s].endsWith('…')) said[s] = say(err);
    tell(err);
  }
  busy = false;
  await render();
}

async function render() {
  try { page = await get('/api/robots/connection'); } catch (err) { tell(err); return; }
  const body = $('liteConnBody');
  if (!body) return;
  const focus = document.activeElement?.id;
  body.replaceChildren(...ARMS.map(card),
    el('p', { class: 'mini-note lite-conn-foot', text: `Kept on this computer (${page.file || 'not saved'})`
      + (page.saved_at ? `, saved ${new Date(page.saved_at * 1000).toLocaleString()}.` : '; today\'s values until saved.')
      + ' Every Connect uses them: the controller\'s address and port, the feed\'s port, and the E-stop\'s input.' }));
  if (focus && $(focus)) $(focus).focus();
}

function build() {
  const close = () => { box.classList.remove('show'); document.body.style.overflow = ''; };
  box = el('div', { class: 'modal lite-conn-modal', id: 'liteConnection', onclick: (e) => { if (e.target === box) close(); } },
    el('div', { class: 'modal-card lite-conn-card-outer', role: 'dialog', 'aria-modal': 'true', 'aria-label': 'Connection' },
      el('div', { class: 'section-head' },
        el('div', {}, el('div', { class: 'eyebrow', text: 'Each arm\'s controller' }),
          el('h3', { text: 'Connection' }),
          el('p', { class: 'mini-note', text: 'The details a new setup needs, as the vendor\'s app shows them (Configuration, '
            + 'Communication). Test connection asks the controller, the E-stop input, the pole and the hand each on its own; '
            + 'a hand or a pole that does not answer blocks every motion until it does.' })),
        el('div', { class: 'cta-row' }, el('button', { class: 'btn light lite-conn-close', type: 'button', text: 'Close', onclick: close }))),
      el('div', { class: 'lite-conn-grid', id: 'liteConnBody' })));
  document.body.append(box);
  document.addEventListener('keydown', (e) => {
    if (e.key === 'Escape' && box.classList.contains('show') && !document.querySelector('.lite-confirm')) close();
  });
}

export async function openConnection() {
  if (!box) build();
  box.classList.add('show');
  document.body.style.overflow = 'hidden';
  await render();
}

// The Operator Journey's entry, below Camera Calibration (the user, 6 Nov: "similar to how we added the calibration").
export function installConnection() {
  if (document.querySelector('.lite-conn-nav')) return;
  const after = document.querySelector('.lite-cal-nav') || document.querySelector('.lite-data-nav')
    || document.querySelector('.navbtn[data-view="logs"]');
  if (!after) return;
  after.after(el('button', { class: 'navbtn lite-conn-nav', type: 'button', onclick: openConnection },
    el('span', { class: 'navicon', text: '⇄' }), el('span', { text: 'Connection' })));
  document.addEventListener('lite:open-connection', openConnection);
}
