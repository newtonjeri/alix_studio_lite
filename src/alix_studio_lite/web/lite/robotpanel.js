// Robot & mode (decision D30), opened from the status pill in the top bar: Visual sim / Hardware
// sim / Real robot, the connection of each arm, each arm's E-stop (IO3) with Recover (R4), and
// its pole and hand as read back. Lite always starts in Visual sim; Real needs a confirmation; an
// E-stop blocks motion on both arms. An engineer can make the stand-in's pole or hand fail to
// arrive, to see that nothing runs when staging fails.

import { get, post } from './api.js';
import { confirmDialog, el, tell } from './ctx.js';

const MODES = [
  ['visual', 'Visual sim', 'The 3D robot moves. No robot commands are sent.'],
  ['hwsim', 'Hardware sim', 'The controllers run programs in their SIM mode: the arms stay still, the pole and hand still move.'],
  ['real', 'Real robot', 'The arms move. Keep the area clear and the E-stop buttons within reach.'],
];
const ESTOP = { CLEAR: 'Clear', ENGAGED: 'E-STOP PRESSED', RECOVERING: 'Released: press Recover', UNKNOWN: 'Unknown: motion blocked', NOT_CONNECTED: 'Not connected' };

let box = null;

async function render() {
  const st = await get('/api/robots/status');
  const body = box.querySelector('.lite-robot-body');
  const modes = el('div', { class: 'lite-mode-choice', role: 'radiogroup', 'aria-label': 'Run mode' },
    MODES.map(([id, name, help]) => el('button', {
      class: `lite-mode-btn${st.mode === id ? ' active' : ''}`, 'data-mode': id, role: 'radio', 'aria-checked': String(st.mode === id),
      onclick: () => setMode(id),
    }, el('b', { text: name }), el('small', { text: help }))));
  const arms = el('div', { class: 'lite-arm-rows' }, ['left', 'right'].map((side) => {
    const a = st.arms[side];
    return el('div', { class: `lite-arm-row ${side}` },
      el('div', {}, el('span', { class: `badge ${side === 'left' ? 'blue' : 'orange'}`, text: `${side.toUpperCase()} RM75` }),
        el('small', { text: ` ${a.ip}` })),
      el('div', { class: 'lite-arm-state' }, el('span', { class: `lite-dot ${a.connected ? 'on' : ''}` }), a.connected ? 'Connected' : 'Not connected'),
      el('div', { class: `lite-estop ${a.estop}`, title: 'E-stop input IO3' }, `E-stop: ${ESTOP[a.estop] || a.estop}`),
      a.connected ? el('small', { class: 'lite-arm-readback', text: `Pole ${a.pole_mm} mm · hand ${a.hand.join(', ')}`
        + (a.faults && a.faults.length ? ` · test fault: ${a.faults.join(' and ')} will not arrive` : '') }) : null,
      el('div', { class: 'cta-row' },
        a.estop === 'RECOVERING' ? el('button', { class: 'btn orange', text: 'Recover', onclick: () => act(`/api/robots/${side}/recover`) }) : null,
        a.connected
          ? el('button', { class: 'btn light', text: 'Disconnect', onclick: () => act(`/api/robots/${side}/disconnect`) })
          : el('button', { class: 'btn', text: 'Connect', onclick: () => act(`/api/robots/${side}/connect`) })));
  }));
  const note = el('div', { class: `statusbar ${st.motion_block ? 'warn' : 'ok'}` },
    st.motion_block || `${st.label}. ${st.backend === 'stand-in' ? 'The robot I/O backend is not built yet: nothing moves.' : ''}`);
  body.replaceChildren(modes, arms, note);
  if (robotControlUnlocked) {
    body.append(el('details', { class: 'advanced' }, el('summary', { text: 'Engineer: E-stop test (stand-in robot)' }),
      el('div', { class: 'advanced-body cta-row' }, ['left', 'right'].flatMap(side => [
        el('button', { class: 'btn outline', text: `Press ${side.toUpperCase()} E-stop`, onclick: () => act(`/api/robots/${side}/estop-test`, { pressed: true }) }),
        el('button', { class: 'btn outline', text: `Release ${side.toUpperCase()}`, onclick: () => act(`/api/robots/${side}/estop-test`, { pressed: false }) }),
      ]))));
    if (st.backend === 'stand-in') {
      body.append(el('details', { class: 'advanced' }, el('summary', { text: 'Engineer: staging failure test (stand-in robot)' }),
        el('div', { class: 'advanced-body' },
          el('p', { class: 'mini-note', text: 'A failing pole stops short; a failing hand closes only half way. Every run, playback '
            + 'and move to a start then ends at the read-back, before the arm moves.' }),
          el('div', { class: 'lite-fault-grid' }, ['left', 'right'].flatMap(side => ['pole', 'hand'].map(part => {
            const on = (st.arms[side].faults || []).includes(part);
            return el('label', { class: 'checkrow' },
              el('input', { type: 'checkbox', checked: on, onchange: (e) => act(`/api/robots/${side}/fault`, { [part]: e.target.checked }) }),
              el('span', { text: `${side.toUpperCase()} ${part} fails to arrive` }));
          }))))));
    }
  }
}

const changed = () => document.dispatchEvent(new CustomEvent('lite:mode'));

async function act(url, body = {}) {
  try { await post(url, body); } catch (err) { tell(err); }
  render().catch(tell);
  changed();
}

async function setMode(mode) {
  try {
    await post('/api/robots/mode', { mode });
  } catch (err) {
    if (err.code === 'confirm') {
      const yes = await confirmDialog({ title: 'Switch to the real robot?', body: 'From now on, Real Test, moves and playback move the physical arms.',
        ok: 'Yes, use the real robot', list: ['The work area is clear of people.', 'Both red E-stop buttons are within reach.'] });
      if (yes) await post('/api/robots/mode', { mode, confirmed: true }).catch(tell);
    } else tell(err);
  }
  render().catch(tell);
  changed();
}

export function openRobotPanel() {
  if (!box) {
    box = el('div', { class: 'modal lite-robot-panel', onclick: (e) => { if (e.target === box) close(); } },
      el('div', { class: 'modal-card accessible-modal' },
        el('div', { class: 'section-head' }, el('div', {}, el('h3', { text: 'Robot & mode' }),
          el('p', { class: 'mini-note', text: 'Choose where the region steps run, and connect the arms.' })),
        el('button', { class: 'btn light', text: '✕', 'aria-label': 'Close', onclick: () => close() })),
        el('div', { class: 'lite-robot-body' })));
    document.body.append(box);
  }
  box.classList.add('show');
  render().catch(tell);
}

function close() { box?.classList.remove('show'); }

export function installRobotPanel() {
  const pill = document.querySelector('.topbar .mode-pill');
  if (pill) {
    pill.setAttribute('role', 'button');
    pill.tabIndex = 0;
    pill.addEventListener('click', openRobotPanel);
    pill.addEventListener('keydown', (e) => { if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); openRobotPanel(); } });
  }
  document.addEventListener('lite:open-robot-panel', openRobotPanel);
  document.addEventListener('keydown', (e) => { if (e.key === 'Escape' && box?.classList.contains('show')) close(); });
}
