// Engineer: the robot console. LEFT and RIGHT side by side (50/50), as the control program
// lays them out: connection and E-stop, pole, hand, the program list 1-100, runs, and a log.
import { api, esc, h, store, refreshState, chip, ARM, icon, watchJob } from '../core.js';
import { toast, showError, confirmMotion, setStatus } from '../ui.js';

const FINGERS = ['Little finger', 'Ring finger', 'Middle finger', 'Index finger', 'Thumb bend', 'Thumb rotate'];
const PRESETS = { GRIP: [0, 0, 0, 0, 150, 1000], 'ALL OPEN': [1000, 1000, 1000, 1000, 1000, 1000] };
let timers = [];

export async function render(main) {
  if (store.state?.role !== 'engineer') {
    main.append(h('div.page', {}, [h('div.callout.info', { html: '<span class="g">i</span><div class="body"><strong>Engineer only.</strong><span>Switch to Engineer with the chip at the top right.</span></div>' })]));
    return;
  }
  const page = h('div.page');
  page.append(h('div.page-head', { html: `<div class="titles"><h2>Robot console</h2><p>Manual control for engineers: connection, E-stop input, pole, hand, stored programs and runs.
    Operators work from Regions.</p></div>` }));
  const split = h('div.arms-split');
  for (const side of ['left', 'right']) split.append(armColumn(side));
  page.append(split);
  page.append(bothCard());
  page.append(h('div.callout.info', { html: `<span class="g">i</span><div class="body"><strong>Sequences of programs (the control program's Page 02)</strong>
    <span class="muted">Building cleaning sequences with TOGETHER, RUN FIRST and WAIT OTHER rules, and saving them as workflow JSON, comes with the Program screen in Release 2.</span></div>` }));
  main.append(page);
  const iv = setInterval(() => { for (const f of timers) f(); }, 1500);
  return () => { clearInterval(iv); timers = []; };
}

function armColumn(side) {
  const s = store.state, a = s.arms[side];
  const col = h(`div.arm-col.${side}`);
  const card = h('div.card');
  card.innerHTML = `<div class="card-head"><h3><span class="arm-badge ${side}">${ARM[side]}</span> arm</h3><span class="conn"></span></div>`;
  const b = h('div.card-body');
  card.append(b);
  col.append(card);

  // connection + E-stop
  const ip = h('input', { type: 'text', value: a.ip, 'aria-label': `${ARM[side]} controller address`, style: { width: '150px' } });
  const conn = h('button.btn', {}, [a.connected ? 'Disconnect' : 'Connect']);
  conn.onclick = async () => {
    try {
      if (a.connected) await api('robot/disconnect', { arm: side });
      else { await api('settings', { arm: side, ip: ip.value }); await api('robot/connect', { arm: side, ip: ip.value }); }
      await refreshState(); rerender();
    } catch (e) { showError(e); }
  };
  b.append(h('div.row', {}, [h('label.field', {}, ['Controller address', ip]), conn]));
  const es = h('div.row');
  es.innerHTML = `<div style="flex:1"><b>E-stop input (IO3, mode 5)</b><br><span class="muted small">Pressing either hardware button blocks motion on both arms.</span></div>
    ${a.connected ? chip(a.estop === 'CLEAR' ? 'ok' : a.estop === 'ENGAGED' ? 'bad' : 'warn', a.estop === 'CLEAR' ? '✓' : a.estop === 'ENGAGED' ? '✗' : '△', a.estop_text) : chip('todo', '·', 'Not connected')}`;
  const test = h('button.btn', {}, [a.estop === 'ENGAGED' ? 'Test: release' : 'Test: press']);
  test.disabled = !a.connected;
  test.title = 'Stand-in only: act as if the IO3 button were pressed or released.';
  test.onclick = async () => { try { await api('robot/estop-test', { arm: side, pressed: a.estop !== 'ENGAGED' }); await refreshState(); rerender(); } catch (e) { showError(e); } };
  es.append(test);
  if (a.estop === 'RECOVERING') es.append(h('button.btn', { onclick: async () => { try { await api('robot/recover', { arm: side }); await refreshState(); rerender(); } catch (e) { showError(e); } } }, ['Recover']));
  b.append(es);
  b.append(h('div.muted.small', { html: `Controller mode: <b>${a.controller_mode === 'SIM' ? 'SIM (hardware sim)' : 'REAL'}</b>. Lite sets it for each run and reads it back.` }));

  const motionOk = a.connected && s.mode !== 'visual' && !s.motion_block;
  const why = !a.connected ? 'Connect first.' : s.mode === 'visual' ? 'Choose Hardware sim or Real robot at the top.' : s.motion_block || '';

  // pole
  const pole = h('input', { type: 'number', min: '0', max: '400', step: '5', value: String(a.pole_mm), style: { width: '100px' } });
  const poleBtn = h('button.btn', {}, ['Move pole']);
  poleBtn.disabled = !motionOk; poleBtn.title = why;
  poleBtn.onclick = () => motion(`Move the ${ARM[side]} pole to ${pole.value} mm?`, [['Arm', ARM[side]], ['Pole now', `${a.pole_mm} mm`], ['Pole after', `${pole.value} mm`]],
    () => api('robot/pole', { arm: side, mm: parseFloat(pole.value) }), poleBtn);
  b.append(h('div.row', {}, [h('label.field', {}, ['Pole height (mm)', pole]), poleBtn, h('span.muted.small', { text: `now ${a.pole_mm} mm` })]));

  // hand
  const hb = h('div.hand-grid');
  const vals = [...a.hand];
  const inputs = [];
  FINGERS.forEach((f, i) => {
    const r = h('input', { type: 'range', min: '0', max: '1000', step: '10', value: String(vals[i]), 'aria-label': f });
    const n = h('input', { type: 'number', min: '0', max: '1000', value: String(vals[i]), style: { width: '64px' } });
    r.oninput = () => { n.value = r.value; vals[i] = +r.value; };
    n.onchange = () => { r.value = n.value; vals[i] = +n.value; };
    inputs.push([r, n]);
    hb.append(h('span.small', { text: f }), r, n);
  });
  const presetRow = h('div.btn-row');
  for (const [name, p] of Object.entries(PRESETS)) {
    presetRow.append(h('button.btn', { onclick: () => p.forEach((v, i) => { inputs[i][0].value = inputs[i][1].value = String(v); vals[i] = v; }) }, [name]));
  }
  const handBtn = h('button.btn', {}, ['Apply hand']);
  handBtn.disabled = !motionOk; handBtn.title = why;
  handBtn.onclick = () => motion(`Set the ${ARM[side]} hand?`, [['Arm', ARM[side]], ['Values', vals.join(', ')]], () => api('robot/hand', { arm: side, counts: vals }), handBtn);
  presetRow.append(handBtn);
  b.append(h('details', {}, [h('summary', { html: '<b>Hand</b> <span class="muted small">(0 = closed, 1000 = open)</span>' }), hb, presetRow]));

  // programs
  const tbl = h('div.slots');
  const runSel = { id: null };
  const runBtn = h('button.btn', {}, [s.mode === 'real' ? 'Run selected (real)' : 'Run selected (hardware sim)']);
  runBtn.disabled = true;
  const loadSlots = async () => {
    try {
      const r = await api(`robot/programs?arm=${side}`);
      const rows = r.programs.filter(p => p.name);
      tbl.innerHTML = `<table><thead><tr><th>#</th><th>Program</th><th>Lines</th><th>Sent</th></tr></thead><tbody>
        ${rows.length ? rows.map(p => `<tr data-id="${p.id}" class="${runSel.id === p.id ? 'sel' : ''}" style="cursor:pointer"><td class="mono">${p.id}</td><td>${esc(p.name)}</td><td class="mono">${p.lines}</td><td>${esc(p.uploaded || '')}</td></tr>`).join('')
          : '<tr><td colspan="4" class="muted">No programs stored. Programs 1–100 are free.</td></tr>'}</tbody></table>`;
      tbl.querySelectorAll('tr[data-id]').forEach(tr => tr.onclick = () => { runSel.id = +tr.dataset.id; runBtn.disabled = !motionOk; loadSlots(); });
    } catch (e) { tbl.textContent = e.message; }
  };
  loadSlots();
  runBtn.onclick = async () => {
    let confirmed = false;
    if (s.mode === 'real') {
      confirmed = await confirmMotion({ title: `Run program ${runSel.id} on the ${ARM[side]} arm?`,
        facts: [['Arm', `${ARM[side]} (${esc(a.ip)})`], ['Mode', 'REAL ROBOT'], ['Program', String(runSel.id)]],
        checklist: ['Nobody is inside the cell.', 'I can reach an E-stop button.'], action: 'Run' });
      if (!confirmed) return;
    }
    try { const r = await api('robot/run-program', { arm: side, program_id: runSel.id, confirmed }); follow(r.jobs[0]); }
    catch (e) { showError(e); }
  };
  b.append(h('div.row', {}, [h('b', { text: 'Stored programs' }), h('button.btn.quiet', { onclick: loadSlots, html: `${icon('list')} Refresh` }), runBtn]));
  b.append(tbl);

  // log
  const log = h('div.telemetry', { 'aria-live': 'off' });
  let stick = true;
  log.addEventListener('scroll', () => { stick = log.scrollTop + log.clientHeight >= log.scrollHeight - 8; });
  const loadLog = async () => {
    try { const r = await api(`robot/log?arm=${side}`); log.textContent = r.lines.join('\n') || 'Nothing yet.'; if (stick) log.scrollTop = log.scrollHeight; }
    catch (e) { /* ignore */ }
  };
  timers.push(loadLog); loadLog();
  b.append(h('b', { text: 'Log' }), log);
  return col;
}

function bothCard() {
  const s = store.state;
  const card = h('div.card');
  card.innerHTML = `<div class="card-head"><h3>Both arms together</h3><span class="muted small">Start one stored program on each arm at the same time.</span></div>`;
  const b = h('div.card-body');
  const l = h('input', { type: 'number', min: '1', max: '100', placeholder: 'LEFT #', style: { width: '110px' } });
  const r = h('input', { type: 'number', min: '1', max: '100', placeholder: 'RIGHT #', style: { width: '110px' } });
  const go = h('button.btn', {}, [s.mode === 'real' ? 'Run both (real)' : 'Run both (hardware sim)']);
  const ok = s.arms.left.connected && s.arms.right.connected && s.mode !== 'visual' && !s.motion_block;
  go.disabled = !ok;
  go.title = ok ? '' : 'Connect both arms and choose Hardware sim or Real robot.';
  go.onclick = async () => {
    let confirmed = false;
    if (s.mode === 'real') {
      confirmed = await confirmMotion({ title: 'Run both arms together?', facts: [['LEFT program', l.value], ['RIGHT program', r.value], ['Mode', 'REAL ROBOT']],
        checklist: ['Nobody is inside the cell.', 'I can reach an E-stop button.'], action: 'Run both' });
      if (!confirmed) return;
    }
    try { const res = await api('robot/run-program', { arm: 'both', program_ids: { left: +l.value, right: +r.value }, confirmed }); res.jobs.forEach(follow); }
    catch (e) { showError(e); }
  };
  b.append(h('div.row', {}, [h('label.field', {}, ['LEFT program', l]), h('label.field', {}, ['RIGHT program', r]), go,
    h('span.muted.small', { text: 'STOP ALL at the top stops both.' })]));
  card.append(b);
  return card;
}

async function motion(title, facts, call, btn) {
  const s = store.state;
  if (s.mode === 'real') {
    const ok = await confirmMotion({ title, facts, checklist: ['Nobody is inside the cell.', 'I can reach an E-stop button.'], action: 'Move' });
    if (!ok) return;
  }
  try { btn.disabled = true; follow(await call()); } catch (e) { showError(e); btn.disabled = false; }
}

async function follow(job) {
  const v = await watchJob(job, (j) => setStatus(`${j.title}: ${j.stage}`, true));
  setStatus(v.state === 'done' ? `${v.title}: done.` : `${v.title}: ${v.error || 'stopped'}.`);
  toast(v.state === 'done' ? `${v.title}: done.` : `${v.title}: ${v.error || 'stopped'}`, v.state === 'done' ? 'ok' : 'warn', v.hint || '');
  await refreshState(); rerender();
}

function rerender() {
  if (location.hash.startsWith('#/robot')) window.dispatchEvent(new HashChangeEvent('hashchange'));
}
