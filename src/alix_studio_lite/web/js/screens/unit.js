// One region, one step per tab: Teach, Optimise and Run (the 3D view is its own tab, view3d.js).
// Each step shows "what to do now" and exactly one primary button.
import { api, esc, h, store, refreshState, statusChip, chip, armBadge, fmtS, durS, ARM, icon, watchJob, PIPE_ORDER, unitCode } from '../core.js';
import { setStatus, toast, showError, confirmMotion, modal } from '../ui.js';
import { plotTheme, plotConfig, jointTraces, css, onTheme, shadeWindows, plotBox } from '../plots.js';
import { setCurrentUnit, stepStates } from '../app.js';
import { titleCase } from './home.js';

const STEPS = [
  { id: 'teach', title: 'Teach' },
  { id: 'optimise', title: 'Optimise' },
  { id: 'view3d', title: '3D view' },
  { id: 'run', title: 'Run' },
];
const NEXT_TEXT = {
  teach: ['Teach this region', 'Use the existing recording, or prepare the robot, hold the arm and teach it again.'],
  optimise: ['Choose the time and optimise', 'Set the time this region should take, press Optimise, then keep the result.'],
  view3d: ['Check the motion in 3D', 'Watch it against the toilet, the walls, the bin and the other arm, then run the 3D check.'],
  run: ['Run it', 'In Visual sim it plays in the 3D view; in Hardware sim the controller runs it with the arms still; on the real robot the arms move.'],
  done: ['This region is done', 'It ran on the real robot. Choose the next region from Regions.'],
};

let ctx = null;          // { key, unit, step, main, cleanups[] }

export async function render(main, params) {
  const [key, step] = params;
  if (!key) { location.hash = '#/regions'; return; }
  ctx = { key, main, cleanups: [], step: step || 'teach' };
  await load();
  return () => { for (const c of ctx?.cleanups || []) { try { c(); } catch (e) { /* ignore */ } } ctx = null; };
}

async function load() {
  const c = ctx;
  if (!c) return;
  try { c.unit = await api(`unit/${c.key}`); } catch (e) { showError(e); location.hash = '#/regions'; return; }
  if (ctx !== c) return;
  setCurrentUnit(c.unit);
  const st = stepStates(c.unit);
  if (st[c.step] === 'locked') { location.replace(`#/${c.step === 'run' ? 'view3d' : 'teach'}/${c.key}`); return; }
  draw();
}

export function unitHeader(u) {
  const head = h('div.unit-head');
  head.innerHTML = `
    ${u.picture || u.slide_picture ? `<img class="pic" src="/deck/${esc(u.picture || u.slide_picture)}" alt="">`
      : `<div class="pic pic-none" aria-hidden="true">${icon('region', 40)}</div>`}
    <div>
      <div class="t1"><h2>${esc(titleCase(u.name))}</h2>${armBadge(u.arm)}<span class="code">${esc(unitCode(u))}</span>
        ${u.provisional ? chip('standin', '·', 'provisional region', u.note || '') : ''}</div>
      <div class="t2">${statusChip(u.status)}
        ${u.kind === 'lid_seat' ? chip('info', '↻', 'Lid and seat move (not cleaning)') : chip('info', '◇', 'Cleaning')}
        <span class="chip"><span class="g">⌂</span>${esc(u.toilet_state)}</span>
        ${u.pole_mm != null ? `<span class="chip"><span class="g">↕</span>Pole ${u.pole_mm} mm</span>` : ''}
        ${u.hand_preset ? `<span class="chip"><span class="g">✋</span>Hand ${esc(u.hand_preset)}</span>` : ''}</div>
    </div>
    <div class="btn-row"><a class="btn quiet" href="#/regions">${icon('home')} All regions</a></div>`;
  return head;
}

export function todoNow(u, step) {
  const st = stepStates(u);
  const allDone = Object.values(st).every(x => x === 'done');
  let [t1, t2] = allDone && step === 'run' ? NEXT_TEXT.done : NEXT_TEXT[step];
  const i = STEPS.findIndex(x => x.id === step);
  const next = STEPS[i + 1];
  if (!allDone && st[step] === 'done' && next) {
    [t1, t2] = [`${STEPS[i].title}: done`, `Next: the ${next.title} tab. ${NEXT_TEXT[next.id][1]}`];
  }
  return h('div.todo-now', { html: `<span class="g">!</span><div class="txt"><b>${t1}</b><span class="muted">${t2}</span></div>` });
}

function draw() {
  const c = ctx, u = c.unit;
  for (const f of c.cleanups.splice(0)) { try { f(); } catch (e) { /* ignore */ } }
  const page = h('div.page');
  page.append(unitHeader(u), todoNow(u, c.step));
  const body = h('div');
  page.append(body);
  c.main.replaceChildren(page);
  ({ teach: drawTeach, optimise: drawOptimise, run: drawRun })[c.step](body);
}

// ───────────────────────────────────────────────────────────── Teach

function drawTeach(body) {
  const c = ctx, u = c.unit, s = store.state;
  const arm = s.arms[u.arm];
  const robotMode = s.mode !== 'visual';
  const rec = u.recordings[u.recordings.length - 1];
  const panel = h('div.step-panel');

  const side = h('div.side-col');
  side.innerHTML = `<div class="card"><img class="picture-big" src="/deck/${esc(u.picture || u.slide_picture)}" alt="The region on the deck">
      <div class="card-body"><dl class="kv">
        <dt>Arm</dt><dd>${armBadge(u.arm)}</dd>
        <dt>Toilet</dt><dd>${esc(u.toilet_state)}</dd>
        <dt>Pole</dt><dd>${u.pole_mm ?? '–'} mm</dd>
        <dt>Hand</dt><dd>${esc(u.hand_preset || '–')}</dd>
        <dt>Target time</dt><dd>${u.settings_view.settings.target_s ? fmtS(u.settings_view.settings.target_s) : 'not set yet'}</dd></dl>
        <a class="small" href="/deck/${esc(u.slide_picture)}" target="_blank" rel="noopener">View the whole slide</a></div></div>`;
  panel.append(side);

  const main = h('div.stack');
  const card = h('div.card');
  card.innerHTML = `<div class="card-head"><h3>Teach this region</h3>${rec ? chip('ok', '✓', 'Recorded') : chip('todo', '○', 'No recording yet')}</div>`;
  const cb = h('div.card-body');
  card.append(cb);
  main.append(card);
  panel.append(main);
  body.append(panel);

  if (s.motion_block) {
    cb.append(h('div.callout.bad', { html: `<span class="g">✗</span><div class="body"><strong>Motion is blocked.</strong><span>${esc(s.motion_block)}</span></div>` }));
  } else if (!robotMode || !arm.connected) {
    cb.append(h('div.callout.info', { html: `<span class="g">i</span><div class="body"><strong>Teaching needs the robot.</strong>
      <span>${!arm.connected ? `Connect the ${ARM[u.arm]} arm with its chip at the top, then` : 'Choose'} <b>Hardware sim</b> or <b>Real robot</b> at the top. Lite never moves the arm by itself. Or use an existing recording below.</span></div>` }));
  }

  const phaseQ = { lid_closed: 'Is the lid closed?', lid_open: 'Is the lid UP and the seat DOWN?', seat_open: 'Are the lid and the seat both UP?' }[u.phase]
    || `Is the toilet in this state: ${u.toilet_state}?`;
  const q = h('label.check', { html: `<input type="checkbox" id="qState"><span><b>${esc(phaseQ)}</b><br><span class="muted small">Look at the toilet before you start.</span></span>` });
  cb.append(q);

  const prep = h('div.stack');
  prep.innerHTML = `<div class="row"><div style="flex:1"><b>Prepare the robot</b><br><span class="muted small">The pole moves to ${u.pole_mm ?? '–'} mm, then the hand to ${esc(u.hand_preset || 'its preset')}.
      Now ${arm.connected ? `${arm.pole_mm} mm` : 'not connected'}.</span></div></div>`;
  const prepBtn = h('button.btn', { html: `${icon('robot')} Prepare robot` });
  const prepProg = h('div.progress', { hidden: true }, [h('i')]);
  prep.querySelector('.row').append(prepBtn);
  prep.append(prepProg);
  cb.append(prep);

  const teachBox = h('div.stack');
  cb.append(teachBox);
  const act = h('div.action-bar');
  cb.append(act);

  const teaching = arm.teaching;
  if (teaching) {
    const timer = h('div.timer.rec', { html: `<span class="rec-dot"></span><span id="tVal">${fmtS(arm.teach_s || 0)}</span>` });
    const target = u.settings_view.settings.target_s;
    teachBox.append(h('p', { html: `<b>Recording.</b> Guide the arm through the region${target ? `, aiming for about ${fmtS(target)}` : ''}. Press <b>Stop and save</b> when you finish.` }));
    teachBox.append(timer);
    const t0 = Date.now() - 1000 * (arm.teach_s || 0);
    const iv = setInterval(() => { const el = document.getElementById('tVal'); if (el) el.textContent = fmtS((Date.now() - t0) / 1000); }, 200);
    ctx.cleanups.push(() => clearInterval(iv));
    act.append(h('button.btn', { onclick: async () => { await api(`unit/${u.key}/teach/cancel`, {}); await refreshState(); load(); } }, ['Cancel recording']));
    act.append(h('button.btn.primary', { onclick: stopTeach }, ['Stop and save']));
  } else {
    if (rec) {
      const from = { library: `Existing recording ${esc(rec.original || '')}`, upload: `Uploaded file ${esc(rec.original || '')}`,
                     teach: 'Taught on this robot' }[rec.source] || 'Recording';
      teachBox.append(h('div.callout.ok', { html: `<span class="g">✓</span><div class="body"><strong>Using: ${from} · ${fmtS(durS(rec.lines))}</strong>
        <span class="muted">Added ${esc(rec.taken || '')}. ${u.recordings.length > 1 ? `${u.recordings.length} recordings for this region; the newest is used.` : ''}
        The recording is kept exactly as it is; optimising makes new versions from it.</span></div>` }));
    }
    act.append(h('button.btn', { onclick: useFile, html: `${icon('upload')} Use another file…`,
      title: 'Use an existing recording from the library, or a .txt file from this computer.' }));
    const startBtn = h(`button.btn${rec ? '' : '.primary'}`, { onclick: startTeach }, [rec ? 'Teach again' : 'Start teaching']);
    act.append(startBtn);
    if (rec) act.append(h('button.btn.primary', { onclick: () => go('optimise') }, ['Next: Optimise']));
    const upd = () => { startBtn.disabled = !!s.motion_block || !(robotMode && arm.connected && document.getElementById('qState')?.checked); };
    q.addEventListener('change', upd); upd();
    startBtn.title = 'Answer the question above first. Then hold the arm and press Start.';
  }
  prepBtn.disabled = !(robotMode && arm.connected) || teaching || !!s.motion_block;
  prepBtn.onclick = async () => {
    try {
      const job = await api(`unit/${u.key}/prepare`, {});
      prepBtn.disabled = true; prepProg.hidden = false;
      const v = await watchJob(job, (j) => { setStatus(j.stage, true); prepProg.firstChild.style.width = `${j.n ? 100 * j.i / j.n : 30}%`; });
      setStatus(v.state === 'done' ? 'Robot prepared: pole and hand read back.' : v.error || 'Stopped.');
      if (v.state === 'done') toast('Robot prepared.', 'ok'); else if (v.error) toast(v.error, 'bad', v.hint);
      await refreshState(); load();
    } catch (e) { showError(e); }
  };

  async function useFile() {
    let lib;
    try { lib = await api('library'); } catch (e) { showError(e); return; }
    const files = lib.files.filter(f => !f.arm || f.arm === u.arm);
    const body = h('div.stack');
    body.append(h('p', { html: `Choose a recording taught with the <b>${ARM[u.arm]}</b> arm for this fixture, or a <code>.txt</code> file from this computer.
      Lite checks every line and keeps the file unchanged.` }));
    const list = h('div.picker-list');
    let resolve;
    const done = new Promise(r => { resolve = r; });
    const close = (v) => { document.querySelector('#modalRoot .scrim')?.remove(); resolve(v); };
    for (const f of files) {
      list.append(h('button', { onclick: () => close({ library_file: f.file }), html:
        `<span class="arm-badge ${f.arm || u.arm}">${ARM[f.arm || u.arm]}</span><span><b>${esc(f.task || f.file)}</b><br>
         <span class="code">${esc(f.file)}${f.unit ? ' · for ' + esc(f.unit) : ''}</span></span><span class="small muted">${fmtS(durS(f.lines))}</span>` }));
    }
    if (!files.length) list.append(h('div.empty', { text: 'No library recordings for this arm and fixture.' }));
    const fileIn = h('input', { type: 'file', accept: '.txt,text/plain', hidden: true });
    fileIn.onchange = async () => {
      const f = fileIn.files[0];
      if (!f) return;
      close({ name: f.name, content: await f.text() });
    };
    body.append(list, h('div.btn-row', {}, [h('button.btn', { onclick: () => fileIn.click(), html: `${icon('upload')} Upload from this computer…` }), fileIn]));
    modal({ title: 'Use another recording', body, wide: true, actions: [{ label: 'Cancel', value: null }] }).then(() => resolve(null));
    const choice = await done;
    if (!choice) return;
    try {
      setStatus('Checking the recording…', true);
      const r = await api(`unit/${u.key}/import`, choice);
      setStatus(`Using ${r.saved}.`);
      toast('Recording added to this region.', 'ok', 'Optimise it next.');
      await load();
    } catch (e) { setStatus('The recording was not added.'); showError(e); }
  }
  async function startTeach() {
    try { await api(`unit/${u.key}/teach/start`, {}); await refreshState(); load(); toast('Recording. Guide the arm now.', 'info'); }
    catch (e) { showError(e); }
  }
  async function stopTeach() {
    try {
      const r = await api(`unit/${u.key}/teach/stop`, {});
      toast(`Saved: ${r.saved}`, 'ok', 'The recording is kept exactly as taught; edits never change it.');
      await refreshState(); load();
    } catch (e) { showError(e); }
  }
}

// ───────────────────────────────────────────────────────────── Optimise

function drawOptimise(body) {
  const c = ctx, u = c.unit, s = store.state;
  const sv = u.settings_view, set = sv.settings;
  const cand = u.candidate;
  const res = cand?.result;
  const panel = h('div.step-panel');

  // settings
  const left = h('div.side-col');
  const sc = h('div.card');
  sc.innerHTML = `<div class="card-head"><h3>Settings</h3><div class="undo-row">
      <button class="icon-btn" id="undoBtn" title="${esc(sv.undo_label ? 'Undo: ' + sv.undo_label : 'Nothing to undo')}" aria-label="Undo">${icon('undo')}</button>
      <button class="icon-btn" id="redoBtn" title="${esc(sv.redo_label ? 'Redo: ' + sv.redo_label : 'Nothing to redo')}" aria-label="Redo">${icon('redo')}</button></div></div>`;
  const sb = h('div.card-body');
  sc.append(sb);
  left.append(sc);

  const optSeg = h('div.segmented', { role: 'radiogroup', 'aria-label': 'Optimiser' });
  for (const [v, label, tip] of [['default', 'Default', 'Keeps the shape of the recording: removes waiting, smooths, then speeds it up evenly.'],
                                 ['toppra', 'Time-optimal', 'The fastest the arm allows: slows down only where the path needs it.']]) {
    const b = h('button', { role: 'radio', title: tip, onclick: () => change({ optimiser: v }, `Optimiser: ${label}`) }, [label]);
    if (set.optimiser === v) b.classList.add('on');
    if (v === 'toppra' && !s.toppra) { b.disabled = true; b.title = 'Not available on this computer (toppra is not installed).'; }
    optSeg.append(b);
  }
  sb.append(h('label.field', {}, ['Optimiser', optSeg, h('span.help', { text: set.optimiser === 'toppra'
    ? 'Time-optimal: the fastest the arm allows. It slows down only where the path needs it.'
    : 'Default: keeps the shape of the recording. It removes waiting, smooths, then speeds up evenly.' })]));

  const fastest = set.target_s == null;
  const tIn = h('input', { type: 'number', min: '0.5', max: '600', step: '0.5', value: fastest ? '' : String(set.target_s), 'aria-label': 'Target time in seconds', placeholder: '–' });
  tIn.disabled = fastest;
  const ti = h('div.time-input', {}, [
    h('button', { title: '1 s less', onclick: () => bump(-1), disabled: fastest }, ['−']), tIn, h('span.unit', {}, ['s']),
    h('button', { title: '1 s more', onclick: () => bump(1), disabled: fastest }, ['+'])]);
  tIn.addEventListener('change', () => { const v = parseFloat(tIn.value); if (v > 0) change({ target_s: v }, `Target ${v} s`); });
  function bump(d) { const v = Math.max(0.5, (parseFloat(tIn.value) || 10) + d); change({ target_s: v }, `Target ${v} s`); }
  const fastBox = h('label.check', { html: `<input type="checkbox" ${fastest ? 'checked' : ''}><span>As fast as the limits allow</span>` });
  fastBox.querySelector('input').onchange = (e) => change({ target_s: e.target.checked ? null : (res?.t_min_s ? Math.ceil(res.t_min_s) : 30) },
    e.target.checked ? 'Fastest safe time' : 'Set a target time');
  sb.append(h('div.field', {}, [h('span', {}, ['Target time']), ti, fastBox,
    h('span.help.muted.small', { text: 'Seconds the region should take on the robot (one line every 5 ms).' })]));

  if (s.role === 'engineer') {
    const adv = h('details');
    adv.innerHTML = `<summary class="muted">Advanced (engineer)</summary>`;
    const g = h('div.stack', { style: { marginTop: '8px' } });
    const win = h('input', { type: 'number', min: '1', max: '201', step: '2', value: String(set.smooth_window || 41) });
    win.onchange = () => change({ smooth_window: parseInt(win.value, 10) || 41 }, `Smoothing ${win.value} lines`);
    const md = h('input', { type: 'number', min: '1', max: '10', step: '0.5', value: set.max_dev_mm ?? '', placeholder: '10' });
    md.onchange = () => change({ max_dev_mm: md.value ? parseFloat(md.value) : null }, `Path limit ${md.value || 10} mm`);
    g.append(h('label.field', {}, ['Smoothing window (lines, Default only)', win]));
    g.append(h('label.field', {}, ['Path accuracy limit, MAX DEV (mm)', md, h('span.help', { text: 'At most 10 mm. Leave empty for 10 mm.' })]));
    adv.append(g);
    sb.append(adv);
  }
  const optBtn = h('button.btn.big', { html: `${icon('optimise')} Optimise` });
  sb.append(h('div.action-bar', {}, [optBtn]));
  panel.append(left);

  // results
  const right = h('div.stack');
  const rc = h('div.card');
  rc.innerHTML = `<div class="card-head"><h3>Result</h3><span id="resChip"></span></div>`;
  const rb = h('div.card-body');
  rc.append(rb);
  right.append(rc);
  panel.append(right);
  body.append(panel);
  body.append(h('div', { style: { height: '16px' } }), stretchesCard(u, set, change));

  const recLines = u.recordings[u.recordings.length - 1]?.lines || 0;
  const segKey = (a) => JSON.stringify((a || []).map(x => [x.start, x.end, x.pct]));
  const r0 = cand?.recipe || {};
  const stale = res && (res.optimiser !== set.optimiser || (res.target_s ?? null) !== (set.target_s ?? null)
    || segKey(r0.segments) !== segKey(set.segments) || (r0.max_dev_mm ?? null) !== (set.max_dev_mm ?? null)
    || (set.optimiser === 'default' && (r0.options?.smooth_window ?? 41) !== (set.smooth_window ?? 41)));
  if (!res) {
    rb.append(h('div.empty', { text: 'No result yet. Choose the settings and press Optimise.' }));
  } else {
    const ok = res.uploadable;
    rc.querySelector('#resChip').innerHTML = stale ? chip('warn', '△', 'Settings changed: optimise again') : ok ? chip('ok', '✓', 'Passes every limit') : chip('bad', '✗', 'Fails a limit');
    const nums = h('div.nums', { html: `
      <div class="bignum"><span class="v">${fmtS(res.recorded_s, 2)}</span><span class="l">Recorded</span></div>
      <span class="arrow">→</span>
      <div class="bignum"><span class="v">${fmtS(res.achieved_s, 2)}</span><span class="l">Result (${res.achieved_s < res.recorded_s ? '−' : '+'}${Math.abs(Math.round(100 * (res.achieved_s - res.recorded_s) / res.recorded_s))} %)</span></div>
      <div class="bignum"><span class="v">${fmtS(res.t_min_s, 2)}</span><span class="l">Fastest safe</span></div>
      <div class="bignum"><span class="l">Optimiser</span>${chip('info', res.optimiser === 'toppra' ? '⚡' : '≈', res.optimiser === 'toppra' ? 'Time-optimal' : 'Default')}</div>` });
    rb.append(nums);
    rb.append(resultCallout(res));
    rb.append(limitGrid(cand.checks));
    const more = h('details');
    more.innerHTML = `<summary class="muted">More checks (shown, not enforced)</summary>`;
    const mg = h('div.limit-grid', { style: { marginTop: '8px' } });
    for (const ck of cand.checks.checks.filter(x => !x.hard)) mg.append(limitEl(ck));
    more.append(mg);
    if (cand.checks.notes?.length) more.append(h('p.muted.small', { text: cand.checks.notes.join(' ') }));
    rb.append(more);
    const PJ = plotBox(), PU = plotBox({ short: true });
    ctx.cleanups.push(PJ.dispose, PU.dispose);
    const pj = PJ.el, pu = PU.el;
    const showRec = h('label.check', { html: '<input type="checkbox" checked><span>Show the recording (dotted, stretched to the same length so the shapes line up)</span>' });
    rb.append(h('h4', { text: 'Joint angles' }), showRec, PJ.box, h('h4', { text: 'How close to the limits' }), PU.box);
    drawPlots(pj, pu, res, showRec.querySelector('input'));
  }

  // one primary: Optimise (no or stale result) or Keep (a passing result not yet kept)
  const kept = u.versions.length && cand && u.versions[u.versions.length - 1].sha256 === cand.track_sha256;
  if (!res || stale || !res.uploadable) {
    optBtn.classList.add('primary');
  } else {
    const keep = h('button.btn.primary', { onclick: keepVersion }, [kept ? 'Kept: go to Check in 3D' : 'Keep this version']);
    if (kept) keep.onclick = () => go('view3d');
    rb.append(h('div.action-bar', {}, [keep]));
  }
  optBtn.onclick = runOptimise;
  sc.querySelector('#undoBtn').disabled = !sv.can_undo;
  sc.querySelector('#redoBtn').disabled = !sv.can_redo;
  sc.querySelector('#undoBtn').onclick = () => undoRedo('undo');
  sc.querySelector('#redoBtn').onclick = () => undoRedo('redo');
  const onKey = (e) => {
    if (!(e.ctrlKey || e.metaKey) || e.target.matches('input')) return;
    if (e.key.toLowerCase() === 'z' && !e.shiftKey) { e.preventDefault(); if (sv.can_undo) undoRedo('undo'); }
    if (e.key.toLowerCase() === 'y' || (e.key.toLowerCase() === 'z' && e.shiftKey)) { e.preventDefault(); if (sv.can_redo) undoRedo('redo'); }
  };
  document.addEventListener('keydown', onKey);
  ctx.cleanups.push(() => document.removeEventListener('keydown', onKey));

  async function change(changes, label) {
    try { await api(`unit/${u.key}/settings`, { changes, label }); await load(); } catch (e) { showError(e); }
  }
  async function undoRedo(which) {
    try { const r = await api(`unit/${u.key}/${which}`, {}); toast(which === 'undo' ? 'Undone.' : 'Redone.', 'info'); await load(); }
    catch (e) { showError(e); }
  }
  async function runOptimise() {
    optBtn.disabled = true;
    const bar = h('div.progress.indet', {}, [h('i')]);
    rb.prepend(bar);
    try {
      const job = await api(`unit/${u.key}/optimise`, {});
      const v = await watchJob(job, (j) => setStatus(`${j.stage}…`, true));
      if (v.state === 'done') { setStatus(v.result.summary.message); toast(v.result.summary.message, v.result.summary.met ? 'ok' : 'warn'); }
      else { setStatus(v.error || 'Stopped.'); toast(v.error || 'Stopped.', 'bad', v.hint); }
    } catch (e) { showError(e); }
    await load();
  }
  async function keepVersion() {
    try { const v = await api(`unit/${u.key}/save-version`, {}); toast(`Version ${v.version} kept.`, 'ok', 'Next: check it in the 3D view.'); await refreshState(); go('view3d'); }
    catch (e) { showError(e); }
  }
}

// ── parts of the motion: pick a stretch on the recording, then set its time % or speed cap
function stretchesCard(u, set, change) {
  const segs = set.segments || [];
  const toppra = set.optimiser === 'toppra';
  const card = h('details.card');
  if (segs.length) card.open = true;
  card.innerHTML = `<summary class="card-head" style="cursor:pointer"><h3>Speed of parts of the motion</h3>
    ${segs.length ? chip('info', '◆', `${segs.length} part${segs.length > 1 ? 's' : ''} set`) : '<span class="muted small">optional</span>'}</summary>`;
  const b = h('div.card-body');
  card.append(b);
  b.append(h('p', { html: toppra
    ? 'Drag across the recording to pick a part, then set a <b>speed cap</b>: the arm moves at most that share of its speed limits there.'
    : 'Drag across the recording to pick a part, then set its <b>time</b> compared with the rest: 100 % = unchanged, 75 % = faster, 120 % = slower. The changes blend in over a quarter of a second.' }));
  const PB = plotBox({ short: true });
  ctx.cleanups.push(PB.dispose);
  b.append(PB.box);
  const list = h('div.stack');
  b.append(list);
  for (const [i, sg] of segs.entries()) {
    const t0 = (sg.start - 1) * 0.005, t1 = (sg.end - 1) * 0.005;
    list.append(h('div.row', {}, [
      h('span', { html: `<b>${fmtS(t0)} – ${fmtS(t1)}</b> of the recording · ${toppra ? 'speed cap' : 'time'} <b>${sg.pct} %</b>` }),
      h('button.btn.quiet', { onclick: () => change({ segments: segs.filter((_, k) => k !== i) }, `Remove part ${fmtS(t0)}–${fmtS(t1)}`) }, ['Remove'])]));
  }
  (async () => {
    let raw;
    try { raw = await api(`unit/${u.key}/track?which=raw`); } catch (e) { return; }
    const el = PB.el;
    if (!window.Plotly || !el.isConnected) return;
    const shapes = segs.map(sg => ({ type: 'rect', xref: 'x', yref: 'paper', x0: (sg.start - 1) * 0.005, x1: (sg.end - 1) * 0.005, y0: 0, y1: 1,
      fillcolor: css('--brand'), opacity: 0.15, line: { width: 0 } }));
    const annotations = segs.map(sg => ({ x: ((sg.start + sg.end) / 2 - 1) * 0.005, y: 0.03, yref: 'paper', yanchor: 'bottom', showarrow: false,
      text: `${sg.pct} %`, font: { size: 14, color: css('--primary-fill') } }));
    await Plotly.react(el, jointTraces(raw.joints, { name: '(recording)' }), plotTheme({
      dragmode: 'select', selectdirection: 'h', shapes, annotations,
      xaxis: { ...plotTheme().xaxis, title: { text: 'Time in the recording (s)' } },
      yaxis: { ...plotTheme().yaxis, title: { text: 'Joint angle (°)' } } }), { ...plotConfig, modeBarButtonsToAdd: ['select2d'] });
    el.on('plotly_selected', async (ev) => {
      const r = ev?.range?.x;
      if (!r) return;
      const [t0, t1] = [Math.max(0, Math.min(...r)), Math.max(...r)];
      const start = Math.round(t0 / 0.005) + 1, end = Math.min(raw.lines, Math.round(t1 / 0.005) + 1);
      if (end - start < 20) { toast('Pick a longer part (at least 0.1 s).', 'warn'); return; }
      if (segs.some(sg => !(end < sg.start || start > sg.end))) { toast('That part overlaps one already set. Remove it first.', 'warn'); return; }
      const inp = h('input', { type: 'number', min: toppra ? '10' : '25', max: toppra ? '100' : '400', step: '5', value: toppra ? '60' : '80' });
      const body = h('div.stack', {}, [h('p', { html: `Part <b>${fmtS(t0)} – ${fmtS(t1)}</b> of the recording.` }),
        h('label.field', {}, [toppra ? 'Speed cap (% of the speed limits)' : 'Time (% of the recorded time)', inp])]);
      setTimeout(() => inp.focus(), 30);
      const ok = await modal({ title: 'Set the speed of this part', body, actions: [{ label: 'Cancel', value: false }, { label: 'Set', value: true, kind: 'primary' }] });
      Plotly.restyle(el, { selectedpoints: [null] });
      if (!ok) return;
      const pct = Math.max(1, Math.min(1000, parseFloat(inp.value) || 100));
      change({ segments: [...segs, { start, end, pct }].sort((a, b) => a.start - b.start) }, `Part ${fmtS(t0)}–${fmtS(t1)} at ${pct} %`);
    });
  })();
  return card;
}

function resultCallout(res) {
  const b = res.binding?.[0];
  if (!res.uploadable) {
    return h('div.callout.bad', { html: `<span class="g">✗</span><div class="body"><strong>Not ready to keep.</strong><span>${esc(res.message)}</span>
      ${res.hints?.length ? `<span class="muted">${esc(res.hints.join(' '))}</span>` : ''}</div>` });
  }
  if (res.target_s == null) {
    return h('div.callout.ok', { html: `<span class="g">✓</span><div class="body"><strong>Fastest safe time: ${fmtS(res.achieved_s)}</strong>
      <span class="muted">${b ? 'What stops it going faster: ' + esc(b.text) + '.' : ''}</span></div>` });
  }
  if (res.met) {
    return h('div.callout.ok', { html: `<span class="g">✓</span><div class="body"><strong>Target met: ${fmtS(res.achieved_s)}</strong>
      <span class="muted">The fastest safe time is ${fmtS(res.t_min_s)}, so there is room to go faster.</span></div>` });
  }
  const tmin = Math.round(res.t_min_s * 1000) / 1000;                 // exactly the fastest safe line count
  const el = h('div.callout.warn', { html: `<span class="g">△</span><div class="body"><strong>${fmtS(res.target_s, 2)} is too fast for the arm. Closest safe time: ${fmtS(tmin, 2)}</strong>
    <span>${b ? 'Limited by ' + esc(b.text) + '.' : ''} The file you get takes ${fmtS(res.achieved_s, 2)}.</span></div>` });
  const use = h('button.btn', { onclick: async () => {
    try { await api(`unit/${ctx.unit.key}/settings`, { changes: { target_s: tmin }, label: `Use ${fmtS(tmin, 2)}` }); await load(); }
    catch (e) { showError(e); } } }, [`Use ${fmtS(tmin, 2)}`]);
  el.append(use);
  return el;
}

const LIMIT_TEXT = { L1: 'Stays clear of the joint end stops', L2: 'Within the joint speed limits', L3: 'Within 600°/s² acceleration',
  L4: 'No jump over 10° between lines', L5: 'Follows the taught path', L6: 'Starts and ends where you taught' };

function limitEl(ck) {
  const state = ck.passed === true ? 'ok' : ck.passed === false ? (ck.hard ? 'bad' : 'warn') : '';
  const g = ck.passed === true ? '✓' : ck.passed === false ? (ck.hard ? '✗' : '△') : '·';
  const val = ck.value == null ? '' : ck.unit === 'mm' ? `${ck.value} mm` : ck.unit === 'deg' ? `${ck.value}°` : ck.unit === 'deg/s^2' ? `${Math.round(ck.value)}°/s²` : ck.unit === 'x limit' ? `${Math.round(ck.value * 100)} % of limit` : ck.unit === 'm/s' ? `${ck.value} m/s` : '';
  return h(`div.limit.${state}`, { title: ck.detail || '', html: `<span class="g">${g}</span><span class="t"><b>${esc(ck.name)}</b><span>${esc(LIMIT_TEXT[ck.id] || '')}${val ? ' · ' + esc(val) : ''}</span></span>` });
}

function limitGrid(checks) {
  const g = h('div.limit-grid');
  for (const ck of checks.checks.filter(x => x.hard)) g.append(limitEl(ck));
  return g;
}

async function drawPlots(pj, pu, res, showRecBox) {
  const u = ctx.unit;
  let raw, opt;
  try {
    [raw, opt] = await Promise.all([api(`unit/${u.key}/track?which=raw`), api(`unit/${u.key}/track?which=candidate`)]);
  } catch (e) { pj.replaceChildren(h('div.empty', { text: e.message })); return; }
  const bindWin = (res.binding || []).slice(0, 1).map(b => [b.t0_s, b.t1_s]);
  const draw = () => {
    if (!window.Plotly || !pj.isConnected) return;
    const stretch = 0.005 * Math.max(1, opt.lines - 1) / Math.max(1, raw.lines - 1);   // recording drawn over the result's length
    const traces = [...(showRecBox.checked ? jointTraces(raw.joints, { name: '(recording, stretched)', dash: 'dot', opacity: 0.45, show: false, cycle: stretch }) : []),
                    ...jointTraces(opt.joints, { name: '' })];
    Plotly.react(pj, traces, plotTheme({ yaxis: { ...plotTheme().yaxis, title: { text: 'Joint angle (°)' } },
      shapes: shadeWindows(bindWin, css('--warning')) }), plotConfig);
    drawUse(pu, ctx?.unit?.candidate_util);
  };
  showRecBox.addEventListener('change', draw);
  draw();
  ctx.cleanups.push(onTheme(draw));
}

function drawUse(pu, util) {
  if (!util || !util.speed || !pu.isConnected) { pu.replaceChildren(h('div.empty', { text: 'Limit use is shown after the next Optimise.' })); return; }
  const n = util.speed.length, T = (util.lines - 1) * 0.005;
  const x = util.speed.map((_, i) => +(i * T / Math.max(1, n - 1)).toFixed(3));
  const traces = [
    { type: 'scatter', mode: 'lines', x, y: util.speed.map(v => 100 * v), name: 'Speed (fastest joint)', line: { color: css('--j2'), width: 1.6 } },
    { type: 'scatter', mode: 'lines', x, y: util.acceleration.map(v => 100 * v), name: 'Acceleration (highest joint)', line: { color: css('--j3'), width: 1.6 } },
  ];
  Plotly.react(pu, traces, plotTheme({ yaxis: { ...plotTheme().yaxis, title: { text: '% of limit' }, range: [0, 110] },
    shapes: [{ type: 'line', xref: 'paper', x0: 0, x1: 1, y0: 100, y1: 100, line: { color: css('--danger'), width: 1.5, dash: 'dash' } }],
    margin: { l: 56, r: 16, t: 8, b: 44 } }), plotConfig);
}


// ───────────────────────────────────────────────────────────── Test on robot

function drawRun(body) {
  const c = ctx, u = c.unit, s = store.state;
  const arm = s.arms[u.arm];
  const ver = u.versions[u.versions.length - 1];
  const set = u.settings_view.settings;
  const ev = (k) => [...u.results].reverse().find(e => e.kind === k && e.sha256 === ver?.sha256);
  const up = ev('uploaded'), dry = ev('dryrun_ok'), real = ev('real_ok'), verdict = ev('verdict');
  const robotMode = s.mode !== 'visual';
  const grid = h('div.grid2');

  // 1 send
  const c1 = h('div.card');
  c1.innerHTML = `<div class="card-head"><h3>1 · Send to the robot</h3>${up ? chip('ok', '✓', 'On the robot') : chip('todo', '○', 'Not sent')}</div>`;
  const b1 = h('div.card-body');
  c1.append(b1);
  const pid = set.program_id;
  const slot = h('div', { html: `<dl class="kv"><dt>Arm</dt><dd>${armBadge(u.arm)}</dd><dt>Version</dt><dd class="mono">${esc(ver?.version || '–')} · ${fmtS(durS(ver?.lines))}</dd>
      <dt>Program number</dt><dd>${pid ? `<span class="mono">${pid}</span>` : '<span class="muted">not set: an engineer sets it</span>'}</dd><dt>In that program now</dt><dd id="slotNow" class="muted">–</dd></dl>` });
  b1.append(slot);
  if (s.role === 'engineer') {
    const pin = h('input', { type: 'number', min: '1', max: '100', value: pid || '', style: { width: '90px' } });
    pin.onchange = async () => { try { await api(`unit/${u.key}/settings`, { changes: { program_id: parseInt(pin.value, 10) || null }, label: `Program ${pin.value}` }); await load(); } catch (e) { showError(e); } };
    b1.append(h('label.field', {}, ['Program number on the controller (1–100, engineer)', pin]));
  }
  const send = h('button.btn', { html: `${icon('upload')} Send to robot` });
  send.disabled = !robotMode || !arm.connected || !pid;
  b1.append(h('div.action-bar', {}, [send]));
  grid.append(c1);

  // 0 visual sim: plays in the 3D view, no hardware
  if (s.mode === 'visual') {
    const c0 = h('div.card');
    c0.innerHTML = `<div class="card-head"><h3>Visual sim</h3>${chip('info', '▶', 'no hardware')}</div>`;
    const b0 = h('div.card-body');
    b0.append(h('p', { html: 'The robot plays this version in the 3D view, with the toilet and the cubicle walls. Nothing is sent to the robot.' }),
      h('div.action-bar', {}, [h('a.btn.primary', { href: `#/view3d/${u.key}?play=1`, html: `${icon('play')} Play in the 3D view` })]),
      h('p.muted.small', { html: 'To send it to the robot and run it, connect an arm and choose <b>Hardware sim</b> or <b>Real robot</b> at the top.' }));
    c0.append(b0);
    body.append(c0, h('div', { style: { height: '16px' } }));
  }

  // 2 run
  const c2 = h('div.card');
  const isReal = s.mode === 'real';
  c2.innerHTML = `<div class="card-head"><h3>2 · ${isReal ? 'Run on the real robot' : 'Hardware sim run'}</h3>
      ${(isReal ? real : dry) ? chip('ok', '✓', isReal ? 'Ran on the robot' : 'Hardware sim OK') : chip('todo', '○', 'Not run')}</div>`;
  const b2 = h('div.card-body');
  c2.append(b2);
  b2.append(h('p', { html: isReal ? 'The arm moves. Stand clear and keep an E-stop button within reach.'
    : s.mode === 'hwsim' ? 'The controller runs the program in its SIM mode with the arm standing still. Nothing moves.'
    : 'Choose <b>Hardware sim</b> or <b>Real robot</b> at the top to run it on the robot.' }));
  const prog = h('div.progress', { hidden: true }, [h('i')]);
  const status = h('div.muted');
  b2.append(prog, status);
  const runBtn = h(`button.btn${isReal ? '.danger' : ''}`, { html: `${icon('play')} ${isReal ? 'Run on real robot' : 'Hardware sim run'}` });
  runBtn.disabled = !robotMode || !arm.connected || !up || !!s.motion_block;
  b2.append(h('div.action-bar', {}, [runBtn]));
  if (s.motion_block) b2.prepend(h('div.callout.bad', { html: `<span class="g">✗</span><div class="body"><strong>Motion is blocked.</strong><span>${esc(s.motion_block)}</span></div>` }));
  grid.append(c2);

  // 3 verdict
  const c3 = h('div.card');
  c3.innerHTML = `<div class="card-head"><h3>3 · How did it go?</h3>${verdict ? chip(verdict.ok ? 'ok' : 'warn', verdict.ok ? '✓' : '△', verdict.ok ? 'Looks good' : 'Needs work') : chip('todo', '○', 'No answer yet')}</div>`;
  const b3 = h('div.card-body');
  c3.append(b3);
  const vrow = h('div.btn-row');
  for (const [v, label] of [['good', 'Looks good'], ['missed', 'Missed a spot'], ['hit', 'Hit something'], ['other', 'Other']]) {
    const b = h('button.btn', { onclick: async () => {
      try { await api(`unit/${u.key}/verdict`, { verdict: v }); toast('Thank you. The answer is saved with this version.', 'ok'); await load(); }
      catch (e) { showError(e); } } }, [label]);
    b.disabled = !(dry || real);
    vrow.append(b);
  }
  b3.append(h('p.muted', { text: 'Answer after a run. "Missed a spot" or "Hit something" takes you back to editing.' }), vrow);
  grid.append(c3);
  body.append(grid);

  // the single primary: the next action in order
  if (s.mode !== 'visual') { if (!up) send.classList.add('primary'); else if (!(isReal ? real : dry)) runBtn.classList.add('primary'); }

  api(`robot/programs?arm=${u.arm}`).then(r => {
    const p = pid ? r.programs.find(x => x.id === pid) : null;
    const el = document.getElementById('slotNow');
    if (el) el.textContent = !pid ? '–' : p?.name ? `${p.name} (${p.lines} lines, ${p.uploaded})` : 'empty';
  }).catch(() => {});

  send.onclick = async () => {
    try {
      const r = await api(`robot/programs?arm=${u.arm}`);
      const cur = r.programs.find(x => x.id === pid);
      let replace = false;
      if (cur?.name && cur.sha256 !== ver.sha256) {
        replace = await confirmMotion({ title: `Replace program ${pid} on the ${ARM[u.arm]} arm?`, danger: false,
          facts: [['Now in program ' + pid, esc(cur.name)], ['Will hold', esc(`${u.key} ${ver.version}`)], ['Arm', `${ARM[u.arm]} (${esc(arm.ip)})`]],
          action: `Replace program ${pid}` });
        if (!replace) return;
      }
      const job = await api(`unit/${u.key}/upload`, { replace });
      if (job.unchanged) { toast('Already on the robot: nothing to send.', 'ok'); await load(); return; }
      prog.hidden = false;
      const v = await watchJob(job, (j) => { setStatus(j.stage, true); });
      setStatus(v.state === 'done' ? `Sent to program ${pid}; name and size read back.` : v.error || 'Stopped.');
      toast(v.state === 'done' ? 'Sent to the robot.' : (v.error || 'Stopped.'), v.state === 'done' ? 'ok' : 'bad', v.hint || '');
    } catch (e) { showError(e); }
    await load();
  };

  runBtn.onclick = async () => {
    let confirmed = false;
    if (isReal) {
      confirmed = await confirmMotion({
        title: `Run ${titleCase(u.name)} on the ${ARM[u.arm]} arm?`,
        facts: [['Arm', `${ARM[u.arm]} (${esc(arm.ip)})`], ['Mode', 'REAL ROBOT: the arm moves'], ['Region', esc(unitCode(u))],
                ['Program', String(pid)], ['Takes', fmtS(durS(ver.lines))], ['E-stop', `${esc(s.arms.left.estop_text)} (LEFT), ${esc(s.arms.right.estop_text)} (RIGHT)`],
                ['3D check', 'Passed for this version']],
        checklist: [`The toilet is in this state: ${u.toilet_state}.`, 'Nobody is inside the cell.', 'I can reach an E-stop button.'],
        action: 'Run on real robot' });
      if (!confirmed) return;
    }
    try {
      const job = await api(`unit/${u.key}/run`, { confirmed });
      prog.hidden = false; runBtn.disabled = true;
      const v = await watchJob(job, (j) => { status.textContent = `${j.stage}: ${fmtS(j.n ? durS(j.n) * j.i / j.n : 0)} of ${fmtS(durS(j.n))}`; prog.firstChild.style.width = `${j.n ? 100 * j.i / j.n : 0}%`; });
      if (v.state === 'done') toast(isReal ? 'Run finished.' : 'Hardware sim run finished.', 'ok');
      else toast(v.error || 'Stopped before the end.', 'bad', v.hint || 'Nothing restarts by itself.');
    } catch (e) { showError(e); }
    await refreshState(); await load();
  };
}

// the toilet state at the START of a step (operations change it during the step)
function startPhase(u) {
  const t = (u.toilet_state || '').toLowerCase();
  if (t.startsWith('lid closed')) return 'lid_closed';
  if (t.startsWith('lid and seat up') || t.startsWith('seat open')) return 'seat_open';
  if (t.startsWith('lid open')) return 'lid_open';
  return 'lid_closed';
}

function go(step) {
  location.hash = `#/${step}/${ctx.key}`;
}
