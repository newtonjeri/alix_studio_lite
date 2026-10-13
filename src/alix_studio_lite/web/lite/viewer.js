// The Trajectory Viewer: taught .txt files, uploaded from this computer (files or a whole folder)
// or from the built-in sets, replayed in 3D on the 5 ms clock (a file of N lines lasts
// (N - 1) x 5 ms). A LEFT and a RIGHT file play together, the joints run on a plot with a
// playhead, and every limit is checked on each file as taught. A file can then become the
// active recording of the region on screen. Nothing here sends anything to the robot.

import { del, get, post, say } from './api.js';
import { ctx, el, fmtS, step, tell } from './ctx.js';
import { ensure3D, invalidate, scene } from './sim.js';

const CYCLE = 0.005;
const ARMS = ['left', 'right'];
const slots = { left: null, right: null };          // {meta, frames, track, checks}
let box = null, lib = null, phase = 'lid_closed';
let t = 0, playing = false, speed = 1, raf = 0, last = 0;
let plotP = null;

const $ = (id) => document.getElementById(id);
const total = () => Math.max(0, ...ARMS.map(a => (slots[a] ? (slots[a].frames.total_lines - 1) * CYCLE : 0)));
const fmtT = (s) => { const m = Math.floor(s / 60); return `${String(m).padStart(2, '0')}:${(s - m * 60).toFixed(1).padStart(4, '0')}`; };
const ARM_BADGE = { left: 'badge blue', right: 'badge orange' };

function loadPlotly() {
  if (!plotP) {
    plotP = window.Plotly ? Promise.resolve() : new Promise((res, rej) => {
      const s = document.createElement('script');
      s.src = '/vendor/plotly.min.js'; s.onload = res; s.onerror = () => rej(new Error('could not load the plot library'));
      document.head.append(s);
    });
  }
  return plotP;
}

function status(text, cls = '') {
  const e = $('liteViewerStatus');
  if (e) { e.textContent = text; e.className = `statusbar ${cls}`.trim(); }
}

// ── the file list

function fileRow(f) {
  const arm = f.arm
    ? el('span', { class: ARM_BADGE[f.arm], text: f.arm.toUpperCase() })
    : el('span', { class: 'lite-arm-pick' }, el('small', { text: 'Arm?' }),
      ...ARMS.map(a => el('button', { class: 'btn light', text: a.toUpperCase(), onclick: () => setArm(f, a) })));
  return el('div', { class: `lite-file-row${ARMS.some(a => slots[a] && slots[a].meta.id === f.id) ? ' on' : ''}` },
    arm,
    el('div', { class: 'lite-file-name' }, el('b', { text: f.name, title: f.folder ? `${f.folder}/${f.name}` : f.name }),
      el('small', { text: [fmtS(f.seconds), `${Number(f.lines).toLocaleString()} rows`, f.task || f.folder || ''].filter(Boolean).join(' · ') })),
    el('div', { class: 'lite-file-actions' },
      el('button', { class: 'btn green', text: '▶ View', disabled: !f.arm, onclick: () => view(f) }),
      f.source === 'upload' ? el('button', { class: 'btn light', text: '✕', 'aria-label': `Remove ${f.name}`, onclick: () => remove(f) }) : null));
}

function renderList() {
  const box2 = $('liteViewerFiles');
  if (!box2 || !lib) return;
  const mine = el('div', { class: 'lite-file-group' }, el('h4', { text: `Your files (${lib.files.length})` }),
    lib.files.length ? lib.files.map(fileRow)
      : el('p', { class: 'mini-note', text: 'Upload taught .txt files above. They are kept on this computer for every project.' }));
  const sets = lib.sets.map(s => el('details', { class: 'lite-file-group' },
    el('summary', {}, el('b', { text: s.name }), el('small', { text: ` · ${s.files.length} files` })),
    el('p', { class: 'mini-note', text: s.note }), s.files.map(fileRow)));
  box2.replaceChildren(mine, ...sets);
}

async function refreshList() {
  try { lib = await get('/api/library'); renderList(); } catch (err) { tell(err); }
}

async function upload(fileList) {
  const files = [...fileList].filter(f => /\.txt$/i.test(f.name));
  if (!files.length) { status('Choose .txt trajectory files.', 'warn'); return; }
  const refused = [];
  let added = 0;
  for (const [i, f] of files.entries()) {
    status(`Checking ${i + 1} of ${files.length}: ${f.name}…`);
    try {
      lib = await post('/api/library', { name: f.name, content: await f.text(), path: f.webkitRelativePath || f.name });
      added++;
    } catch (err) { refused.push(`${f.name}: ${err.message}`); }
  }
  renderList();
  const noArm = lib.files.filter(f => !f.arm).length;
  status(`${added} file${added === 1 ? '' : 's'} added${refused.length ? `, ${refused.length} refused` : ''}.`
    + (noArm ? ` ${noArm} need${noArm === 1 ? 's' : ''} an arm: choose LEFT or RIGHT on ${noArm === 1 ? 'its' : 'their'} row.` : '')
    + (refused.length ? ` ${refused.join(' · ')}` : ''), refused.length ? 'warn' : 'ok');
}

async function setArm(f, arm) {
  try { lib = await post(`/api/library/${f.id}`, { arm }); renderList(); } catch (err) { tell(err); }
}

async function remove(f) {
  if (!confirm(`Remove ${f.name} from your files?\n\nIt moves to the library's trash folder on this computer.`)) return;
  for (const a of ARMS) if (slots[a] && slots[a].meta.id === f.id) clearSlot(a);
  try { lib = await del(`/api/library/${f.id}`); renderList(); } catch (err) { tell(err); }
}

// ── the player

async function view(f) {
  const side = f.arm;
  status(`Loading ${f.name} on the ${side.toUpperCase()} arm…`);
  try {
    const { viewer, robot } = await ensure3D();
    await viewer.attach($('liteViewer3d'));
    await scene(phase);
    const [frames, track, checks] = await Promise.all([
      get(`/api/library/${f.id}/frames?side=${side}&every=4`),
      get(`/api/library/${f.id}/track`),
      get(`/api/library/${f.id}/checks?side=${side}`)]);
    robot.loadFrames(side, frames);
    slots[side] = { meta: f, frames, track, checks };
    pause();
    t = 0;
    show();
    renderSlots(); renderChecks(); renderList();
    await drawPlot();
    status(`${f.name}: ${fmtS(f.seconds)} on the ${side.toUpperCase()} arm. Press ▶ Play.`, 'ok');
  } catch (err) { status(say(err), 'warn'); }
}

async function clearSlot(side) {
  const { robot, viewer } = await ensure3D();
  robot.clearTrack(side);
  viewer.dirty = true;
  slots[side] = null;
  t = Math.min(t, total());
  renderSlots(); renderChecks(); renderList(); drawPlot();
}

function show() {
  ensure3D().then(({ robot, viewer }) => {
    for (const a of ARMS) {
      const s = slots[a];
      if (s) robot.showTrackLine(Math.min(s.frames.total_lines - 1, Math.round(t / CYCLE)), a);
    }
    viewer.dirty = true;
  });
  const T = total();
  if ($('liteViewerTime')) $('liteViewerTime').textContent = `${fmtT(t)} / ${fmtT(T)}`;
  const sl = $('liteViewerSlider');
  if (sl && document.activeElement !== sl) { sl.max = String(Math.round(T * 1000)); sl.value = String(Math.round(t * 1000)); }
  placePlayhead();
}

function tick(now) {
  if (!playing) return;
  t = Math.min(total(), t + ((now - last) / 1000) * speed);
  last = now;
  show();
  if (t >= total()) { playing = false; status('Finished. Press ↺ Reset or ▶ Play to watch again.', 'ok'); return; }
  raf = requestAnimationFrame(tick);
}

function play() {
  if (!total()) { status('Choose a file with ▶ View first.', 'warn'); return; }
  if (t >= total()) t = 0;
  playing = true;
  last = performance.now();
  cancelAnimationFrame(raf);
  raf = requestAnimationFrame(tick);
  status('Playing at the 5 ms clock' + (speed !== 1 ? ` × ${speed}` : '') + '.', 'ok');
}
function pause() { playing = false; cancelAnimationFrame(raf); }
function reset() { pause(); t = 0; show(); }

function renderSlots() {
  const strip = $('liteViewerSlots');
  if (!strip) return;
  strip.replaceChildren(...ARMS.map(a => {
    const s = slots[a];
    return el('div', { class: `lite-slot ${a}` },
      el('span', { class: ARM_BADGE[a], text: `${a.toUpperCase()} ARM` }),
      s ? el('b', { text: ` ${s.meta.name} · ${fmtS(s.meta.seconds)}` }) : el('span', { class: 'mini-note', text: ' standing pose' }),
      s ? el('button', { class: 'btn light', text: '✕', 'aria-label': `Clear the ${a} arm`, onclick: () => clearSlot(a) }) : null);
  }));
}

const CHECK_IDS = ['L1', 'L2', 'L3', 'L4'];

function useTarget(side) {
  const s = ctx.pid ? step() : null;
  return s && !s.later && s.arm === side ? s : null;
}

function renderChecks() {
  const host = $('liteViewerChecks');
  if (!host) return;
  host.replaceChildren(...ARMS.filter(a => slots[a]).map(a => {
    const { meta, checks } = slots[a];
    const fails = checks.checks.filter(c => c.hard && c.passed === false);
    const chips = el('div', { class: 'lite-limits' }, CHECK_IDS.map(id => {
      const c = checks.checks.find(x => x.id === id);
      if (!c) return null;
      return el('span', { class: `lite-limit ${c.passed === false ? 'bad' : 'ok'}`, title: c.detail || '' },
        `${c.passed === false ? '✕' : '✓'} ${id} ${c.name}${c.detail && c.passed === false ? `: ${c.detail}` : ''}`);
    }));
    const target = useTarget(a);
    return el('div', { class: 'lite-viewer-check' },
      el('div', {}, el('span', { class: ARM_BADGE[a], text: a.toUpperCase() }), el('b', { text: ` ${meta.name}` }),
        el('small', { text: ` · ${fmtS(checks.seconds)} · ${checks.lines.toLocaleString()} rows` })),
      chips,
      el('p', { class: 'mini-note', text: fails.length
        ? `As taught, this file breaks ${fails.map(c => c.id).join(', ')}: tune it in a region before it runs on the robot.`
        : 'As taught, this file stays within every limit (path and end checks apply after tuning).' }),
      target ? el('button', { class: 'btn', text: `Use for ${TASKS[currentTask].label} · ${target.display}`, onclick: () => useFor(a, target) }) : null);
  }));
}

async function useFor(side, target) {
  const f = slots[side].meta;
  try {
    const r = await post(`/api/library/${f.id}/use`, { pid: ctx.pid, key: target.key });
    ctx.units[target.key] = r.unit;
    close();
    window.showView('teach');
    armSelectionLocked = true;
    window.updateArmLockUI();
    window.setStage('review');
    window.toast(`${f.name} is now the active recording of ${target.display}.`);
  } catch (err) { tell(err); }
}

// ── the plot: LEFT solid, RIGHT dashed, on the 5 ms time axis

async function drawPlot() {
  const div = $('liteViewerPlot');
  if (!div) return;
  const loaded = ARMS.filter(a => slots[a]);
  if (!loaded.length) { if (window.Plotly) window.Plotly.purge(div); return; }
  await loadPlotly();
  const colours = ['--blue', '--orange', '--green', '--violet', '--cyan', '--red', '--muted']
    .map(v => getComputedStyle(document.documentElement).getPropertyValue(v).trim());
  const ink = getComputedStyle(document.documentElement).getPropertyValue('--muted').trim();
  const traces = [];
  for (const a of loaded) {
    const tr = slots[a].track;
    const stride = Math.max(1, Math.ceil(tr.lines / 2000));
    const idx = [];
    for (let i = 0; i < tr.lines; i += stride) idx.push(i);
    tr.joints.forEach((q, j) => traces.push({
      x: idx.map(i => i * CYCLE), y: idx.map(i => q[i]), name: `J${j + 1}`, legendgroup: a, mode: 'lines', hoverinfo: 'skip',
      legendgrouptitle: j === 0 ? { text: a === 'left' ? 'LEFT (solid)' : 'RIGHT (dashed)' } : undefined,
      line: { width: 1.4, color: colours[j], dash: a === 'right' ? 'dash' : 'solid' },
    }));
  }
  await window.Plotly.react(div, traces, {
    margin: { l: 44, r: 8, t: 8, b: 30 }, paper_bgcolor: 'transparent', plot_bgcolor: 'transparent',
    xaxis: { range: [0, total()], ticksuffix: ' s', fixedrange: true, color: ink, zeroline: false },
    yaxis: { ticksuffix: '°', fixedrange: true, color: ink, gridcolor: ink, gridwidth: 0.3, zeroline: false },
    legend: { orientation: 'v', x: 1.01, y: 1, font: { size: 11, color: ink } }, showlegend: true,
  }, { staticPlot: true, responsive: true, displayModeBar: false });
  placePlayhead();
}

function placePlayhead() {
  const div = $('liteViewerPlot'), ph = $('liteViewerPlayhead');
  if (!div || !ph || !div._fullLayout || !div._fullLayout.xaxis) { if (ph) ph.style.display = 'none'; return; }
  const xa = div._fullLayout.xaxis;
  ph.style.display = 'block';
  ph.style.left = `${xa._offset + xa.l2p(Math.min(t, total()))}px`;
  ph.style.top = `${div._fullLayout.margin.t}px`;
  ph.style.height = `${xa._length ? div._fullLayout.height - div._fullLayout.margin.t - div._fullLayout.margin.b : 0}px`;
}

// ── the window

function build() {
  const files = el('input', { type: 'file', accept: '.txt,text/plain', multiple: true, hidden: true, onchange: (e) => { upload(e.target.files); e.target.value = ''; } });
  const folder = el('input', { type: 'file', multiple: true, hidden: true, onchange: (e) => { upload(e.target.files); e.target.value = ''; } });
  folder.setAttribute('webkitdirectory', '');
  const speedSel = el('select', { id: 'liteViewerSpeed', 'aria-label': 'Speed', onchange: (e) => { speed = Number(e.target.value); if (playing) play(); } },
    [['0.25', '25 %'], ['0.5', '50 %'], ['1', '100 % (real time)'], ['2', '200 %']].map(([v, n]) => el('option', { value: v, text: n, selected: v === '1' })));
  const phaseSel = el('select', { id: 'liteViewerPhase', 'aria-label': 'Toilet state', onchange: (e) => { phase = e.target.value; scene(phase); } },
    [['lid_closed', 'Lid closed'], ['lid_open', 'Lid open'], ['seat_open', 'Seat open']].map(([v, n]) => el('option', { value: v, text: n })));
  const slider = el('input', { id: 'liteViewerSlider', type: 'range', min: '0', max: '0', step: '5', value: '0',
    oninput: (e) => { pause(); t = Number(e.target.value) / 1000; show(); } });
  box = el('div', { class: 'modal lite-viewer', id: 'liteViewer' },
    el('div', { class: 'modal-card lite-viewer-card', role: 'dialog', 'aria-modal': 'true', 'aria-label': 'Trajectory Viewer' },
      el('div', { class: 'section-head' },
        el('div', {}, el('div', { class: 'eyebrow', text: 'Taught files · nothing is sent to the robot' }),
          el('h3', { text: 'Trajectory Viewer' }),
          el('p', { class: 'mini-note', text: 'Upload taught .txt files, then replay them on the robot model: LEFT and RIGHT together, at the 5 ms clock.' })),
        el('div', { class: 'cta-row' },
          el('label', { class: 'lite-inline-field' }, 'Toilet ', phaseSel),
          el('button', { class: 'btn light', text: '✕ Close', onclick: () => close() }))),
      el('div', { class: 'lite-viewer-body' },
        el('aside', { class: 'lite-viewer-side' },
          el('div', { class: 'cta-row' },
            el('button', { class: 'btn', text: '⇧ Upload .txt files', onclick: () => files.click() }),
            el('button', { class: 'btn outline', text: '⇧ Upload a folder', onclick: () => folder.click() }), files, folder),
          el('div', { id: 'liteViewerStatus', class: 'statusbar', text: 'A LEFT or RIGHT folder, or a name starting L_ or R_, sets the arm.' }),
          el('div', { id: 'liteViewerFiles', class: 'lite-viewer-files' })),
        el('section', { class: 'lite-viewer-main' },
          el('div', { id: 'liteViewerSlots', class: 'lite-viewer-slots' }),
          el('div', { id: 'liteViewer3d', class: 'lite-viewer-3d lite-3d' }),
          el('div', { class: 'lite-viewer-transport' },
            el('button', { class: 'btn green', text: '▶ Play', onclick: play }),
            el('button', { class: 'btn orange', text: 'Ⅱ Pause', onclick: () => { pause(); status('Paused.'); } }),
            el('button', { class: 'btn light', text: '↺ Reset', onclick: reset }),
            speedSel, slider, el('span', { id: 'liteViewerTime', class: 'lite-viewer-time', text: '00:00.0 / 00:00.0' })),
          el('div', { class: 'lite-viewer-plot-wrap' }, el('div', { id: 'liteViewerPlot', class: 'lite-viewer-plot' }),
            el('div', { id: 'liteViewerPlayhead', class: 'lite-viewer-playhead' })),
          el('div', { id: 'liteViewerChecks', class: 'lite-viewer-checks' })))));
  document.body.append(box);
  document.addEventListener('keydown', (e) => { if (e.key === 'Escape' && box.classList.contains('show')) close(); });
}

export async function openViewer() {
  if (!box) build();
  box.classList.add('show');
  document.body.style.overflow = 'hidden';
  invalidate();                                       // the region's simulation reloads after this
  renderSlots(); renderChecks();
  refreshList();
  const { viewer, robot } = await ensure3D();
  await viewer.attach($('liteViewer3d'));
  await scene(phase);
  robot.home();
  for (const a of ARMS) if (slots[a]) robot.loadFrames(a, slots[a].frames);
  show();
  drawPlot().catch(() => {});
}

function close() {
  pause();
  box?.classList.remove('show');
  document.body.style.overflow = '';
  document.dispatchEvent(new CustomEvent('lite:viewer-closed'));
}

export function installViewer() {
  const open = () => openViewer().catch(tell);
  // the Operator Journey, Home, and Save & Review's previous-file card
  const nav = document.querySelector('.navbtn[data-view="project"]');
  nav?.after(el('button', { class: 'navbtn lite-viewer-nav', onclick: open },
    el('span', { class: 'navicon', text: '▶' }), el('span', { text: 'Trajectory Viewer' })));
  document.querySelector('#view-home .hero .cta-row')
    ?.append(el('button', { class: 'btn light big', text: '▶ View Taught Trajectories', onclick: open }));
  document.querySelector('#stage-review .browse-source-card')
    ?.append(el('button', { class: 'btn outline wide-btn lite-viewer-link', text: '▶ Open the Trajectory Viewer', onclick: open }));
}
