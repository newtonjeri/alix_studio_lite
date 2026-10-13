// Tune + Simulation (R2, R6, R11, R12; decisions D24 and D36).
//
// The page's processing controls are the region's tuning settings on the server, one undo step per
// change: (1) Trim (idle stretches, plus the rows marked with Trim Selected Region), (2) Mean Smooth
// with its window (41 lines by default, R11's Standard), (3) Scale regions (time %). Added in the
// page's style: the optimiser (Default or Time-optimal), the total time (the recorded pace, a target,
// or as fast as safe), and a result strip with the limits L1–L6. The plot shows the real recording
// (the page's drag-select and playhead work on its rows); the simulation plays the tuned file.
// "Simulation Looks Good" keeps the tuned file as the version to upload, only if every limit passes.

import { get, post, say } from './api.js';
import { ctx, el, fmtS, loadUnit, step, tell, unitUrl, watchJob } from './ctx.js';
import { invalidate, sim, showFile } from './sim.js';

let unit = null;               // the server's view of the arm step on screen
let syncing = false;           // writing the server's settings into the page's controls
let pushTimer = 0;
let plotP = null;
let tuning = null;             // the running tuning job, if any
let track = { key: null, file: null, data: null };   // the recording on the plot

const pane = () => document.getElementById('stage-optimize');
const $ = (id) => document.getElementById(id);
const settingsOf = (u) => u.settings_view.settings;

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

// ── controls added to the page

function addControls() {
  const card = pane()?.querySelector('.processing-card');
  if (!card || $('liteOptimiser')) return;
  const row = el('div', { class: 'lite-opt-row' },
    el('div', { class: 'field' }, el('label', { for: 'liteOptimiser', text: 'Optimiser' }),
      el('select', { id: 'liteOptimiser' },
        el('option', { value: 'default', text: 'Default (prototype algorithm)' }),
        el('option', { value: 'toppra', text: 'Time-optimal (TOPP-RA)' }))),
    el('div', { class: 'field' }, el('label', { for: 'liteTimeMode', text: 'Total time' }),
      el('select', { id: 'liteTimeMode' },
        el('option', { value: 'natural', text: 'Keep the recorded pace' }),
        el('option', { value: 'target', text: 'Target time' }),
        el('option', { value: 'fastest', text: 'As fast as safe' }))),
    el('div', { class: 'field' }, el('label', { for: 'liteTarget', text: 'Target time (s)' }),
      el('input', { id: 'liteTarget', type: 'number', min: '0.05', max: '600', step: '0.01' })),
    el('div', { class: 'lite-history' },
      el('button', { class: 'btn light', id: 'liteUndo', text: '↶ Undo', onclick: () => history('undo') }),
      el('button', { class: 'btn light', id: 'liteRedo', text: '↷ Redo', onclick: () => history('redo') })));
  card.append(row);
  const cuts = el('div', { id: 'liteCuts', class: 'lite-cuts' });
  pane().querySelector('.manual-scale-card')?.append(cuts);
  const res = el('div', { id: 'liteResults', class: 'lite-results' });
  pane().querySelector('.manual-scale-card')?.after(res);
}

function renderCuts() {
  const box = $('liteCuts');
  if (!box) return;
  box.replaceChildren(manualTrimRegions.length
    ? el('div', { class: 'lite-cut-list' }, el('b', { text: 'Rows removed by Trim: ' }),
      manualTrimRegions.map((c, i) => el('span', { class: 'chip' }, `rows ${c.start}–${c.end} `,
        el('button', { class: 'lite-chip-x', 'aria-label': `Keep rows ${c.start} to ${c.end}`, text: '✕',
          onclick: () => { manualTrimRegions.splice(i, 1); renderCuts(); queuePush('Trim region removed'); } }))))
    : '');
}

// ── page controls <-> server settings

function writeControls(s) {
  syncing = true;
  try {
    $('procTrim').checked = !!s.trim;
    $('procSmooth').checked = !!s.smooth;
    $('meanSmoothWindow').value = s.smooth_window;
    $('procScale').checked = !!s.scale;
    manualScaleRegions = (s.segments || []).map(r => ({ start: r.start, end: r.end, scale: r.pct }));
    manualTrimRegions = (s.cuts || []).map(r => ({ start: r.start, end: r.end }));
    $('liteOptimiser').value = s.optimiser;
    $('liteOptimiser').querySelector('[value="toppra"]').disabled = !unit.toppra;
    $('liteTimeMode').value = s.time_mode;
    $('liteTarget').value = s.target_s ?? (unit.target_s || Number($('targetSec')?.value) || '');
    $('liteTarget').disabled = s.time_mode !== 'target';
    $('meanSmoothWindow').disabled = s.optimiser === 'toppra';
    window.renderManualScaleRegions();
    renderCuts();
  } finally { syncing = false; }
}

function readControls() {
  const mode = $('liteTimeMode').value;
  const target = Number($('liteTarget').value);
  return {
    trim: $('procTrim').checked, smooth: $('procSmooth').checked,
    smooth_window: Math.max(1, Math.min(501, Number($('meanSmoothWindow').value) || 41)),
    scale: $('procScale').checked,
    segments: manualScaleRegions.map(r => ({ start: r.start, end: r.end, pct: r.scale })),
    cuts: manualTrimRegions.map(r => ({ start: r.start, end: r.end })),
    optimiser: $('liteOptimiser').value, time_mode: mode,
    target_s: mode === 'target' && target > 0 ? target : settingsOf(unit).target_s,
  };
}

function queuePush(label) {
  if (syncing || !unit) return;
  clearTimeout(pushTimer);
  pushTimer = setTimeout(() => push(label), 250);
}

async function push(label) {
  const now = readControls(), was = settingsOf(unit);
  const changes = Object.fromEntries(Object.entries(now).filter(([k, v]) => JSON.stringify(v) !== JSON.stringify(was[k])));
  if (!Object.keys(changes).length) return;
  try {
    unit = await post(unitUrl(unit.key, 'settings'), { changes, label });
    ctx.units[unit.key] = unit;
  } catch (err) { tell(err); }
  writeControls(settingsOf(unit));
  showResults();
  drawPlot().catch(() => {});
}

async function history(dir) {
  if (!unit) return;
  try { unit = await post(unitUrl(unit.key, dir), {}); } catch (err) { tell(err); return; }
  writeControls(settingsOf(unit));
  showResults();
  drawPlot().catch(() => {});
}

// ── results

const CHECK_ORDER = ['L1', 'L2', 'L3', 'L4', 'L5', 'L6'];

function fmtCheck(c) {
  if (c.value == null) return c.detail || '';
  const v = Math.abs(c.value) >= 100 ? Math.round(c.value) : Number(c.value).toFixed(2);
  return `${v}${c.unit ? ' ' + c.unit : ''}${c.limit != null ? ` (limit ${c.limit}${c.unit ? ' ' + c.unit : ''})` : ''}`;
}

function showResults(job = null) {
  const box = $('liteResults');
  if (!box || !unit) return;
  const cand = unit.candidate, r = cand && cand.result;
  const raw = unit.recordings.find(x => x.file === unit.active);
  const head = el('div', { class: 'lite-results-head' }, el('span', { class: 'badge violet', text: 'RESULT' }));
  if (job && job.state === 'running') {
    head.append(el('b', { text: ` Tuning… ${job.stage || ''}` }));
    box.replaceChildren(head);
    return;
  }
  if (!r) {
    head.append(el('b', { text: raw ? ' Not tuned yet: press ▶ Re-run Simulation.' : ' No recording yet: record the region or load a file.' }));
    box.replaceChildren(head);
    return;
  }
  const ready = r.uploadable && unit.fresh;
  head.append(el('b', { class: ready ? 'lite-ok' : 'lite-bad',
    text: !unit.fresh ? ' Settings changed: press ▶ Re-run Simulation to update.' : r.uploadable ? ' Ready for Real Test ✓' : ' Not ready for the robot' }));
  const opt = r.optimiser === 'toppra' ? 'Time-optimal' : 'Default';
  const grid = el('div', { class: 'lite-results-grid' },
    el('div', {}, el('span', { text: 'Recording' }), el('b', { text: fmtS(r.recorded_s) }), el('small', { text: raw ? `${raw.lines.toLocaleString()} rows` : '' })),
    el('div', {}, el('span', { text: 'Tuned' }), el('b', { text: fmtS(r.achieved_s) }), el('small', { text: `${r.lines.toLocaleString()} rows · ${opt}` })),
    el('div', {}, el('span', { text: 'Fastest safe' }), el('b', { text: fmtS(r.t_min_s) }),
      el('small', { text: (r.binding && r.binding[0] && r.binding[0].text) || 'within every limit' })));
  if (r.target_s != null && !r.met) {
    grid.append(el('div', {}, el('span', { text: 'Target' }), el('b', { text: fmtS(r.target_s) }),
      el('button', { class: 'btn light', text: `Use ${r.t_min_s.toFixed(2)} s`, onclick: () => useTime(r.t_min_s) })));
  }
  const checks = (cand.checks && cand.checks.checks) || [];
  const chips = el('div', { class: 'lite-limits' }, CHECK_ORDER.map((id) => {
    const c = checks.find(x => x.id === id);
    if (!c) return null;
    const cls = c.passed === false ? 'bad' : c.passed ? 'ok' : 'na';
    return el('span', { class: `lite-limit ${cls}`, title: c.detail || '' },
      `${c.passed === false ? '✕' : c.passed ? '✓' : '–'} ${id} ${c.name}: ${fmtCheck(c)}`);
  }), checks.filter(c => !c.hard && c.passed === false).map(c => el('span', { class: 'lite-limit warn', title: c.detail || '' }, `! ${c.id} ${c.name}`)));
  const msg = el('p', { class: 'mini-note', text: r.message });
  const hints = (r.hints || []).length ? el('ul', { class: 'lite-hints' }, r.hints.map(h => el('li', { text: h }))) : null;
  const cut = unit.candidate && unit.candidate.recipe && unit.candidate.recipe.cut;
  const bridges = cut && cut.bridges && cut.bridges.length
    ? el('p', { class: 'mini-note', text: `Trim removed ${cut.removed_lines.toLocaleString()} rows; ${cut.bridges.map(b => `the gap after row ${b.after_line} is crossed in ${b.seconds.toFixed(2)} s`).join('; ')}. Check it in the simulation.` })
    : null;
  box.replaceChildren(head, grid, chips, msg, hints || '', bridges || '');
}

function useTime(s) {
  $('liteTimeMode').value = 'target';
  $('liteTarget').disabled = false;
  $('liteTarget').value = s.toFixed(2);
  push(`Target ${s.toFixed(2)} s`);
}

// ── the plot of the recording

async function drawPlot() {
  const host = $('trajectoryPlotRegion');
  if (!host || !unit || !unit.active) return;
  await loadPlotly();
  if (track.key !== unit.key || track.file !== unit.active) {
    track = { key: unit.key, file: unit.active, data: await get(`${unitUrl(unit.key, 'track')}?which=raw`) };
  }
  const tr = track.data;
  activeTrajectoryRows = tr.lines;
  activeTrajectoryName = (unit.recordings.find(r => r.file === unit.active) || {}).name || activeTrajectoryName;
  window.updateActiveTrajectoryUI();
  window.resetPlotSelection();
  let div = host.querySelector('.lite-plot');
  if (!div) {
    div = el('div', { class: 'lite-plot' });
    host.prepend(div);
    new ResizeObserver(() => { if (div.data) window.Plotly.Plots.resize(div); }).observe(host);
  }
  host.classList.add('lite-plotted');
  const stride = Math.max(1, Math.ceil(tr.lines / 2500));
  const x = [];
  for (let i = 0; i < tr.lines; i += stride) x.push(i + 1);
  const colours = ['--blue', '--orange', '--green', '--violet', '--cyan', '--red', '--muted']
    .map(v => getComputedStyle(document.documentElement).getPropertyValue(v).trim());
  const ink = getComputedStyle(document.documentElement).getPropertyValue('--muted').trim();
  const traces = tr.joints.map((q, j) => ({
    x, y: x.map(i => q[i - 1]), name: `J${j + 1}`, mode: 'lines', line: { width: 1.6, color: colours[j] }, hoverinfo: 'skip',
  }));
  const shapes = manualScaleRegions.map(r => ({ type: 'rect', xref: 'x', yref: 'paper', x0: r.start, x1: r.end, y0: 0, y1: 1,
    fillcolor: colours[3], opacity: 0.1, line: { width: 0 } }))
    .concat(manualTrimRegions.map(r => ({ type: 'rect', xref: 'x', yref: 'paper', x0: r.start, x1: r.end, y0: 0, y1: 1,
      fillcolor: colours[5], opacity: 0.14, line: { width: 0 } })));
  await window.Plotly.react(div, traces, {
    margin: { l: 0, r: 0, t: 26, b: 0 }, paper_bgcolor: 'transparent', plot_bgcolor: 'transparent',
    xaxis: { range: [1, tr.lines], fixedrange: true, showgrid: false, zeroline: false, ticklabelposition: 'inside', color: ink },
    yaxis: { fixedrange: true, ticklabelposition: 'inside', ticksuffix: '°', zeroline: false, gridcolor: ink, gridwidth: 0.3, color: ink },
    legend: { orientation: 'h', x: 0, y: 1.0, yanchor: 'bottom', font: { size: 11, color: ink } },
    shapes, showlegend: true,
  }, { staticPlot: true, responsive: true, displayModeBar: false });
}

// ── tuning and the simulation

async function loadSimulation() {
  const host = $('simulationViewport');
  if (!host || !unit || !unit.active) return;
  const which = unit.candidate ? 'candidate' : 'raw';
  const progress = unit.candidate_extra && unit.candidate_extra.progress;
  try {
    await showFile(host, unit, which, progress);
    window.resetSimulationPlayback(false);
    $('simStatus').textContent = which === 'candidate'
      ? `Tuned trajectory loaded: ${fmtS(sim.lines ? (sim.lines - 1) * 0.005 : 0)}. Press ▶ Start.`
      : 'The recording before tuning. Press ▶ Re-run Simulation to tune it.';
  } catch (err) { $('simStatus').textContent = `The 3D view could not load: ${say(err)}`; }
}

export async function retune() {
  if (!unit) return null;
  if (tuning) return tuning;
  const status = $('simStatus');
  tuning = (async () => {
    try {
      const job = await post(unitUrl(unit.key, 'optimise'), {});
      const done = await watchJob(job, (j) => { showResults(j); if (status) status.textContent = `Tuning: ${j.stage}…`; });
      unit = await loadUnit(unit.key);
      showResults();
      if (done.state !== 'done') { if (status) status.textContent = done.error || 'Tuning stopped.'; tell(Object.assign(new Error(done.error || 'Tuning stopped.'), { hint: done.hint })); return false; }
      invalidate();
      await loadSimulation();
      return true;
    } catch (err) { tell(err); return false; } finally { tuning = null; }
  })();
  return tuning;
}

async function enter() {
  const s = step();
  if (!s || !ctx.pid) return;
  addControls();
  try { unit = await loadUnit(s.key); } catch (err) { tell(err); return; }
  writeControls(settingsOf(unit));
  showResults();
  if (!unit.active) {
    $('simStatus').textContent = 'No recording yet: go back to Drag & Teach or Save & Review.';
    return;
  }
  await drawPlot().catch(tell);
  if (!unit.candidate || !unit.fresh) await retune();
  else await loadSimulation();
}

export function installTune() {
  const pageSetStage = window.setStage;
  window.setStage = async function setStage(s, scroll = true) {
    if (s === 'real' && currentStage === 'optimize' && step() && ctx.pid) {
      const ok = await keep();
      if (!ok) return;
    }
    pageSetStage(s, scroll);
    if (currentStage === 'optimize' && (s === 'optimize' || s === 'simulate')) enter();
    document.dispatchEvent(new CustomEvent('lite:stage', { detail: currentStage }));
  };

  const pageRerun = window.rerunSimulationAndFocus;
  window.rerunSimulationAndFocus = async function rerunSimulationAndFocus() {
    if (unit && (!unit.candidate || !unit.fresh)) { const ok = await retune(); if (!ok) return; }
    pageRerun();
  };

  const pageRender = window.renderManualScaleRegions;
  window.renderManualScaleRegions = function renderManualScaleRegions() {
    pageRender();
    if (!syncing && currentStage === 'optimize') queuePush('Scale regions');
  };

  const pageTrimSel = window.trimSelectedPlotRegion;
  window.trimSelectedPlotRegion = function trimSelectedPlotRegion() {
    pageTrimSel();
    renderCuts();
    queuePush('Trim region');
  };

  const pageApplySel = window.applySelectedPlotRegion;
  window.applySelectedPlotRegion = function applySelectedPlotRegion() {
    const w = Number($('selectedPlotSmooth')?.value);
    pageApplySel();
    if (w && w !== Number($('meanSmoothWindow').value)) {
      window.toast(`Smoothing applies to the whole trajectory (window ${$('meanSmoothWindow').value}); one window per region comes later.`);
    }
  };

  document.addEventListener('change', (e) => {
    if (!pane()?.contains(e.target) || syncing) return;
    const id = e.target.id;
    if (id === 'liteTimeMode') $('liteTarget').disabled = e.target.value !== 'target';
    if (id === 'liteOptimiser') $('meanSmoothWindow').disabled = e.target.value === 'toppra';
    const labels = { procTrim: 'Trim', procSmooth: 'Mean smooth', meanSmoothWindow: 'Smoothing window', procScale: 'Scale',
      liteOptimiser: 'Optimiser', liteTimeMode: 'Total time', liteTarget: 'Target time' };
    if (labels[id]) queuePush(labels[id]);
  });

  document.addEventListener('lite:viewer-closed', () => {   // the 3D view is back from the Trajectory Viewer
    if (currentStage === 'optimize' && unit) loadSimulation();
  });

  const pageTheme = window.toggleTheme;
  window.toggleTheme = function toggleTheme() {
    pageTheme();
    if (currentStage === 'optimize') drawPlot().catch(() => {});
  };
}

// Keeps the tuned file as the version to upload (the page's "Simulation Looks Good → Real Test").
async function keep() {
  if (!unit) return true;
  if (!unit.candidate || !unit.fresh) {
    const ok = await retune();
    if (!ok) return false;
  }
  try {
    unit = await post(unitUrl(unit.key, 'keep'), {});
    ctx.units[unit.key] = unit;
    return true;
  } catch (err) { tell(err); showResults(); return false; }
}
