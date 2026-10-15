// Tune + Simulation (R2, R6, R11, R12; decisions D24 and D36), laid out as the Trajectory V2
// processor the owner chose for it (14 Oct; docs/reference/ui_v32/Alix_Studio_Lite_Trajectory_V2.html):
// its six steps on V32's Tune stage, opened from the pipeline at the top or with Back / Next.
//
//   1 Source    the region's active recording (another is chosen or loaded in Save & Review)
//   2 Setup     the stages (1) Trim, (2) Mean Smooth, (3) Scale with the processor's parameters, and
//               Lite's optimiser and total time; the region's settings on the server, one undo
//               step per change
//   3 Analyze   the processor's automatic optimisation: review regions with a suggested time % each
//               (optimise/analysis.py, the processor's own rules), applied to the scale regions
//   4 Review    the recording played in 3D beside its plot; rows selected on the plot get their
//               scale (V32's manual scaling regions) or are trimmed
//   5 Generate  tuned on the server with every limit L1–L6 checked; the tuned file kept in the
//               project under its name (V32's Save Tuned File As…) or downloaded as the processor does
//   6 Compare   the tuned file played in 3D beside its plot, the recording's plot below at the same
//               moment; "Simulation Looks Good" keeps it for Real Test, only if every limit passes
//
// The plots are the processor's: a tooltip with the line and every joint, its tools (zoom, pan,
// a box-select of rows, reset, a PNG), and Play / Pause / Restart under them, which run the 3D.

import { get, post, say } from './api.js';
import { confirmDialog, ctx, el, fmtS, loadUnit, step, tell, unitUrl, watchJob } from './ctx.js';
import { elsewhere, invalidate, onPlayhead, release, say3d, showFile, showStanding, sim, unload } from './sim.js';

const CYCLE_S = 0.005;
let unit = null;               // the server's view of the arm step on screen
let syncing = false;           // writing the server's settings into the page's controls
let pushTimer = 0;
let plotP = null;
let tuning = null;             // the running tuning job, if any
let pipeStep = 1;              // the processor's step on screen
const stepOf = new Map();      // each arm step's last step
const tracks = new Map();      // the plotted files, by what they are made from
let analysis = null;           // {key, file, data}: the automatic optimisation of the active recording
const head = { line: 1, row: 1 };   // the moment on the plots

const pane = () => document.getElementById('stage-optimize');
const $ = (id) => document.getElementById(id);
const settingsOf = (u) => u.settings_view.settings;
const rec = () => unit && unit.recordings.find(r => r.file === unit.active);
const pct = (a, b) => (a > 0 ? ((a - b) / a) * 100 : 0);

const STEPS = [null,
  { title: 'Source', sub: 'Recording', note: 'The recording this region tunes. Choose or load another in Save & Review.' },
  { title: 'Setup', sub: 'Parameters', note: 'Choose Trim / Mean Smooth / Scale and their parameters, and the optimiser.' },
  { title: 'Analyze', sub: 'Suggestions', note: 'Review the motion regions and the speed suggested for each.' },
  { title: 'Review', sub: 'Fine-tune', note: 'Play the recording in 3D and on the plot; select rows and give them a scale.' },
  { title: 'Generate', sub: 'Save output', note: 'Tune it with every limit checked, then save the tuned file.' },
  { title: 'Compare', sub: 'Validate', note: 'Play the tuned file beside the recording, then Simulation Looks Good.' },
];

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

function palette() {
  const cs = getComputedStyle(document.documentElement), v = (n) => cs.getPropertyValue(n).trim();
  return { joints: ['--blue', '--orange', '--green', '--violet', '--cyan', '--red', '--muted'].map(v), ink: v('--muted'),
    blue: v('--blue'), green: v('--green'), orange: v('--orange'), violet: v('--violet'), red: v('--red') };
}

const metric = (k, v) => el('div', { class: 'lite-metric' }, el('div', { class: 'k', text: k }), el('div', { class: 'v', text: v }));

// ── the processor's pipeline on V32's Tune stage

function addControls() {
  const p = pane();
  if (!p || $('litePipe')) return;
  const tag = (sel, steps) => { const n = p.querySelector(sel); if (n) n.dataset.liteSteps = steps; };
  tag('.processing-card', '2'); tag('.workspace-50-50', '4 6'); tag('.post-analysis-tuning', '4');
  tag('.tuned-output-card', '5'); tag('.workspace-footer', '6');

  const track = el('div', { class: 'lite-pipe-track', role: 'tablist', 'aria-label': 'Tuning steps' },
    STEPS.slice(1).map((m, i) => el('button', { type: 'button', class: 'lite-pipe-node', 'data-step': String(i + 1), role: 'tab',
      onclick: () => goStep(i + 1) },
    el('span', { class: 'step-num', text: String(i + 1) }),
    el('span', { class: 'step-copy' }, el('span', { class: 'step-title', text: m.title }), el('span', { class: 'step-sub', text: m.sub })))));
  p.querySelector('.tuning-active-file-card')?.after(el('div', { id: 'litePipe', class: 'lite-pipe' },
    el('div', { class: 'lite-pipe-head' }, el('strong', { text: 'Trajectory Optimization Pipeline' }), el('span', { id: 'litePipeSummary' })),
    track));

  // 1 Source
  $('litePipe').after(el('div', { id: 'liteSource', class: 'lite-step-card', 'data-lite-steps': '1' },
    el('div', { class: 'lite-step-title' }, el('h3', { text: '1. Source trajectory' }),
      el('span', { class: 'mini-note', text: 'One line per 5 ms: {"point":[J1,J2,J3,J4,J5,J6,J7]}' })),
    el('div', { id: 'liteSourceMetrics', class: 'lite-metrics' }),
    el('div', { class: 'cta-row' }, el('button', { class: 'btn outline', text: '📂 Choose or load another recording (Save & Review)',
      onclick: () => window.setStage('review') }))));

  // 2 Setup: the processor's parameters under V32's stages, and Lite's optimiser and total time
  const num = (id, label, attrs) => el('div', { class: 'field' }, el('label', { for: id, text: label }), el('input', { id, type: 'number', ...attrs }));
  const setup = el('div', { id: 'liteSetup', class: 'lite-step-card', 'data-lite-steps': '2' },
    el('div', { class: 'lite-step-title' }, el('h3', { text: '2. Processing parameters' }),
      el('span', { class: 'mini-note', text: 'The defaults are the trajectory algorithm used before, with Lite’s smoothing window of 41.' })),
    el('div', { class: 'lite-param-row' },
      num('liteSampleMs', 'Sample interval (ms)', { value: '5', readonly: true, title: 'Lite’s only clock: 5 ms per line' }),
      num('liteIdleWindow', 'Idle rolling window', { min: '3', max: '201', step: '2' }),
      num('liteIdleThreshold', 'Idle threshold (counts/line)', { min: '0', max: '1000', step: '0.1' }),
      num('liteIdleRun', 'Minimum idle run (lines)', { min: '1', max: '20000', step: '1' })),
    el('div', { class: 'lite-hint', text: 'Select any one stage or any combination. When several are on they are always applied in this order: '
      + 'Trim → Mean Smooth → Scale. A stage that is off is skipped completely.' }),
    el('div', { class: 'lite-opt-row' },
      el('div', { class: 'field' }, el('label', { for: 'liteOptimiser', text: 'Optimiser' }),
        el('select', { id: 'liteOptimiser' },
          el('option', { value: 'default', text: 'Default (the trajectory algorithm)' }),
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
        el('button', { class: 'btn light', id: 'liteRedo', text: '↷ Redo', onclick: () => history('redo') }))),
    el('details', {}, el('summary', { text: 'Algorithm used' }),
      el('p', { class: 'mini-note', text: 'Idle trim: the change between consecutive lines of each joint, as a centred rolling mean; a line is '
        + 'idle only when all 7 joints are at or below the threshold for at least the minimum run; the first and last line of each '
        + 'idle stretch are kept and the lines between removed.' }),
      el('p', { class: 'mini-note', text: 'Mean smoothing: a centred moving mean (41 lines by default; mirrored at both ends so the start '
        + 'and the end stay put). Its change to the hand’s path is checked as L5.' }),
      el('p', { class: 'mini-note', text: 'Region scaling: the time of each selected region as a percentage, all 7 joints together. '
        + '100 % = unchanged, 150 % = 50 % longer (slower), 50 % = half the time (faster); a region’s edges are blended over 0.25 s '
        + 'so the speed changes gradually. Time-optimal uses ALIX’s own chain instead, and its region % caps the speed.' })));
  p.querySelector('.processing-card')?.after(setup);

  // 3 Analyze: the processor's automatic optimisation
  $('liteSetup').after(el('div', { id: 'liteAnalysis', class: 'lite-step-card', 'data-lite-steps': '3' },
    el('div', { class: 'lite-step-title' }, el('h3', { text: '3. Automatic speed optimization analysis' }),
      el('span', { class: 'mini-note', text: 'Motion analysis and region-based recommendations' })),
    el('div', { class: 'lite-hint', text: 'The recording’s joint motion is analysed for movement, direction changes and peaks. It is cut into '
      + 'review regions, each with a suggestion: shorter (faster), longer (slower) or unchanged.' }),
    el('div', { class: 'lite-note', text: 'Important: the recording holds joint positions only. The analysis finds motion patterns; it cannot '
      + 'prove cleaning contact or that nothing collides. Smooth continuous regions are therefore possible cleaning or straight-pass '
      + 'candidates for you to review, and every limit is still checked when the file is generated.' }),
    el('div', { class: 'cta-row lite-analysis-actions' },
      el('button', { class: 'btn light', id: 'liteAnalyze', text: '↻ Analyze Again', onclick: () => loadAnalysis(true) }),
      el('div', { class: 'field lite-target-field' }, el('label', { for: 'liteAnalysisTarget', text: 'Optimised time (s)' }),
        el('input', { id: 'liteAnalysisTarget', type: 'number', min: '0.1', max: '600', step: '0.1', placeholder: 'e.g. 20',
          onkeydown: (e) => { if (e.key === 'Enter') loadAnalysis(true, true); } })),
      el('button', { class: 'btn', id: 'liteSuggestTarget', text: 'Suggest for this time', onclick: () => loadAnalysis(true, true) }),
      el('button', { class: 'btn green', id: 'liteApplySuggest', text: 'Apply Selected Suggestions to Scale Regions', onclick: applySuggestions })),
    el('div', { id: 'liteAnalysisStatus', class: 'statusbar' }),
    el('div', { id: 'liteAnalysisMetrics', class: 'lite-metrics' }),
    el('div', { class: 'lite-table-scroll' }, el('table', { class: 'lite-table', id: 'liteSuggestions' },
      el('thead', {}, el('tr', {}, ['Use', '#', 'Rows', 'Original time', 'Motion type', 'Peak guidance', 'Recommendation', 'Scale (%)',
        'Estimated time', 'Reason'].map(h => el('th', { text: h })))), el('tbody')))));

  // 4 Review and 6 Compare: Play / Pause / Restart and the moment under the plot
  // One set of controls (the owner, 15 Oct: Start and Play did the same): the 3D's, named as the
  // processor's; the plot shows the moment and the region.
  for (const [action, text] of [['start', '▶ Play'], ['stop', '❚❚ Pause'], ['reset', '↺ Restart']]) {
    const b = p.querySelector(`#simulationPane button[onclick="simulationControl('${action}')"]`);
    if (b) b.textContent = text;
  }
  p.querySelector('.analysis-pane #plotSelectionStatus')?.before(el('div', { id: 'litePlotBar', class: 'lite-play-bar' },
    el('div', { class: 'lite-play-time' }, el('b', { id: 'litePlotTime', text: '0.000 s' }), el('span', { id: 'litePlotTotal', text: ' / 0.000 s' }),
      el('div', { id: 'liteRegionLabel', class: 'mini-note', text: 'Current region: –' }))));

  // 5 Generate
  const gen = el('div', { id: 'liteGenerate', class: 'lite-step-card', 'data-lite-steps': '5' },
    el('div', { class: 'lite-step-title' }, el('h3', { text: '5. Generate the tuned trajectory' }),
      el('span', { class: 'mini-note', text: 'Only the stages that are on are applied; every limit L1–L6 is checked.' })),
    el('div', { class: 'cta-row' },
      el('button', { class: 'btn green', id: 'liteGenerateBtn', text: '⚙ Confirm & Generate', onclick: generate }),
      el('button', { class: 'btn', id: 'liteDlTxt', text: 'Download tuned TXT', onclick: () => download('txt') }),
      el('button', { class: 'btn outline', id: 'liteDlReport', text: 'Download processing report', onclick: () => download('report') }),
      el('button', { class: 'btn light', id: 'liteDlPng', text: 'Download tuned plot PNG', onclick: () => download('png') })),
    el('div', { class: 'lite-hint', text: 'Save As: a download opens the browser’s Save As window where it can, to choose the folder and the '
      + 'name. Save Tuned File As… below keeps the tuned file in the project, under its name, for Real Test.' }),
    el('div', { id: 'liteGenStatus', class: 'statusbar' }),
    el('div', { id: 'liteGenMetrics', class: 'lite-metrics' }));
  p.querySelector('.tuned-output-card')?.before(gen);
  gen.after(el('div', { id: 'liteResults', class: 'lite-results', 'data-lite-steps': '5' }));
  p.querySelector('.manual-scale-card')?.append(el('div', { id: 'liteCuts', class: 'lite-cuts' }));

  // 6 Compare
  p.querySelector('.workspace-50-50')?.after(el('div', { id: 'liteCompare', class: 'lite-step-card', 'data-lite-steps': '6' },
    el('div', { class: 'lite-step-title' }, el('h3', { text: '6. Recording vs tuned comparison' }),
      el('span', { class: 'mini-note', text: 'Duration, time saved, reduction and the regions changed. The original recording and the tuned file, on the same time axis.' })),
    el('div', { id: 'liteCompareMetrics', class: 'lite-metrics' }),
    el('div', { id: 'liteComparePlot', class: 'lite-compare-plots' },
      el('div', { id: 'liteCompareOriginal', class: 'lite-compare-plot' }), el('div', { id: 'liteCompareTuned', class: 'lite-compare-plot' })),
    el('h4', { text: 'Modified / scaled regions' }),
    el('div', { class: 'lite-table-scroll' }, el('table', { class: 'lite-table', id: 'liteCompareRegions' },
      el('thead', {}, el('tr', {}, ['#', 'Recording rows', 'Scale (%)', 'Rows in', 'Effect'].map(h => el('th', { text: h })))), el('tbody')))));

  // Back / Next, as the processor's steps have
  p.querySelector('.workspace-footer')?.before(el('div', { id: 'litePipeNav', class: 'lite-pipe-nav' },
    el('button', { class: 'btn outline', id: 'litePipeBack', onclick: () => (pipeStep > 1 ? goStep(pipeStep - 1) : window.setStage('review')) }),
    el('div', { class: 'mini-note', id: 'litePipeNote' }),
    el('button', { class: 'btn', id: 'litePipeNext', onclick: () => goStep(pipeStep + 1) })));
}

function unlocked(n) {
  if (n <= 1) return true;
  if (!unit || !unit.active) return false;
  if (n <= 5) return true;
  return !!(unit.candidate && unit.fresh);
}

function renderPipe() {
  const p = pane();
  if (!p) return;
  p.dataset.liteStep = String(pipeStep);
  p.querySelectorAll('.lite-pipe-node').forEach((b) => {
    const n = Number(b.dataset.step), ok = unlocked(n);
    b.classList.toggle('active', n === pipeStep);
    b.classList.toggle('done', n < pipeStep && ok);
    b.classList.toggle('locked', !ok);
    b.disabled = !ok;
    b.setAttribute('aria-selected', String(n === pipeStep));
  });
  if ($('litePipeSummary')) $('litePipeSummary').textContent = `Step ${pipeStep} of 6 · ${STEPS[pipeStep].title}`;
  if ($('litePipeNote')) $('litePipeNote').textContent = STEPS[pipeStep].note;
  const back = $('litePipeBack'), next = $('litePipeNext');
  if (back) back.textContent = pipeStep > 1 ? `← ${STEPS[pipeStep - 1].title}` : '← Save & Review';
  if (next) {
    next.hidden = pipeStep >= 6;
    next.disabled = pipeStep < 6 && !unlocked(pipeStep + 1);
    next.textContent = pipeStep === 5 && !unlocked(6) ? 'Generate first to unlock Compare' : pipeStep < 6 ? `Next: ${STEPS[pipeStep + 1].title} →` : '';
  }
}

async function goStep(n, scroll = true) {
  n = Math.max(1, Math.min(6, Number(n) || 1));
  if (!unlocked(n)) {
    window.toast(n === 6 ? 'Generate first: Compare opens once the tuned file is made from these settings.'
      : 'This region has no recording yet: record it, or load a file in Save & Review.');
    return;
  }
  const was = pipeStep;
  pipeStep = n;
  if (unit) stepOf.set(unit.key, n);
  if (was !== n) release(false);                       // what the 3D played stops with its step
  renderPipe();
  if (scroll) $('litePipe')?.scrollIntoView({ behavior: 'smooth', block: 'start' });
  if (n === 1) renderSource();
  if (n === 3) await loadAnalysis(false);
  if (n === 4) await Promise.all([drawPlot('raw').catch(tell), loadSimulation('raw')]);          // the plot and the 3D together
  if (n === 5) renderGenerate();
  if (n === 6) await Promise.all([drawPlot('candidate').catch(tell), loadSimulation('candidate'), renderCompare().catch(tell)]);
}

// ── 1 Source

function renderSource() {
  const box = $('liteSourceMetrics');
  if (!box) return;
  const r = rec();
  box.replaceChildren(...(r ? [metric('Recording', r.name), metric('Lines', r.lines.toLocaleString()),
    metric('Skipped / malformed lines', '0 (checked when kept)'), metric('Sample interval', '5 ms'),
    metric('Recording time', fmtS(r.seconds)), metric('Taught on', `${String(r.taught_arm || '?').toUpperCase()} arm · ${r.stage ? r.stage.text : 'stage unknown'}`)]
    : [metric('Recording', 'none yet: record the region, or load a file in Save & Review')]));
}

// ── page controls <-> server settings

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

function writeControls(s) {
  syncing = true;
  try {
    $('procTrim').checked = !!s.trim;
    $('procSmooth').checked = !!s.smooth;
    $('meanSmoothWindow').value = s.smooth_window;
    $('procScale').checked = !!s.scale;
    $('liteIdleWindow').value = s.idle_window ?? 11;
    $('liteIdleThreshold').value = s.idle_threshold_md ?? 8;
    $('liteIdleRun').value = s.idle_min_run ?? 20;
    manualScaleRegions = (s.segments || []).map(r => ({ start: r.start, end: r.end, scale: r.pct }));
    manualTrimRegions = (s.cuts || []).map(r => ({ start: r.start, end: r.end }));
    $('liteOptimiser').value = s.optimiser;
    $('liteOptimiser').querySelector('[value="toppra"]').disabled = !unit.toppra;
    $('liteTimeMode').value = s.time_mode;
    $('liteTarget').value = s.target_s ?? (unit.target_s || Number($('targetSec')?.value) || '');
    $('liteTarget').disabled = s.time_mode !== 'target';
    const tp = s.optimiser === 'toppra';
    for (const id of ['meanSmoothWindow', 'liteIdleWindow', 'liteIdleThreshold', 'liteIdleRun']) $(id).disabled = tp;
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
    idle_window: Math.max(3, Math.min(201, Number($('liteIdleWindow').value) || 11)),
    idle_threshold_md: Math.max(0, Math.min(1000, Number($('liteIdleThreshold').value) || 0)),
    idle_min_run: Math.max(1, Math.min(20000, Number($('liteIdleRun').value) || 20)),
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
  afterSettings();
}

async function history(dir) {
  if (!unit) return;
  try { unit = await post(unitUrl(unit.key, dir), {}); } catch (err) { tell(err); return; }
  afterSettings();
}

function afterSettings() {
  writeControls(settingsOf(unit));
  showResults();
  syncTunedOutput();
  renderPipe();
  if (pipeStep === 4) refreshShapes();
  if (pipeStep === 5) renderGenerate();
}

// ── 3 Analyze: the processor's automatic optimisation

// `withTarget`: the suggestions fitted to the optimised time typed (the owner, 15 Oct).
async function loadAnalysis(again, withTarget = false) {
  const status = $('liteAnalysisStatus');
  if (!unit || !unit.active) return;
  const target = withTarget ? Number($('liteAnalysisTarget')?.value) : null;
  if (withTarget && !(target > 0)) { window.toast('Type the optimised time in seconds first, e.g. 20.'); $('liteAnalysisTarget')?.focus(); return; }
  if (again || !analysis || analysis.key !== unit.key || analysis.file !== unit.active) {
    if (status) { status.textContent = 'Analysing the recording…'; status.className = 'statusbar'; }
    try {
      const data = await get(unitUrl(unit.key, 'analysis') + (target ? `?target_s=${target}` : ''));
      analysis = { key: unit.key, file: unit.active, data };
    } catch (err) {
      if (status) { status.textContent = `The analysis could not run: ${say(err)}`; status.className = 'statusbar warn'; }
      return;
    }
  }
  renderAnalysis();
}

function renderAnalysis() {
  const tbody = $('liteSuggestions')?.querySelector('tbody');
  if (!tbody || !analysis) return;
  const regions = analysis.data.regions;
  const badge = (r) => el('span', { class: `badge ${r.scale_pct < 100 ? 'green' : r.scale_pct > 100 ? 'orange' : 'blue'}`, text: r.action });
  tbody.replaceChildren(...regions.map((r, i) => {
    const time = el('td', { text: fmtS(r.original_s * r.scale_pct / 100) });
    const act = el('td', {}, badge(r));
    const input = el('input', { type: 'number', min: '5', max: '400', step: '1', value: String(r.scale_pct), 'aria-label': `Scale of region ${i + 1}`,
      oninput: (e) => {
        r.scale_pct = Math.max(5, Math.min(400, Number(e.target.value) || 100));
        r.action = r.scale_pct < 100 ? 'Shorter / faster' : r.scale_pct > 100 ? 'Longer / slower' : 'Unchanged';
        time.textContent = fmtS(r.original_s * r.scale_pct / 100);
        act.replaceChildren(badge(r));
        analysisMetrics();
      } });
    return el('tr', {},
      el('td', {}, el('input', { type: 'checkbox', checked: r.selected, 'aria-label': `Use region ${i + 1}`,
        onchange: (e) => { r.selected = e.target.checked; analysisMetrics(); } })),
      el('td', { text: String(i + 1) }), el('td', { text: `${r.start.toLocaleString()} – ${r.end.toLocaleString()}` }),
      el('td', { text: fmtS(r.original_s) }), el('td', { text: r.motion_type }), el('td', { text: r.peak_guidance }),
      act, el('td', {}, input), time, el('td', { class: 'lite-reason', text: r.reason }));
  }));
  const status = $('liteAnalysisStatus');
  const t = analysis.data.target;
  if (status) {
    status.textContent = `Automatic analysis of ${analysis.data.file}: ${regions.length} review regions. `
      + (t ? (t.reached ? `Suggestions fitted to ${fmtS(t.target_s)}: every region’s suggestion × ${t.factor}, so each keeps its guidance against the others. `
        : `${fmtS(t.target_s)} cannot be reached with scales of 5–400 %: this is as near as it goes (${fmtS(t.shortest_s)} to ${fmtS(t.longest_s)} for this recording). `)
        + 'The estimate is motion-only: the idle trim can make the tuned file shorter still. ' : '')
      + 'Review the suggested scale values before applying them. Motion-only analysis does not replace the 3D check or the hardware test.';
    status.className = `statusbar ${t && !t.reached ? 'warn' : 'ok'}`;
  }
  analysisMetrics();
}

function analysisMetrics() {
  const box = $('liteAnalysisMetrics');
  if (!box || !analysis) return;
  const rs = analysis.data.regions;
  const original = rs.reduce((a, r) => a + r.original_s, 0);
  const est = rs.reduce((a, r) => a + (r.selected ? r.original_s * r.scale_pct / 100 : r.original_s), 0);
  const saved = original - est;
  box.replaceChildren(metric('Review regions', String(rs.length)),
    metric('Faster / shorter candidates', String(rs.filter(r => r.selected && r.scale_pct < 100).length)),
    metric('Slower / longer candidates', String(rs.filter(r => r.selected && r.scale_pct > 100).length)),
    metric('Motion-only estimated time', fmtS(est)),
    metric('Estimated change', `${saved >= 0 ? '−' : '+'}${Math.abs(saved).toFixed(3)} s (${Math.abs(pct(original, est)).toFixed(1)} %)`));
}

async function applySuggestions() {
  if (!unit || !analysis) return;
  const chosen = analysis.data.regions.filter(r => r.selected);
  if (!chosen.length) { window.toast('No suggestion is ticked: tick at least one region to apply.'); return; }
  if (manualScaleRegions.length) {
    const yes = await confirmDialog({ title: 'Replace the scale regions?', danger: false, ok: 'Replace them',
      body: `The ${manualScaleRegions.length} scale region(s) set now are replaced by the ${chosen.length} selected suggestion(s).` });
    if (!yes) return;
  }
  const changes = { scale: true, segments: chosen.map(r => ({ start: r.start, end: r.end, pct: r.scale_pct })) };
  if (settingsOf(unit).optimiser !== 'default') changes.optimiser = 'default';     // the suggestions are the processor's time %
  try {
    unit = await post(unitUrl(unit.key, 'settings'), { changes, label: 'Automatic suggestions' });
    ctx.units[unit.key] = unit;
  } catch (err) { tell(err); return; }
  afterSettings();
  const status = $('liteAnalysisStatus');
  if (status) {
    status.textContent = `${chosen.length} suggestion(s) copied to the scale regions${changes.optimiser ? ' (with the Default optimiser, '
      + 'whose time % they are)' : ''}. Each can still be changed in Review before generating.`;
    status.className = 'statusbar ok';
  }
}

// ── the plots

function trackData(which) {
  const c = unit.candidate_extra || {};
  const id = which === 'raw' ? `${unit.key}|raw|${unit.active}` : `${unit.key}|candidate|${c.made || ''}|${unit.candidate?.result?.lines || ''}`;
  if (!tracks.has(id)) {
    const p = get(`${unitUrl(unit.key, 'track')}?which=${which}`);
    p.catch(() => tracks.delete(id));
    tracks.set(id, p);
    if (tracks.size > 12) tracks.delete(tracks.keys().next().value);
  }
  return tracks.get(id);
}

// `seconds`: x is the time (line - 1) x 5 ms, over `span` lines (Compare: both files on one axis).
function figure(tr, { selectable = false, title = '', shapes = [], seconds = false, span = null } = {}) {
  const pal = palette();
  const n = tr.lines, stride = n > 15000 ? Math.ceil(n / 15000) : 1;
  const lines = [];
  for (let i = 0; i < n; i += stride) lines.push(i + 1);
  const x = seconds ? lines.map(i => (i - 1) * CYCLE_S) : lines;
  const data = [{ x, y: lines.map(i => tr.joints[0][i - 1]), name: 'Line', mode: 'lines', line: { width: 0, color: pal.ink }, showlegend: false,
    customdata: lines, hovertemplate: seconds ? 'Line %{customdata:,d} · %{x:.3f} s<extra></extra>' : 'Line %{x:,d}<extra></extra>' }]
    .concat(tr.joints.map((q, j) => ({ x, y: lines.map(i => q[i - 1]), name: `J${j + 1}`, mode: 'lines', line: { width: 1.6, color: pal.joints[j] },
      hovertemplate: `J${j + 1}: %{y:.2f}°<extra></extra>` })));
  const layout = {
    title: title ? { text: title, font: { size: 12, color: pal.ink }, x: 0.01, xanchor: 'left', y: 0.99 } : undefined,
    margin: { l: 46, r: 30, t: title ? 30 : 10, b: 36 }, paper_bgcolor: 'transparent', plot_bgcolor: 'transparent',
    hovermode: 'x unified', dragmode: selectable ? 'select' : 'zoom', selectdirection: 'h',
    hoverlabel: { font: { size: 12 } },
    xaxis: seconds
      ? { title: { text: 'Time (s, 5 ms per line)', font: { size: 11, color: pal.ink } }, range: [0, (span ?? n - 1) * CYCLE_S], showgrid: false, zeroline: false, color: pal.ink, hoverformat: '.3f' }
      : { title: { text: 'Line (5 ms each)', font: { size: 11, color: pal.ink } }, range: [1, n], showgrid: false, zeroline: false, color: pal.ink, hoverformat: ',d' },
    yaxis: { ticksuffix: '°', zeroline: false, gridcolor: pal.ink, gridwidth: 0.3, color: pal.ink, automargin: true },
    legend: { orientation: 'h', x: 0, y: 1.0, yanchor: 'bottom', font: { size: 11, color: pal.ink } },
    modebar: { orientation: 'v', bgcolor: 'transparent', color: pal.ink, activecolor: pal.blue },
    shapes, showlegend: true,
  };
  return { data, layout };
}

const PLOT_CONFIG = { responsive: true, scrollZoom: true, displaylogo: false, displayModeBar: true,
  modeBarButtonsToRemove: ['lasso2d', 'autoScale2d', 'toggleSpikelines', 'hoverClosestCartesian', 'hoverCompareCartesian'] };

const rect = (a, b, colour, opacity, edge = false) => ({ type: 'rect', xref: 'x', yref: 'paper', x0: a, x1: b, y0: 0, y1: 1,
  fillcolor: colour, opacity, line: { width: edge ? 1 : 0, color: colour }, layer: 'below' });

// What the recording's plot marks in Review: the analysis' regions, the scale regions, the rows
// trimmed, the selection, and the moment (always last).
function reviewShapes() {
  const pal = palette(), out = [];
  if (analysis && analysis.key === unit.key && analysis.file === unit.active) {
    for (const r of analysis.data.regions) out.push(rect(r.start, r.end, r.scale_pct < 100 ? pal.green : r.scale_pct > 100 ? pal.orange : pal.blue, 0.07));
  }
  for (const r of manualScaleRegions) out.push(rect(r.start, r.end, pal.violet, 0.13));
  for (const r of manualTrimRegions) out.push(rect(r.start, r.end, pal.red, 0.15));
  if (plotSelection && plotSelection.start != null) out.push(rect(plotSelection.start, plotSelection.end, pal.blue, 0.18, true));
  return out;
}

function plotDiv(host, cls) {
  let div = host.querySelector(`:scope > .${cls}`);
  if (!div) {
    div = el('div', { class: cls });
    host.prepend(div);
    new ResizeObserver(() => { if (div.data) window.Plotly.Plots.resize(div); }).observe(host);
  }
  return div;
}

const mainPlot = () => $('trajectoryPlotRegion')?.querySelector(':scope > .lite-plot');

// The analysis pane's plot: the recording (Review: rows are selected on it) or the tuned file (Compare).
async function drawPlot(which = pipeStep === 6 ? 'candidate' : 'raw') {
  const host = $('trajectoryPlotRegion');
  if (!host || !unit || !unit.active) return;
  await loadPlotly();
  const tr = await trackData(which);
  if (which === 'raw') {
    const r = rec();
    activeTrajectoryRows = tr.lines;
    activeTrajectoryName = (r && r.name) || activeTrajectoryName;
    window.updateActiveTrajectoryUI();
  }
  host.classList.add('lite-plotted');
  const div = plotDiv(host, 'lite-plot');
  div.dataset.which = which;
  const title = which === 'raw' ? `Recording — ${(rec() || {}).name || ''}` : `Tuned — ${tunedName()}`;
  const shapes = which === 'raw' ? reviewShapes() : [];
  const fig = figure(tr, { selectable: which === 'raw', title, shapes });
  await window.Plotly.react(div, fig.data, fig.layout, { ...PLOT_CONFIG, toImageButtonOptions: { filename: title.replace(/[^\w.-]+/g, '_') } });
  cursorAt(div, which === 'raw' ? head.row : head.line);
  if (!div.dataset.liteSel) {
    div.dataset.liteSel = '1';
    // the processor's box-select of rows, into the page's own selection (its start and end fields, its line)
    div.on('plotly_selected', (ev) => {
      if (div.dataset.which !== 'raw' || !ev || !ev.range || !ev.range.x) return;
      const a = Math.ceil(Math.min(...ev.range.x)), b = Math.floor(Math.max(...ev.range.x));
      if (b <= a) { window.toast('Select at least two different lines.'); return; }
      window.setPlotSelection(a, b);
      try { window.Plotly.restyle(div, { selectedpoints: [null] }); } catch (_) { /* nothing was dimmed */ }
      refreshShapes();
    });
    // the page's own drag-select on the plot area is not used: these drags are Plotly's
    for (const t of ['pointerdown', 'pointermove', 'pointerup']) div.addEventListener(t, (e) => e.stopPropagation());
  }
  setTotal();
}

function refreshShapes() {
  const div = mainPlot();
  if (!div || !div.data || !unit) return;
  const shapes = div.dataset.which === 'raw' ? reviewShapes() : [];
  window.Plotly.relayout(div, { shapes }).catch(() => {});
}

// The moment on a plot: a thin line laid over it, moved without redrawing the plot (the owner,
// 15 Oct: playback lagged; the plot was laid out again at every frame). `x` in the plot's own units.
function cursorAt(div, x) {
  const fl = div && div._fullLayout;
  if (!fl || !fl.xaxis || !div.parentElement) return;
  div._liteX = x;
  let c = div._liteCursor;
  if (!c || !c.isConnected) {
    c = div._liteCursor = el('div', { class: 'lite-plot-cursor', 'aria-hidden': 'true' });
    div.parentElement.append(c);
    if (!div._liteCursorZoom) { div._liteCursorZoom = true; div.on('plotly_relayout', () => cursorAt(div, div._liteX)); }
  }
  const [a, b] = fl.xaxis.range;
  c.hidden = x == null || x < Math.min(a, b) || x > Math.max(a, b);
  if (c.hidden) return;
  c.style.left = `${div.offsetLeft + fl._size.l + fl.xaxis.l2p(x)}px`;
  c.style.top = `${div.offsetTop + fl._size.t}px`;
  c.style.height = `${fl._size.h}px`;
}

function setTotal() {
  const total = sim.lines > 1 ? (sim.lines - 1) * CYCLE_S : 0;
  if ($('litePlotTotal')) $('litePlotTotal').textContent = ` / ${total.toFixed(3)} s`;
}

// The moment on the plots, as the 3D plays (sim.js): the recording's row in Review; in Compare the
// tuned file's line on its plot, and the recording's row on the recording's plot below.
function moveHead(line, row) {
  head.line = line; head.row = row;
  const div = mainPlot();
  if (div && div.data) cursorAt(div, div.dataset.which === 'raw' ? row : line);
  if (pipeStep === 6) {                                   // Compare: both files on one time axis
    cursorAt($('liteCompareOriginal')?.querySelector(':scope > .lite-plot-compare'), (row - 1) * CYCLE_S);
    cursorAt($('liteCompareTuned')?.querySelector(':scope > .lite-plot-compare'), (line - 1) * CYCLE_S);
  }
  const t = Math.max(0, line - 1) * CYCLE_S;
  if ($('litePlotTime')) $('litePlotTime').textContent = `${t.toFixed(3)} s`;
  const lab = $('liteRegionLabel');
  if (lab) {
    const r = analysis && unit && analysis.key === unit.key && analysis.file === unit.active
      ? analysis.data.regions.findIndex(x => row >= x.start && row <= x.end) : -1;
    lab.textContent = r >= 0 ? `Current region: ${r + 1} · ${analysis.data.regions[r].motion_type}` : 'Current region: –';
  }
}

// ── 5 Generate

const tunedName = () => ($('tunedOutputName')?.value || '').trim() || window.tunedSuggestedName();

function renderGenerate() {
  const box = $('liteGenMetrics'), status = $('liteGenStatus');
  if (!box || !unit) return;
  const cand = unit.candidate, r = cand && cand.result, ready = !!(r && unit.fresh);
  for (const id of ['liteDlTxt', 'liteDlReport', 'liteDlPng']) if ($(id)) $(id).disabled = !ready;
  if (status) {
    status.textContent = !unit.active ? 'No recording yet.' : !r ? 'Ready: press Confirm & Generate.'
      : !unit.fresh ? 'The settings or the recording changed since the last generation: press Confirm & Generate again.'
        : r.uploadable ? `Generated: ${fmtS(r.achieved_s)}, every limit passed. Save it under its name below, or compare it first.`
          : `Generated, but not every limit passes: ${r.message} Change the settings and generate again.`;
    status.className = `statusbar ${ready ? (r.uploadable ? 'ok' : 'warn') : ''}`;
  }
  if (!ready) { box.replaceChildren(); return; }
  const s = settingsOf(unit), plan = (cand.recipe && cand.recipe.plan) || {}, raw = rec();
  const stages = [s.trim && 'Trim', s.smooth && 'Mean Smooth', s.scale && 'Scale'].filter(Boolean).join(' + ') || 'None';
  const original = raw ? raw.seconds : r.recorded_s;
  box.replaceChildren(metric('Optimiser', r.optimiser === 'toppra' ? 'Time-optimal' : 'Default'), metric('Selected stages', stages),
    metric('Recording lines', raw ? raw.lines.toLocaleString() : '—'),
    metric('Idle lines removed', plan.trimmed_lines != null ? plan.trimmed_lines.toLocaleString() : '—'),
    metric('Lines after Trim', plan.kept_lines != null ? plan.kept_lines.toLocaleString() : '—'),
    metric('Smoothing window', plan.smooth_window ? `${plan.smooth_window} lines` : 'off'),
    metric('Scale regions applied', String((plan.segments || []).length)),
    metric('Final lines', r.lines.toLocaleString()), metric('Recording time', fmtS(original)), metric('Final time', fmtS(r.achieved_s)),
    metric('Time saved', `${(original - r.achieved_s).toFixed(3)} s`), metric('Reduction', `${pct(original, r.achieved_s).toFixed(1)} %`));
}

async function generate() {
  if (!unit) return;
  const status = $('liteGenStatus');
  if (status) { status.textContent = 'Generating: tuning on the server…'; status.className = 'statusbar'; }
  const ok = await retune();
  syncTunedOutput();
  renderGenerate();
  renderPipe();
  if (ok && unit.candidate && unit.candidate.result && !unit.candidate.result.uploadable) {
    window.toast(`Generated, but not every limit passes: ${unit.candidate.result.message}`);
  }
}

async function saveBlobAs(blob, name, types) {
  if (typeof window.showSaveFilePicker === 'function') {
    try {
      const h = await window.showSaveFilePicker({ suggestedName: name, types });
      const w = await h.createWritable();
      await w.write(blob);
      await w.close();
      return h.name || name;
    } catch (err) {
      if (err && err.name === 'AbortError') return null;
    }
  }
  const url = URL.createObjectURL(blob);
  const a = el('a', { href: url, download: name });
  document.body.append(a); a.click(); a.remove();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
  return name;
}

async function download(kind) {
  if (!unit || !unit.candidate || !unit.fresh) { window.toast('Generate first: Confirm & Generate makes the tuned file.'); return; }
  const base = tunedName().replace(/\.txt$/i, '');
  try {
    let saved;
    if (kind === 'txt') {
      const blob = await get(`${unitUrl(unit.key, 'file')}?which=candidate`, { raw: true });
      saved = await saveBlobAs(blob, `${base}.txt`, [{ description: 'Trajectory', accept: { 'text/plain': ['.txt'] } }]);
    } else if (kind === 'report') {
      const report = { source: rec() && rec().name, region: unit.key, arm: unit.arm, generated: unit.candidate_extra && unit.candidate_extra.made,
        settings: settingsOf(unit), result: unit.candidate.result, checks: unit.candidate.checks, recipe: unit.candidate.recipe,
        automatic_analysis: analysis && analysis.key === unit.key ? analysis.data : null };
      saved = await saveBlobAs(new Blob([JSON.stringify(report, null, 2)], { type: 'application/json' }), `${base}_processing_report.json`,
        [{ description: 'Report', accept: { 'application/json': ['.json'] } }]);
    } else {
      await loadPlotly();
      const fig = figure(await trackData('candidate'), { title: `Tuned — ${base}.txt` });
      const url = await window.Plotly.toImage({ data: fig.data, layout: { ...fig.layout, paper_bgcolor: 'white', plot_bgcolor: 'white' } },
        { format: 'png', width: 1600, height: 900 });
      saved = await saveBlobAs(await (await fetch(url)).blob(), `${base}_J1_to_J7.png`, [{ description: 'PNG image', accept: { 'image/png': ['.png'] } }]);
    }
    if (saved) window.toast(`Saved as ${saved}.`);
  } catch (err) { tell(err); }
}

// V32's tuned output: "Apply Tuning" generates; "Save Tuned File As…" keeps the tuned file in the
// project under its name (one name per file), for Real Test. Only a file that passes every limit.
function syncTunedOutput() {
  if (!unit) return;
  const r = rec(), vs = unit.versions || [], ver = vs[vs.length - 1];
  tuningSourceFileName = (r && r.name) || tuningSourceFileName;
  tuningApplied = !!(unit.candidate && unit.fresh);
  tunedOutputFileName = ver && ver.name && unit.candidate && unit.fresh && ver.sha256 === unit.candidate.track_sha256 ? ver.name : '';
  const input = $('tunedOutputName');
  if (input && !input.value) input.value = window.tunedSuggestedName();
  window.updateTunedOutputUI();
}

// ── results (the limits)

const CHECK_ORDER = ['L1', 'L2', 'L3', 'L4', 'L5', 'L6'];

function fmtCheck(c) {
  if (c.value == null) return c.detail || '';
  const v = Math.abs(c.value) >= 100 ? Math.round(c.value) : Number(c.value).toFixed(2);
  return `${v}${c.unit ? ' ' + c.unit : ''}${c.limit != null ? ` (limit ${c.limit}${c.unit ? ' ' + c.unit : ''})` : ''}`;
}

// The gates between this step and any execution, as the server judges them (arm, stage, limits, robot).
export function gateChips(u) {
  return el('div', { class: 'lite-gates' }, (u.gates || []).map(g =>
    el('span', { class: `lite-limit ${g.ok ? 'ok' : g.id === 'robot' || g.id === 'limits' ? 'na' : 'bad'}`, title: g.id },
      `${g.ok ? '✓' : '✕'} ${g.text}`)));
}

function showResults(job = null) {
  const box = $('liteResults');
  if (!box || !unit) return;
  const cand = unit.candidate, r = cand && cand.result;
  const raw = rec();
  const top = el('div', { class: 'lite-results-head' }, el('span', { class: 'badge violet', text: 'LIMITS' }));
  if (job && job.state === 'running') {
    top.append(el('b', { text: ` Tuning… ${job.stage || ''}` }));
    box.replaceChildren(top);
    return;
  }
  if (!r) {
    top.append(el('b', { text: raw ? ' Not generated yet: press Confirm & Generate.' : ' No recording yet: record the region or load a file.' }));
    box.replaceChildren(top, gateChips(unit));
    return;
  }
  const ready = r.uploadable && unit.fresh;
  top.append(el('b', { class: ready ? 'lite-ok' : 'lite-bad',
    text: !unit.fresh ? ' Settings changed: press Confirm & Generate again.' : r.uploadable ? ' Every limit passes ✓' : ' Not ready for the robot' }));
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
  const cut = cand.recipe && cand.recipe.cut;
  const bridges = cut && cut.bridges && cut.bridges.length
    ? el('p', { class: 'mini-note', text: `Trim removed ${cut.removed_lines.toLocaleString()} rows; ${cut.bridges.map(b => `the gap after row ${b.after_line} is crossed in ${b.seconds.toFixed(2)} s`).join('; ')}. Check it in the simulation.` })
    : null;
  box.replaceChildren(top, gateChips(unit), grid, chips, msg, hints || '', bridges || '');
}

function useTime(s) {
  $('liteTimeMode').value = 'target';
  $('liteTarget').disabled = false;
  $('liteTarget').value = s.toFixed(2);
  push(`Target ${s.toFixed(2)} s`);
}

// ── 6 Compare

async function renderCompare() {
  const box = $('liteCompareMetrics');
  if (!box || !unit || !unit.candidate || !unit.fresh) return;
  const r = unit.candidate.result, raw = rec(), plan = (unit.candidate.recipe && unit.candidate.recipe.plan) || {};
  const original = raw ? raw.seconds : r.recorded_s;
  box.replaceChildren(metric('Recording duration', fmtS(original)), metric('Tuned duration', fmtS(r.achieved_s)),
    metric('Time saved', `${(original - r.achieved_s).toFixed(3)} s`), metric('Percentage reduction', `${pct(original, r.achieved_s).toFixed(1)} %`),
    metric('Modified scale regions', String((plan.segments || []).length)));
  const tbody = $('liteCompareRegions')?.querySelector('tbody');
  if (tbody) {
    const segs = plan.segments || [];
    tbody.replaceChildren(...(segs.length ? segs.map((g, i) => el('tr', {},
      el('td', { text: String(i + 1) }), el('td', { text: `${g.start} – ${g.end}` }), el('td', { text: String(g.pct) }),
      el('td', { text: String(Math.max(0, g.last_kept - g.first_kept + 1)) }),
      el('td', { text: g.pct < 100 ? `${(100 - g.pct).toFixed(1)} % shorter` : g.pct > 100 ? `${(g.pct - 100).toFixed(1)} % longer` : 'unchanged' })))
      : [el('tr', {}, el('td', { colspan: '5', class: 'mini-note', text: 'No scale region was applied.' }))]));
  }
  await loadPlotly();
  const [orig, tuned] = await Promise.all([trackData('raw'), trackData('candidate')]);
  const span = Math.max(orig.lines, tuned.lines, 2) - 1;
  const draw = async (host, tr, title, at) => {
    const div = plotDiv(host, 'lite-plot-compare');
    const fig = figure(tr, { title, seconds: true, span });
    await window.Plotly.react(div, fig.data, fig.layout, { ...PLOT_CONFIG, toImageButtonOptions: { filename: title.replace(/[^\w.-]+/g, '_') } });
    cursorAt(div, at);
  };
  await draw($('liteCompareOriginal'), orig, `Original — ${(raw || {}).name || ''} · ${fmtS(original)}`, (head.row - 1) * CYCLE_S);
  await draw($('liteCompareTuned'), tuned, `Tuned — ${tunedName()} · ${fmtS(r.achieved_s)}`, (head.line - 1) * CYCLE_S);
}

// ── the simulation beside the plot

async function loadSimulation(which) {
  const host = $('simulationViewport');
  if (!host || !unit) return;
  if (elsewhere()) { invalidate(); return; }              // the Viewer, the full sequence or a picture has the 3D: later
  if (!unit.active || (which === 'candidate' && !(unit.candidate && unit.fresh))) {
    unload(Object.assign(new Error(unit.active ? 'Generate first: the tuned file is played once it is made.' : 'This region has no recording yet.'),
      { hint: unit.active ? 'Confirm & Generate in step 5.' : 'Record it, or use an existing or loaded file.', key: unit.key }));
    await showStanding(host, unit).catch((err) => say3d(host, `The 3D view could not start: ${say(err)}`, err));
    return;
  }
  const progress = which === 'candidate' ? unit.candidate_extra && unit.candidate_extra.progress : null;
  if (!host.querySelector('canvas')) say3d(host, 'Loading the 3D view…');   // the first time: the robot's meshes come first
  try {
    if (!(await showFile(host, unit, which, progress))) return;     // another file was asked for meanwhile
    window.resetSimulationPlayback(false);
    $('simStatus').textContent = (which === 'candidate' ? `The tuned file: ${fmtS(sim.lines ? (sim.lines - 1) * CYCLE_S : 0)}.`
      : `The recording as taught: ${fmtS(sim.lines ? (sim.lines - 1) * CYCLE_S : 0)}.`)
      + ` ▶ Start stages ${sim.tl.stage.text} first, then moves to the start.`;
    $('simStatus').className = 'statusbar ok';
    setTotal();
  } catch (err) {
    $('simStatus').textContent = `Nothing plays: ${say(err)}`;
    $('simStatus').className = 'statusbar warn';
    host.dataset.lite3d = 'blocked';
    say3d(host, `Nothing plays: ${say(err)}`, err);
  }
}

export async function retune() {
  if (!unit) return null;
  if (tuning) return tuning;
  const status = $('simStatus');
  const key = unit.key;
  // Nothing plays until the new tuning is in: not the step's earlier file, nor the page's stand-in timer.
  const refused = (err) => {
    if (sim.key === key || !sim.tl) unload(Object.assign(err, { key }));
    if (status && step()?.key === key) { status.textContent = `Nothing plays: ${say(err)}`; status.className = 'statusbar warn'; }
    tell(err);
    return false;
  };
  tuning = (async () => {
    try {
      const job = await post(unitUrl(key, 'optimise'), {});
      const done = await watchJob(job, (j) => {
        showResults(j);
        if ($('liteGenStatus')) $('liteGenStatus').textContent = `Generating: ${j.stage}…`;
      });
      unit = await loadUnit(key);
      showResults();
      if (done.state !== 'done') return refused(Object.assign(new Error(done.error || 'Tuning stopped.'), { hint: done.hint }));
      invalidate();
      if (pipeStep === 6) await loadSimulation('candidate');
      return true;
    } catch (err) { return refused(err); } finally { tuning = null; }
  })();
  return tuning;
}

async function enter() {
  const s = step();
  if (!s || !ctx.pid) return;
  addControls();
  if (sim.key !== s.key) unload();                      // another step's file never plays here
  try { unit = await loadUnit(s.key); } catch (err) { tell(err); return; }
  writeControls(settingsOf(unit));
  showResults();
  syncTunedOutput();
  const n = stepOf.get(unit.key) || (unit.candidate && unit.fresh ? 5 : 1);
  pipeStep = 0;                                          // whichever step it is, it is drawn afresh
  await goStep(unlocked(n) ? n : 1, false);
}

export function installTune() {
  onPlayhead(moveHead);

  const pageSetStage = window.setStage;
  window.setStage = async function setStage(s, scroll = true) {
    if (s === 'real' && currentStage === 'optimize' && step() && ctx.pid) {
      const ok = await keep();
      if (!ok) return;
    }
    const was = currentStage;
    // Prepare stays open to the arm being taught (the owner, 14 Oct: what was forgotten there is set
    // there and carries on). The page refuses it once the arm is locked, its own Record Again
    // included; it is entered as Prepare, and the arm stays locked, so the sides never swap.
    if (s === 'prepare' && armSelectionLocked && was !== 'prepare') currentStage = 'prepare';
    await pageSetStage(s, scroll);                       // Prepare's gate may hold it (prepare.js)
    if (currentStage !== was) release(false);           // the simulation or the preview stops with its stage
    if (currentStage === 'optimize' && (s === 'optimize' || s === 'simulate')) enter();
    document.dispatchEvent(new CustomEvent('lite:stage', { detail: currentStage }));
  };

  // V32's footer: Re-run Simulation plays the tuned file again (generated first if it is stale).
  const pageRerun = window.rerunSimulationAndFocus;
  window.rerunSimulationAndFocus = async function rerunSimulationAndFocus() {
    if (unit && (!unit.candidate || !unit.fresh)) { const ok = await retune(); if (!ok) return; }
    if (pipeStep !== 6 && unlocked(6)) await goStep(6, false);
    pageRerun();
  };

  window.applyTuningToActiveTrajectory = function applyTuningToActiveTrajectory() { generate(); };
  window.saveTunedTrajectoryAs = async function saveTunedTrajectoryAs() {
    if (!unit) return;
    if (!unit.candidate || !unit.fresh) { window.toast('Generate first: Confirm & Generate makes the tuned file.'); return; }
    let name = tunedName();
    if (!/\.txt$/i.test(name)) name += '.txt';
    try { unit = await post(unitUrl(unit.key, 'keep'), { name }); ctx.units[unit.key] = unit; } catch (err) { tell(err); return; }
    syncTunedOutput();
    showResults();
    window.toast(`${name} is kept in the project: Real Test uploads it.`);
  };

  const pageRender = window.renderManualScaleRegions;
  window.renderManualScaleRegions = function renderManualScaleRegions() {
    pageRender();
    if (!syncing && currentStage === 'optimize') queuePush('Scale regions');
    if (pipeStep === 4) refreshShapes();
  };

  const pageTrimSel = window.trimSelectedPlotRegion;
  window.trimSelectedPlotRegion = function trimSelectedPlotRegion() {
    pageTrimSel();
    renderCuts();
    queuePush('Trim region');
    refreshShapes();
  };

  const pageApplySel = window.applySelectedPlotRegion;
  window.applySelectedPlotRegion = function applySelectedPlotRegion() {
    const w = Number($('selectedPlotSmooth')?.value);
    pageApplySel();
    if (w && w !== Number($('meanSmoothWindow').value)) {
      window.toast(`Smoothing applies to the whole trajectory (window ${$('meanSmoothWindow').value}); one window per region comes later.`);
    }
  };

  // Reset View: the plot's own zoom back too, and no selection
  const pageResetSel = window.resetPlotSelection;
  window.resetPlotSelection = function resetPlotSelection() {
    pageResetSel();
    const div = mainPlot();
    if (div && div.data) window.Plotly.relayout(div, { 'xaxis.autorange': true, 'yaxis.autorange': true }).then(refreshShapes).catch(() => {});
  };

  document.addEventListener('change', (e) => {
    if (!pane()?.contains(e.target) || syncing) return;
    const id = e.target.id;
    if (id === 'liteTimeMode') $('liteTarget').disabled = e.target.value !== 'target';
    if (id === 'liteOptimiser') {
      for (const x of ['meanSmoothWindow', 'liteIdleWindow', 'liteIdleThreshold', 'liteIdleRun']) $(x).disabled = e.target.value === 'toppra';
    }
    const labels = { procTrim: 'Trim', procSmooth: 'Mean smooth', meanSmoothWindow: 'Smoothing window', procScale: 'Scale',
      liteIdleWindow: 'Idle window', liteIdleThreshold: 'Idle threshold', liteIdleRun: 'Minimum idle run',
      liteOptimiser: 'Optimiser', liteTimeMode: 'Total time', liteTarget: 'Target time' };
    if (labels[id]) queuePush(labels[id]);
  });

  document.addEventListener('lite:viewer-closed', () => {   // the 3D view is back from the Trajectory Viewer
    if (currentStage === 'optimize' && unit && (pipeStep === 4 || pipeStep === 6)) loadSimulation(pipeStep === 6 ? 'candidate' : 'raw');
  });

  const pageTheme = window.toggleTheme;
  window.toggleTheme = function toggleTheme() {
    pageTheme();
    if (currentStage === 'optimize' && (pipeStep === 4 || pipeStep === 6)) drawPlot().catch(() => {});
    if (currentStage === 'optimize' && pipeStep === 6) renderCompare().catch(() => {});
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
