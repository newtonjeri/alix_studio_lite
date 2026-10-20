// Tune + Simulation (R2, R6, R11, R12): the trajectory HTML's front end as it is, its steps, fields,
// buttons, tables, messages and default values (the owner, 15 Oct: "use entirely the implementation of
// the front end in the html ... with everything adopted, including the default values";
// docs/reference/ui_v32/Alix_Studio_Lite_Trajectory_V2.html), on V32's Tune stage, with what Lite adds:
// the robot in 3D beside the plots, the limits (shown, never enforced for manual tuning: the hardware
// decides), the tuned file kept in the project for Real Test, Analyze's optimised time, and the other
// tuning, Time-optimal (TOPP-RA), which takes none of the HTML's steps.
//
// Its steps, Review before Analyze (the owner, 16 Oct: the original first, in the simulation view):
//   1 Source    the region's recording (the HTML's upload: another is chosen or loaded in Save & Review)
//   2 Setup     the HTML's parameters and processing stages, or Time-optimal's time
//   3 Review    the recording played in real time on its plot and in 3D; scale regions by box-select or rows
//   4 Analyze   the HTML's automatic analysis (optimise/analysis.py); fitted to an optimised time after Trim
//   5 Generate  the HTML's processing, on the server (optimise/manual.py), its figures and its three downloads
//   6 Compare   original vs optimized on one synchronized time cursor (or an external file), the tuned file in 3D

import { get, post, say } from './api.js';
import { confirmDialog, ctx, el, fitPlot, loadUnit, step, tell, timed, unitUrl, watchJob } from './ctx.js';
import { elsewhere, fileKey, invalidate, onMoment, onPlayhead, release, say3d, scene, seekTo, showFile, showStanding, sim, unload } from './sim.js';
import { checkJob, spansOf, Tools } from './view3d.js';
import { frame } from './vp.js';

const CYCLE_S = 0.005;
let unit = null;               // the server's view of the arm step on screen
let syncing = false;           // writing the server's settings into the controls
let pushTimer = 0;             // a change on screen waiting to go to the server (its label in pushLabel)
let pushLabel = null;
let pushing = null;            // a change on its way to the server
let plotP = null;
let tuning = null;             // the running tune, if any
let pipeStep = 1;              // the HTML's step on screen
const stepOf = new Map();      // each arm step's last step
const tracks = new Map();      // the plotted files (the file's own joint values), by what they are made from
let analysis = null;           // {key, file, data}: the automatic analysis of the active recording
let regions = [];              // the scale regions on screen: {start, end, scalePct}, original rows
let external = null;           // Compare's optional optimized file: {name, rows, skipped}
let cmp3d = 'candidate';       // what Compare's 3D plays: the processed file or the original (the owner, 16 Oct)
const plotted = { raw: null, opt: null };   // the files on the plots now
const pb = { playing: false, at: { review: 0, compare: 0 }, startPerf: 0, raf: 0, kind: null };   // the HTML's real-time
let lastChange = null;         // playbacks, each at its own moment; the last settings change, for Generate's status
let suggestionsPending = false; // Analyze's suggestions refitted or edited and not applied to the scale regions yet

const pane = () => document.getElementById('stage-optimize');
const $ = (id) => document.getElementById(id);
const settingsOf = (u) => u.settings_view.settings;
const rec = () => unit && unit.recordings.find(r => r.file === unit.active);
// The analysis on screen is the active recording's, as it is now (Save & Review's Trim can overwrite it).
const analysisNow = () => !!(analysis && unit && analysis.key === unit.key && analysis.file === unit.active
  && analysis.sha === (rec() || {}).sha256);
const isManual = () => !unit || settingsOf(unit).optimiser !== 'toppra';
const fresh = () => !!(unit && unit.candidate && unit.fresh);
const report = () => (fresh() && unit.candidate.recipe && unit.candidate.recipe.report) || null;
const dur = (rows) => Math.max(0, Number(rows) - 1) * CYCLE_S;          // Lite's one clock: N rows last (N - 1) x 5 ms

// The HTML's own wording of the steps (its PIPELINE_META); step 1 is the region's recording. Review
// comes before Analyze (the owner, 16 Oct); both are manual tuning's only.
const REVIEW = 3, ANALYZE = 4;
const STEPS = [null,
  { title: 'Source', sub: 'Source file', name: 'Source trajectory', note: 'The region’s recording: choose or load another in Save & Review.' },
  { title: 'Setup', sub: 'Parameters', name: 'Processing setup', note: 'Choose Trim / Mean Smooth / Scale and review processing parameters.' },
  { title: 'Review', sub: 'Fine-tune', name: 'Playback & fine-tune', note: 'Play the source trajectory and manually adjust scaling regions if needed.' },
  { title: 'Analyze', sub: 'Suggestions', name: 'Automatic analysis', note: 'Review motion regions and optimization recommendations.' },
  { title: 'Generate', sub: 'Save output', name: 'Generate & save', note: 'Create the optimized output, then save TXT / report / plot.' },
  { title: 'Compare', sub: 'Validate', name: 'Compare & validate', note: 'Compare original and optimized timing and review modified regions.' },
];

// The HTML's formatSeconds.
function formatSeconds(seconds) {
  if (!Number.isFinite(seconds)) return '-';
  const mins = Math.floor(seconds / 60), secs = seconds - mins * 60;
  return mins > 0 ? `${seconds.toFixed(3)} s (${mins} min ${secs.toFixed(1)} s)` : `${seconds.toFixed(3)} s`;
}

// The HTML's setStatus: a status line, plain, ok, warn or err.
function setStatus(node, text, type = '') {
  if (!node) return;
  node.className = `lite-status${type ? ` ${type}` : ''}`;
  node.textContent = text;
}

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
  return { joints: ['--blue', '--orange', '--green', '--red', '--violet', '--cyan', '--muted'].map(v), ink: v('--muted'),
    line: v('--line'), blue: v('--blue'), green: v('--green'), orange: v('--orange'), violet: v('--violet'), red: v('--red'), bg: 'transparent' };
}

// The HTML's plot image: its JOINT_COLORS on white with Plotly's own ink, whichever theme is on (lite/tokens.css).
function printPalette() {
  const cs = getComputedStyle(document.documentElement), v = (n) => cs.getPropertyValue(n).trim();
  return { joints: [1, 2, 3, 4, 5, 6, 7].map(j => v(`--png-j${j}`)), ink: v('--png-ink'), line: v('--png-grid'), bg: v('--png-bg') };
}

const metric = (k, v) => el('div', { class: 'lite-metric' }, el('div', { class: 'k', text: k }), el('div', { class: 'v', text: v }));
const numField = (id, label, attrs) => el('div', { class: 'field' }, el('label', { for: id, text: label }), el('input', { id, type: 'number', ...attrs }));
const title = (h, note, ...more) => el('div', { class: 'lite-section-title' }, el('h3', { text: h }), el('span', { class: 'mini-note', text: note }), ...more);
const statusLine = (id, text) => el('div', { id, class: 'lite-status', text });
const thead = (cols) => el('thead', {}, el('tr', {}, cols.map(c => el('th', { text: c }))));

// ── the HTML's pipeline on V32's Tune stage

function playBar(kind) {
  const compare = kind === 'compare';
  return el('div', { class: 'lite-playback' },
    el('button', { class: 'btn', id: `litePlay_${kind}`, text: compare ? 'Play Comparison' : 'Play', onclick: () => play(kind) }),
    el('button', { class: 'btn outline', text: 'Pause', onclick: () => pause() }),
    el('button', { class: 'btn light', text: 'Restart', onclick: () => restart(kind) }),
    el('div', { class: 'lite-timeline' },
      el('label', { for: `liteTimeline_${kind}`, text: compare ? 'Synchronized time cursor' : 'Trajectory timeline' }),
      el('div', { id: `liteTrack_${kind}`, class: 'lite-rev3d-track' },
        el('input', { id: `liteTimeline_${kind}`, type: 'range', min: '0', max: '0', step: '0.001', value: '0',
          oninput: (e) => scrub(kind, Number(e.target.value)) }))),
    el('div', { class: 'lite-readout' },
      el('div', {}, el('b', { id: `liteNow_${kind}`, text: '0.000 s' }), ' / ', el('span', { id: `liteTotal_${kind}`, text: '0.000 s' })),
      compare ? null : el('div', { id: 'liteRegionLabel', class: 'mini-note', text: 'Current region: -' })));
}

function addControls() {
  const p = pane();
  if (!p || $('litePipe')) return;
  // V32's own tuning cards give way to the HTML's steps; its 3D pane moves into Review and Compare
  for (const sel of ['.processing-card', '.workspace-50-50', '.post-analysis-tuning', '.tuned-output-card']) {
    const n = p.querySelector(sel);
    if (n) n.dataset.liteSteps = 'none';
  }
  const foot = p.querySelector('.workspace-footer');
  if (foot) foot.dataset.liteSteps = '6';

  const track = el('div', { class: 'lite-pipe-track', role: 'tablist', 'aria-label': 'Trajectory processing steps' },
    STEPS.slice(1).map((m, i) => el('button', { type: 'button', class: 'lite-pipe-node', 'data-step': String(i + 1), role: 'tab',
      onclick: () => goStep(i + 1) },
    el('span', { class: 'step-num', text: String(i + 1) }),
    el('span', { class: 'step-copy' }, el('span', { class: 'step-title', text: m.title }), el('span', { class: 'step-sub', text: m.sub })))));
  const pipe = el('div', { id: 'litePipe', class: 'lite-pipe' },
    el('div', { class: 'lite-pipe-head' }, el('strong', { text: 'Trajectory Optimization Pipeline' }), el('span', { id: 'litePipeSummary' })),
    track);
  const anchor = p.querySelector('.tuning-active-file-card');
  if (anchor) anchor.after(pipe); else p.prepend(pipe);

  // 1 Source: the region's recording, with the HTML's figures
  const source = el('div', { id: 'liteSource', class: 'lite-step-card', 'data-lite-steps': '1' },
    title('1. Source trajectory', 'Expected line format: {"point":[J1,J2,J3,J4,J5,J6,J7]}'),
    el('div', { class: 'lite-row' },
      el('div', { class: 'field lite-grow' }, el('label', { text: 'Original TXT / JSONL file' }), el('div', { id: 'liteSourceName', class: 'lite-source-name', text: '-' })),
      el('button', { id: 'liteSourceOther', class: 'btn outline', text: 'Choose or load another (Save & Review)', onclick: () => window.setStage('review') }),
      el('button', { class: 'btn light', id: 'liteReset', text: 'Reset', disabled: true, onclick: resetPipe })),
    statusLine('liteLoadStatus', 'No file loaded.'),
    el('div', { id: 'liteSourceMetrics', class: 'lite-metrics' }));

  // 2 Setup: the HTML's parameters and stages, its defaults; or Time-optimal's time
  const stage = (id, name, small) => el('label', { class: 'lite-stage-option' }, el('input', { id, type: 'checkbox', checked: true }),
    el('span', {}, `${name} `, el('small', { text: small })));
  const mode = (value, name, small) => el('label', { class: 'lite-mode-option' },
    el('input', { type: 'radio', name: 'liteMode', value, id: `liteMode_${value}` }), el('span', {}, el('b', { text: name }), el('small', { text: small })));
  const setup = el('div', { id: 'liteSetup', class: 'lite-step-card', 'data-lite-steps': '2' },
    title('2. Processing parameters', 'Defaults match the trajectory algorithm used previously.',
      el('div', { class: 'lite-history' },
        el('button', { class: 'btn light', id: 'liteUndo', text: '↶ Undo', onclick: () => history('undo') }),
        el('button', { class: 'btn light', id: 'liteRedo', text: '↷ Redo', onclick: () => history('redo') }))),
    el('div', { class: 'lite-mode', role: 'radiogroup', 'aria-label': 'Tuning' },
      mode('manual', 'Manual tuning', 'The trajectory algorithm below (the default). Its limits are shown, never enforced: the hardware decides.'),
      mode('toppra', 'Time-optimal (TOPP-RA)', 'Automatic, on the recording as taught: none of the steps below, and every limit kept.')),
    el('div', { id: 'liteManualSetup' },
      el('div', { class: 'lite-row' },
        numField('liteSampleMs', 'Sample interval (ms)', { value: '5', min: '0.001', step: '0.001', readonly: true,
          title: 'Lite’s one clock: the controller plays one line every 5 ms' }),
        numField('liteIdleWindow', 'Idle rolling window', { value: '11', min: '3', step: '2' }),
        numField('liteIdleThreshold', 'Idle threshold (counts/sample)', { value: '8', min: '0', step: '0.1' }),
        numField('liteIdleRun', 'Minimum idle run', { value: '20', min: '1', step: '1' }),
        numField('liteSmoothWindow', 'Mean smoothing window', { value: '101', min: '3', step: '2' })),
      el('div', { class: 'lite-stages-head', text: 'Processing stages to apply' }),
      el('div', { class: 'lite-stage-options' },
        stage('liteStageTrim', '(1) Trim', 'idle-region removal'),
        stage('liteStageSmooth', '(2) Mean Smooth', 'centered moving mean'),
        stage('liteStageScale', '(3) Scale', 'selected-region resampling')),
      el('div', { class: 'lite-hint' }, 'Select any one stage or any combination. When multiple stages are enabled, they are always applied in this order: ',
        el('b', { text: 'Trim → Mean Smooth → Scale' }), '. Unchecked stages are skipped completely.'),
      el('details', {}, el('summary', { text: 'Algorithm used' }),
        el('p', { class: 'mini-note', text: 'Idle trim: calculate absolute change between consecutive rows for each joint, apply a centered rolling mean, '
          + 'mark a step idle only when all 7 joints are at or below the threshold, require the condition to persist for the minimum run, '
          + 'then keep the first and last point of each idle block and remove the interior points.' }),
        el('p', { class: 'mini-note', text: 'Mean smoothing: centered moving mean (default 101 points), then recenter each joint so its overall '
          + 'trimmed mean is preserved; exact first and last joint positions are restored.' }),
        el('p', { class: 'mini-note' }, 'Region scaling: synchronized linear interpolation across all 7 joints. ', el('b', { text: '100%' }),
          ' = unchanged duration, ', el('b', { text: '150%' }), ' = 50% longer/slower, ', el('b', { text: '50%' }),
          ' = half as many intervals/faster, ', el('b', { text: '30%' }), ' = 30% of the original region duration.'))),
    el('div', { id: 'liteToppraSetup', hidden: true },
      el('div', { class: 'lite-row' },
        el('div', { class: 'field' }, el('label', { for: 'liteTopMode', text: 'Total time' }),
          el('select', { id: 'liteTopMode' }, el('option', { value: 'fastest', text: 'As fast as the limits allow' }),
            el('option', { value: 'target', text: 'Target time' }))),
        numField('liteTopTarget', 'Target time (s)', { min: '0.05', max: '600', step: '0.01' })),
      el('div', { class: 'lite-hint', text: 'Time-optimal works on the recording as taught, through ALIX’s own chain (its 5 Hz smoothing, its '
        + 'pause removal and 30 mm path smoothing, then TOPP-RA within 0.95 × the joint ratings and 600 °/s²). It never uses Trim, '
        + 'Mean Smooth, Scale, the suggestions or the scale regions, and a file that fails a limit never goes to the robot.' }),
      el('div', { id: 'liteToppraNote', class: 'lite-status' })));

  // 3 Analyze: the HTML's automatic speed optimisation analysis
  const analyze = el('div', { id: 'liteAnalysis', class: 'lite-step-card', 'data-lite-steps': String(ANALYZE) },
    title(`${ANALYZE}. Automatic speed optimization analysis`, 'Section B1/B2 - motion analysis and region-based recommendations'),
    el('div', { class: 'lite-hint', text: 'After a trajectory is loaded, Lite automatically analyzes joint-motion intensity, direction-change activity '
      + 'and peak motion. It divides the trajectory into review regions and suggests whether each region should be shorter/faster, longer/slower or unchanged.' }),
    el('div', { class: 'lite-note' }, el('b', { text: 'Important:' }), ' The uploaded trajectory contains joint positions only. Lite can identify '
      + 'motion patterns, but it cannot prove physical cleaning contact or collision safety without force/contact and simulation data. '
      + 'Smooth continuous regions are therefore marked as ', el('b', { text: 'possible cleaning/straight-pass candidates' }), ' for operator review.'),
    el('div', { class: 'lite-row' },
      el('button', { class: 'btn', id: 'liteAnalyze', text: 'Analyze Again', onclick: () => loadAnalysis(true) }),
      el('button', { class: 'btn green', id: 'liteApplySuggest', text: 'Apply Selected Suggestions to Scale Regions', disabled: true, onclick: applySuggestions }),
      el('div', { class: 'field lite-target-field' }, el('label', { for: 'liteAnalysisTarget', text: 'Optimised time (s)' }),
        el('input', { id: 'liteAnalysisTarget', type: 'number', min: '0.1', max: '600', step: '0.1', placeholder: 'e.g. 20',
          onkeydown: (e) => { if (e.key === 'Enter') loadAnalysis(true, true); } })),
      el('button', { class: 'btn light', id: 'liteSuggestTarget', text: 'Suggest for this time', onclick: () => loadAnalysis(true, true) })),
    statusLine('liteAnalysisStatus', 'Load a file to run automatic analysis.'),
    el('div', { id: 'liteAnalysisMetrics', class: 'lite-metrics' }),
    el('div', { class: 'lite-table-scroll' }, el('table', { class: 'lite-table', id: 'liteSuggestions' },
      thead(['Use', '#', 'Rows', 'Original time', 'Motion type', 'Peak guidance', 'Recommendation', 'Scale (%)', 'Estimated time', 'Reason']),
      el('tbody'))));

  // 4 Review: playback and the manual scaling regions, the 3D beside the plot
  const review = el('div', { id: 'liteReview', class: 'lite-step-card', 'data-lite-steps': String(REVIEW) },
    title(`${REVIEW}. Original trajectory - playback & manual scaling regions`, 'Play the trajectory in real time or manually select/adjust scaling regions.'),
    el('div', { class: 'lite-hint' }, 'Selection is made on the ', el('b', { text: 'original row numbers' }), '. Each selected region is added to the '
      + 'table below. Enter an independent scale percentage for every region. Regions may not overlap. If ', el('b', { text: 'Scale' }),
      ' is unchecked, these regions are retained but ignored during generation.'),
    playBar('review'),
    el('div', { class: 'lite-view-grid' }, el('div', { class: 'lite-view-3d', id: 'liteReview3d' }),
      el('div', { class: 'lite-plot-host' }, el('div', { id: 'liteOrigPlot', class: 'lite-plot-box' }))),
    el('div', { class: 'lite-row' },
      numField('liteManualStart', 'Manual start row', { min: '1', step: '1' }),
      numField('liteManualEnd', 'Manual end row', { min: '1', step: '1' }),
      el('button', { class: 'btn light', id: 'liteAddManual', text: 'Add manual region',
        onclick: () => addRegion(Number($('liteManualStart').value), Number($('liteManualEnd').value)) }),
      el('button', { class: 'btn outline', id: 'liteClearRegions', text: 'Clear regions', onclick: clearRegions })),
    el('div', { class: 'lite-table-scroll' }, el('table', { class: 'lite-table', id: 'liteRegions' },
      thead(['#', 'Original start row', 'Original end row', 'Scale (%)', 'Meaning', '']), el('tbody'))),
    statusLine('liteRegionStatus', 'No scaling regions selected. The file can still be trimmed and smoothed.'));

  // 5 Generate
  const generateCard = el('div', { id: 'liteGenerate', class: 'lite-step-card', 'data-lite-steps': '5' },
    title('5. Generate optimized / processed trajectory', 'Only checked processing stages will be applied.'),
    el('div', { class: 'lite-row' },
      el('button', { class: 'btn green', id: 'liteGenerateBtn', text: 'Confirm & Generate', onclick: generate }),
      el('button', { class: 'btn', id: 'liteDlTxt', text: 'Download processed TXT', onclick: () => download('txt') }),
      el('button', { class: 'btn outline', id: 'liteDlReport', text: 'Download processing report', onclick: () => download('report') }),
      el('button', { class: 'btn light', id: 'liteDlPng', text: 'Download processed plot PNG', onclick: () => download('png') })),
    el('div', { class: 'lite-hint' }, el('b', { text: 'Save As:' }), ' Clicking a download button will open the browser’s Save As dialog when '
      + 'supported, so you can choose the folder and rename the file before saving.'),
    statusLine('liteProcessStatus', 'Load a file to begin.'),
    el('div', { id: 'liteProcessedMetrics', class: 'lite-metrics' }),
    el('div', { id: 'liteResults', class: 'lite-results' }),
    el('div', { class: 'lite-keep' },
      el('div', { class: 'field lite-grow' }, el('label', { for: 'liteKeepName', text: 'Keep in the project for Real Test, as' }),
        el('input', { id: 'liteKeepName', type: 'text', autocomplete: 'off', oninput: (e) => { e.target.dataset.typed = '1'; } })),
      el('button', { class: 'btn green', id: 'liteKeepBtn', text: 'Save Tuned File As…', onclick: () => window.saveTunedTrajectoryAs() }),
      el('div', { id: 'liteKeepNote', class: 'mini-note' })));

  // 6 Compare
  const compare = el('div', { id: 'liteCompare', class: 'lite-step-card', 'data-lite-steps': '6' },
    title('6. Original vs optimized comparison', 'Section B4 - duration, time saved, percentage reduction and modified regions'),
    el('div', { class: 'lite-row' },
      el('div', { class: 'field lite-grow' }, el('label', { for: 'liteExtFile', text: 'Optional optimized TXT / JSONL file for external comparison' }),
        el('input', { id: 'liteExtFile', type: 'file', accept: '.txt,.jsonl,text/plain', onchange: loadExternal })),
      el('button', { class: 'btn light', id: 'liteUseGenerated', text: 'Use Generated Optimized File', onclick: useGenerated })),
    statusLine('liteCompareStatus', 'Comparison uses the generated optimized trajectory unless an external optimized file is loaded.'),
    el('div', { id: 'liteCompareMetrics', class: 'lite-metrics' }),
    playBar('compare'),
    el('div', { class: 'lite-row lite-cmp3d', role: 'radiogroup', 'aria-label': 'What the simulation plays' },
      el('span', { class: 'lite-cmp3d-head', text: 'Simulation plays' }),
      [['raw', 'Original'], ['candidate', 'Processed']].map(([v, n]) => el('label', { class: 'lite-stage-option lite-cmp3d-option' },
        el('input', { type: 'radio', name: 'liteCmp3d', value: v, checked: v === cmp3d, onchange: () => setCmp3d(v) }), el('span', { text: n })))),
    el('div', { class: 'lite-view-grid' }, el('div', { class: 'lite-view-3d', id: 'liteCompare3d' }),
      el('div', { class: 'lite-compare-stack' },
        el('div', { class: 'lite-plot-host small' }, el('div', { id: 'liteCmpOrig', class: 'lite-plot-box' })),
        el('div', { class: 'lite-plot-host small' }, el('div', { id: 'liteCmpProc', class: 'lite-plot-box' })))),
    el('h4', { text: 'Modified / scaled regions' }),
    el('div', { class: 'lite-table-scroll' }, el('table', { class: 'lite-table', id: 'liteCompareRegions' },
      thead(['#', 'Original rows', 'Scale (%)', 'Input rows', 'Output rows', 'Effect']), el('tbody'))));

  pipe.after(source, setup, review, analyze, generateCard, compare);
  compare.after(el('div', { id: 'litePipeNav', class: 'lite-pipe-nav' },
    el('button', { class: 'btn outline', id: 'litePipeBack', text: '← Back', onclick: () => goStep(prevStep(pipeStep)) }),
    el('div', { class: 'mini-note', id: 'litePipeNote' }),
    el('button', { class: 'btn', id: 'litePipeNext', onclick: () => goStep(nextStep(pipeStep)) }),
    el('span', { id: 'litePipeDone', class: 'lite-pipe-done', text: 'Pipeline complete', hidden: true })));
}

// The HTML's unlocking: Setup to Generate once there is a source, Compare once there is an output.
// Time-optimal has no Analyze or Review.
function unlocked(n) {
  if (n <= 1) return true;
  if (!unit || !unit.active) return false;
  if (manualOnly(n) && !isManual()) return false;
  if (n <= 5) return true;
  return fresh() || !!external;
}
const manualOnly = (n) => n === REVIEW || n === ANALYZE;
const nextStep = (n) => { let k = n + 1; while (k < 6 && manualOnly(k) && !isManual()) k += 1; return k; };
const prevStep = (n) => { let k = n - 1; while (k > 1 && manualOnly(k) && !isManual()) k -= 1; return k; };

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
    b.title = !ok && manualOnly(n) && unit && unit.active ? 'Time-optimal (TOPP-RA) takes none of the manual steps' : '';
  });
  if ($('litePipeSummary')) $('litePipeSummary').textContent = `Step ${pipeStep} of 6 - ${STEPS[pipeStep].name}`;
  if ($('litePipeNote')) $('litePipeNote').textContent = ctx.tuneKey && pipeStep === 1 ? 'The merge from S1, as it was merged there.' : STEPS[pipeStep].note;
  const back = $('litePipeBack'), next = $('litePipeNext');
  if (back) back.hidden = pipeStep <= 1;
  if (next) {
    const k = nextStep(pipeStep);
    next.hidden = pipeStep >= 6;
    next.disabled = pipeStep >= 6 || !unlocked(k);
    next.textContent = pipeStep >= 6 ? '' : pipeStep === 5 && !unlocked(6) ? 'Generate first to unlock Compare' : `Next: ${STEPS[k].name} →`;
  }
  if ($('litePipeDone')) $('litePipeDone').hidden = pipeStep < 6;
  if ($('liteReset')) $('liteReset').disabled = !(unit && unit.active);
}

// Suggestions refitted or edited in Analyze and not applied would leave what comes after on the earlier result
// (the owner, 16 Oct: "Changing an earlier step should invalidate or refresh everything downstream"): asked
// before any step that uses the scale regions, from wherever it is reached. Apply them (the processed file is
// then made again), Not now (the scale regions as they are), or Escape: stay here. {go, applied}.
async function settleSuggestions(uses) {
  if (!(suggestionsPending && isManual() && analysisNow())) return { go: true, applied: false };
  const t = analysis.data.target;
  const yes = await confirmDialog({ title: 'Apply the suggestions first?', danger: false, ok: 'Apply them', cancelText: 'Not now',
    escape: null, focus: 'ok',
    body: `The suggestions shown in Analyze${t ? ` (fitted to ${formatSeconds(t.target_s)})` : ''} are not in the scale regions yet, `
      + `and ${uses}. Apply them now? The processed file is then made again from them. (Escape: stay here.)` });
  if (yes === null) return { go: false, applied: false };
  suggestionsPending = false;
  if (yes) await applySuggestions(true);
  return { go: true, applied: !!yes };
}

async function goStep(n, scroll = true) {
  n = Math.max(1, Math.min(6, Number(n) || 1));
  await flushPush();                                       // a change typed a moment ago counts before the step decides
  if (n >= 5) {                                            // Generate and Compare use the scale regions
    const s = await settleSuggestions('Generate and Compare use the scale regions');
    if (!s.go) return;
    if (s.applied && n === 6 && !unlocked(6)) n = 5;       // the earlier processed file no longer stands: Generate it
  }
  if (!unlocked(n)) {
    window.toast(n === 6 ? 'Generate first: Compare opens once the processed file is made from these settings.'
      : manualOnly(n) && unit && unit.active ? 'Time-optimal (TOPP-RA) takes none of the manual steps: choose Manual tuning in Setup.'
        : 'This region has no recording yet: record it, or load a file in Save & Review.', 'warn');
    return;
  }
  const was = pipeStep;
  pipeStep = n;
  if (unit) stepOf.set(unit.key, n);
  if (was !== n) { pause(); release(false); }                // what played stops with its step
  renderPipe();
  if (scroll) $('litePipe')?.scrollIntoView({ behavior: 'smooth', block: 'start' });
  if (n === 1) renderSource();
  if (n === 2) renderSetup();
  if (n === ANALYZE) await loadAnalysis(false);
  if (n === REVIEW) {                                    // the HTML analyses on load: Review shows its regions
    if (!analysisNow()) await loadAnalysis(false);
    place3d('liteReview3d');
    await Promise.all([drawReview().catch(tell), loadSimulation('raw')]);
  }
  if (n === 5) { renderGenerate(); showResults(); }
  if (n === 6) { place3d('liteCompare3d'); await Promise.all([renderCompare().catch(tell), loadSimulation(cmp3d)]); }
}

// V32's 3D pane (the robot, its status line) beside Review's or Compare's plots.
function place3d(hostId) {
  const pane3d = $('simulationPane'), host = $(hostId);
  if (pane3d && host && pane3d.parentElement !== host) host.append(pane3d);
}

// ── 1 Source

function renderSource() {
  const box = $('liteSourceMetrics'), r = rec();
  if (!box) return;
  $('liteSourceName').textContent = r ? timed(r.name, r.seconds) : '-';
  if (!r) {
    setStatus($('liteLoadStatus'), 'No file loaded. Record the region, or load a file in Save & Review.');
    box.replaceChildren();
    return;
  }
  activeTrajectoryRows = r.lines;
  activeTrajectoryName = r.name || activeTrajectoryName;
  window.updateActiveTrajectoryUI();
  if (ctx.tuneKey && $('tuningTrajectoryMeta')) $('tuningTrajectoryMeta').textContent = `Merge from S1 · ${Number(r.lines).toLocaleString()} rows`;
  setStatus($('liteLoadStatus'), `Loaded ${r.name}: ${Number(r.lines).toLocaleString()} valid rows.`, 'ok');
  box.replaceChildren(metric('Valid rows', Number(r.lines).toLocaleString()), metric('Skipped / malformed lines', '0'),
    metric('Sample interval', '5 ms'), metric('Original file time', formatSeconds(dur(r.lines))),
    metric('Taught on', `${String(r.taught_arm || '?').toUpperCase()} arm · ${r.stage ? r.stage.text : 'stage unknown'}`));
}

// The HTML's Reset: the scale regions, the analysis, the comparison and the playback cleared and every
// stage checked again (one undo step), back at step 1. The recording stays: it is the region's.
async function resetPipe() {
  if (!unit || !unit.active) return;
  pause();
  analysis = null;
  external = null;
  pb.at.review = pb.at.compare = 0;
  if ($('liteExtFile')) $('liteExtFile').value = '';
  $('liteSuggestions')?.querySelector('tbody')?.replaceChildren();
  $('liteAnalysisMetrics')?.replaceChildren();
  syncing = true;
  for (const id of ['liteStageTrim', 'liteStageSmooth', 'liteStageScale']) $(id).checked = true;
  syncing = false;
  regions = [];
  updateStageUI();
  clearTimeout(pushTimer);
  pushTimer = 0;
  await push('Reset');
  setStatus($('liteRegionStatus'), 'No scaling regions selected. The file can still be trimmed and smoothed.');
  await goStep(1);
}

// ── 2 Setup: the controls <-> the region's settings on the server (one undo step per change)

function stageSel() {
  return { trim: !!$('liteStageTrim')?.checked, smooth: !!$('liteStageSmooth')?.checked, scale: !!$('liteStageScale')?.checked };
}

function stageLabel(st = stageSel()) {
  const names = [st.trim && 'Trim', st.smooth && 'Mean Smooth', st.scale && 'Scale'].filter(Boolean);
  return names.length ? names.join(' + ') : 'None';
}

// A field being typed in keeps what is typed (a reply to an earlier change never overwrites it), and the
// regions on screen stay while one of their scales is being typed.
const typing = (node) => !!node && document.activeElement === node;
const editingRegion = () => !!$('liteRegions')?.contains(document.activeElement) && document.activeElement.tagName === 'INPUT';

function writeControls(s) {
  syncing = true;
  try {
    $('liteStageTrim').checked = s.trim !== false;
    $('liteStageSmooth').checked = s.smooth !== false;
    $('liteStageScale').checked = s.scale !== false;
    // the numbers as typed, as the HTML's fields keep them (empty: its default is used)
    for (const [id, k] of [['liteIdleWindow', 'idle_window'], ['liteIdleThreshold', 'idle_threshold_md'], ['liteIdleRun', 'idle_min_run'],
      ['liteSmoothWindow', 'smooth_window']]) if (!typing($(id))) $(id).value = s[k] ?? '';
    if (!editingRegion()) regions = (s.segments || []).map(r => ({ start: r.start, end: r.end, scalePct: r.pct }));
    const tp = s.optimiser === 'toppra';
    $('liteMode_manual').checked = !tp;
    $('liteMode_toppra').checked = tp;
    $('liteMode_toppra').disabled = !unit.toppra && !tp;
    $('liteTopMode').value = s.time_mode === 'target' ? 'target' : 'fastest';
    if (!typing($('liteTopTarget'))) $('liteTopTarget').value = s.target_s ?? '';
    $('liteTopTarget').disabled = s.time_mode !== 'target';
    $('liteManualSetup').hidden = tp;
    $('liteToppraSetup').hidden = !tp;
    setStatus($('liteToppraNote'), unit.toppra ? '' : 'Time-optimal is not available on this computer (toppra is not installed).',
      unit.toppra ? '' : 'warn');
    const h = unit.settings_view;
    $('liteUndo').disabled = !h.can_undo;
    $('liteUndo').title = h.undo_label ? `Undo: ${h.undo_label}` : '';
    $('liteRedo').disabled = !h.can_redo;
    $('liteRedo').title = h.redo_label ? `Redo: ${h.redo_label}` : '';
    updateStageUI();
  } finally { syncing = false; }
}

// The HTML's updateStageUI: a stage that is off greys out its own parameters.
function updateStageUI() {
  const st = stageSel(), scaleOn = st.scale && !!(unit && unit.active);
  for (const id of ['liteManualStart', 'liteManualEnd', 'liteAddManual', 'liteClearRegions']) if ($(id)) $(id).disabled = !scaleOn;
  for (const id of ['liteIdleWindow', 'liteIdleThreshold', 'liteIdleRun']) if ($(id)) $(id).disabled = !st.trim;
  if ($('liteSmoothWindow')) $('liteSmoothWindow').disabled = !st.smooth;
  renderRegions();
}

const typed = (id) => { const v = String($(id).value).trim(); return v === '' ? null : Number(v); };

function readControls() {
  const tp = $('liteMode_toppra').checked, target = Number($('liteTopTarget').value);
  return {
    optimiser: tp ? 'toppra' : 'manual',
    trim: $('liteStageTrim').checked, smooth: $('liteStageSmooth').checked, scale: $('liteStageScale').checked,
    idle_window: typed('liteIdleWindow'), idle_threshold_md: typed('liteIdleThreshold'),       // read the HTML's way
    idle_min_run: typed('liteIdleRun'), smooth_window: typed('liteSmoothWindow'),               // when a file is made
    segments: regions.map(r => ({ start: r.start, end: r.end, pct: r.scalePct })),
    time_mode: $('liteTopMode').value,
    target_s: $('liteTopMode').value === 'target' && target > 0 ? target : settingsOf(unit).target_s,
  };
}

function queuePush(label) {
  if (syncing || !unit) return;
  clearTimeout(pushTimer);
  pushLabel = label;
  pushTimer = setTimeout(() => { pushTimer = 0; push(label); }, 250);
}

// What is on screen reaches the server first: Generate, keeping and another arm step use these settings.
async function flushPush() {
  if (pushTimer) { clearTimeout(pushTimer); pushTimer = 0; await push(pushLabel); }
  while (pushing) await pushing;
}

async function push(label) {
  if (!unit) return;
  const now = readControls(), was = settingsOf(unit);
  const changes = Object.fromEntries(Object.entries(now).filter(([k, v]) => JSON.stringify(v) !== JSON.stringify(was[k])));
  if (!Object.keys(changes).length) return;
  const req = post(unitUrl(unit.key, 'settings'), { changes, label });
  const mine = pushing = req.catch(() => null);
  try {
    unit = await req;
    ctx.units[unit.key] = unit;
  } catch (err) { tell(err); } finally { if (pushing === mine) pushing = null; }
  if (pushTimer) return;                      // a newer change goes next: the controls keep what is on screen
  afterSettings(label);
}

async function history(dir) {
  if (!unit) return;
  try { unit = await post(unitUrl(unit.key, dir), {}); ctx.units[unit.key] = unit; } catch (err) { tell(err); return; }
  afterSettings();
}

function afterSettings(label = null) {
  if (label) lastChange = label;
  writeControls(settingsOf(unit));
  if (label == null || TRIM_LABELS.includes(label)) dropFit();   // the fit was "after Trim" with the earlier values
  renderPipe();
  if (pipeStep === REVIEW) refreshShapes();
  if (pipeStep === 5) { renderGenerate(label); showResults(); }
  if (unit.candidate && !unit.fresh) {        // the HTML's invalidateProcessed: Compare waits for a new output
    if (pipeStep === 6 && !external) goStep(5, false);
  }
}

function renderSetup() { if (unit) writeControls(settingsOf(unit)); }

// ── 3 Analyze: the HTML's automatic analysis, from the server (the HTML's own rules)

// `withTarget`: the suggestions fitted to the optimised time typed, after the region's Trim (the owner, 15 Oct).
async function loadAnalysis(again, withTarget = false) {
  const status = $('liteAnalysisStatus');
  if (!unit || !unit.active) { setStatus(status, 'Load a file to run automatic analysis.'); return; }
  const target = withTarget ? Number($('liteAnalysisTarget')?.value) : null;
  if (withTarget && !(target > 0)) { window.toast('Type the optimised time in seconds first, e.g. 20.', 'warn'); $('liteAnalysisTarget')?.focus(); return; }
  if (again || !analysisNow()) {
    setStatus(status, 'Analysing…');
    if ($('liteApplySuggest')) $('liteApplySuggest').disabled = true;
    try {
      const data = await get(unitUrl(unit.key, 'analysis') + (target ? `?target_s=${target}` : ''));
      analysis = { key: unit.key, file: unit.active, sha: (rec() || {}).sha256, data };
      if (withTarget) suggestionsPending = true;            // fitted to a new time: not in the scale regions yet
    } catch (err) {
      setStatus(status, `Automatic analysis failed: ${say(err)}`, 'err');
      return;
    }
  }
  renderAnalysis();
}

function badge(r) {
  return el('span', { class: `lite-badge ${r.scale_pct < 100 ? 'fast' : r.scale_pct > 100 ? 'slow' : 'keep'}`, text: r.action });
}

function renderAnalysis() {
  const tbody = $('liteSuggestions')?.querySelector('tbody');
  if (!tbody || !analysis) return;
  const rs = analysis.data.regions;
  tbody.replaceChildren(...rs.map((r, i) => {
    const time = el('td', { text: formatSeconds(r.original_s * r.scale_pct / 100) });
    const act = el('td', {}, badge(r));
    const input = el('input', { type: 'number', min: '1', max: '300', step: '1', value: String(r.scale_pct), 'aria-label': `Scale of region ${i + 1}`,
      oninput: (e) => {
        const v = Math.max(1, Math.min(300, Number(e.target.value) || 100));
        r.scale_pct = v;
        r.estimated_s = r.original_s * v / 100;
        r.action = v < 100 ? 'Shorter / faster' : v > 100 ? 'Longer / slower' : 'Unchanged';
        time.textContent = formatSeconds(r.estimated_s);
        act.replaceChildren(badge(r));
        suggestionsPending = true;
        dropFit();
        analysisMetrics();
        refreshShapes();
      } });
    return el('tr', {},
      el('td', {}, el('input', { type: 'checkbox', checked: r.selected, 'aria-label': `Use region ${i + 1}`,
        onchange: (e) => { r.selected = e.target.checked; suggestionsPending = true; dropFit(); analysisMetrics(); } })),
      el('td', { text: String(i + 1) }), el('td', { text: `${r.start.toLocaleString()} - ${r.end.toLocaleString()}` }),
      el('td', { text: formatSeconds(r.original_s) }), el('td', { text: r.motion_type }), el('td', { text: r.peak_guidance }),
      act, el('td', {}, input), time, el('td', { class: 'lite-reason', text: r.reason }));
  }));
  $('liteApplySuggest').disabled = !rs.length;
  analysisStatus();
  analysisMetrics();
  refreshShapes();
}

function analysisStatus() {
  const rs = analysis.data.regions, t = analysis.data.target;
  let text = `Automatic analysis complete: ${rs.length} review region(s) detected. Review the suggested scale values before applying them. `
    + 'Motion-only analysis does not replace collision or hardware validation.';
  if (t) {
    text = (t.reached
      ? `Suggestions fitted to ${formatSeconds(t.target_s)}: after Trim, the generated file comes to ${formatSeconds(t.estimated_s)} `
        + `(every region’s suggestion × ${t.factor}, so each keeps its guidance against the others). Apply them, then Confirm & Generate.\n`
      : `${formatSeconds(t.target_s)} cannot be reached with scales of 1–300 %: the nearest is ${formatSeconds(t.estimated_s)} `
        + `(${formatSeconds(t.shortest_s)} to ${formatSeconds(t.longest_s)} for this recording with these settings).\n`) + text;
  }
  setStatus($('liteAnalysisStatus'), text, t && !t.reached ? 'warn' : 'ok');
}

// The fitted time holds only for the suggestions as fitted, after Trim as it was then: a tick, a scale
// or a Trim setting changed, it is no longer said (Suggest for this time fits them again).
const TRIM_LABELS = ['Trim', 'Idle rolling window', 'Idle threshold', 'Minimum idle run'];
function dropFit() {
  if (!analysis || !analysis.data.target) return;
  analysis.data.target = null;
  analysisStatus();
}

function analysisMetrics() {
  const box = $('liteAnalysisMetrics');
  if (!box || !analysis) return;
  const rs = analysis.data.regions;
  const original = rs.reduce((a, r) => a + r.original_s, 0);
  const estimated = rs.reduce((a, r) => a + (r.selected ? r.original_s * r.scale_pct / 100 : r.original_s), 0);
  const saved = original - estimated, pct = original > 0 ? saved / original * 100 : 0;
  box.replaceChildren(metric('Detected review regions', String(rs.length)),
    metric('Faster / shorter candidates', String(rs.filter(r => r.selected && r.scale_pct < 100).length)),
    metric('Slower / longer candidates', String(rs.filter(r => r.selected && r.scale_pct > 100).length)),
    metric('Motion-only estimated time', formatSeconds(estimated)),
    metric('Estimated change', `${saved >= 0 ? '-' : '+'}${Math.abs(saved).toFixed(3)} s (${Math.abs(pct).toFixed(1)}%)`));
}

async function applySuggestions(asked = false) {
  if (!unit || !analysis) return;
  const chosen = analysis.data.regions.filter(r => r.selected);
  const status = $('liteAnalysisStatus');
  if (!chosen.length) { setStatus(status, 'No suggestions are selected. Tick at least one region to apply.', 'warn'); return; }
  if (regions.length && asked !== true) {
    const yes = await confirmDialog({ title: 'Replace the scale regions?', danger: false, ok: 'Replace them',
      body: 'Replace the current manual scaling regions with the selected automatic suggestions?' });
    if (!yes) return;
  }
  regions = chosen.map(r => ({ start: r.start, end: r.end, scalePct: r.scale_pct })).sort((a, b) => a.start - b.start);
  syncing = true;
  $('liteStageScale').checked = true;
  $('liteMode_manual').checked = true;                     // the suggestions are manual tuning's Scale
  syncing = false;
  await push('Automatic suggestions');
  suggestionsPending = false;
  setStatus(status, `${chosen.length} selected suggestion(s) were copied to the Scale regions. You can still edit every scale value manually before generation `
    + `(step ${REVIEW}, Review).`, 'ok');
}

// ── the plots (the HTML's plotTrajectory: the file's own joint values against the row number)

function trackData(which) {
  const c = unit.candidate_extra || {};
  const id = which === 'raw' ? `${unit.key}|raw|${unit.active}|${(rec() || {}).sha256 || ''}`
    : `${unit.key}|candidate|${c.made || ''}|${unit.candidate?.result?.lines || ''}`;
  if (!tracks.has(id)) {
    const p = get(`${unitUrl(unit.key, 'track')}?which=${which}&units=counts`);
    p.catch(() => tracks.delete(id));
    tracks.set(id, p);
    if (tracks.size > 12) tracks.delete(tracks.keys().next().value);
  }
  return tracks.get(id);
}

// The HTML's yRangeFor: both files' range, 5 % padding (at least 1).
function yRangeFor(...trs) {
  let mn = Infinity, mx = -Infinity;
  for (const tr of trs.filter(Boolean)) for (const q of tr.joints) for (const v of q) { if (v < mn) mn = v; if (v > mx) mx = v; }
  const pad = Math.max(1, (mx - mn) * 0.05);
  return [mn - pad, mx + pad];
}

function figure(tr, plotTitle, { selectable = false, yRange = null, pal = palette() } = {}) {
  const n = tr.lines;
  const x = Array.from({ length: n }, (_, i) => i + 1);
  const type = n > 15000 ? 'scattergl' : 'scatter';      // as the HTML: SVG unless the file is long
  const data = tr.joints.map((q, j) => ({ x, y: q, type, mode: 'lines', name: `J${j + 1}`, line: { color: pal.joints[j], width: 1.5 },
    hovertemplate: `J${j + 1}<br>Row %{x}<br>Value %{y}<extra></extra>` }));
  const layout = {
    title: { text: plotTitle, font: { size: 16, color: pal.ink } }, margin: { l: 80, r: 25, t: 55, b: 60 },
    xaxis: { title: { text: 'Sample / Row Number' }, rangeslider: { visible: false }, color: pal.ink, gridcolor: pal.line },
    yaxis: { title: { text: 'Joint Value' }, automargin: true, color: pal.ink, gridcolor: pal.line, ...(yRange ? { range: yRange } : {}) },
    hovermode: 'x unified', legend: { orientation: 'h', x: 0.5, xanchor: 'center', y: 1.08, font: { color: pal.ink } },
    dragmode: selectable ? 'select' : 'zoom', ...(selectable ? { selectdirection: 'h' } : {}),
    paper_bgcolor: pal.bg, plot_bgcolor: pal.bg, shapes: [],
    modebar: { orientation: 'v', bgcolor: 'transparent', color: pal.ink, activecolor: pal.blue },   // clear of the title beside the 3D
  };
  return { data, layout };
}

const PLOT_CONFIG = { responsive: true, scrollZoom: true, displaylogo: false, modeBarButtonsToRemove: ['lasso2d'] };

const rect = (a, b, colour, opacity) => ({ type: 'rect', xref: 'x', yref: 'paper', x0: a, x1: b, y0: 0, y1: 1,
  fillcolor: colour, opacity, line: { width: 0 }, layer: 'below' });

// The HTML's analysisShapes (the review regions, by their suggestion), and the scale regions set.
function reviewShapes() {
  const pal = palette(), out = [];
  if (analysisNow()) {
    for (const r of analysis.data.regions) out.push(rect(r.start, r.end, r.scale_pct < 100 ? pal.green : r.scale_pct > 100 ? pal.orange : pal.blue, 0.07));
  }
  for (const r of regions) out.push(rect(r.start, r.end, pal.violet, 0.12));
  return out;
}

async function drawReview() {
  const div = $('liteOrigPlot');
  if (!div || !unit || !unit.active) return;
  await loadPlotly();
  const tr = await trackData('raw');
  plotted.raw = tr;
  const r = rec();
  if ($('liteManualStart') && !$('liteManualStart').value) $('liteManualStart').value = '1';
  if ($('liteManualEnd') && !$('liteManualEnd').value) $('liteManualEnd').value = String(tr.lines);
  $('liteManualStart').max = String(tr.lines);
  $('liteManualEnd').max = String(tr.lines);
  const fig = figure(tr, `Original — ${(r && r.name) || ''}`, { selectable: true, yRange: yRangeFor(tr) });
  fig.layout.shapes = reviewShapes();
  await window.Plotly.react(div, fig.data, fig.layout, PLOT_CONFIG);
  fitPlot(div);
  if (!div.dataset.liteSel) {
    div.dataset.liteSel = '1';
    div.on('plotly_selected', (ev) => {                   // the HTML's box-select of rows: a region at 100 %
      if (!ev) return;
      let a, b;
      if (ev.range && ev.range.x) [a, b] = ev.range.x;
      else if (ev.points && ev.points.length) { const xs = ev.points.map(q => q.x); a = Math.min(...xs); b = Math.max(...xs); }
      if (Number.isFinite(a) && Number.isFinite(b)) {
        addRegion(Math.ceil(a), Math.floor(b));
        try { window.Plotly.restyle(div, { selectedpoints: [null] }); } catch (_) { /* nothing was dimmed */ }
      }
    });
    for (const t of ['pointerdown', 'pointermove', 'pointerup']) div.addEventListener(t, (e) => e.stopPropagation());
  }
  clickToSeek(div, 'review');
  setTotals('review');
  showMoment('review', false);
}

function refreshShapes() {
  const div = $('liteOrigPlot');
  if (!div || !div.data || !unit) return;
  window.Plotly.relayout(div, { shapes: reviewShapes() }).catch(() => {});
}

// A click on a plot is that moment (the owner, 16 Oct): the 3D goes to that frame, and the plot's
// cursor, the time and the play bar's timeline with it; Play goes on from there. The row clicked is
// that file's line, on the one time axis (Compare's two files share it).
function clickToSeek(div, kind) {
  if (!div || div.dataset.liteClick) return;
  div.dataset.liteClick = '1';
  div.on('plotly_click', (ev) => {
    const x = ev && ev.points && ev.points[0] && Number(ev.points[0].x);
    if (Number.isFinite(x)) scrub(kind, Math.max(0, x - 1) * CYCLE_S);
  });
}

function setCmp3d(v) {
  cmp3d = v === 'raw' ? 'raw' : 'candidate';
  pause();
  pb.at.compare = Math.min(pb.at.compare, totalFor('compare'));   // the cursor's range is the chosen file's
  setTotals('compare');
  showMoment('compare', false);
  if (currentStage === 'optimize' && unit && pipeStep === 6) loadSimulation(cmp3d);
}

// The moment on a plot: a thin line laid over it, moved without redrawing the plot (the owner,
// 15 Oct: playback lagged when the plot was laid out again at every frame). `x`: a row.
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

// ── 4 Review's regions (the HTML's addRegion, refreshRegionsTable)

function meaning(v) {
  return v > 100 ? `${(v - 100).toFixed(1)}% longer` : v < 100 ? `${(100 - v).toFixed(1)}% shorter` : 'unchanged';
}

function addRegion(start, end) {
  const status = $('liteRegionStatus');
  if (!unit || !unit.active) return;
  if (!stageSel().scale) { setStatus(status, 'Enable the Scale stage before selecting a scaling region.', 'warn'); return; }
  const n = (plotted.raw && plotted.raw.lines) || (rec() && rec().lines) || 1;
  const s = Math.max(1, Math.min(n, Math.round(Math.min(start, end))));
  const e = Math.max(1, Math.min(n, Math.round(Math.max(start, end))));
  if (!(e > s)) { setStatus(status, 'Selected region is too small. Choose at least two different row positions.', 'warn'); return; }
  if (regions.some(r => !(e < r.start || s > r.end))) {
    setStatus(status, 'The selected region overlaps an existing region. Delete or adjust the existing region first.', 'err');
    return;
  }
  regions.push({ start: s, end: e, scalePct: 100 });
  regions.sort((a, b) => a.start - b.start);
  renderRegions();
  refreshShapes();
  queuePush('Scaling regions');
}

function clearRegions() {
  regions = [];
  renderRegions();
  refreshShapes();
  queuePush('Scaling regions cleared');
}

// The HTML's invalidateProcessed as a scale is typed: the downloads wait for Confirm & Generate.
function typedStale() {
  for (const id of ['liteDlTxt', 'liteDlReport', 'liteDlPng', 'liteKeepBtn']) if ($(id)) $(id).disabled = true;
}

function renderRegions() {
  const tbody = $('liteRegions')?.querySelector('tbody');
  if (!tbody) return;
  const scaleOn = stageSel().scale;
  if (editingRegion()) {                                // a scale being typed keeps its field
    tbody.querySelectorAll('input').forEach((x) => { x.disabled = !scaleOn; });
  } else {
    tbody.replaceChildren(...regions.map((r, i) => {
      const mean = el('td', { text: meaning(r.scalePct) });
      return el('tr', {},
        el('td', { text: String(i + 1) }), el('td', { text: String(r.start) }), el('td', { text: String(r.end) }),
        el('td', {}, el('input', { type: 'number', min: '1', step: '1', value: String(r.scalePct), disabled: !scaleOn, 'aria-label': `Scale of region ${i + 1}`,
          oninput: (e) => { r.scalePct = Math.max(1, Number(e.target.value) || 100); mean.textContent = meaning(r.scalePct); typedStale(); },
          onchange: () => queuePush('Region scale') })),
        mean,
        el('td', {}, el('button', { class: 'btn red', text: 'Delete', onclick: () => { regions.splice(i, 1); renderRegions(); refreshShapes(); queuePush('Scaling regions changed'); } })));
    }));
  }
  const status = $('liteRegionStatus');
  if (!scaleOn) {
    setStatus(status, `Scale stage is disabled. ${regions.length ? `${regions.length} saved region(s) will be ignored.` : 'No scaling will be applied.'}`, 'warn');
  } else if (!regions.length) {
    setStatus(status, 'Scale stage is enabled, but no scaling regions are selected yet.');
  } else {
    setStatus(status, `${regions.length} scaling region(s) selected. Enter the target duration percentage for each region.`, 'ok');
  }
}

// ── the HTML's playback (Review: the recording; Compare: both files on one time cursor), the 3D with it

const rowAt = (lines, t) => Math.max(1, Math.min(lines || 1, 1 + Math.round(t / CYCLE_S)));
const optLines = () => (external ? external.rows.length : (plotted.opt && plotted.opt.lines) || 0);

// Compare's synchronized cursor runs over the file its simulation plays (the owner, 16 Oct: "Original selected:
// the progress bar uses only the original file's time ... Processed selected: ... 0 to the processed file's
// maximum"); the other plot's cursor stops at its own end.
function totalFor(kind) {
  const raw = (plotted.raw && plotted.raw.lines) || (rec() && rec().lines) || 0;
  if (kind === 'review' || cmp3d === 'raw') return dur(raw);
  // Processed: the file the 3D plays, the one made here; an external file is only plotted beside it
  const made = external && fresh() ? Number(unit.candidate?.result?.lines) || 0 : 0;
  return dur(made || optLines() || raw);
}

function setTotals(kind) {
  const total = totalFor(kind);
  pb.at[kind] = Math.min(pb.at[kind], total);              // a shorter file made since: the moment stays inside it
  const tl = $(`liteTimeline_${kind}`);
  if (tl) tl.max = String(total);
  if ($(`liteTotal_${kind}`)) $(`liteTotal_${kind}`).textContent = `${total.toFixed(3)} s`;
  if (tools3d && kind === (pipeStep === REVIEW ? 'review' : 'compare') && $(`liteTrack_${kind}`)) tools3d.track($(`liteTrack_${kind}`), total, true);
}

// `seek`: move the 3D too (not when the 3D itself plays: the page's Re-run Simulation).
function showMoment(kind, seek = true) {
  const t = pb.at[kind];
  if ($(`liteNow_${kind}`)) $(`liteNow_${kind}`).textContent = `${t.toFixed(3)} s`;
  if ($('simulationViewport')) frame($('simulationViewport')).time(`${t.toFixed(3)} s / ${totalFor(kind).toFixed(3)} s`);
  const tl = $(`liteTimeline_${kind}`);
  if (tl && document.activeElement !== tl) tl.value = String(t);
  const raw = (plotted.raw && plotted.raw.lines) || 0;
  if (kind === 'review') {
    const row = rowAt(raw, t);
    cursorAt($('liteOrigPlot'), row);
    const lab = $('liteRegionLabel');
    if (lab) {
      const i = analysisNow()
        ? analysis.data.regions.findIndex(x => row >= x.start && row <= x.end) : -1;
      lab.textContent = i >= 0 ? `Current region: ${i + 1} - ${analysis.data.regions[i].motion_type}` : 'Current region: -';
    }
    if (seek && sim.key === unit?.key && sim.which === 'raw') seekTo(t);
  } else {
    cursorAt($('liteCmpOrig'), rowAt(raw, t));
    cursorAt($('liteCmpProc'), rowAt(optLines(), t));
    if (seek && sim.key === unit?.key && sim.which === cmp3d) seekTo(t);
  }
}

function play(kind) {
  if (!unit || (kind === 'compare' && !optLines()) || !(plotted.raw && plotted.raw.lines)) return;
  const total = totalFor(kind);
  if (pb.playing && pb.kind !== kind) pause();
  if (pb.at[kind] >= total - 0.001) pb.at[kind] = 0;
  pb.kind = kind;
  pb.playing = true;
  pb.startPerf = performance.now() - pb.at[kind] * 1000;
  cancelAnimationFrame(pb.raf);
  const tick = (now) => {
    if (!pb.playing) return;
    pb.at[kind] = Math.min(total, (now - pb.startPerf) / 1000);
    showMoment(kind);
    if (pb.at[kind] >= total) { pause(); return; }
    pb.raf = requestAnimationFrame(tick);
  };
  pb.raf = requestAnimationFrame(tick);
}

function pause() {
  pb.playing = false;
  cancelAnimationFrame(pb.raf);
  pb.raf = 0;
}

function restart(kind) { pause(); pb.kind = kind; pb.at[kind] = 0; showMoment(kind); }

function scrub(kind, t) { pause(); pb.kind = kind; pb.at[kind] = Math.max(0, Math.min(totalFor(kind), t)); showMoment(kind); }

// The 3D playing on its own (V32's Re-run Simulation): the plots follow it.
function moveHead(line) {
  if (pb.playing || (pipeStep !== REVIEW && pipeStep !== 6)) return;
  const kind = pipeStep === REVIEW ? 'review' : 'compare';
  pb.kind = kind;
  pb.at[kind] = Math.max(0, line - 1) * CYCLE_S;
  showMoment(kind, false);
}

// ── 5 Generate

async function generate() {
  if (!unit) return;
  await flushPush();
  const status = $('liteProcessStatus');
  if (isManual() && !Object.values(stageSel()).some(Boolean)) {
    setStatus(status, 'Processing failed: Select at least one processing stage: Trim, Mean Smooth, or Scale.', 'err');
    return;
  }
  setStatus(status, isManual() ? `Processing selected stages: ${stageLabel()}…` : 'Processing: Time-optimal (TOPP-RA)…');
  const ok = await retune();
  renderGenerate();
  showResults();
  renderPipe();
  if (ok) {
    external = null;
    lastChange = null;
    if ($('liteExtFile')) $('liteExtFile').value = '';
    setStatus($('liteCompareStatus'), 'Comparison is ready. Step 6 is now unlocked for original-vs-optimized validation.', 'ok');
  }
}

function renderGenerate(changed = null) {
  const box = $('liteProcessedMetrics'), status = $('liteProcessStatus');
  if (!box || !unit) return;
  const cand = unit.candidate, r = cand && cand.result, ready = fresh();
  for (const id of ['liteDlTxt', 'liteDlReport', 'liteDlPng']) if ($(id)) $(id).disabled = !ready;
  if ($('liteGenerateBtn')) $('liteGenerateBtn').disabled = !unit.active;
  syncKeep();
  if (!unit.active) { setStatus(status, 'Load a file to begin.'); box.replaceChildren(); return; }
  if (!ready) {
    const why = changed || lastChange;
    box.replaceChildren();
    if (r) setStatus(status, staleText(why), 'warn');
    else if (!isManual()) setStatus(status, 'Ready. Time-optimal (TOPP-RA): click Confirm & Generate.');
    else setStatus(status, STAGE_LABELS.includes(why) ? `Ready. Selected stages: ${stageLabel()}. Click Confirm & Generate.`
      : `Ready. Selected stages: ${stageLabel()}. Configure any enabled stage, then click Confirm & Generate.`);
    return;
  }
  const rep = report();
  if (rep) {                                                   // manual: the HTML's figures
    const orig = dur(rep.sourceValidRows), fin = dur(rep.finalRows);
    box.replaceChildren(metric('Selected stages', rep.selectedStageLabel), metric('Original rows', rep.sourceValidRows.toLocaleString()),
      metric('Idle points removed', rep.idlePointsRemoved.toLocaleString()), metric('Rows after Trim', rep.rowsAfterTrim.toLocaleString()),
      metric('Rows after Mean Smooth', rep.rowsAfterMeanSmooth.toLocaleString()), metric('Scaling regions applied', String(rep.scalingRegions.length)),
      metric('Final rows', rep.finalRows.toLocaleString()), metric('Original time', formatSeconds(orig)), metric('Final time', formatSeconds(fin)),
      metric('Time saved', `${(orig - fin).toFixed(3)} s`), metric('Reduction', `${orig > 0 ? ((orig - fin) / orig * 100).toFixed(1) : '0.0'}%`));
    setStatus(status, r.message, rep.skippedRegions.length || !r.limits_ok ? 'warn' : 'ok');
  } else {                                                     // time-optimal
    const raw = rec(), orig = raw ? dur(raw.lines) : r.recorded_s;
    box.replaceChildren(metric('Optimiser', 'Time-optimal (TOPP-RA)'), metric('Original rows', raw ? Number(raw.lines).toLocaleString() : '-'),
      metric('Final rows', r.lines.toLocaleString()), metric('Original time', formatSeconds(orig)), metric('Final time', formatSeconds(r.achieved_s)),
      metric('Fastest within the limits', formatSeconds(r.t_min_s)), metric('Time saved', `${(orig - r.achieved_s).toFixed(3)} s`),
      metric('Reduction', `${orig > 0 ? ((orig - r.achieved_s) / orig * 100).toFixed(1) : '0.0'}%`));
    setStatus(status, `Generated with Time-optimal (TOPP-RA).\n${r.message}`, r.uploadable ? 'ok' : 'warn');
  }
}

// The HTML's word for what made the processed file stale (its invalidateProcessed messages).
const STAGE_LABELS = ['Trim', 'Mean Smooth', 'Scale'];
function staleText(label) {
  if (STAGE_LABELS.includes(label)) return `Stage selection changed. Current selection: ${stageLabel()}. Click Confirm & Generate again.`;
  if (label === 'Automatic suggestions') return 'Automatic optimization suggestions applied. Click Confirm & Generate to create the optimized trajectory.';
  if (label === 'Scaling regions cleared') return 'Scaling regions cleared. Click Confirm & Generate again.';
  if (/^(Scaling regions|Region scale)/.test(label || '')) return 'Scaling regions changed. Click Confirm & Generate again.';
  return 'Processing options changed. Click Confirm & Generate again.';
}

// The processed file's name as the HTML makes it: <recording>_<stages>_processed.txt.
function outputName() {
  const rep = report();
  if (rep && rep.outputName) return rep.outputName;
  const base = ((rec() && rec().name) || 'trajectory').replace(/\.[^.]+$/, '');
  return `${base}_toppra_processed.txt`;
}
const outputBase = () => outputName().replace(/_processed\.txt$/i, '');

export async function saveBlobAs(blob, name, types) {
  if (typeof window.showSaveFilePicker === 'function') {
    try {
      const h = await window.showSaveFilePicker({ suggestedName: name, types });
      const w = await h.createWritable();
      await w.write(blob);
      await w.close();
      return { name: h.name || name, picker: true };
    } catch (err) {
      if (err && err.name === 'AbortError') return null;
    }
  }
  const url = URL.createObjectURL(blob);
  const a = el('a', { href: url, download: name });
  document.body.append(a); a.click(); a.remove();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
  return { name, picker: false };
}

async function download(kind) {
  const status = $('liteProcessStatus');
  if (!fresh()) { setStatus(status, 'Generate first: Confirm & Generate makes the processed trajectory.', 'warn'); return; }
  const what = { txt: ['Processed trajectory', 'Save cancelled. Processed trajectory was not written.', 'processed TXT'],
    report: ['Processing report', 'Save cancelled. Processing report was not written.', 'processing report'],
    png: ['Processed plot', 'Save cancelled. Plot image was not written.', 'plot PNG'] }[kind];
  try {
    let saved;
    if (kind === 'txt') {
      const blob = await get(`${unitUrl(unit.key, 'file')}?which=candidate`, { raw: true });
      saved = await saveBlobAs(blob, outputName(), [{ description: 'TXT file', accept: { 'text/plain': ['.txt'] } }]);
    } else if (kind === 'report') {
      const rep = report();
      const body = rep                                   // the HTML's report as made with the file, and Lite's limits
        ? { ...rep, limits: { enforced: false, ok: unit.candidate.result.limits_ok, text: unit.candidate.result.limits_text, checks: unit.candidate.checks } }
        : { sourceFile: rec() && rec().name, optimiser: 'Time-optimal (TOPP-RA)', result: unit.candidate.result, checks: unit.candidate.checks,
          recipe: unit.candidate.recipe };
      saved = await saveBlobAs(new Blob([JSON.stringify(body, null, 2)], { type: 'application/json' }), `${outputBase()}_processing_report.json`,
        [{ description: 'JSON file', accept: { 'application/json': ['.json'] } }]);
    } else {
      await loadPlotly();                                // the HTML's Compare plot: the original's y-range shared
      const [orig, opt] = await Promise.all([trackData('raw'), trackData('candidate')]);
      const fig = figure(opt, `Optimized - ${outputName()}`, { yRange: yRangeFor(orig, opt), pal: printPalette() });
      const url = await window.Plotly.toImage({ data: fig.data, layout: fig.layout }, { format: 'png', width: 1600, height: 900, scale: 1 });
      saved = await saveBlobAs(await (await fetch(url)).blob(), `${outputBase()}_processed_J1_to_J7.png`, [{ description: 'PNG image', accept: { 'image/png': ['.png'] } }]);
    }
    if (!saved) { setStatus(status, what[1], 'warn'); return; }
    setStatus(status, saved.picker ? `${what[0]} saved as: ${saved.name}`
      : `${what[0]} download started as: ${saved.name}. To always choose the folder, enable your browser’s “Ask where to save each file” setting.`, 'ok');
  } catch (err) { setStatus(status, `Could not save ${what[2]}: ${say(err)}`, 'err'); }
}

// Keeping the processed file in the project, under its name, for Real Test (V32's Save Tuned File As…).
function keptVersion() {
  if (!fresh()) return null;
  return (unit.versions || []).find(v => v.sha256 === unit.candidate.track_sha256) || null;
}

function syncKeep() {
  const input = $('liteKeepName'), note = $('liteKeepNote'), btn = $('liteKeepBtn');
  if (!input) return;
  const ver = keptVersion();
  if (!input.dataset.typed || !input.value) input.value = (ver && ver.name) || outputName();
  if (btn) btn.disabled = !fresh();
  if (note) {
    note.textContent = !fresh() ? 'Generate first: the processed file is kept once it is made.'
      : ver ? `Kept as ${ver.version}: ${timed(ver.name || ver.file || '', ver.seconds)}. ${ctx.tuneKey ? 'It is a merge in S1 now: S1 sends it.' : 'Real Test sends this file.'}`
        : 'Not kept yet. “Simulation Looks Good” in Compare keeps it too.';
  }
  tuningSourceFileName = (rec() && rec().name) || tuningSourceFileName;
  tuningApplied = fresh();
  tunedOutputFileName = ver && ver.name ? ver.name : '';
}

async function keepAs(name, numbered) {
  unit = await post(unitUrl(unit.key, 'keep'), { name, numbered });
  ctx.units[unit.key] = unit;
  if ($('liteKeepName')) delete $('liteKeepName').dataset.typed;
  syncKeep();
  showResults();
  renderPipe();
}

// ── the limits (Lite): manual shows them, never enforces them; Time-optimal keeps every one

const CHECK_ORDER = ['L1', 'L2', 'L3', 'L4', 'L5', 'L6'];

function fmtCheck(c) {
  if (c.value == null) return c.detail || '';
  const v = Math.abs(c.value) >= 100 ? Math.round(c.value) : Number(c.value).toFixed(2);
  return `${v}${c.unit ? ' ' + c.unit : ''}${c.limit != null ? ` (limit ${c.limit}${c.unit ? ' ' + c.unit : ''})` : ''}`;
}

// The gates between this step and any execution, as the server judges them (arm, stage, limits, robot).
export function gateChips(u) {
  return el('div', { class: 'lite-gates' }, (u.gates || []).map(g =>
    el('span', { class: `lite-limit ${g.warn ? 'warn' : g.ok ? 'ok' : g.id === 'robot' || g.id === 'limits' ? 'na' : 'bad'}`, title: g.id },
      `${g.warn ? '!' : g.ok ? '✓' : '✕'} ${g.text}`)));
}

function showResults(job = null) {
  const box = $('liteResults');
  if (!box || !unit) return;
  const cand = unit.candidate, r = cand && cand.result;
  const top = el('div', { class: 'lite-results-head' }, el('span', { class: 'badge violet', text: 'LIMITS' }));
  if (job && job.state === 'running') {
    top.append(el('b', { text: ` Processing… ${job.stage || ''}` }));
    box.replaceChildren(top);
    return;
  }
  if (!r || !unit.fresh) {
    top.append(el('b', { text: !unit.active ? ' No recording yet.' : ' Checked on the file Confirm & Generate makes.' }));
    box.replaceChildren(top, gateChips(unit));
    return;
  }
  const manualFile = r.optimiser !== 'toppra';
  const ok = manualFile ? r.limits_ok : r.uploadable;
  top.append(el('b', { class: ok ? 'lite-ok' : manualFile ? 'lite-warn' : 'lite-bad',
    text: ok ? ' Every limit passes ✓' : manualFile ? ' Some limits are broken: shown, not enforced (the hardware decides)' : ' Not ready for the robot' }));
  const checks = (cand.checks && cand.checks.checks) || [];
  const chips = el('div', { class: 'lite-limits' }, CHECK_ORDER.map((id) => {
    const c = checks.find(x => x.id === id);
    if (!c) return null;
    const cls = c.passed === false ? (manualFile ? 'warn' : 'bad') : c.passed ? 'ok' : 'na';
    return el('span', { class: `lite-limit ${cls}`, title: c.detail || '' },
      `${c.passed === false ? (manualFile ? '!' : '✕') : c.passed ? '✓' : '–'} ${id} ${c.name}: ${fmtCheck(c)}`);
  }), checks.filter(c => !c.hard && c.passed === false).map(c => el('span', { class: 'lite-limit warn', title: c.detail || '' }, `! ${c.id} ${c.name}`)));
  const note = el('p', { class: 'mini-note', text: manualFile
    ? 'Manual tuning: Lite reports these limits and never holds the file to them. Sending a file that breaks one asks you once; the robot’s own protections still apply.'
    : 'Time-optimal: a file that fails a limit is never kept or sent.' });
  const parts = [top, gateChips(unit), chips, note];
  if (!manualFile) {
    const grid = el('div', { class: 'lite-results-grid' },
      el('div', {}, el('span', { text: 'Fastest safe' }), el('b', { text: formatSeconds(r.t_min_s) }),
        el('small', { text: (r.binding && r.binding[0] && r.binding[0].text) || 'within every limit' })));
    if (r.target_s != null && !r.met) {
      grid.append(el('div', {}, el('span', { text: 'Target' }), el('b', { text: formatSeconds(r.target_s) }),
        el('button', { class: 'btn light', text: `Use ${r.t_min_s.toFixed(2)} s`, onclick: () => useTime(r.t_min_s) })));
    }
    parts.splice(2, 0, grid);
    if ((r.hints || []).length) parts.push(el('ul', { class: 'lite-hints' }, r.hints.map(h => el('li', { text: h }))));
  }
  box.replaceChildren(...parts);
}

function useTime(s) {
  syncing = true;
  $('liteTopMode').value = 'target';
  $('liteTopTarget').disabled = false;
  $('liteTopTarget').value = s.toFixed(2);
  syncing = false;
  push(`Target ${s.toFixed(2)} s`);
}

// ── 6 Compare (the HTML's updateComparisonView)

function parseTrajectoryText(text) {                  // the HTML's parseTrajectoryFile
  const rows = [], skipped = [];
  text.split(/\r?\n/).forEach((raw, idx) => {
    const line = raw.trim();
    if (!line) return;
    try {
      const obj = JSON.parse(line);
      if (!obj || !Array.isArray(obj.point) || obj.point.length !== 7 || !obj.point.every(Number.isFinite)) { skipped.push(idx + 1); return; }
      rows.push(obj.point.map(Number));
    } catch (_) { skipped.push(idx + 1); }
  });
  if (rows.length < 2) throw new Error('The file must contain at least two valid 7-joint rows.');
  return { rows, skipped };
}

async function loadExternal(ev) {
  const file = ev.target.files && ev.target.files[0];
  const status = $('liteCompareStatus');
  if (!file) return;
  try {
    setStatus(status, 'Loading optimized comparison file...');
    const parsed = parseTrajectoryText(await file.text());
    external = { name: file.name, ...parsed };
    pause();
    pb.at.compare = 0;
    await renderCompare();
    setStatus(status, `External optimized file loaded: ${file.name}. Original source remains unchanged. The 3D plays the generated file.`,
      parsed.skipped.length ? 'warn' : 'ok');
  } catch (err) {
    setStatus(status, `Optimized comparison load failed: ${err.message}`, 'err');
  }
}

async function useGenerated() {
  if (!fresh()) return;
  external = null;
  if ($('liteExtFile')) $('liteExtFile').value = '';
  pause();
  pb.at.compare = 0;
  await renderCompare();
  setStatus($('liteCompareStatus'), 'Comparison switched back to the generated optimized trajectory.', 'ok');
}

async function renderCompare() {
  const box = $('liteCompareMetrics');
  if (!box || !unit || !(fresh() || external)) return;
  await loadPlotly();
  const orig = await trackData('raw');
  plotted.raw = orig;
  let opt;
  if (external) opt = { lines: external.rows.length, joints: Array.from({ length: 7 }, (_, j) => external.rows.map(r => r[j])) };
  else opt = await trackData('candidate');
  plotted.opt = opt;
  const rep = external ? null : report();
  const originalSeconds = dur(orig.lines), optimizedSeconds = dur(opt.lines);
  const saved = originalSeconds - optimizedSeconds, pct = originalSeconds > 0 ? saved / originalSeconds * 100 : 0;
  box.replaceChildren(metric('Original duration', formatSeconds(originalSeconds)), metric('Optimized duration', formatSeconds(optimizedSeconds)),
    metric('Time saved', `${saved >= 0 ? saved.toFixed(3) : '-' + Math.abs(saved).toFixed(3)} s`), metric('Percentage reduction', `${pct.toFixed(1)}%`),
    metric('Modified scale regions', external ? '-' : String(rep ? rep.scalingRegions.length : 0)));
  const yRange = yRangeFor(orig, opt);
  const r = rec();
  const optName = external ? external.name : outputName();
  for (const [id, tr, t] of [['liteCmpOrig', orig, `Original - ${(r && r.name) || ''}`], ['liteCmpProc', opt, `Optimized - ${optName}`]]) {
    const fig = figure(tr, t, { yRange });
    await window.Plotly.react($(id), fig.data, fig.layout, PLOT_CONFIG);
    fitPlot($(id));
    clickToSeek($(id), 'compare');
  }
  const tbody = $('liteCompareRegions')?.querySelector('tbody');
  if (tbody) {
    const rows = rep ? rep.scalingRegions : [];
    tbody.replaceChildren(...(rows.length ? rows.map((g, i) => el('tr', {},
      el('td', { text: String(i + 1) }), el('td', { text: `${g.originalStartRow} - ${g.originalEndRow}` }), el('td', { text: String(g.scalePct) }),
      el('td', { text: String(g.inputRows) }), el('td', { text: String(g.outputRows) }),
      el('td', { text: g.scalePct < 100 ? `${(100 - g.scalePct).toFixed(1)}% shorter` : g.scalePct > 100 ? `${(g.scalePct - 100).toFixed(1)}% longer` : 'unchanged' })))
      : [el('tr', {}, el('td', { colspan: '6', class: 'mini-note', text: external
        ? 'External optimized file loaded; region-level modification metadata is not available in the TXT file.' : 'No scaling regions were applied.' }))]));
  }
  if ($('liteUseGenerated')) $('liteUseGenerated').disabled = !fresh();
  setTotals('compare');
  showMoment('compare', false);
}

// ── the simulation beside the plots

// The 3D's tools (view3d.js): the commode's state shown, the collision check of the file playing (the
// recording in Review; the processed file, or the original, in Compare), its links in red.
let tools3d = null;
function tools() {
  const host = $('simulationViewport');
  if (!tools3d && host) {
    tools3d = new Tools({
      what: 'the file in 3D',
      check: async (phase, progress) => {
        const which = sim.which || 'raw';
        const r = await checkJob(post, unitUrl(unit.key, 'collisions'), { which, phase, task: TASKS[currentTask]?.label || null }, progress);
        return { result: r, spans: spansOf(r) };
      },
      seek: (t) => scrub(pipeStep === REVIEW ? 'review' : 'compare', t),
      changed: (phase) => { if (unit) scene(phase || unit.phase, undefined, sim.place || null).catch(() => {}); },
    });
  }
  if (tools3d && host) { tools3d.mount(host); frame(host); }       // V32's caption is its title; the time in the view
  return tools3d;
}

async function loadSimulation(which) {
  const host = $('simulationViewport');
  if (!host || !unit) return;
  const t3 = tools();
  const whose = ctx.tuneKey ? "the merge's" : "the region's";
  if (elsewhere()) { invalidate(); return; }              // the Viewer, the full sequence or a picture has the 3D: later
  if (!unit.active || (which === 'candidate' && !fresh())) {
    unload(Object.assign(new Error(unit.active ? 'Generate first: the processed file is played once it is made.' : 'This region has no recording yet.'),
      { hint: unit.active ? 'Confirm & Generate in step 5.' : 'Record it, or use an existing or loaded file.', key: unit.key }));
    t3.load(`${unit.key}|standing`, { phase: unit.phase, text: whose, enabled: false });
    await showStanding(host, unit, undefined, t3.phase).catch((err) => say3d(host, `The 3D view could not start: ${say(err)}`, err));
    return;
  }
  const progress = which === 'candidate' ? unit.candidate_extra && unit.candidate_extra.progress : null;
  if (!host.querySelector('canvas')) say3d(host, 'Loading the 3D view…');   // the first time: the robot's meshes come first
  const kind = pipeStep === REVIEW ? 'review' : 'compare', at = pb.at[kind];   // loading rewinds the 3D: the view keeps its moment
  t3.load(fileKey(unit, which), { phase: unit.phase, text: whose });
  try {
    if (!(await showFile(host, unit, which, progress, undefined, t3.phase))) return;     // another file was asked for meanwhile
    onMoment((t) => t3.at(t == null ? -1 : t));
    setTotals(kind);
    window.resetSimulationPlayback(false);
    pb.at[kind] = Math.min(at, totalFor(kind));
    seekTo(pb.at[kind]);
    if (pipeStep === (kind === 'review' ? REVIEW : 6)) showMoment(kind, false);
    $('simStatus').textContent = (which === 'candidate' ? 'The processed file' : 'The recording as taught')
      + ` in 3D, staged first (${sim.tl.stage.text}): Play / Pause / Restart and the timeline above move it with the plot.`;
    $('simStatus').className = 'statusbar ok';
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
  const key = unit.key;
  const refused = (err) => {
    if (sim.key === key || !sim.tl) unload(Object.assign(err, { key }));
    setStatus($('liteProcessStatus'), `Processing failed: ${say(err)}`, 'err');
    tell(err);
    return false;
  };
  tuning = (async () => {
    try {
      await flushPush();                                 // the settings on screen are the ones tuned
      const job = await post(unitUrl(key, 'optimise'), {});
      if (job && job.error) return refused(Object.assign(new Error(job.error), { hint: job.hint, code: job.code }));
      const done = await watchJob(job, (j) => {
        showResults(j);
        setStatus($('liteProcessStatus'), isManual() ? `Processing selected stages: ${stageLabel()}…`
          : `Processing: Time-optimal (TOPP-RA)… ${j.stage || ''}`.trim());
      });
      unit = await loadUnit(key);
      if (done.state !== 'done') return refused(Object.assign(new Error(done.error || 'Processing stopped.'), { hint: done.hint }));
      invalidate();
      return true;
    } catch (err) { return refused(err); } finally { tuning = null; }
  })();
  return tuning;
}

// A merge sent from Merge + Sequence S1 (the owner, 16 Oct: "transfer the file to the Tune +
// Simulation Workspace"): Tune opens the merged file as its source (a unit of its own on the
// server), says so above its steps, and what is kept goes back to S1 as a new merge, never to Real
// Test. Choosing a task, an arm or another stage goes back to the region.
const SAVE_TO_S1 = 'Simulation Looks Good → Save to S1';
let footerText = null;

// A merge has a screen of its own (the owner, 16 Oct: "the merged file should not be confused by the region
// tasks, so a different UI without the task marks should be used, but the pipeline and algorithms should be
// the same"): the region loop's task board, region cards and stage strip give way to the merge's header; the
// steps below are Tune's own.
function mergeBanner() {
  const p = pane();
  if (!p) return;
  let b = $('liteTuneMerge');
  if (!b) { b = el('div', { id: 'liteTuneMerge', class: 'lite-tune-merge' }); p.prepend(b); }
  const m = ctx.tuneKey ? ctx.tuneMerge : null;
  b.hidden = !m;
  document.body.classList.toggle('lite-merge-tune', !!m);
  if (m) {
    const parts = (m.parts || []).map(x => x.name);
    b.replaceChildren(
      el('div', {}, el('div', { class: 'eyebrow', text: '03 · Merge + Sequence · S1' }),
        el('h2', { text: `Tune + Simulation · ${m.side.toUpperCase()} merge ${m.name}` }),
        el('p', { class: 'mini-note', text: `${parts.length ? `Its files: ${parts.join(' → ')}${m.joins && m.joins.length ? ` (${m.joins.length} join${m.joins.length === 1 ? '' : 's'})` : ''}. ` : ''}`
          + 'The same steps and algorithms as a region\u2019s file; what you keep goes back to S1 as a new merge, never to a region.' })),
      el('button', { class: 'btn light', type: 'button', text: 'Back to S1', onclick: () => backToS1(null) }));
    document.querySelectorAll('.navbtn[data-view]').forEach(n => n.classList.toggle('active', n.dataset.view === 'sequence'));
  }
  const foot = p.querySelector('button[onclick="setStage(\'real\')"]');
  if (foot) {
    if (footerText === null) footerText = foot.innerHTML;
    if (m) foot.textContent = SAVE_TO_S1; else foot.innerHTML = footerText;
  }
}

function leaveMerge() {
  if (!ctx.tuneKey) return;
  ctx.tuneKey = null;
  ctx.tuneMerge = null;
  mergeBanner();
}

function backToS1(merge) {
  const m = ctx.tuneMerge;
  leaveMerge();
  pause();
  unload();                                              // the merge's file never plays for the region
  unit = null;
  document.dispatchEvent(new CustomEvent('lite:open-s1', { detail: merge ? { side: merge.side, file: merge.file } : (m || {}) }));
  if (currentStage === 'optimize') enter();              // Tune is the region's again when it is next shown
}

async function enter() {
  const s = ctx.tuneKey ? { key: ctx.tuneKey } : step();
  mergeBanner();
  if (!s || !ctx.pid) return;
  await flushPush();                                     // the step left keeps what was typed there
  addControls();
  if (sim.key !== s.key) unload();                      // another step's file never plays here
  try { unit = await loadUnit(s.key); } catch (err) { tell(err); return; }
  if (!analysis || analysis.key !== unit.key) analysis = null;
  external = null;
  lastChange = null;
  if (!analysis) suggestionsPending = false;              // kept with its analysis: still asked before Generate, Compare, Real Test
  pause();
  pb.at.review = pb.at.compare = 0;
  plotted.raw = plotted.opt = null;
  if ($('liteManualStart')) $('liteManualStart').value = '';
  if ($('liteManualEnd')) $('liteManualEnd').value = '';
  writeControls(settingsOf(unit));
  showResults();
  syncKeep();
  const n = stepOf.get(unit.key) || (fresh() ? 5 : 1);
  pipeStep = 0;                                          // whichever step it is, it is drawn afresh
  await goStep(unlocked(n) ? n : 1, false);
}

export function installTune() {
  onPlayhead(moveHead);
  document.addEventListener('lite:step', () => {                // another task or arm: the region again
    if (!ctx.tuneKey) return;
    leaveMerge();
    if (currentStage === 'optimize') enter();
  });

  const pageSetStage = window.setStage;
  window.setStage = async function setStage(s, scroll = true) {
    if (s === 'real' && currentStage === 'optimize' && ctx.tuneKey && ctx.pid) {     // a merge goes back to S1
      const ok = await keep();
      if (!ok) return;
      let back = null;
      try { back = (await post(unitUrl(ctx.tuneKey, 'keep'), {})).merge; } catch (err) { tell(err); return; }
      window.toast(`${back.name} is saved in S1 as a new ${back.side.toUpperCase()} merge.`, 'good');
      backToS1(back);
      return;
    }
    if (s === 'real' && currentStage === 'optimize' && step() && ctx.pid) {
      const ok = await keep();
      if (!ok) return;
    }
    if (ctx.tuneKey && s !== 'optimize' && s !== 'simulate') leaveMerge();
    const was = currentStage;
    // Prepare stays open to the arm being taught (the owner, 14 Oct: what was forgotten there is set
    // there and carries on). The page refuses it once the arm is locked, its own Record Again
    // included; it is entered as Prepare, and the arm stays locked, so the sides never swap.
    if (s === 'prepare' && armSelectionLocked && was !== 'prepare') currentStage = 'prepare';
    await pageSetStage(s, scroll);                       // Prepare's gate may hold it (prepare.js)
    if (currentStage !== was) { pause(); release(false); }   // the simulation stops with its stage
    if (currentStage === 'optimize' && (s === 'optimize' || s === 'simulate')) enter();
    document.dispatchEvent(new CustomEvent('lite:stage', { detail: currentStage }));
  };

  // V32's footer: Re-run Simulation plays the processed file again in 3D, staged first (generated
  // first if it is stale); the plots follow it.
  const pageRerun = window.rerunSimulationAndFocus;
  window.rerunSimulationAndFocus = async function rerunSimulationAndFocus() {
    if (unit && !fresh()) { const ok = await retune(); if (!ok) return; renderGenerate(); }
    if (pipeStep !== 6 && unlocked(6)) await goStep(6, false);
    pause();
    pageRerun();
  };

  window.applyTuningToActiveTrajectory = function applyTuningToActiveTrajectory() { generate(); };
  window.saveTunedTrajectoryAs = async function saveTunedTrajectoryAs() {
    if (!unit) return;
    await flushPush();
    if (!fresh()) { window.toast('Generate first: Confirm & Generate makes the processed file.', 'warn'); return; }
    let name = ($('liteKeepName')?.value || '').trim() || outputName();
    if (!/\.txt$/i.test(name)) name += '.txt';
    const typed = !!$('liteKeepName')?.dataset.typed;
    try { await keepAs(name, !typed); } catch (err) { tell(err); return; }
    const ver = keptVersion();
    if (ctx.tuneKey && unit.merge) { window.toast(`${unit.merge.name} is saved in S1 as a new ${unit.merge.side.toUpperCase()} merge.`, 'good'); return; }
    window.toast(`${(ver && ver.name) || name} is kept in the project: Real Test uploads it.`, 'good');
  };

  document.addEventListener('change', (e) => {
    if (!pane()?.contains(e.target) || syncing) return;
    const id = e.target.id;
    if (id === 'liteTopMode') $('liteTopTarget').disabled = e.target.value !== 'target';
    if (e.target.name === 'liteMode') {
      $('liteManualSetup').hidden = e.target.value === 'toppra';
      $('liteToppraSetup').hidden = e.target.value !== 'toppra';
      queuePush(e.target.value === 'toppra' ? 'Time-optimal (TOPP-RA)' : 'Manual tuning');
      return;
    }
    if (['liteStageTrim', 'liteStageSmooth', 'liteStageScale'].includes(id)) updateStageUI();
    const labels = { liteStageTrim: 'Trim', liteStageSmooth: 'Mean Smooth', liteStageScale: 'Scale', liteIdleWindow: 'Idle rolling window',
      liteIdleThreshold: 'Idle threshold', liteIdleRun: 'Minimum idle run', liteSmoothWindow: 'Mean smoothing window',
      liteTopMode: 'Total time', liteTopTarget: 'Target time' };
    if (labels[id]) queuePush(labels[id]);
  });

  document.addEventListener('lite:speed', () => {           // the move to the start at the new speed: the 3D again
    if (currentStage === 'optimize' && unit && (pipeStep === REVIEW || pipeStep === 6)) loadSimulation(pipeStep === 6 ? cmp3d : 'raw');
  });

  document.addEventListener('lite:viewer-closed', () => {   // the 3D view is back from the Trajectory Viewer
    if (currentStage === 'optimize' && unit && (pipeStep === REVIEW || pipeStep === 6)) loadSimulation(pipeStep === 6 ? cmp3d : 'raw');
  });

  const pageTheme = window.toggleTheme;
  window.toggleTheme = function toggleTheme() {
    pageTheme();
    if (currentStage === 'optimize' && pipeStep === REVIEW) drawReview().catch(() => {});
    if (currentStage === 'optimize' && pipeStep === 6) renderCompare().catch(() => {});
  };
}

// Keeps the processed file as the version to upload (the page's "Simulation Looks Good → Real Test"),
// under the HTML's name (numbered when the project already has it).
async function keep() {
  if (!unit) return true;
  await flushPush();
  const s = await settleSuggestions('the file Real Test gets is made from the scale regions');
  if (!s.go) return false;
  if (!fresh()) {
    const ok = await retune();
    if (!ok) return false;
  }
  if (keptVersion()) return true;
  const typed = !!$('liteKeepName')?.dataset.typed;
  let name = (typed && $('liteKeepName').value.trim()) || outputName();
  if (!/\.txt$/i.test(name)) name += '.txt';
  try {
    await keepAs(name, !typed);
    return true;
  } catch (err) { tell(err); showResults(); return false; }
}
