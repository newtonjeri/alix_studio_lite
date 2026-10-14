// Prepare and Setup & Safety (R7, R4). Each arm step has a SETUP on the server: the pole height and
// hand pose it is taught with (from its existing recording's record or the region catalogue).
// Prepare shows it; Apply Pole / Apply Hand keep the page's values as the setup and stage them:
// in Real on the robot (pole, then hand, each read back, after a confirmation); in Hardware sim
// emulated, as nothing moves there. The hand sent is the one on screen: the preset, or with Custom
// the six channels, where an empty channel is held (left where it is).
// Drag teaching starts only with the arm in that setup, so every recording knows its true stage,
// and every execution stages that stage first (staging.py). The start-pose moves wait for the arm;
// with the live link the start poses are the controller's global points (MoveJ or MoveL).

import { post, say } from './api.js';
import { REAL_CHECKS, confirmDialog, ctx, el, jobLine, loadUnit, robotCall, step, tell, unitUrl, watchJob } from './ctx.js';
import { hasController, loadPoints, matches, movePoint } from './points.js';
import { liteMode } from './status.js';

const $ = (id) => document.getElementById(id);
const GRIP = [0, 0, 0, 0, 150, 1000], OPEN = [1000, 1000, 1000, 1000, 1000, 1000];
const PRESETS = { 'Grip / Cleaning': GRIP, 'All Open': OPEN };
let lastKey = null;
let lastMoveError = null;

const handInputs = () => [...document.querySelectorAll('#handFields input')];

function handFromPage() {
  const preset = PRESETS[$('handPreset')?.value];
  return preset ? preset.slice() : handInputs().map(x => (x.value === '' ? null : Number(x.value)));
}

function fillHand(hand) {
  handInputs().forEach((x, i) => { x.value = hand[i] == null ? '' : hand[i]; x.placeholder = hand[i] == null ? 'held' : ''; });
}

function showSetup(setup) {
  if (!setup) return;
  if ($('poleHeight')) $('poleHeight').value = setup.pole_mm;
  fillHand(setup.hand);
  const preset = $('handPreset');
  const k = JSON.stringify(setup.hand);
  if (preset) preset.value = Object.keys(PRESETS).find(n => JSON.stringify(PRESETS[n]) === k) || 'Custom';
  let note = $('liteSetupNote');
  if (!note) {
    note = Object.assign(document.createElement('div'), { id: 'liteSetupNote', className: 'mini-note lite-setup-note' });
    $('stage-prepare')?.querySelector('.grid')?.after(note);
  }
  note.textContent = `Region setup: ${setup.text} (from ${setup.source}). Every run stages its recording's own pole and hand first; drag teaching starts only with the arm in this setup.`;
}

async function refresh() {
  const s = step();
  if (!s || !ctx.pid || s.later) return;
  if (s.key !== lastKey) { lastKey = s.key; showSetup(s.setup); }
  try { const u = await loadUnit(s.key); showSetup(u.setup); } catch (_) { /* the page still works */ }
}

// Back in Prepare after teaching (the owner, 14 Oct: what was forgotten is set here and carries on to
// the later stages). The hand can be changed for the recording already made, when the operator says
// so (it is tuned and kept again); the pole cannot: the arm is mounted on the pole, so a recording
// runs only at the height it was taught at, and the new height is for the next recording.
async function afterTeaching(s, u, set) {
  const rec = (u.recordings || []).find(r => r.file === u.active);
  if (!rec || !rec.stage || !armSelectionLocked) return;
  if (Math.abs(Number(rec.stage.pole_mm) - set.pole_mm) > 5) {
    window.toast(`${rec.name} was taught at pole ${rec.stage.pole_mm} mm. The arm moves with the pole, so it runs only at that `
      + `height: ${set.pole_mm} mm is for the next recording (Record Again).`);
    return;
  }
  const same = rec.stage.hand.every((v, i) => set.hand[i] == null || (v != null && Math.abs(v - set.hand[i]) <= 30));
  if (same) return;
  const yes = await confirmDialog({ title: `Use this hand for ${rec.name}?`, danger: false, ok: 'Use it for this recording',
    body: `${rec.name} was taught with the hand ${rec.stage.hand_text}. From now on every run of it stages the hand set here. `
      + 'Tune it again and keep it: Real Test runs the kept version.' });
  if (!yes) { window.toast(`The hand set here is for the next recording; ${rec.name} keeps its own.`); return; }
  try {
    ctx.units[s.key] = await post(`${unitUrl(s.key, 'recordings')}/${rec.file}/hand`, { hand: set.hand, confirmed: true });
    window.toast(`${rec.name} now runs with this hand. Tune it again, then keep it.`);
  } catch (err) { tell(err); }
}

async function applySetup(kind) {
  const s = step();
  if (!s || !ctx.pid) return;
  const mode = await liteMode();
  const body = { pole_mm: Number($('poleHeight').value), hand: handFromPage(), apply: mode === 'hwsim' || mode === 'real' };
  try {
    const out = await robotCall(unitUrl(s.key, 'setup'), body, { what: `Moving the ${s.arm.toUpperCase()} pole and hand` });
    showSetup(out.unit.setup);
    if (!out.job) {
      window.toast(`Setup kept for ${s.display}: ${out.unit.setup.text}. In Visual sim nothing moves.`);
      await afterTeaching(s, out.unit, body);
      return;
    }
    const done = await watchJob(out.job, (j) => { const n = $('liteSetupNote'); if (n) n.textContent = jobLine(j); });
    showSetup(out.unit.setup);
    if (done.state !== 'done') {                       // the arm is NOT in its setup: say so where it shows
      const said = `Not staged: ${done.error || 'staging stopped.'} ${done.hint || ''}`;
      $('liteSetupNote').textContent = said;
      $('liteSetupNote').classList.add('lite-bad');
      window.toast(said);
      return;
    }
    $('liteSetupNote').classList.remove('lite-bad');
    window.toast(`${s.arm.toUpperCase()} staged: ${out.unit.setup.text} `
      + (mode === 'real' ? '(read back).' : '(emulated: in Hardware sim nothing moves).'));
    window.rcSyncGuidedValues();                       // the Advanced panel shows the same values (nothing is sent)
    await afterTeaching(s, out.unit, body);
  } catch (err) { tell(err); }
}

// ── the start poses as the controller's global points (the owner, 14 Oct): with the arm linked, its
// start-pose list (Setup & Safety, and Redo Preparation, which copies it) holds the global points read
// from its controller, a search field narrows it, and MoveJ or MoveL takes the arm there at 20 %.
// The stand-in keeps the page's own profiles (it has no controller).

const POINT_SPEED = 20;                          // % : the control program's default for a global point
const ownOptions = {};                           // each arm's list as the page made it

function filterOptions(sel, q) {
  if (!sel) return;
  let first = null;
  for (const o of sel.options) {
    o.hidden = !matches(o.textContent, q);
    if (!o.hidden && !first) first = o;
  }
  if (sel.selectedOptions[0]?.hidden && first) sel.value = first.value;
}

// A search field above a start-pose list and the motion beside it, made once.
function pointControls(sel, id) {
  if (!sel || $(`${id}Search`)) return;
  const search = el('input', { type: 'search', id: `${id}Search`, class: 'lite-point-search', placeholder: 'Search the global points',
    'aria-label': 'Search the global points', oninput: (e) => filterOptions(sel, e.target.value) });
  const motion = el('select', { id: `${id}Motion`, class: 'lite-point-motion', 'aria-label': 'Motion to the point',
    title: `How the arm moves to a global point (at ${POINT_SPEED} %)` },
  el('option', { value: 'movej', text: 'MoveJ' }), el('option', { value: 'movel', text: 'MoveL' }));
  (sel.closest('.inline') || sel).before(search);
  sel.after(motion);
}

async function fillPoses(side) {
  const sel = $(`${side}PoseSelect`);
  if (!sel) return;
  if (!(side in ownOptions)) ownOptions[side] = sel.innerHTML;
  pointControls(sel, `lite_${side}_pose`);
  const linkedArm = hasController(side);
  for (const id of [`lite_${side}_poseSearch`, `lite_${side}_poseMotion`]) if ($(id)) $(id).hidden = !linkedArm;
  if (!linkedArm) { if (sel.querySelector('optgroup.lite-points')) sel.innerHTML = ownOptions[side]; return; }
  try {
    const d = await loadPoints(side);
    const keep = sel.value;
    sel.replaceChildren(d.points.length
      ? el('optgroup', { class: 'lite-points', label: `${side.toUpperCase()} controller · ${d.points.length} global points` },
        d.points.map(pt => el('option', { value: `gp:${pt.name}`, text: pt.name })))
      : el('optgroup', { class: 'lite-points', label: d.note || 'No global points on this controller' }));
    if ([...sel.options].some(o => o.value === keep)) sel.value = keep;
    filterOptions(sel, $(`lite_${side}_poseSearch`)?.value);
  } catch (err) {
    const e = $(`${side}PoseStatus`);
    if (e) e.textContent = `The ${side.toUpperCase()} controller's global points could not be read: ${say(err)}`;
  }
}

async function move(url, body, what, statusEl) {
  const job = await robotCall(url, body, { what });
  const done = await watchJob(job, (j) => { if (statusEl) statusEl.textContent = jobLine(j); });
  if (done.state !== 'done') throw Object.assign(new Error(done.error || `${what} stopped.`), { hint: done.hint });
  return done;
}

export function installPrepare() {
  window.demoApply = function demoApply(kind) {
    if (!ctx.pid) { window.toast('Open a project first: Apply keeps the setup of its region.'); return; }
    applySetup(kind);
  };

  // Move to the selected start pose. BOTH asks one confirmation in Real, then sends both arms, and
  // says what each arm did: one arm refused or stopped never hides that the other one moved.
  window.demoMove = async function demoMove(which) {
    if (!isSetupReady()) { window.toast('Complete all physical preparation checks first.'); return false; }
    const arms = which === 'BOTH' ? ['left', 'right'] : [String(which).toLowerCase()];
    const mode = await liteMode();
    let extra = {};
    if (arms.length > 1 && mode === 'real') {
      const yes = await confirmDialog({ title: 'Real robot: both arms will move', ok: 'Yes, move both arms', list: REAL_CHECKS,
        body: 'The LEFT and the RIGHT arm move to their selected start poses on the real robot.' });
      if (!yes) { window.toast('Cancelled: nothing was sent to the robot.'); return false; }
      extra = { confirmed: true };
    }
    lastMoveError = null;
    const results = await Promise.allSettled(arms.map((arm) => {
      const value = $(`${arm}PoseSelect`)?.value || 'Default Start Pose';
      const point = value.startsWith('gp:') ? value.slice(3) : null;           // a global point of the controller
      const motion = $(`lite_${arm}_poseMotion`)?.value || 'movej';
      const status = $(`${arm}PoseStatus`);
      const go = point
        ? movePoint(arm, point, motion, POINT_SPEED, 0, extra, (j) => { if (status) status.textContent = jobLine(j); }).then((done) => {
          if (done.state !== 'done') throw Object.assign(new Error(done.error || 'The move did not finish.'), { hint: done.hint });
        })
        : move(`/api/robots/${arm}/move-pose`, { name: value, ...extra }, `Moving the ${arm.toUpperCase()} arm to its start pose`, status);
      return go.then(() => {
        if (status) {
          status.textContent = `${arm.toUpperCase()} at “${point || value}”` + (point ? ` (${motion === 'movel' ? 'MoveL' : 'MoveJ'})` : '')
            + (mode === 'real' ? ' (joints read back).' : ' (dry run: nothing moved).');
        }
      });
    }));
    results.forEach((r, i) => {
      if (r.status !== 'rejected') return;
      const e = $(`${arms[i]}PoseStatus`);
      if (e) e.textContent = `${arms[i].toUpperCase()}: ${say(r.reason)}`;
      lastMoveError = lastMoveError || r.reason;
    });
    if (lastMoveError) {
      const moved = arms.filter((a, i) => results[i].status === 'fulfilled');
      if (moved.length) window.toast(`${say(lastMoveError)} The ${moved.map(a => a.toUpperCase()).join(' and ')} arm did reach its start pose.`);
      else tell(lastMoveError);
      return false;
    }
    return true;
  };

  // Redo Preparation copies the arm's list as it opens: read the controller's points first, then
  // give the copy its own search and motion.
  const pageRedo = window.openRedoPoseModal;
  window.openRedoPoseModal = async function openRedoPoseModal() {
    await fillPoses(selectedArm).catch(() => {});
    pageRedo();
    const dest = $('redoPoseSelect');
    pointControls(dest, 'lite_redo');
    for (const o of dest?.options || []) o.hidden = false;
    const linkedArm = hasController(selectedArm);
    if ($('lite_redoSearch')) { $('lite_redoSearch').value = ''; $('lite_redoSearch').hidden = !linkedArm; }
    if ($('lite_redoMotion')) { $('lite_redoMotion').value = $(`lite_${selectedArm}_poseMotion`)?.value || 'movej'; $('lite_redoMotion').hidden = !linkedArm; }
  };

  window.moveRedoToStart = async function moveRedoToStart() {
    const dest = $('redoPoseSelect'), source = $(`${selectedArm}PoseSelect`);
    if (source && dest) source.value = dest.value;
    if ($('lite_redoMotion') && $(`lite_${selectedArm}_poseMotion`)) $(`lite_${selectedArm}_poseMotion`).value = $('lite_redoMotion').value;
    $('redoPoseStatus').textContent = `Moving the ${selectedArm.toUpperCase()} arm to “${dest?.value || 'its start pose'}”…`;
    const ok = await window.demoMove(selectedArm.toUpperCase());
    redoPoseMoved = ok;
    const real = (await liteMode()) === 'real';
    $('redoPoseStatus').textContent = ok
      ? `${selectedArm.toUpperCase()} arm at its start pose${real ? '' : ' (dry run: nothing moved)'} · ready to record again.`
      : `The arm is not at its start pose: ${lastMoveError ? say(lastMoveError) : 'the move did not finish.'}`;
    $('redoReturnBtn').disabled = !ok;
  };

  // The hand preset and the six channels always agree: a preset fills them, editing one makes it Custom.
  $('handPreset')?.addEventListener('change', (e) => { if (PRESETS[e.target.value]) fillHand(PRESETS[e.target.value]); });
  $('handFields')?.addEventListener('input', () => { if ($('handPreset')) $('handPreset').value = 'Custom'; });

  document.addEventListener('lite:step', refresh);
  document.addEventListener('lite:stage', (e) => { if (e.detail === 'prepare') refresh(); });

  // the start-pose lists follow each arm's controller: read when Setup & Safety opens and when an arm
  // connects or disconnects
  const pageShow = window.showView;
  window.showView = function showView(v, btn) {
    pageShow(v, btn);
    if (v === 'setup') { fillPoses('left'); fillPoses('right'); }
  };
  document.addEventListener('lite:points', (e) => fillPoses(e.detail));
}

