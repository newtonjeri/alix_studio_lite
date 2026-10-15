// Prepare and Setup & Safety (R7, R4). Each arm step has a SETUP on the server: the pole height and
// hand pose it is taught with (from its existing recording's record or the region catalogue).
// Prepare shows it; Apply Pole / Apply Hand keep the page's values as the setup and stage them:
// in Real on the robot (pole, then hand, each read back, after a confirmation); in Hardware sim
// emulated, as nothing moves there. The hand sent is the one on screen: the preset, or with Custom
// the six channels, where an empty channel is held (left where it is).
// Drag teaching starts only with the arm in that setup, so every recording knows its true stage,
// and every execution stages that stage first (staging.py). The start-pose moves wait for the arm;
// with the live link the start poses are the controller's global points (MoveJ or MoveL).

import { get, post, say } from './api.js';
import { REAL_CHECKS, confirmDialog, ctx, el, jobLine, loadUnit, robotCall, step, tell, unitUrl, watchJob } from './ctx.js';
import { standoffSaved } from './placement.js';
import { POINT_SPEED, fillPointGroup, filterOptions, hasController, loadPoints, movePoint, pointControls } from './points.js';
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
  regionPoints();
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
  const body = { pole_mm: Number($('poleHeight').value), hand: handFromPage(), apply: mode === 'hwsim' || mode === 'real',
    kind: String(kind || '').toLowerCase() };                  // which Apply: teaching waits for both (the owner, 15 Oct)
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

// ── Configuration Ready → Start Teaching waits for both Applies, with the values on screen, and the
// task's UGV standoff (the owner, 15 Oct: "without click the apply buttons, it not allow to move next")

const sameHand = (a, b) => Array.isArray(a) && a.length === b.length && a.every((v, i) => (v == null ? null : Number(v)) === (b[i] == null ? null : Number(b[i])));

async function missingForTeaching() {
  const s = step();
  if (!s || !ctx.pid || s.later) return [];
  let u;
  try { u = await loadUnit(s.key); } catch (_) { return []; }
  const done = u.applied || {}, missing = [];
  if (!done.pole || Math.abs(Number(done.pole.pole_mm) - Number($('poleHeight')?.value)) > 0.5) missing.push('press Apply beside the pole height');
  if (!done.hand || !sameHand(done.hand.hand, handFromPage())) missing.push('press Apply beside the hand preset');
  if (!(await standoffSaved())) missing.push('enter the UGV standoff and save the UGV placement');
  return missing;
}

// ── the start poses as the controller's global points (the owner, 14 Oct): with the arm linked, its
// start-pose list (Setup & Safety, and Redo Preparation, which copies it) holds the global points read
// from its controller, a search field narrows it, and MoveJ or MoveL takes the arm there at 20 %.
// The stand-in keeps the page's own profiles (it has no controller).

const ownOptions = {};                           // each arm's list as the page made it

async function fillPoses(side) {
  const sel = $(`${side}PoseSelect`);
  if (!sel) return;
  if (!(side in ownOptions)) ownOptions[side] = sel.innerHTML;
  const linkedArm = hasController(side);
  pointControls(sel, `lite_${side}_pose`, linkedArm);
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

// Prepare's Region start pose (feedback v2, slide 7): after the page's own choices, the arm's global
// points from its controller, a search, MoveJ / MoveL, and ▶ Move to Point.
async function regionPoints() {
  const sel = $('regionPose');
  if (!sel) return;
  const side = selectedArm, linkedArm = hasController(side);
  pointControls(sel, 'lite_region', linkedArm);
  let go = $('liteRegionMove');
  if (!go) {
    go = el('button', { id: 'liteRegionMove', class: 'btn green', type: 'button', text: '▶ Move to Point', onclick: moveRegion });
    sel.closest('.inline')?.append(go);
    sel.closest('.field')?.append(el('div', { id: 'liteRegionStatus', class: 'mini-note' }));
    sel.addEventListener('change', () => { go.disabled = !String(sel.value).startsWith('gp:'); });
  }
  try { await fillPointGroup(sel, side); } catch (err) { $('liteRegionStatus').textContent = say(err); }
  filterOptions(sel, $('lite_regionSearch')?.value);
  go.hidden = !linkedArm;
  go.disabled = !String(sel.value).startsWith('gp:');
}

async function moveRegion() {
  const sel = $('regionPose'), status = $('liteRegionStatus');
  if (!String(sel.value).startsWith('gp:')) { window.toast('Choose one of the controller’s global points first.'); return; }
  const side = selectedArm, name = sel.value.slice(3), motion = $('lite_regionMotion')?.value || 'movej';
  try {
    const done = await movePoint(side, name, motion, POINT_SPEED, 0, {}, (j) => { status.textContent = jobLine(j); });
    const real = (await liteMode()) === 'real';
    status.textContent = done.state === 'done'
      ? `${side.toUpperCase()} at “${name}” (${motion === 'movel' ? 'MoveL' : 'MoveJ'})${real ? ', read back' : ': the controller’s SIM mode, nothing moved'}.`
      : `Stopped: ${done.error || 'the move did not finish.'} ${done.hint || ''}`;
  } catch (err) { status.textContent = say(err); tell(err); }
}

// Setup & Safety (feedback v2, slide 13): one button reads both arms' global points again.
async function refreshAllPoints() {
  const sides = ['left', 'right'].filter(hasController);
  if (!sides.length) { window.toast('Connect the arms with the live link (--robot realman) to read their global points.'); return; }
  const counts = await Promise.all(sides.map(async (side) => {
    try { const d = await loadPoints(side, true); return `${side.toUpperCase()} ${d.points.length}`; }
    catch (err) { return `${side.toUpperCase()}: ${say(err)}`; }
  }));
  for (const side of sides) document.dispatchEvent(new CustomEvent('lite:points', { detail: side }));
  window.toast(`Global points read again from the controllers: ${counts.join(' · ')}.`);
}

function refreshButton() {
  const head = [...document.querySelectorAll('#view-setup h2')].find((h) => /Robot start poses/i.test(h.textContent))?.closest('.section-head');
  if (!head || $('liteRefreshPoints')) return;
  head.append(el('button', { id: 'liteRefreshPoints', class: 'btn light', type: 'button', text: '↻ Refresh global points (both arms)',
    onclick: refreshAllPoints }));
}

// The Different Start Pose window (feedback v2, slide 14): the arm's pose now, read from its controller
// into J1-J7, and saved on the controller as a global point under the profile's name.
function poseTools() {
  const grid = $('poseGrid');
  if (!grid || $('litePoseTools')) return;
  grid.after(el('div', { id: 'litePoseTools', class: 'lite-pose-tools' },
    el('div', { class: 'cta-row' },
      el('button', { class: 'btn light', type: 'button', text: '📍 Fetch current pose', onclick: fetchPose }),
      el('button', { class: 'btn', type: 'button', text: '💾 Save to controller as global point', onclick: savePoint })),
    el('div', { id: 'litePoseStatus', class: 'mini-note' })));
}

const poseInputs = () => [...document.querySelectorAll('#poseGrid input')];

async function fetchPose() {
  const st = $('litePoseStatus');
  try {
    const c = await get(`/api/robots/${poseSide}/capture`);
    poseInputs().forEach((x, i) => { x.value = c.joints_deg[i].toFixed(3); });
    st.textContent = `${poseSide.toUpperCase()} arm now: J1–J7 read from its controller (${c.controller.ip}) at ${c.at.slice(11, 19)}.`;
  } catch (err) { st.textContent = say(err); tell(err); }
}

async function savePoint() {
  const st = $('litePoseStatus'), name = ($('poseName')?.value || '').trim();
  if (!/^[A-Za-z0-9_-]{1,32}$/.test(name)) {
    st.textContent = 'Name the global point with 1 to 32 letters, digits, _ or - (for example STP_SEAT_1).';
    $('poseName')?.focus();
    return;
  }
  const shown = poseInputs().map((x) => (x.value === '' ? null : Number(x.value)));
  if (shown.some((v) => v == null)) { st.textContent = 'Fetch the current pose first: the controller saves the arm where it is.'; return; }
  try {
    const out = await robotCall(`/api/robots/${poseSide}/points/save`, { name, joints_deg: shown },
      { what: `Saving “${name}” on the ${poseSide.toUpperCase()} controller`, replacing: 'the global point' });
    await loadPoints(poseSide, true);
    document.dispatchEvent(new CustomEvent('lite:points', { detail: poseSide }));
    st.textContent = `Saved on the ${poseSide.toUpperCase()} controller as “${out.name}”${out.replaced ? ' (replaced)' : ''}: `
      + `the arm's joints and its pose in the ${out.work_frame || 'current'} work frame and ${out.tool_frame || 'current'} tool frame, read back.`;
    window.toast(`Global point “${out.name}” saved on the ${poseSide.toUpperCase()} controller.`);
  } catch (err) { st.textContent = say(err); tell(err); }
}

async function move(url, body, what, statusEl) {
  const job = await robotCall(url, body, { what });
  const done = await watchJob(job, (j) => { if (statusEl) statusEl.textContent = jobLine(j); });
  if (done.state !== 'done') throw Object.assign(new Error(done.error || `${what} stopped.`), { hint: done.hint });
  return done;
}

export function installPrepare() {
  const pageStage = window.setStage;
  window.setStage = async function setStage(stage, ...rest) {
    if (stage === 'teach' && currentStage === 'prepare' && ctx.pid) {
      const missing = await missingForTeaching();
      if (missing.length) {
        window.toast(`Before teaching ${step()?.display || 'this region'}: ${missing.join('; ')}.`);
        return undefined;
      }
    }
    return pageStage.call(this, stage, ...rest);
  };

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
    const linkedArm = hasController(selectedArm);
    pointControls(dest, 'lite_redo', linkedArm);
    for (const o of dest?.options || []) o.hidden = false;
    if ($('lite_redoSearch')) $('lite_redoSearch').value = '';
    if ($('lite_redoMotion')) $('lite_redoMotion').value = $(`lite_${selectedArm}_poseMotion`)?.value || 'movej';
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
  // connects or disconnects; Refresh reads both again
  const pageShow = window.showView;
  window.showView = function showView(v, btn) {
    pageShow(v, btn);
    if (v === 'setup') { refreshButton(); fillPoses('left'); fillPoses('right'); }
  };
  refreshButton();
  const pageOpenPose = window.openPose;
  window.openPose = function openPose(side) {
    pageOpenPose(side);
    poseTools();
    if ($('litePoseStatus')) $('litePoseStatus').textContent = hasController(poseSide)
      ? 'Fetch the arm’s current pose, then save it on the controller as a global point under the name above.'
      : 'With the arm connected by the live link, its current pose can be fetched and saved on its controller.';
  };
  document.addEventListener('lite:points', (e) => { fillPoses(e.detail); if (e.detail === selectedArm) regionPoints(); });
}

