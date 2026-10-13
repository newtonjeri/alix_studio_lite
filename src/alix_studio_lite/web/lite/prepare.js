// Prepare and Setup & Safety (R7, R4). Each arm step has a SETUP on the server: the pole height and
// hand pose it is taught with (from its existing recording's record or the region catalogue).
// Prepare shows it; Apply Pole / Apply Hand keep the page's values as the setup and stage them:
// in Real on the robot (pole, then hand, each read back, after a confirmation); in Hardware sim
// emulated, as nothing moves there.
// Drag teaching starts only with the arm in that setup, so every recording knows its true stage,
// and every execution stages that stage first (staging.py). The start-pose moves wait for the arm.

import { ctx, jobLine, loadUnit, robotCall, step, tell, unitUrl, watchJob } from './ctx.js';

const $ = (id) => document.getElementById(id);
const GRIP = [0, 0, 0, 0, 150, 1000], OPEN = [1000, 1000, 1000, 1000, 1000, 1000];
let lastKey = null;

function handFromPage() {
  return [...document.querySelectorAll('#handFields input')].map(x => (x.value === '' ? null : Number(x.value)));
}

function showSetup(setup) {
  if (!setup) return;
  if ($('poleHeight')) $('poleHeight').value = setup.pole_mm;
  const hand = setup.hand.map((c, i) => (c == null ? (i < 4 ? 1000 : [150, 1000][i - 4]) : c));
  [...document.querySelectorAll('#handFields input')].forEach((x, i) => { x.value = hand[i]; });
  const preset = $('handPreset');
  if (preset) preset.value = JSON.stringify(hand) === JSON.stringify(GRIP) ? 'Grip / Cleaning' : JSON.stringify(hand) === JSON.stringify(OPEN) ? 'All Open' : 'Custom';
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

async function applySetup(kind) {
  const s = step();
  if (!s || !ctx.pid) return;
  const mode = document.querySelector('.topbar .mode-pill')?.dataset.lite;
  const body = { pole_mm: Number($('poleHeight').value), hand: handFromPage(), apply: mode === 'hwsim' || mode === 'real' };
  try {
    const out = await robotCall(unitUrl(s.key, 'setup'), body, { what: `Moving the ${s.arm.toUpperCase()} pole and hand` });
    showSetup(out.unit.setup);
    if (!out.job) { window.toast(`Setup kept for ${s.display}: ${out.unit.setup.text}. In Visual sim nothing moves.`); return; }
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
    window.demoApplyToRobotControl(kind);              // the Advanced panel shows the same values
  } catch (err) { tell(err); }
}

async function move(url, body, what, statusEl) {
  const job = await robotCall(url, body, { what });
  const done = await watchJob(job, (j) => { if (statusEl) statusEl.textContent = jobLine(j); });
  if (done.state !== 'done') throw Object.assign(new Error(done.error || `${what} stopped.`), { hint: done.hint });
  return done;
}

export function installPrepare() {
  const pageApply = window.demoApply;
  window.demoApply = function demoApply(kind) {
    if (!ctx.pid) { pageApply(kind); return; }
    applySetup(kind);
  };

  window.demoMove = async function demoMove(which) {
    if (!isSetupReady()) { window.toast('Complete all physical preparation checks first.'); return false; }
    const arms = which === 'BOTH' ? ['left', 'right'] : [String(which).toLowerCase()];
    try {
      await Promise.all(arms.map((arm) => {
        const name = $(`${arm}PoseSelect`)?.value || 'Default Start Pose';
        return move(`/api/robots/${arm}/move-pose`, { name }, `Moving the ${arm.toUpperCase()} arm to its start pose`, $(`${arm}PoseStatus`))
          .then(() => {
            const e = $(`${arm}PoseStatus`);
            const real = document.querySelector('.topbar .mode-pill')?.dataset.lite === 'real';
            if (e) e.textContent = `${arm.toUpperCase()} at “${name}” ` + (real ? '(joints read back).' : '(dry run: nothing moved).');
          });
      }));
      return true;
    } catch (err) { tell(err); return false; }
  };

  window.moveRedoToStart = async function moveRedoToStart() {
    const dest = $('redoPoseSelect'), source = $(`${selectedArm}PoseSelect`);
    if (source && dest) source.value = dest.value;
    $('redoPoseStatus').textContent = `Moving the ${selectedArm.toUpperCase()} arm to “${dest?.value || 'its start pose'}”…`;
    const ok = await window.demoMove(selectedArm.toUpperCase());
    redoPoseMoved = ok;
    $('redoPoseStatus').textContent = ok ? `${selectedArm.toUpperCase()} arm at its start pose · ready to record again.`
      : 'The arm did not move. Choose Hardware sim or Real robot and connect the arm, then try again.';
    $('redoReturnBtn').disabled = !ok;
  };

  document.addEventListener('lite:step', refresh);
  document.addEventListener('lite:stage', (e) => { if (e.detail === 'prepare') refresh(); });
}

