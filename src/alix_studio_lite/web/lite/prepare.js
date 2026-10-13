// Prepare and Setup & Safety (R7, R4). Each arm step starts from its region's own pole height and
// hand values in the catalogue (D35) instead of one value for every region. Apply Pole, Apply Hand
// and the start-pose moves go to the robot in Hardware sim and Real (with a confirmation in Real);
// Redo's "Ready → Record Again" opens only once the arm has really reached its start pose.

import { ctx, jobLine, robotCall, step, tell, watchJob } from './ctx.js';

const $ = (id) => document.getElementById(id);
const GRIP = [0, 0, 0, 0, 150, 1000], OPEN = [1000, 1000, 1000, 1000, 1000, 1000];
let lastKey = null;

function defaults() {
  const s = step();
  if (!s || s.key === lastKey) return;
  lastKey = s.key;
  if (s.pole_mm != null && $('poleHeight')) $('poleHeight').value = s.pole_mm;
  const hand = s.hand_counts || (s.hand_preset === 'OPEN' ? OPEN : null);
  if (hand) {
    [...document.querySelectorAll('#handFields input')].forEach((x, i) => { x.value = hand[i] ?? x.value; });
    const preset = $('handPreset');
    if (preset) preset.value = JSON.stringify(hand) === JSON.stringify(GRIP) ? 'Grip / Cleaning' : JSON.stringify(hand) === JSON.stringify(OPEN) ? 'All Open' : 'Custom';
  }
}

async function move(url, body, what, statusEl) {
  const job = await robotCall(url, body, { what });
  const done = await watchJob(job, (j) => { if (statusEl) statusEl.textContent = jobLine(j); });
  if (done.state !== 'done') throw Object.assign(new Error(done.error || `${what} stopped.`), { hint: done.hint });
  return done;
}

export function installPrepare() {
  const pageApply = window.demoApply;
  window.demoApply = async function demoApply(kind) {
    if (!ctx.pid) { pageApply(kind); return; }
    const arm = selectedArm;
    try {
      if (kind === 'Pole') {
        const mm = Number($('poleHeight').value);
        await move(`/api/robots/${arm}/pole`, { mm }, 'Moving the pole');
        window.toast(`${arm.toUpperCase()} pole at ${mm} mm (read back).`);
      } else {
        const counts = Object.values(window.readHandValues());
        await move(`/api/robots/${arm}/hand`, { counts }, 'Moving the hand');
        window.toast(`${arm.toUpperCase()} hand set: ${counts.join(', ')}.`);
      }
      window.demoApplyToRobotControl(kind);              // the Advanced panel shows the same values
    } catch (err) { tell(err); }
  };

  window.demoMove = async function demoMove(which) {
    if (!isSetupReady()) { window.toast('Complete all physical preparation checks first.'); return false; }
    const arms = which === 'BOTH' ? ['left', 'right'] : [String(which).toLowerCase()];
    try {
      await Promise.all(arms.map((arm) => {
        const name = $(`${arm}PoseSelect`)?.value || 'Default Start Pose';
        return move(`/api/robots/${arm}/move-pose`, { name }, `Moving the ${arm.toUpperCase()} arm to its start pose`, $(`${arm}PoseStatus`))
          .then(() => { const e = $(`${arm}PoseStatus`); if (e) e.textContent = `${arm.toUpperCase()} at “${name}” (joints read back).`; });
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

  document.addEventListener('lite:step', defaults);
}
