// Real Test (R4, R13; decision D30). Upload sends the kept tuned version to the Program ID (1–100,
// checked by the server; a slot holding another program needs a confirmation). Move to Start and
// Run go to the robot: in Hardware sim nothing moves (the controller runs the program in its SIM
// mode, the pole and the hand are emulated), in Real the arm, the pole and the hand move after a
// confirmation, and in Visual sim nothing is sent. The run's real outcome is
// shown, and PASS is accepted only after a completed run (tasks.js).

import { get } from './api.js';
import { ctx, el, fmtS, jobLine, loadUnit, robotCall, step, tell, unitUrl, watchJob } from './ctx.js';
import { gateChips } from './tune.js';

const $ = (id) => document.getElementById(id);

const MODE_NOTE = {
  visual: ['notice', 'Visual sim: nothing is sent to the robot. Choose Hardware sim or Real robot in the status pill at the top to test on the controller.'],
  hwsim: ['notice', 'Hardware sim: nothing moves. The controller runs the program in its SIM mode (the arm stays still); the pole and the hand are emulated.'],
  real: ['notice warn', 'REAL ROBOT: the arm, the pole and the hand move. Keep the area clear and the red E-stop buttons within reach. Each move asks for confirmation.'],
};

async function refresh() {
  const s = step();
  if (!s || !ctx.pid || s.later) return;
  let u;
  try { u = await loadUnit(s.key); } catch (err) { tell(err); return; }
  const st = await get('/api/robots/status').catch(() => null);
  const note = $('stage-real').querySelector('.notice');
  if (note && st) { const [cls, text] = MODE_NOTE[st.mode]; note.className = cls; note.textContent = text; }
  const v = u.versions[u.versions.length - 1];
  $('realTaskLabel').textContent = `${TASKS[currentTask].label} · ${s.display}`;
  const pid = u.settings_view.settings.program_id;
  if (pid && !$('programId').value) $('programId').value = pid;
  window.validateProgramId();
  let box = $('liteVersion');
  if (!box) { box = el('div', { id: 'liteVersion', class: 'lite-version mini-note' }); $('stage-real').querySelector('.real-program-row')?.after(box); }
  box.replaceChildren(gateChips(u), el('div', { text: v
    ? `Tuned version ${v.version}: ${fmtS(v.achieved_s)}, ${Number(v.lines).toLocaleString()} rows, every limit passed.`
      + (v.stage ? ` Every run stages ${v.stage.text} first (read back in Real, emulated in Hardware sim); if either fails, nothing runs.` : '')
      + (u.last_run ? ` Last run: ${u.last_run.ok ? 'completed ✓' : 'did not finish'} (${u.last_run.at}).` : ' Not run yet.')
    : 'No tuned version yet: finish Tune + Simulation first.' }));
}

export function installRealTest() {
  window.simulateUpload = async function simulateUpload() {
    if (!window.validateProgramId()) { window.toast('Enter a valid Program ID from 1 to 100 before Upload.'); $('programId')?.focus(); return; }
    const s = step();
    if (!s || !ctx.pid) return;
    const pid = Number($('programId').value);
    const status = $('uploadStatus');
    try {
      const out = await robotCall(unitUrl(s.key, 'upload'), { program_id: pid }, { what: 'Upload' });
      if (out.unchanged) { status.textContent = `Program ${pid} already holds this version ✓`; return; }
      const done = await watchJob(out, (j) => { status.textContent = jobLine(j); });
      status.textContent = done.state === 'done' ? `Uploaded to Program ID ${pid} ✓ (name and size read back)` : (done.error || 'Upload stopped.');
      if (done.state === 'done') {                          // the page's own record of the Program ID
        const rec = window.captureCurrentTaskRecord();
        rec.programId = pid;
        rec[`${selectedArm}ProgramId`] = pid;
        taskRecords[currentTask] = rec;
        window.safeSaveTaskRecords();
      }
    } catch (err) { status.textContent = err.message; tell(err); }
    refresh();
  };

  window.simulateRealRun = async function simulateRealRun() {
    const s = step();
    if (!s || !ctx.pid) return;
    const status = $('realRunStatus');
    try {
      const job = await robotCall(unitUrl(s.key, 'run'), {}, { what: 'Running the test' });
      status.className = 'mini-note';
      const done = await watchJob(job, (j) => { status.textContent = jobLine(j); });
      status.textContent = done.state === 'done'
        ? `Test complete ✓ (${done.result && done.result.dry_run ? 'Hardware sim' : 'real robot'}) — choose PASS, Needs Tuning or Record Again.`
        : `Stopped: ${done.error || 'the run did not finish.'} ${done.hint || ''}`;
      status.className = done.state === 'done' ? 'mini-note lite-ok' : 'mini-note lite-bad';
    } catch (err) { status.textContent = err.message; status.className = 'mini-note lite-bad'; tell(err); }
    refresh();
  };

  const pageSync = window.syncRealTestTask;
  window.syncRealTestTask = function syncRealTestTask() {
    pageSync();
    refresh();
  };

  document.addEventListener('lite:stage', (e) => { if (e.detail === 'real') refresh(); });
  document.addEventListener('lite:mode', () => { if (currentStage === 'real') refresh(); });
}
