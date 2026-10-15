// Real Test (R4, R13; decision D30). Upload sends the kept tuned version to the Program ID (1–100,
// checked by the server; a slot holding another program needs a confirmation). Move to Start and
// Run go to the robot: in Hardware sim nothing moves (the controller runs the program in its SIM
// mode, the pole and the hand are emulated), in Real the arm, the pole and the hand move after a
// confirmation, and in Visual sim nothing is sent. The run's real outcome is
// shown, and PASS is accepted only after a completed run (tasks.js). Until Lite's robot commands
// exist the robot is the stand-in, and the screens say that nothing reached a controller.

import { get, say } from './api.js';
import { ctx, el, fmtS, jobLine, loadUnit, robotCall, step, tell, unitUrl, watchJob } from './ctx.js';
import { fillPointGroup, filterOptions, hasController, movePoint, pointControls } from './points.js';
import { liteMode, standIn } from './status.js';
import { gateChips } from './tune.js';

const $ = (id) => document.getElementById(id);

const MODE_NOTE = {
  visual: ['notice', 'Visual sim: nothing is sent to the robot. Choose Hardware sim or Real robot in the status pill at the top to test on the controller.'],
  hwsim: ['notice', 'Hardware sim: nothing moves. The controller runs the program in its SIM mode (the arm stays still); the pole and the hand are emulated.'],
  real: ['notice warn', 'REAL ROBOT: the arm, the pole and the hand move. Keep the area clear and the red E-stop buttons within reach. Each move asks for confirmation.'],
};

// Move to start (feedback v2, slide 7): the tuned trajectory's first line, as before, or one of the
// arm's global points from its controller, by MoveJ or MoveL.
const moveCard = () => $('stage-real')?.querySelector('button[onclick*="trajectory start pose"]')?.closest('.card');

async function startPoints() {
  const card = moveCard();
  if (!card) return;
  let sel = $('liteRealStart');
  if (!sel) {
    sel = el('select', { id: 'liteRealStart', 'aria-label': 'Where Move to Start goes' },
      el('option', { value: 'trajectory', text: 'The tuned trajectory’s first line' }));
    card.querySelector('button')?.before(el('div', { class: 'inline lite-real-start' }, sel));
  }
  const side = selectedArm, linkedArm = hasController(side);
  pointControls(sel, 'lite_realstart', linkedArm);
  await fillPointGroup(sel, side).catch(() => {});
  filterOptions(sel, $('lite_realstartSearch')?.value);
}

async function refresh() {
  startPoints();
  const s = step();
  if (!s || !ctx.pid || s.later) return;
  let u;
  try { u = await loadUnit(s.key); } catch (err) { tell(err); return; }
  const st = await get('/api/robots/status').catch(() => null);
  const note = $('stage-real').querySelector('.notice');
  if (note && st) {
    const [cls, text] = MODE_NOTE[st.mode];
    const linked = Object.values(st.arms || {}).some((a) => a.link);
    note.className = cls;
    note.textContent = st.mode === 'visual' || st.backend !== 'stand-in' || linked ? text
      : `${text} The robot is the stand-in: nothing is sent to a controller.`;
  }
  const v = u.versions.find((x) => x.version === u.version) || u.versions[u.versions.length - 1];     // the version it runs
  $('realTaskLabel').textContent = `${TASKS[currentTask].label} · ${s.display}`;
  const pid = u.settings_view.settings.program_id;
  if (pid && !$('programId').value) $('programId').value = pid;
  window.validateProgramId();
  let box = $('liteVersion');
  if (!box) { box = el('div', { id: 'liteVersion', class: 'lite-version mini-note' }); $('stage-real').querySelector('.real-program-row')?.after(box); }
  const file = v ? (v.name || `${s.key}_${v.version}.txt`) : null;    // the file this section sends (the owner, 15 Oct)
  const limits = !v ? '' : v.limits_ok === false
    ? ` It breaks ${v.limits_text || 'a limit'}: shown, not enforced (the hardware decides); the upload asks once.`
    : ' Every limit passes.';
  box.replaceChildren(
    v ? el('div', { class: 'lite-real-file' }, el('span', { text: 'File' }), el('b', { text: file }), el('small', { text: `version ${v.version}` })) : '',
    gateChips(u), el('div', { text: v
      ? `Tuned version ${v.version} (${file}): ${fmtS(v.achieved_s)}, ${Number(v.lines).toLocaleString()} rows.${limits}`
        + (v.stage ? ` Every run stages ${v.stage.text} first (read back in Real, emulated in Hardware sim); if either fails, nothing runs.` : '')
        + (u.last_run ? ` Last run: ${u.last_run.ok ? 'completed ✓' : 'did not finish'} (${u.last_run.at}).` : ' Not run yet.')
      : 'No tuned version yet: finish Tune + Simulation first.' }));
  const btn = $('uploadTrajectoryBtn');
  let up = $('liteUploadFile');
  if (!up && btn) { up = el('div', { id: 'liteUploadFile', class: 'lite-upload-file mini-note' }); btn.before(up); }
  if (up) up.textContent = file ? `Sends ${file} (${v.version})` : 'No tuned file yet';
}

// The upload as a horizontal bar with its percentage (the owner, 14 Oct), under the page's status
// line. On the robot the SDK sends in one call, so the server's percentage is estimated until the
// stored size is read back; the bar is red when the upload did not finish.
function uploadBar(status) {
  let box = $('liteUploadBar');
  if (!box && status) {
    box = el('div', { id: 'liteUploadBar', class: 'lite-progress', role: 'progressbar', 'aria-valuemin': '0', 'aria-valuemax': '100' },
      el('div', { class: 'lite-progress-fill' }), el('span', { class: 'lite-progress-text' }));
    status.after(box);
  }
  return {
    show(pct, failed = false) {
      if (!box) return;
      box.hidden = false;
      box.classList.toggle('failed', !!failed);
      if (pct == null) return;
      const v = Math.max(0, Math.min(100, Math.round(pct)));
      box.querySelector('.lite-progress-fill').style.width = `${v}%`;
      box.querySelector('.lite-progress-text').textContent = `${v} %`;
      box.setAttribute('aria-valuenow', String(v));
    },
  };
}

export function installRealTest() {
  window.simulateUpload = async function simulateUpload() {
    if (!window.validateProgramId()) { window.toast('Enter a valid Program ID from 1 to 100 before Upload.'); $('programId')?.focus(); return; }
    const s = step();
    if (!s || !ctx.pid) return;
    const pid = Number($('programId').value);
    const status = $('uploadStatus');
    const bar = uploadBar(status);
    try {
      const out = await robotCall(unitUrl(s.key, 'upload'), { program_id: pid }, { what: 'Upload' });
      if (out.unchanged) { status.textContent = `Program ${pid} already holds this version ✓`; bar.show(100); return; }
      bar.show(0);
      const done = await watchJob(out, (j) => { status.textContent = jobLine(j); bar.show(j.n ? (100 * j.i) / j.n : 0); });
      bar.show(done.state === 'done' ? 100 : null, done.state !== 'done');
      status.textContent = done.state !== 'done' ? (done.error || 'Upload stopped.')
        : standIn() ? `Uploaded to Program ID ${pid} on the stand-in robot ✓ (nothing reached a controller)`
          : `Uploaded to Program ID ${pid} ✓ (name and size read back)`;
      if (done.state === 'done') {                          // the page's own record of the Program ID
        const rec = window.captureCurrentTaskRecord();
        rec.programId = pid;
        rec[`${selectedArm}ProgramId`] = pid;
        taskRecords[currentTask] = rec;
        window.safeSaveTaskRecords();
      }
    } catch (err) { status.textContent = err.message; bar.show(null, true); tell(err); }
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
      const how = `${done.result && done.result.dry_run ? 'Hardware sim' : 'real robot'}${standIn() ? ', stand-in robot: nothing reached a controller' : ''}`;
      status.textContent = done.state === 'done'
        ? `Test complete ✓ (${how}) — choose PASS, Needs Tuning or Record Again.`
        : `Stopped: ${done.error || 'the run did not finish.'} ${done.hint || ''}`;
      status.className = done.state === 'done' ? 'mini-note lite-ok' : 'mini-note lite-bad';
    } catch (err) { status.textContent = err.message; status.className = 'mini-note lite-bad'; tell(err); }
    refresh();
  };

  const pageAction = window.demoAction;
  window.demoAction = async function demoAction(x) {
    const v = $('liteRealStart')?.value || '';
    if (currentStage !== 'real' || !/start pose/i.test(String(x)) || !v.startsWith('gp:')) return pageAction(x);
    const side = selectedArm, name = v.slice(3), motion = $('lite_realstartMotion')?.value || 'movej';
    const status = moveCard()?.querySelector('.mini-note');
    try {
      const done = await movePoint(side, name, motion, 0, {}, (j) => { if (status) status.textContent = jobLine(j); });
      const real = (await liteMode()) === 'real';
      const said = done.state === 'done'
        ? `${side.toUpperCase()} at “${name}” (${motion === 'movel' ? 'MoveL' : 'MoveJ'})${real ? ', read back' : ': the controller’s SIM mode, nothing moved'}.`
        : `Stopped: ${done.error || 'the move did not finish.'} ${done.hint || ''}`;
      if (status) status.textContent = said;
      window.toast(said);
    } catch (err) { if (status) status.textContent = say(err); tell(err); }
    return undefined;
  };
  document.addEventListener('lite:points', (e) => { if (currentStage === 'real' && e.detail === selectedArm) startPoints(); });

  const pageSync = window.syncRealTestTask;
  window.syncRealTestTask = function syncRealTestTask() {
    pageSync();
    refresh();
  };

  document.addEventListener('lite:stage', (e) => { if (e.detail === 'real') refresh(); });
  document.addEventListener('lite:mode', () => { if (currentStage === 'real') refresh(); });
}
