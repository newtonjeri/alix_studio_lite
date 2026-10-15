// Drag & Teach and Save & Review (R1, R3; decision D31).
//
// Drag & Teach: when the region already has a recording (the fixture's existing taught set, or an
// earlier one), using it is offered first (D31); recording a new one stays available. START and
// STOP go to the robot (the stand-in until robot I/O exists), and a stopped recording is saved in
// the project with the page's file name. Save & Review: every recording of the region is listed and
// one is active; a file loaded from this computer is added and never replaces the new recording.
// Every file name in a project is used once (the owner, 14 Oct): Save TXT refuses one already used,
// and a take or a loaded file gets a numbered name. Save & Review has no playback: the
// recorded motion is seen in Tune's 3D simulation and in the Trajectory Viewer.

import { post, say } from './api.js';
import { armFromName, ctx, el, fmtS, loadUnit, robotCall, stageDialog, step, tell, unitUrl, watchJob } from './ctx.js';
import { liteMode } from './status.js';
import { SOURCE } from './tasks.js';

const $ = (id) => document.getElementById(id);
let teaching = null;           // the drag teaching running: {pid, key, arm}, where its take is saved

async function current() {
  const s = step();
  if (!s || !ctx.pid || s.later) return null;
  return loadUnit(s.key);
}

function activeRec(u) { return u && u.recordings.find(r => r.file === u.active); }

function syncActive(u) {
  const r = activeRec(u);
  if (!r) return;
  activeTrajectorySource = r.source === 'upload' ? 'previous' : 'recorded';
  activeTrajectoryName = r.name;
  activeTrajectoryRows = r.lines;
  window.updateActiveTrajectoryUI();
  const meta = $('activeTrajectoryMeta');
  if (meta) meta.textContent = `${SOURCE[r.source] || r.source} · ${r.lines.toLocaleString()} rows · ${fmtS(r.seconds)}`;
}

// ── Drag & Teach

async function refreshTeach() {
  const host = $('stage-teach');
  let card = $('liteExisting');
  const u = await current().catch(() => null);
  const r = activeRec(u);
  if (!r) { card?.remove(); return; }
  if (!card) {
    card = el('div', { id: 'liteExisting', class: 'lite-existing' });
    host.querySelector('.voice-panel')?.before(card);
  }
  card.replaceChildren(
    el('div', { class: 'lite-existing-copy' },
      el('span', { class: 'badge green', text: r.source === 'library' ? 'EXISTING RECORDING' : 'RECORDING READY' }),
      el('b', { text: r.source === 'library' ? ' This region already has a taught recording.' : ' This region already has a recording.' }),
      el('span', { text: ` ${r.name} · ${fmtS(r.seconds)} · ${SOURCE[r.source] || r.source}` })),
    el('div', { class: 'cta-row' },
      el('button', { class: 'btn green big', text: 'Use this recording → Save & Review', onclick: useExisting }),
      el('span', { class: 'mini-note', text: 'or record a new one with START below.' })));
}

function useExisting() {
  armSelectionLocked = true;                     // the arm stays locked through Real Test, as after a recording
  window.updateArmLockUI();
  window.setStage('review');
  window.setJourneyVisible(false, false);
}

// ── Save & Review

async function refreshReview() {
  const u = await current().catch(() => null);
  if (!u) return;
  syncActive(u);
  const r = activeRec(u);
  $('savedFileLabel').textContent = r ? r.name : 'No trajectory saved yet';
  $('savedFileMeta').textContent = r ? `${SOURCE[r.source] || r.source} · ${r.lines.toLocaleString()} rows · ${fmtS(r.seconds)} · saved in the project` : 'Record a trajectory first.';
  if (r && $('fileName') && r.source !== 'upload') $('fileName').value = r.name;
  let list = $('liteRecordings');
  if (!list) {
    list = el('div', { id: 'liteRecordings', class: 'lite-recordings' });
    $('stage-review').querySelector('.active-trajectory-bar')?.before(list);
  }
  list.replaceChildren(el('div', { class: 'lite-recordings-head' }, el('b', { text: 'Recordings of this region' }),
    el('span', { class: 'mini-note', text: ' The one selected is the one tuned.' })),
  ...u.recordings.slice().reverse().map(rec => el('label', { class: 'checkrow' },
    el('input', { type: 'radio', name: 'liteRecording', checked: rec.file === u.active, onchange: () => choose(u.key, rec.file) }),
    el('span', {}, el('strong', { text: rec.name }),
      el('span', { text: `${SOURCE[rec.source] || rec.source} · ${rec.lines.toLocaleString()} rows · ${fmtS(rec.seconds)} · ${rec.taken || ''}` }),
      el('span', { class: rec.stage && rec.taught_arm === u.arm ? 'lite-ok' : 'lite-bad',
        text: rec.taught_arm && rec.taught_arm !== u.arm ? `✕ taught on the ${rec.taught_arm.toUpperCase()} arm: never runs on the ${u.arm.toUpperCase()} arm`
          : rec.stage ? `✓ ${rec.taught_arm.toUpperCase()} arm · staged first: ${rec.stage.text}` : '✕ pole and hand not known: it cannot run' }),
      !rec.stage && (!rec.taught_arm || rec.taught_arm === u.arm)
        ? el('button', { class: 'btn light', type: 'button', text: 'State pole & hand…', onclick: (e) => { e.preventDefault(); stateStage(u, rec); } })
        : null))));
}

async function stateStage(u, rec) {
  const v = await stageDialog({ title: `How was ${rec.name} taught?`, setup: u.setup, arm: rec.taught_arm || null,
    askArm: !rec.taught_arm,
    body: 'Its joints only mean something at the pole height and hand pose it was taught with: every run stages them first.' });
  if (!v) return;
  try { ctx.units[u.key] = await post(`${unitUrl(u.key, 'recordings')}/${rec.file}/stage`, v); } catch (err) { tell(err); }
  refreshReview();
}

async function choose(key, file) {
  try { const u = await post(unitUrl(key, 'active'), { file }); ctx.units[key] = u; } catch (err) { tell(err); }
  refreshReview();
}

export function installTeach() {
  const pageStart = window.startTeach;
  window.startTeach = async function startTeach() {
    if (!isSetupReady()) { window.toast('Complete Setup & Safety first.'); return; }
    if (timerInt) return;
    const s = step();
    if (s && ctx.pid && !s.later) {
      $('timerResult').textContent = `Starting drag teaching on the ${selectedArm.toUpperCase()} arm…`;
      try {
        await robotCall(unitUrl(s.key, 'teach/start'), {}, { what: 'Drag teaching' });
      } catch (err) { $('timerResult').textContent = say(err); tell(err); return; }
      teaching = { pid: ctx.pid, key: s.key, arm: s.arm, task: TASKS[currentTask]?.label };     // the task: its UGV placement is kept with the recording
    }
    pageStart();
  };

  // STOP DRAG saves the take in the region it was started in, whatever is on screen by then. When
  // the robot refuses (teaching had already ended), nothing was recorded: the page's "Redo or
  // Continue" closes, as there is no take to accept.
  const pageStop = window.stopTeach;
  window.stopTeach = async function stopTeach() {
    pageStop();
    const t = teaching;
    if (!t) return;
    teaching = null;
    const here = () => ctx.pid === t.pid && step()?.key === t.key;
    const asked = ($('fileName')?.value || '').trim();
    try {
      const r = await post(`/api/projects/${t.pid}/units/${t.key}/teach/stop`, { name: asked, task: t.task });
      if (ctx.pid === t.pid) ctx.units[t.key] = r.unit;
      if (r.name && asked && r.name !== asked) window.toast(`Saved as ${r.name}: ${asked} is already used in this project.`);
      if (here()) {
        if ($('fileName') && r.name) $('fileName').value = r.name;
        syncActive(r.unit);
        $('savedFileMeta').textContent = `Recorded and saved: ${(activeRec(r.unit) || {}).lines?.toLocaleString() || ''} rows · ${r.stage.text}`;
      }
      if (r.take) window.toast(`Robot stand-in: the take is ${r.take}, a ${t.arm.toUpperCase()}-arm recording (${r.stage.text}).`);
    } catch (err) {
      $('postDragModal')?.classList.remove('show');
      if ($('timerResult')) $('timerResult').textContent = `Nothing was recorded: ${say(err)}`;
      tell(err);
      if (here()) {
        const u = await loadUnit(t.key).catch(() => null);
        if (activeRec(u)) syncActive(u);
        $('savedFileMeta').textContent = activeRec(u) ? 'Nothing new was recorded: the earlier recording stays active.' : 'Nothing was recorded.';
      }
    }
  };

  const pageReset = window.resetTimer;
  window.resetTimer = function resetTimer() {
    const t = teaching;
    if (t) {
      teaching = null;
      post(`/api/projects/${t.pid}/units/${t.key}/teach/cancel`, {}).catch(() => {});
    }
    pageReset();
  };

  window.demoSave = async function demoSave() {             // Save TXT: the operator's name for the active file
    const s = step();
    if (!s || !ctx.pid) return;
    const name = ($('fileName')?.value || '').trim() || `${s.key}.txt`;
    try {
      ctx.units[s.key] = await post(unitUrl(s.key, 'name'), { name });     // refused if the project has that name
      tuningApplied = false; tunedOutputFileName = ''; tuningSourceFileName = name;   // V32: a new source to tune
      window.toast(`Saved in the project as ${name}.`);
    } catch (err) { tell(err); }
    refreshReview();
  };

  window.handlePreviousTrajectory = async function handlePreviousTrajectory(event) {
    const file = event.target.files?.[0];
    const s = step();
    const st = $('previousTrajectoryStatus');
    if (!file || !s || !ctx.pid) return;
    const named = armFromName(file.name);
    if (named && named !== s.arm) {
      st.textContent = `${file.name} is named for the ${named.toUpperCase()} arm: it can never be used on this ${s.arm.toUpperCase()}-arm region.`;
      st.className = 'statusbar warn';
      event.target.value = '';
      return;
    }
    const v = await stageDialog({ title: `How was ${file.name} taught?`, setup: (ctx.units[s.key] || s).setup,
      arm: named || null, askArm: !named,
      body: `It becomes the active recording of ${s.display} (${s.arm.toUpperCase()} arm). Say the arm and the pole and hand it was taught with: they are staged before every run.` });
    if (!v) { event.target.value = ''; return; }
    st.textContent = `Checking ${file.name}…`;
    try {
      const r = await post(unitUrl(s.key, 'recordings'), { source: 'upload', name: file.name, content: await file.text(), ...v,
        task: TASKS[currentTask]?.label });
      ctx.units[s.key] = r.unit;
      tuningApplied = false; tunedOutputFileName = ''; tuningSourceFileName = file.name;   // V32: a new source to tune
      const rec = activeRec(r.unit);
      st.textContent = `Loaded ${rec.name} · ${rec.lines.toLocaleString()} rows · ${fmtS(rec.seconds)}. The new recording is kept too.`
        + (rec.name !== file.name ? ` (${file.name} is already used in this project, so this one is ${rec.name}.)` : '');
      st.className = 'statusbar ok';
      window.resetPlotSelection();
      refreshReview();
    } catch (err) {
      st.textContent = say(err);
      st.className = 'statusbar warn';
    } finally { event.target.value = ''; }
  };

  const pageAction = window.demoAction;
  window.demoAction = async function demoAction(x) {
    const s = step();
    if (!/start pose/i.test(String(x)) || !s || !ctx.pid) { pageAction(x); return; }
    const which = currentStage === 'real' ? 'version' : 'raw';
    try {
      const job = await robotCall(unitUrl(s.key, 'move-start'), { which }, { what: 'Moving the arm to its start' });
      const done = await watchJob(job);
      const real = (await liteMode()) === 'real';
      window.toast(done.state === 'done' ? `${s.arm.toUpperCase()} arm at the start${real ? '' : ' (dry run: nothing moved)'}.`
        : `Stopped: ${done.error || 'the move did not finish.'} ${done.hint || ''}`);
    } catch (err) { tell(err); }
  };

  document.addEventListener('lite:stage', (e) => {
    if (e.detail === 'teach') refreshTeach();
    if (e.detail === 'review') refreshReview();
  });
  document.addEventListener('lite:step', () => {
    if (currentStage === 'teach') refreshTeach();
    if (currentStage === 'review') refreshReview();
  });
}
