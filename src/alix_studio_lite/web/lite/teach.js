// Drag & Teach and Save & Review (R1, R3; decision D31).
//
// Drag & Teach: when the region already has a recording (the fixture's existing taught set, or an
// earlier one), using it is offered first (D31); recording a new one stays available. START and
// STOP go to the robot (the stand-in until robot I/O exists), and a stopped recording is saved in
// the project with the page's file name. Save & Review: every recording of the region is listed and
// one is active; a file loaded from this computer is added and never replaces the new recording.
// Playback runs in the 3D preview in Visual sim, or on the robot in Hardware sim and Real.

import { post, say } from './api.js';
import { ctx, el, fmtS, jobLine, loadUnit, robotCall, step, tell, unitUrl, watchJob } from './ctx.js';
import { showFile, showLine, sim } from './sim.js';
import { SOURCE } from './tasks.js';

const $ = (id) => document.getElementById(id);
let teaching = false;

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
    el('span', { class: 'mini-note', text: ' The one selected is used for tuning and playback.' })),
  ...u.recordings.slice().reverse().map(rec => el('label', { class: 'checkrow' },
    el('input', { type: 'radio', name: 'liteRecording', checked: rec.file === u.active, onchange: () => choose(u.key, rec.file) }),
    el('span', {}, el('strong', { text: rec.name }),
      el('span', { text: `${SOURCE[rec.source] || rec.source} · ${rec.lines.toLocaleString()} rows · ${fmtS(rec.seconds)} · ${rec.taken || ''}` })))));
  const preview = $('stage-review').querySelector('.playback-preview');
  if (preview && sim.key !== u.key) preview.classList.remove('lite-3d');
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
      teaching = true;
    }
    pageStart();
  };

  const pageStop = window.stopTeach;
  window.stopTeach = async function stopTeach() {
    pageStop();
    const s = step();
    if (!teaching || !s || !ctx.pid) return;
    teaching = false;
    try {
      const r = await post(unitUrl(s.key, 'teach/stop'), { name: $('fileName')?.value });
      ctx.units[s.key] = r.unit;
      syncActive(r.unit);
      $('savedFileMeta').textContent = `Recorded and saved: ${(activeRec(r.unit) || {}).lines?.toLocaleString() || ''} rows`;
    } catch (err) { tell(err); }
  };

  const pageReset = window.resetTimer;
  window.resetTimer = function resetTimer() {
    const s = step();
    if (teaching && s && ctx.pid) {
      teaching = false;
      post(unitUrl(s.key, 'teach/cancel'), {}).catch(() => {});
    }
    pageReset();
  };

  window.demoSave = async function demoSave() {             // Save TXT: the operator's name for the active file
    const s = step();
    if (!s || !ctx.pid) return;
    const name = ($('fileName')?.value || '').trim() || `${s.key}.txt`;
    try { ctx.units[s.key] = await post(unitUrl(s.key, 'name'), { name }); window.toast(`Saved in the project as ${name}.`); } catch (err) { tell(err); }
    refreshReview();
  };

  window.handlePreviousTrajectory = async function handlePreviousTrajectory(event) {
    const file = event.target.files?.[0];
    const s = step();
    const st = $('previousTrajectoryStatus');
    if (!file || !s || !ctx.pid) return;
    st.textContent = `Checking ${file.name}…`;
    try {
      const r = await post(unitUrl(s.key, 'recordings'), { source: 'upload', name: file.name, content: await file.text() });
      ctx.units[s.key] = r.unit;
      const rec = activeRec(r.unit);
      st.textContent = `Loaded ${file.name} · ${rec.lines.toLocaleString()} rows · ${fmtS(rec.seconds)}. The new recording is kept too.`;
      st.className = 'statusbar ok';
      window.resetPlotSelection();
      refreshReview();
    } catch (err) {
      st.textContent = say(err);
      st.className = 'statusbar warn';
    } finally { event.target.value = ''; }
  };

  window.demoPlayback = async function demoPlayback() {
    const s = step();
    if (!s || !ctx.pid) return;
    const u = await current();
    if (!u || !u.active) { window.toast('There is no recording to play back yet.'); return; }
    const mode = document.querySelector('.topbar .mode-pill')?.dataset.lite;
    if (mode === 'hwsim' || mode === 'real') {
      try {
        const job = await robotCall(unitUrl(s.key, 'playback'), {}, { what: 'Playback' });
        window.toast(`Playback on the ${u.arm.toUpperCase()} arm started.`);
        const done = await watchJob(job, (j) => { $('timerResult') && ($('timerResult').textContent = jobLine(j)); });
        window.toast(done.state === 'done' ? 'Playback finished.' : (done.error || 'Playback stopped.'));
      } catch (err) { tell(err); }
      return;
    }
    const preview = $('stage-review').querySelector('.playback-preview');
    try {
      await showFile(preview, u, 'raw');
      playPreview();
    } catch (err) { tell(err); }
  };

  const pageAction = window.demoAction;
  window.demoAction = async function demoAction(x) {
    const s = step();
    if (!/start pose/i.test(String(x)) || !s || !ctx.pid) { pageAction(x); return; }
    const which = currentStage === 'real' ? 'version' : 'raw';
    try {
      const job = await robotCall(unitUrl(s.key, 'move-start'), { which }, { what: 'Moving the arm to its start' });
      const done = await watchJob(job);
      window.toast(done.state === 'done' ? `${s.arm.toUpperCase()} arm at the start.` : (done.error || 'The move stopped.'));
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

// The 3D preview plays the recording once at 1x (its own clock: (N - 1) x 5 ms).
let previewRaf = 0;
function playPreview() {
  cancelAnimationFrame(previewRaf);
  const t0 = performance.now(), total = Math.max(0, sim.lines - 1) * 0.005;
  const tick = () => {
    const t = Math.min(total, (performance.now() - t0) / 1000);
    showLine(Math.round(t / 0.005));
    if (t < total) previewRaf = requestAnimationFrame(tick);
    else window.toast('Playback preview finished.');
  };
  tick();
}
