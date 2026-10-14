// Advanced Robot Controls (engineer PIN, server-checked). Connected so far: the controller's run
// mode per arm (SIM / REAL / CHECK), the hand and the pole, the controller's program list (RUN), and
// its global points (GLOBAL POINTS: the list read from the controller, found by name, MoveJ / MoveL
// and a sequence of them, by the control program's runner v14). Uploading any file, stored-program
// runs and the arm's own drag teach keep the page's template behaviour (docs/UI_CONTRACT.md).
// With the live link they command the controllers (the pole and the hand at 50 % speed, the run
// mode read back); without it, the stand-in.

import { get, say } from './api.js';
import { el, jobLine, robotCall, tell, watchJob } from './ctx.js';
import { loadPoints, matches, movePoint } from './points.js';
import { standIn } from './status.js';

const S = (side) => (side === 'left' ? 'LEFT' : 'RIGHT');
const readBack = () => (standIn() ? 'stand-in robot: nothing reached a controller' : 'read back');

async function job(side, url, body, what) {
  const j = await robotCall(url, body, { what });
  const done = await watchJob(j, (v) => window.rcSetStatus(side, jobLine(v), true));
  window.rcSetStatus(side, done.state === 'done' ? `${what}: done ✓` : (done.error || `${what} stopped`), false);
  window.rcAppend(side, `${what} // ${done.state.toUpperCase()}${done.error ? ' // ' + done.error : ''}`);
}

export function installAdvanced() {
  window.rcSetMode = async function rcSetMode(side, mode) {
    try {
      const a = await robotCall(`/api/robots/${side}/controller-mode`, { mode: mode === 'real' ? 'REAL' : 'SIM' },
        { what: `Switching the ${S(side)} controller to REAL` });
      robotControlState[side].mode = a.controller_mode;
      window.rcSetStatus(side, `MODE // ${a.controller_mode} (${readBack()})`, false);
      window.rcAppend(side, `MODE SWITCH // ${a.controller_mode} (${readBack()})`);
    } catch (err) { tell(err); window.rcAppend(side, `MODE SWITCH refused // ${err.message}`); }
  };

  window.rcCheckMode = async function rcCheckMode(side) {
    try {
      const st = await get('/api/robots/status');
      const link = st.arms[side] && st.arms[side].link;
      if (link) {                                     // the live controller, as read (Lite only reads it)
        const mode = (link.feedback && link.feedback.run_mode) || 'not read';
        window.rcAppend(side, `MODE CHECK // live controller: ${mode} (read only) // E-stop ${st.arms[side].estop}`);
        window.toast(`${S(side)} controller: run mode ${mode}, E-stop ${st.arms[side].estop}. Lite only reads it.`);
        return;
      }
      const a = await robotCall(`/api/robots/${side}/controller-mode`, { mode: 'CHECK' });
      robotControlState[side].mode = a.controller_mode;
      window.rcAppend(side, `MODE CHECK // ${a.connected ? a.controller_mode : 'not connected'} // E-stop ${a.estop}`);
      window.toast(`${S(side)}: ${a.connected ? a.controller_mode : 'not connected'}, E-stop ${a.estop_text}.`);
    } catch (err) { tell(err); }
  };

  window.rcApplyHand = async function rcApplyHand(side) {
    try { await job(side, `/api/robots/${side}/hand`, { counts: window.rcHandValues(side) }, 'Hand'); } catch (err) { tell(err); }
  };

  window.rcApplyPole = async function rcApplyPole(side) {
    const mm = Number(document.getElementById(`rc${S(side)}PoleNum`)?.value || 0);
    try { await job(side, `/api/robots/${side}/pole`, { mm }, `Pole to ${mm} mm`); } catch (err) { tell(err); }
  };

  // RUN: the controller's programs as a list (the owner, 14 Oct: the run-on line was unreadable); a
  // row fills "Program ID or name".
  window.rcRefreshPrograms = async function rcRefreshPrograms(side) {
    const count = document.getElementById(`rc${S(side)}ProgramCount`);
    try {
      const d = await get(`/api/robots/${side}/programs`);
      const where = standIn() ? ' on the stand-in robot' : '';
      if (count) count.textContent = d.programs.length ? `${d.programs.length} programs${where}. Choose one to fill Program ID.` : `No programs stored yet${where}.`;
      listBox(count, `rc${S(side)}ProgramList`).replaceChildren(table(['ID', 'Name', 'Size'], d.programs.map(p => ({
        cells: [String(p.id), p.name || '', p.bytes || p.size ? `${Math.round((p.bytes || p.size) / 1000)} kB` : ''],
        pick: () => { const f = document.getElementById(`rc${S(side)}StoredProgram`); if (f) f.value = String(p.id); },
      }))));
      window.rcAppend(side, `PROGRAM LIST // ${d.programs.length} stored`);
    } catch (err) { tell(err); }
  };

  // GLOBAL POINTS: the controller's list, filtered by "Point name / Search" as it is typed; a row
  // fills the name. MoveJ / MoveL at the tab's speed and wait.
  const shown = { left: [], right: [] };
  const renderPoints = (side) => {
    const q = document.getElementById(`rc${S(side)}PointSearch`)?.value || '';
    const rows = shown[side].filter(p => matches(p.name, q));
    listBox(document.getElementById(`rc${S(side)}PointCount`), `rc${S(side)}PointList`).replaceChildren(rows.length
      ? table(['Point', 'Joints (deg)', 'Frames', 'Saved'], rows.map(p => ({
        cells: [p.name, p.joints.map(v => v.toFixed(1)).join(', '), [p.work_frame, p.tool_frame].filter(Boolean).join(' / ') || '—', p.time || ''],
        pick: () => { const f = document.getElementById(`rc${S(side)}PointSearch`); if (f) { f.value = p.name; renderPoints(side); } },
      })))
      : el('div', { class: 'mini-note', text: shown[side].length ? 'No point matches the search.' : '' }));
  };
  window.rcRefreshPoints = async function rcRefreshPoints(side) {
    const count = document.getElementById(`rc${S(side)}PointCount`);
    try {
      const d = await loadPoints(side, true);
      shown[side] = d.points;
      if (count) count.textContent = d.note || `${d.points.length} global points on the ${S(side)} controller. Type to search; choose one to use it.`;
      renderPoints(side);
      window.rcAppend(side, `GLOBAL POINTS // ${d.points.length} read from the controller`);
    } catch (err) { if (count) count.textContent = say(err); tell(err); }
  };
  for (const side of ['left', 'right']) {
    document.getElementById(`rc${S(side)}PointSearch`)?.addEventListener('input', () => renderPoints(side));
  }
  const speedOf = (side) => Math.max(1, Math.min(100, Number(document.getElementById(`rc${S(side)}PointSpeed`)?.value) || 20));
  const waitOf = (side) => Math.max(0, Number(document.getElementById(`rc${S(side)}PointWait`)?.value) || 0);

  window.rcPointMove = async function rcPointMove(side, motion) {
    const name = (document.getElementById(`rc${S(side)}PointSearch`)?.value || '').trim();
    if (!name) { window.toast('Enter or choose a Global Point name first.'); return; }
    const move = String(motion).toLowerCase() === 'movel' ? 'movel' : 'movej';
    try {
      const done = await movePoint(side, name, move, speedOf(side), waitOf(side), {}, (v) => window.rcSetStatus(side, jobLine(v), true));
      const what = `GLOBAL POINT // ${motion} // ${name} // ${speedOf(side)} %`;
      window.rcSetStatus(side, done.state === 'done' ? `${what}: done ✓` : (done.error || `${what} stopped`), false);
      window.rcAppend(side, `${what} // ${done.state.toUpperCase()}${done.error ? ' // ' + done.error : ''}`);
      if (done.state !== 'done') window.toast(`${done.error || 'The move stopped.'} ${done.hint || ''}`);
    } catch (err) { tell(err); }
  };

  // The page's point sequence, step by step; one confirmation for all of it, and it stops at the
  // first step that does not finish.
  window.rcRunPointSequence = async function rcRunPointSequence(side) {
    const steps = robotControlState[side].pointSequence;
    if (!steps.length) { window.toast('Add one or more Global Points first.'); return; }
    let extra = {};
    for (const [i, st] of steps.entries()) {
      const move = String(st.motion).toLowerCase() === 'movel' ? 'movel' : 'movej';
      try {
        const done = await movePoint(side, st.name, move, Math.max(1, Math.min(100, Number(st.speed) || 20)), Number(st.wait) || 0, extra,
          (v) => window.rcSetStatus(side, `Step ${i + 1} of ${steps.length}: ${jobLine(v)}`, true));
        extra = { confirmed: true };
        window.rcAppend(side, `SEQUENCE ${i + 1}/${steps.length} // ${st.motion} // ${st.name} // ${done.state.toUpperCase()}`);
        if (done.state !== 'done') {
          window.rcSetStatus(side, `Sequence stopped at step ${i + 1}: ${done.error || 'the move did not finish'}`, false);
          window.toast(`Sequence stopped at step ${i + 1}: ${done.error || 'the move did not finish.'}`);
          return;
        }
      } catch (err) { tell(err); return; }
    }
    window.rcSetStatus(side, `Global point sequence done ✓ (${steps.length} steps)`, false);
  };
}

// A list under a count line, made once.
function listBox(after, id) {
  let box = document.getElementById(id);
  if (!box && after) { box = el('div', { id, class: 'lite-rc-list' }); after.after(box); }
  return box || el('div');
}

function table(heads, rows) {
  return el('table', { class: 'lite-table lite-rc-table' },
    el('thead', {}, el('tr', {}, heads.map(h => el('th', { text: h })))),
    el('tbody', {}, rows.map(r => el('tr', { tabindex: '0', title: 'Use this one', onclick: r.pick,
      onkeydown: (e) => { if (e.key === 'Enter') r.pick(); } }, r.cells.map(c => el('td', { text: c }))))));
}
