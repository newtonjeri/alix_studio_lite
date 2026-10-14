// Advanced Robot Controls (engineer PIN, server-checked). Connected so far: the controller's run
// mode per arm (SIM / REAL / CHECK), the hand and the pole, and the program list. Uploading any
// file, stored-program runs, Global Points and the arm's own drag teach come with the robot I/O
// backend (docs/UI_CONTRACT.md); until then those buttons keep the page's template behaviour.
// With the live link they command the controllers (the pole and the hand at 50 % speed, the run
// mode read back); without it, the stand-in.

import { get } from './api.js';
import { jobLine, robotCall, tell, watchJob } from './ctx.js';
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

  window.rcRefreshPrograms = async function rcRefreshPrograms(side) {
    try {
      const d = await get(`/api/robots/${side}/programs`);
      const box = document.getElementById(`rc${S(side)}ProgramCount`);
      const where = standIn() ? ' on the stand-in robot' : '';
      if (box) box.textContent = d.programs.length ? `${d.programs.length} programs${where}: ${d.programs.map(p => `${p.id} ${p.name}`).join(' · ')}` : `No programs stored yet${where}.`;
      window.rcAppend(side, `PROGRAM LIST // ${d.programs.length} stored`);
    } catch (err) { tell(err); }
  };
}
