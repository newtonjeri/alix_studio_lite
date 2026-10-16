// The project summary's Approved Setup, for each arm of a task (the owner, 16 Oct), in this order:
// 1. the pole height, 2. the hand: its preset and six values, 3. the start pose and the speed it was
// moved at, 4. the UGV placement: the standoff and the moves from it, 5. the room: its width and
// length and where the commode stands in it. Taken when the arm is approved (Real Test's PASS): the
// kept version's own pole and hand, the start pose chosen, the speed slider, the task's UGV placement
// and the project's room. A setup approved before this shows the page's own record, with the
// placement and room the project has now.

import { get } from './api.js';
import { ctx, loadUnit, step } from './ctx.js';
import { handPresetName } from './prepare.js';
import { speedPct } from './status.js';

let place = null;                     // the project's placement (room and tasks), {pid, room, tasks}
let asking = false;

const esc = (v) => String(v).replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
const mm = (v) => (v == null || v === '' ? '—' : `${Number(v).toLocaleString()}`);
const poseName = (v) => String(v || '').replace(/^gp:/, '') || '—';
const PAGE_HAND = ['Little', 'Ring', 'Middle', 'Index', 'Thumb Bend', 'Thumb Rotate'];

async function loadPlace() {
  if (!ctx.pid || asking) return;
  asking = true;
  try { place = { pid: ctx.pid, ...(await get(`/api/projects/${ctx.pid}/placement`)) }; } catch (_) { place = null; } finally { asking = false; }
}

// What Lite knows of the arm step being approved.
async function facts(side) {
  const s = step(currentTask, side);
  const out = { start_pose: poseName(document.getElementById(`${side}PoseSelect`)?.value), speed_pct: speedPct() };
  if (s && ctx.pid && !s.later) {
    const u = ctx.units[s.key] || await loadUnit(s.key);
    const v = (u.versions || []).find(x => x.version === u.version) || (u.versions || [])[(u.versions || []).length - 1];
    const st = (v && v.stage) || u.setup;
    const preset = st && handPresetName(st.hand);                // else the stage's own words: "fingers 50 % open, thumb held"
    if (st) Object.assign(out, { pole_mm: st.pole_mm, hand: st.hand,
      hand_preset: preset !== 'Custom' ? preset : String(st.text || '').split(', ').slice(1).join(', ') || 'Custom' });
  }
  await loadPlace();
  const task = place && place.tasks && place.tasks[TASKS[currentTask]?.label];
  if (task) out.placement = { standoff_mm: task.standoff_mm, front_mm: task.front_mm, rear_mm: task.rear_mm, left_mm: task.left_mm, right_mm: task.right_mm };
  if (place && place.room) out.room = { ...place.room };
  return out;
}

function lines(rec, a) {
  const L = a.lite || {};
  const hand = L.hand || PAGE_HAND.map(k => (a.handChannels || {})[k]);
  const preset = L.hand_preset || a.handPreset || 'Custom';
  const now = place && place.pid === ctx.pid ? place : null;
  const p = L.placement || (now && now.tasks && now.tasks[rec.taskLabel]) || null;
  const room = L.room || (now && now.room) || null;
  const moves = p ? [['front', p.front_mm], ['rear', p.rear_mm], ['left', p.left_mm], ['right', p.right_mm]]
    .filter(([, v]) => v).map(([k, v]) => ` · ${k} ${mm(v)}`).join('') : '';
  return [
    `Pole: ${mm(L.pole_mm ?? a.poleHeightMm)} mm`,
    `Hand: ${preset} (${hand.map(v => (v == null ? 'held' : v)).join(', ')})`,
    `Start pose: ${poseName(L.start_pose || a.startPose)}${L.speed_pct ? ` at ${L.speed_pct} %` : ''}`,
    p ? `UGV: standoff ${mm(p.standoff_mm)} mm${moves ? `${moves} mm` : ' · no moves'}` : 'UGV: not placed',
    room ? `Room: ${mm(room.width_mm)} × ${mm(room.length_mm)} mm · commode ${mm(room.left_mm)} mm from the left wall, `
      + `${mm(room.right_mm)} mm from the right, ${mm(room.front_mm)} mm from the front` : 'Room: not measured',
  ];
}

export function installSummary() {
  window.setupSummaryHtml = function setupSummaryHtml(rec, side) {
    const a = rec && rec.approvedSetup && rec.approvedSetup[side];
    if (!a) return '<span class="mini-note">—</span>';
    const [first, ...rest] = lines(rec, a);
    return `<div class="approved-setup lite-approved-setup"><div>${esc(first)}</div>${rest.map(t => `<div>${esc(t)}</div>`).join('')}`
      + `<div class="mini-note">Program: ${esc(a.programId || '—')}</div></div>`;
  };

  const pageApprove = window.saveApprovedArmSetup;
  window.saveApprovedArmSetup = function saveApprovedArmSetup(side) {
    pageApprove(side);
    const i = currentTask;
    facts(side).then((lite) => {
      const rec = taskRecords[i];
      if (!rec || !rec.approvedSetup || !rec.approvedSetup[side]) return;
      rec.approvedSetup[side].lite = lite;
      safeSaveTaskRecords();
      if (document.getElementById('view-project')?.classList.contains('active')) window.renderSummary();
    }).catch(() => {});
  };

  const pageRender = window.renderSummary;
  window.renderSummary = function renderSummary() {
    pageRender();
    if (ctx.pid && (!place || place.pid !== ctx.pid) && !asking) loadPlace().then(() => { if (place) pageRender(); });
  };
  document.addEventListener('lite:placement', () => { place = null; });
}
