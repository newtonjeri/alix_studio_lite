// The controller's global points (the owner, 14 Oct): listed from each linked arm's controller and
// moved to with MoveJ or MoveL, both by the control program's own runner v14 functions on the
// server (GET /api/robots/<arm>/points, POST …/points/move). Used by the start-pose lists (Setup &
// Safety, Redo Preparation: prepare.js) and Advanced Robot Controls' GLOBAL POINTS (advanced.js).

import { get } from './api.js';
import { el, robotCall, watchJob } from './ctx.js';
import { pointPicker } from './picker.js';
import { speedPct } from './status.js';

const lists = { left: null, right: null };      // {at, data}: the last list read from each controller
const reach = { left: false, right: false };     // the arm is connected with the live link: it has a controller

document.addEventListener('lite:status', (e) => {
  for (const side of ['left', 'right']) {
    const a = (e.detail && e.detail.arms && e.detail.arms[side]) || {};
    const now = !!(a.connected && a.link);
    if (now !== reach[side]) {
      reach[side] = now;
      lists[side] = null;                            // connected again: read it again
      document.dispatchEvent(new CustomEvent('lite:points', { detail: side }));
    }
  }
});

// The controller was read again (Data Management, S2's Refresh, S1): every list of its points follows.
document.addEventListener('lite:controller', (e) => {
  const side = e.detail && e.detail.side;
  for (const s of side ? [side] : ['left', 'right']) {
    lists[s] = null;
    document.dispatchEvent(new CustomEvent('lite:points', { detail: s }));
  }
});

export const hasController = (side) => reach[side];

// The arm's global points as Data Management's content has them (server/content.py), read from the controller
// again when `force` (a Refresh: completely): {points, note, total, read}.
export async function loadPoints(side, force = false) {
  const c = lists[side];
  if (!force && c && Date.now() - c.at < 30000) return c.data;
  const data = await get(`/api/robots/${side}/points${force ? '?refresh=1' : ''}`);
  lists[side] = { at: Date.now(), data };
  return data;
}

// After a list is filled again: a choice of a point the controller no longer holds is cleared, and said.
export function dropGone(sel, was, side) {
  if (!sel || !String(was).startsWith('gp:') || [...sel.options].some(o => o.value === was)) return false;
  sel.dispatchEvent(new Event('change', { bubbles: true }));
  window.toast(`“${String(was).slice(3)}” is no longer on the ${side.toUpperCase()} controller: its choice was cleared.`);
  return true;
}

// MoveJ / MoveL to a global point; resolves the finished job. `extra`: {confirmed} when asked already.
// At the speed slider's % (the server's own, the same for every MoveJ / MoveL of both arms).
export async function movePoint(side, name, move = 'movej', waitS = 0, extra = {}, onProgress = null) {
  const job = await robotCall(`/api/robots/${side}/points/move`, { name, move, wait_s: waitS, ...extra },
    { what: `${move === 'movel' ? 'MoveL' : 'MoveJ'} of the ${side.toUpperCase()} arm to “${name}”` });
  return watchJob(job, onProgress);
}

export const matches = (name, q) => !q || String(name).toLowerCase().includes(String(q).trim().toLowerCase());

// Hides the options of a list that do not match the search; the selection moves to the first shown.
export function filterOptions(sel, q) {
  if (!sel) return;
  let first = null;
  for (const o of sel.options) {
    o.hidden = !matches(o.textContent, q);
    if (!o.hidden && !first) first = o;
  }
  if (sel.selectedOptions[0]?.hidden && first) sel.value = first.value;
}

// A list of global points as a field that opens a searchable list (picker.js, the owner, 16 Oct), with
// the motion beside it, made once (`${id}Motion`, shown only while the arm has a controller: `show`).
export function pointControls(sel, id, show = true) {
  if (!sel) return;
  pointPicker(sel);
  if (!document.getElementById(`${id}Motion`)) {
    const motion = el('select', { id: `${id}Motion`, class: 'lite-point-motion', 'aria-label': 'Motion to the point',
      title: 'How the arm moves to a global point (at the speed slider’s %, top bar)' },
    el('option', { value: 'movej', text: 'MoveJ' }), el('option', { value: 'movel', text: 'MoveL' }));
    sel.after(motion);
  }
  const m = document.getElementById(`${id}Motion`);
  if (m) m.hidden = !show;
}

// What a point is, under its name in the lists: its joints and its frames.
export function pointDetail(pt) {
  const j = Array.isArray(pt.joints) && pt.joints.length ? `J1–J7 ${pt.joints.map(v => Number(v).toFixed(1)).join(', ')}°` : '';
  return [j, [pt.work_frame, pt.tool_frame].filter(Boolean).join(' / ')].filter(Boolean).join(' · ');
}

// The arm's global points as a group at the end of a page's own list (its own options stay first):
// Prepare's Region start pose and Real Test's Move to start (feedback v2, slide 7). Returns how many.
export async function fillPointGroup(sel, side) {
  if (!sel) return 0;
  const clear = () => sel.querySelectorAll('optgroup.lite-points').forEach((g) => g.remove());
  if (!hasController(side)) { clear(); return 0; }
  const d = await loadPoints(side);
  const was = sel.value;
  clear();                                       // after the read: refreshes that overlap leave one group
  sel.append(el('optgroup', { class: 'lite-points', label: `${side.toUpperCase()} controller · ${d.points.length} global points` },
    d.points.map((pt) => el('option', { value: `gp:${pt.name}`, text: pt.name, 'data-detail': pointDetail(pt) }))));
  if ([...sel.options].some(o => o.value === was)) sel.value = was;
  else dropGone(sel, was, side);
  return d.points.length;
}
