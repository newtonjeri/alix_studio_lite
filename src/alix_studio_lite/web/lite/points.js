// The controller's global points (the owner, 14 Oct): listed from each linked arm's controller and
// moved to with MoveJ or MoveL, both by the control program's own runner v14 functions on the
// server (GET /api/robots/<arm>/points, POST …/points/move). Used by the start-pose lists (Setup &
// Safety, Redo Preparation: prepare.js) and Advanced Robot Controls' GLOBAL POINTS (advanced.js).

import { get } from './api.js';
import { el, robotCall, watchJob } from './ctx.js';

export const POINT_SPEED = 20;                   // % : the control program's default for a global point

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

export const hasController = (side) => reach[side];

// The arm's global points, read again when `force` or after a minute: {points, note, total}.
export async function loadPoints(side, force = false) {
  const c = lists[side];
  if (!force && c && Date.now() - c.at < 60000) return c.data;
  const data = await get(`/api/robots/${side}/points`);
  lists[side] = { at: Date.now(), data };
  return data;
}

// MoveJ / MoveL to a global point; resolves the finished job. `extra`: {confirmed} when asked already.
export async function movePoint(side, name, move = 'movej', speed = 20, waitS = 0, extra = {}, onProgress = null) {
  const job = await robotCall(`/api/robots/${side}/points/move`, { name, move, speed, wait_s: waitS, ...extra },
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

// A search field above a list of global points and the motion beside it, made once (`${id}Search`,
// `${id}Motion`); shown only while the arm has a controller (`show`).
export function pointControls(sel, id, show = true) {
  if (!sel) return;
  if (!document.getElementById(`${id}Search`)) {
    const search = el('input', { type: 'search', id: `${id}Search`, class: 'lite-point-search', placeholder: 'Search the global points',
      'aria-label': 'Search the global points', oninput: (e) => filterOptions(sel, e.target.value) });
    const motion = el('select', { id: `${id}Motion`, class: 'lite-point-motion', 'aria-label': 'Motion to the point',
      title: `How the arm moves to a global point (at ${POINT_SPEED} %)` },
    el('option', { value: 'movej', text: 'MoveJ' }), el('option', { value: 'movel', text: 'MoveL' }));
    (sel.closest('.inline') || sel).before(search);
    sel.after(motion);
  }
  for (const k of ['Search', 'Motion']) { const e = document.getElementById(`${id}${k}`); if (e) e.hidden = !show; }
}

// The arm's global points as a group at the end of a page's own list (its own options stay first):
// Prepare's Region start pose and Real Test's Move to start (feedback v2, slide 7). Returns how many.
export async function fillPointGroup(sel, side) {
  if (!sel) return 0;
  const clear = () => sel.querySelectorAll('optgroup.lite-points').forEach((g) => g.remove());
  if (!hasController(side)) { clear(); return 0; }
  const d = await loadPoints(side);
  clear();                                       // after the read: refreshes that overlap leave one group
  sel.append(el('optgroup', { class: 'lite-points', label: `${side.toUpperCase()} controller · ${d.points.length} global points` },
    d.points.map((pt) => el('option', { value: `gp:${pt.name}`, text: pt.name }))));
  return d.points.length;
}
