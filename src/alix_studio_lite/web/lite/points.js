// The controller's global points (the owner, 14 Oct): listed from each linked arm's controller and
// moved to with MoveJ or MoveL, both by the control program's own runner v14 functions on the
// server (GET /api/robots/<arm>/points, POST …/points/move). Used by the start-pose lists (Setup &
// Safety, Redo Preparation: prepare.js) and Advanced Robot Controls' GLOBAL POINTS (advanced.js).

import { get } from './api.js';
import { robotCall, watchJob } from './ctx.js';

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
