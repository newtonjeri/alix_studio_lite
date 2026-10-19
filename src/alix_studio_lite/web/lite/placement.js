// The room, the commode in it and each task's UGV placement (V32 feedback v2, slide 1; the owner,
// 15 Oct), in Prepare. Kept on the server (server/placement.py), which builds the 3D scene from them
// in ALIX's robot base frame (X forward towards the commode, Y to the robot's left):
// - "Room & commode", once per project: the room's width and length, and where the commode stands
//   in it (left wall to its left edge, its right edge to the right wall, its front tip to the front
//   wall), as seen facing the commode;
// - "UGV placement", once per task (both arms ride the UGV): the STANDOFF, required for every task,
//   from the front face of the UGV's base to the commode's front tip, and the page's four moves from
//   there (Front / Rear along X, Left / Right along Y; no rotation).
// A guide with a top view says how to take each measurement. Every recording keeps the placement of
// its task when it is taught or loaded, and the 3D plays it there.

import { get, post } from './api.js';
import { ctx, el, tell } from './ctx.js';

const $ = (id) => document.getElementById(id);
const ROOM = [
  ['width_mm', 'Room width', 'Left wall to right wall'],
  ['length_mm', 'Room length', 'Wall behind the commode to the front wall'],
  ['left_mm', 'Left wall → commode', 'To the commode’s left edge'],
  ['right_mm', 'Commode → right wall', 'From the commode’s right edge'],
  ['front_mm', 'Commode front → front wall', 'From the commode’s front tip'],
  ['back_mm', 'Commode back → wall behind', 'From the commode’s back; 0 when it stands against the wall'],
];
const MOVES = [['ugvFrontMm', 'front_mm', 'Move front (mm, +X)'], ['ugvRearMm', 'rear_mm', 'Move rear (mm, −X)'],
  ['ugvLeftMm', 'left_mm', 'Move left (mm, +Y)'], ['ugvRightMm', 'right_mm', 'Move right (mm, −Y)']];

let view = null;                                   // the server's placement for the task on screen

// How far the lid and the seat open before the wall behind (server/fixtures.wall_clamp): said beside the saved room.
function lidSentence(ls) {
  const parts = Object.entries((ls && ls.parts) || {}).filter(([, c]) => c.touches_wall);
  if (!parts.length) return '';
  const say = parts.map(([name, c]) => `the ${name} to ${Math.abs(c.used_deg).toFixed(1)}° (ALIX's model: ${Math.abs(c.alix_deg).toFixed(0)}°)`);
  return ` The wall behind lets ${say.join(' and ')} open: the 3D and the collision check stop there.`;
}
const task = () => TASKS[currentTask]?.label || null;
const num = (v) => (v === '' || v == null ? null : Number(v));
const changed = () => document.dispatchEvent(new CustomEvent('lite:placement'));

export async function loadPlacement() {
  if (!ctx.pid) return null;
  view = await get(`/api/projects/${ctx.pid}/placement${task() ? `?task=${encodeURIComponent(task())}` : ''}`);
  return view;
}

// What teaching still needs here: the task's standoff, saved.
export async function standoffSaved() {
  if (!ctx.pid) return true;
  const v = view && view.task === task() ? view : await loadPlacement().catch(() => null);
  return !!(v && v.tasks[task()] && v.tasks[task()].standoff_mm != null);
}

// ── the cards

function guideButton() {
  return el('button', { class: 'btn light lite-guide-btn', type: 'button', text: '📐 How to measure', onclick: openGuide });
}

function roomCard() {
  if ($('liteRoomCard')) return;
  const ugv = document.querySelector('.ugv-position-card');
  if (!ugv) return;
  const card = el('div', { class: 'ugv-position-card lite-room-card', id: 'liteRoomCard' },
    el('div', { class: 'section-head compact-head ugv-position-head' },
      el('div', {}, el('h3', {}, 'Room & commode · this project ', el('span', { class: 'mandatory-star', text: '*' })),
        el('p', { text: 'Measured once for this toilet room, in millimetres. The 3D view builds the walls and places the commode from them.' })),
      guideButton()),
    el('div', { class: 'ugv-position-grid lite-room-grid' }, ROOM.map(([k, name, hint]) =>
      el('div', { class: 'field' }, el('label', { for: `liteRoom_${k}` }, `${name} (mm) `, el('span', { class: 'mandatory-star', text: '*' })),
        el('input', { id: `liteRoom_${k}`, type: 'number', min: '1', step: '1', placeholder: hint, title: hint })))),
    el('div', { id: 'liteRoomTotals', class: 'lite-room-totals', hidden: true }),
    el('div', { class: 'ugv-position-footer' },
      el('span', { id: 'liteRoomNote', class: 'mini-note', text: 'Not measured yet: the 3D view shows ALIX’s standard cubicle.' }),
      el('button', { class: 'btn light', type: 'button', text: '💾 Save Room', onclick: saveRoom })));
  ugv.before(card);
}

function ugvCard() {
  const card = document.querySelector('.ugv-position-card:not(.lite-room-card)');
  if (!card || $('liteStandoffMm')) return;
  const head = card.querySelector('.ugv-position-head');
  const h3 = head?.querySelector('h3');
  if (h3) h3.replaceChildren('UGV placement · this task, both arms ', el('span', { class: 'mandatory-star', text: '*' }));
  const p = head?.querySelector('p');
  if (p) p.textContent = 'Required for every task: the UGV’s standoff from the commode. Then any move from there, in millimetres (no rotation).';
  head?.querySelector('#ugvPositionTaskBadge')?.before(guideButton());
  const range = (view && view.standoff_range_mm) || [250, 2000];
  card.querySelector('.ugv-position-grid')?.before(el('div', { class: 'lite-standoff-row' },
    el('div', { class: 'field' }, el('label', { for: 'liteStandoffMm' }, 'Standoff (mm) ', el('span', { class: 'mandatory-star', text: '*' })),
      el('input', { id: 'liteStandoffMm', type: 'number', min: String(range[0]), max: String(range[1]), step: '1', placeholder: 'Required' })),
    el('div', { class: 'mini-note', id: 'liteStandoffHint',
      text: `From the front face of the UGV’s base, at floor level, to the commode’s front tip, with the UGV parked centred on the commode and facing it. ALIX’s range: ${range[0]}–${range[1]} mm.` })));
  for (const [id, , label] of MOVES) {
    const lab = $(id)?.closest('.field')?.querySelector('label');
    if (lab) lab.textContent = label;
    if ($(id)) { $(id).placeholder = '0'; $(id).min = '0'; }
  }
  const save = card.querySelector('.ugv-position-footer .btn');
  if (save) save.textContent = '💾 Save UGV Placement';
}

function fill() {
  if (!view) return;
  for (const [k] of ROOM) if ($(`liteRoom_${k}`)) $(`liteRoom_${k}`).value = view.room ? view.room[k] : '';
  const t = view.tasks[task()] || null;
  if ($('liteStandoffMm')) $('liteStandoffMm').value = t && t.standoff_mm != null ? t.standoff_mm : '';
  for (const [id, k] of MOVES) if ($(id)) $(id).value = t && t[k] != null ? t[k] : '';
  const notes = view.scene.notes || [];
  const rn = $('liteRoomNote');
  if (rn) {
    const roomNotes = notes.filter(n => !n.startsWith('No UGV standoff'));
    rn.textContent = !view.room ? 'Not measured yet: the 3D view shows ALIX’s standard cubicle.'
      : roomNotes.length ? `Warning: ${roomNotes.join(' ')}` : `Saved · ${view.room.width_mm} × ${view.room.length_mm} mm room; the commode `
        + `${view.room.left_mm} mm from the left wall, ${view.room.right_mm} mm from the right, ${view.room.front_mm} mm from the front, `
        + `${view.room.back_mm ?? 0} mm from the wall behind.` + lidSentence(view.scene.lid_seat);
    rn.classList.toggle('lite-warn', roomNotes.length > 0);
  }
  paintTotals(view.room ? view.scene.totals || [] : []);
  const un = $('ugvPositionSavedNote');
  if (un) {
    un.textContent = t ? `Saved for ${task()} · standoff ${t.standoff_mm} mm`
      + MOVES.filter(([, k]) => t[k]).map(([, k, label]) => ` · ${label.split(' (')[0].toLowerCase()} ${t[k]} mm`).join('')
      : `Enter the standoff for ${task() || 'this task'} and save it: teaching needs it.`;
    un.classList.toggle('lite-bad', !t);
  }
  const badge = $('ugvPositionTaskBadge');
  if (badge) badge.textContent = task() || 'TASK';
}

// The room's totals against their parts, each within ±10 mm (the owner, 16 Oct): the values always,
// in orange past the tolerance. A warning only: the 3D builds the walls from what was entered.
function paintTotals(totals) {
  const box = $('liteRoomTotals');
  if (!box) return;
  const mm = (v) => Math.round(v).toLocaleString('en-US');
  box.hidden = !totals.length;
  box.replaceChildren(...totals.map((t) => {
    const sums = t.parts.map(p => (p.name === 'commode' ? `${mm(p.mm)} (commode)` : mm(p.mm))).join(' + ');
    const off = Math.abs(t.diff_mm);
    return el('div', { class: `lite-room-total ${t.ok ? 'ok' : 'warn'}` },
      el('b', { text: `${t.title}: ` }), `${sums} = ${mm(t.sum_mm)} mm · measured ${mm(t.room_mm)} mm · `,
      el('span', { class: t.ok ? 'lite-ok' : 'lite-warn',
        text: t.ok ? `within ±${t.tol_mm} mm` : `${mm(off)} mm apart, ${mm(off - t.tol_mm)} mm past ±${t.tol_mm} mm` }));
  }));
}

export async function refreshPlacement() {
  if (!ctx.pid || !document.querySelector('.ugv-position-card')) return;
  try { await loadPlacement(); } catch (_) { return; }
  roomCard();
  ugvCard();
  fill();
}

async function saveRoom() {
  if (!ctx.pid) { window.toast('Open a project first.'); return; }
  const body = Object.fromEntries(ROOM.map(([k]) => [k, num($(`liteRoom_${k}`)?.value)]));
  try {
    view = await post(`/api/projects/${ctx.pid}/placement/room`, { ...body, task: task() });
    fill();
    changed();
    const notes = (view.scene.notes || []).filter(n => !n.startsWith('No UGV standoff'));
    window.toast(notes.length ? `Room saved, with a warning: ${notes.join(' ')}` : 'Room saved: the 3D view uses these walls.');
  } catch (err) { tell(err); }
}

async function saveTask() {
  if (!ctx.pid) { window.toast('Open a project first.'); return false; }
  const body = { task: task(), standoff_mm: num($('liteStandoffMm')?.value) };
  for (const [id, k] of MOVES) body[k] = num($(id)?.value);
  try {
    view = await post(`/api/projects/${ctx.pid}/placement/task`, body);
    fill();
    changed();
    // the page's own records keep the moves per arm: the same for both arms (one UGV)
    const rec = window.taskRecordForStartTools();
    const ugv = { frontMm: body.front_mm, rearMm: body.rear_mm, leftMm: body.left_mm, rightMm: body.right_mm };
    rec.ugvPositionByArm = { left: ugv, right: ugv };
    rec.ugvStandoffMm = body.standoff_mm;
    window.safeSaveTaskRecords();
    window.toast(`UGV placement saved for ${task()}: standoff ${body.standoff_mm} mm (both arms).`);
    return true;
  } catch (err) { tell(err); return false; }
}

// ── the guide

const GUIDE_SVG = `
<svg viewBox="0 0 420 470" role="img" aria-label="Top view of the room, the commode and the UGV with the measurements">
  <defs><marker id="liteGuideArrow" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">
    <path class="lite-guide-arrowhead" d="M0,0 L10,5 L0,10 z"/></marker></defs>
  <path class="lite-guide-wall" d="M60,440 L60,40 L360,40 L360,440"/>
  <line class="lite-guide-front" x1="60" y1="440" x2="360" y2="440"/>
  <text class="lite-guide-label" x="210" y="462" text-anchor="middle">front wall (door side)</text>
  <text class="lite-guide-label" x="210" y="30" text-anchor="middle">wall behind the commode</text>
  <rect class="lite-guide-commode" x="171" y="42" width="78" height="112" rx="10"/>
  <text class="lite-guide-label" x="210" y="102" text-anchor="middle">commode</text>
  <rect class="lite-guide-ugv" x="162" y="236" width="96" height="96" rx="6"/>
  <text class="lite-guide-label" x="210" y="288" text-anchor="middle">UGV</text>
  <line class="lite-guide-axis" x1="210" y1="300" x2="210" y2="262" marker-end="url(#liteGuideArrow)"/>
  <text class="lite-guide-axis-label" x="216" y="260">X</text>
  <line class="lite-guide-axis" x1="210" y1="300" x2="178" y2="300" marker-end="url(#liteGuideArrow)"/>
  <text class="lite-guide-axis-label" x="170" y="314">Y</text>
  <line class="lite-guide-dim" x1="62" y1="420" x2="358" y2="420" marker-start="url(#liteGuideArrow)" marker-end="url(#liteGuideArrow)"/>
  <text class="lite-guide-num" x="100" y="414">1</text>
  <line class="lite-guide-dim" x1="340" y1="42" x2="340" y2="438" marker-start="url(#liteGuideArrow)" marker-end="url(#liteGuideArrow)"/>
  <text class="lite-guide-num" x="346" y="200">2</text>
  <line class="lite-guide-dim" x1="62" y1="96" x2="169" y2="96" marker-start="url(#liteGuideArrow)" marker-end="url(#liteGuideArrow)"/>
  <text class="lite-guide-num" x="110" y="90">3</text>
  <line class="lite-guide-dim" x1="251" y1="96" x2="358" y2="96" marker-start="url(#liteGuideArrow)" marker-end="url(#liteGuideArrow)"/>
  <text class="lite-guide-num" x="300" y="90">4</text>
  <line class="lite-guide-dim" x1="150" y1="156" x2="150" y2="438" marker-start="url(#liteGuideArrow)" marker-end="url(#liteGuideArrow)"/>
  <text class="lite-guide-num" x="134" y="380">5</text>
  <line class="lite-guide-dim lite-guide-standoff" x1="236" y1="156" x2="236" y2="234" marker-start="url(#liteGuideArrow)" marker-end="url(#liteGuideArrow)"/>
  <text class="lite-guide-num" x="242" y="200">6</text>
</svg>`;

function openGuide() {
  const width = view && view.scene ? Math.round(view.scene.commode_width_mm) : 368;
  const depth = view && view.scene && view.scene.commode_depth_mm ? Math.round(view.scene.commode_depth_mm) : 538;
  const steps = [
    ['Room width', 'From the inside face of the left wall to the inside face of the right wall.'],
    ['Room length', 'From the wall behind the commode to the front wall (the door side).'],
    ['Left wall → commode', 'From the left wall to the commode’s left edge, its widest point (left and right as you face the commode).'],
    ['Commode → right wall', 'From the commode’s right edge to the right wall.'],
    ['Commode front → front wall', 'From the commode’s front tip to the front wall.'],
    ['Standoff, every task', 'Park the UGV centred on the commode and facing it. Measure from the front face of the UGV’s base, at floor '
      + 'level, to the commode’s front tip. Not from the lip 30–40 cm up the body: it stands 21 mm further forward.'],
  ];
  const box = el('div', { class: 'modal show lite-confirm', id: 'litePlacementGuide' },
    el('div', { class: 'modal-card accessible-modal lite-guide-card', role: 'dialog', 'aria-modal': 'true', 'aria-labelledby': 'liteGuideTitle' },
      el('h3', { id: 'liteGuideTitle', text: 'How to measure the room, the commode and the UGV' }),
      el('p', { class: 'mini-note', text: 'With a tape, horizontally, in millimetres. Measure the room and the commode once per project; the standoff for every task.' }),
      el('div', { class: 'lite-guide-body' },
        el('div', { class: 'lite-guide-figure' }),
        el('ol', { class: 'lite-guide-steps' }, steps.map(([name, text]) => el('li', {}, el('b', { text: name }), ` ${text}`)),
          el('li', { class: 'lite-guide-moves' }, el('b', { text: 'Moves (only if this task needs them)' }),
            ' If the UGV must stand off that position: Front / Rear along the robot’s X (front is towards the commode), Left / Right along its Y. No rotation.'))),
      el('div', { class: 'notice lite-guide-check' },
        `Lite checks the totals, each within ±10 mm: the left distance + the commode’s ${width} mm + the right distance should make the `
        + `room width, and the front distance + the commode’s ${depth} mm depth the room length (the commode standing at the wall behind it). `
        + 'Past that it warns with the values. The commode, the UGV and their sizes are ALIX’s own.'),
      el('div', { class: 'modal-actions' }, el('button', { class: 'btn', type: 'button', text: 'Close', onclick: () => box.remove() }))));
  box.querySelector('.lite-guide-figure').innerHTML = GUIDE_SVG;
  box.addEventListener('click', (e) => { if (e.target === box) box.remove(); });
  document.body.append(box);
  box.querySelector('.modal-actions .btn').focus();
}

export function installPlacement() {
  const pageSave = window.savePrepareArmSetup;
  window.savePrepareArmSetup = async function savePrepareArmSetup() {
    if (!ctx.pid) return pageSave();
    window.storePrepareInputsForArm(selectedArm, false);           // the page's snapshot of this arm
    return saveTask();
  };
  const pageLoad = window.loadPrepareInputsForArm;
  window.loadPrepareInputsForArm = function loadPrepareInputsForArm(...args) {
    const out = pageLoad.apply(this, args);
    if (ctx.pid) refreshPlacement();                                  // the task's placement, from the server
    return out;
  };
  document.addEventListener('lite:step', refreshPlacement);
  document.addEventListener('lite:stage', (e) => { if (e.detail === 'prepare') refreshPlacement(); });
}
