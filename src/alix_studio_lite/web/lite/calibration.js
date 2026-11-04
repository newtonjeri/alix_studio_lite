// Camera calibration (Advanced Robot Controls): the right wrist camera's position on the wrist, measured with a
// chessboard on the closed lid (server/hand_eye_cal.py). Lite plans the views (rest, inspection pose 1 and
// inspection pose 4, each with wrist turns, every move checked against the room, the commode and the left arm),
// the operator confirms each move, and ALIX's solver gives the result. Nothing is applied.

import { get, post, say } from './api.js';
import { ctx, el, jobLine, robotCall, tell, watchJob } from './ctx.js';

const $ = (id) => document.getElementById(id);
let busy = false;

function card() {
  let c = $('liteCalCard');
  if (c) return c;
  const after = document.querySelector('#robotControlModal .rc-dual-toolbar');
  if (!after) return null;
  c = el('div', { id: 'liteCalCard', class: 'lite-rc-sim lite-cal' },
    el('div', { class: 'lite-section-title' }, el('h3', { text: 'Camera calibration' }),
      el('span', { class: 'mini-note', id: 'liteCalIntro' })),
    el('div', { class: 'lite-cal-row' },
      el('label', { class: 'field' }, el('span', { text: 'Measured square (mm)' }),
        el('input', { id: 'liteCalSquare', type: 'number', step: '0.01', min: '20', max: '40', value: '30.00' })),
      el('button', { class: 'btn', id: 'liteCalPlan', text: 'Plan the views', onclick: () => act(plan) }),
      el('button', { class: 'btn primary', id: 'liteCalNext', text: 'Take the next view', onclick: () => act(next) }),
      el('button', { class: 'btn', id: 'liteCalRest', text: 'Back to rest', onclick: () => act(rest) }),
      el('button', { class: 'btn', id: 'liteCalSolve', text: 'Solve', onclick: () => act(solve) })),
    el('div', { class: 'mini-note', id: 'liteCalSays' }),
    el('div', { class: 'mini-note', id: 'liteCalCoverage' }),
    el('table', { class: 'lite-cal-views', id: 'liteCalViews' }),
    el('div', { class: 'lite-cal-result', id: 'liteCalResult' }));
  after.after(c);
  return c;
}

const says = (t) => { const s = $('liteCalSays'); if (s) s.textContent = t; };

async function act(fn) {
  if (busy) return;
  busy = true;
  for (const id of ['liteCalPlan', 'liteCalNext', 'liteCalRest', 'liteCalSolve']) $(id)?.setAttribute('disabled', '');
  try { await fn(); } catch (err) { tell(err); says(say(err)); } finally {
    busy = false;
    for (const id of ['liteCalPlan', 'liteCalNext', 'liteCalRest', 'liteCalSolve']) $(id)?.removeAttribute('disabled');
    await refresh();
  }
}

async function plan() {
  if (!ctx.pid) { window.toast('Open the project first: its room is what the moves are checked against.', 'warn'); return; }
  says('Planning the views (each move checked against the room, the commode and the left arm) ...');
  const r = await post('/api/calibration/plan', { pid: ctx.pid, square_mm: Number($('liteCalSquare').value) });
  const d = r.plan.diversity;
  says(`${r.session.views} views planned in ${r.plan.seconds} s. Nothing moved.`
    + (d.enough ? '' : ' The wrist turns fall short of RealMan\'s 30 deg about an axis: the result may not be accepted.'));
}

async function next() {
  const s = (await get('/api/calibration')).session;
  if (!s) { window.toast('Plan the views first.', 'warn'); return; }
  if (s.next >= s.views) { window.toast('Every planned view is taken: press Solve.', 'warn'); return; }
  const v = s.planned[s.next];
  const j = await robotCall('/api/calibration/next', {},
    { what: `Moving the right arm to calibration view ${s.next + 1} of ${s.views} (${v.base.replace(/_/g, ' ')})` });
  const done = await watchJob(j, (x) => says(jobLine(x)));
  says(done.state === 'done' ? `View ${s.next + 1} taken.` : (done.error || 'The view was not taken.'));
}

async function rest() {
  const j = await robotCall('/api/calibration/rest', {}, { what: 'Moving the right arm back to rest' });
  const done = await watchJob(j, (x) => says(jobLine(x)));
  says(done.state === 'done' ? 'The right arm is at rest.' : (done.error || 'The arm did not reach rest.'));
}

async function solve() {
  says('Solving (five methods, then a refinement over every corner) ...');
  await post('/api/calibration/solve', {});
  says('Solved: see the result below. Nothing was applied.');
}

const fmt = (v, n = 1) => (v == null ? '-' : Number(v).toFixed(n));

function paint(d) {
  const intro = $('liteCalIntro');
  if (intro) {
    intro.textContent = `Lay the ${d.board.squares} chessboard of ${d.board.square_mm} mm squares (${d.board.print}, printed at 100 % on A3) `
      + `flat on the closed lid, its long side pointing away from the robot. The right pole goes to ${d.pole_mm} mm; the arm moves at `
      + `${d.speed_pct} % and you confirm each move.`;
  }
  const s = d.session;
  const tab = $('liteCalViews');
  const cov = $('liteCalCoverage');
  const res = $('liteCalResult');
  if (!tab || !cov || !res) return;
  tab.replaceChildren();
  res.replaceChildren();
  if (!s) { cov.textContent = 'No session yet: press Plan the views.'; return; }
  const taken = new Map(s.rows.map((r) => [r.view, r]));
  tab.append(el('tr', {}, ...['View', 'Pose', 'Wrist turn (deg)', 'Planned range, tilt', 'Taken: PnP, tilt, range'].map((t) => el('th', { text: t }))));
  for (const v of s.planned) {
    const o = v.offsets_deg;
    const turn = ['j1', 'j4', 'j5', 'j6', 'j7'].filter((k) => o[k]).map((k) => `${k.toUpperCase()} ${o[k] > 0 ? '+' : ''}${o[k]}`).join(', ') || 'none';
    const r = taken.get(v.index);
    tab.append(el('tr', { class: v.index === s.next ? 'next' : '' },
      el('td', { text: String(v.index + 1) }), el('td', { text: v.base.replace(/_/g, ' ') }), el('td', { text: turn }),
      el('td', { text: `${fmt(v.range_m, 2)} m, ${fmt(v.tilt_deg, 0)} deg` }),
      el('td', { text: r ? `${fmt(r.board_pose_rms_px, 2)} px, ${fmt(r.board_tilt_deg, 0)} deg, ${fmt(r.board_range_m, 2)} m` : (v.index === s.next ? 'next' : '') })));
  }
  const dv = s.diversity || s.planned_diversity;
  cov.textContent = `${s.taken} of ${s.views} views taken (${s.folder}).`
    + (dv ? ` Wrist turn about X / Y / Z: ${dv.max_rotation_about_end_xyz_deg.join(' / ')} deg ${s.diversity ? 'so far' : 'planned'} (RealMan: over 30 each).` : '');
  const r = s.result;
  if (r) {
    const sp = r.board_spread || {};
    const e = r.end_frame_correction || {};
    res.append(
      el('div', { class: r.accepted ? 'good' : 'warn', text: r.accepted ? 'ACCEPTED by ALIX\'s limits' : 'NOT ACCEPTED by ALIX\'s limits' }),
      el('div', { text: `The camera is ${fmt(r.translation_norm_mm)} mm and ${fmt(r.rotation_deg, 2)} deg from the URDF's nominal mount `
        + `(${(r.translation_mm_in_end || []).map((x) => fmt(x)).join(' / ')} mm along the wrist frame's X / Y / Z).` }),
      el('div', { text: `The board's spread through the arm: ${fmt(sp.translation_rms_mm, 2)} mm RMS, ${fmt(sp.rotation_rms_deg, 2)} deg RMS `
        + `(at most 3 mm); reprojection ${fmt(r.reprojection_rms_px, 2)} px RMS (at most 1.5).` }),
      el('div', { text: e.xyz_mm ? `As a fixed correction after R_CameraHolderLink: xyz ${e.xyz_mm.join(' / ')} mm, rpy ${e.rpy_deg.join(' / ')} deg.` : '' }),
      el('div', { class: 'mini-note', text: `Written to ${r.file}. Nothing is applied: applying it changes the camera's mount in the URDF, and the camera's place references are taken again after it.` }));
  }
}

async function refresh() {
  if (!card()) return;
  try { paint(await get('/api/calibration')); } catch (err) { says(say(err)); }
}

export function installCalibration() {
  if (!card()) return;
  const modal = $('robotControlModal');
  if (modal) new MutationObserver(() => { if (modal.classList.contains('show')) refresh(); }).observe(modal, { attributes: true, attributeFilter: ['class'] });
  refresh();
}
