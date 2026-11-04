// Camera Calibration (the Operator Journey's entry below Data Management; the user, 4 Nov): the right wrist camera's
// position on the wrist, measured with a chessboard on the closed lid (server/hand_eye_cal.py). Lite plans the
// views (rest, inspection pose 1 and inspection pose 4, each with wrist turns; every MoveJ checked offline against the
// room's walls, the commode, the board on its lid and the left arm), shows the tour in 3D, dry-runs it in Hardware
// sim, runs it in one go on the robot (no confirmation per move; Real asks once) and solves it with ALIX's solver.
// Nothing is applied.

import { get, post, say } from './api.js';
import { ctx, el, jobLine, robotCall, tell, watchJob } from './ctx.js';
import { borrow, Player } from './seq3d.js';
import { frame } from './vp.js';

const $ = (id) => document.getElementById(id);
const BUTTONS = ['liteCalPlan', 'liteCalShow', 'liteCalRun', 'liteCalRest', 'liteCalSolve'];
const MODE_WORDS = { visual: 'Visual sim', hwsim: 'Hardware sim', real: 'Real Machine Mode' };
let box = null;
let busy = false;

// the tour in the window's own 3D, nothing sent (as Advanced Robot Controls' simulation shows a move)
const view3d = {
  player: null, three: null, total: 0, done: null,
  async play(timeline, says) {
    const host = $('liteCalHost');
    if (!this.player) {
      this.player = new Player({ onTime: (t) => this.show(t), onEnd: () => { const f = this.done; this.done = null; if (f) f(); } });
      frame(host).title('The calibration tour', 'nothing is sent');
    }
    this.three = await borrow(host);
    this.three.robot.home();
    this.three.robot.clearTrack('left');
    this.total = this.three.robot.loadTimeline('right', timeline).total_s;
    $('liteCalTime').max = String(Math.round(this.total * 1000));
    frame(host).title(`The calibration tour · ${says}`, 'nothing is sent');
    this.player.load(this.total);
    await new Promise((resolve) => { this.done = resolve; this.player.play(); });
  },
  show(t) {
    const host = $('liteCalHost');
    if (this.three && this.three.viewer.host === host) {
      this.three.robot.showTime(t, 'right');
      this.three.viewer.dirty = true;
    }
    const bar = $('liteCalTime');
    if (bar && document.activeElement !== bar) bar.value = String(Math.round(t * 1000));
    frame(host).time(`${t.toFixed(1)} s / ${this.total.toFixed(1)} s`);
  },
};

const num = (id, label, value, attrs = {}) => el('label', { class: 'field' }, el('span', { text: label }),
  el('input', { id, type: 'number', value: String(value), ...attrs }));

function build() {
  const close = () => { box.classList.remove('show'); document.body.style.overflow = ''; view3d.player?.pause(); };
  box = el('div', { class: 'modal lite-viewer lite-cal-modal', id: 'liteCalibration' },
    el('div', { class: 'modal-card lite-viewer-card lite-cal-card', role: 'dialog', 'aria-modal': 'true', 'aria-label': 'Camera Calibration' },
      el('div', { class: 'section-head' },
        el('div', {}, el('div', { class: 'eyebrow', text: 'The right wrist camera' }),
          el('h3', { text: 'Camera Calibration' }),
          el('p', { class: 'mini-note', text: 'Where the D435 really sits on the right wrist, measured with a chessboard lying on the '
            + 'closed lid (RealMan’s eye-in-hand method, solved by ALIX’s solver). Lite plans the views around rest, inspection pose 1 '
            + 'and inspection pose 4, checks every move against the room, the commode, the board and the left arm, and runs them in one go. '
            + 'Nothing is applied to the robot’s model.' })),
        el('div', { class: 'cta-row' }, el('button', { class: 'btn light', type: 'button', text: 'Close', onclick: close }))),
      el('div', { class: 'lite-cal-grid' },
        el('div', { class: 'lite-cal-left' },
          el('ol', { class: 'lite-cal-steps', id: 'liteCalSteps' }),
          el('div', { class: 'lite-cal-row' },
            num('liteCalSqX', 'Squares along', 10, { min: '4', max: '20', step: '1' }),
            num('liteCalSqY', 'Squares across', 7, { min: '4', max: '20', step: '1' }),
            num('liteCalSquare', 'Measured square (mm)', '30.00', { min: '10', max: '60', step: '0.01' })),
          el('div', { class: 'lite-cal-row' },
            el('button', { class: 'btn', id: 'liteCalPlan', type: 'button', text: 'Plan the views', onclick: () => act(plan) }),
            el('button', { class: 'btn', id: 'liteCalShow', type: 'button', text: 'Show the tour in 3D', onclick: () => act(show) }),
            el('button', { class: 'btn primary', id: 'liteCalRun', type: 'button', text: 'Run the calibration', onclick: () => act(run) }),
            el('button', { class: 'btn', id: 'liteCalRest', type: 'button', text: 'Back to rest', onclick: () => act(rest) }),
            el('button', { class: 'btn', id: 'liteCalSolve', type: 'button', text: 'Solve again', onclick: () => act(solve) })),
          el('div', { id: 'liteCalSays', class: 'lite-status' }),
          el('div', { id: 'liteCalCoverage', class: 'mini-note' }),
          el('div', { class: 'lite-cal-tablebox' }, el('table', { class: 'lite-cal-views', id: 'liteCalViews' }))),
        el('div', { class: 'lite-cal-right' },
          el('div', { id: 'liteCalHost', class: 'lite-rev3d-host lite-cal-host' }),
          el('input', { id: 'liteCalTime', type: 'range', min: '0', max: '0', step: '10', value: '0', 'aria-label': 'Tour time',
            oninput: (e) => { view3d.player?.pause(); view3d.player?.seek(Number(e.target.value) / 1000); } }),
          el('div', { class: 'lite-cal-result', id: 'liteCalResult' })))));
  document.body.append(box);
  document.addEventListener('keydown', (e) => {
    if (e.key === 'Escape' && box.classList.contains('show') && !document.querySelector('.lite-confirm, .lite-picker-panel')) close();
  });
}

const says = (t) => { const s = $('liteCalSays'); if (s) s.textContent = t; };

async function act(fn) {
  if (busy) return;
  busy = true;
  for (const id of BUTTONS) $(id)?.setAttribute('disabled', '');
  try { await fn(); } catch (err) { tell(err); says(say(err)); } finally {
    busy = false;
    for (const id of BUTTONS) $(id)?.removeAttribute('disabled');
    await refresh();
  }
}

async function plan() {
  if (!ctx.pid) { window.toast('Open the project first: its room is what the moves are checked against.', 'warn'); return; }
  says('Planning the views: every MoveJ checked against the room’s walls, the commode, the board on the lid and the left arm ...');
  const r = await post('/api/calibration/plan', {
    pid: ctx.pid, squares: [Number($('liteCalSqX').value), Number($('liteCalSqY').value)], square_mm: Number($('liteCalSquare').value) });
  says(`${r.session.views} views planned in ${r.plan.seconds} s; the board may lie up to ${r.plan.tolerance_mm} mm from the lid’s middle, `
    + 'turned any way. Nothing moved.');
}

async function show() {
  if (!(await get('/api/calibration')).session) { window.toast('Plan the views first.', 'warn'); return; }
  says('The tour in 3D: nothing is sent to the robot.');
  const tl = await post('/api/calibration/preview', {});
  await view3d.play(tl, tl.what);
  says(`Shown: ${tl.what}. Nothing was sent.`);
}

async function run() {
  const d = await get('/api/calibration');
  const s = d.session;
  if (!s) { window.toast('Plan the views first.', 'warn'); return; }
  const left = d.mode === 'hwsim' ? s.views : s.views - s.next;
  if (left <= 0) { window.toast('Every planned view is taken: press Solve again to re-solve.', 'warn'); return; }
  const j = await robotCall('/api/calibration/run', {},
    { what: `The camera calibration: ${left} view(s) and back to rest at ${s.speed_pct} %, in one go` });
  let seen = -1;
  const done = await watchJob(j, (x) => { says(jobLine(x)); if (x.i !== seen) { seen = x.i; refresh(); } });
  const r = done.result || {};
  says(done.state !== 'done' ? (done.error || 'The calibration stopped.')
    : r.mode === 'hwsim' ? r.said
      : `${r.taken} of ${r.views} views taken${(r.missed || []).length ? `, ${r.missed.length} left out (board not found)` : ''}; `
        + (r.result ? (r.result.accepted ? 'solved: ACCEPTED.' : 'solved: NOT ACCEPTED (see the result).') : 'too few views to solve.'));
}

async function rest() {
  const j = await robotCall('/api/calibration/rest', {}, { what: 'Moving the right arm back to rest' });
  const done = await watchJob(j, (x) => says(jobLine(x)));
  says(done.state === 'done' ? 'The right arm is at rest.' : (done.error || 'The arm did not reach rest.'));
}

async function solve() {
  says('Solving the views taken again (five methods, then a refinement over every corner) ...');
  await post('/api/calibration/solve', {});
  says('Solved: see the result. Nothing was applied.');
}

const fmt = (v, n = 1) => (v == null ? '-' : Number(v).toFixed(n));

function paintSteps(d) {
  const s = d.session;
  const tol = s ? `${s.board.tolerance_mm} mm` : 'the planned distance';
  $('liteCalSteps').replaceChildren(...[
    `The board: ${d.board.squares[0]} x ${d.board.squares[1]} squares of ${d.board.square_mm} mm (${d.board.print}, printed at 100 % on A3 `
      + 'and glued flat), or the lab’s rigid plate: enter its squares and measure ten squares with calipers.',
    `Lay it flat on the closed lid, near its middle (within ${tol}), turned any way; the seat down, the left arm at its rest.`,
    'Plan the views. Nothing moves.',
    'Check them offline: Show the tour in 3D; in Hardware sim, Run the calibration runs the same MoveJs in the controllers’ SIM mode.',
    `In Real Machine Mode, Run the calibration: the right pole to ${d.pole_mm} mm, the ${s ? s.views : 15} views at ${d.speed_pct} %, `
      + 'back to rest, then the solve. STOP stops it; a new run goes on from the next view.',
  ].map((t) => el('li', { text: t })));
  if (!$('liteCalSays').textContent) says(`Lite is in ${MODE_WORDS[d.mode] || d.mode}.`);
}

function paint(d) {
  paintSteps(d);
  const s = d.session;
  const tab = $('liteCalViews');
  const cov = $('liteCalCoverage');
  const res = $('liteCalResult');
  tab.replaceChildren();
  res.replaceChildren();
  if (!s) { cov.textContent = 'No session yet: press Plan the views.'; return; }
  $('liteCalSqX').value = String(s.board.squares[0]);
  $('liteCalSqY').value = String(s.board.squares[1]);
  $('liteCalSquare').value = Number(s.board.square_mm).toFixed(2);
  const taken = new Map(s.rows.map((r) => [r.view, r]));
  tab.append(el('tr', {}, ...['View', 'Pose', 'Wrist turn (deg)', 'Planned: range, tilt', 'Taken: PnP, tilt, range'].map((t) => el('th', { text: t }))));
  for (const v of s.planned) {
    const o = v.offsets_deg;
    const turn = ['j1', 'j4', 'j5', 'j6', 'j7'].filter((k) => o[k]).map((k) => `${k.toUpperCase()} ${o[k] > 0 ? '+' : ''}${o[k]}`).join(', ') || 'none';
    const r = taken.get(v.index);
    tab.append(el('tr', { class: v.index === s.next ? 'next' : '' },
      el('td', { text: String(v.index + 1) }), el('td', { text: v.base.replace(/_/g, ' ') }), el('td', { text: turn }),
      el('td', { text: `${fmt(v.range_m, 2)} m, ${fmt(v.tilt_deg, 0)} deg` }),
      el('td', { text: r ? `${fmt(r.board_pose_rms_px, 2)} px, ${fmt(r.board_tilt_deg, 0)} deg, ${fmt(r.board_range_m, 2)} m`
        : (s.missed || []).includes(v.index) ? 'left out: board not found' : (v.index === s.next ? 'next' : '') })));
  }
  const dv = s.diversity || s.planned_diversity;
  cov.textContent = `${s.taken} of ${s.views} views taken; kept in ${s.folder}.`
    + (dv ? ` Wrist turn about X / Y / Z: ${dv.max_rotation_about_end_xyz_deg.join(' / ')} deg ${s.diversity ? 'so far' : 'planned'} (RealMan: over 30 each).` : '');
  const r = s.result;
  if (!r) { res.append(el('div', { class: 'mini-note', text: 'The result shows here once the views are solved.' })); return; }
  const sp = r.board_spread || {};
  const e = r.end_frame_correction || {};
  res.append(
    el('div', { class: r.accepted ? 'good' : 'warn', text: r.accepted ? 'ACCEPTED by ALIX’s limits' : 'NOT ACCEPTED by ALIX’s limits' }),
    el('div', { text: `The camera is ${fmt(r.translation_norm_mm)} mm and ${fmt(r.rotation_deg, 2)} deg from the URDF’s nominal mount `
      + `(${(r.translation_mm_in_end || []).map((x) => fmt(x)).join(' / ')} mm along the wrist frame’s X / Y / Z).` }),
    el('div', { text: `The board seen through the arm scatters ${fmt(sp.translation_rms_mm, 2)} mm and ${fmt(sp.rotation_rms_deg, 2)} deg RMS `
      + `(at most 3 mm); the corners reproject within ${fmt(r.reprojection_rms_px, 2)} px RMS (at most 1.5).` }),
    el('div', { text: e.xyz_mm ? `As a fixed correction after R_CameraHolderLink: xyz ${e.xyz_mm.join(' / ')} mm, rpy ${e.rpy_deg.join(' / ')} deg.` : '' }),
    el('div', { class: 'mini-note', text: `Written to ${r.file}. Nothing is applied: applying it changes the camera’s mount in the URDF, `
      + 'and the camera’s place references are taken again after it.' }));
}

async function refresh() {
  if (!box) return;
  try { paint(await get('/api/calibration')); } catch (err) { says(say(err)); }
}

export function openCalibration() {
  if (!box) build();
  box.classList.add('show');
  document.body.style.overflow = 'hidden';
  refresh();
}

// The Operator Journey's entry, below Data Management (the user, 4 Nov).
export function installCalibration() {
  if (document.querySelector('.lite-cal-nav')) return;
  const after = document.querySelector('.lite-data-nav') || document.querySelector('.navbtn[data-view="logs"]');
  if (!after) return;
  after.after(el('button', { class: 'navbtn lite-cal-nav', type: 'button', onclick: openCalibration },
    el('span', { class: 'navicon', text: '◎' }), el('span', { text: 'Camera Calibration' })));
}
