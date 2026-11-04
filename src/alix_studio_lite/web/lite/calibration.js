// Camera Calibration (the Operator Journey's entry below Data Management; the user, 4 Nov): the right wrist camera's
// position on the wrist, measured with a chessboard on the closed lid (server/hand_eye_cal.py). Lite plans the
// views (rest, inspection pose 1 and inspection pose 4, each with wrist turns; every MoveJ checked offline against the
// room's walls, the commode, the board on its lid and the left arm), shows the tour in 3D, dry-runs it in Hardware
// sim, runs it in one go on the robot (no confirmation per move; Real asks once) and solves it with ALIX's solver.
// Nothing is applied.

import { get, post, say } from './api.js';
import { ctx, el, jobLine, robotCall, tell, watchJob } from './ctx.js';
import { home3d, Player } from './seq3d.js';
import { css } from './three_scene.js';
import { frame } from './vp.js';

const $ = (id) => document.getElementById(id);
const BUTTONS = ['liteCalPlan', 'liteCalShow', 'liteCalRun', 'liteCalRest', 'liteCalSolve'];
const MODE_WORDS = { visual: 'Visual sim', hwsim: 'Hardware sim', real: 'Real Machine Mode' };
let box = null;
let busy = false;

// The board as the plan expects it in 3D: its squares flat on the closed lid (base_link, z up), long side outward,
// and a ring as far from the lid's middle as the plan lets it lie (any turn inside it is seen whole).
function boardObject(b) {
  const THREE = window.THREE;
  const [sx, sy] = b.squares;
  const sq = b.square_mm / 1000;
  const px = 16;
  const canvas = Object.assign(document.createElement('canvas'), { width: sx * px, height: sy * px });
  const g = canvas.getContext('2d');
  g.fillStyle = css('--vp-board-light'); g.fillRect(0, 0, canvas.width, canvas.height); g.fillStyle = css('--vp-board-dark');
  for (let r = 0; r < sy; r++) for (let c = 0; c < sx; c++) if ((r + c) % 2 === 0) g.fillRect(c * px, r * px, px, px);
  const tex = new THREE.CanvasTexture(canvas);
  tex.magFilter = THREE.NearestFilter;
  const [cx, cy, cz] = b.centre_m;
  const plate = new THREE.Mesh(new THREE.PlaneGeometry(sx * sq, sy * sq), new THREE.MeshBasicMaterial({ map: tex, side: THREE.DoubleSide }));
  plate.position.set(cx, cy, cz + 0.003);
  const r = b.radius_m + b.tolerance_mm / 1000;
  const ring = new THREE.Mesh(new THREE.RingGeometry(r - 0.003, r + 0.003, 72),
    new THREE.MeshBasicMaterial({ color: new THREE.Color(css('--vp-board-ring')), side: THREE.DoubleSide }));
  ring.position.set(cx, cy, cz + 0.004);
  const group = new THREE.Group();
  group.add(plate, ring);
  return group;
}

// the tour in the window's own 3D, in the project's room with the commode's lid closed; nothing sent
const view3d = {
  player: null, three: null, total: 0, done: null, board: null,
  async stage(session) {                         // the robot at home, the commode, the board on its lid
    const host = $('liteCalHost');
    if (!this.player) {
      this.player = new Player({ onTime: (t) => this.show(t), onEnd: () => { const f = this.done; this.done = null; if (f) f(); } });
    }
    this.three = await home3d(host);
    if (this.board) this.three.viewer.remove(this.board);
    this.board = null;
    if (session) { this.board = boardObject(session.board); await this.three.viewer.add(this.board); }
    frame(host).title('The calibration tour', session ? 'the robot at home; the board on the lid' : 'plan the views to see them');
  },
  async play(timeline, says, session) {
    const host = $('liteCalHost');
    await this.stage(session);
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
            num('liteCalSqX', 'Squares along', 12, { min: '4', max: '20', step: '1' }),
            num('liteCalSqY', 'Squares across', 9, { min: '4', max: '20', step: '1' }),
            num('liteCalSquare', 'Measured square (mm)', '15.00', { min: '10', max: '60', step: '0.01' }),
            num('liteCalPlate', 'Plate (mm, blank: a print)', 200, { min: '100', max: '600', step: '1' })),
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
  const plate = $('liteCalPlate').value.trim();
  const r = await post('/api/calibration/plan', {
    pid: ctx.pid, squares: [Number($('liteCalSqX').value), Number($('liteCalSqY').value)], square_mm: Number($('liteCalSquare').value),
    plate_mm: plate === '' ? null : Number(plate) });
  says(`${r.session.views} views planned in ${r.plan.seconds} s; the board may lie up to ${r.plan.tolerance_mm} mm from the lid’s middle, `
    + 'turned any way. Nothing moved.');
  await view3d.stage(r.session);
}

async function show(note = '') {
  const s = (await get('/api/calibration')).session;
  if (!s) { window.toast('Plan the views first.', 'warn'); return; }
  says(`${note}The tour in 3D: nothing is sent to the robot.`);
  const tl = await post('/api/calibration/preview', {});
  await view3d.play(tl, tl.what, s);
  says(`${note}Shown: ${tl.what}. Nothing was sent.`);
}

async function run() {
  const d = await get('/api/calibration');
  const s = d.session;
  if (!s) { window.toast('Plan the views first.', 'warn'); return; }
  if (d.mode === 'visual') {                     // Visual sim: the run is the motion in 3D; no arm or camera is needed
    await show('Visual sim: ');
    return;
  }
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
    `The board: the lab’s ${d.board.name} plate (${d.board.squares[0]} x ${d.board.squares[1]} squares of ${d.board.square_mm} mm on a `
      + `${d.board.plate_mm} mm plate, as its footnote says), or another: enter its squares, measure ten squares with calipers and `
      + `divide by ten, and give its plate (blank for a print, such as ${d.board.print}: 10 x 7 squares of 30 mm on A3).`,
    `Lay it flat on the closed lid, near its middle (within ${tol}), turned any way; the seat down, the left arm at its rest.`,
    'Plan the views. Nothing moves.',
    'Check them offline: Show the tour in 3D (in Visual sim, Run the calibration shows it too; no arm or camera needed); '
      + 'in Hardware sim, Run the calibration sends the same MoveJs to the controllers in their SIM mode.',
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
  $('liteCalPlate').value = s.board.plate_mm == null ? '' : String(s.board.plate_mm);
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

export async function openCalibration() {
  if (!box) build();
  box.classList.add('show');
  document.body.style.overflow = 'hidden';
  await refresh();
  try { await view3d.stage((await get('/api/calibration')).session); } catch (err) { says(say(err)); }
}

// The Operator Journey's entry, below Data Management (the user, 4 Nov).
export function installCalibration() {
  if (document.querySelector('.lite-cal-nav')) return;
  const after = document.querySelector('.lite-data-nav') || document.querySelector('.navbtn[data-view="logs"]');
  if (!after) return;
  after.after(el('button', { class: 'navbtn lite-cal-nav', type: 'button', onclick: openCalibration },
    el('span', { class: 'navicon', text: '◎' }), el('span', { text: 'Camera Calibration' })));
}
