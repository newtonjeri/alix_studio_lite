// The simulation (R3): the robot in 3D in the page's simulation pane (and in Save & Review's
// playback preview), executing the region's file the way the robot will: the pole to the file's
// stage height, then the hand to its pose, then the arm to the first line, then the trajectory at
// 1x on the 5 ms clock (a file of N lines lasts (N - 1) x 5 ms). The server builds that timeline
// (staging.execution_timeline) and refuses it for a file of the other arm or without a stage, so
// such a file never plays. The page's timer and plot playhead run the trajectory; the staging
// plays first, as a pre-roll with its own status line.

import { get } from './api.js';
import { ctx, unitUrl } from './ctx.js';
import { loadRobot } from './robot3d.js';
import { createViewer } from './three_scene.js';

const CYCLE_S = 0.005;
let viewer = null, robot = null, robotP = null;
const scenes = new Map();                       // fixture -> scene spec request

// What is loaded: unit, which file, its timeline (phases, traj_t0, stage), its line -> row map.
export const sim = { key: null, which: null, lines: 0, tl: null, progress: null, token: 0, staged: false, abort: null };
let preRaf = 0;

async function ensure() {
  if (!viewer) viewer = createViewer();
  if (!robotP) robotP = loadRobot().then((r) => { robot = r; viewer.add(r.group); return r; });
  await viewer.ready;
  return robotP;
}

// The one 3D view and robot of the page, shared with the Trajectory Viewer (viewer.js).
export async function ensure3D() {
  await ensure();
  return { viewer, robot };
}

// The fixture (the open project's, else commode C) in `phase`, inside the cubicle.
export async function scene(phase, fixture = ctx.fixture || 'commode_c') {
  if (!scenes.has(fixture)) scenes.set(fixture, get(`/api/fixtures/${fixture}/scene`));
  const d = await scenes.get(fixture);
  await viewer.setScene({ cubicleMm: d.cubicle_mm, fixtureSpec: d.spec, fixtureId: fixture, phase: phase || 'lid_closed' });
}

// Loads unit `key`'s execution timeline (`which`: raw | candidate | version) into `host`, at its
// start: the robot standing, before the pole moves. Throws the server's refusal (wrong arm, no stage).
export async function showFile(host, unit, which, progress = null) {
  const token = ++sim.token;
  host.classList.add('lite-3d');
  host.dataset.lite3d = 'loading';
  await ensure();
  await viewer.attach(host);
  await scene(unit.phase);
  if (sim.key !== unit.key || sim.which !== which || sim.stale) {
    const r = await get(`${unitUrl(unit.key, 'timeline')}?which=${which}`);
    if (token !== sim.token) return;                       // a newer request won
    robot.home();
    sim.tl = robot.loadTimeline(unit.arm, r);
    Object.assign(sim, { key: unit.key, which, lines: r.lines, stale: false });
  }
  sim.progress = progress;
  rewind();
  host.dataset.lite3d = 'ready';
  const cap = host.querySelector('.simulation-caption');
  if (cap) {
    const what = which === 'raw' ? 'Recording before tuning' : which === 'version' ? 'Kept version' : 'Tuned trajectory';
    cap.querySelector('strong').textContent = `3D simulation · ${unit.arm.toUpperCase()} ARM · ${unit.display}`;
    cap.querySelector('span').textContent = `${what} · ${duration().toFixed(2)} s at 5 ms per line · staged first: ${sim.tl.stage.text}`;
  }
}

export function invalidate() { sim.stale = true; }

// The robot standing, before anything moves (the start of every execution).
export function rewind() {
  if (sim.abort) sim.abort();
  sim.staged = false;
  if (robot && sim.tl) { robot.showTime(0); viewer.dirty = true; }
}

// Line i (0-based) of the trajectory, after the staging.
export function showLine(i) {
  if (!robot || !sim.tl) return;
  robot.showTime(sim.tl.traj_t0 + Math.max(0, Math.min(sim.lines - 1, i)) * CYCLE_S);
  viewer.dirty = true;
}

export const duration = () => (sim.lines > 1 ? (sim.lines - 1) * CYCLE_S : 0);

// Plays the staging (pole, hand, move to the start) at 1x; resolves true when it reached the
// trajectory's first line, false when stopped. `sayLine(text)` gets each phase's line.
export function preRoll(sayLine) {
  return new Promise((resolve) => {
    if (!sim.tl) { resolve(false); return; }
    cancelAnimationFrame(preRaf);
    const t0 = performance.now(), end = sim.tl.traj_t0;
    let shown = '';
    const finish = (ok) => { cancelAnimationFrame(preRaf); sim.abort = null; resolve(ok); };
    const tick = () => {
      const t = Math.min(end, (performance.now() - t0) / 1000);
      robot.showTime(t);
      viewer.dirty = true;
      const ph = sim.tl.phases.find(p => t >= p.t0 && t < p.t1) || sim.tl.phases[sim.tl.phases.length - 1];
      if (ph && ph.text !== shown) { shown = ph.text; sayLine(`Staging · ${ph.text}`); }
      if (t >= end) { sim.staged = true; finish(true); return; }
      preRaf = requestAnimationFrame(tick);
    };
    sim.abort = () => finish(false);
    tick();
  });
}

// The recording's row (1-based) at the file's line (1-based), through the tuning's map.
export function rowAt(line) {
  const p = sim.progress;
  if (sim.which === 'raw' || !p || !p.length) return line;
  let lo = 0, hi = p.length - 1;
  if (line <= p[0][0]) return p[0][1];
  if (line >= p[hi][0]) return p[hi][1];
  while (hi - lo > 1) { const mid = (lo + hi) >> 1; if (p[mid][0] <= line) lo = mid; else hi = mid; }
  const [a0, b0] = p[lo], [a1, b1] = p[hi];
  return Math.round(b0 + (b1 - b0) * (line - a0) / Math.max(1, a1 - a0));
}

function placePlayhead(row) {
  const ph = document.getElementById('simPlotPlayhead');
  if (!ph) return;
  const x = 800 * (row - 1) / Math.max(1, activeTrajectoryRows - 1);
  ph.setAttribute('x1', String(x));
  ph.setAttribute('x2', String(x));
}

export function installSim() {
  const pageDuration = window.simulationDurationSec;
  window.simulationDurationSec = function simulationDurationSec() {
    return duration() || pageDuration();
  };

  const pageFrame = window.updateSimulationPlaybackFrame;
  window.updateSimulationPlaybackFrame = function updateSimulationPlaybackFrame() {
    pageFrame();
    if (!sim.tl) return;
    const line = Math.round(Math.min(simElapsedSec, duration()) / CYCLE_S);
    showLine(line);
    placePlayhead(rowAt(line + 1));
  };

  const pageReset = window.resetSimulationPlayback;
  window.resetSimulationPlayback = function resetSimulationPlayback(showToast = true) {
    pageReset(showToast);
    rewind();
    if (sim.tl) placePlayhead(rowAt(1));
  };

  const pageControl = window.simulationControl;
  window.simulationControl = async function simulationControl(action) {
    const status = document.getElementById('simStatus');
    if (action === 'start' && sim.tl && !sim.staged && !simRunning) {
      // the pole and the hand first, then the move to the start: the trajectory never plays before them
      const badge = document.getElementById('simStateBadge');
      if (badge) { badge.textContent = 'STAGING'; badge.className = 'badge orange'; }
      const ok = await preRoll((text) => { if (status) { status.textContent = text; status.className = 'statusbar warn'; } });
      if (!ok) return;
    }
    if (action !== 'start' && sim.abort) sim.abort();
    pageControl(action);
    if (action === 'stop' && sim.tl && sim.staged) {
      const line = Math.round(Math.min(simElapsedSec, duration()) / CYCLE_S);
      showLine(line);
      placePlayhead(rowAt(line + 1));
    }
  };
}
