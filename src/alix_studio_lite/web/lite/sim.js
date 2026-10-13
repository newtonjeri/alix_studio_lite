// The simulation (R3): the robot in 3D in the page's simulation pane (and in Save & Review's
// playback preview), playing the region's tuned file, or its recording before tuning, at 1x on the
// 5 ms clock: a file of N lines lasts (N - 1) x 5 ms. The page's own timer drives it; here each
// frame also poses the robot and puts the plot's playhead on the recording's row for that moment.

import { get } from './api.js';
import { ctx, unitUrl } from './ctx.js';
import { loadRobot } from './robot3d.js';
import { createViewer } from './three_scene.js';

const CYCLE_S = 0.005;
let viewer = null, robot = null, robotP = null;
const scenes = new Map();                       // fixture -> scene spec request

// What is loaded: unit, which file, its line count, and its line -> recording row map.
export const sim = { key: null, which: null, lines: 0, progress: null, token: 0 };

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

// Shows unit `key`'s file (`which`: raw | candidate | version) in `host`, at its first line.
export async function showFile(host, unit, which, progress = null) {
  const token = ++sim.token;
  host.classList.add('lite-3d');
  host.dataset.lite3d = 'loading';
  await ensure();
  await viewer.attach(host);
  await scene(unit.phase);
  if (sim.key !== unit.key || sim.which !== which || sim.stale) {
    const r = await get(`${unitUrl(unit.key, 'frames')}?which=${which}&every=4`);
    if (token !== sim.token) return;                       // a newer request won
    robot.home();
    const info = robot.loadFrames(unit.arm, r);
    Object.assign(sim, { key: unit.key, which, lines: info.lines, stale: false });
  }
  sim.progress = progress;
  showLine(0);
  host.dataset.lite3d = 'ready';
  const cap = host.querySelector('.simulation-caption');
  if (cap) {
    const what = which === 'raw' ? 'Recording before tuning' : which === 'version' ? 'Kept version' : 'Tuned trajectory';
    cap.querySelector('strong').textContent = `3D simulation · ${unit.arm.toUpperCase()} ARM · ${unit.display}`;
    cap.querySelector('span').textContent = `${what} · ${duration().toFixed(2)} s at 5 ms per line · poses from ALIX's kinematics`;
  }
}

export function invalidate() { sim.stale = true; }

export function showLine(i) {
  if (!robot || !sim.lines) return;
  robot.showTrackLine(Math.max(0, Math.min(sim.lines - 1, i)));
  viewer.dirty = true;
}

export const duration = () => (sim.lines > 1 ? (sim.lines - 1) * CYCLE_S : 0);

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
    if (!sim.lines) return;
    const line = Math.round(Math.min(simElapsedSec, duration()) / CYCLE_S);
    showLine(line);
    placePlayhead(rowAt(line + 1));
  };

  const pageReset = window.resetSimulationPlayback;
  window.resetSimulationPlayback = function resetSimulationPlayback(showToast = true) {
    pageReset(showToast);
    showLine(0);
    if (sim.lines) placePlayhead(rowAt(1));
  };

  const pageControl = window.simulationControl;
  window.simulationControl = function simulationControl(action) {
    pageControl(action);
    if (action === 'stop' && sim.lines) {
      const line = Math.round(Math.min(simElapsedSec, duration()) / CYCLE_S);
      showLine(line);
      placePlayhead(rowAt(line + 1));
    }
  };
}
