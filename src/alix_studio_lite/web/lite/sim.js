// The simulation (R3): the robot in 3D in the page's simulation pane (and in Save & Review's
// playback preview), executing the region's file the way the robot will: the pole to the file's
// stage height, then the hand to its pose, then the arm to the first line, then the trajectory at
// 1x on the 5 ms clock (a file of N lines lasts (N - 1) x 5 ms). The server builds that timeline
// (staging.execution_timeline) and refuses it for a file of the other arm or without a stage, so
// such a file never plays. The page's timer and plot playhead run the trajectory; the staging
// plays first, as a pre-roll with its own status line.

import { get, say } from './api.js';
import { ctx, step, unitUrl } from './ctx.js';
import { loadRobot } from './robot3d.js';
import { createViewer } from './three_scene.js';

const CYCLE_S = 0.005;
let viewer = null, robot = null, robotP = null;
const scenes = new Map();                       // fixture -> scene spec request

// What is loaded: unit, which file (`file`: what its timeline depends on), its timeline (phases,
// traj_t0, stage), its line -> row map; `blocked`: the server's refusal of the step's file.
export const sim = { key: null, which: null, file: null, arm: null, lines: 0, tl: null, progress: null, token: 0,
  staged: false, abort: null, stale: false, blocked: null };
let preRaf = 0, playRaf = 0;

async function ensure() {
  if (!viewer) viewer = createViewer();
  if (!robotP) {
    robotP = loadRobot().then((r) => { robot = r; viewer.add(r.group); return r; });
    robotP.catch(() => { robotP = null; });          // tried again next time (after a new log-in, say)
  }
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
  if (!scenes.has(fixture)) {
    const p = get(`/api/fixtures/${fixture}/scene`);
    p.catch(() => scenes.delete(fixture));
    scenes.set(fixture, p);
  }
  const d = await scenes.get(fixture);
  await viewer.setScene({ cubicleMm: d.cubicle_mm, fixtureSpec: d.spec, fixtureId: fixture, phase: phase || 'lid_closed' });
}

// What a step's timeline depends on: the file, its arm and stated stage, and its tuning or version.
function fileOf(unit, which) {
  const rec = (unit.recordings || []).find(r => r.file === unit.active) || {};
  const ver = (unit.versions || [])[(unit.versions || []).length - 1] || {};
  return JSON.stringify([which, unit.active, rec.taught_arm, rec.stage && rec.stage.text,
    which === 'candidate' ? unit.candidate_extra && unit.candidate_extra.made : null,
    which === 'version' ? [ver.version, ver.sha256] : null]);
}

// Forgets the loaded file: nothing plays until the next one loads. `why`: the refusal to show.
export function unload(why = null) {
  sim.token++;
  if (sim.abort) sim.abort();
  Object.assign(sim, { key: null, which: null, file: null, tl: null, lines: 0, staged: false, blocked: why });
  if (robot) { robot.home(); viewer.dirty = true; }
}

// Loads unit `key`'s execution timeline (`which`: raw | candidate | version) into `host`, at its
// start: the robot standing, before the pole moves. Resolves true when shown, false when a newer
// request replaced it; throws the server's refusal (wrong arm, no stage), and then nothing plays.
export async function showFile(host, unit, which, progress = null) {
  const token = ++sim.token;
  if (sim.abort) sim.abort();                             // nothing of the previous file keeps playing
  host.classList.add('lite-3d');
  host.dataset.lite3d = 'loading';
  await ensure();
  await viewer.attach(host);
  await scene(unit.phase);
  if (token !== sim.token) return false;
  const file = fileOf(unit, which);
  if (sim.key !== unit.key || sim.file !== file || sim.stale) {
    let r;
    try {
      r = await get(`${unitUrl(unit.key, 'timeline')}?which=${which}`);
    } catch (err) {
      if (token === sim.token) unload(Object.assign(err, { key: unit.key }));
      throw err;
    }
    if (token !== sim.token) return false;                 // a newer request won
    robot.home();
    sim.tl = robot.loadTimeline(unit.arm, r);
    Object.assign(sim, { key: unit.key, which, file, arm: unit.arm, lines: r.lines, stale: false, blocked: null });
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
  return true;
}

export function invalidate() { sim.stale = true; }

// Whether the Trajectory Viewer or the full sequence has the 3D: the region's simulation waits.
export const elsewhere = () => !!document.querySelector('#liteViewer.show, #fullSequenceSimModal.show');

// The stage is left, or another view takes the 3D (`stale`: it replaces the robot's tracks, so the
// file loads again when it is shown next): nothing of the region's file keeps playing.
export function release(stale = true) {
  if (sim.abort) sim.abort();
  if (simRunning) window.resetSimulationPlayback(false);
  if (stale) sim.stale = true;
}

// The robot standing, before anything moves (the start of every execution).
export function rewind() {
  if (sim.abort) sim.abort();
  sim.staged = false;
  if (robot && sim.tl) { robot.showTime(0, sim.arm); viewer.dirty = true; }
}

// Line i (0-based) of the trajectory, after the staging.
export function showLine(i) {
  if (!robot || !sim.tl) return;
  robot.showTime(sim.tl.traj_t0 + Math.max(0, Math.min(sim.lines - 1, i)) * CYCLE_S, sim.arm);
  viewer.dirty = true;
}

export const duration = () => (sim.lines > 1 ? (sim.lines - 1) * CYCLE_S : 0);

// Plays the staging (pole, hand, move to the start) at 1x; resolves true when it reached the
// trajectory's first line, false when stopped. `sayLine(text)` gets each phase's line.
export function preRoll(sayLine) {
  return new Promise((resolve) => {
    if (!sim.tl) { resolve(false); return; }
    if (sim.abort) sim.abort();                           // one staging at a time: the earlier one resolves false
    const t0 = performance.now(), end = sim.tl.traj_t0;
    let shown = '';
    const finish = (ok) => { cancelAnimationFrame(preRaf); sim.abort = null; resolve(ok); };
    const tick = () => {
      const t = Math.min(end, (performance.now() - t0) / 1000);
      robot.showTime(t, sim.arm);
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

// Plays the loaded file once at 1x, as the robot executes it: the staging, then the trajectory on
// its own clock ((N - 1) x 5 ms). Resolves true at its end; false when stopped or replaced.
export async function playOnce(sayLine) {
  const token = sim.token;
  if (!(await preRoll(sayLine)) || token !== sim.token) return false;
  return new Promise((resolve) => {
    const t0 = performance.now(), total = duration();
    const finish = (ok) => { cancelAnimationFrame(playRaf); sim.abort = null; resolve(ok); };
    const tick = () => {
      if (token !== sim.token) { finish(false); return; }
      const t = Math.min(total, (performance.now() - t0) / 1000);
      showLine(Math.round(t / CYCLE_S));
      if (t >= total) { finish(true); return; }
      playRaf = requestAnimationFrame(tick);
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
    const warn = (text) => { if (status) { status.textContent = text; status.className = 'statusbar warn'; } };
    if (action === 'start') {
      // only the step on screen's own file plays (never the page's stand-in timer, nor another step's file)
      const s = ctx.pid ? step() : null;
      if (!sim.tl || !s || sim.key !== s.key) {
        const why = sim.blocked && s && sim.blocked.key === s.key ? `Nothing plays: ${say(sim.blocked)}`
          : !s ? 'Open a project and choose a region first.' : 'The simulation is still loading: press ▶ Start again in a moment.';
        warn(why);
        window.toast(why);
        return;
      }
      if (sim.abort) return;                              // already staging
      if (sim.staged && simElapsedSec >= duration()) rewind();   // a finished run starts again from the staging
    }
    if (action === 'start' && sim.tl && !sim.staged && !simRunning) {
      // the pole and the hand first, then the move to the start: the trajectory never plays before them
      const badge = document.getElementById('simStateBadge');
      if (badge) { badge.textContent = 'STAGING'; badge.className = 'badge orange'; }
      const ok = await preRoll(warn);
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
