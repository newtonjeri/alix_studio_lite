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
import { createViewer, webglFail } from './three_scene.js';

const CYCLE_S = 0.005;
let viewer = null, robot = null, robotP = null;
const scenes = new Map();                       // fixture -> scene spec request

// What is loaded: unit, which file (`file`: what its timeline depends on), its timeline (phases,
// traj_t0, stage), its line -> row map; `blocked`: the server's refusal of the step's file.
export const sim = { key: null, which: null, file: null, arm: null, lines: 0, tl: null, progress: null, token: 0,
  staged: false, abort: null, stale: false, blocked: null, place: null };
let preRaf = 0, playRaf = 0;

async function ensure() {
  if (!viewer) viewer = createViewer();
  if (!robotP) {
    robotP = loadRobot().then(async (r) => { robot = r; await viewer.add(r.group); return r; });
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

// The project's room and a task's UGV placement as the scene shows them (server/placement.py), read
// once per project and task until Prepare changes them ('lite:placement').
const places = new Map();
export function placementOf(task = null) {
  if (!ctx.pid) return Promise.resolve(null);
  const k = `${ctx.pid}|${task || ''}`;
  if (!places.has(k)) {
    const p = get(`/api/projects/${ctx.pid}/placement${task ? `?task=${encodeURIComponent(task)}` : ''}`);
    p.catch(() => places.delete(k));
    places.set(k, p);
  }
  return places.get(k);
}
document.addEventListener('lite:placement', () => { places.clear(); sim.stale = true; });

// The fixture (the open project's, else commode C) in `phase`, inside the cubicle: the measured room
// with the commode where the UGV's placement puts it (`place`: a scene from the server; by default
// the room alone, at ALIX's calibrated standoff).
export async function scene(phase, fixture = ctx.fixture || 'commode_c', place = undefined) {
  if (!scenes.has(fixture)) {
    const p = get(`/api/fixtures/${fixture}/scene`);
    p.catch(() => scenes.delete(fixture));
    scenes.set(fixture, p);
  }
  const d = await scenes.get(fixture);
  if (place === undefined) place = (await placementOf(null).catch(() => null))?.scene || null;
  await viewer.setScene({ cubicleMm: d.cubicle_mm, fixtureSpec: d.spec, fixtureId: fixture, phase: phase || 'lid_closed', placement: place });
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
export async function showFile(host, unit, which, progress = null, task = TASKS[currentTask]?.label) {
  const token = ++sim.token;
  if (sim.abort) sim.abort();                             // nothing of the previous file keeps playing
  host.classList.add('lite-3d');
  host.dataset.lite3d = 'loading';
  await ensure();
  await viewer.attach(host);
  if (token !== sim.token) return false;
  const file = fileOf(unit, which) + `|${task || ''}`;      // a region in two tasks plays at each task's placement
  if (sim.key !== unit.key || sim.file !== file || sim.stale) {
    let r;
    try {
      r = await get(`${unitUrl(unit.key, 'timeline')}?which=${which}${task ? `&task=${encodeURIComponent(task)}` : ''}`);
    } catch (err) {
      if (token === sim.token) unload(Object.assign(err, { key: unit.key }));
      throw err;
    }
    if (token !== sim.token) return false;                 // a newer request won
    robot.home();
    sim.tl = robot.loadTimeline(unit.arm, r);
    Object.assign(sim, { key: unit.key, which, file, arm: unit.arm, lines: r.lines, stale: false, blocked: null, place: r.scene || null });
  }
  await scene(unit.phase, undefined, sim.place || null);   // where its recording was taught: the room, the UGV's placement
  if (token !== sim.token) return false;
  sim.progress = progress;
  rewind();
  host.dataset.lite3d = 'ready';
  say3d(host, null);
  syncScrub();
  const cap = host.querySelector('.simulation-caption');
  if (cap) {
    const what = which === 'raw' ? 'Recording before tuning' : which === 'version' ? 'Kept version' : 'Tuned trajectory';
    cap.querySelector('strong').textContent = `3D simulation · ${unit.arm.toUpperCase()} ARM · ${unit.display}`;
    cap.querySelector('span').textContent = `${what} · ${duration().toFixed(2)} s at 5 ms per line · staged first: ${sim.tl.stage.text}`;
  }
  return true;
}

// Nothing to play (no recording yet, or the tuning still running): the pane shows the scene, the
// robot standing in the region's toilet state, instead of the designer's picture. No file loads.
export async function showStanding(host, unit, note = 'No recording yet: the arm stands. Record the region, or use an existing or loaded file.') {
  const token = ++sim.token;                              // the scene changes: whoever set it before sets it again
  host.classList.add('lite-3d');
  host.dataset.lite3d = 'loading';
  await ensure();
  if (token !== sim.token) return false;
  await viewer.attach(host);
  const place = (await placementOf(TASKS[currentTask]?.label).catch(() => null))?.scene || null;
  await scene(unit.phase, undefined, place);
  if (token !== sim.token) return false;
  robot.home();
  viewer.dirty = true;
  host.dataset.lite3d = 'ready';
  say3d(host, null);
  syncScrub();
  const cap = host.querySelector('.simulation-caption');
  if (cap) {
    cap.querySelector('strong').textContent = `3D simulation · ${unit.arm.toUpperCase()} ARM · ${unit.display}`;
    cap.querySelector('span').textContent = note;
  }
  return true;
}

export function invalidate() { sim.stale = true; }

// A 3D host that shows nothing playable says why, in place of any picture (V19's placeholder is
// never shown): the server's refusal, or how to get the browser's 3D back. `text` null clears it.
export function say3d(host, text, err = null) {
  if (!host) return;
  let m = host.querySelector(':scope > .lite-3d-msg');
  if (!text) { m?.remove(); return; }
  if (!m) { m = Object.assign(document.createElement('div'), { className: 'lite-3d-msg' }); host.append(m); }
  const noGl = !!err && err.code === 'webgl';
  m.classList.remove('webgl-fail');
  if (noGl) webglFail(m);                                  // ALIX Studio's notice, with its fixes
  else m.textContent = text;
  const cap = host.querySelector('.simulation-caption');
  if (cap) {
    cap.querySelector('strong').textContent = '3D simulation';
    cap.querySelector('span').textContent = noGl ? '3-D view unavailable in this browser' : 'Nothing plays';
  }
}

// Whether the Trajectory Viewer, the full sequence or one of the page's pictures (pictures.js: Home,
// Setup & Safety, the arm cards in Prepare, Drag & Teach and Real Test) has the 3D: the region's
// simulation waits, and loads again when Tune shows it.
let pictureHolds = false;
export function pictureHas(on) { pictureHolds = !!on; }
export const elsewhere = () => pictureHolds || !!document.querySelector('#liteViewer.show, #fullSequenceSimModal.show, #liteTransitionSim.show');

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

// The trajectory scrubber in the simulation pane's controls: any moment of the trajectory, the
// arm and the plot's playhead with it.
function syncScrub(t = null) {
  const sc = document.getElementById('liteSimScrub');
  if (!sc) return;
  sc.max = String(Math.round(duration() * 1000));
  sc.disabled = !sim.tl;
  if (document.activeElement !== sc) sc.value = String(Math.round((t ?? 0) * 1000));
}

// The moment on the plots: Tune draws it (its plots zoom and pan, so the page's own playhead, drawn
// for its picture of a plot, no longer lines up). `fn(line, row)`: the file's line and the
// recording's row at that line (the same for the recording itself), both 1-based.
let playheadTo = null;
export function onPlayhead(fn) { playheadTo = fn; }

function placePlayhead(row, line = row) {
  if (playheadTo) playheadTo(line, row);
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
    placePlayhead(rowAt(line + 1), line + 1);
    syncScrub(Math.min(simElapsedSec, duration()));
  };

  const pageReset = window.resetSimulationPlayback;
  window.resetSimulationPlayback = function resetSimulationPlayback(showToast = true) {
    pageReset(showToast);
    rewind();
    if (sim.tl) placePlayhead(rowAt(1), 1);
    syncScrub(0);
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
    if (action === 'stop') syncScrub(Math.min(simElapsedSec, duration()));
    if (action === 'stop' && sim.tl && sim.staged) {
      const line = Math.round(Math.min(simElapsedSec, duration()) / CYCLE_S);
      showLine(line);
      placePlayhead(rowAt(line + 1), line + 1);
    }
  };

  // the scrubber: dragging it pauses the run and shows that moment of the trajectory
  const bar = document.querySelector('#simulationViewport .sim-controls-overlay');
  if (bar && !document.getElementById('liteSimScrub')) {
    const sc = Object.assign(document.createElement('input'), { type: 'range', id: 'liteSimScrub', className: 'lite-sim-scrub',
      min: '0', max: '0', step: '5', value: '0', disabled: true });
    sc.setAttribute('aria-label', 'Trajectory time');
    sc.title = 'Drag through the trajectory';
    sc.addEventListener('input', () => {
      if (!sim.tl) return;
      if (sim.abort) sim.abort();
      if (simRunning) pageControl('stop');
      const t = Math.min(duration(), Number(sc.value) / 1000);
      simElapsedSec = t;
      sim.staged = true;                                   // Start goes on from here, without the staging again
      const line = Math.round(t / CYCLE_S);
      showLine(line);
      placePlayhead(rowAt(line + 1), line + 1);
      const timer = document.getElementById('simRunTimer');
      if (timer) timer.textContent = fmt(t);
    });
    bar.insertBefore(sc, document.getElementById('simExpandBtn'));
  }
}
