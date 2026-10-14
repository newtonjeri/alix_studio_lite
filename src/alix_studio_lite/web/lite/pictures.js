// The page's pictures in 3D. V19 shows still images in five places; each now shows the page's one
// 3D scene (the robot, the cubicle and the project's fixture), live, turned by dragging:
//
//   Home               the robot standing at the open project's fixture (commode C when no project
//                      is open), lid closed
//   Setup & Safety     the fixture in its three states (lid closed, lid open, seat open), one every
//                      5 s, with the page's label, counter and dots in step
//   the arm cards      the task's toilet state, each arm at the start of its region's active recording
//                      (the pole and the hand staged, the arm at the first line) and the hand's path in
//                      that recording as a thin line; an arm without a recording stands. Each card
//                      looks at its own arm; 🔍 shows the same view large.
//   Drag & Teach       the same scene, looking at the arm step on screen
//   single-arm merge   the scene for that arm; playing the merged files comes with the merge
//
// One robot and one scene (three_scene.js): a picture borrows the page's 3D view, and the two cards
// each get a view of their own of the same scene (a renderer with its own camera, never a second
// robot). A picture takes the scene only while it is on screen and neither the Trajectory Viewer nor
// the full sequence has it; in Save & Review and Tune the cards show what the 3D shows there, and
// those stages take the 3D as before (sim.js). Taking the scene stops the region's simulation and
// forgets its file, which loads again when Tune shows it: a file never plays from a picture's pose.
// The server refuses a recording of the other arm or without its pole and hand: that arm stands.

import { get, say, session } from './api.js';
import { ctx, el, loadUnit, step, unitUrl } from './ctx.js';
import { PATH_LAYERS, css } from './three_scene.js';
import { ensure3D, pictureHas, release, scene, sim, unload } from './sim.js';

const $ = (id) => document.getElementById(id);
const ARMS = ['left', 'right'];
const STATES = [['lid_closed', 'LID COVER CLOSED'], ['lid_open', 'LID COVER OPEN'], ['seat_open', 'SEAT COVER OPEN']];
const CARD_STAGES = new Set(['prepare', 'teach', 'real']);     // Save & Review and Tune have the 3D themselves
// Cameras (from the target towards the camera, base frame: x forward, y left, z up): each arm card
// from its own arm's side, a little behind and above, so the arm reaches into the region; Home from
// the robot's right and in front, the robot and the fixture side by side; Setup from the right, above.
const SIDE_DIR = { left: [-0.25, 0.7, 0.85], right: [-0.25, -0.7, 0.85] };
const CELL_DIR = [0.55, -1, 0.5], FIXTURE_DIR = [-0.2, -0.8, 0.9];

const pic = {
  owner: null,              // the picture that set the scene last: hero | carousel | cards | single
  sig: '',                  // what it set ('' = set it again)
  simToken: -1,             // sim.token then: a newer one means the simulation showed a file since
  borrowSig: '',            // which picture has the page's view, aimed how
  token: 0, timer: 0,       // the newest refresh
  viewer: null, robot: null, home: null,
  views: null, viewsP: null,                 // the cards' own views {left, right}
  paths: { left: null, right: null }, pathIds: { left: '', right: '' },
  zoomed: null,             // the arm whose card view is in the 🔍 window
  single: 'left',
};
const starts = new Map();   // a recording's start and hand path, by its file and stage (never the whole timeline)

// ── where each picture is

function frame(host, alt = '') {
  if (!host || host.classList.contains('lite-pic')) return host;
  host.classList.add('lite-pic', 'lite-3d');
  if (!host.querySelector('button, input, select')) {     // the picture's own words, now for the 3D view
    host.setAttribute('role', 'img');
    host.setAttribute('aria-label', `${alt || host.querySelector(':scope > img')?.alt || 'Robot and toilet'} (3D view: drag to turn)`);
  }
  host.append(el('div', { class: 'lite-pic-msg', text: 'The 3D view starts after the log-in.' }),
    el('div', { class: 'lite-pic-chip', hidden: true }));
  return host;
}

function heroHost() {
  const art = document.querySelector('#view-home .hero-art');
  if (!art) return null;
  let h = art.querySelector('.lite-hero-3d');
  if (!h) { h = el('div', { class: 'lite-hero-3d' }); art.prepend(h); art.classList.add('lite-pic-frame'); }
  return frame(h, art.querySelector(':scope > img')?.alt);
}
const carouselHost = () => frame($('visualCarousel'));
const cardHost = (side) => frame(document.querySelector(`#${side === 'left' ? 'choiceLeft' : 'choiceRight'} .choice-region-thumb`));
const teachHost = () => frame(document.querySelector('#stage-teach .teach-reference-image-wrap'));
const zoomHost = () => frame(document.querySelector('#imageModal .image-zoom-stage'));
const singleHost = () => frame(document.querySelector('#singleArmMergeSimModal .single-arm-sim-viewport'));

function say3d(host, state, chip = '', title = '') {
  if (!host) return;
  host.dataset.lite3d = state;
  const c = host.querySelector(':scope > .lite-pic-chip');
  if (c) { c.textContent = chip; c.title = title || chip; c.hidden = !chip; }
  const m = host.querySelector(':scope > .lite-pic-msg');
  if (m && state === 'loading') m.textContent = 'Loading the 3D view…';
}

// ── what is on screen

const projectOpen = () => !!(currentProjectId && ctx.pid);
const fixtureId = () => (projectOpen() ? ctx.fixture || 'commode_c' : 'commode_c');
const fixtureName = () => (projectOpen() ? ctx.fixtureName || ctx.fixture : '') || 'Commode C';
const modalHas3D = () => !!document.querySelector('#liteViewer.show, #fullSequenceSimModal.show');

function desired() {
  if (!session.token || modalHas3D()) return null;
  if ($('singleArmMergeSimModal')?.classList.contains('show')) return 'single';
  const view = document.querySelector('.view.active')?.id;
  if (view === 'view-home') return 'hero';
  if (view === 'view-setup') return 'carousel';
  if (view === 'view-teach') return CARD_STAGES.has(currentStage) ? 'cards' : 'mirror';
  return null;
}

// Something changed on screen. Whether a picture holds the scene is known at once (the region's
// simulation must not load into a hidden pane meanwhile); the pictures follow a moment later.
function schedule() {
  pictureHas(['hero', 'carousel', 'cards', 'single'].includes(desired()));
  clearTimeout(pic.timer);
  pic.timer = setTimeout(refresh, 30);
}

async function refresh() {
  const my = ++pic.token;
  const want = desired();
  pictureHas(['hero', 'carousel', 'cards', 'single'].includes(want));
  if (!want) return;
  const hosts = want === 'hero' ? [heroHost()] : want === 'carousel' ? [carouselHost()] : want === 'single' ? [singleHost()]
    : [cardHost('left'), cardHost('right'), ...(want === 'cards' && currentStage === 'teach' ? [teachHost()] : [])];
  try {
    for (const h of hosts) if (h && h.dataset.lite3d !== 'ready') say3d(h, 'loading');
    const { viewer, robot } = await ensure3D();
    Object.assign(pic, { viewer, robot });
    if (my !== pic.token) return;
    if (want === 'hero') await showHero(my);
    else if (want === 'carousel') await showCarousel(my);
    else if (want === 'single') await showSingle(my);
    else await showCards(my, want === 'cards');
  } catch (err) {
    if (my !== pic.token) return;
    for (const h of hosts) {
      say3d(h, 'failed', 'The 3D view could not start.', say(err));
      const m = h && h.querySelector(':scope > .lite-pic-msg');
      if (m) m.textContent = `The 3D view could not start: ${say(err)}`;
    }
  }
}

// A picture sets the scene: the region's simulation stops and forgets its file (Tune loads it again
// when it shows it), and Save & Review's preview goes back to its placeholder, as its file is gone.
function take(owner) {
  if (pic.owner !== owner || sim.token !== pic.simToken) {
    release(true);
    unload();
    document.querySelector('#stage-review .playback-preview')?.classList.remove('lite-3d');
    pic.sig = '';
    pic.tuneOwed = true;                       // Tune shows its file again when it is next on screen
  }
  pic.owner = owner;
  pic.simToken = sim.token;
}

const corners = (b) => [[b.min.x, b.min.y, b.min.z], [b.max.x, b.max.y, b.max.z]];

function remember() {                         // the robot standing: its box, each arm's shoulder and hand
  if (pic.home) return;
  const at = (n) => { const m = pic.robot.linkWorldMatrix(n); return m ? [m.elements[12], m.elements[13], m.elements[14]] : null; };
  pic.home = {
    box: corners(new window.THREE.Box3().setFromObject(pic.robot.group)),
    left: { hand: at('ILH_plam') || at('L_Link7'), shoulder: at('L_Link1') },
    right: { hand: at('IRH_plam') || at('R_Link7'), shoulder: at('R_Link1') },
  };
}

function boxOf(points) {
  const lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
  for (const p of points) for (let k = 0; k < 3; k++) { lo[k] = Math.min(lo[k], p[k]); hi[k] = Math.max(hi[k], p[k]); }
  return { lo, centre: lo.map((l, k) => (l + hi[k]) / 2), half: 0.5 * Math.hypot(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]) };
}

const fixtureBox = () => { const b = pic.viewer.fixtureBox(); return b ? corners(b) : [[0.47, -0.18, 0], [1.01, 0.18, 0.45]]; };

// Home: the robot and the fixture side by side.
function cellLook() {
  const b = boxOf([...((pic.home && pic.home.box) || [[-0.36, -0.23, 0], [0.54, 0.23, 1.34]]), ...fixtureBox()]);
  return { target: b.centre, dir: CELL_DIR, radius: b.half * 0.9 };
}

// Setup & Safety: the fixture, with room for the lid and the seat open.
function fixtureLook() {
  const b = boxOf(fixtureBox());
  return { target: [b.centre[0], b.centre[1], b.lo[2] + 0.34], dir: FIXTURE_DIR, radius: Math.max(0.58, b.half * 1.05) };
}

// An arm's card: the region its hand covers (the path, from the start); standing, the arm and the
// fixture in front of it.
function armLook(side, a) {
  if (a && a.path) {
    const pts = [];
    for (let i = 0; i < a.path.length; i += 3) pts.push([a.path[i], a.path[i + 1], a.path[i + 2]]);
    const b = boxOf(pts);
    return { target: b.centre, dir: SIDE_DIR[side], radius: Math.max(0.3, b.half * 1.08) };
  }
  const h = (pic.home && pic.home[side]) || {};
  const f = boxOf(fixtureBox());
  const b = boxOf([h.shoulder, h.hand, f.centre].filter(Boolean));
  return { target: b.centre, dir: SIDE_DIR[side], radius: Math.max(0.35, b.half * 1.05) };
}

async function borrowFor(host, key, look, opts) {
  if (pic.viewer.borrowedBy === host && pic.borrowSig === key) return;
  await pic.viewer.borrow(host, look, opts);
  pic.borrowSig = key;
}

// ── Home and Setup & Safety

async function showHero(my) {
  const host = heroHost(), fx = fixtureId();
  take('hero');
  if (pic.sig !== `hero:${fx}`) {
    pic.robot.home();
    await scene('lid_closed', fx);
    if (my !== pic.token) return;
    pic.sig = `hero:${fx}`;
    remember();
  }
  await borrowFor(host, `hero:${fx}`, cellLook());
  if (my !== pic.token) return;
  say3d(host, 'ready', `${fixtureName()} · lid closed · drag to turn`);
}

async function showCarousel(my) {
  const host = carouselHost(), fx = fixtureId();
  take('carousel');
  if (pic.sig !== `carousel:${fx}`) pic.robot.home();
  await scene(STATES[visualReferenceIndex % STATES.length][0], fx);
  if (my !== pic.token) return;
  pic.sig = `carousel:${fx}`;
  remember();
  await borrowFor(host, `carousel:${fx}`, fixtureLook());
  if (my !== pic.token) return;
  say3d(host, 'ready', 'Drag to turn');
}

// ── the arm cards, Drag & Teach, the zoom

// An arm step's active recording at its start, and the hand's path, from the execution the server
// builds for it (GET …/timeline?which=raw: the pole, the hand, the move to the first line, then the
// trajectory); a refusal (another arm's file, no pole and hand) is kept as the reason it stands.
async function startOf(s) {
  if (!projectOpen()) return { stand: 'Open a project to see its recordings.' };
  if (!s) return { stand: 'Not used in this task: the arm stands.' };
  if (s.later) return { stand: 'Not in Release 1: the arm stands.' };
  let u;
  try { u = await loadUnit(s.key); } catch (err) { return { stand: 'The arm stands.', why: say(err) }; }
  const rec = (u.recordings || []).find(r => r.file === u.active);
  if (!rec) return { stand: 'No recording yet: the arm stands.' };
  const id = JSON.stringify([ctx.pid, s.key, rec.file, rec.sha256, rec.taught_arm, rec.stage ? rec.stage.text : null]);
  if (!starts.has(id)) {
    const p = get(`${unitUrl(s.key, 'timeline')}?which=raw&coarse=1`).then((r) => reduce(r, rec, id), (err) => {
      if (err.code === 'offline' || err.code === 'login') starts.delete(id);
      return { id, stand: `${rec.name} cannot run on this arm: the arm stands.`, why: say(err) };
    });
    starts.set(id, p);
    if (starts.size > 60) starts.delete(starts.keys().next().value);
  }
  return starts.get(id);
}

function bytes(b64) {
  const s = atob(b64), out = new Uint8Array(s.length);
  for (let i = 0; i < s.length; i++) out[i] = s.charCodeAt(i);
  return out;
}
function base64(f32) {
  const u8 = new Uint8Array(f32.buffer, f32.byteOffset, f32.byteLength);
  let s = '';
  for (let i = 0; i < u8.length; i++) s += String.fromCharCode(u8[i]);
  return btoa(s);
}

// Keeps what a card needs: the frame at the trajectory's first line (pole and hand staged, the arm at
// line 1) as a one-frame timeline for robot3d.js, and the hand's path over the trajectory (the
// middle finger's pad, at the timeline's own step of a few lines).
function reduce(r, rec, id) {
  const L = r.links.length, n = r.times.length;
  const all = new Float32Array(bytes(r.frames).buffer);
  let i0 = r.times.findIndex(t => t >= r.traj_t0 - 1e-6);
  if (i0 < 0) i0 = n - 1;
  const pick = (...res) => { for (const re of res) { const k = r.links.findIndex(l => re.test(l)); if (k >= 0) return k; } return -1; };
  const k = Math.max(0, pick(/_middle_pad$/, /_plam$/, /_Link7$/));
  const path = new Float32Array((n - i0) * 3);
  for (let f = i0, j = 0; f < n; f++, j += 3) {
    const o = (f * L + k) * 16;
    path[j] = all[o + 3]; path[j + 1] = all[o + 7]; path[j + 2] = all[o + 11];
  }
  return {
    id, file: rec.name, stageText: r.stage && r.stage.text, path,
    start: { side: r.side, links: r.links, times: [0], frames: base64(all.slice(i0 * L * 16, (i0 + 1) * L * 16)),
      traj_t0: 0, total_s: 0, lines: r.lines, stage: r.stage, phases: [] },
  };
}

// Each region's hand path, a thin line over the fixture, on its arm's layer: each card sees its own,
// and the simulation's camera neither.
function setPaths(arms) {
  const THREE = window.THREE;
  for (const side of ARMS) {
    const a = arms[side], id = a && a.path ? a.id : '';
    if (pic.pathIds[side] === id) continue;
    const old = pic.paths[side];
    if (old) { pic.viewer.remove(old); old.geometry.dispose(); old.material.dispose(); }
    pic.paths[side] = null;
    pic.pathIds[side] = id;
    if (!id) continue;
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(a.path, 3));
    const line = new THREE.Line(g, new THREE.LineBasicMaterial({
      color: new THREE.Color(css(side === 'left' ? '--vp-path-left' : '--vp-path-right')), depthTest: false, transparent: true }));
    line.renderOrder = 10;                       // over the fixture and the arm: the region always shows
    line.layers.set(PATH_LAYERS[side]);
    pic.viewer.add(line);
    pic.paths[side] = line;
  }
}

function cardViews() {
  if (!pic.viewsP) {
    pic.viewsP = Promise.all(ARMS.map(side => pic.viewer.view({ paused: modalHas3D, paths: side })))
      .then(([left, right]) => { pic.views = { left, right }; return pic.views; });
    pic.viewsP.catch(() => { pic.viewsP = null; });
  }
  return pic.viewsP;
}

function chipFor(a, teach = false) {
  if (!a) return ['', ''];
  if (a.stand) return [a.stand, a.why || a.stand];
  const line = `${a.file}: pole and hand staged (${a.stageText}), the arm at its first line. The line is the hand's path in the recording.`;
  return [teach ? `Start of ${a.file} · the line is the hand's path` : `Start of ${a.file}`, line];
}

// `own`: Prepare, Drag & Teach and Real Test, where the cards set the scene. In Save & Review and
// Tune they show the scene as those stages set it (each card still looks at its own arm).
async function showCards(my, own) {
  const views = await cardViews();
  if (my !== pic.token) return;
  for (const side of ARMS) {
    const w = views[side];
    w.minGapMs = own || pic.zoomed === side ? 0 : 80;      // a small copy of a running simulation: ~12 pictures a second
    w.attach(pic.zoomed === side ? zoomHost() : cardHost(side));
  }
  if (own) take('cards');
  else pic.owner = null;                                   // Save & Review and Tune set the scene: the cards set it again after
  const fx = fixtureId();
  const task = JSON.stringify([ctx.pid, currentTask]);
  if (pic.task !== task) {                                 // another task: no word of the last one's recordings meanwhile
    pic.task = task;
    for (const side of ARMS) say3d(cardHost(side), 'ready');
  }
  const steps = { left: step(currentTask, 'left'), right: step(currentTask, 'right') };
  const phase = (steps[selectedArm] || {}).phase || (steps.left || {}).phase || (steps.right || {}).phase || 'lid_closed';
  const arms = {};
  await Promise.all(ARMS.map(async (side) => { arms[side] = await startOf(steps[side]); }));
  if (my !== pic.token) return;
  const sig = JSON.stringify(['cards', fx, phase, ARMS.map(s => (arms[s] && arms[s].path ? arms[s].id : ''))]);
  if (own && pic.sig !== sig) {
    await scene(phase, fx);
    if (my !== pic.token) return;
    pic.robot.home();
    remember();
    for (const side of ARMS) {
      const a = arms[side];
      if (a && a.start) { pic.robot.loadTimeline(side, a.start); pic.robot.showTime(0, side); }
    }
    pic.viewer.dirty = true;
    pic.sig = sig;
    pic.simToken = sim.token;
  }
  setPaths(arms);
  const t = TASKS[currentTask] || {};
  for (const side of ARMS) {
    const w = views[side], key = JSON.stringify([fx, arms[side] && arms[side].path ? arms[side].id : `stand:${side}`]);
    if (w.lookKey !== key) { w.aim(armLook(side, arms[side])); w.lookKey = key; }
    const [chip, title] = own ? chipFor(arms[side]) : ['', ''];
    say3d(cardHost(side), 'ready', chip, title);
    if (pic.zoomed === side) say3d(zoomHost(), 'ready', chip ? `${chip} · drag to turn, scroll to zoom` : 'Drag to turn, scroll to zoom', title);
  }
  if (own && currentStage === 'teach') {
    const host = teachHost(), a = arms[selectedArm];
    const key = JSON.stringify(['teach', fx, selectedArm, a && a.path ? a.id : 'stand']);
    await borrowFor(host, key, armLook(selectedArm, a), { paths: selectedArm });
    if (my !== pic.token) return;
    const [chip, title] = chipFor(a, true);
    say3d(host, 'ready', chip, title || `${selectedArm.toUpperCase()} ARM · ${selectedArm === 'left' ? t.left : t.right}`);
  }
}

// ── the single-arm merged simulation

async function showSingle(my) {
  const host = singleHost(), fx = fixtureId(), side = pic.single;
  take('single');
  if (pic.sig !== `single:${fx}`) {
    pic.robot.home();
    await scene('lid_closed', fx);
    if (my !== pic.token) return;
    pic.sig = `single:${fx}`;
    remember();
  }
  const look = armLook(side, null);
  await borrowFor(host, `single:${fx}:${side}`, { ...look, radius: look.radius * 1.3 }, { zoom: true });   // a large window: more of the cell
  if (my !== pic.token) return;
  say3d(host, 'ready');
}

// ── the page's functions that set these pictures

export function installPictures() {
  heroHost(); carouselHost(); cardHost('left'); cardHost('right'); teachHost(); zoomHost(); singleHost();

  // A click on a card's view still chooses its arm; a drag only turns the view.
  for (const side of ARMS) {
    const host = cardHost(side);
    let at = null;
    host?.addEventListener('pointerdown', (e) => { at = [e.clientX, e.clientY]; }, true);
    host?.addEventListener('click', (e) => { if (at && Math.hypot(e.clientX - at[0], e.clientY - at[1]) > 4) e.stopPropagation(); at = null; }, true);
  }

  // Setup & Safety: three states of the fixture in place of the two reference pictures.
  const head = document.querySelector('#view-setup .visual-reference-card .section-head p');
  if (head) head.textContent = 'The toilet in 3D: its three states rotate every 5 seconds. Drag to turn it.';
  const dots = document.querySelector('#view-setup .visual-carousel-dots');
  for (let i = dots ? dots.children.length : STATES.length; i < STATES.length; i++) {
    dots.append(el('button', { 'aria-label': `Show reference ${i + 1}`, onclick: () => window.showVisualReference(i) }));
  }
  window.showVisualReference = function showVisualReference(i) {
    visualReferenceIndex = (Number(i) + STATES.length) % STATES.length;       // the page's own index and timer
    const [phase, label] = STATES[visualReferenceIndex];
    if ($('visualRefLabel')) $('visualRefLabel').textContent = label;
    if ($('visualRefCounter')) $('visualRefCounter').textContent = `${visualReferenceIndex + 1} / ${STATES.length}`;
    document.querySelectorAll('.visual-carousel-dots button').forEach((b, j) => b.classList.toggle('active', j === visualReferenceIndex));
    window.resetVisualReferenceTimer();
    if (pic.owner === 'carousel' && desired() === 'carousel') scene(phase, fixtureId()).catch(() => {});
  };
  carouselHost()?.addEventListener('pointerdown', () => window.resetVisualReferenceTimer());   // turning it holds the state
  window.showVisualReference(0);

  // The arm cards and Drag & Teach: no picture is cut from the task's slide any more.
  window.setArmRegionImages = function setArmRegionImages() {
    window.syncTeachReference();                 // the page's other update: the Drag & Teach label
  };
  const pageSync = window.syncTeachReference;
  window.syncTeachReference = function syncTeachReference() {
    pageSync();                                  // the label "LEFT ARM · region" (the picture it copies is not shown)
    schedule();
  };

  // 🔍: the card's own view, large; it goes back to the card when the window closes. The page's
  // task-wide zoom (the whole slide) shows the arm on screen.
  const zoomIn = (side) => {
    pic.zoomed = side;
    if (pic.views) { const w = pic.views[side]; w.controls.enableZoom = true; w.minGapMs = 0; w.attach(zoomHost()); }
    say3d(zoomHost(), pic.views ? 'ready' : 'loading', 'Drag to turn, scroll to zoom');
    schedule();
  };
  const pageZoom = window.zoomArmReference;
  window.zoomArmReference = function zoomArmReference(side) {
    pageZoom(side);                              // the title and the window
    zoomIn(side);
  };
  const pageZoomTask = window.zoomReference;
  window.zoomReference = function zoomReference() {
    pageZoomTask();
    zoomIn(selectedArm);
  };
  const pageClose = window.closeImageModal;
  window.closeImageModal = function closeImageModal() {
    pageClose();
    const side = pic.zoomed;
    pic.zoomed = null;
    if (side && pic.views) { const w = pic.views[side]; w.controls.enableZoom = false; w.attach(cardHost(side)); }
    schedule();
  };

  // The single-arm merged simulation: the 3D scene of that arm. Merging files is still a template,
  // so nothing plays: the window says so, and ▶ Start does not pretend a run.
  const pageRunMerged = window.runMergedArmSimulation;
  window.runMergedArmSimulation = function runMergedArmSimulation(side) {
    pageRunMerged(side);
    if (!$('singleArmMergeSimModal')?.classList.contains('show')) return;    // the page refused: no merge chosen
    pic.single = side === 'right' ? 'right' : 'left';
    let note = $('liteSingleNote');
    if (!note) { note = el('p', { id: 'liteSingleNote', class: 'mini-note lite-single-note' }); $('singleArmSimFiles')?.after(note); }
    note.textContent = `The ${pic.single.toUpperCase()} arm in 3D: drag to turn, scroll to zoom. Playing these merged files comes with `
      + 'the merge: Lite does not join files yet, so nothing plays here. The full sequence plays each region\'s kept version.';
    schedule();
  };
  const pageSingle = window.singleArmMergeSimControl;
  window.singleArmMergeSimControl = function singleArmMergeSimControl(action) {
    if (action === 'start') {
      window.toast('The single-arm play comes with the merge: nothing plays yet.');
      return;
    }
    pageSingle(action);
    if (action === 'exit') schedule();
  };

  // Where the page changes what is on screen.
  const pageShowView = window.showView;
  window.showView = function showView(v, btn) {
    pageShowView(v, btn);
    schedule();
    // Back on Tune after a picture had the scene: Tune loads its file again (tune.js), as after the
    // Trajectory Viewer. Entering Tune by its stage (and a project being opened) loads it anyway.
    if (pic.tuneOwed && !projectRestoreInProgress && document.querySelector('.view.active')?.id === 'view-teach'
      && currentStage === 'optimize') {
      pic.tuneOwed = false;
      document.dispatchEvent(new CustomEvent('lite:viewer-closed'));
    }
  };
  const pageTheme = window.toggleTheme;
  window.toggleTheme = function toggleTheme() {
    pageTheme();
    pic.viewer?.retheme();
  };
  document.addEventListener('lite:viewer-closed', () => { pic.sig = ''; pic.borrowSig = ''; schedule(); });   // the 3D is back
  document.addEventListener('lite:stage', (e) => { if (e.detail === 'optimize') pic.tuneOwed = false; });   // Tune loads its file itself
  for (const name of ['lite:logged-in', 'lite:stage', 'lite:step']) document.addEventListener(name, schedule);
  schedule();
}
