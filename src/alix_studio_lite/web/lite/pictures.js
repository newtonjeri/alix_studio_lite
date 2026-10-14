// The page's pictures. The 3D scene is only where motion is shown (the owner, 14 Oct): Tune's
// simulation beside the plot (sim.js), the Trajectory Viewer, the full sequence, and here the
// single-arm merged simulation. Everywhere else the page shows its own region images from the
// slides (the arm cards, the zoom, Drag & Teach, Home, Setup & Safety, the Project Summary).
//
// The single-arm window borrows the page's one 3D view (three_scene.js) for that arm. Taking the
// scene stops the region's simulation and forgets its file, which loads again when Tune shows it:
// a file never plays from a picture's pose.

import { say } from './api.js';
import { ctx, el } from './ctx.js';
import { webglFail } from './three_scene.js';
import { ensure3D, pictureHas, release, scene, sim, unload } from './sim.js';

const $ = (id) => document.getElementById(id);

const pic = { owner: null, sig: '', simToken: -1, borrowSig: '', token: 0, timer: 0, viewer: null, robot: null, single: 'left' };

function frame(host) {
  if (!host || host.classList.contains('lite-pic')) return host;
  host.classList.add('lite-pic', 'lite-3d');
  host.append(el('div', { class: 'lite-pic-msg', text: 'The 3D view starts after the log-in.' }));
  return host;
}
const singleHost = () => frame(document.querySelector('#singleArmMergeSimModal .single-arm-sim-viewport'));

const projectOpen = () => !!(currentProjectId && ctx.pid);
const fixtureId = () => (projectOpen() ? ctx.fixture || 'commode_c' : 'commode_c');
const singleOpen = () => !!$('singleArmMergeSimModal')?.classList.contains('show');

function schedule() {
  pictureHas(singleOpen());
  clearTimeout(pic.timer);
  pic.timer = setTimeout(refresh, 30);
}

async function refresh() {
  const my = ++pic.token;
  pictureHas(singleOpen());
  if (!singleOpen()) return;
  const host = singleHost();
  const m = host && host.querySelector(':scope > .lite-pic-msg');
  try {
    if (m && host.dataset.lite3d !== 'ready') { m.classList.remove('webgl-fail'); m.textContent = 'Loading the 3D view…'; }
    const { viewer, robot } = await ensure3D();
    Object.assign(pic, { viewer, robot });
    if (my !== pic.token) return;
    await showSingle(my);
  } catch (err) {
    if (my !== pic.token || !host) return;
    host.dataset.lite3d = 'failed';
    if (m && err && err.code === 'webgl') webglFail(m);           // ALIX Studio's notice, with its fixes
    else if (m) m.textContent = `The 3D view could not start: ${say(err)}`;
  }
}

// The single-arm window sets the scene: the region's simulation stops and forgets its file (Tune
// loads it again when it shows it).
function take(owner) {
  if (pic.owner !== owner || sim.token !== pic.simToken) {
    release(true);
    unload();
    pic.sig = '';
    pic.tuneOwed = true;                       // Tune shows its file again when it is next on screen
  }
  pic.owner = owner;
  pic.simToken = sim.token;
}

const SIDE_DIR = { left: [-0.25, 0.7, 0.85], right: [-0.25, -0.7, 0.85] };

async function showSingle(my) {
  const host = singleHost(), fx = fixtureId(), side = pic.single;
  take('single');
  if (pic.sig !== `single:${fx}`) {
    pic.robot.home();
    await scene('lid_closed', fx);
    if (my !== pic.token) return;
    pic.sig = `single:${fx}`;
  }
  // the arm and the fixture in front of it, a little behind and above that arm's side
  const at = (n) => { const mm = pic.robot.linkWorldMatrix(n); return mm ? [mm.elements[12], mm.elements[13], mm.elements[14]] : null; };
  const fb = pic.viewer.fixtureBox();
  const fixture = fb ? [0, 1, 2].map(k => (fb.min.getComponent(k) + fb.max.getComponent(k)) / 2) : [0.74, 0, 0.22];
  const pts = [at(side === 'left' ? 'L_Link1' : 'R_Link1'), at(side === 'left' ? 'L_Link7' : 'R_Link7'), fixture].filter(Boolean);
  const lo = [0, 1, 2].map(k => Math.min(...pts.map(p => p[k]))), hi = [0, 1, 2].map(k => Math.max(...pts.map(p => p[k])));
  const look = { target: lo.map((l, k) => (l + hi[k]) / 2), dir: SIDE_DIR[side],
    radius: 1.3 * Math.max(0.35, 0.5 * Math.hypot(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2])) };
  const key = `single:${fx}:${side}`;
  if (!(pic.viewer.borrowedBy === host && pic.borrowSig === key)) {
    await pic.viewer.borrow(host, look, { zoom: true });                   // a large window: the wheel zooms
    pic.borrowSig = key;
  }
  if (my !== pic.token) return;
  host.dataset.lite3d = 'ready';
}

export function installPictures() {
  singleHost();

  // The single-arm merged simulation: the 3D scene of that arm. Merging files is still a template,
  // so nothing plays: the window says so, and ▶ Start does not pretend a run.
  const pageRunMerged = window.runMergedArmSimulation;
  window.runMergedArmSimulation = function runMergedArmSimulation(side) {
    pageRunMerged(side);
    if (!singleOpen()) return;                   // the page refused: no merge chosen
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

  // Back on Tune after the single-arm window had the scene: Tune loads its file again (tune.js), as
  // after the Trajectory Viewer. Entering Tune by its stage (and opening a project) loads it anyway.
  const pageShowView = window.showView;
  window.showView = function showView(v, btn) {
    pageShowView(v, btn);
    schedule();
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
  schedule();
}
