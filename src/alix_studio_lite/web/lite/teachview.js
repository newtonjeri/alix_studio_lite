// Drag & Teach's view of an uploaded file (the owner, 21 Oct: "For an uploaded project (as opposed to a taught
// one), show a viewport that visualizes the trajectory, so the user can check that it is the correct one. It
// should be like the other trajectory-visualizing viewports already in the app"). While the region's recording in
// use is a file loaded from this computer (Drag & Teach's Upload existing project, or Save & Review's Browse:
// teach.js), Drag & Teach shows it in 3D as the robot runs it: staged first (its pole and hand), in the project's
// room with the commode where its task's UGV placement puts it. The same viewport as every other (vp.js: the
// title and the time in the view, maximize, the view cube), the same tools under it (view3d.js: the commode's
// state, Check collisions, Prev / Next collision, the links in collision red) and a player. A taught take shows
// nothing here: Save & Review plays every recording.

import { post } from './api.js';
import { ctx, el, fmtS, loadUnit, step, unitUrl } from './ctx.js';
import { duration, elsewhere, invalidate, onMoment, say3d, scene, seekTo, showFile, sim } from './sim.js';
import { SOURCE } from './tasks.js';
import { useRecording } from './teach.js';
import { checkJob, spansOf, Tools } from './view3d.js';
import { frame } from './vp.js';

const $ = (id) => document.getElementById(id);
let unit = null, rec = null, token = 0, tools = null, vp = null;
const pb = { playing: false, t: 0, start: 0, raf: 0 };

const fmt = (s) => `${Number(s).toFixed(3)} s`;
const task = () => TASKS[currentTask]?.label || null;
const total = () => duration();

function card() {
  let c = $('liteTeach3d');
  if (c) return c;
  const stage = $('stage-teach');
  if (!stage) return null;
  const btn = (text, onclick, cls = 'btn light') => el('button', { class: cls, type: 'button', text, onclick });
  c = el('div', { id: 'liteTeach3d', class: 'lite-rev3d lite-teach3d', hidden: true },
    el('div', { class: 'lite-section-title' },
      el('h3', { text: 'Uploaded file' }),
      el('span', { class: 'mini-note', text: 'Check that it is the right one: played as the robot runs it, its pole and hand staged first, '
        + 'in the room and UGV placement of this task.' })),
    el('div', { class: 'lite-rev3d-grid' },
      el('div', { class: 'lite-rev3d-main' }, el('div', { id: 'liteTeach3dHost', class: 'lite-rev3d-host' })),
      el('div', { class: 'lite-rev3d-side' },
        el('div', { class: 'lite-rev3d-player' },
          el('div', { class: 'lite-row' }, btn('Play', play, 'btn'), btn('Pause', pause, 'btn outline'), btn('Restart', () => { pause(); setT(0); })),
          el('div', { class: 'lite-rev3d-track' },
            el('input', { id: 'liteTeach3dTimeline', type: 'range', min: '0', max: '0', step: '5', value: '0', 'aria-label': 'Uploaded file time',
              oninput: (e) => { pause(); setT(Number(e.target.value) / 1000); } }))),
        el('div', { class: 'lite-rev3d-block' },
          el('div', { class: 'lite-rev3d-head' }, el('h4', { text: 'The file' })),
          el('div', { id: 'liteTeach3dFile', class: 'mini-note' })),
        el('div', { class: 'lite-rev3d-block' },
          el('div', { class: 'lite-rev3d-head' }, el('h4', { text: 'Collision check' })),
          el('div', { id: 'liteTeach3dCheck', class: 'mini-note', text: 'Check collisions under the view checks every line of the file.' })),
        el('div', { class: 'lite-rev3d-block' },
          el('p', { class: 'mini-note', text: 'The right one: use it and go on to Save & Review. Not the right one: upload another, or teach '
            + 'the region with START.' }),
          btn('Use this recording → Save & Review', useRecording, 'btn green')))));
  ($('timerResult') || stage.lastElementChild).after(c);
  tools = new Tools({
    what: 'the file',
    check: async (phase, progress) => {
      const r = await checkJob(post, unitUrl(unit.key, 'collisions'), { which: 'raw', file: rec.file, task: task(), phase }, progress);
      return { result: r, spans: spansOf(r) };
    },
    seek: (t) => { pause(); setT(t); },
    changed: (phase) => { if (unit) scene(phase || unit.phase, undefined, sim.place || null).catch(() => {}); },
    shown: paintCheck,
  }).mount($('liteTeach3dHost'));
  tools.track(c.querySelector('.lite-rev3d-track'));
  vp = frame($('liteTeach3dHost'));
  return c;
}

// The region's recording in use, when it is a file loaded from this computer.
async function show() {
  const s = step();
  const mine = ++token;
  pause();
  if (!s || !ctx.pid || s.later || currentStage !== 'teach') { hide(); return; }
  let u;
  try { u = await loadUnit(s.key); } catch (_) { return; }
  if (mine !== token) return;
  const r = (u.recordings || []).find(x => x.file === u.active);
  if (!r || r.source !== 'upload') { hide(); return; }
  const c = card();
  if (!c) return;
  c.hidden = false;
  unit = u; rec = r;
  vp.title(r.name, 'The uploaded file, staged first');
  $('liteTeach3dFile').textContent = `${r.name} · ${SOURCE[r.source] || r.source} · ${Number(r.lines).toLocaleString()} rows · ${fmtS(r.seconds)}`
    + (r.stage ? ` · ${r.taught_arm ? `${r.taught_arm.toUpperCase()} arm, ` : ''}staged first: ${r.stage.text}` : ' · its pole and hand are not known');
  tools.load(JSON.stringify([u.key, r.file, r.sha256, task()]), { phase: u.phase, text: "the region's" });
  const host = $('liteTeach3dHost');
  if (elsewhere()) { invalidate(); return; }
  if (!host.querySelector('canvas')) say3d(host, 'Loading the 3D view…');
  try {
    if (!(await showFile(host, u, 'raw', null, task(), tools.phase)) || mine !== token) return;
  } catch (err) {
    say3d(host, `Nothing plays: ${err.message || err}`, err);
    return;
  }
  onMoment((t) => tools.at(t == null ? -1 : t));
  const T = total();
  tools.setLength(T);
  $('liteTeach3dTimeline').max = String(Math.round(T * 1000));
  setT(0);
}

function hide() {
  const c = $('liteTeach3d');
  if (c) c.hidden = true;
  unit = null; rec = null;
}

function setT(t, seek = true) {
  pb.t = Math.max(0, Math.min(total(), Number(t) || 0));
  const tl = $('liteTeach3dTimeline');
  if (tl && document.activeElement !== tl) tl.value = String(Math.round(pb.t * 1000));
  if (vp) vp.time(`${fmt(pb.t)} / ${fmt(total())}`);
  if (seek && sim.tl) seekTo(pb.t);
}

function play() {
  if (!sim.tl || !total()) return;
  if (pb.t >= total() - 0.001) pb.t = 0;
  pb.playing = true;
  pb.start = performance.now() - pb.t * 1000;
  cancelAnimationFrame(pb.raf);
  const tick = (now) => {
    if (!pb.playing) return;
    setT((now - pb.start) / 1000);
    if (pb.t >= total()) { pause(); return; }
    pb.raf = requestAnimationFrame(tick);
  };
  pb.raf = requestAnimationFrame(tick);
}

function pause() { pb.playing = false; cancelAnimationFrame(pb.raf); pb.raf = 0; }

function paintCheck(r) {
  const note = $('liteTeach3dCheck');
  if (!note) return;
  if (!r) { note.textContent = 'Check collisions under the view checks every line of the file.'; return; }
  const c = r.contact || {};
  note.textContent = (r.ok ? 'No collision.' : `${r.colliding_lines.toLocaleString()} of ${r.lines.toLocaleString()} lines collide `
    + `(Prev / Next collision under the view goes to them).`)
    + (c.lines ? ` The hand touches the commode on ${c.lines.toLocaleString()} lines from line ${c.first_line.toLocaleString()} `
      + `(deepest ${c.deepest_mm} mm; up to ${r.checked.contact_depth_mm} mm is the pad's give).` : '');
}

export function installTeachView() {
  const start = $('startTeachBtn');                      // the countdown or a take: the file's 3D stops playing
  if (start) new MutationObserver(() => { if (start.disabled) pause(); }).observe(start, { attributes: true, attributeFilter: ['disabled'] });
  document.addEventListener('lite:stage', (e) => { if (e.detail === 'teach') show(); else { pause(); token += 1; } });
  document.addEventListener('lite:step', () => { if (currentStage === 'teach') show(); });
  document.addEventListener('lite:recordings', () => { if (currentStage === 'teach') show(); });
  document.addEventListener('lite:placement', () => { if (tools) tools.load(null, { phase: unit?.phase || 'lid_closed' }); });
  document.addEventListener('lite:viewer-closed', () => { if (currentStage === 'teach') show(); });
}
