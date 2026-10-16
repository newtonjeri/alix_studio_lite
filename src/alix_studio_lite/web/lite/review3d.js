// Save & Review's simulation and tools (feedback v5, the owner, 16 Oct):
// - the recording selected in the list above, played in 3D as the robot runs it (staged first: the
//   pole and the hand as it was taught), in the project's room with the commode where its task's UGV
//   placement puts it; a choice of view, and no plot (slide 5);
// - its collision check, on demand (the owner, 16 Oct: "a button, to do the collision check, on every
//   trajectory"): every 5 ms line against the walls measured in Prepare, the commode in the region's
//   lid and seat state (or the one chosen under the view) and the other arm, standing, with ALIX's
//   collision code (server/collision_check.py). The view's tools (view3d.js) mark the collisions on
//   the timeline and turn the links red; the first ones are listed here, one click away, so the
//   region can be taught again. It reports; it blocks nothing;
// - Trim and save (slide 1, as the controller's Track clip): the first and the last moment kept,
//   Save overwriting the recording (the owner's choice) or Save As a new one.

import { post, say } from './api.js';
import { confirmDialog, ctx, el, loadUnit, step, tell, unitUrl } from './ctx.js';
import { duration, elsewhere, invalidate, onMoment, say3d, scene, seekTo, showFile, sim } from './sim.js';
import { checkJob, spansOf, Tools } from './view3d.js';

const CYCLE_S = 0.005;
const $ = (id) => document.getElementById(id);
let unit = null, rec = null, token = 0, tools = null;
const pb = { playing: false, t: 0, start: 0, raf: 0 };
const trim = { a: 0, b: 0 };                 // the first and the last moment kept (s)

const fmt = (s) => `${Number(s).toFixed(3)} s`;
const task = () => TASKS[currentTask]?.label || null;
const total = () => duration();

// ── the card

function card() {
  let c = $('liteRev3d');
  if (c) return c;
  const stage = $('stage-review');
  if (!stage) return null;
  const btn = (text, onclick, cls = 'btn light') => el('button', { class: cls, type: 'button', text, onclick });
  c = el('div', { id: 'liteRev3d', class: 'lite-rev3d' },
    el('div', { class: 'lite-section-title' },
      el('h3', {}, 'Simulation · ', el('span', { id: 'liteRev3dName', text: '-' })),
      el('span', { class: 'mini-note', text: 'The recording selected above, staged first, in its room and UGV placement.' })),
    el('div', { class: 'lite-rev3d-grid' },
      el('div', { class: 'lite-rev3d-main' }, el('div', { id: 'liteRev3dHost', class: 'lite-rev3d-host' })),
      el('div', { class: 'lite-rev3d-side' },
        el('div', { class: 'lite-rev3d-player' },
          el('div', { class: 'lite-row' }, btn('Play', play, 'btn'), btn('Pause', pause, 'btn outline'), btn('Restart', () => { pause(); setT(0); }),
            el('b', { id: 'liteRev3dTime', class: 'lite-rev3d-time', text: '0.000 s / 0.000 s' })),
          el('div', { class: 'lite-rev3d-track' },
            el('input', { id: 'liteRev3dTimeline', type: 'range', min: '0', max: '0', step: '5', value: '0', 'aria-label': 'Recording time',
              oninput: (e) => { pause(); setT(Number(e.target.value) / 1000); } }))),
        el('div', { class: 'lite-rev3d-block' },
          el('div', { class: 'lite-rev3d-head' }, el('h4', { text: 'Collision check' })),
          el('div', { id: 'liteRev3dCheck', class: 'mini-note', text: 'Check collisions under the view checks every line of the recording.' }),
          el('div', { id: 'liteRev3dEvents', class: 'lite-rev3d-events' }),
          el('div', { class: 'mini-note', text: 'Walls as measured in Prepare (a few millimetres out changes the result). The glove, pad '
            + 'and brush are not modelled, so a real contact comes some 20 mm earlier. Nothing is blocked by this check.' })),
        el('div', { class: 'lite-rev3d-block' },
          el('div', { class: 'lite-rev3d-head' }, el('h4', { text: 'Trim start and end' }), btn('Reset', () => { trimTo(0, total()); })),
          el('div', { class: 'lite-dual-range' },
            el('input', { id: 'liteTrimA', type: 'range', min: '0', max: '0', step: '5', value: '0', 'aria-label': 'First moment kept',
              oninput: (e) => trimTo(Number(e.target.value) / 1000, trim.b, 'a') }),
            el('input', { id: 'liteTrimB', type: 'range', min: '0', max: '0', step: '5', value: '0', 'aria-label': 'Last moment kept',
              oninput: (e) => trimTo(trim.a, Number(e.target.value) / 1000, 'b') })),
          el('div', { class: 'lite-row' },
            el('div', { class: 'field' }, el('label', { for: 'liteTrimAs', text: 'Start (s)' }),
              el('input', { id: 'liteTrimAs', type: 'number', min: '0', step: '0.005', onchange: (e) => trimTo(Number(e.target.value), trim.b, 'a') })),
            el('div', { class: 'field' }, el('label', { for: 'liteTrimBs', text: 'End (s)' }),
              el('input', { id: 'liteTrimBs', type: 'number', min: '0', step: '0.005', onchange: (e) => trimTo(trim.a, Number(e.target.value), 'b') }))),
          el('div', { id: 'liteTrimNote', class: 'mini-note' }),
          el('div', { class: 'lite-row' },
            Object.assign(btn('Save', () => saveTrim(false), 'btn'), { id: 'liteTrimSave' }),
            el('div', { class: 'field lite-grow' }, el('label', { for: 'liteTrimName', text: 'Save As (a new recording)' }),
              el('input', { id: 'liteTrimName', type: 'text', autocomplete: 'off' })),
            Object.assign(btn('Save As', () => saveTrim(true), 'btn outline'), { id: 'liteTrimSaveAs' }))))));
  const anchor = $('liteRecordings') || stage.querySelector('.active-trajectory-bar');
  if ($('liteRecordings')) anchor.after(c); else if (anchor) anchor.before(c); else stage.append(c);
  tools = new Tools({
    what: 'the recording',
    check: async (phase, progress) => {
      const r = await checkJob(post, unitUrl(unit.key, 'collisions'), { which: 'raw', file: rec.file, task: task(), phase }, progress);
      return { result: r, spans: spansOf(r) };
    },
    seek: (t) => { pause(); setT(t); },
    changed: (phase) => { if (unit) scene(phase || unit.phase, undefined, sim.place || null).catch(() => {}); },
    shown: paintCheck,
  }).mount($('liteRev3dHost'));
  tools.track(c.querySelector('.lite-rev3d-track'));
  return c;
}

// ── what is shown: the active recording of the arm step on screen

async function show() {
  const s = step();
  const mine = ++token;
  if (!s || !ctx.pid || s.later) { $('liteRev3d')?.setAttribute('hidden', ''); return; }
  let u;
  try { u = await loadUnit(s.key); } catch (_) { return; }
  if (mine !== token) return;
  const r = (u.recordings || []).find(x => x.file === u.active);
  const c = card();
  if (!c) return;
  c.hidden = !r;
  if (!r) return;
  unit = u; rec = r;
  $('liteRev3dName').textContent = r.name;
  pause();
  tools.load(JSON.stringify([u.key, r.file, r.sha256, task()]), { phase: u.phase, text: "the region's" });
  const host = $('liteRev3dHost');
  if (elsewhere()) { invalidate(); return; }
  if (!host.querySelector('canvas')) say3d(host, 'Loading the 3D view…');
  try {
    if (!(await showFile(host, u, 'raw', null, task(), tools.phase)) || mine !== token) return;
  } catch (err) {
    say3d(host, `Nothing plays: ${say(err)}`, err);
    return;
  }
  onMoment((t) => tools.at(t == null ? -1 : t));
  const T = total();
  tools.setLength(T);
  for (const id of ['liteRev3dTimeline', 'liteTrimA', 'liteTrimB']) $(id).max = String(Math.round(T * 1000));
  setT(0);
  trimTo(0, T);
  const stem = r.name.replace(/\.txt$/i, '');
  if (!$('liteTrimName').dataset.for || $('liteTrimName').dataset.for !== r.file) {
    $('liteTrimName').value = `${stem}_trimmed.txt`;
    $('liteTrimName').dataset.for = r.file;
  }
}

// ── the player: the recording's own time; the 3D at that moment, staged

function setT(t, seek = true) {
  pb.t = Math.max(0, Math.min(total(), Number(t) || 0));
  const tl = $('liteRev3dTimeline');
  if (tl && document.activeElement !== tl) tl.value = String(Math.round(pb.t * 1000));
  if ($('liteRev3dTime')) $('liteRev3dTime').textContent = `${fmt(pb.t)} / ${fmt(total())}`;
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

// ── the collision check's findings (server/collision_check.py): the first collisions, one click away

const pairText = (p) => `${p.link} – ${p.object}${p.depth_mm ? ` (${p.depth_mm} mm deep)` : ''}`;

function paintCheck(r) {
  const box = $('liteRev3dEvents'), note = $('liteRev3dCheck');
  if (!box || !note) return;
  if (!r) {
    note.textContent = 'Check collisions under the view checks every line of the recording.';
    box.replaceChildren();
    return;
  }
  const c = r.contact || {};
  note.textContent = (r.ok ? 'No collision.' : `${r.colliding_lines.toLocaleString()} of ${r.lines.toLocaleString()} lines collide.`)
    + (c.lines ? ` The hand touches the commode on ${c.lines.toLocaleString()} lines from line ${c.first_line.toLocaleString()} `
      + `(deepest ${c.deepest_mm} mm; up to ${r.checked.contact_depth_mm} mm is the pad's give).` : '');
  box.replaceChildren(...r.events.slice(0, 6).map(e => el('div', { class: 'lite-rev3d-event' },
    el('span', { text: `Lines ${e.first_line.toLocaleString()}–${e.last_line.toLocaleString()} (${fmt((e.first_line - 1) * CYCLE_S)}): `
      + `${e.pairs.map(pairText).join('; ') || e.what.join(', ')}` }),
    el('button', { class: 'btn light', type: 'button', text: 'Show', onclick: () => { pause(); setT((e.first_line - 1) * CYCLE_S); } }))),
  r.ok ? '' : el('button', { class: 'btn', type: 'button', text: 'Teach again', onclick: () => window.setStage('teach') }));
}

// ── Trim: the first and the last moment kept

function trimTo(a, b, moved = null) {
  const T = total();
  a = Math.max(0, Math.min(T, Number(a) || 0));
  b = Math.max(0, Math.min(T, Number.isFinite(Number(b)) ? Number(b) : T));
  if (moved === 'a') a = Math.min(a, b - CYCLE_S);
  if (moved === 'b') b = Math.max(b, a + CYCLE_S);
  trim.a = Math.max(0, Math.round(a / CYCLE_S) * CYCLE_S);
  trim.b = Math.min(T, Math.round(b / CYCLE_S) * CYCLE_S);
  if ($('liteTrimA')) $('liteTrimA').value = String(Math.round(trim.a * 1000));
  if ($('liteTrimB')) $('liteTrimB').value = String(Math.round(trim.b * 1000));
  if ($('liteTrimAs') && document.activeElement !== $('liteTrimAs')) $('liteTrimAs').value = trim.a.toFixed(3);
  if ($('liteTrimBs') && document.activeElement !== $('liteTrimBs')) $('liteTrimBs').value = trim.b.toFixed(3);
  if (moved) { pause(); setT(moved === 'a' ? trim.a : trim.b); }      // the 3D at the moment being cut
  const first = Math.round(trim.a / CYCLE_S) + 1, last = Math.round(trim.b / CYCLE_S) + 1, lines = rec ? rec.lines : 0;
  if ($('liteTrimNote')) {
    $('liteTrimNote').textContent = rec ? `Keeps ${fmt(trim.a)} to ${fmt(trim.b)}: lines ${first.toLocaleString()}–${last.toLocaleString()}, `
      + `${(last - first + 1).toLocaleString()} of ${Number(lines).toLocaleString()} lines (${fmt((last - first) * CYCLE_S)}).` : '';
  }
  const cut = rec && (first > 1 || last < lines);
  for (const id of ['liteTrimSave', 'liteTrimSaveAs']) if ($(id)) $(id).disabled = !cut;
}

async function saveTrim(asNew) {
  if (!unit || !rec) return;
  const first = Math.round(trim.a / CYCLE_S) + 1, last = Math.round(trim.b / CYCLE_S) + 1;
  const name = ($('liteTrimName')?.value || '').trim();
  if (asNew && !name) { window.toast('Enter the name of the new recording.'); $('liteTrimName')?.focus(); return; }
  if (!asNew) {
    const yes = await confirmDialog({ title: `Overwrite ${rec.name}?`, ok: 'Overwrite it',
      body: `${rec.name} keeps lines ${first.toLocaleString()}–${last.toLocaleString()} of ${Number(rec.lines).toLocaleString()}; the rest is `
        + 'deleted from the file. Tune makes its file again from the trimmed recording.' });
    if (!yes) return;
  }
  try {
    const out = await post(`${unitUrl(unit.key, 'recordings')}/${encodeURIComponent(rec.file)}/trim`,
      { start: first, end: last, ...(asNew ? { name } : {}) });
    ctx.units[unit.key] = out.unit;
    const now = out.unit.recordings.find(x => x.file === out.unit.active);
    invalidate();                                       // the 3D reads the file again (Save keeps its name)
    window.toast(asNew ? `Saved as ${now ? now.name : name}: ${(last - first + 1).toLocaleString()} lines. It is the selected recording.`
      : `${rec.name} is trimmed: ${(last - first + 1).toLocaleString()} lines kept.`);
    document.dispatchEvent(new CustomEvent('lite:recordings'));
  } catch (err) { tell(err); }
}

export function installReview3d() {
  document.addEventListener('lite:stage', (e) => { if (e.detail === 'review') show(); else { pause(); token += 1; } });
  document.addEventListener('lite:step', () => { if (currentStage === 'review') show(); });
  document.addEventListener('lite:recordings', () => { if (currentStage === 'review') show(); });
  document.addEventListener('lite:placement', () => { if (tools) tools.load(null, { phase: unit?.phase || 'lid_closed' }); });   // the room changed: checked again
  document.addEventListener('lite:viewer-closed', () => { if (currentStage === 'review') show(); });
}
