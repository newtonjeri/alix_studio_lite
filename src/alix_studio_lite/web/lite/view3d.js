// Every 3D view's own tools, the same in each (the owner, 16 Oct: "All 3d view ports should have the
// same properties"): Tune's Review and Compare, Save & Review, Prepare's transition window, the
// Trajectory Viewer, S1's review of a merge and S2's simulation. Under the view:
// - Commode: the state the commode is shown in. The preset is the region's own (each region has
//   one, set in Prepare), a merge's, or each Program's in S2; a state chosen here overrides it in
//   this view only, never in the region's settings;
// - Check collisions, on demand, of the trajectory on screen in the state shown (the walls, the
//   commode and the other arm: server/collision_check.py), with Prev / Next collision;
// - at the moment shown, the robot's links in collision are red and go back to their own colours
//   after (ALIX Studio's dual-arm collision tab, editor.js dacHighlight), whether the view plays,
//   is dragged or clicked on its timeline, or jumps to a collision; red marks on the timeline show
//   where.
// The view cube and Home View are the 3D's own (three_scene.js), in every view. Only the view that
// has the 3D (its host holds the picture) colours the robot.

import { say } from './api.js';
import { el, watchJob } from './ctx.js';
import { loaded3D } from './sim.js';

const CYCLE_S = 0.005;
export const PHASES = [['lid_closed', 'Lid closed'], ['lid_open', 'Lid open'], ['seat_open', 'Seat open']];
export const PHASE_WORDS = { lid_closed: 'lid closed', lid_open: 'lid open', seat_open: 'seat open' };
const fmt = (s) => `${Number(s).toFixed(3)} s`;

// A check's colliding moments on the view's clock, [t0, t1, links] in order. A file's check numbers
// its lines from 1 (`first`: the time of its line 1 on the view's clock; line n shows from half a line
// before its time to half a line after); a run's check gives its own times (S2, the Viewer's pair).
export function spansOf(r, first = 0) {
  if (!r || !Array.isArray(r.spans)) return [];
  if (r.samples == null) return r.spans.map(([a, b, links]) => [first + (a - 1.5) * CYCLE_S, first + (b - 0.5) * CYCLE_S, links]);
  const h = (Number(r.dt) || 0) / 2;
  return r.spans.map(([a, b, links]) => [a - h, b + h, links]);
}

// What a check says, in a line: the walls, the commode's state, the other arm.
export function checkWords(r) {
  if (!r) return '';
  const c = r.checked || {};
  const n = r.lines != null ? `${Number(r.lines).toLocaleString()} lines` : `${Number(r.samples || 0).toLocaleString()} moments`;
  const against = [c.walls, r.phase || c.commode ? `the commode (${PHASE_WORDS[r.phase || c.commode] || r.phase || c.commode})` : 'the commode',
    c.other_arm || (c.two_arms ? 'each other' : null)].filter(Boolean).join(', ');
  return `${n} checked against ${against}`;
}

export class Tools {
  // `check(phase, progress)`: the view's check of what it shows, in `phase` (null: the preset's states),
  // resolving to {result, spans} (spans on the view's clock); `seek(t)`: the view shows moment t, its
  // player paused; `changed(phase)`: the commode's state to show (null: the preset's); `shown(result)`:
  // the result shown now (null: none); `time(t)`: a moment as the view writes its times.
  constructor({ check, seek, changed, shown = null, what = 'the trajectory', time = fmt }) {
    Object.assign(this, { checkFn: check, seek, changed, shown, what, time, host: null, key: null, presetPhase: 'lid_closed', presetText: '',
      override: '', results: new Map(), spans: [], events: [], t: 0, length: 0, said: '', busy: 0, tracks: [], enabled: false });
    const btn = (text, onclick, cls = 'btn light') => el('button', { class: cls, type: 'button', text, onclick });
    this.sel = el('select', { 'aria-label': 'The commode in this view', onchange: () => this.pick(this.sel.value) });
    this.go = btn('Check collisions', () => this.run(), 'btn');
    this.prev = btn('Prev collision', () => this.jump(-1));
    this.next = btn('Next collision', () => this.jump(1));
    this.state = el('span', { class: 'lite-vtools-state', role: 'status', text: 'Not checked yet.' });
    this.now = el('div', { class: 'lite-vtools-now', 'aria-live': 'off' });
    this.el = el('div', { class: 'lite-vtools' },
      el('div', { class: 'lite-row lite-vtools-row' },
        el('label', { class: 'lite-vtools-field' }, el('span', { text: 'Commode' }), this.sel),
        this.go, this.prev, this.next),
      this.state, this.now);
    this.options();
    this.paint();
  }

  // Under the view's 3D (`after`: the element the tools follow, the host by default).
  mount(host, after = host) {
    this.host = host;
    if (this.el.previousElementSibling !== after) after.after(this.el);
    return this;
  }

  // A timeline of the view (a range input's box): its red marks. `length`: the timeline's length (s);
  // `only`: the view's one timeline now (Tune's Review or Compare), the others lose their marks.
  track(box, length = null, only = false) {
    if (only) {
      for (const b of this.tracks) if (b !== box) b.querySelector(':scope > .lite-vtools-marks')?.remove();
      this.tracks = this.tracks.filter(b => b === box);
    }
    if (!this.tracks.includes(box)) this.tracks.push(box);
    if (length != null) this.length = Math.max(0, Number(length) || 0);
    this.marks();
  }

  setLength(length) { this.length = Math.max(0, Number(length) || 0); this.marks(); }

  // What the view shows now (`key`: the file and what it was made from); another one forgets the checks
  // and the state chosen. `phase`: its preset state (null where it changes over the run: S2), `text`:
  // whose it is. `enabled`: there is something to check.
  load(key, { phase = 'lid_closed', text = "the region's", enabled = true } = {}) {
    const fresh = key !== this.key, was = this.phase;
    this.key = key;
    this.enabled = enabled;
    this.presetPhase = phase;
    this.presetText = text;
    if (fresh) {
      this.busy += 1;                                       // a check still running is for the file before
      this.results.clear();
      this.override = '';
      this.use(null);
    } else if (this.phase !== was) {                        // the region's state changed in Prepare: its own check
      this.use(this.results.get(this.phase || '') || null);
    }
    this.options();
    this.paint();
  }

  // The state shown: the one chosen here, else the preset (null: each moment's own, S2).
  get phase() { return this.override || this.presetPhase; }

  options() {
    const preset = this.presetPhase ? `Preset: ${PHASE_WORDS[this.presetPhase]} (${this.presetText})` : `Preset: ${this.presetText}`;
    this.sel.replaceChildren(el('option', { value: '', text: preset }), ...PHASES.map(([v, n]) => el('option', { value: v, text: n })));
    this.sel.value = this.override;
  }

  pick(v) {
    this.override = v && v !== this.presetPhase ? v : '';
    this.sel.value = this.override;
    this.use(this.results.get(this.phase || '') || null);
    if (this.changed) this.changed(this.phase);
    this.paint();
  }

  // A result the view already has (S2's simulation checks its own run): kept for `phase` (null: the preset's).
  provide(phase, out) {
    this.results.set(phase || '', out);
    if ((this.phase || '') === (phase || '')) this.use(out);
    this.paint();
  }

  async run() {
    if (!this.enabled || !this.checkFn) return;
    const my = ++this.busy, phase = this.phase, key = this.key;
    this.go.disabled = true;
    this.say(`Checking ${this.what} for collisions${phase ? ` with the ${PHASE_WORDS[phase]}` : ''}…`);
    try {
      const out = await this.checkFn(phase, (text) => { if (my === this.busy) this.say(text); });
      if (my !== this.busy || key !== this.key) return;
      this.results.set(phase || '', out);
      if ((this.phase || '') === (phase || '')) this.use(out);
    } catch (err) {
      if (my === this.busy) this.say(`The collision check could not run: ${say(err)}`, 'warn');
      return;
    } finally {
      if (my === this.busy) this.go.disabled = !this.enabled;
    }
    this.paint();
  }

  // A result shown ({result, spans}), or none.
  use(out) {
    this.out = out;
    this.spans = out ? [...out.spans].sort((a, b) => a[0] - b[0]) : [];
    const ev = [];                                           // the collisions as the timeline shows them
    for (const [a, b] of this.spans) {
      const last = ev[ev.length - 1];
      if (last && a <= last[1] + 0.02) last[1] = Math.max(last[1], b); else ev.push([a, b]);
    }
    this.events = ev;
    this.marks();
    this.at(this.t);
    if (this.shown) this.shown(out && out.result);             // the view's own words on it (Save & Review's list)
  }

  say(text, cls = '') {
    this.state.textContent = text;
    this.state.className = `lite-vtools-state${cls ? ` ${cls}` : ''}`;
  }

  paint() {
    this.go.disabled = !this.enabled;
    const r = this.out && this.out.result;
    this.prev.disabled = this.next.disabled = !this.events.length;
    if (!this.enabled) { this.say('Nothing to check here yet.'); return; }
    if (!r) {
      this.say(this.results.size ? `Not checked with the ${PHASE_WORDS[this.phase] || 'preset states'} yet: press Check collisions.`
        : 'Not checked yet: press Check collisions.');
      return;
    }
    if (r.ok) this.say(`No collision: ${checkWords(r)}.`, 'ok');
    else {
      const first = this.events[0];
      this.say(`${this.events.length} collision${this.events.length === 1 ? '' : 's'}${first ? `, the first at ${this.time(Math.max(0, first[0] + CYCLE_S))}` : ''}: `
        + `${checkWords(r)}. The links in collision turn red as it plays.`, 'err');
    }
  }

  // The red marks on each timeline.
  marks() {
    const T = Math.max(1e-6, this.length);
    for (const box of this.tracks) {
      let layer = box.querySelector(':scope > .lite-vtools-marks');
      if (!layer) { layer = el('div', { class: 'lite-vtools-marks', 'aria-hidden': 'true' }); box.append(layer); }
      layer.replaceChildren(...(this.length ? this.events : []).map(([a, b]) => el('span', { class: 'lite-vtools-mark',
        style: `left:${Math.max(0, (100 * a) / T)}%;width:${Math.max(0.4, (100 * (Math.min(T, b) - Math.max(0, a))) / T)}%` })));
    }
  }

  // The links in collision at t (s, the view's clock), or null.
  linksAt(t) {
    const s = this.spans;
    let lo = 0, hi = s.length - 1, hit = -1;
    while (lo <= hi) {                                       // the last span starting at or before t
      const mid = (lo + hi) >> 1;
      if (s[mid][0] <= t) { hit = mid; lo = mid + 1; } else hi = mid - 1;
    }
    for (let i = hit; i >= 0 && i > hit - 4; i -= 1) if (t < s[i][1]) return s[i][2];
    return null;
  }

  // The view shows moment t: its links in collision red, the others their own colours.
  at(t) {
    this.t = Number(t) || 0;
    const links = this.linksAt(this.t);
    const three = loaded3D();
    if (three && this.host && three.viewer.host === this.host) {      // only the view that has the 3D colours it
      if (three.robot.highlight(links || [])) three.viewer.dirty = true;
    }
    const text = links ? `At ${this.time(this.t)}: ${links.join(', ')} in collision.` : '';
    if (text !== this.said) { this.said = text; this.now.textContent = text; this.now.classList.toggle('on', !!links); }
  }

  // Prev / Next collision: the first colliding moment of the one before or after the moment shown (from
  // either end, round to the other).
  jump(dir) {
    if (!this.events.length) return;
    const t = this.t, eps = 0.003, at = (x) => Math.max(0, x[0] + CYCLE_S);
    let e = dir > 0 ? this.events.find(x => at(x) > t + eps) : [...this.events].reverse().find(x => at(x) < t - eps);
    if (!e) e = dir > 0 ? this.events[0] : this.events[this.events.length - 1];
    this.seek(at(e));
  }
}

// A check run as a job (the routes answer with one), its progress said as it goes.
export async function checkJob(post, url, body, progress) {
  const job = await post(url, body);
  const done = await watchJob(job, (j) => { if (j.state === 'running' && progress) progress(`${j.stage || j.title}…`); });
  if (done.state !== 'done') throw Object.assign(new Error(done.error || 'The check stopped.'), { hint: done.hint });
  return done.result;
}
