// The page's one 3D view, borrowed by Merge + Full Sequence (S1's review of a merge, S2's
// simulation of a run), and the player both use: Play / Pause / Restart on the run's own clock, at
// 1x unless a faster replay is chosen. Taking the view makes the region's simulation (Tune, Save &
// Review) load its file again when it is shown next (sim.invalidate; fullseq.js tells it on the way
// back).

import { ensure3D, invalidate, say3d } from './sim.js';

let held = false;

export async function borrow(host) {
  const { viewer, robot } = await ensure3D();
  host.classList.add('lite-3d');
  await viewer.attach(host);
  invalidate();
  held = true;
  say3d(host, null);
  return { viewer, robot };
}

// Whether S1 or S2 had the view (the region's simulation must load its file again), once.
export function hadIt() {
  const was = held;
  held = false;
  return was;
}

// A clock: `onTime(t)` shows moment t (s); `onEnd()` when it reaches `total` while playing.
export class Player {
  constructor({ onTime, onEnd = null, onState = null }) {
    Object.assign(this, { onTime, onEnd, onState, total: 0, t: 0, speed: 1, playing: false, raf: 0, t0: 0, w0: 0 });
  }

  load(total) {
    this.pause();
    this.total = Math.max(0, Number(total) || 0);
    this.seek(0);
  }

  play() {
    if (!this.total) return;
    if (this.t >= this.total - 1e-3) this.seek(0);
    this.playing = true;
    this.t0 = this.t;
    this.w0 = performance.now();
    cancelAnimationFrame(this.raf);
    const tick = (now) => {
      if (!this.playing) return;
      const t = Math.min(this.total, this.t0 + ((now - this.w0) / 1000) * this.speed);
      this.t = t;
      this.onTime(t, true);
      if (t >= this.total) {
        this.pause();
        if (this.onEnd) this.onEnd();
        return;
      }
      this.raf = requestAnimationFrame(tick);
    };
    this.raf = requestAnimationFrame(tick);
    if (this.onState) this.onState(true);
  }

  pause() {
    const was = this.playing;
    this.playing = false;
    cancelAnimationFrame(this.raf);
    this.raf = 0;
    if (was && this.onState) this.onState(false);
  }

  setSpeed(x) {
    const go = this.playing;
    if (go) this.pause();
    this.speed = Math.max(0.25, Math.min(8, Number(x) || 1));
    if (go) this.play();
  }

  seek(t) {
    this.t = Math.max(0, Math.min(this.total, Number(t) || 0));
    if (this.playing) { this.t0 = this.t; this.w0 = performance.now(); }
    this.onTime(this.t, false);
  }
}
