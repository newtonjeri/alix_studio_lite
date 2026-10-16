// Every 3D viewport the same (the owner, 16 Oct): by default a square half the screen wide; maximized to
// 90 % of the screen with the button in its bottom-left corner (Escape, or the button again, gives it back);
// the title in its top-left corner and the time readout in its bottom-right one; the view cube and Home
// View in its top-right corner (three_scene.js). Their sizes follow the viewport's (lite.css: container
// units; the cube from its width), and the zoom is held between a nearest and a farthest distance
// (three_scene.js) so the robot stays in view.

import { el } from './ctx.js';

const frames = new WeakMap();
let open = null;                                 // the viewport maximized now: {host, hole, scrim, btn, inert}

function restore() {
  if (!open) return;
  const { host, hole, scrim, btn, inert } = open;
  open = null;
  host.classList.remove('lite-vp-max');
  for (const n of inert) n.inert = false;
  if (hole.isConnected) hole.replaceWith(host);
  else host.remove();                             // its view was closed meanwhile (a window removed): it goes with it
  scrim.remove();
  btn.title = 'Maximize the view (90 % of the screen)';
  btn.setAttribute('aria-label', btn.title);
  btn.setAttribute('aria-pressed', 'false');
  document.removeEventListener('keydown', onKey, true);
}

function onKey(e) {
  if (e.key === 'Escape' && open && !document.querySelector('.lite-confirm, .lite-picker-panel')) { e.stopPropagation(); restore(); }
}

// The maximized viewport leaves a hole of its size where it was (the page does not move) and sits on a
// scrim, above the page and the windows, below the questions Lite asks. What is behind it cannot be
// reached by the keyboard either (inert); the fixed STOP BOTH (safety.js) and what Lite says stay in reach.
function maximize(host, btn) {
  if (open) { const was = open.host; restore(); if (was === host) return; }
  const r = host.getBoundingClientRect();
  const hole = el('div', { class: 'lite-vp-hole', 'aria-hidden': 'true', style: `width:${r.width}px;height:${r.height}px` });
  const scrim = el('div', { class: 'lite-vp-scrim', 'aria-hidden': 'true', onclick: restore });
  host.replaceWith(hole);
  const inert = [...document.body.children].filter(n => !n.inert && !n.matches('.lite-stop-fixed, #toast, script, style'));
  for (const n of inert) n.inert = true;
  document.body.append(scrim, host);
  host.classList.add('lite-vp-max');
  btn.title = 'Back to the page';
  btn.setAttribute('aria-label', btn.title);
  btn.setAttribute('aria-pressed', 'true');
  open = { host, hole, scrim, btn, inert };
  document.addEventListener('keydown', onKey, true);
}

const ICON = '<svg viewBox="0 0 24 24" width="100%" height="100%" aria-hidden="true" fill="none" stroke="currentColor" stroke-width="2.2" '
  + 'stroke-linecap="round" stroke-linejoin="round"><path d="M4 9V4h5M20 9V4h-5M4 15v5h5M20 15v5h-5"/></svg>';

// The frame of a view's 3D element: {title(text, sub), time(text)}. A title of the page's own (V32's
// caption in Tune) stays and is placed as the others are.
export function frame(host) {
  if (!host) return { title() {}, time() {} };
  if (frames.has(host)) return frames.get(host);
  host.classList.add('lite-vp');
  let title = host.querySelector(':scope > .simulation-caption');
  if (title) title.classList.add('lite-vp-title');
  else {
    title = el('div', { class: 'lite-vp-title', hidden: true }, el('strong'), el('span'));
    host.append(title);
  }
  const time = el('div', { class: 'lite-vp-time', hidden: true, 'aria-live': 'off' });
  const btn = el('button', { type: 'button', class: 'lite-vp-maxbtn', title: 'Maximize the view (90 % of the screen)',
    'aria-label': 'Maximize the view (90 % of the screen)', 'aria-pressed': 'false' });
  btn.innerHTML = ICON;
  btn.addEventListener('click', () => maximize(host, btn));
  host.append(time, btn);
  const f = {
    title(text, sub = '') {
      const s = title.querySelector('strong'), p = title.querySelector('span');
      if (s) s.textContent = text || '';
      if (p) p.textContent = sub || '';
      title.hidden = !text;
    },
    time(text) {
      if (time.textContent !== text) time.textContent = text || '';
      time.hidden = !text;
    },
  };
  frames.set(host, f);
  return f;
}

// Leaving a view with its viewport maximized gives the viewport back first.
export function restoreViewport() { restore(); }

export function installVp() {
  for (const ev of ['lite:stage', 'lite:step', 'lite:open-s1']) document.addEventListener(ev, restore);
  const pageShow = window.showView;
  window.showView = function showView(...a) { restore(); return pageShow(...a); };
}
