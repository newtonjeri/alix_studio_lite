// STOP: every stop button of the page reaches the backend first, before any page work, and
// needs no login (handoff sections 21.13, 21.14 and 24). The page's own stop functions still
// run afterwards for their screen updates. While anything covers the top bar's STOP BOTH (the
// log-in screen, the Project Manager, a dialog, the Trajectory Viewer), a second one, in the
// page's own STOP style, stays on top of everything (lite.css).

let nested = 0;               // inside a STOP BOTH: the page's per-arm stops send nothing more

function send(path) {
  return fetch(path, {
    method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}', keepalive: true,
  }).then((r) => r.json()).catch(() => ({ error: 'STOP could not reach the Lite server.' }));
}

export const stopBoth = () => send('/api/robots/stop-both');
export const stopArm = (side) => send(`/api/robots/${side === 'right' ? 'right' : 'left'}/stop`);

function report(promise, done) {
  promise.then((d) => {
    if (d && d.error) window.toast(`${d.error} Press the red E-stop button on the robot if it is still moving.`);
    else window.toast(done);
  });
}

export function installSafety() {
  const fixed = document.createElement('button');
  fixed.type = 'button';
  fixed.className = 'emstop lite-stop-fixed';
  fixed.textContent = 'STOP BOTH';
  fixed.setAttribute('aria-label', 'STOP BOTH arms');
  fixed.addEventListener('click', () => window.demoStop());
  document.body.append(fixed);

  const pageStop = window.demoStop;
  window.demoStop = function demoStop() {               // top bar: STOP BOTH
    const sent = stopBoth();
    nested++;
    try { pageStop(); } finally { nested--; report(sent, 'STOP BOTH sent to both arms.'); }
  };

  const pageStopBoth = window.rcStopBoth;
  window.rcStopBoth = function rcStopBoth() {           // Advanced: Stop Both, and the page's own callers
    const sent = nested ? null : stopBoth();
    nested++;
    try { pageStopBoth(); } finally { nested--; if (sent) report(sent, 'STOP BOTH sent to both arms.'); }
  };

  const pageSlowStop = window.rcSlowStop;
  window.rcSlowStop = function rcSlowStop(side) {       // Advanced: SLOW STOP of one arm
    const sent = nested ? null : stopArm(side);
    try { pageSlowStop(side); } finally { if (sent) report(sent, `${String(side).toUpperCase()} arm stopped.`); }
  };
}
