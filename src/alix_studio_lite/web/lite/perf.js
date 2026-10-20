// The page's own speed, said and logged (the owner, 17 Oct: "sometimes, when the simulation is visualized
// in the browser, it lags a lot ... probably the same as the slowed counting ... traced to the browser
// cache"). Measured here: with the 3D drawn in software (no GPU), each frame of the robot holds the page's
// thread about 130 ms (7 frames a second), and every timer waits behind it, the drag timer's voice too (late,
// then catching up); with a GPU a frame takes a few ms. Chrome draws in software when its GPU process has
// failed, until it is started again (Lite clears the GPU caches of its own Chrome window at each start:
// server/__init__.py).
// - the 3D's device (three_scene.js): in software, the operator is told how to get the GPU back, and it is
//   logged (the Activity Logs, SYSTEM);
// - the page's long tasks (over 50 ms): a minute with many, or one over half a second, is logged with the
//   3D's device, so a slow session shows in the lab's logs.

let device = null, software = false, told = false;
const longs = [];

function log(event, details, result = 'WARNING') {
  try { window.auditLog?.('SYSTEM', event, details, result); } catch (_) { /* the page still works */ }
}

export function installPerf() {
  document.addEventListener('lite:3d-device', (e) => {
    ({ device, software } = e.detail || {});
    log('3D device', `${software ? 'software rendering (no GPU in use)' : 'GPU'}: ${device}`, software ? 'WARNING' : 'INFO');
    if (software && !told) {
      told = true;
      window.toast('The 3D is drawn in software (no graphics card in use), so simulations will lag. Close this Chrome '
        + 'window and open Lite again (its terminal: run_lite); if it stays, restart the computer.', 'warn');
    }
  });
  try {
    new PerformanceObserver((list) => { for (const e of list.getEntries()) longs.push(e.duration); })
      .observe({ type: 'longtask', buffered: false });
  } catch (_) { return; }                       // a browser without long-task timing: nothing to say
  setInterval(() => {
    const n = longs.length, worst = n ? Math.max(...longs) : 0, held = longs.reduce((a, b) => a + b, 0);
    longs.length = 0;
    if (n >= 30 || worst >= 500) {
      log('Page slow', `${n} long tasks in the last minute (${(held / 1000).toFixed(1)} s held, the longest ${Math.round(worst)} ms); `
        + `3D: ${device ? `${software ? 'software' : 'GPU'} (${device})` : 'not started'}`);
    }
  }, 60000);
}
