// Engineer: settings and versions.
import { api, esc, h, store, refreshState, chip, ARM } from '../core.js';
import { toast, showError } from '../ui.js';

export async function render(main) {
  const s = store.state;
  if (s?.role !== 'engineer') {
    main.append(h('div.page', {}, [h('div.callout.info', { html: '<span class="g">i</span><div class="body"><strong>Engineer only.</strong><span>Switch to Engineer with the chip at the top right.</span></div>' })]));
    return;
  }
  const page = h('div.page');
  page.append(h('div.page-head', { html: '<div class="titles"><h2>Settings</h2><p>Robot addresses, the cubicle used by the 3D check, and versions.</p></div>' }));
  const grid = h('div.grid2');

  const robot = h('div.card');
  robot.innerHTML = '<div class="card-head"><h3>Robot</h3></div>';
  const rb = h('div.card-body');
  for (const side of ['left', 'right']) {
    const ip = h('input', { type: 'text', value: s.arms[side].ip, style: { width: '170px' } });
    ip.onchange = async () => { try { await api('settings', { arm: side, ip: ip.value }); await refreshState(); toast(`${ARM[side]} address saved.`, 'ok'); } catch (e) { showError(e); } };
    rb.append(h('label.field', {}, [`${ARM[side]} controller address`, ip]));
  }
  rb.append(h('div.muted.small', { html: `Robot connection: <b>${esc(s.robot_label)}</b>. The E-stop input is IO3 on each controller (mode 5); Lite checks it at connect and never changes IO modes.` }));
  robot.append(rb);

  const cell = h('div.card');
  cell.innerHTML = '<div class="card-head"><h3>Cubicle</h3></div>';
  const cb = h('div.card-body');
  const seg = h('div.segmented', { role: 'radiogroup' });
  for (const mm of [1400, 800]) {
    const b = h('button', { role: 'radio', onclick: async () => { try { await api('settings', { cubicle_mm: mm }); await refreshState(); render(main.replaceChildren() || main); } catch (e) { showError(e); } } },
      [`${mm} mm wide${mm === 1400 ? ' (default)' : ''}`]);
    if (s.cubicle_mm === mm) b.classList.add('on');
    seg.append(b);
  }
  cb.append(h('p', { text: 'The 3D check tests every motion against a single cubicle: a back wall 1.215 m in front of the robot base and two side walls.' }), seg,
    h('p.muted.small', { text: 'Confirm on site once: replay three known-good recordings through the 3D check and expect no wall contact.' }));
  cell.append(cb);

  const ver = h('div.card');
  ver.innerHTML = `<div class="card-head"><h3>About</h3></div><div class="card-body"><dl class="kv">
    <dt>ALIX Studio Lite</dt><dd class="mono">${esc(s.version)}</dd>
    <dt>Time base</dt><dd>5 ms per line (the controller cycle)</dd>
    <dt>Time-optimal optimiser</dt><dd>${s.toppra ? chip('ok', '✓', 'available') : chip('warn', '△', 'not installed')}</dd>
    <dt>Project</dt><dd>${esc(s.project?.name || '')}</dd>
    <dt>Engineer PIN</dt><dd>The same PIN as the Lite UI's Advanced Robot Controls (default 1234). Change it with <code>run_lite.sh --set-pin engineer</code>.</dd></dl></div>`;

  grid.append(robot, cell, ver);
  page.append(grid);
  main.append(page);
}
