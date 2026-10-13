// The 3D view tab: the robot, the fixture in the region's lid/seat state and the cubicle walls.
// Every joint can be moved by hand (the joint panel); a region's version or recording plays on
// the robot (Visual sim); the 3D check runs here.
import { api, esc, h, store, refreshState, chip, fmtS, durS, ARM, icon, watchJob, PIPE_ORDER, debounce } from '../core.js';
import { setStatus, toast, showError } from '../ui.js';
import { plotTheme, plotConfig, jointTraces, css, onTheme, plotBox } from '../plots.js';
import { setCurrentUnit, routeQuery } from '../app.js';
import { mountScene } from '../scene.js';
import { unitHeader, todoNow } from './unit.js';

export async function render(main, params) {
  const key = params[0] || null;
  const cleanups = [];
  const s = store.state;
  let u = null;
  if (key) {
    try { u = await api(`unit/${key}`); setCurrentUnit(u); } catch (e) { showError(e); }
  }
  let fx;
  try { fx = await api('fixtures'); } catch (e) { showError(e); return; }

  const page = h('div.page');
  if (u) page.append(unitHeader(u), todoNow(u, 'view3d'));
  else page.append(h('div.page-head', { html: `<div class="titles"><h2>3D view</h2><p>${esc(fx.spec.name)} in the ${s.cubicle_mm} mm cubicle.
    Move any joint with the panel. Choose a region (top left) to play its motion here.</p></div>` }));

  const wrap = h('div.split-3d.view3d');
  const left = h('div.stack');
  const vp = h('div.viewport.resizable', { id: 'vp3d' });
  const note = h('div.vp-note', { text: 'Loading the robot…' });
  vp.append(note);
  left.append(vp);

  // transport
  const play = h('button.btn', { html: `${icon('play')} Play` });
  const speed = h('select', { 'aria-label': 'Playback speed' });
  for (const v of ['0.25', '0.5', '1', '2', '4']) speed.append(h('option', { value: v, selected: v === '1' }, [`×${v}`]));
  const scrub = h('input', { type: 'range', min: '0', max: '1000', value: '0', 'aria-label': 'Position in the motion' });
  const tLab = h('span.mono', { text: '0.0 s' });
  const source = h('select', { 'aria-label': 'What to play' });
  const ver = u?.versions?.[u.versions.length - 1];
  if (ver) source.append(h('option', { value: ver.version }, [`Version ${ver.version} (${fmtS(durS(ver.lines))})`]));
  if (u?.recordings?.length) source.append(h('option', { value: 'raw' }, ['The recording']));
  const hasMotion = source.options.length > 0;
  left.append(h('div.transport', {}, hasMotion ? [play, speed, scrub, tLab, source] : [h('span.muted', { text: u ? 'This region has no recording yet: teach it or use an existing one.' : 'Choose a region to play its motion.' })]));

  const right = h('div.stack');
  const jointCard = h('div.card');
  jointCard.innerHTML = `<div class="card-head"><h3>Robot joints</h3><span class="muted small">Drag a slider to move a joint</span></div>`;
  const jointBody = h('div.card-body.joint-body', { html: '<div class="muted">Loading…</div>' });
  jointCard.append(jointBody);

  // the 3D check (needs a kept version)
  const checkCard = h('div.card');
  const last = ver ? [...u.results].reverse().find(e => e.kind === 'check_ok' && e.sha256 === ver.sha256) : null;
  checkCard.innerHTML = `<div class="card-head"><h3>3D check</h3>${last ? chip('ok', '✓', 'No problem found') : chip('todo', '○', ver ? 'Not checked yet' : 'Keep a version first')}
      ${last?.detail?.standin ? chip('standin', '·', 'stand-in') : ''}</div>`;
  const cb = h('div.card-body');
  checkCard.append(cb);
  cb.append(h('div', { html: `<b>What is checked</b><ul class="small" style="margin:6px 0 0;padding-left:20px">
      <li>Both arms with their hands, every line${ver ? ' of version ' + esc(ver.version) : ''}</li>
      <li>${esc(fx.spec.name)}${u ? ' in this step\'s state: ' + esc(u.toilet_state) : ''}</li>
      <li>The cubicle walls: ${s.cubicle_mm} mm wide</li><li>The bin: stowed</li><li>The other arm</li></ul>` }));
  if (last) cb.append(h('div.callout.ok', { html: `<span class="g">✓</span><div class="body"><strong>No collision found.</strong>
      <span class="muted">${esc(last.detail?.note || '')} Checked ${esc(last.at)}.</span></div>` }));
  const run = h('button.btn', { html: `${icon('cube')} ${last ? 'Check again' : 'Run 3D check'}` });
  run.disabled = !ver;
  const act = h('div.action-bar', {}, [run]);
  if (ver && !last) run.classList.add('primary');
  if (last) act.append(h('a.btn.primary', { href: `#/run/${u.key}` }, ['Next: Run']));
  cb.append(act);

  const plotCard = h('div.card');
  const PB = plotBox({ short: true });
  cleanups.push(PB.dispose);
  const pj = PB.el;
  if (hasMotion) {
    plotCard.innerHTML = '<div class="card-head"><h3>Joint angles</h3></div>';
    plotCard.append(h('div.card-body', {}, [PB.box]));
  }
  right.append(checkCard, jointCard);
  wrap.append(left, right);
  page.append(wrap);
  if (hasMotion) page.append(plotCard);
  main.append(page);

  run.onclick = async () => {
    run.disabled = true;
    const bar = h('div.progress', {}, [h('i')]);
    cb.prepend(bar);
    try {
      const job = await api(`unit/${u.key}/check3d`, {});
      const v = await watchJob(job, (j) => { setStatus(j.stage, true); bar.firstChild.style.width = `${j.n ? 100 * j.i / j.n : 0}%`; });
      setStatus(v.state === 'done' ? '3D check finished: no collision found (stand-in).' : v.error || 'Stopped.');
      if (v.state === 'done') toast('3D check: no collision found.', 'ok', 'Stand-in result until the 3D check backend is built.');
    } catch (e) { showError(e); }
    await refreshState();
    window.dispatchEvent(new HashChangeEvent('hashchange'));
  };

  // ── scene, robot and joint panel
  let sceneApi = null, robot = null, panel = null;
  const phase = u ? startPhase(u) : 'lid_closed';
  cleanups.push(mountScene(vp, { cubicleMm: s.cubicle_mm, phase, fixture: fx.spec, onReady: async (a) => {
    sceneApi = a;
    try {
      const [{ loadRobot }, { mountJointPanel }] = await Promise.all([import('../robot3d.js'), import('../jointpanel.js')]);
      robot = await loadRobot();
      a.scene.add(robot.group);
      a.render();
      note.textContent = `${fx.spec.name} · cubicle ${s.cubicle_mm} mm · drag to orbit, scroll to zoom`;
      jointBody.innerHTML = '';
      // a pole, hand or bin move changes the rest of the robot under a prepared motion: prepare again
      const reprepare = debounce(() => { if (track && !playing) prepare(); }, 600);
      panel = mountJointPanel(jointBody, robot, { onChange: (name) => { a.render(); if (!/^[LR]_joint[1-7]$/.test(name)) reprepare(); } });
      cleanups.push(() => panel?.dispose?.());
      if (track) { await prepare(); showFrame(0); }
      if (routeQuery().get('play') === '1' && hasMotion) play.click();
    } catch (e) {
      note.textContent = `${fx.spec.name} · cubicle ${s.cubicle_mm} mm`;
      jointBody.innerHTML = '<div class="muted">The robot model is not available yet.</div>';
    }
  } }));

  // ── playback (Visual sim): the region's arm follows the file at 5 ms per line
  let track = null, T = 0, cur = 0, playing = false, lastT = 0, raf = 0, lastPanel = 0;
  const head = (t) => [{ type: 'line', xref: 'x', yref: 'paper', x0: t, x1: t, y0: 0, y1: 1, line: { color: css('--brand'), width: 2 } }];
  const armJoints = () => [1, 2, 3, 4, 5, 6, 7].map(j => `${u.arm === 'left' ? 'L' : 'R'}_joint${j}`);
  function showFrame(t) {
    if (!track) return;
    const i = Math.min(track.lines - 1, Math.max(0, Math.round(t / 0.005)));
    if (robot) {
      // the server's FK frames, prefetched once per track (ALIX Studio's one-FK rule)
      if (robot.showTrackLine && prepared) robot.showTrackLine(i);
      else robot.setArmFromTrack(u.arm, track.joints.map(q => q[i]));
      const now = performance.now();
      if (panel && now - lastPanel > 120) {
        panel.update(Object.fromEntries(armJoints().map((n, j) => [n, track.joints[j][i]])));
        lastPanel = now;
      }
      sceneApi?.render();
    }
  }
  let prepared = false;
  async function prepare() {
    prepared = false;
    if (!robot || !track || !robot.prepareTrack) return;
    const every = Math.max(4, Math.ceil(track.lines / 1500));            // at most 1500 frames per track
    try { setStatus('Preparing the motion for the 3D view…', true); await robot.prepareTrack(u.arm, track.joints, every); prepared = true; setStatus('Ready to play.'); }
    catch (e) { setStatus('The 3D motion could not be prepared.'); }
  }
  function setT(t) {
    cur = Math.max(0, Math.min(T, t));
    tLab.textContent = fmtS(cur);
    scrub.value = String(T ? Math.round(1000 * cur / T) : 0);
    showFrame(cur);
    if (window.Plotly && pj.isConnected && pj.data) Plotly.relayout(pj, { shapes: head(cur) });
  }
  async function loadTrack() {
    if (!hasMotion) return;
    try { track = await api(`unit/${u.key}/track?which=${source.value}`); } catch (e) { showError(e); return; }
    T = durS(track.lines);
    await prepare();
    if (window.Plotly && pj.isConnected) {
      Plotly.react(pj, jointTraces(track.joints), plotTheme({ yaxis: { ...plotTheme().yaxis, title: { text: 'Joint angle (°)' } }, shapes: head(0) }), plotConfig);
    }
    setT(0);
  }
  const loop = (now) => {
    if (!playing) return;
    const dt = (now - lastT) / 1000; lastT = now;
    setT(cur + dt * parseFloat(speed.value));
    if (cur >= T) stopPlay(); else raf = requestAnimationFrame(loop);
  };
  function stopPlay() {
    playing = false; cancelAnimationFrame(raf);
    if (sceneApi) sceneApi.animating = false;
    play.innerHTML = `${icon('play')} Play`;
    if (panel && robot) panel.update(robot.getJoints());
  }
  play.onclick = () => {
    if (!track) return;
    if (playing) { stopPlay(); return; }
    playing = true; play.innerHTML = `${icon('pause')} Pause`;
    if (sceneApi) sceneApi.animating = true;
    if (cur >= T) setT(0);
    lastT = performance.now(); raf = requestAnimationFrame(loop);
  };
  scrub.oninput = () => setT(T * parseInt(scrub.value, 10) / 1000);
  source.onchange = loadTrack;
  const onStop = () => stopPlay();
  window.addEventListener('lite-stop', onStop);
  cleanups.push(() => { stopPlay(); window.removeEventListener('lite-stop', onStop); });
  cleanups.push(onTheme(() => { if (track && window.Plotly && pj.isConnected) Plotly.relayout(pj, plotTheme({ shapes: head(cur) })); }));
  await loadTrack();

  return () => { for (const f of cleanups) { try { f(); } catch (e) { /* ignore */ } } };
}

// the toilet state at the START of a step (lid and seat moves change it during the step)
export function startPhase(u) {
  const t = (u.toilet_state || '').toLowerCase();
  if (t.startsWith('lid closed')) return 'lid_closed';
  if (t.startsWith('lid and seat up') || t.startsWith('seat open')) return 'seat_open';
  if (t.startsWith('lid open')) return 'lid_open';
  return u.phase || 'lid_closed';
}
