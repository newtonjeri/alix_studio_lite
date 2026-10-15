// The 3D view of the Lite UI: the project's fixture in a region's lid/seat state inside the cubicle,
// in the robot's base frame (x forward, y left, z up), with the robot (robot3d.js) posed by ALIX's FK.
// One scene and one robot for the page, so the robot model is loaded once. Its renderer moves:
// attach(el) puts it in V19's simulation pane, the expanded view or the review preview; a picture
// of the page (pictures.js) borrows it with a camera of its own, and the next attach() gives the
// camera back as it was. Where pictures show side by side (the two arm cards), view() adds a
// renderer of the SAME scene with its own camera and controls. Built from Legacy Lite's scene.js.

export function css(name) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
}

// Objects only the pictures show: each region's hand path, on its arm's layer (each arm card sees its
// own); the simulation's own camera never sees them.
export const PATH_LAYERS = { left: 1, right: 2 };

// Points a camera at look = {target, dir, radius} (metres, base frame; dir: from the target towards
// the camera): a ball of that radius around the target fills the view, whatever its shape.
function aim(cam, controls, look) {
  const THREE = window.THREE;
  const half = Math.atan(Math.tan((cam.fov * Math.PI) / 360) * Math.min(1, cam.aspect));
  const dir = new THREE.Vector3(...look.dir).normalize();
  controls.target.set(...look.target);
  cam.position.copy(controls.target).addScaledVector(dir, look.radius / Math.sin(half));
  controls.update();
}

let ready = null;
function loadScript(src) {
  return new Promise((res, rej) => {
    const s = document.createElement('script');
    s.src = src; s.onload = res; s.onerror = () => rej(new Error('could not load ' + src));
    document.head.append(s);
  });
}
export function ensureThree() {
  if (!ready) {
    ready = (window.THREE ? Promise.resolve() : loadScript('/vendor/three.min.js'))
      .then(() => (window.THREE.OrbitControls ? null : loadScript('/vendor/OrbitControls.js')));
    ready.catch(() => { ready = null; });              // a failed load is tried again next time
  }
  return ready;
}

const meshCache = new Map();
function loadSTL(url) {
  if (!meshCache.has(url)) {
    const p = fetch(url).then(r => { if (!r.ok) throw new Error(url); return r.arrayBuffer(); }).then(parseSTL);
    p.catch(() => meshCache.delete(url));              // a failed download is tried again next time
    meshCache.set(url, p);
  }
  return meshCache.get(url);
}
function parseSTL(buf) {
  const THREE = window.THREE;
  const dv = new DataView(buf);
  const n = dv.getUint32(80, true);
  const pos = new Float32Array(n * 9), nor = new Float32Array(n * 9);
  let o = 84;
  for (let i = 0; i < n; i++, o += 50) {
    const nx = dv.getFloat32(o, true), ny = dv.getFloat32(o + 4, true), nz = dv.getFloat32(o + 8, true);
    for (let v = 0; v < 3; v++) {
      const k = i * 9 + v * 3, p = o + 12 + v * 12;
      pos[k] = dv.getFloat32(p, true); pos[k + 1] = dv.getFloat32(p + 4, true); pos[k + 2] = dv.getFloat32(p + 8, true);
      nor[k] = nx; nor[k + 1] = ny; nor[k + 2] = nz;
    }
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  g.setAttribute('normal', new THREE.BufferAttribute(nor, 3));
  return g;
}

const CUBICLE_POSE = { pos: [1.215, 0, 0], yawDeg: 180 };               // the cubicle venue files
const D2R = Math.PI / 180;

// The fixture as a Group for `phase` (lid_closed | lid_open | seat_open); each part's pose comes
// from the server, computed with ALIX's scene_pose (venue rotation and hinges included).
async function buildFixture(spec, phase) {
  const THREE = window.THREE;
  const col = (n) => new THREE.Color(css(n));
  const porcelain = new THREE.MeshStandardMaterial({ color: col('--vp-porcelain'), roughness: 0.35, side: THREE.DoubleSide });
  const seatMat = new THREE.MeshStandardMaterial({ color: col('--vp-seat'), roughness: 0.5, side: THREE.DoubleSide });
  const root = new THREE.Group();
  for (const part of spec.parts || []) {
    const url = (part.state_meshes && part.state_meshes[phase]) || part.mesh;
    const m = part.matrices && (part.matrices[phase] || part.matrices.lid_closed);
    if (!url || !m) continue;
    const mesh = new THREE.Mesh(await loadSTL(url), part.name === 'lid' || part.name === 'seat' ? seatMat : porcelain);
    mesh.scale.set(...(part.scale || [1, 1, 1]));
    const g = new THREE.Group();
    new THREE.Matrix4().set(...m).decompose(g.position, g.quaternion, g.scale);
    g.add(mesh);
    root.add(g);
  }
  return root;
}

// When the browser gives no WebGL, the 3D panes say so with ALIX Studio's notice (web_editor/editor.js
// initThree, its .webgl-fail block) and its fixes for this browser (webglHelpHtml). Chrome turns WebGL
// off for every page where the computer's graphics run in software (Mesa llvmpipe, e.g. while the GPU
// driver is not loaded: the lab laptop, 14 Oct), so Lite's launcher opens its own Chrome window with
// the GPU blocklist ignored (server/__init__.py); the notice's first fix is to use that window.
export const NO_WEBGL = '3-D view unavailable: WebGL could not start a rendering context on this browser/GPU.';

// ALIX Studio's webglHelpHtml (web_editor/editor.js), with Lite's own window first for Chromium browsers.
function webglHelpHtml() {
  const ua = navigator.userAgent || '';
  const lite = 'Open Lite with <code>tools/run_lite.sh</code>: it starts its own Chrome window, which draws the '
    + '3-D even when the graphics run in software. ';
  const chromiumFix = scheme =>
    'First: open <code>' + scheme + '://gpu</code> — it names the reason under '
    + '<em>Graphics Feature Status</em>. Then: <code>' + scheme + '://settings/system</code> → '
    + '"Use graphics acceleration when available", <code>' + scheme
    + '://flags/#ignore-gpu-blocklist</code> → Enabled, or relaunch with '
    + '<code>--enable-unsafe-swiftshader</code> if it fell back to software rendering.';
  if (/Edg\//.test(ua)) return 'Fixes (Edge): ' + chromiumFix('edge');
  if (/OPR\//.test(ua)) return 'Fixes (Opera): ' + chromiumFix('opera');
  if (/Chrome\//.test(ua)) return 'Fixes (Chrome): ' + lite + chromiumFix('chrome');
  if (/Firefox\//.test(ua)) {
    return 'Fixes (Firefox): open <code>about:support</code> → <em>Graphics</em> for the reason. '
      + 'Then: <code>about:config</code> → <code>webgl.disabled=false</code>, '
      + '<code>gfx.webrender.all=true</code>, or launch with <code>MOZ_X11_EGL=1</code>. '
      + 'The snap build often cannot reach the GPU at all — the deb build can.';
  }
  return 'Fixes: enable hardware acceleration in your browser’s settings, update the GPU driver, '
    + 'or try a different browser. Your browser’s GPU diagnostic page (<code>about:support</code> on '
    + 'Firefox, <code>&lt;browser&gt;://gpu</code> on Chromium-based ones) names the actual reason.';
}

// Fills `el` with ALIX Studio's in-viewport notice: what is off, what still works, and the fixes.
export function webglFail(el) {
  el.classList.add('webgl-fail');
  el.innerHTML = '<strong>⚠ 3-D view unavailable</strong>'
    + '<span>WebGL could not start a rendering context on this browser/GPU. Teaching, tuning and the '
    + 'robot still work — only the 3-D view is off.</span>'
    + '<span>' + webglHelpHtml() + '</span>';
}

// The viewer: created once, attached to one host element at a time. ONE WebGL context for the
// whole page: the pictures shown side by side (view()) are drawn by this same renderer and copied
// into plain canvases, so the robot's meshes are held once (where the graphics run in software,
// every context draws on the processor).
export function createViewer() {
  const v = { dirty: true, animating: false, host: null, ready: null, borrowedBy: null };
  let renderer = null, scene = null, cam = null, controls = null, ro = null, wall = null, fixture = null;
  let wallKey = '', fixtureKey = '', wallGen = 0, fixtureGen = 0, sceneCall = 0;
  let room = null, roomKey = '', offset = [0, 0];          // the measured room, and the commode's offset for the UGV's placement
  const views = new Set();                       // the secondary views (view()), drawn by the same loop
  let lent = null, borrowed = null;              // the camera before a picture borrowed it; the picture's aim

  v.ready = ensureThree().then(() => {
    const THREE = window.THREE;
    const col = (n) => new THREE.Color(css(n));
    // ALIX Studio's WebGL start (editor.js initThree): 'high-performance' can fail to create a
    // context on some hybrid Intel+NVIDIA Linux setups; fall back to the default GPU, then to bare
    // options, before giving up. The rest of the page works without 3-D.
    for (const opts of [{ antialias: true, powerPreference: 'high-performance' }, { antialias: true }, {}]) {
      try { renderer = new THREE.WebGLRenderer(opts); break; }
      catch (err) { console.warn('[ALIX Studio Lite] WebGLRenderer failed for', opts, err); }
    }
    if (!renderer) throw Object.assign(new Error(NO_WEBGL), { code: 'webgl' });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.domElement.className = 'lite-3d-canvas';
    // a lost context (the GPU reset) comes back by itself when the browser restores it
    renderer.domElement.addEventListener('webglcontextlost', (e) => {
      e.preventDefault();
      v.lost = true;
      document.dispatchEvent(new CustomEvent('lite:3d-lost'));
    });
    renderer.domElement.addEventListener('webglcontextrestored', () => {
      v.lost = false;
      v.dirty = true;
      for (const w of views) w.dirty = true;
      document.dispatchEvent(new CustomEvent('lite:3d-restored'));
    });
    scene = new THREE.Scene();
    scene.background = col('--lite-viewport-bg');
    cam = new THREE.PerspectiveCamera(45, 1.5, 0.01, 50);
    cam.up.set(0, 0, 1);
    cam.position.set(1.15, -1.95, 1.55);                                  // side view: robot left, fixture right
    controls = new THREE.OrbitControls(cam, renderer.domElement);
    controls.target.set(0.45, 0, 0.72);
    controls.update();
    controls.addEventListener('change', () => { v.dirty = true; });
    scene.add(new THREE.HemisphereLight(col('--vp-sky'), col('--vp-ground'), 0.75));
    const sun = new THREE.DirectionalLight(col('--vp-sky'), 0.75);
    sun.position.set(-2, -3, 4); scene.add(sun);
    const grid = new THREE.GridHelper(4, 40, col('--vp-grid'), col('--vp-grid-minor'));
    grid.rotation.x = Math.PI / 2; grid.position.x = 0.6; scene.add(grid);
    ro = new ResizeObserver(() => v.resize());
    // One loop draws every view of the scene: each when the scene changed, or its own camera or size.
    // The side-by-side views go first (each borrows the canvas at its size), the main view last.
    const loop = () => {
      if (!v.lost) {
        const changed = v.dirty || v.animating;
        let drawn = false, again = false;
        for (const w of views) if (w.frame(changed)) { drawn = true; again = true; }
        if (v.host && (changed || again)) { renderer.render(scene, cam); drawn = true; }
        if (drawn) v.dirty = false;
      }
      requestAnimationFrame(loop);
    };
    loop();
    Object.assign(v, { THREE, scene });
    return v;
  });

  // A narrow pane sees less across, so the camera steps back along its line of sight until
  // the cell (robot to cubicle wall) fits as it does in a 4:3 pane.
  const HOME_TARGET = [0.45, 0, 0.72], HOME_OFFSET = [0.70, -1.95, 0.83];
  let fittedAspect = 0;
  v.fit = (aspect) => {
    fittedAspect = aspect;
    if (borrowed) { aim(cam, controls, borrowed); return; }               // a picture's own view
    const k = Math.max(1, 1.33 / aspect);
    controls.target.set(...HOME_TARGET);
    cam.position.set(...HOME_TARGET.map((t, i) => t + HOME_OFFSET[i] * k));
    controls.update();
  };
  v.resize = () => {
    if (!renderer || !v.host) return;
    const W = v.host.clientWidth, H = v.host.clientHeight;
    if (!W || !H) return;
    renderer.setSize(W, H); cam.aspect = W / H; cam.updateProjectionMatrix();
    if (!fittedAspect || Math.abs(Math.log(cam.aspect / fittedAspect)) > 0.15) v.fit(cam.aspect);
    renderer.render(scene, cam);                                         // resizing clears the canvas
  };

  const place = (el) => {
    if (v.host === el) { v.resize(); return; }
    if (v.host) ro.unobserve(v.host);
    v.host = el;
    el.prepend(renderer.domElement);
    ro.observe(el);
    v.resize();
    v.dirty = true;
  };

  // The simulation pane, the preview, the Viewer and the full sequence: the camera comes back as it
  // was before a picture borrowed the view.
  v.attach = async (el) => {
    await v.ready;
    if (lent) {
      cam.position.copy(lent.position); controls.target.copy(lent.target);
      controls.enableZoom = lent.zoom; cam.layers.mask = lent.layers; fittedAspect = lent.fitted;
      controls.update();
      lent = null; borrowed = null; v.borrowedBy = null;
    }
    place(el);
  };

  // A picture shows the scene with a camera of its own (`look`, see aim()); the camera as it was is
  // kept for the next attach(). `zoom`: the wheel zooms (in a window; on the page it scrolls the
  // page). `paths`: 'left' or 'right', that arm's region path shows too (PATH_LAYERS).
  v.borrow = async (el, look, { zoom = false, paths = null } = {}) => {
    await v.ready;
    if (!lent) lent = { position: cam.position.clone(), target: controls.target.clone(), zoom: controls.enableZoom, layers: cam.layers.mask, fitted: fittedAspect };
    borrowed = look;
    v.borrowedBy = el;
    controls.enableZoom = zoom;
    for (const layer of Object.values(PATH_LAYERS)) cam.layers.disable(layer);
    if (PATH_LAYERS[paths]) cam.layers.enable(PATH_LAYERS[paths]);
    place(el);
    v.aim(look);
  };
  // While borrowed: the picture's camera, aimed again (another arm, another fixture).
  v.aim = (look) => {
    if (!v.borrowedBy) return;
    borrowed = look;
    aim(cam, controls, look);
    fittedAspect = cam.aspect;
    v.dirty = true;
  };

  // A secondary view: a plain canvas with its own camera and controls, showing this same scene (no
  // second robot, no second WebGL context): the one renderer draws it at its size and the picture
  // is copied in. For pictures shown side by side; pictures.js keeps two and reuses them.
  // `paused()`: true while another window has the scene (the view keeps its last picture). `paths`:
  // 'left' or 'right', that arm's region path shows too.
  const kept = { x: 0, y: 0 };
  const drawView = (w) => {
    const THREE = window.THREE;
    const size = renderer.getSize(new THREE.Vector2());
    kept.x = size.x; kept.y = size.y;
    renderer.setSize(w.W, w.H, false);
    renderer.render(scene, w.cam);
    w.g2.clearRect(0, 0, w.canvas.width, w.canvas.height);
    w.g2.drawImage(renderer.domElement, 0, 0, w.canvas.width, w.canvas.height);
    renderer.setSize(kept.x, kept.y, false);
  };
  v.view = async ({ paused = null, paths = null } = {}) => {
    await v.ready;
    const THREE = window.THREE;
    const canvas = document.createElement('canvas');
    canvas.className = 'lite-3d-canvas';
    const g2 = canvas.getContext('2d');
    const c = new THREE.PerspectiveCamera(45, 1.5, 0.01, 50);
    c.up.set(0, 0, 1);
    if (PATH_LAYERS[paths]) c.layers.enable(PATH_LAYERS[paths]);
    const oc = new THREE.OrbitControls(c, canvas);
    oc.enableZoom = false;
    const w = { host: null, cam: c, controls: oc, canvas, g2, W: 0, H: 0, look: null, shown: false, dirty: true,
      missed: false, touched: false, fitted: 0, minGapMs: 0, last: 0 };
    oc.addEventListener('change', () => { w.dirty = true; });
    oc.addEventListener('start', () => { w.touched = true; });            // turned by hand: a resize keeps it
    const wro = new ResizeObserver(() => w.resize());
    const io = new IntersectionObserver((es) => { w.shown = es[es.length - 1].isIntersecting; if (w.shown) w.dirty = true; });
    w.resize = () => {
      if (!w.host) return;
      const W = w.host.clientWidth, H = w.host.clientHeight;
      if (!W || !H) return;
      const pr = renderer.getPixelRatio();
      w.W = W; w.H = H;
      canvas.width = Math.round(W * pr); canvas.height = Math.round(H * pr);
      canvas.style.width = `${W}px`; canvas.style.height = `${H}px`;
      c.aspect = W / H; c.updateProjectionMatrix();
      if (w.look && !w.touched && (!w.fitted || Math.abs(Math.log(c.aspect / w.fitted)) > 0.15)) { aim(c, oc, w.look); w.fitted = c.aspect; }
      w.dirty = true;
    };
    w.attach = (el) => {
      if (w.host !== el) {
        if (w.host) { wro.unobserve(w.host); io.unobserve(w.host); }
        w.host = el;
        el.prepend(canvas);
        wro.observe(el); io.observe(el);
      }
      w.resize();
    };
    w.aim = (look) => { w.look = look; w.touched = false; aim(c, oc, look); w.fitted = c.aspect; w.dirty = true; };
    // Draws when due; `changed`: the scene changed. A view off screen or paused catches up later.
    // minGapMs: at most one picture per gap while the scene keeps changing (a small mirror of a run).
    w.frame = (changed) => {
      if (!(changed || w.dirty || w.missed)) return false;
      if (!w.host || !w.shown || (paused && paused())) { w.missed = true; return false; }
      const now = performance.now();
      if (!w.dirty && w.minGapMs && now - w.last < w.minGapMs) { w.missed = true; return false; }
      if (!w.W || !w.H) { w.missed = true; return false; }
      drawView(w);
      w.dirty = w.missed = false;
      w.last = now;
      return true;
    };
    views.add(w);
    return w;
  };

  // The fixture's box in the scene (THREE.Box3), or null while it is not built.
  v.fixtureBox = () => (fixture ? new window.THREE.Box3().setFromObject(fixture) : null);
  // The theme changed: the viewport's background follows it.
  v.retheme = () => { if (scene) { scene.background = new window.THREE.Color(css('--lite-viewport-bg')); v.dirty = true; } };

  // The measured room (server/placement.py): three walls like ALIX's cubicle meshes, open at the front,
  // and the front wall as a line on the floor. Without one, ALIX's venue cubicle stays.
  const showRoom = (THREE, place) => {
    const key = place && place.walls ? JSON.stringify([place.walls, place.front_x, place.room_y]) : '';
    if (key !== roomKey) {
      roomKey = key;
      if (room) { scene.remove(room); room = null; }
      if (key) {
        const col = (n) => new THREE.Color(css(n));
        const mat = new THREE.MeshStandardMaterial({ color: col('--vp-wall'), transparent: true, opacity: 0.18, side: THREE.DoubleSide, depthWrite: false });
        const edge = new THREE.LineBasicMaterial({ color: col('--vp-wall-edge') });
        room = new THREE.Group();
        for (const w of place.walls) {
          const g = new THREE.BoxGeometry(...w.size);
          const m = new THREE.Mesh(g, mat);
          const e = new THREE.LineSegments(new THREE.EdgesGeometry(g), edge);
          m.position.set(...w.center); e.position.set(...w.center);
          room.add(m, e);
        }
        if (place.front_x != null && place.room_y) {
          const pts = [new THREE.Vector3(place.front_x, place.room_y[0], 0.002), new THREE.Vector3(place.front_x, place.room_y[1], 0.002)];
          room.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), edge));
        }
        scene.add(room);
      }
    }
    if (wall) wall.visible = !room;
  };

  // The cubicle walls and the fixture in `phase`, rebuilt only when they change. `placement`: the
  // scene of the room and the UGV's placement (server/placement.py: the fixture's offset, the walls).
  v.setScene = async ({ cubicleMm = 1400, fixtureSpec = null, fixtureId = '', phase = 'lid_closed', placement = null } = {}) => {
    await v.ready;
    const THREE = window.THREE;
    const col = (n) => new THREE.Color(css(n));
    const call = ++sceneCall;
    // Changes can overlap (a slider dragged across states): only the newest one is put in the scene.
    if (wallKey !== String(cubicleMm)) {
      const gen = ++wallGen;
      wallKey = String(cubicleMm);
      try {
        const g = await loadSTL(`/mesh/cubicle/${cubicleMm}.stl`);
        if (gen !== wallGen) return;
        const m = new THREE.Mesh(g, new THREE.MeshStandardMaterial({ color: col('--vp-wall'), transparent: true, opacity: 0.18, side: THREE.DoubleSide, depthWrite: false }));
        const edges = new THREE.LineSegments(new THREE.EdgesGeometry(g, 30), new THREE.LineBasicMaterial({ color: col('--vp-wall-edge') }));
        if (wall) scene.remove(wall);
        wall = new THREE.Group(); wall.add(m, edges);
        wall.position.set(...CUBICLE_POSE.pos); wall.rotation.z = CUBICLE_POSE.yawDeg * D2R;
        wall.visible = !room;                              // a measured room replaces the venue's walls
        scene.add(wall);
      } catch (e) { /* the view works without walls */ }
    }
    if (call !== sceneCall) { v.dirty = true; return; }   // a newer change came while the walls loaded: it sets the fixture
    offset = (placement && placement.offset_m) || [0, 0];
    showRoom(THREE, placement);
    const key = `${fixtureId}:${phase}`;
    if (fixtureSpec && fixtureKey !== key) {
      const gen = ++fixtureGen;
      fixtureKey = key;
      let built = null;
      try { built = await buildFixture(fixtureSpec, phase); } catch (e) { /* the view works without the fixture */ }
      if (gen !== fixtureGen) return;
      if (fixture) scene.remove(fixture);
      fixture = built;
      if (fixture) scene.add(fixture);
    }
    if (fixture) fixture.position.set(offset[0], offset[1], 0);
    v.dirty = true;
  };

  v.add = async (obj) => { await v.ready; scene.add(obj); v.dirty = true; };
  v.remove = (obj) => { if (scene && obj) { scene.remove(obj); v.dirty = true; } };
  return v;
}
