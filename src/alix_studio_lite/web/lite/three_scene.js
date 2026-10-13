// The 3D view of the Lite UI: the project's fixture in a region's lid/seat state inside the cubicle,
// in the robot's base frame (x forward, y left, z up), with the robot (robot3d.js) posed by ALIX's FK.
// One renderer for the page: attach(el) moves it into V19's simulation pane, the expanded view or
// the review preview, so the robot model is loaded once. Built from Legacy Lite's scene.js.

export function css(name) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
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
  if (!ready) ready = (window.THREE ? Promise.resolve() : loadScript('/vendor/three.min.js'))
    .then(() => (window.THREE.OrbitControls ? null : loadScript('/vendor/OrbitControls.js')));
  return ready;
}

const meshCache = new Map();
function loadSTL(url) {
  if (!meshCache.has(url)) {
    meshCache.set(url, fetch(url).then(r => { if (!r.ok) throw new Error(url); return r.arrayBuffer(); }).then(parseSTL));
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

// The viewer: created once, attached to one host element at a time.
export function createViewer() {
  const v = { dirty: true, animating: false, host: null, ready: null };
  let renderer = null, scene = null, cam = null, controls = null, ro = null, wall = null, fixture = null;
  let wallKey = '', fixtureKey = '';

  v.ready = ensureThree().then(() => {
    const THREE = window.THREE;
    const col = (n) => new THREE.Color(css(n));
    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
    renderer.domElement.className = 'lite-3d-canvas';
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
    const loop = () => {
      if (v.host && (v.dirty || v.animating)) { renderer.render(scene, cam); v.dirty = false; }
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
    const k = Math.max(1, 1.33 / aspect);
    controls.target.set(...HOME_TARGET);
    cam.position.set(...HOME_TARGET.map((t, i) => t + HOME_OFFSET[i] * k));
    controls.update();
    fittedAspect = aspect;
  };
  v.resize = () => {
    if (!renderer || !v.host) return;
    const W = v.host.clientWidth, H = v.host.clientHeight;
    if (!W || !H) return;
    renderer.setSize(W, H); cam.aspect = W / H; cam.updateProjectionMatrix();
    if (!fittedAspect || Math.abs(Math.log(cam.aspect / fittedAspect)) > 0.15) v.fit(cam.aspect);
    renderer.render(scene, cam);                                         // resizing clears the canvas
  };

  v.attach = async (el) => {
    await v.ready;
    if (v.host === el) { v.resize(); return; }
    if (v.host) ro.unobserve(v.host);
    v.host = el;
    el.prepend(renderer.domElement);
    ro.observe(el);
    v.resize();
    v.dirty = true;
  };

  // The cubicle walls and the fixture in `phase`, rebuilt only when they change.
  v.setScene = async ({ cubicleMm = 1400, fixtureSpec = null, fixtureId = '', phase = 'lid_closed' } = {}) => {
    await v.ready;
    const THREE = window.THREE;
    const col = (n) => new THREE.Color(css(n));
    if (wallKey !== String(cubicleMm)) {
      if (wall) scene.remove(wall);
      wall = null;
      wallKey = String(cubicleMm);
      try {
        const g = await loadSTL(`/mesh/cubicle/${cubicleMm}.stl`);
        const m = new THREE.Mesh(g, new THREE.MeshStandardMaterial({ color: col('--vp-wall'), transparent: true, opacity: 0.18, side: THREE.DoubleSide, depthWrite: false }));
        const edges = new THREE.LineSegments(new THREE.EdgesGeometry(g, 30), new THREE.LineBasicMaterial({ color: col('--vp-wall-edge') }));
        wall = new THREE.Group(); wall.add(m, edges);
        wall.position.set(...CUBICLE_POSE.pos); wall.rotation.z = CUBICLE_POSE.yawDeg * D2R;
        scene.add(wall);
      } catch (e) { /* the view works without walls */ }
    }
    const key = `${fixtureId}:${phase}`;
    if (fixtureSpec && fixtureKey !== key) {
      if (fixture) scene.remove(fixture);
      fixture = null;
      fixtureKey = key;
      try { fixture = await buildFixture(fixtureSpec, phase); scene.add(fixture); } catch (e) { /* nor without the fixture */ }
    }
    v.dirty = true;
  };

  v.add = async (obj) => { await v.ready; scene.add(obj); v.dirty = true; };
  return v;
}
