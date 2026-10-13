// The 3D scene: the selected fixture in its lid/seat state inside the selected cubicle, in the
// robot's base frame (x forward, y left, z up). The robot is added by the caller (view3d.js).
import { css } from './plots.js';

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
export async function loadSTL(url) {
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

// The fixture as a Group in the base frame for `phase` (lid_closed | lid_open | seat_open). Each part's
// pose comes from the server, computed with ALIX's own scene_pose (venue rotation and hinges included).
export async function buildFixture(spec, phase = 'lid_closed') {
  const THREE = window.THREE;
  const col = (n) => new THREE.Color(css(n));
  const porcelain = new THREE.MeshStandardMaterial({ color: col('--vp-porcelain'), roughness: 0.35, metalness: 0.0, side: THREE.DoubleSide });
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

// Mounts the scene in `el`. `onReady(api)` gets {THREE, scene, camera, controls, render()}; set
// api.animating = true while something moves, or call api.render() after a change.
export function mountScene(el, { cubicleMm = 1400, phase = 'lid_closed', fixture = null, onReady = null } = {}) {
  let disposed = false, renderer = null, raf = 0, ro = null, controls = null;
  const api = { dirty: true, animating: false };
  ensureThree().then(async () => {
    if (disposed) return;
    const THREE = window.THREE;
    const col = (n) => new THREE.Color(css(n));
    const w = el.clientWidth || 600, h = el.clientHeight || 420;
    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
    renderer.setSize(w, h);
    el.prepend(renderer.domElement);
    const scene = new THREE.Scene();
    scene.background = col('--viewport-bg');
    const cam = new THREE.PerspectiveCamera(45, w / h, 0.01, 50);
    cam.up.set(0, 0, 1);
    cam.position.set(1.15, -1.95, 1.55);                                  // side view: robot left, fixture right
    controls = new THREE.OrbitControls(cam, renderer.domElement);
    controls.target.set(0.45, 0, 0.72);
    controls.update();
    controls.addEventListener('change', () => { api.dirty = true; });
    scene.add(new THREE.HemisphereLight(col('--vp-sky'), col('--vp-ground'), 0.75));
    const sun = new THREE.DirectionalLight(col('--vp-sky'), 0.75);
    sun.position.set(-2, -3, 4); scene.add(sun);
    const grid = new THREE.GridHelper(4, 40, col('--vp-grid'), col('--vp-grid-minor'));
    grid.rotation.x = Math.PI / 2; grid.position.x = 0.6; scene.add(grid);
    scene.add(new THREE.AxesHelper(0.25));                                   // the robot base frame
    Object.assign(api, { THREE, scene, camera: cam, controls, render: () => { api.dirty = true; } });

    try {                                                                    // cubicle walls
      const g = await loadSTL(`/mesh/cubicle/${cubicleMm}.stl`);
      const m = new THREE.Mesh(g, new THREE.MeshStandardMaterial({ color: col('--vp-wall'), transparent: true, opacity: 0.18, side: THREE.DoubleSide, depthWrite: false }));
      const edges = new THREE.LineSegments(new THREE.EdgesGeometry(g, 30), new THREE.LineBasicMaterial({ color: col('--vp-wall-edge') }));
      const wall = new THREE.Group(); wall.add(m, edges);
      wall.position.set(...CUBICLE_POSE.pos); wall.rotation.z = CUBICLE_POSE.yawDeg * D2R;
      scene.add(wall);
    } catch (e) { /* the view works without walls */ }
    if (fixture) {
      try { api.fixture = await buildFixture(fixture, phase); scene.add(api.fixture); } catch (e) { /* nor without the fixture */ }
    }
    api.dirty = true;
    const loop = () => {
      if (disposed) return;
      if (api.dirty || api.animating) { renderer.render(scene, cam); api.dirty = false; }
      raf = requestAnimationFrame(loop);
    };
    loop();
    ro = new ResizeObserver(() => {
      const W = el.clientWidth, H = el.clientHeight;
      if (!W || !H) return;
      renderer.setSize(W, H); cam.aspect = W / H; cam.updateProjectionMatrix();
      renderer.render(scene, cam);                                       // resizing clears the canvas
    });
    ro.observe(el);
    if (onReady) onReady(api);
  }).catch(() => { /* the 3D view is optional; the page still works */ });
  return () => {
    disposed = true;
    cancelAnimationFrame(raf);
    if (ro) ro.disconnect();
    if (controls) controls.dispose();
    if (renderer) { renderer.dispose(); renderer.domElement.remove(); }
  };
}
