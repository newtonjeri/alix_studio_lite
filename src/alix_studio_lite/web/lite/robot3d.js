// The robot in 3D, posed the way ALIX Studio poses its robot overlay. The server sends each link's
// meshes once (/api/lite/robot-model) and computes every link's base-frame transform with ALIX's FK
// (/api/lite/robot-pose, or /api/lite/robot-track-frames for a whole track: server/robot_model.py);
// this module applies one 4x4 per link. Its API takes degrees and millimetres; the server works in
// radians and metres. scene.js renders on demand: render after a change (setJoint's promise and the
// joint panel's onChange come once the new pose is shown), or set api.animating while a motion plays.
//
// Copied from Legacy Lite (web_legacy/js/robot3d.js). Changes: its two imports are local, and
// loadFrames() shows frames the server computed for a whole file (GET …/units/<KEY>/frames).
import { css, ensureThree } from './three_scene.js';

async function api(path, body) {                     // Legacy Lite's robot routes, shared by both UIs
  const opt = body === undefined ? {} : { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) };
  const r = await fetch('/api/lite/' + path, opt);
  const d = await r.json();
  if (d && d.error) throw new Error(d.error);
  return d;
}

const D2R = Math.PI / 180;
const MOVABLE = new Set(['revolute', 'continuous', 'prismatic']);
const POSE_GAP_MS = 50;           // at most ~20 pose requests a second; the latest state wins
const ALBEDO_MAX = 0.62;          // ALIX Studio's robot look: CAD white clamped to real white paint

export async function loadRobot() {
  await ensureThree();
  const THREE = window.THREE;
  const d = await api('robot-model');

  // One Group per link with meshes, directly under the robot's base frame, so a link's base-frame
  // transform is its Group's transform.
  const group = new THREE.Group();
  group.name = 'robot';
  const linkObj = {};
  const materials = new Map();
  const geometries = [];
  for (const [link, ids] of Object.entries(d.links)) {
    if (!ids.length) continue;
    const g = new THREE.Group();
    g.name = link;
    for (const id of ids) {
      const mesh = meshFromPayload(THREE, d.meshes[id], materials);
      mesh.name = id;
      geometries.push(mesh.geometry);
      g.add(mesh);
    }
    linkObj[link] = g;
    group.add(g);
  }
  const M = new THREE.Matrix4();
  const applyPose = (poseMap) => applyRobotPose(M, linkObj, poseMap);

  // Joint values, in degrees or mm, for the joints that are set (mimic joints follow on the server).
  const isLift = (j) => j.type === 'prismatic';
  const toSI = (j, v) => (isLift(j) ? v / 1000 : v * D2R);
  const fromSI = (j, v) => (isLift(j) ? v * 1000 : v / D2R);
  const joints = d.joints.filter(j => MOVABLE.has(j.type)).map(j => ({
    name: j.name, type: j.type, unit: isLift(j) ? 'mm' : 'deg', parent: j.parent, child: j.child,
    lower: j.lower == null ? -180 : fromSI(j, j.lower),       // a continuous joint has no limits
    upper: j.upper == null ? 180 : fromSI(j, j.upper),
    home: fromSI(j, d.home[j.name] || 0), mimic: j.mimic,
  }));
  const spec = new Map(joints.map(j => [j.name, j]));
  const independent = joints.filter(j => !j.mimic);
  const values = Object.fromEntries(independent.map(j => [j.name, j.home]));
  const clampTo = (j, v) => Math.min(j.upper, Math.max(j.lower, Number(v)));
  const isNum = (v) => (typeof v === 'number' ? Number.isFinite(v) : typeof v === 'string' && v.trim() !== '' && Number.isFinite(Number(v)));
  const settable = (name, v) => { const j = spec.get(name); return j && !j.mimic && isNum(v) ? j : null; };
  const siState = () => Object.fromEntries(independent.map(j => [j.name, toSI(j, values[j.name])]));

  // Poses: every change bumps `want`; one request at a time, at most one per POSE_GAP_MS, always
  // for the newest state, so a reply is never older than the pose on screen.
  let want = 0, shown = 0, inFlight = false, timer = 0, lastSend = 0;
  const waiting = [];                                          // [seq, resolve, value]
  const settle = () => {
    for (let i = waiting.length - 1; i >= 0; i--) {
      if (waiting[i][0] <= shown) { waiting[i][1](waiting[i][2]); waiting.splice(i, 1); }
    }
  };
  const pump = () => {
    if (inFlight || timer || shown >= want) return;
    timer = setTimeout(send, Math.max(0, lastSend + POSE_GAP_MS - performance.now()));
  };
  async function send() {
    timer = 0;
    if (shown >= want) return;
    const seq = want;
    inFlight = true;
    lastSend = performance.now();
    try {
      const r = await api('robot-pose', { joints: siState() });
      if (seq > shown) applyPose(r.pose);
    } catch (e) {
      console.warn('robot pose:', e.message);                  // the pose stays; the next change asks again
    }
    shown = Math.max(shown, seq);
    inFlight = false;
    settle();
    pump();
  }
  const changed = (result) => {                                // a change the server has to pose
    want += 1;
    const p = new Promise(res => waiting.push([want, res, result]));
    pump();
    return p;
  };
  const shownHere = () => { want += 1; shown = want; settle(); };   // posed without a request

  // Sets one joint (degrees or mm), clamped to its limits. Resolves to the value set once the new
  // pose is on the robot, or to null for an unknown or mimic joint.
  function setJoint(name, value) {
    const j = settable(name, value);
    if (!j) return Promise.resolve(null);
    values[name] = clampTo(j, value);
    return changed(values[name]);
  }
  // Sets several joints; resolves to all joint values once the new pose is on the robot.
  function setJoints(map) {
    let any = false;
    for (const [name, value] of Object.entries(map || {})) {
      const j = settable(name, value);
      if (j) { values[name] = clampTo(j, value); any = true; }
    }
    return any ? changed(getJoints()) : Promise.resolve(getJoints());
  }
  function getJoints() {
    return { ...values };
  }
  // ALIX Studio's standing pose; its transforms come with the model, so this needs no request.
  function home() {
    for (const j of independent) values[j.name] = j.home;
    applyPose(d.homePose);
    shownHere();
    return Promise.resolve(getJoints());
  }
  // The link's frame in the robot's base frame (Lite's scene world), or null for a link with no mesh.
  function linkWorldMatrix(name) {
    const g = linkObj[name];
    return g ? new THREE.Matrix4().compose(g.position, g.quaternion, g.scale) : null;
  }

  // Tracks: the server poses the moving links for every k-th line in one request; showing a line
  // then applies the nearest frame at once. A recording's 7 values (degrees, joint1..7) are the
  // URDF's joint1..7 as they are: ALIX feeds the controller's joint array to its FK the same way.
  let track = null;
  const armNames = (s) => [1, 2, 3, 4, 5, 6, 7].map(i => `${s}_joint${i}`);
  const sideOf = (side) => {
    const s = String(side || '').charAt(0).toUpperCase();
    if (s !== 'L' && s !== 'R') throw new Error('Say which arm: left or right.');
    return s;
  };
  const restOf = (s) => { const o = siState(); for (const n of armNames(s)) delete o[n]; return o; };
  const sameRest = () => {                                     // frames are only right for the rest as it was
    const now = restOf(track.side);
    return Object.keys(now).every(k => now[k] === track.rest[k]);
  };
  const lineKey = (q) => q.map(v => Number(v).toFixed(3)).join(',');
  function applyFrame(f) {
    const L = track.links.length, a = track.frames;
    for (let k = 0; k < L; k++) {
      const g = track.links[k];
      if (!g) continue;
      const o = (f * L + k) * 16;
      M.set(a[o], a[o + 1], a[o + 2], a[o + 3], a[o + 4], a[o + 5], a[o + 6], a[o + 7],
        a[o + 8], a[o + 9], a[o + 10], a[o + 11], a[o + 12], a[o + 13], a[o + 14], a[o + 15]);
      M.decompose(g.position, g.quaternion, g.scale);
    }
  }
  // Fetches the frames of a track ([7][N] degrees) for every `everyLines`-th line, with the rest of
  // the robot as it is now. Resolves to {frames, lines, every}.
  async function prepareTrack(side, jointsDeg7xN, everyLines = 1) {
    const s = sideOf(side);
    const rest = restOf(s);
    const r = await api('robot-track-frames', {
      side: s === 'L' ? 'left' : 'right', joints_deg: jointsDeg7xN, every: Math.max(1, Math.round(everyLines)), joints: rest,
    });
    const byKey = new Map(r.lines.map((line, f) => [lineKey(jointsDeg7xN.map(q => q[line])), f]));
    track = { side: s, names: armNames(s), q: jointsDeg7xN, lines: r.lines, every: r.every, rest, byKey,
      links: r.links.map(l => linkObj[l] || null), frames: new Float32Array(decode(r.frames).buffer) };
    return { frames: r.lines.length, lines: jointsDeg7xN[0].length, every: r.every };
  }
  // Frames the server computed for a whole file (same shape as robot-track-frames, plus
  // total_lines), for the robot as it stands now. Resolves to {frames, lines, every}.
  function loadFrames(side, r) {
    const s = sideOf(side);
    track = { side: s, names: armNames(s), q: null, lines: r.lines, every: r.every, rest: restOf(s), byKey: new Map(),
      links: r.links.map(l => linkObj[l] || null), frames: new Float32Array(decode(r.frames).buffer), total: r.total_lines };
    return { frames: r.lines.length, lines: r.total_lines, every: r.every };
  }
  // Shows line i of the prepared track: the nearest frame, at once. If the rest of the robot moved
  // since prepareTrack, the server poses the line instead (the returned promise resolves then).
  function showTrackLine(i) {
    if (!track) return Promise.resolve(false);
    const n = track.q ? track.q[0].length : track.total;
    const line = Math.min(n - 1, Math.max(0, Math.round(Number(i) || 0)));
    if (track.q) track.names.forEach((name, j) => { values[name] = clampTo(spec.get(name), track.q[j][line]); });
    else if (!sameRest()) return Promise.resolve(false);
    if (!sameRest()) return changed(false);
    const last = track.lines.length - 1;
    let f = Math.min(last, Math.round(line / track.every));
    if (Math.abs(track.lines[last] - line) < Math.abs(track.lines[f] - line)) f = last;
    applyFrame(f);
    shownHere();
    return Promise.resolve(true);
  }
  // Sets an arm from a recording's 7 values (degrees). A line of the prepared track shows at once;
  // any other values are posed by the server.
  function setArmFromTrack(side, q7deg) {
    const s = sideOf(side);
    if (track && track.side === s && sameRest()) {
      const f = track.byKey.get(lineKey(q7deg));
      if (f !== undefined) {
        track.names.forEach((name, j) => { values[name] = clampTo(spec.get(name), q7deg[j]); });
        applyFrame(f);
        shownHere();
        return Promise.resolve(getJoints());
      }
    }
    return setJoints(Object.fromEntries(armNames(s).map((name, j) => [name, q7deg[j]])));
  }
  function dispose() {
    clearTimeout(timer);
    if (group.parent) group.parent.remove(group);
    for (const g of geometries) g.dispose();
    for (const m of materials.values()) m.dispose();
  }

  home();
  return {
    group, joints, groups: d.groups, setJoint, setJoints, getJoints, home, linkWorldMatrix,
    setArmFromTrack, prepareTrack, loadFrames, showTrackLine, dispose, stats: d.stats,
  };
}

// ALIX Studio's applyRobotPose (web_editor/editor.js:12426-12437). poseMap: link -> 16 numbers,
// row-major, base frame; decomposed into position/quaternion/scale so the normal (auto-update)
// transform path is used. Here a link's meshes share one Group; Studio poses each mesh.
function applyRobotPose(M, linkObj, poseMap) {
  if (!poseMap) return;
  for (const [link, arr] of Object.entries(poseMap)) {
    const g = linkObj[link];
    if (!g || arr.length !== 16) continue;
    M.set(...arr);                                             // .set() takes row-major args
    M.decompose(g.position, g.quaternion, g.scale);
  }
}

// ALIX Studio's meshFromPayload, indexed branch (web_editor/editor.js:667-702), with the robot visual
// set's look from loadRobotGeometry (editor.js:12366-12379): matte, CAD white clamped. The payload
// carries base64 float32 positions and uint32 indices instead of JSON lists. One material per colour.
function meshFromPayload(THREE, m, materials) {
  const geometry = new THREE.BufferGeometry();
  const pos = new Float32Array(decode(m.positions).buffer);
  const idx = new Uint32Array(decode(m.indices).buffer);
  geometry.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  geometry.setIndex(new THREE.BufferAttribute(pos.length / 3 <= 65536 ? Uint16Array.from(idx) : idx, 1));
  geometry.computeVertexNormals();
  geometry.computeBoundingSphere();
  geometry.computeBoundingBox();
  const key = m.color ? m.color.join(',') : '';
  if (!materials.has(key)) {
    materials.set(key, new THREE.MeshStandardMaterial({ color: robotColour(THREE, m.color), roughness: 0.9, metalness: 0.0, side: THREE.DoubleSide }));
  }
  return new THREE.Mesh(geometry, materials.get(key));
}

// Studio turns the authored (sRGB) colour linear, clamps its brightest channel to 0.62 so CAD white
// does not glow, and renders with sRGB output. Lite's renderer writes colours as they are, so the
// same on-screen colour is that clamp turned back to sRGB. A mesh with no colour gets the scene's
// blue-grey, Studio's default robot tint.
function robotColour(THREE, rgb) {
  const c = rgb ? new THREE.Color(rgb[0], rgb[1], rgb[2]) : new THREE.Color(css('--vp-wall-edge'));
  c.convertSRGBToLinear();
  const mx = Math.max(c.r, c.g, c.b);
  if (mx > ALBEDO_MAX) c.multiplyScalar(ALBEDO_MAX / mx);
  return c.convertLinearToSRGB();
}

function decode(b64) {
  const s = atob(b64);
  const out = new Uint8Array(s.length);
  for (let i = 0; i < s.length; i++) out[i] = s.charCodeAt(i);
  return out;
}
