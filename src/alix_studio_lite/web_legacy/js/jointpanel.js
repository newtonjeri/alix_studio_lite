// The joint panel: a slider and a number per joint, in degrees or mm, grouped LEFT arm, RIGHT arm,
// poles, hands and the rest (hands and the rest start folded). It drives a robot from robot3d.js;
// each change is reported through onChange(name, value) once the robot shows it, so the caller can
// redraw then.
import { esc, h } from './core.js';

const OPEN = new Set(['left_arm', 'right_arm', 'poles']);
const STEP = { deg: 0.1, mm: 0.5 };
const SIDE = { L: 'left', R: 'right' };

export function mountJointPanel(el, robot, { onChange } = {}) {
  const spec = new Map(robot.joints.map(j => [j.name, j]));
  const rows = new Map();                       // joint name -> {range, num}

  const show = (name, value) => {
    const r = rows.get(name);
    if (!r || !Number.isFinite(value)) return;
    r.range.value = String(value);
    const t = value.toFixed(1);
    r.num.value = t === '-0.0' ? '0.0' : t;
  };
  const set = (name, value) => {
    const j = spec.get(name);
    const v = Number.isFinite(value) ? Math.min(j.upper, Math.max(j.lower, value)) : null;
    show(name, v ?? robot.getJoints()[name]);       // an empty or bad entry snaps back
    if (v === null) return;
    Promise.resolve(robot.setJoint(name, v)).then((got) => { if (got != null && onChange) onChange(name, got); });
  };
  // Runs a robot call that moves many joints, then reports the ones that moved.
  const many = async (call) => {
    const before = robot.getJoints();
    await call();
    for (const [name, v] of Object.entries(robot.getJoints())) {
      show(name, v);
      if (v !== before[name] && onChange) onChange(name, v);
    }
  };

  const root = h('div.jp', {}, [
    h('div.jp-bar', {}, [
      h('button.btn', { type: 'button', title: 'Every joint to the home pose', onclick: () => many(() => robot.home()) }, 'Home'),
      h('button.btn', { type: 'button', title: 'Every arm joint (J1 to J7, both arms) to 0°',
        onclick: () => many(() => robot.setJoints(Object.fromEntries(['L', 'R'].flatMap(s => [1, 2, 3, 4, 5, 6, 7].map(i => [`${s}_joint${i}`, 0]))))) }, 'Zero arms'),
    ]),
  ]);

  for (const g of robot.groups) {
    const names = g.joints.filter(n => spec.has(n) && !spec.get(n).mimic);
    if (!names.length) continue;
    const grid = h('div.jp-rows');
    for (const name of names) {
      const j = spec.get(name);
      const label = shortLabel(name);
      const what = `${g.label}, ${label}`;
      // limits in degrees come out as -177.99999999999997: a range input snaps to min + n x step,
      // so an unrounded min would turn 0 into -3e-14
      const [lo, hi] = [j.lower, j.upper].map(v => Number(v.toFixed(3)));
      const range = h('input', { type: 'range', min: lo, max: hi, step: STEP[j.unit], 'aria-label': what });
      const num = h('input.mono', { type: 'number', min: lo, max: hi, step: STEP[j.unit], 'aria-label': `${what}, ${j.unit}` });
      range.addEventListener('input', () => set(name, Number(range.value)));
      num.addEventListener('change', () => set(name, num.value === '' ? NaN : Number(num.value)));
      const arm = /^[LR]_joint(\d)$/.exec(name);
      const side = /_sliding_plate_joint$/.test(name) ? SIDE[name[0]] : '';
      grid.append(h('div.jp-row' + (arm ? `.j${arm[1]}` : ''), {}, [
        h('span.jp-label' + (side ? `.jp-side.${side}` : ''), { title: name }, label),
        range, num, h('span.jp-unit', {}, j.unit === 'deg' ? '°' : 'mm'),
      ]));
      rows.set(name, { range, num });
    }
    const title = g.arm ? `<span class="jp-side ${g.arm}">${esc(g.label.split(' ')[0])}</span> ${esc(g.label.split(' ').slice(1).join(' '))}`
      : esc(g.label);
    root.append(h(`details.jp-group${g.arm ? '.' + g.arm : ''}`, { open: OPEN.has(g.key) }, [
      h('summary', { html: `${title} <span class="jp-count">${names.length}</span>` }), grid,
    ]));
  }
  el.append(root);
  update();

  // Shows joint values (degrees or mm) set elsewhere, for example by a playback; moves nothing.
  // With no map, shows the robot's current values.
  function update(map) {
    for (const [name, v] of Object.entries(map || robot.getJoints())) show(name, Number(v));
  }
  function dispose() {
    root.remove();
    rows.clear();
  }
  return { update, dispose };
}

// J1..J7 for arm joints, the side for a pole, the finger for a hand joint; else the joint's name.
function shortLabel(name) {
  let m = /^[LR]_joint(\d)$/.exec(name);
  if (m) return `J${m[1]}`;
  if (/_sliding_plate_joint$/.test(name)) return name[0] === 'L' ? 'LEFT' : 'RIGHT';
  m = /_(thumb|index|middle|ring|little)_(\d)_joint$/.exec(name);
  if (m) return cap(m[1]) + (m[1] === 'thumb' ? ` ${m[2]}` : '');
  return cap(name.replace(/_joint$/, '').replace(/^glove_wearing_unit_arm$/, 'glove arm').replace(/^UGV_/, 'UGV ').replace(/_/g, ' '));
}

function cap(s) {
  return s.charAt(0).toUpperCase() + s.slice(1);
}
