// Home: every region of commode C, LEFT and RIGHT side by side, in the deck's order, grouped by
// the toilet's state. One primary action: teach the next region that has no recording.
import { api, esc, h, statusChip, chip, PIPE_ORDER, ARM, unitCode, icon } from '../core.js';
import { showError } from '../ui.js';

const PHASE_ORDER = [
  { id: 'lid_closed', title: 'Lid closed' },
  { op: [5], title: 'Open the lid and the seat' },
  { op: [6], title: 'Close the seat only' },
  { id: 'lid_open', title: 'Lid open, seat down' },
  { op: [10], title: 'Open the seat' },
  { id: 'seat_open', title: 'Lid and seat up' },
  { op: [20, 21], title: 'Glove change (later)' },
  { op: [22], title: 'Close the seat and the lid' },
];

let filter = { arm: 'both', all: false };

export async function render(main) {
  let data;
  try { data = await api('catalogue'); } catch (e) { showError(e); return; }
  const units = data.units;
  const first = units.filter(u => u.first_release);
  const next = first.filter(u => u.status === 'not_taught').sort((a, b) => a.step - b.step || (a.arm === 'left' ? -1 : 1))[0];

  const page = h('div.page');
  const head = h('div.page-head');
  const deck = !units.some(u => u.provisional);
  head.innerHTML = `<div class="titles"><h2>${esc(data.project?.name || 'Regions')}</h2>
      <p>${deck ? `${first.length} regions for the first release: ${first.filter(u => u.kind === 'cleaning').length} cleaning regions and
      ${first.filter(u => u.kind === 'lid_seat').length} lid and seat moves.`
        : `${first.length} provisional regions, taken from ALIX's regions file for this fixture (there is no regions deck for it yet).`}
      Open a region to teach, optimise, check and run it. To change the fixture, click its name under the title at the top left.</p></div>`;
  const btnNext = h('button.btn.primary.big', { onclick: () => { if (next) location.hash = `#/teach/${next.key}`; } },
    next ? [`Teach next region: ${unitCode(next)} (${ARM[next.arm]})`] : ['Every region has a recording']);
  btnNext.disabled = !next;
  head.append(h('div.btn-row', {}, [btnNext]));
  page.append(head);

  // progress summary
  const strip = h('div.summary-strip');
  const labels = { not_taught: 'Not taught', taught: 'Taught', optimised: 'Optimised', check_ok: '3D check OK',
                   uploaded: 'On the robot', dryrun_ok: 'Hardware sim OK', real_ok: 'Tested on robot' };
  for (const s of PIPE_ORDER) {
    strip.append(h('div.sum-tile', { html: `<div class="n">${data.counts[s] || 0}</div><div class="l">${statusChip(s)}</div>`,
      title: labels[s] }));
  }
  page.append(strip);

  // filters
  const bar = h('div.row');
  const seg = (name, opts) => {
    const s = h('div.segmented', { role: 'radiogroup' });
    for (const [v, label] of opts) {
      const b = h('button', { role: 'radio', 'aria-checked': String(filter[name] === v),
        onclick: () => { filter[name] = v; render(main.replaceChildren() || main); } }, [label]);
      if (filter[name] === v) b.classList.add('on');
      s.append(b);
    }
    return s;
  };
  bar.append(seg('arm', [['both', 'Both arms'], ['left', 'LEFT arm'], ['right', 'RIGHT arm']]));
  bar.append(seg('all', [[false, 'First release'], [true, 'Show later tasks too']]));
  bar.append(h('span.muted.small', { html: 'Glove and drain-brush tasks are planned for later.' }));
  page.append(bar);

  // the board
  const board = h('div.board');
  const showL = filter.arm !== 'right', showR = filter.arm !== 'left';
  board.style.gridTemplateColumns = `56px ${showL ? '1fr' : ''} ${showR ? '1fr' : ''}`;
  board.append(h('div'));
  if (showL) board.append(h('div.col-head.left', { html: `LEFT arm <span class="muted small">${first.filter(u => u.arm === 'left').length} regions</span>` }));
  if (showR) board.append(h('div.col-head.right', { html: `RIGHT arm <span class="muted small">${first.filter(u => u.arm === 'right').length} regions</span>` }));

  const byStep = new Map();
  for (const u of units) {
    if (!filter.all && !u.first_release) continue;
    if (!byStep.has(u.step)) byStep.set(u.step, { left: [], right: [] });
    byStep.get(u.step)[u.arm].push(u);
  }
  const phaseOf = (u) => u.phase;
  const order = deck ? PHASE_ORDER : [{ id: 'lid_closed', title: 'Lid closed' }, { id: 'lid_open', title: 'Lid open' },
    { id: 'seat_open', title: 'Lid and seat up' }, { id: null, title: 'Any state' }];
  for (const ph of order) {
    const steps = [...byStep.keys()].sort((a, b) => a - b).filter(st => {
      const us = [...byStep.get(st).left, ...byStep.get(st).right];
      if (ph.op) return ph.op.includes(st);
      if (!deck) return us.some(u => (phaseOf(u) || null) === ph.id);
      return us.some(u => phaseOf(u) === ph.id) && !PHASE_ORDER.some(p => p.op && p.op.includes(st));
    });
    if (!steps.length) continue;
    const q = data.phases.find(p => p.id === ph.id);
    board.append(h('div.phase', { html: `<h4>${esc(ph.title)}</h4>${ph.op ? chip('info', '↻', 'lid and seat move, not cleaning') : q ? `<span class="muted small">${esc(q.question)}</span>` : ''}` }));
    for (const st of steps) {
      const g = byStep.get(st);
      const rows = Math.max(showL ? g.left.length : 0, showR ? g.right.length : 0);
      for (let i = 0; i < rows; i++) {
        board.append(h('div.step-no', { text: i === 0 ? `Step ${st}` : '' }));
        if (showL) board.append(cell(g.left[i]));
        if (showR) board.append(cell(g.right[i]));
      }
    }
  }
  page.append(board);
  main.append(page);
}

function cell(u) {
  if (!u) return h('div.unit-cell.none');
  const later = !u.first_release;
  const done = PIPE_ORDER.indexOf(u.status);
  const el = h(`button.unit-cell.${u.arm}${later ? '.later' : ''}${u.kind === 'lid_seat' ? '.op' : ''}`, {
    title: later ? 'Planned for a later release.' : `Open ${u.code} ${u.name}`,
    onclick: () => { if (!later) location.hash = `#/${nextTab(u)}/${u.key}`; },
  });
  el.innerHTML = `
    ${u.picture || u.slide_picture ? `<img class="thumb" src="/deck/${esc(u.picture || u.slide_picture)}" alt="" loading="lazy">`
      : `<div class="thumb thumb-none" aria-hidden="true">${icon('region', 28)}</div>`}
    <div>
      <div class="name">${esc(titleCase(u.name))}</div>
      <div class="code">${esc(unitCode(u))}</div>
      <div class="meta">${later ? chip('todo', '·', 'Later') : statusChip(u.status)}
        ${later ? '' : `<span class="pipe" title="Progress: ${done} of 6 steps">${PIPE_ORDER.slice(1).map((_, i) => `<i class="${i < done ? 'done' : ''}"></i>`).join('')}</span>`}
        ${u.flags.includes('legacy_v6') && u.status === 'not_taught' ? chip('standin', '·', 'Older program exists', 'An older program (the V6 package) covers this area. Teach it again for Lite.') : ''}</div>
    </div>`;
  return el;
}

// the tab a region opens on: its first unfinished step
function nextTab(u) {
  const i = PIPE_ORDER.indexOf(u.status);
  return i <= 0 ? 'teach' : i === 1 ? 'optimise' : i === 2 ? 'view3d' : 'run';
}

export function titleCase(s) {
  return String(s).toLowerCase().replace(/\b([a-z])/g, (m) => m.toUpperCase())
    .replace(/\bPart (\d)/gi, 'part $1').replace(/\(Master\)/i, '(master)').replace(/\(Support\)/i, '(support)');
}
