// The teaching screen's "Choose arm for this run": the task before and the task after, beside the
// current task's picture (the owner, 19 Oct). In each arm's card the current picture sits in the
// middle, larger; on the viewer's left the previous task's picture for that arm, on the right the
// next task's, each with its step's name, its pole height (mm) and its hand pose (the preset's
// name). The first task has nothing on the left, the last nothing on the right. The pictures are the
// page's own region pictures (resolveRegionPair, the same crops the cards show), the pole and hand
// the step's staged set-up as the task list has it (step_view: the set-up Prepare saved, else the
// existing recording's, else the region catalogue's).

import { el } from './ctx.js';
import { handPresetName } from './prepare.js';

const ARMS = ['left', 'right'];

function panel(which) {
  return el('div', { class: `lite-side lite-${which}` },
    el('div', { class: 'lite-side-tag' }),
    el('div', { class: 'lite-side-pic' }, el('img', { alt: '' })),
    el('div', { class: 'lite-side-name' }),
    el('div', { class: 'lite-side-meta' }));
}

// The three-picture row around the arm card's picture, made once and kept: the page's picture
// element (#leftRegionImage / #rightRegionImage) stays where the page's own code finds it.
function trio(card) {
  let row = card.querySelector('.lite-trio');
  if (row) return row;
  const thumb = card.querySelector('.choice-region-thumb');
  if (!thumb) return null;
  row = el('div', { class: 'lite-trio' });
  thumb.before(row);
  const mid = el('div', { class: 'lite-mid' }, thumb);          // the picture with its zoom row under it, as before
  const zoom = row.nextElementSibling;
  if (zoom && zoom.classList.contains('choice-zoom-row')) mid.append(zoom);
  row.append(panel('prev'), mid, panel('next'));
  card.classList.add('lite-has-trio');
  return row;
}

// The step's pole height and hand pose, as Prepare words them.
export function setupWords(s) {
  if (!s) return '';
  const parts = [];
  if (s.pole_mm != null) parts.push(`Pole ${s.pole_mm} mm`);
  if (s.hand_counts) parts.push(handPresetName(s.hand_counts));
  else if (s.hand_preset) parts.push(String(s.hand_preset));
  if (parts.length) return parts.join(' · ');
  return s.mechanism ? "Pre-recorded: ALIX's plan" : s.later ? 'Not in Release 1' : 'No pole or hand set yet';
}

// The arm's picture of task `t`, as the arm card shows it for the current task (setArmRegionImages).
async function picture(t, arm, img) {
  const slide = Number(t.slide);
  const want = `${slide}:${arm}`;
  img.dataset.want = want;
  const source = SLIDES[slide];
  const custom = CUSTOM_REGION_IMAGES[slide];
  const step = t.lite && t.lite[arm];
  const first = custom ? custom[arm] : source || (step && step.picture) || '';
  img.closest('.lite-side').classList.toggle('lite-no-pic', !first);
  if (first) img.src = first; else img.removeAttribute('src');
  if (!source) return;
  const pair = await resolveRegionPair(slide);
  if (img.dataset.want !== want) return;
  img.src = pair[arm] || source;
}

function fill(side, which, t, arm) {
  side.classList.toggle('lite-empty', !t);
  if (!t) {
    side.querySelector('.lite-side-tag').textContent = '';
    side.querySelector('.lite-side-name').textContent = '';
    side.querySelector('.lite-side-meta').textContent = '';
    side.querySelector('img').removeAttribute('src');
    return;
  }
  const s = t.lite ? t.lite[arm] : null;
  side.querySelector('.lite-side-tag').textContent = `${which === 'prev' ? 'Previous' : 'Next'} · ${t.label}`;
  side.querySelector('.lite-side-name').textContent = s ? s.display : (t[arm] && t[arm] !== '—' ? t[arm] : '');
  side.querySelector('.lite-side-meta').textContent = s ? setupWords(s) : t.lite ? `Not used by the ${arm.toUpperCase()} arm` : '';
  picture(t, arm, side.querySelector('img')).catch(() => {});
}

export function renderNeighbours() {
  const i = currentTask;
  const prev = i > 0 ? TASKS[i - 1] : null;
  const next = i < TASKS.length - 1 ? TASKS[i + 1] : null;
  for (const arm of ARMS) {
    const card = document.getElementById(arm === 'left' ? 'choiceLeft' : 'choiceRight');
    const row = card && trio(card);
    if (!row) continue;
    fill(row.querySelector('.lite-prev'), 'prev', prev, arm);
    fill(row.querySelector('.lite-next'), 'next', next, arm);
  }
}

export function installNeighbours() {
  document.addEventListener('lite:step', renderNeighbours);     // the task or the arm changed, a project opened
  renderNeighbours();
}
