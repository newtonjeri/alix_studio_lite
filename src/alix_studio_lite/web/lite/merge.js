// The custom trajectory merge, one arm at a time (Merge + Full Sequence's "Custom Trajectory Merge &
// Single-Arm Test"; feedback v2, slide 10). The page only listed the files chosen; with Lite the files
// are merged on the server (server/merges.py): in the order chosen, joined where one ends away from
// where the next begins, with the one pole height and hand pose the merged program runs with, its
// limits checked. Kept in the project (merged/<arm>/) under its own name, it can be renamed,
// downloaded, played in 3D (the run section's simulation), sent to the controller and run there on
// its arm alone.

import { call, get, post } from './api.js';
import { REAL_CHECKS, confirmDialog, ctx, el, jobLine, robotCall, stageDialog, tell, watchJob } from './ctx.js';
import { liteMode } from './status.js';
import { saveBlobAs } from './tune.js';

const $ = (id) => document.getElementById(id);
const ARM = (s) => s.toUpperCase();
const last = { left: null, right: null };          // each arm's latest merge (its record)
const url = (side, file, verb = '') => `/api/projects/${ctx.pid}/merges/${side}/${file}${verb ? `/${verb}` : ''}`;

function status(text, cls = '') {
  const s = $('mergeStatus');
  if (s) { s.textContent = text; s.className = `statusbar ${cls}`.trim(); }
}

async function load() {
  if (!ctx.pid) return;
  try {
    const d = await get(`/api/projects/${ctx.pid}/merges`);
    for (const side of ['left', 'right']) { last[side] = d[side][d[side].length - 1] || null; paint(side); }
  } catch (_) { /* the cards keep the page's text */ }
}

function paint(side) {
  const box = $(`${side}MergeSummary`);
  if (!box) return;
  const m = last[side];
  box.parentElement?.querySelector(':scope > .lite-merge-tools')?.remove();
  if (!m) { box.textContent = `No custom ${ARM(side)} merge prepared.`; return; }
  box.replaceChildren(el('b', { text: m.name }),
    ` · ${m.parts.map((x) => x.name).join(' → ')} · ${m.lines.toLocaleString()} lines, ${m.seconds.toFixed(2)} s`
      + (m.joins.length ? ` · ${m.joins.length} join${m.joins.length === 1 ? '' : 's'} (${m.joins.map((j) => `${j.seconds.toFixed(2)} s`).join(', ')})` : '')
      + ` · staged first: ${m.stage.text} · `,
    el('span', { class: m.limits_ok ? 'lite-ok' : 'lite-warn', text: m.limits_ok ? 'passes L1–L4'
      : `${m.limits_text}: shown, not enforced (sending it asks once)` }));
  const name = el('input', { type: 'text', value: m.name, 'aria-label': `${ARM(side)} merge file name`, maxlength: '120' });
  box.after(el('div', { class: 'lite-merge-tools' },
    el('div', { class: 'inline' }, name, el('button', { class: 'btn light', type: 'button', text: 'Rename', onclick: () => rename(side, name.value) })),
    el('div', { class: 'cta-row' },
      el('button', { class: 'btn outline', type: 'button', text: '⇩ Download', onclick: () => download(side) }),
      el('button', { class: 'btn', type: 'button', text: '⇧ Upload to controller', onclick: () => upload(side) }))));
}

async function rename(side, name) {
  const m = last[side];
  if (!m) return;
  try {
    last[side] = await post(url(side, m.file, 'name'), { name });
    paint(side);
    window.toast(`The ${ARM(side)} merge is now ${last[side].name}.`);
  } catch (err) { tell(err); }
}

async function download(side) {
  const m = last[side];
  if (!m) return;
  try {
    const blob = await call('GET', url(side, m.file, 'download'), undefined, { raw: true });
    const saved = await saveBlobAs(blob, m.name, [{ description: 'Trajectory', accept: { 'text/plain': ['.txt'] } }]);
    if (saved) window.toast(`Saved ${saved}.`);
  } catch (err) { tell(err); }
}

function programId(side) {
  const inp = $(`${side}MergeProgramId`), pid = Number(inp?.value);
  if (!(Number.isInteger(pid) && pid >= 1 && pid <= 100)) { inp?.focus(); throw new Error(`Enter the ${ARM(side)} Program ID (1 to 100) first.`); }
  return pid;
}

async function sendTo(side, pid) {
  const m = last[side];
  const job = await robotCall(url(side, m.file, 'upload'), { program_id: pid }, { what: `Sending ${m.name}` });
  if (job && job.id) {
    const done = await watchJob(job, (j) => status(`${ARM(side)} merge: ${jobLine(j)}`));
    if (done.state !== 'done') throw Object.assign(new Error(done.error || 'The upload did not finish.'), { hint: done.hint });
  }
}

async function upload(side) {
  if (!last[side]) return;
  try {
    const pid = programId(side);
    await sendTo(side, pid);
    status(`${last[side].name} is on the ${ARM(side)} controller as program ${pid} (name and size read back).`, 'ok');
  } catch (err) { status(err.message, 'warn'); tell(err); }
}

export function installMerge() {
  window.confirmCustomMerge = async function confirmCustomMerge(action = 'merge') {
    const files = pendingMergeFiles.slice(), side = mergeModalSide;
    if (!files.length) { window.toast('Choose at least one trajectory file to merge.'); return; }
    if (!ctx.pid) { window.toast('Open a project first.'); return; }
    const said = await stageDialog({ title: `${ARM(side)} merge: its pole and hand`, arm: side, setup: null,
      body: `The merged program runs with one pole height and hand pose, for all of its ${files.length} file${files.length === 1 ? '' : 's'}: say them. They are staged before it runs.` });
    if (!said) return;
    try {
      const n = ((await get(`/api/projects/${ctx.pid}/merges`))[side] || []).length + 1;
      last[side] = await post(`/api/projects/${ctx.pid}/merges/${side}`, { name: `${ARM(side)[0]}_merge_${n}.txt`,
        files: await Promise.all(files.map(async (f) => ({ name: f.name, content: await f.text() }))), pole_mm: said.pole_mm, hand: said.hand });
    } catch (err) { tell(err); return; }
    mergeSelections[side] = files.map((f, i) => ({ order: i + 1, name: f.name, size: f.size, type: f.type || 'text/plain' }));
    window.safeSaveTaskRecords?.();
    window.closeMergeModal();
    paint(side);
    const m = last[side];
    status(`${ARM(side)} merge ${m.name}: ${m.parts.length} files, ${m.joins.length} join(s), ${m.seconds.toFixed(2)} s${m.limits_ok ? ', within the limits.' : `; it breaks ${m.limits_text}: shown, not enforced (the hardware decides; sending it asks once).`}`, m.limits_ok ? 'ok' : 'warn');
    if (action === 'simulation') window.runMergedArmSimulation(side);
    if (action === 'real') window.runMergedArmReal(side);
  };

  window.runMergedArmSimulation = function runMergedArmSimulation(side) {
    const m = last[side];
    if (!m) { window.toast(`Choose and merge ${ARM(side)} trajectory files first.`); return; }
    document.dispatchEvent(new CustomEvent('lite:run-merge', { detail: { side, file: m.file, name: m.name } }));
  };

  window.runMergedArmReal = async function runMergedArmReal(side) {
    const m = last[side];
    if (!m) { window.toast(`Choose and merge ${ARM(side)} trajectory files first.`); return; }
    try {
      const pid = programId(side);
      const mode = await liteMode();
      let extra = {};
      if (mode === 'real') {
        const yes = await confirmDialog({ title: `Real robot: the ${ARM(side)} arm will move`, ok: `Yes, run it on the ${ARM(side)} arm`, list: REAL_CHECKS,
          body: `${m.name} runs on the ${ARM(side)} arm alone (program ${pid}), its pole and hand staged first. The other arm stays stopped.` });
        if (!yes) { window.toast('Cancelled: nothing was sent to the robot.'); return; }
        extra = { confirmed: true };
      }
      await sendTo(side, pid);
      const job = await robotCall(url(side, m.file, 'run'), { program_id: pid, ...extra }, { what: `Running ${m.name}` });
      const done = await watchJob(job, (j) => status(`${ARM(side)} merge: ${jobLine(j)}`));
      status(done.state === 'done' ? `${m.name} ran on the ${ARM(side)} arm${mode === 'real' ? '' : ' (Hardware sim: nothing moved)'}.`
        : `Stopped: ${done.error || done.state}`, done.state === 'done' ? 'ok' : 'warn');
    } catch (err) { status(err.message, 'warn'); tell(err); }
  };

  const pageShow = window.showView;
  window.showView = function showView(v, btn) {
    pageShow(v, btn);
    if (v === 'sequence') load();
  };
}
