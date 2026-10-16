// Merge + Full Sequence (feedback v5, slide 9; the owner, 16 Oct): "remove the all existing controls
// and section ... and implement as above 2 sections (provide as like below tabs) S1, S2". The page's
// own sections (the guide, the sequence file, the flow table, the custom merge, the step editor,
// the visual confirmation and both full-run cards) are hidden; in their place two tabs:
// - S1, Merge & Upload (s1.js): the project's files of each arm merged, saved, sent to the
//   controller, reviewed in 3D, and sent to Tune + Simulation;
// - S2, Full & Partial Sequence (s2.js): the control program's Page 02 (V53), in Simulation Mode or
//   on the real machine.

import { el } from './ctx.js';
import { hadIt } from './seq3d.js';
import { openS1, stopS1 } from './s1.js';
import { openS2, stopS2 } from './s2.js';

const $ = (id) => document.getElementById(id);
const TABS = [['s1', 'S1', 'Merge & Upload'], ['s2', 'S2', 'Full & Partial Sequence']];
let tab = 's1';
let pending = {};                                           // what S1 shows when it opens (a merge back from Tune)
try { tab = localStorage.getItem('liteSeqTab') === 's2' ? 's2' : 's1'; } catch (_) { /* no storage: S1 */ }

function build() {
  const view = $('view-sequence');
  if (!view || $('liteSeq')) return !!view;
  for (const n of view.children) n.hidden = true;            // the page's sections: gone from view
  const root = el('div', { id: 'liteSeq', class: 'lite-seq' },
    el('div', { class: 'section-head' },
      el('div', {},
        el('div', { class: 'eyebrow', text: '03 · Standalone Project Workflow' }),
        el('h2', {}, 'Merge + Full Sequence ', el('span', { class: 'mandatory-star', text: '*' })),
        el('p', { text: 'S1 merges and uploads the trajectory files of each arm. S2 builds the LEFT / RIGHT sequences and runs them, '
          + 'a single task or a dual-arm task, in simulation first, then on the real machine.' })),
      el('span', { class: 'badge violet', text: 'DUAL-ARM PROJECT FLOW' })),
    el('div', { class: 'lite-tabs', role: 'tablist', 'aria-label': 'Merge + Full Sequence' },
      TABS.map(([id, code, name]) => el('button', { id: `liteSeqTab_${id}`, class: 'lite-tab', type: 'button', role: 'tab',
        'aria-controls': `liteSeqPanel_${id}`, onclick: () => open(id) }, el('b', { text: code }), ` ${name}`))),
    TABS.map(([id]) => el('div', { id: `liteSeqPanel_${id}`, class: 'lite-seq-panel', role: 'tabpanel', 'aria-labelledby': `liteSeqTab_${id}` })));
  view.append(root);
  return true;
}

export function open(id, opts = {}) {
  if (!build()) return;
  tab = id === 's2' ? 's2' : 's1';
  if (tab === 's1') stopS2(); else stopS1();               // the other tab's playback stops with it
  try { localStorage.setItem('liteSeqTab', tab); } catch (_) { /* a convenience only */ }
  for (const [t] of TABS) {
    const on = t === tab;
    $(`liteSeqTab_${t}`).classList.toggle('active', on);
    $(`liteSeqTab_${t}`).setAttribute('aria-selected', String(on));
    $(`liteSeqPanel_${t}`).hidden = !on;
  }
  if (tab === 's1') openS1($('liteSeqPanel_s1'), opts);
  else openS2($('liteSeqPanel_s2'), opts);
}

export function installFullSeq() {
  const pageShow = window.showView;
  window.showView = function showView(v, btn) {
    pageShow(v, btn);
    if (v === 'sequence') { const o = pending; pending = {}; open(tab, o); return; }
    stopS1();                                              // nothing of S1 or S2 plays on another view
    stopS2();
    // Back in the region loop after S1 or S2 had the 3D view: Tune or Save & Review loads its file again.
    if (hadIt() && !projectRestoreInProgress && document.querySelector('.view.active')?.id === 'view-teach') {
      document.dispatchEvent(new CustomEvent('lite:viewer-closed'));
    }
  };
  document.addEventListener('lite:open-s1', (e) => { tab = 's1'; pending = e.detail || {}; window.showView('sequence'); });
  // The Trajectory Viewer closed over S1 or S2: the tab on screen loads its 3D again.
  document.addEventListener('lite:viewer-closed', () => { if (document.getElementById('view-sequence')?.classList.contains('active')) open(tab); });
}
