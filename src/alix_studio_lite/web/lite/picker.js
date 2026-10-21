// Lists of global points the operator can find (the owner, 16 Oct: "the global points dropdown menu is
// not friendly, it is very hard for the user to select the point ... All the list, where such issue
// might be prone"). In place of the page's dropdown, a field that opens a searchable list: type to
// narrow it (every word typed must be in the name), the arrow keys and Enter, or a click; the page's
// own choices first, then the controller's points as one group with their count and each point's
// joints. The page's <select> stays, hidden, and is kept in step (its value and its change event), so
// the page's own code reads and sets it as before. Used by Setup & Safety's start poses, Prepare's
// Region start pose, Redo Preparation and Real Test's Move to start (pointControls, points.js), and
// by S2's Global Point field (choose()).

import { el } from './ctx.js';

let open = null;                                    // the list open now: {close}

const words = (q) => String(q || '').toLowerCase().split(/\s+/).filter(Boolean);
const fits = (text, ws) => { const t = String(text).toLowerCase(); return ws.every(w => t.includes(w)); };

// Opens the list under `anchor`: `groups` [{label, rows: [{value, text, detail, search, disabled, note}]}]
// (the search looks in a row's text, its `search` words and its group's label), `current`
// the value chosen now; `pick(value)` when one is chosen. A disabled row is shown greyed with its note (why
// it cannot be chosen) and is never chosen. `files`: a list of files, each shown as an entry of its own (the owner,
// 21 Oct: "show the files as clearly separated entries"; S1's files, S2's programs).
export function choose(anchor, groups, current, pick, { placeholder = 'Search the global points', files = false } = {}) {
  if (open) open.close();
  const all = groups.flatMap(g => g.rows);
  const search = el('input', { type: 'search', class: 'lite-picker-search', placeholder, 'aria-label': placeholder,
    autocomplete: 'off', role: 'combobox', 'aria-expanded': 'true', 'aria-autocomplete': 'list' });
  const list = el('div', { class: 'lite-picker-list', role: 'listbox', id: `litePickerList${Date.now()}` });
  const count = el('div', { class: 'lite-picker-count' });
  const panel = el('div', { class: `lite-picker-panel${files ? ' lite-picker-files' : ''}`, role: 'dialog',
    'aria-label': files ? 'Choose a file' : 'Choose a point' }, search, count, list);
  search.setAttribute('aria-controls', list.id);
  document.body.append(panel);
  let shown = [], active = -1;

  const place = () => {
    const r = anchor.getBoundingClientRect();
    const w = Math.max(r.width, 340), room = window.innerHeight - r.bottom - 12;
    panel.style.width = `${Math.min(w, window.innerWidth - 16)}px`;
    panel.style.left = `${Math.max(8, Math.min(r.left, window.innerWidth - w - 8))}px`;
    if (room < 260 && r.top > room) {                // no room below: above the field
      panel.style.top = '';
      panel.style.bottom = `${window.innerHeight - r.top + 4}px`;
      list.style.maxHeight = `${Math.max(140, Math.min(360, r.top - 90))}px`;
    } else {
      panel.style.bottom = '';
      panel.style.top = `${r.bottom + 4}px`;
      list.style.maxHeight = `${Math.max(140, Math.min(360, room - 70))}px`;
    }
  };
  const mark = (i) => {
    active = i;
    list.querySelectorAll('.lite-picker-row:not(.disabled)').forEach((n, k) => {
      n.classList.toggle('active', k === i);
      n.setAttribute('aria-selected', String(k === i));
    });
    const n = list.querySelectorAll('.lite-picker-row:not(.disabled)')[i];
    if (!n) { search.removeAttribute('aria-activedescendant'); return; }
    search.setAttribute('aria-activedescendant', n.id);
    if (n.offsetTop < list.scrollTop) list.scrollTop = n.offsetTop;              // the list scrolls, never the page
    else if (n.offsetTop + n.offsetHeight > list.scrollTop + list.clientHeight) list.scrollTop = n.offsetTop + n.offsetHeight - list.clientHeight;
  };
  const paint = () => {
    const ws = words(search.value);
    shown = [];
    const kids = [];
    for (const g of groups) {
      const rows = g.rows.filter(r => fits(`${r.text} ${r.search || ''} ${g.label || ''}`, ws));
      if (!rows.length) continue;
      if (g.label) kids.push(el('div', { class: 'lite-picker-group', role: 'presentation', text: g.label }));
      for (const r of rows) {
        if (r.disabled) {
          kids.push(el('div', { class: 'lite-picker-row disabled', role: 'option', 'aria-disabled': 'true' },
            el('b', { text: r.text }), r.detail ? el('small', { text: r.detail }) : null, r.note ? el('small', { class: 'lite-picker-note', text: r.note }) : null));
          continue;
        }
        const k = shown.length;
        shown.push(r);
        kids.push(el('div', { class: `lite-picker-row${r.value === current ? ' current' : ''}`, role: 'option', id: `${list.id}_${k}`,
          onmousemove: () => { if (active !== k) mark(k); }, onclick: () => done(r.value) },
        el('b', { text: r.text }), r.detail ? el('small', { text: r.detail }) : null, r.note ? el('small', { class: 'lite-picker-note', text: r.note }) : null));
      }
    }
    list.replaceChildren(...(kids.length ? kids : [el('div', { class: 'lite-picker-none', text: all.length ? 'No point matches the search.' : 'Nothing to choose yet.' })]));
    const can = all.filter(r => !r.disabled).length;
    count.textContent = (ws.length ? `${shown.length} of ${can} shown` : `${can} to choose from`)
      + (can < all.length ? ` (${all.length - can} greyed: they cannot be chosen now)` : '');
    const at = shown.findIndex(r => r.value === current);
    mark(ws.length || at < 0 ? (shown.length ? 0 : -1) : at);     // the one chosen now, else the first: Enter takes it
  };
  const close = () => {
    panel.remove();
    document.removeEventListener('pointerdown', outside, true);
    window.removeEventListener('resize', place);
    window.removeEventListener('scroll', onScroll, true);
    anchor.setAttribute('aria-expanded', 'false');
    if (open && open.panel === panel) open = null;
  };
  const done = (value) => { close(); anchor.focus(); pick(value); };
  const outside = (e) => { if (!panel.contains(e.target) && !anchor.contains(e.target)) close(); };
  const onScroll = (e) => { if (!panel.contains(e.target)) place(); };
  search.addEventListener('input', paint);
  search.addEventListener('keydown', (e) => {
    if (e.key === 'ArrowDown') { e.preventDefault(); if (shown.length) mark(Math.min(shown.length - 1, active + 1)); }
    else if (e.key === 'ArrowUp') { e.preventDefault(); if (shown.length) mark(Math.max(0, active - 1)); }
    else if (e.key === 'PageDown') { e.preventDefault(); if (shown.length) mark(Math.min(shown.length - 1, active + 8)); }
    else if (e.key === 'PageUp') { e.preventDefault(); if (shown.length) mark(Math.max(0, active - 8)); }
    else if (e.key === 'Enter') { e.preventDefault(); if (shown[active]) done(shown[active].value); }
    else if (e.key === 'Escape') { e.preventDefault(); e.stopPropagation(); close(); anchor.focus(); }
    else if (e.key === 'Tab') close();
  });
  document.addEventListener('pointerdown', outside, true);
  window.addEventListener('resize', place);
  window.addEventListener('scroll', onScroll, true);
  anchor.setAttribute('aria-expanded', 'true');
  place();
  paint();
  search.focus();
  open = { panel, close };
  return open;
}

// The rows of a <select>: its optgroups as groups, its own options first.
function groupsOf(sel) {
  const groups = [], loose = { label: '', rows: [] };
  const row = (o) => ({ value: o.value, text: o.textContent, detail: o.dataset.detail || '' });
  for (const c of sel.children) {
    if (c.tagName === 'OPTGROUP') groups.push({ label: c.label, rows: [...c.children].filter(o => !o.disabled).map(row) });
    else if (c.tagName === 'OPTION' && !c.disabled) loose.rows.push(row(c));
  }
  return loose.rows.length ? [loose, ...groups] : groups;
}

// A <select> shown as a field that opens the list (made once per select). Returns the field.
export function pointPicker(sel) {
  if (!sel) return null;
  if (sel._litePicker) return sel._litePicker;
  const value = el('span', { class: 'lite-picker-value' });
  const btn = el('button', { type: 'button', class: 'lite-picker-btn', 'aria-haspopup': 'listbox', 'aria-expanded': 'false',
    title: 'Choose: type to search, the arrow keys and Enter, or a click' }, value);
  const label = sel.id && document.querySelector(`label[for="${sel.id}"]`);
  btn.setAttribute('aria-label', sel.getAttribute('aria-label') || (label && label.textContent) || 'Choose a point');
  const refresh = () => {
    const o = sel.selectedOptions[0];
    value.textContent = o ? o.textContent : 'Choose…';
    btn.disabled = sel.disabled;
  };
  btn.addEventListener('click', () => {
    if (open && open.anchor === btn) { open.close(); return; }
    const o = choose(btn, groupsOf(sel), sel.value, (v) => {
      if (sel.value !== v) { sel.value = v; sel.dispatchEvent(new Event('change', { bubbles: true })); }
      refresh();
    }, { placeholder: `Search ${sel.querySelectorAll('option').length} choices` });
    o.anchor = btn;
  });
  btn.addEventListener('keydown', (e) => {                // the arrow keys open it, as a dropdown does
    if (e.key === 'ArrowDown' || e.key === 'ArrowUp') { e.preventDefault(); btn.click(); }
  });
  sel.before(btn);
  sel.classList.add('lite-picker-native');
  sel.setAttribute('tabindex', '-1');
  sel.addEventListener('change', refresh);
  new MutationObserver(refresh).observe(sel, { childList: true, subtree: true, attributes: true, attributeFilter: ['disabled'] });
  // the page sets the value itself too (a project restored, Redo copying the arm's list): the field follows
  const proto = Object.getOwnPropertyDescriptor(HTMLSelectElement.prototype, 'value');
  Object.defineProperty(sel, 'value', { configurable: true, get() { return proto.get.call(this); }, set(v) { proto.set.call(this, v); refresh(); } });
  const idx = Object.getOwnPropertyDescriptor(HTMLSelectElement.prototype, 'selectedIndex');
  Object.defineProperty(sel, 'selectedIndex', { configurable: true, get() { return idx.get.call(this); }, set(v) { idx.set.call(this, v); refresh(); } });
  refresh();
  sel._litePicker = btn;
  return btn;
}
