// The page's projects, kept by the backend in one folder per project (handoff sections 5 and 20)
// instead of the browser's localStorage. The page reads and writes its project database through
// two functions, loadProjectDatabase() and saveProjectDatabase(db); here they work on a copy of
// the server's projects, and every change is sent to the server in order. Duplicate, delete,
// export and import go to the server too, because a project now owns files.

import { del, get, post, put, say } from './api.js';

let cache = {};                       // id -> the project record as the server has it
let chain = Promise.resolve();        // writes go out one after the other
let failed = false;
let nextFixture = null;               // the fixture of the next new project (New Project panel)

export function setNextFixture(f) { nextFixture = f; }

const clone = (v) => JSON.parse(JSON.stringify(v));

function queue(work) {
  chain = chain.then(work).catch((err) => {
    failed = true;
    window.setProjectSaveIndicator('unsaved', 'Save failed');
    window.toast(`Could not save the project: ${say(err)}`);
  });
  return chain;
}

export function flush() { return chain; }

export async function loadProjects() {
  const d = await get('/api/projects');
  cache = Object.fromEntries(d.projects.map((p) => [p.id, p]));
  try {
    if (d.last_id) localStorage.setItem(ALIX_LAST_PROJECT_KEY, d.last_id);
    else localStorage.removeItem(ALIX_LAST_PROJECT_KEY);
  } catch (_) { /* the page works without it */ }
  return d;
}

function remember(rec) { cache[rec.id] = rec; }

function saveFile(blob, name, description, accept) {
  if (window.showSaveFilePicker) {
    return window.showSaveFilePicker({ suggestedName: name, types: [{ description, accept }] })
      .then((h) => h.createWritable().then((w) => w.write(blob).then(() => w.close())));
  }
  const url = URL.createObjectURL(blob);
  const a = Object.assign(document.createElement('a'), { href: url, download: name });
  document.body.appendChild(a); a.click(); a.remove();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
  return Promise.resolve();
}

export function installProjects() {
  window.loadProjectDatabase = () => clone(cache);

  window.saveProjectDatabase = (db) => {
    for (const id of Object.keys(cache)) {
      if (!(id in db)) { delete cache[id]; queue(() => del(`/api/projects/${id}`)); }
    }
    for (const [id, rec] of Object.entries(db)) {
      if (JSON.stringify(cache[id]) === JSON.stringify(rec)) continue;
      const isNew = !(id in cache);
      cache[id] = clone(rec);
      const body = clone(rec);
      const url = isNew && nextFixture ? `/api/projects/${id}?fixture=${encodeURIComponent(nextFixture)}` : `/api/projects/${id}`;
      if (isNew) nextFixture = null;
      queue(() => put(url, body, { keepalive: true }).then((saved) => {
        if (cache[id]) cache[id].lite = saved.lite;
        if (failed) { failed = false; window.updateCurrentProjectHeader(); }
      }));
    }
    return true;
  };

  window.duplicateProject = async (id) => {
    try {
      await flush();
      remember(await post(`/api/projects/${id}/duplicate`));
      window.renderProjectManager();
      window.toast('Project duplicated, with its files.');
    } catch (err) { window.toast(`Could not duplicate the project: ${say(err)}`); }
  };

  window.deleteProject = async (id) => {
    const p = cache[id];
    if (!p) return;
    if (!confirm(`Delete project “${p.name}”?\n\nIts folder and files move to the trash folder on this computer. `
      + 'Save a Project File first if you need it on another computer.')) return;
    try {
      await flush();
      await del(`/api/projects/${id}`);
      delete cache[id];
      if (id === currentProjectId) { currentProjectId = null; window.updateCurrentProjectHeader(); }
      window.renderProjectManager();
      window.toast('Project deleted.');
    } catch (err) { window.toast(`Could not delete the project: ${say(err)}`); }
  };

  window.exportProjectFile = async (id = currentProjectId) => {
    if (id === currentProjectId) window.saveCurrentProjectNow('Project file export');
    const p = cache[id];
    if (!p) { window.toast('No project selected.'); return; }
    try {
      await flush();
      const blob = await get(`/api/projects/${id}/export`, { raw: true });
      const name = `${(p.name || 'ALIX_Project').replace(/[^A-Za-z0-9_-]+/g, '_')}.alixproj`;
      await saveFile(blob, name, 'ALIX Studio Project', { 'application/zip': ['.alixproj'] });
      window.toast(`Project file saved: ${p.name}`);
    } catch (err) {
      if (err?.name !== 'AbortError') window.toast(`Could not save the project file: ${say(err)}`);
    }
  };

  window.importProjectFile = async (event) => {
    const file = event.target.files?.[0];
    if (!file) return;
    try {
      const data = await file.arrayBuffer();
      remember(await post(`/api/projects/import?name=${encodeURIComponent(file.name)}`, data));
      window.renderProjectManager();
      window.toast('Project imported. Open it from the Project Library.');
    } catch (err) {
      window.toast(`Invalid ALIX project file: ${say(err)}`);
    } finally {
      event.target.value = '';
    }
  };
}
