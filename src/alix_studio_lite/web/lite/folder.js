// V32's Save Project Folder, with the real files (handoff: "the production Python backend should
// write each real trajectory file into trajectories/left or trajectories/right"). Lite's server
// already keeps each project as that folder. The page still writes its layout, manifests and README
// into the folder the operator picks; Lite then copies in every file the server keeps for the
// project: the trajectories as recorded, loaded and kept (each with its record of arm and stage),
// the transitions, the regions' settings and results; and the project's audit trail from the
// server's logs. Chrome / Edge only (File System Access); elsewhere the page saves the .alixproj
// file instead, which is the same folder zipped.

import { call, get } from './api.js';
import { tell } from './ctx.js';

async function writeFile(dir, name, data) {
  const h = await dir.getFileHandle(name, { create: true });
  const w = await h.createWritable();
  await w.write(data);
  await w.close();
}

const README_NOTE = `With ALIX Studio Lite:
Every file Lite keeps for this project is in this folder, as Lite's server holds it:
- trajectories/<arm>/<step>/raw          the recordings (taught, existing, loaded), never edited,
                                         each with its .source.json (arm, pole and hand, name)
- trajectories/<arm>/<step>/versions     the kept tuned files (the ones uploaded), with their records
- trajectories/<arm>/<step>/unit.json    the step's settings and its active recording
- trajectories/<arm>/<step>/results.json uploads, runs and verdicts, tied to each file's sha256
- trajectories/transitions/<arm>/<step>  the transition trajectories, with their records
- logs/audit_log.json, audit_log.csv     the project's audit trail from Lite's logs
`;

export function installFolder() {
  const page = window.saveProjectFolder;
  window.saveProjectFolder = async function saveProjectFolder(id = currentProjectId) {
    const picker = globalThis.showDirectoryPicker;        // the browser's folder picker (not a page function):
    if (!id || typeof picker !== 'function') return page(id);
    let parent = null;                                     // the folder the operator picks for the page
    globalThis.showDirectoryPicker = async (opts) => (parent = await picker.call(globalThis, opts));
    try { await page(id); } finally { globalThis.showDirectoryPicker = picker; }
    if (!parent) return;                                   // cancelled, or the page could not start
    const project = loadProjectDatabase()[id];
    if (!project) return;
    const safe = (project.name || 'ALIX_Project').replace(/[<>:"/\\|?*\x00-\x1F]+/g, '_').trim() || 'ALIX_Project';
    try {
      const root = await parent.getDirectoryHandle(safe, { create: true });
      const { files } = await get(`/api/projects/${id}/folder`);
      window.toast(`Copying ${files.length} project files from Lite…`);
      let bytes = 0;
      for (const f of files) {
        const parts = f.path.split('/');
        const name = parts.pop();
        const dir = await window.fsEnsurePath(root, parts);
        await writeFile(dir, name, await call('GET', `/api/projects/${id}/folder/file?path=${encodeURIComponent(f.path)}`, undefined, { raw: true }));
        bytes += f.bytes;
      }
      const logs = await get('/api/logs?from=0');
      const rows = logs.events.filter(e => e.projectId === id);
      const logsDir = await window.fsEnsurePath(root, ['logs']);
      await window.fsWriteJson(logsDir, 'audit_log.json', { schema: 'asl_activity_log_v1', exportedAt: new Date().toISOString(),
        projectId: id, projectName: project.name, source: "Lite's logs", eventCount: rows.length, events: rows });
      await writeFile(logsDir, 'audit_log.csv', window.auditEventsToCSV(rows));
      const manifest = { schema: 'alix_project_folder_v1', generated_at: new Date().toISOString(), project_id: id, project_name: project.name,
        resume: window.currentResumeDescription(project.state), completed_tasks: window.projectProgress(project.state), total_tasks: TASKS.length,
        note: "Written by ALIX Studio Lite: the trajectory files and their records are the ones Lite's server keeps for this project.",
        files: files.map(f => ({ path: f.path, bytes: f.bytes, sha256: f.sha256 })) };
      await window.fsWriteJson(root, 'project_manifest.json', manifest);
      const readme = await (await (await root.getFileHandle('README.txt')).getFile()).text();
      await writeFile(root, 'README.txt', readme.split('IMPORTANT:')[0] + README_NOTE);
      window.toast(`Project folder saved: ${project.name} · ${files.length} files from Lite (${(bytes / 1e6).toFixed(1)} MB) and ${rows.length} log events.`);
    } catch (err) {
      tell(Object.assign(err, { message: `The project folder has the page's manifests, but Lite's files were not all copied: ${err.message}` }));
    }
  };
}
