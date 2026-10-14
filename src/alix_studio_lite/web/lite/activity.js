// V32's audit trail (its 05 Activity Logs page), kept by Lite's server: the backend is the audit
// authority (handoff V30, V31). The page's own events (auditLog: navigation, buttons, inputs, the 30 s
// heartbeat, log-in attempts) go to the server in batches (POST /api/audit, server/logbook.py's
// activity log), and the page's table, filters and exports read the server's whole record
// (GET /api/logs): the page's events with every request, job and controller command the server
// logged. Nothing is kept only in this browser; a log that cannot be sent is tried again and never
// stops the page.

import { session } from './api.js';

const cache = { rows: [], ids: new Set(), from: 0, loading: null, last: 0 };
let pending = [];
let flushTimer = 0;

function send(events, keepalive = false) {
  const headers = { 'Content-Type': 'application/json' };
  if (session.token) headers['X-Lite-Token'] = session.token;
  return fetch('/api/audit', { method: 'POST', headers, body: JSON.stringify({ events }), keepalive })
    .then((r) => { if (!r.ok) throw new Error(String(r.status)); });
}

function flush(keepalive = false) {
  clearTimeout(flushTimer);
  flushTimer = 0;
  if (!pending.length) return;
  const batch = pending.splice(0, 200);
  send(batch, keepalive).catch(() => { pending = batch.concat(pending).slice(-2000); });   // tried again later
  if (pending.length) flushTimer = setTimeout(flush, 1000);
}

function remember(row) {
  if (cache.ids.has(row.id)) return;
  cache.ids.add(row.id);
  cache.rows.push(row);
}

// The server's record from `from` (epoch seconds), with this page's events not sent yet.
async function load(from) {
  if (cache.loading) return cache.loading;
  cache.loading = (async () => {
    try {
      const r = await fetch(`/api/logs?from=${Math.floor(from)}`, { headers: session.token ? { 'X-Lite-Token': session.token } : {} });
      const d = await r.json();
      if (!d.events) return;
      const unsent = pending.slice();
      cache.rows = []; cache.ids = new Set(); cache.from = from;
      for (const row of d.events) remember(row);
      for (const row of unsent) remember(row);
      cache.rows.sort((a, b) => String(a.timestamp).localeCompare(String(b.timestamp)));
      cache.last = Date.now();
    } catch (_) { /* the table keeps what it has */ } finally { cache.loading = null; }
  })();
  return cache.loading;
}

const logsOpen = () => !!document.getElementById('view-logs')?.classList.contains('active');

// The earliest moment the filters ask for (the last 24 hours at least).
function wantedFrom() {
  const f = document.getElementById('auditFrom')?.value;
  const day = Date.now() / 1000 - 86400;
  const asked = f ? new Date(f).getTime() / 1000 : day;
  return Math.min(day, Number.isFinite(asked) ? asked : day);
}

async function refresh() {
  if (!session.token) return;
  await load(wantedFrom());
  pageRender();
}

let pageRender = () => {};

export function installActivity() {
  window.auditLog = function auditLog(category, event, details = '', result = 'INFO', extra = {}) {
    if (ASL_AUDIT_RENDER_GUARD) return undefined;
    const c = window.auditCurrentContext();
    const row = {
      id: `log_${Date.now().toString(36)}_${Math.random().toString(36).slice(2, 7)}`,
      timestamp: new Date().toISOString(),
      sessionId: ASL_AUDIT_SESSION_ID,
      user: extra.user ?? (operatorLoggedIn ? 'Operator' : 'Unauthenticated'),
      projectId: extra.projectId ?? c.projectId,
      projectName: extra.projectName ?? c.projectName,
      page: extra.page ?? c.page,
      stage: extra.stage ?? c.stage,
      task: extra.task ?? c.task,
      arm: extra.arm ?? c.arm,
      category: String(category || 'SYSTEM').toUpperCase(),
      event: String(event || 'Event'),
      details: typeof details === 'string' ? details : JSON.stringify(details),
      result: String(result || 'INFO').toUpperCase(),
      source: extra.source || 'FRONTEND',
    };
    pending.push(row);
    remember(row);
    if (!flushTimer) flushTimer = setTimeout(flush, 800);
    const badge = document.getElementById('auditLastEventBadge');
    if (badge) badge.textContent = `Last event: ${new Date(row.timestamp).toLocaleTimeString()}`;
    if (logsOpen()) pageRender();
    return row;
  };
  // What the page logged before this ran (its "ASL frontend initialized"), from the browser's copy
  // to the server; the browser keeps none of it after that.
  try {
    for (const row of JSON.parse(localStorage.getItem(ASL_AUDIT_LOG_KEY) || '[]')) {
      if (row && row.sessionId === ASL_AUDIT_SESSION_ID) { pending.push(row); remember(row); }
    }
    localStorage.removeItem(ASL_AUDIT_LOG_KEY);
  } catch (_) { /* nothing to carry over */ }
  if (pending.length) flushTimer = setTimeout(flush, 800);
  window.auditLoad = function auditLoad() { return cache.rows; };
  window.auditSave = function auditSave() { return true; };          // the server keeps them

  // The page's table, as the page draws it, from the server's record: read when the page opens and
  // every few seconds while it is open (the page redraws every second), and on Refresh / Today /
  // Last 24 Hours / a new From.
  pageRender = window.renderAuditLog;
  window.renderAuditLog = function renderAuditLog() {
    pageRender();
    if (logsOpen() && (Date.now() - cache.last > 3000 || wantedFrom() < cache.from)) refresh();
  };
  for (const name of ['auditSetToday', 'auditSetLast24Hours', 'auditClearFilters']) {
    const page = window[name];
    if (typeof page === 'function') window[name] = function (...args) { const out = page.apply(this, args); refresh(); return out; };
  }
  document.getElementById('auditFrom')?.addEventListener('change', refresh);
  const note = document.querySelector('.audit-log-note span');
  if (note) note.textContent = 'Lite\'s server keeps this trail in its logs (Resources/Logs, one folder a day): the page\'s own events '
    + '(screens, buttons, settings, the 30 s heartbeat while logged in) with every request, job result and controller command. '
    + 'Source FRONTEND is the page, BACKEND the server, ROBOT the controllers. The last 24 hours are read, or from the From date.';
  const total = document.getElementById('auditTotalSaved')?.nextElementSibling;
  if (total) total.textContent = 'events read from Lite\'s logs';

  document.addEventListener('lite:logged-in', () => { flush(); refresh(); });
  document.addEventListener('lite:open-robot-panel', () => window.auditLog('ROBOT', 'Robot & mode opened', '', 'INFO'));
  window.addEventListener('pagehide', () => flush(true));
  document.addEventListener('visibilitychange', () => { if (document.visibilityState === 'hidden') flush(true); });
}
