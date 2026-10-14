// The page's own steps, for Lite's activity log (server/logbook.py, the owner's request of
// 14 October: "log every activity"). Every request that changes something is logged by the
// server already; this adds what only the page knows: the view, the stage, the task and the arm
// on screen, and the windows opened. Sent quietly: a log that cannot be written never stops the page.

import { post, session } from './api.js';

function note(event, detail) {
  if (!session.token) return;
  post('/api/activity', { event, detail }).catch(() => {});
}

function wrap(name, describe) {
  const page = window[name];
  if (typeof page !== 'function') return;
  window[name] = function (...args) {
    const out = page.apply(this, args);
    const done = () => { try { note(name, describe(...args)); } catch (_) { /* never in the way */ } };
    Promise.resolve(out).then(done, done);          // what it led to, once an async step has settled
    return out;
  };
}

export function installActivity() {
  wrap('showView', (v) => ({ view: v }));
  wrap('setStage', (s) => ({ asked: s, stage: currentStage, task: TASKS[currentTask]?.label, arm: selectedArm }));
  wrap('loadTask', (i) => ({ task: TASKS[i]?.label }));
  wrap('selectArm', (a) => ({ arm: a, task: TASKS[currentTask]?.label }));
  wrap('openProjectById', (id) => ({ project: id }));
  wrap('openRobotControl', () => ({}));
  wrap('runFullSequenceSimulation', () => ({}));
  wrap('toggleTheme', () => ({ theme: document.documentElement.dataset.theme }));
  document.addEventListener('lite:open-robot-panel', () => note('robot panel', {}));
}
