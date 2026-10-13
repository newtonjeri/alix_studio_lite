// Log-in and the engineer PIN, checked by the backend (handoff section 19) instead of the
// template PINs in the page. What the page does after a correct PIN is kept as it was.

import { post, say, session } from './api.js';
import { flush, loadProjects } from './projects.js';

let engineerLock = Promise.resolve();

function loginMessage(text, cls = '') {
  const msg = document.getElementById('operatorLoginMessage');
  if (msg) { msg.textContent = text; msg.className = `operator-login-message ${cls}`.trim(); }
}

export function installAuth() {
  session.onLost(() => {
    if (!operatorLoggedIn) return;
    session.set(null);
    window.showOperatorLogin();
    window.toast('Please log in again: the Lite session has ended.');
  });

  window.submitOperatorLogin = async function submitOperatorLogin() {
    const input = document.getElementById('operatorLoginPin');
    const pin = (input?.value || '').trim();
    let r;
    try {
      r = await post('/api/auth/login', { pin });
      session.set(r.token);
      await loadProjects();
    } catch (err) {
      session.set(null);
      loginMessage(say(err), 'error');
      if (input) { input.value = ''; input.focus(); }
      return;
    }
    document.dispatchEvent(new CustomEvent('lite:logged-in'));
    // the page's own steps after a correct PIN (V19 submitOperatorLogin)
    operatorLoggedIn = true;
    loginMessage('Login successful', 'ok');
    document.getElementById('operatorLoginScreen')?.classList.add('hidden');
    document.getElementById('mainApp')?.classList.remove('app-login-locked');
    window.showView('home');
    window.setJourneyVisible(true, false);
    try { operatorFirstRunTourPending = localStorage.getItem(TOUR_STORAGE_KEY) !== 'true'; } catch (_) { operatorFirstRunTourPending = true; }
    setTimeout(() => {
      window.showProjectManager();
      window.toast(r.default_pin
        ? 'Logged in. The operator PIN is still the template PIN: ask an engineer to change it.'
        : 'Operator login successful · choose a project to continue.');
    }, 180);
  };

  const pageLock = window.lockOperatorSession;
  window.lockOperatorSession = async function lockOperatorSession() {
    pageLock();                                  // saves the project and shows the log-in screen
    // Fix (page defect 2): the next log-in opens the Project Manager, which saved the open project
    // again with the view the log-in had just set (Home), so "Resume" lost the task and stage.
    // After a lock no project is open until the operator picks one; its record keeps the stage.
    currentProjectId = null;
    window.updateCurrentProjectHeader();
    await flush();
    await post('/api/auth/logout').catch(() => {});
    session.set(null);
  };

  window.submitRobotPin = async function submitRobotPin() {
    const input = document.getElementById('robotPinInput');
    const msg = document.getElementById('robotPinMessage');
    const pin = (input?.value || '').trim();
    try {
      await engineerLock;                        // a lock sent just before must land first
      await post('/api/auth/engineer', { pin });
    } catch (err) {
      robotPinAttempts++;
      if (msg) { msg.textContent = say(err); msg.className = 'pin-message error'; }
      if (input) { input.value = ''; input.focus(); }
      return;
    }
    robotPinAttempts = 0;
    robotControlUnlocked = true;
    if (msg) { msg.textContent = 'Access granted'; msg.className = 'pin-message ok'; }
    setTimeout(() => { window.closeRobotPin(); window.openRobotControl(); }, 220);
  };

  // Advanced Robot Controls lock again on the server whenever the page locks them.
  for (const name of ['requestRobotControlAccess', 'closeRobotControl', 'lockRobotControl']) {
    const page = window[name];
    window[name] = function (...args) {
      engineerLock = post('/api/auth/engineer/lock').catch(() => {});
      return page.apply(this, args);
    };
  }
}
