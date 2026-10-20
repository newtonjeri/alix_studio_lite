// Notifications (the page's toast) stay on screen at least 15 s: the page's 2.6 s was gone before
// an operator at the robot could read it (the owner, 14 Oct). A click closes one sooner, and a new
// one replaces it.
//
// Each one is coloured by what it says (the owner, 20 Oct): green for a success or a good status
// (saved, applied, read, connected again), orange for a warning (something to do first, a refusal,
// a stop, a limit), red for an error (something failed). Lite's own modules say the level with the
// message, window.toast(text, 'good' | 'warn' | 'error'); the page's own messages carry none and
// are read by their words (levelOf).

export const TOAST_MS = 15000;
export const LEVELS = ['good', 'warn', 'error'];

// Something failed, or cannot work at all.
const ERROR = [
  /^could not\b/i, /^invalid\b/i, /\bfailed\b/i, /^the (program|move) stopped\b/i, /\bstopped at step\b/i, /\bstopped: /i,
  /^nothing plays\b/i, /\bnot supported by this browser\b/i, /^lite cannot reach\b/i, /^something went wrong\b/i,
  /\bfault is latched\b/i, /press the red e-stop/i, /\bis not shown: /i, /\bdid not (reach|answer|finish)\b/i,
  /\bhas not (answered|confirmed)\b/i, /^not staged: /i, /\berror\b/i,
];
// Something to do first, a refusal, a limit, a stop: nothing broke.
const WARN = [
  /\bfirst\b/i, /^(choose|select|enter|add|type|open|connect|complete|confirm|finish|move|build|run|log in|apply|please|re-browse)\b/i,
  /\bmust\b/i, /\block(ed)?\b/i, /^dual safety interlock\b/i, /\bnot (run|switched|connected|in release|on the controller|read|staged)\b/i,
  /^no project\b/i, /^project not found/i, /^cancelled\b/i, /\bstays? as it is\b/i, /\balready\b/i, /\bis empty\b/i, /\bnever merged\b/i,
  /\bno longer\b/i, /\bstill\b/i, /^before\b/i, /\bhas no\b/i, /\bneeds\b/i, /\brequired\b/i, /\bwas stopped\b/i, /^stop both\b/i,
  /\bnothing was (sent|run)\b/i, /\bno hardware command\b/i, /\bwill be connected to\b/i, /\bdisabled\b/i, /\bcannot\b/i, /\bcan.t\b/i,
  /\bnot drawn\b/i,
];

// The level of a message that came without one: by its words.
export function levelOf(text) {
  const t = String(text || '');
  if (ERROR.some(re => re.test(t))) return 'error';
  if (WARN.some(re => re.test(t))) return 'warn';
  return 'good';
}

export function installNotify() {
  const box = document.getElementById('toast');
  if (!box) return;
  box.title = 'Click to close';
  box.addEventListener('click', () => { clearTimeout(box._x); box.classList.remove('show'); });
  window.toast = function toast(msg, level) {
    const lv = LEVELS.includes(level) ? level : levelOf(msg);
    box.textContent = msg;
    for (const l of LEVELS) box.classList.toggle(`lite-${l}`, l === lv);
    box.setAttribute('role', lv === 'error' ? 'alert' : 'status');
    box.classList.add('show');
    clearTimeout(box._x);
    box._x = setTimeout(() => box.classList.remove('show'), TOAST_MS);
  };
  window.toast.levelOf = levelOf;
}
