// The top bar fits the window at any width (the owner, 16 Oct: "fit 100% of the available screen
// width, ensuring that all controls, including the Stop button, are fully visible without horizontal
// scrolling"). Its buttons get their labels in a span here, and the bar is measured: when it is wider
// than the window, it gives way one level at a time (lite.css: lite-fit-1 to lite-fit-6, labels to
// icons, then narrower), however many controls it holds. It is measured again when the window or
// anything in the bar changes (the status, the project's name, the speed).

const BUTTONS = ['.project-save-btn', '.project-manager-btn', '.robot-control-toggle', '.top-tour-btn', '.top-lock-btn'];

const span = (className, text) => Object.assign(document.createElement('span'), { className, textContent: text });

const LEVELS = 6;
let fitting = 0;

export function fitTopbar() {
  const bar = document.querySelector('.topbar');
  if (!bar) return;
  for (let i = 1; i <= LEVELS; i += 1) bar.classList.remove(`lite-fit-${i}`);
  for (let level = 1; level <= LEVELS && bar.scrollWidth > bar.clientWidth + 1; level += 1) bar.classList.add(`lite-fit-${level}`);
}

function fitSoon() {
  if (fitting) return;
  fitting = requestAnimationFrame(() => { fitting = 0; fitTopbar(); });
}

export function installLayout() {
  for (const sel of BUTTONS) {
    const b = document.querySelector(`.topbar ${sel}`);
    if (!b || b.querySelector('.lite-label')) continue;
    const m = b.textContent.trim().match(/^(\S+)\s+(.+)$/u);
    if (!m) continue;
    const [, icon, label] = m;
    const text = span('lite-label', ` ${label}`);
    if (sel === '.robot-control-toggle') {             // "Advanced Robot Controls" -> "Advanced"
      const [first, ...rest] = label.split(' ');
      text.replaceChildren(document.createTextNode(` ${first}`), span('lite-long', ` ${rest.join(' ')}`));
    }
    b.replaceChildren(span('lite-icon', icon), text);
    b.title = b.title || label;
    b.setAttribute('aria-label', label);
  }
  window.addEventListener('resize', fitSoon);
  const bar = document.querySelector('.topbar');
  if (bar) new MutationObserver(fitSoon).observe(bar, { childList: true, subtree: true, characterData: true });
  fitSoon();
}
