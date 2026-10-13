// The page's top bar needs about 1,610 px: on narrower screens it pushed STOP BOTH off the
// screen. The top-bar buttons get their labels in a span here, so lite.css can show their icons
// only on narrower screens; STOP BOTH and the status then stay visible down to 900 px.

const BUTTONS = ['.project-save-btn', '.project-manager-btn', '.robot-control-toggle', '.top-tour-btn', '.top-lock-btn'];

const span = (className, text) => Object.assign(document.createElement('span'), { className, textContent: text });

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
}
