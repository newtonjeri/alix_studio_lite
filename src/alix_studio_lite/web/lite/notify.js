// Notifications (the page's toast) stay on screen at least 15 s: the page's 2.6 s was gone before
// an operator at the robot could read it (the owner, 14 Oct). A click closes one sooner, and a new
// one replaces it.

export const TOAST_MS = 15000;

export function installNotify() {
  const box = document.getElementById('toast');
  if (!box) return;
  box.title = 'Click to close';
  box.addEventListener('click', () => { clearTimeout(box._x); box.classList.remove('show'); });
  window.toast = function toast(msg) {
    box.textContent = msg;
    box.classList.add('show');
    clearTimeout(box._x);
    box._x = setTimeout(() => box.classList.remove('show'), TOAST_MS);
  };
}
