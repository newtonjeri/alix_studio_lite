// ALIX's brand mark, drawn as ALIX Studio draws it (alix_taskgen/web_editor/template.html: four
// facets in the brand blues, with dark seams), in place of the page's "AX" placeholder tiles in the
// top bar and on the log-in screen, and as the browser tab's icon. The colours are ALIX's brand
// tokens (lite/tokens.css), so the mark follows the light and dark themes as ALIX Studio's does.

const FACETS = [
  ['--lite-brand', 'M0.60 1.31 L1.52 0.60 L2.60 0.75 L15.45 11.23 L15.63 12.18 L2.20 22.96 L1.12 22.96 L0.60 22.44 Z'],
  ['--lite-brand', 'M31.40 22.44 L30.88 22.96 L29.80 22.96 L16.37 12.18 L16.55 11.23 L29.40 0.75 L30.48 0.60 L31.40 1.31 Z'],
  ['--lite-brand-strong', 'M2.02 24.35 L15.69 13.17 L10.95 31.25 L10.36 31.40 Z'],
  ['--lite-brand-strong', 'M21.64 31.40 L21.05 31.25 L16.31 13.17 L29.98 24.35 Z'],
];

// The mark as SVG markup; `colour(name)` gives each token's paint (the variable itself on the page).
function mark(colour = (name) => `var(${name})`) {
  return '<svg xmlns="http://www.w3.org/2000/svg" class="lite-brand-mark" viewBox="0 0 32 32" aria-hidden="true" focusable="false">'
    + `<g stroke="${colour('--lite-brand-seam')}" stroke-width="0.55" stroke-linejoin="round">`
    + FACETS.map(([name, d]) => `<path fill="${colour(name)}" d="${d}"/>`).join('')
    + '</g></svg>';
}

export function installBrand() {
  for (const tile of document.querySelectorAll('.topbar .brand .logo, .login-brand-mark')) {
    tile.innerHTML = mark();
    tile.classList.add('lite-brand');
    tile.setAttribute('role', 'img');
    tile.setAttribute('aria-label', 'ALIX');
  }
  // the tab's icon, in the theme's colours at load (an icon cannot use the page's variables)
  const css = getComputedStyle(document.documentElement);
  const icon = mark((name) => css.getPropertyValue(name).trim());
  document.querySelectorAll('link[rel~="icon"]').forEach(l => l.remove());
  document.head.append(Object.assign(document.createElement('link'),
    { rel: 'icon', type: 'image/svg+xml', href: `data:image/svg+xml,${encodeURIComponent(icon)}` }));
}
