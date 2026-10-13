// Charts on the ALIX tokens: text >= 14 px, grid and text colours from the theme, the seven-colour
// joint palette (no brand blue, no state red, green or amber). Re-styled when the theme changes.

export function css(name) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
}

export function jointColours() {
  return [1, 2, 3, 4, 5, 6, 7].map(j => css(`--j${j}`));
}

export function plotTheme(extra = {}) {
  const text = css('--text'), muted = css('--muted'), grid = css('--border'), line = css('--border-strong');
  const axis = { gridcolor: grid, linecolor: line, zerolinecolor: line, tickfont: { size: 14, color: muted },
                 title: { font: { size: 14, color: muted } }, automargin: true };
  return {
    paper_bgcolor: css('--panel'), plot_bgcolor: css('--panel'),
    font: { family: css('--font'), size: 14, color: text },
    margin: { l: 66, r: 16, t: 40, b: 48 },
    xaxis: { ...axis, title: { text: 'Time (s)', font: { size: 14, color: muted } } },
    yaxis: { ...axis },
    legend: { orientation: 'h', x: 0, y: 1.02, yanchor: 'bottom', font: { size: 14, color: text } },
    hoverlabel: { font: { family: css('--mono'), size: 14 } },
    ...extra,
  };
}

export const plotConfig = { displaylogo: false, responsive: true,
  modeBarButtonsToRemove: ['lasso2d', 'select2d', 'autoScale2d', 'toggleSpikelines'] };

// joints: [7][N] degrees at 5 ms per line; dash: 'solid' | 'dot'
export function jointTraces(joints, { name = '', dash = 'solid', opacity = 1, show = true, cycle = 0.005 } = {}) {
  const cols = jointColours();
  const n = joints[0].length;
  const step = n > 6000 ? Math.ceil(n / 6000) : 1;           // keep the browser fast on long files
  const t = [];
  for (let i = 0; i < n; i += step) t.push(+(i * cycle).toFixed(3));
  return joints.map((q, j) => ({
    type: 'scattergl', mode: 'lines', x: t, y: step === 1 ? q : q.filter((_, i) => i % step === 0),
    name: `J${j + 1}${name ? ' ' + name : ''}`, line: { color: cols[j], width: dash === 'solid' ? 1.8 : 1.2, dash },
    opacity, showlegend: show, hovertemplate: `J${j + 1} %{y:.1f}°<extra>${name}</extra>`,
  }));
}

export function shadeWindows(windows, color) {
  return windows.map(w => ({ type: 'rect', xref: 'x', yref: 'paper', x0: w[0], x1: w[1], y0: 0, y1: 1,
    fillcolor: color, opacity: 0.18, line: { width: 0 }, layer: 'below' }));
}

export function onTheme(fn) {
  const h = () => fn();
  window.addEventListener('lite-theme', h);
  return () => window.removeEventListener('lite-theme', h);
}

// A plot in a box the user can drag taller, with a full-size toggle above it (Esc returns). The
// plot follows the box's size, so it re-flows when the window or a neighbouring panel changes.
export function plotBox({ short = false } = {}) {
  const box = document.createElement('div');
  box.className = 'plot-box' + (short ? ' short' : '');
  const tools = document.createElement('div');
  tools.className = 'plot-tools';
  const area = document.createElement('div');
  area.className = 'plot-area';
  const el = document.createElement('div');
  el.className = 'plot';
  const btn = document.createElement('button');
  btn.className = 'btn quiet';
  btn.type = 'button';
  const resize = () => { if (window.Plotly && el.data) window.Plotly.Plots.resize(el); };
  const toggle = (on = !box.classList.contains('maxed')) => {
    box.classList.toggle('maxed', on);
    btn.textContent = on ? '⤡ Exit full size' : '⤢ Full size';
    btn.title = on ? 'Back to normal size (Esc)' : 'Make this plot full size';
    requestAnimationFrame(resize);
  };
  btn.onclick = () => toggle();
  toggle(false);
  const onKey = (e) => { if (e.key === 'Escape' && box.classList.contains('maxed')) toggle(false); };
  document.addEventListener('keydown', onKey);
  const ro = new ResizeObserver(() => resize());
  ro.observe(area);
  tools.append(btn);
  area.append(el);
  box.append(tools, area);
  return { box, el, dispose: () => { ro.disconnect(); document.removeEventListener('keydown', onKey); } };
}
