// Help: how a region goes from teaching to the robot, and the words Lite uses.
import { h } from '../core.js';

const GLOSSARY = [
  ['Region', 'One area of the toilet that one arm cleans, or one lid or seat move, as numbered on the regions deck (for example L-R04, LEFT HINGE).'],
  ['Teach', 'Guide the arm by hand through the region while Lite records it.'],
  ['Recording', 'What you taught, kept exactly as recorded. Lite never edits it; it makes new versions from it.'],
  ['Optimise', 'Make the motion shorter and smoother without leaving the taught path.'],
  ['Default optimiser', 'Keeps the shape of the recording: removes waiting, smooths, then speeds it up evenly.'],
  ['Time-optimal optimiser', 'The fastest the arm allows: it slows down only where the path needs it.'],
  ['Target time', 'How long the region should take. If the arm cannot do it that fast, Lite gives the closest safe time and says what limits it.'],
  ['Limits', 'What every file must stay within: joint end stops, joint speed, 600°/s² acceleration, no jump over 10° between lines, the taught path, and the taught start and end.'],
  ['Version', 'An optimised file you kept. Checks and tests always belong to one version.'],
  ['3D check', 'Lite plays the motion against the toilet, the cubicle walls, the bin and the other arm before it ever runs on the robot.'],
  ['Visual sim', 'The robot moves in the 3D view only. No hardware is used.'],
  ['Hardware sim', 'The controller runs the program in its own SIM mode while the arms stand still.'],
  ['Real robot', 'The arms move. Lite asks you to confirm first.'],
  ['STOP', 'Software stop at the top of the screen: the arm slows to a stop. It never asks a question.'],
  ['E-stop', 'The red hardware buttons. They are the safety stop. While one is pressed, Lite blocks all motion on both arms.'],
  ['Recover', 'What to do after an E-stop: make the cell safe, release the button, then let Lite clear the controller and check the way back.'],
  ['Send to robot', 'Store the version on the arm controller so it can run.'],
];

export function render(main) {
  const page = h('div.page');
  page.append(h('div.page-head', { html: '<div class="titles"><h2>Help</h2><p>How a region goes from teaching to the robot, and what the words mean.</p></div>' }));
  const steps = h('div.card');
  steps.innerHTML = `<div class="card-head"><h3>The four steps of every region</h3></div><div class="card-body"><ol class="stack" style="margin:0;padding-left:20px">
    <li><b>Teach.</b> Check the toilet's state, prepare the robot (pole, then hand), hold the arm, press Start, guide it, press Stop.</li>
    <li><b>Optimise.</b> Choose Default or Time-optimal and the target time, press Optimise, then keep the result.</li>
    <li><b>3D view.</b> Lite plays the motion against the toilet, walls, bin and the other arm.</li>
    <li><b>Run.</b> Play it in the 3D view (Visual sim), send it to the robot and run it in Hardware sim, then on the real robot, and say how it went.</li></ol>
    <p class="muted">Keys: Ctrl+Z undoes and Ctrl+Y redoes a settings change on the Optimise step.</p></div>`;
  const gl = h('div.card');
  gl.innerHTML = `<div class="card-head"><h3>Glossary</h3></div><div class="card-body"><dl class="glossary">${GLOSSARY.map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join('')}</dl></div>`;
  page.append(steps, gl);
  main.append(page);
}
