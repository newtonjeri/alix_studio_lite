// Defects found in the designer's page (V19), fixed here so the file itself stays unchanged.
// Each is listed in docs/UI_V19_CONTRACT.md for the designer's next version.

export function installFixes() {
  // 2. Home's "Prototype rule" notice says the hardware buttons only demonstrate the workflow and
  //    that controller integration comes later; with Lite's backend that is no longer so.
  const rule = [...document.querySelectorAll('#view-home .notice')].find(n => /Prototype rule/.test(n.textContent));
  if (rule) {
    rule.replaceChildren(Object.assign(document.createElement('b'), { textContent: 'How Lite runs:' }),
      ' every step is saved with the project. With the robot connected (the status pill at the top), Hardware sim runs '
      + 'the programs in the controllers\' SIM mode and Real moves the arms, the poles and the hands, each move after a '
      + 'confirmation. STOP BOTH is always at hand.');
  }

  // 1. readHandValues() returns {Little: n, ...}, but these two called .forEach on it, so opening
  //    Advanced Robot Controls threw and "Apply Hand" in Prepare never reached the arm panel.
  window.rcSyncGuidedValues = function rcSyncGuidedValues() {
    const S = rcSideId(selectedArm);
    rcPoleSync(selectedArm, Number(document.getElementById('poleHeight')?.value || 0), 'num');
    Object.values(readHandValues()).forEach((v, i) => rcHandSync(selectedArm, i, v, 'num'));
    const pid = Number(document.getElementById('programId')?.value || 0);
    if (pid >= 1 && pid <= 100) {
      const slot = document.getElementById(`rc${S}Slot`);
      if (slot) slot.value = pid;
    }
  };

  window.demoApplyToRobotControl = function demoApplyToRobotControl(kind) {
    if (kind === 'Pole') {
      rcPoleSync(selectedArm, Number(document.getElementById('poleHeight')?.value || 0), 'num');
      rcApplyPole(selectedArm);
    } else if (kind === 'Hand') {
      Object.values(readHandValues()).forEach((v, i) => rcHandSync(selectedArm, i, v, 'num'));
      rcApplyHand(selectedArm);
    }
  };
}
