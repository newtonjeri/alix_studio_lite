// Defects found in the designer's page, fixed here so the file itself stays unchanged. Each is
// listed in docs/UI_CONTRACT.md for the designer's next version (V19's still in V32 are kept).

export function installFixes() {
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
