// Moving between tasks (the owner, 21 Oct: "The user can move to another task without completing the current one, for
// any reason"; "When the user moves away from a task, capture and save its current state: its status, progress or
// level. The task's progress is kept, so the user can come back to it"; "A task that was left partway shows as 'in
// progress'"). The page opened another task only once Real Test had passed on the arm being taught (jumpTask: "Finish
// Real Test for the current arm before opening another task"). Now the task list, the Summary's Open and every other
// way a task is chosen open it at once. The task left is saved first: the page's own record of it (its stage, its arm,
// each arm's setup, the arms approved, the tuning on screen) and whether its arm was locked; a task opened again
// resumes there: its arm, its stage, its lock. What a task holds on the server (its recordings, kept versions, test
// results) was always kept. One thing holds a switch: a drag-teaching take or its countdown (STOP DRAG, which saves the
// take, is in that task's Drag & Teach).
//
// The task list and the Summary say IN PROGRESS for a task left partway: one an arm of which has a recording of the
// operator's own (not the fixture's taught set), a kept version, an approval, or was left past Prepare.

import { el } from './ctx.js';
import { refreshTasks } from './tasks.js';
import { teachingNow } from './teach.js';

const PAST_PREPARE = new Set(['teach', 'review', 'optimize', 'real']);
const STAGE_WORDS = { prepare: 'Prepare', teach: 'Drag & Teach', review: 'Save & Review', optimize: 'Tune + Simulation', real: 'Real Test' };

// The task on screen, as it stands, kept with the page's record of it (saved with the project).
export function leaveTask() {
  if (projectRestoreInProgress || !TASKS[currentTask] || completed[currentTask]) return;
  try {
    const rec = window.captureCurrentTaskRecord();
    rec.liteResume = { stage: currentStage, arm: selectedArm, locked: !!armSelectionLocked, left: new Date().toISOString() };
    taskRecords[currentTask] = rec;
    window.safeSaveTaskRecords();
  } catch (_) { /* the page still opens the other task */ }
}

// A task opened again, where it was left: the arms approved, its arm, its lock, its stage.
async function resumeTask(i) {
  const rec = taskRecords[i] || {}, r = rec.liteResume;
  if (!r || completed[i] || i !== currentTask) return;
  armDone = { left: !!rec.leftApproved || armDone.left, right: !!rec.rightApproved || armDone.right };   // armDone: a step not in Release 1
  armSelectionLocked = false;
  window.selectArm(r.arm === 'right' || r.arm === 'left' ? r.arm : selectedArm);
  armSelectionLocked = !!r.locked;
  window.updateArmLockUI();
  if (r.stage && r.stage !== currentStage && STAGE_WORDS[r.stage]) await window.setStage(r.stage, false);   // Prepare's gate may hold it
  window.updateTeachingStageContext(currentStage);
  window.renderTaskList();
  window.renderSummary();
  if (PAST_PREPARE.has(r.stage) || r.locked) {
    window.toast(`${TASKS[i].label}: back where it was left, the ${selectedArm.toUpperCase()} arm in ${STAGE_WORDS[currentStage] || currentStage}.`, 'good');
  }
}

// Whether a task (or one arm of it) was left partway.
export function armProgressed(i, side) {
  const t = TASKS[i], s = t && t.lite && t.lite[side], rec = taskRecords[i] || {}, r = rec.liteResume;
  if (!t || completed[i]) return false;
  if (s && !s.later && (Number(s.own) > 0 || s.version)) return true;
  if ((side === 'left' && rec.leftApproved) || (side === 'right' && rec.rightApproved) || (rec.approvedSetup || {})[side]) return true;
  return !!(r && r.arm === side && (PAST_PREPARE.has(r.stage) || r.locked));
}
export const progressed = (i) => armProgressed(i, 'left') || armProgressed(i, 'right');

const IN_PROGRESS = 'IN PROGRESS';

export function installProgress() {
  const pageJump = window.jumpTask;
  window.jumpTask = async function jumpTask(i) {
    i = Number(i);
    if (!isSetupReady()) { pageJump(i); return; }                    // the page's own message
    if (teachingNow()) {
      window.toast(`Drag teaching is running on the ${selectedArm.toUpperCase()} arm: press STOP DRAG first (it saves the take), `
        + 'then open another task.', 'warn');
      return;
    }
    if (i === currentTask && !completed[i]) { window.showView('teach'); return; }     // open already: as it is
    leaveTask();
    if (completed[i]) { window.openApprovedTask(i); return; }       // the page's Edit / Redo of an approved task
    armSelectionLocked = false;                                     // the page refused while the arm was locked
    pageJump(i);
    if (currentTask === i) await resumeTask(i);
    refreshTasks().catch(() => {});                                 // each step's recordings and versions, for the statuses
  };
  const pageApproved = window.openApprovedTask;
  window.openApprovedTask = function openApprovedTask(i) {          // the Summary's Edit / Redo: the task left is kept first
    if (Number(i) !== currentTask) leaveTask();
    return pageApproved(i);
  };

  // the task list's queue and each arm's status say IN PROGRESS for a task left partway
  const pageQueue = window.taskQueueStatus;
  window.taskQueueStatus = function taskQueueStatus(i, t) {
    const own = pageQueue(i, t);
    return own.cls === 'pending' && progressed(i) ? { label: IN_PROGRESS, cls: 'lite-inprogress' } : own;
  };
  const pageArm = window.armTaskStatusInfo;
  window.armTaskStatusInfo = function armTaskStatusInfo(i, side) {
    const own = pageArm(i, side);
    return own.label === 'PENDING' && armProgressed(i, side) ? { label: IN_PROGRESS, cls: 'orange' } : own;
  };
  // the Summary's Status column (the page writes it in place: PENDING becomes IN PROGRESS there)
  const pageSummary = window.renderSummary;
  window.renderSummary = function renderSummary() {
    pageSummary();
    const rows = document.querySelectorAll('#summaryTable > tr');
    TASKS.forEach((t, i) => {
      const cells = rows[i] && rows[i].children;
      const badge = cells && cells[cells.length - 2] && cells[cells.length - 2].querySelector('.badge');
      if (badge && badge.textContent === 'PENDING' && i !== currentTask && progressed(i)) {
        badge.replaceWith(el('span', { class: 'badge orange lite-inprogress-badge', text: IN_PROGRESS,
          title: 'Left partway: Open goes back where it was left' }));
      }
    });
  };
}
