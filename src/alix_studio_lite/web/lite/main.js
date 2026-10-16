// ALIX Studio Lite's bridge for the designer's page (web/index.html, V19 Project Manager).
//
// The page is the designer's file, unchanged; it runs first and shows its log-in screen. This
// module then replaces the page's template functions with versions that use the backend, one
// group per file. docs/UI_CONTRACT.md lists every replaced function and what is still a
// template. Each install function keeps the page's own screen updates and adds the backend call.

import { installActivity } from './activity.js';
import { installAdvanced } from './advanced.js';
import { installAlerts } from './alerts.js';
import { installAuth } from './auth.js';
import { installBrand } from './brand.js';
import { installFixes } from './fixes.js';
import { installFolder } from './folder.js';
import { installFullSeq } from './fullseq.js';
import { installLayout } from './layout.js';
import { installNotify } from './notify.js';
import { installPlacement } from './placement.js';
import { installPrepare } from './prepare.js';
import { installProjects } from './projects.js';
import { installRealTest } from './realtest.js';
import { installRecovery } from './recovery.js';
import { installReview3d } from './review3d.js';
import { installRobotPanel } from './robotpanel.js';
import { installSafety } from './safety.js';
import { installSim } from './sim.js';
import { installStatus } from './status.js';
import { installSummary } from './summary.js';
import { installTasks } from './tasks.js';
import { installTeach } from './teach.js';
import { installTransitions } from './transitions.js';
import { installTune } from './tune.js';
import { installViewer } from './viewer.js';

installSafety();          // first: STOP must reach the robot before and without a log-in
installLayout();          // ...and stay on the screen (the page's top bar needs 1,610 px)
installRecovery();        // Arm Drag Mode / Normal Mode beside STOP BOTH (the owner, 16 Oct)
installBrand();           // ALIX's brand mark, as ALIX Studio draws it
installNotify();          // notifications stay 15 s (the owner, 14 Oct)
installFixes();
installProjects();
installFolder();          // Save Project Folder with Lite's real files (V32)
installAuth();
installStatus();
installAlerts();          // hardware errors and failures, bottom right, until closed
installRobotPanel();      // Visual sim / Hardware sim / Real, connections, E-stop (D30, R4)
installTasks();           // the 22 tasks tied to the region catalogue (D35), fixtures (D29)
installPrepare();
installSummary();         // the project summary's Approved Setup: pole, hand, start pose, UGV, room (the owner, 16 Oct)
installReview3d();        // Save & Review: the recording in 3D, its collision check, Trim (the owner, 16 Oct)
installPlacement();       // the room, the commode in it and each task's UGV placement (V32 feedback v2)
installTransitions();     // Prepare's transition trajectories and task start points (V32)
installTeach();           // existing recordings first (D31), drag teach, Save & Review
installSim();             // the 3D robot in the simulation pane, on the 5 ms clock
installTune();            // tuning with the optimiser, limits, target time (D24, D36)
installRealTest();
installAdvanced();
installViewer();          // taught .txt files: upload, check, replay in 3D
installFullSeq();         // Merge + Full Sequence: tabs S1 Merge & Upload, S2 Full & Partial Sequence (the owner, 16 Oct)
installActivity();        // last: the page's audit trail goes to the server, its Activity Logs page reads it
document.documentElement.dataset.lite = 'ready';
