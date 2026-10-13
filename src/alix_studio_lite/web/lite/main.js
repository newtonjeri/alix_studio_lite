// ALIX Studio Lite's bridge for the designer's page (web/index.html, V19 Project Manager).
//
// The page is the designer's file, unchanged; it runs first and shows its log-in screen. This
// module then replaces the page's template functions with versions that use the backend, one
// group per file. docs/UI_V19_CONTRACT.md lists every replaced function and what is still a
// template. Each install function keeps the page's own screen updates and adds the backend call.

import { installAdvanced } from './advanced.js';
import { installAuth } from './auth.js';
import { installFixes } from './fixes.js';
import { installLayout } from './layout.js';
import { installPrepare } from './prepare.js';
import { installProjects } from './projects.js';
import { installRealTest } from './realtest.js';
import { installRobotPanel } from './robotpanel.js';
import { installSafety } from './safety.js';
import { installSim } from './sim.js';
import { installStatus } from './status.js';
import { installTasks } from './tasks.js';
import { installTeach } from './teach.js';
import { installTune } from './tune.js';
import { installViewer } from './viewer.js';

installSafety();          // first: STOP must reach the robot before and without a log-in
installLayout();          // ...and stay on the screen (the page's top bar needs 1,610 px)
installFixes();
installProjects();
installAuth();
installStatus();
installRobotPanel();      // Visual sim / Hardware sim / Real, connections, E-stop (D30, R4)
installTasks();           // the 22 tasks tied to the region catalogue (D35), fixtures (D29)
installPrepare();
installTeach();           // existing recordings first (D31), drag teach, Save & Review
installSim();             // the 3D robot in the simulation pane, on the 5 ms clock
installTune();            // tuning with the optimiser, limits, target time (D24, D36)
installRealTest();
installAdvanced();
installViewer();          // taught .txt files: upload, check, replay in 3D
document.documentElement.dataset.lite = 'ready';
