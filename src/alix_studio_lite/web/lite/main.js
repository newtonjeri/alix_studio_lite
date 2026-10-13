// ALIX Studio Lite's bridge for the designer's page (web/index.html, V19 Project Manager).
//
// The page is the designer's file, unchanged; it runs first and shows its log-in screen. This
// module then replaces the page's template functions with versions that use the backend, one
// group per file. docs/UI_V19_CONTRACT.md lists every replaced function and what is still a
// template. Each install function keeps the page's own screen updates and adds the backend call.

import { installAuth } from './auth.js';
import { installFixes } from './fixes.js';
import { installLayout } from './layout.js';
import { installProjects } from './projects.js';
import { installSafety } from './safety.js';
import { installStatus } from './status.js';

installSafety();          // first: STOP must reach the robot before and without a log-in
installLayout();          // ...and stay on the screen (the page's top bar needs 1,610 px)
installFixes();
installProjects();
installAuth();
installStatus();
document.documentElement.dataset.lite = 'ready';
