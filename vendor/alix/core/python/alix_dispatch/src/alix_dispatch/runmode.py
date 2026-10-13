"""Contract C9 — the controller's SIM/REAL run mode, and nothing else.

C9's ladder is **emulator -> SIM -> REAL**, and it is explicit that "SIM and
REAL both require hardware access": SIM is the CONTROLLER's own simulation
mode, not Gazebo and not the ROS stack. The SDK pair is
`rm_set_arm_run_mode(mode)` / `rm_get_arm_run_mode()` with 0 = SIMULATION,
1 = REAL.

Pure primitives: no SDK import, no socket, no session state. Ported 1:1 from
`dual_arm_common.apply_run_mode / restore_run_modes / report_run_modes /
mode_label` (C17's rule that our implementation and Newton's agree), keyed by
side string because alix has no upstream `ConceptArm` object.

TWO DELIBERATE DIVERGENCES from upstream, both marked D-1 / D-2 below. Both
tighten a promise upstream cannot keep; neither changes what a successful run
does.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple

SIMULATION = 0
REAL = 1


class RunModeError(RuntimeError):
    """A run mode could not be engaged, verified, or agreed across the arms."""


def mode_label(mode: Optional[int]) -> str:
    """Human name for a mode value; anything unknown is 'as-found'."""
    return {0: "SIMULATION", 1: "REAL"}.get(mode, "as-found")


def parse_mode_arg(value: Optional[str]) -> Optional[int]:
    """`--mode` argument -> 0 / 1 / None.

    `None` means AS-FOUND: run in whatever mode the arm is already in. That is
    a legitimate third state, not a missing value, and it is why every caller
    below distinguishes `{}` from `None` by identity rather than truthiness.

    Spelled exactly as `dual_arm_common.parse_mode_arg`, so `--mode` reads the
    same here as in every other script on the ladder.
    """
    if value is None:
        return None
    text = str(value).strip().upper()
    if text in ("SIM", "SIMULATION", "0"):
        return SIMULATION
    if text in ("REAL", "1"):
        return REAL
    raise SystemExit("usage: --mode SIM|REAL  (got %r)" % value)


def read_run_mode(robot) -> Tuple[int, Optional[int]]:
    """`(ret, mode)` from the arm, with every failure folded to one shape.

    The SDK asymmetry here is the easiest possible mis-unpack and it is worth
    naming: the SETTER returns a bare `int`, the GETTER returns `(ret, mode)`.
    Reading the setter's return as a mode would make every verification pass.
    """
    try:
        ret, mode = robot.rm_get_arm_run_mode()
        return int(ret), (int(mode) if ret == 0 else None)
    except Exception:
        return -1, None


def apply_run_mode(target: Optional[int],
                   robots: Dict[str, object]) -> Optional[Dict[str, Optional[int]]]:
    """Engage `target` on every arm, VERIFIED BY READBACK.

        target is None   -> {}               as-found; restore is a no-op
        engaged+verified -> {side: original} hand this to restore_run_modes
        ANY refusal      -> None             AFTER rolling back; abort

    Callers MUST distinguish `{}` from `None` **by identity** — `{}` is falsy,
    so `if not originals:` would abort the legitimate as-found path.

    THE SETTER'S RETURN IS NEVER THE GATE. The controller refuses some settings
    with a bare `ret=1` and silently leaves the previous value in force — the
    same failure mode the contract records for the speed limits, with the same
    remedy: always verify by readback. Upstream puts it plainly: "a SIM request
    that silently stayed REAL would move real metal, and vice versa a REAL
    request stuck in SIM produces phantom runs."
    """
    if target is None:
        return {}

    originals: Dict[str, Optional[int]] = {}
    ok = True
    problems: List[str] = []

    for side, robot in robots.items():
        if robot is None:
            continue
        pre_ret, pre_mode = read_run_mode(robot)
        if pre_ret != 0 or pre_mode is None:
            # D-1 (deliberate divergence). Upstream records `None` for a failed
            # pre-read and STILL sets the mode; its restore then skips that arm,
            # leaving it permanently in the requested mode. We refuse instead:
            # you may not promise a restore you cannot perform. The 2026-08-08
            # tool-frame leak persisted across a reboot for exactly this reason.
            problems.append(f"{side}: cannot read the current run mode "
                            f"(ret={pre_ret}) — refusing to change a mode we "
                            "could not restore")
            ok = False
            continue
        try:
            robot.rm_set_arm_run_mode(int(target))
        except Exception as exc:
            problems.append(f"{side}: run-mode set raised {exc!r}")
            ok = False
            continue
        originals[side] = pre_mode
        post_ret, now = read_run_mode(robot)
        if post_ret != 0 or now != target:
            problems.append(f"{side}: could not engage {mode_label(target)} "
                            f"(ret={post_ret} mode={now})")
            ok = False

    if ok:
        return originals

    # Every arm is attempted before rolling back — a partial engage across two
    # controllers is the state we must never leave behind.
    restore_run_modes(originals, robots)
    for problem in problems:
        print(f"  [FAIL] {problem}")
    return None


def restore_run_modes(originals: Optional[Dict[str, Optional[int]]],
                      robots: Dict[str, object]) -> List[str]:
    """Teardown. NEVER raises — it runs in `finally` blocks already unwinding.

    D-2 (deliberate divergence): this re-reads and reports a restore that did
    not take. Upstream sets and moves on. A silent failed restore leaves an arm
    in SIM, and the next run in that shell is a phantom.
    """
    warnings: List[str] = []
    for side, mode in (originals or {}).items():
        robot = robots.get(side)
        if robot is None or mode is None:
            continue
        try:
            robot.rm_set_arm_run_mode(int(mode))
            ret, now = read_run_mode(robot)
            if ret != 0 or now != mode:
                warnings.append(f"{side}: run mode NOT restored to "
                                f"{mode_label(mode)} (ret={ret} mode={now})")
            else:
                print(f"  [INFO] {side}: run mode restored to {mode_label(mode)}")
        except Exception as exc:
            warnings.append(f"{side}: run-mode restore raised {exc!r}")
    return warnings


def report_run_modes(robots: Dict[str, object]) -> bool:
    """Advisory print of each arm's mode; True iff EVERY arm reads REAL.

    A SIM-mode arm executes every planned move virtually: dispatches succeed,
    events fire, and nothing physical moves — the root cause of the 2026-08-06
    "no motion at all" run.
    """
    all_real = True
    for side, robot in robots.items():
        if robot is None:
            continue
        ret, mode = read_run_mode(robot)
        print(f"  [INFO] {side}: run mode {mode_label(mode)}"
              + (f" (read ret={ret})" if ret != 0 else ""))
        if mode != REAL:
            all_real = False
    return all_real


def resolve_sim(forced: Optional[int], observed: Dict[str, Optional[int]]) -> bool:
    """Is this run a SIM run? Decides whether pole and hand are commanded.

    `forced` is `--mode`; `observed` is what each arm reads back. An UNREADABLE
    mode with no `--mode` resolves to REAL (pole and hand ARE commanded), never
    to SIM: assuming SIM on an arm that is actually REAL would skip the pole
    stage and drop the pole under a moving arm.

    Arms that disagree are refused rather than reconciled. There is no honest
    single answer for a left-SIM / right-REAL pair, and every downstream
    analyser reads one boolean.
    """
    if forced is not None:
        return forced == SIMULATION
    modes = {side: mode for side, mode in observed.items() if mode is not None}
    if not modes:
        return False
    distinct = set(modes.values())
    if len(distinct) > 1:
        raise RunModeError(
            "the arms are in different run modes "
            + ", ".join(f"{s}={mode_label(m)}" for s, m in sorted(modes.items()))
            + " — pass --mode to force one, rather than having this guess")
    return distinct.pop() == SIMULATION


def snapshot_run_mode(robot) -> dict:
    """The run mode as a RECORD FIELD — what `controller_snapshot` embeds.

    M47 (2026-09-08): `grep run_mode` over the whole track-upload path returned
    nothing, and **no record in the 157-run ledger carries a run-mode field**.
    An arm left in SIMULATION is a phantom — dispatches succeed, events fire,
    transitions look perfect, nothing moves — and it still scores `complete`.

    DO NOT infer SIM from the motor current channel: F30 measured 2150 mA of
    holding torque with the controller in SIMULATION, so "SIM zeroes current"
    is false and any detector built on it would clear a phantom arm.

    Returns ``{"ret", "mode", "label", "real"}``; `mode`/`real` are None when
    the getter could not be read, which is UNKNOWN and never "REAL".
    """
    ret, mode = read_run_mode(robot)
    return {"ret": int(ret), "mode": mode, "label": mode_label(mode),
            "real": (mode == REAL) if mode is not None else None}


def require_real(robots: Dict[str, object], *, allow_sim: bool = False) -> dict:
    """Gate a run on every arm reading REAL. Returns a verdict, never raises.

    ``{"ok", "sim", "modes": {side: snapshot}, "refusal": str|None}``.

    `allow_sim=True` lets a deliberate SIM pass through, and then `sim` is
    True so the caller can stamp `sim: true` on the record and skip the pole
    and hand stages (a SIM arm that dropped the pole for real would be worse
    than a phantom). An UNREADABLE mode is refused either way: a mode we could
    not read is not evidence of anything, and the executive would otherwise be
    deciding pole and hand commands on a guess.
    """
    modes = {}
    problems: List[str] = []
    any_sim = False
    for side, robot in sorted(robots.items()):
        if robot is None:
            continue
        snap = snapshot_run_mode(robot)
        modes[side] = snap
        if snap["mode"] is None:
            problems.append(f"{side}: run mode unreadable (ret={snap['ret']})")
        elif snap["mode"] == SIMULATION:
            any_sim = True
            if not allow_sim:
                problems.append(f"{side}: run mode is SIMULATION — every dispatch "
                                "would succeed with nothing moving; pass "
                                "--allow-sim only for a deliberate phantom run")
        elif snap["mode"] != REAL:
            # HARDENING (2026-09-08 review): the gate used to be "not None and
            # not SIMULATION", so ANY other value the controller might return —
            # a firmware that grows a third mode, a getter that hands back a
            # status word — read as REAL and let the run proceed. `mode_label`
            # already calls such a value "as-found", which is precisely not
            # evidence of a real arm. Refuse it in BOTH `allow_sim` states, the
            # same way an unreadable mode is refused: this function's whole
            # promise is "every arm reads REAL", and only REAL is REAL.
            problems.append(f"{side}: run mode reads {snap['mode']!r}, which is "
                            f"neither REAL ({REAL}) nor SIMULATION ({SIMULATION}) — "
                            "refusing to run on a mode this build does not know")
    if not modes:
        problems.append("no arms to read a run mode from")
    return {"ok": not problems, "sim": any_sim, "modes": modes,
            "refusal": ("; ".join(problems) if problems else None)}
