"""Lite's commands to the real arms, through ALIX's own robot code (the owner's decision of 14
October: Lite does every write and setup step that teaching and running need). One RealArm per
connected controller, on the link's connection (robot_link.ArmLink):

- the pole and the hand to a file's stage, ALIX's way (``staging.stage_arm``: the hand opened when
  the pole must move, the arm checked at rest, the pole, then the hand, each read back), at 50 %
  speed. Real only: the controller cannot simulate them, so in Hardware sim they are taken as
  arrived (robot_standin.py);
- the controller's run mode, SIM or REAL, verified by reading it back (``runmode``);
- drag teaching: the SDK's drag-teach start and stop, and the controller's recording saved by the
  SDK itself, as the control program does, so the files are byte for byte the control program's;
- upload to a program number, with the stored size read back (``track``);
- a stored program's run: the move to its first line at 30 %, the run, and its verdict
  (``track.run_verified``);
- a move to a joint pose at 30 % (``staging.park_arm``), checked against the pose.

Every command goes to the hardware log with its return code and what it means (logbook.py), and
ALIX's own lines with it. While a command runs, the link only watches the E-stop input, so two
callers never compete for one controller (ALIX measured reply timeouts when they do).
"""
from __future__ import annotations

import tempfile
import time
from pathlib import Path

from .jobs import UserError
from .robot_link import return_text

POLE_SPEED_PCT = 50       # the pole in Real (the owner's choice, 14 Oct; ALIX itself ran it at 100 %)
HAND_SPEED_SET = 500      # the hand's SPEED_SET, of 1000: 50 %
MOVE_SPEED = 30           # % for a move to a start: what Lite's simulation models (staging.TRANSIT_FRACTION)
RUN_SPEED = 50            # ALIX's speed for a stored program's run
RESTORE_PCT = 50          # the global speed ratio left after a run or a move, as ALIX leaves it


def _words(code: str | None) -> str:
    return (code or "no reason given").replace("_", " ")


class RealArm:
    def __init__(self, link, log=None):
        self.link, self.log, self.side = link, log, link.side
        self.SIDE = link.side.upper()

    # ── plumbing
    @property
    def handle(self):
        if self.link.handle is None:
            raise UserError(f"The {self.SIDE} controller is not connected.", "Connect it in Robot & mode.")
        return self.link.handle

    @property
    def robot(self):
        return self.handle.robot

    def _alix_log(self):
        return self.log.hw_logger(self.side) if self.log is not None else (lambda *_: None)

    def _hw(self, verb, args=None, ret=None, ok=None, elapsed_s=None, text=None) -> None:
        if self.log is not None:
            self.log.hardware(self.side, verb, args=args, ret=ret, ok=ok, elapsed_s=elapsed_s, text=text)

    def _call(self, verb: str, fn, *args):
        """One SDK call, logged with its return code and what it means."""
        t0 = time.monotonic()
        try:
            out = fn(*args)
        except Exception as exc:                                                    # noqa: BLE001
            self._hw(verb, list(args), None, False, time.monotonic() - t0, f"raised {exc!r}")
            raise UserError(f"The {self.SIDE} controller did not answer {verb} ({exc}).",
                            "Check the controller's power and cable.") from None
        ret = out[0] if isinstance(out, tuple) else out
        self._hw(verb, list(args), ret, ret == 0, time.monotonic() - t0, None if ret == 0 else return_text(ret, verb))
        return out

    # ── the pole and the hand
    def stage(self, stage) -> dict:
        """The pole and the hand to ``stage`` (staging.Stage), each read back. Raises when either
        did not arrive: then nothing else may run."""
        from alix_dispatch import staging as alix_staging

        log = self._alix_log()
        with self.link.busy("staging the pole and the hand"):
            pole, hand, opened = alix_staging.open_devices(self.handle, log=log, speed_set=HAND_SPEED_SET)
            if hand is None:
                self._hw("stage", text=f"refused: {_words(opened.get('refusal'))}", ok=False)
                raise UserError(f"The {self.SIDE} hand cannot be reached (Modbus), so nothing was staged.",
                                "Check the hand's cable and power, then try again.")
            task = {"side": self.side, "pre_stroke": {"pole_m": stage.pole_mm / 1000.0, "hand_counts": list(stage.hand)}}
            out = alix_staging.stage_arm(self.handle, task, pole=pole, hand=hand, log=log, pole_speed_pct=POLE_SPEED_PCT)
        p, h = out.get("pole") or {}, out.get("hand") or {}
        found = p.get("found_m")
        self._hw("stage", args=dict(pole_mm=stage.pole_mm, hand=list(stage.hand), speed_pct=POLE_SPEED_PCT),
                 ok=bool(out.get("ok")), text=(f"pole {found * 1000:.0f} mm" if found is not None else "pole not read")
                 + f", hand {h.get('read')}" + ("" if out.get("ok") else f"; refused: {_words(out.get('refusal'))}"))
        if not out.get("ok"):
            raise UserError(f"The {self.SIDE} arm was not staged: {_words(out.get('refusal'))} "
                            f"(pole {'?' if found is None else f'{found * 1000:.0f}'} mm of {stage.pole_mm:.0f} mm). "
                            "Nothing else ran.", "Check the pole and the hand, then try again.")
        return out

    def pole_to(self, mm: float) -> dict:
        """The pole alone to ``mm`` (Advanced), at 50 %, arrival verified."""
        from alix_dispatch import peripherals

        with self.link.busy("moving the pole"):
            out = peripherals.Pole(self.robot, log=self._alix_log()).move(float(mm) / 1000.0, speed_pct=POLE_SPEED_PCT)
        self._hw("pole move", args=dict(mm=mm, speed_pct=POLE_SPEED_PCT), ret=out.get("ret"), ok=bool(out.get("ok")),
                 elapsed_s=out.get("t_s"), text=f"final {out.get('final_m')} m, verified by {out.get('verified_by')}")
        if not out.get("ok"):
            raise UserError(f"The {self.SIDE} pole did not reach {mm:.0f} mm ({out.get('error') or return_text(out.get('ret'))}).",
                            "Check the pole, then try again.")
        return out

    def hand_to(self, counts: list) -> dict:
        """The hand alone to six counts (Advanced), at 50 %, read back."""
        from alix_dispatch import peripherals

        with self.link.busy("moving the hand"):
            hand = peripherals.HandModbus(self.robot, self.side, log=self._alix_log())
            ret = hand.open(speed_set=HAND_SPEED_SET)
            if ret != 0:
                self._hw("hand open", ret=ret, ok=False, text=return_text(ret))
                raise UserError(f"The {self.SIDE} hand cannot be reached (Modbus).", "Check the hand's cable and power.")
            out = hand.command(list(counts), wait=True)
        self._hw("hand move", args=dict(counts=list(counts), speed_set=HAND_SPEED_SET), ret=out.get("ret"),
                 ok=bool(out.get("ok") and out.get("arrived")), elapsed_s=out.get("t_s"), text=f"read back {out.get('after')}")
        if not (out.get("ok") and out.get("arrived")):
            raise UserError(f"The {self.SIDE} hand did not reach its pose (it reads {out.get('after')}).",
                            "Check the hand, then try again.")
        return out

    # ── the controller's run mode
    def set_run_mode(self, mode: str) -> dict:
        """SIM or REAL, read back. Returns what to hand to ``restore_run_mode``."""
        from alix_dispatch import runmode

        with self.link.busy(f"switching to {mode}"):
            originals = runmode.apply_run_mode(runmode.SIMULATION if mode == "SIM" else runmode.REAL,
                                               {self.side: self.robot})
        self._hw(f"run mode {mode}", ok=originals is not None, text=None if originals is not None else "not taken (read back)")
        if originals is None:
            raise UserError(f"The {self.SIDE} controller did not switch to its {mode} mode (read back), so nothing ran.",
                            "Check the controller's errors on its pendant.")
        return originals

    def restore_run_mode(self, originals: dict) -> None:
        from alix_dispatch import runmode

        failed = runmode.restore_run_modes(originals, {self.side: self.robot})
        if failed and self.log is not None:
            self.log.alert("warn", f"The controller's run mode could not be set back ({', '.join(map(str, failed))}).",
                           hint="Check its mode on the pendant before the next run.", side=self.side, source="runmode")

    # ── drag teaching, as the control program does it
    def teach_start(self) -> None:
        with self.link.busy("starting drag teaching"):
            ret = self._call("rm_start_drag_teach", self.robot.rm_start_drag_teach, 1)
        if ret != 0:
            raise UserError(f"The {self.SIDE} controller refused drag teaching ({return_text(ret, 'rm_start_drag_teach')}).",
                            "Check the arm's errors on its pendant, then try again.")

    def teach_stop(self, path: Path) -> int | None:
        """Leave drag teaching and save the controller's recording at ``path`` (written by the
        SDK, as the control program's files are). Returns its number of points."""
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        with self.link.busy("saving the recording"):
            ret = self._call("rm_stop_drag_teach", self.robot.rm_stop_drag_teach)
            out = self._call("rm_save_trajectory", self.robot.rm_save_trajectory, str(path))
        if ret != 0:
            raise UserError(f"The {self.SIDE} arm may still be in drag teaching ({return_text(ret, 'rm_stop_drag_teach')}).",
                            "Press STOP, or the red E-stop button, before anyone lets go of the arm.")
        r, n = out if isinstance(out, tuple) else (out, None)
        if r != 0 or not Path(path).is_file() or Path(path).stat().st_size == 0:
            raise UserError(f"The {self.SIDE} controller did not save the recording ({return_text(r, 'rm_save_trajectory')}).",
                            "Record the region again.")
        return n

    def teach_cancel(self) -> None:
        """Leave drag teaching without saving (STOP, an E-stop, a cancel, Lite closing). Never raises."""
        try:
            with self.link.busy("leaving drag teaching"):
                self._call("rm_stop_drag_teach", self.robot.rm_stop_drag_teach)
        except Exception:                                                           # noqa: BLE001
            pass

    # ── programs
    def programs(self) -> list[dict]:
        from alix_dispatch import track

        with self.link.busy("reading the program list"):
            ret, items = track.program_list(self.robot)
        self._hw("program list", ret=ret, ok=ret == 0, text=f"{len(items or [])} programs")
        if ret != 0:
            raise UserError(f"The {self.SIDE} controller's program list could not be read ({return_text(ret)}).")
        return items

    def upload(self, path: Path, program_id: int, name: str) -> dict:
        """Store a track file as program ``program_id``; the stored size is read back."""
        from alix_dispatch import track

        with self.link.busy(f"sending program {program_id}"):
            with tempfile.TemporaryDirectory() as tmp:
                proj = Path(tmp) / f"{name}.txt"
                info = track.build_project_file(Path(path), proj)
                ret, err_line, secs = track.send_project(self.robot, proj, int(program_id))
            self._hw("rm_send_project", args=dict(program_id=program_id, file=proj.name, bytes=info["bytes"]), ret=ret,
                     ok=ret == 0 and err_line == -1, elapsed_s=secs,
                     text=None if ret == 0 and err_line == -1 else f"err_line {err_line}")
            if ret != 0 or err_line != -1:
                where = " (its data length was refused)" if err_line == 0 else f" (line {err_line} was refused)" if err_line > 0 else ""
                raise UserError(f"The {self.SIDE} controller refused program {program_id}{where}: "
                                f"{return_text(ret, 'rm_send_project')}.", "Check the file, then send it again.")
            r, items = track.program_list(self.robot)
        sent = info["bytes"]
        item = next((i for i in items or [] if int(i.get("id", -1)) == int(program_id)), None)
        size = int((item or {}).get("size") or 0)
        ok = r == 0 and item is not None and abs(size - sent) <= max(64, sent // 1000)
        self._hw("program read back", args=dict(program_id=program_id), ret=r, ok=ok,
                 text=f"stored {size} bytes as {(item or {}).get('name')!r} (sent {sent})")
        if not ok:
            raise UserError(f"Program {program_id} is not confirmed on the {self.SIDE} controller "
                            f"(it reads {size} bytes, {sent} were sent).", "Send it again.")
        return dict(program_id=int(program_id), bytes=sent, points=info["points"], stored_name=item.get("name"),
                    seconds=round(secs, 1))

    def run(self, program_id: int, first_row_millideg: list, points: int, on_transition=None) -> dict:
        """Move to the program's first line at 30 % and run it; ALIX judges whether it completed."""
        from alix_dispatch import track

        with self.link.busy(f"running program {program_id}"):
            out = track.run_verified(self.robot, save_id=int(program_id), band_s=track.expected_band_s(int(points)),
                                     first_row_millideg=list(first_row_millideg), points=int(points), speed=RUN_SPEED,
                                     restore_pct=RESTORE_PCT, movej_speed=MOVE_SPEED, log=self._alix_log(),
                                     on_transition=on_transition)
        self._hw("run program", args=dict(program_id=program_id, points=points, speed=RUN_SPEED, movej_speed=MOVE_SPEED),
                 ret=out.get("run_ret"), ok=bool(out.get("ok")), elapsed_s=out.get("duration_s"),
                 text=f"verdict {out.get('verdict')}, gap {out.get('gap_deg')} deg, replay {out.get('replay_s')} s")
        return out

    def move_to(self, target_millideg: list) -> dict:
        """A move to a joint pose at 30 %, checked against it (within 2 degrees)."""
        from alix_dispatch import staging as alix_staging
        from alix_dispatch import track

        deg = [v / 1000.0 for v in target_millideg]
        with self.link.busy("moving to the start"):
            out = alix_staging.park_arm(self.handle, {}, rest_deg=deg, rest_speed=MOVE_SPEED, restore_speed=RESTORE_PCT,
                                        log=self._alix_log())
            gap = track.start_gap_deg(self.robot, list(target_millideg))
        ok = bool(out.get("ok")) and gap is not None and gap <= alix_staging.START_TOLERANCE_DEG
        self._hw("move to pose", args=dict(deg=[round(d, 2) for d in deg], speed=MOVE_SPEED), ok=ok,
                 text=f"gap {gap} deg" + ("" if out.get("ok") else f"; refused: {_words(out.get('refusal'))}"))
        if not ok:
            raise UserError(f"The {self.SIDE} arm did not reach the start (" + (_words(out.get("refusal")) if not out.get("ok")
                            else f"{'?' if gap is None else round(gap, 1)} degrees away") + ").",
                            "Check the arm on its pendant, then try again.")
        return dict(gap_deg=gap)
