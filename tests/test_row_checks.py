"""A lid or seat row's ``checks`` (the user, 16 Nov): the camera checks of its task the run asks, by name; the others are
not asked, to save the camera's time (Open the lid's mid-lift and end checks the run's only looks). A row without it asks
them all, as before; what a kept check finds decides as it always has (a failed end check's run again, the seat that came
up with the lid). The same in the row check, the plan, the Runner and the simulation (test_s2_row_checks_emulated.py)."""
import pytest

from alix_studio_lite import vendor_paths

vendor_paths.activate()

from alix_studio_lite.server import flow, mechanism, workflow  # noqa: E402
from alix_studio_lite.server.gap_check import Answer  # noqa: E402
from alix_studio_lite.server.jobs import UserError  # noqa: E402

LID_ONLY = ["verify_lid_lifted", "verify_lid_open"]


class FakeExec:
    """Records what the runner asks for; the camera answers from a script (test_mechanism.py's)."""

    def __init__(self, answers):
        self.answers, self.calls = list(answers), []

    def approach(self, plan, targets, at, words=None, entries=None, speeds=None):
        self.calls.append(("approach", plan.name, at))

    def move(self, e, bridge=False):
        self.calls.append(("move", e.stage, e.side, e.kind))

    def hold(self, seconds, label):
        self.calls.append(("hold", label, seconds))

    def ask(self, action, expect, observe=False):
        if observe:
            if not (self.answers and isinstance(self.answers[0], dict) and "also" in self.answers[0]):
                return Answer(success=True, fixture_state="no_" + expect, state_confidence=1.0)
            state = self.answers.pop(0)["also"]
        else:
            state = self.answers.pop(0)
        self.calls.append(("ask", action.get("stage"), expect, state))
        return Answer(success=state is not None, fixture_state=state or "not_lifted", state_confidence=1.0 if state else 0.0)

    def check_stop(self):
        pass


def asked(ex):
    return [c[1] for c in ex.calls if c[0] == "ask"]


def test_each_tasks_checks_are_named_by_where_they_come():
    roles = {t: [(c["stage"], c["role"]) for c in mechanism.task_checks(t)] for t in ("lid_open", "seat_open", "lid_close")}
    assert roles["lid_open"] == [("verify_lid_closed", "start"), ("verify_lid_lifted", "lift"), ("verify_lid_open", "end")]
    assert roles["seat_open"] == [("verify_lid_open", "start"), ("verify_seat_closed", "start"), ("verify_seat_lifted", "lift"),
                                  ("verify_seat_open", "end")]
    assert roles["lid_close"] == [("verify_lid_open", "start"), ("verify_lid_closed", "end")]
    assert mechanism.task_checks("lid_open")[1]["words"] == "mid-lift check: lid lifted"
    assert [c["stage"] for c in mechanism.Plan("lid_open").view()["camera_checks"]] == ["verify_lid_closed"] + LID_ONLY


def test_a_row_keeps_the_checks_it_names_in_the_plans_order_and_a_file_round_trips():
    rows = [{"type": "mechanism", "task": "lid_open", "checks": ["verify_lid_open", "verify_lid_lifted"], "uid": "a1"},
            {"type": "mechanism", "task": "seat_open", "checks": [], "uid": "a2"},
            {"type": "mechanism", "task": "lid_close", "uid": "a3"}]
    got = workflow.check_steps(rows, "left")
    assert got[0]["checks"] == LID_ONLY                       # the plan's order, whatever the file's
    assert got[1]["checks"] == [] and "checks" not in got[2]  # [] asks none; none given asks all, as before
    again = workflow.from_document("left", workflow.document("left", got))
    assert [r.get("checks") for r in again] == [LID_ONLY, [], None] and [r["uid"] for r in again] == ["a1", "a2", "a3"]


@pytest.mark.parametrize("value, words", [(["verify_seat_open"], "has no camera check 'verify_seat_open'"),
                                          ("verify_lid_open", "must be a list"), ([3], "must be a list")])
def test_a_row_naming_a_check_its_task_lacks_is_refused_with_the_tasks_checks(value, words):
    with pytest.raises(UserError) as e:
        workflow.check_steps([{"type": "mechanism", "task": "lid_open", "checks": value}], "right")
    assert "RIGHT row 1: Open the lid" in str(e.value) and words in str(e.value)
    assert "verify_lid_lifted (mid-lift check: lid lifted)" in e.value.hint and 'give [] to keep none' in e.value.hint


def test_both_arms_rows_of_a_task_keep_the_same_checks():
    lid = {"type": "mechanism", "task": "lid_open"}
    left = workflow.check_steps([dict(lid, checks=LID_ONLY)], "left")
    with pytest.raises(flow.PlanError) as e:
        flow.build(left, workflow.check_steps([lid], "right"))
    assert "the camera checks it keeps differ between the arms (LEFT: verify_lid_lifted, verify_lid_open; RIGHT: all)" in str(e.value)
    plan = flow.build(left, workflow.check_steps([dict(lid, checks=LID_ONLY)], "right"))
    assert plan[0]["kind"] == "mechanism" and plan[0]["checks"] == LID_ONLY
    assert "checks" not in flow.build(workflow.check_steps([lid], "left"), workflow.check_steps([lid], "right"))[0]


def test_a_check_not_kept_goes_with_the_settle_before_it():
    entries = mechanism.Plan("lid_open").entries
    kept, gone = mechanism.keep_only_checks(entries, ["verify_lid_open"])
    assert [a["stage"] for a in gone] == ["verify_lid_closed", "verify_lid_lifted"]
    assert not any(a.get("stage") == "settle_before_check" for e in kept for a in e.pre_actions)
    kept, gone = mechanism.keep_only_checks(entries, LID_ONLY)
    assert [a["stage"] for a in gone] == ["verify_lid_closed"]
    assert sum(a.get("stage") == "settle_before_check" for e in kept for a in e.pre_actions) == 1   # its check stays
    assert mechanism.keep_only_checks(entries, None)[0] == list(entries)


def test_the_runner_asks_only_the_kept_checks():
    # the run's first task (its start check would be asked) keeping its mid-lift and end checks: those two only
    ex = FakeExec(["lid_lifted", "lid_open"])
    out = mechanism.Runner(ex, keep=LID_ONLY).run("lid_open")
    assert asked(ex) == LID_ONLY and out["plans"][0]["outcome"] == "done" and not ex.answers
    assert ("hold", "settle_before_check", 0.5) in ex.calls          # the settle before the kept mid-lift check stays
    for task in ("seat_open", "lid_close"):                          # [] asks none, nor holds for the camera
        ex = FakeExec([])
        out = mechanism.Runner(ex, keep=[]).run(task)
        assert asked(ex) == [] and out["plans"][0]["outcome"] == "done"
        assert not any(c[0] == "hold" and c[1] == "settle_before_check" for c in ex.calls)
    ex = FakeExec(["lid_closed", "lid_lifted", "lid_open"])          # no setting: all three, as before
    mechanism.Runner(ex).run("lid_open")
    assert asked(ex) == ["verify_lid_closed"] + LID_ONLY


def test_what_a_kept_check_finds_still_decides():
    # Open the lid's end check fails: the same verdict as with every check kept
    said = []
    for keep, answers in ((LID_ONLY, ["lid_lifted", "lid_closed"]), (None, ["lid_closed", "lid_lifted", "lid_closed"])):
        with pytest.raises(UserError) as e:
            mechanism.Runner(FakeExec(answers), keep=keep).run("lid_open")
        said.append((str(e.value), e.value.code))
    assert said[0] == said[1] == ("verify_lid_open: NOT lid_open (on_fail abort).", "escalate")
    # a missed lift: the kept mid-lift check's re-lifts (the plan's rungs) as before, the re-lift's own check asked
    ex2 = FakeExec(["not_lifted", "lid_lifted", "lid_open"])
    mechanism.Runner(ex2, keep=LID_ONLY).run("lid_open")
    assert asked(ex2) == ["verify_lid_lifted", "verify_lid_lifted", "verify_lid_open"]
    # the seat came up with the lid (Open the lid's end frames): Open the seat, next in the sequence, is not run
    ex = FakeExec(["lid_lifted", "lid_open", {"also": "seat_lifted"}])
    out = mechanism.Runner(ex, keep=LID_ONLY, next_task="seat_open").run("lid_open")
    assert out["seat_after_lid"] == "up" and asked(ex) == LID_ONLY + ["verify_seat_with_lid"]
    # ... and with no Open the seat next, Close the seat follows with all its own checks (its start just seen)
    ex = FakeExec(["lid_lifted", "lid_open", {"also": "seat_lifted"}, "seat_closed"])
    out = mechanism.Runner(ex, keep=LID_ONLY).run("lid_open")
    assert [p["plan"] for p in out["plans"]] == ["lid_open", "seat_close"]
    assert asked(ex) == LID_ONLY + ["verify_seat_with_lid", "verify_seat_closed"]
