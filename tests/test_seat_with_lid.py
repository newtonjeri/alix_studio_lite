"""Open the lid reads the seat on its end check's frames (mechanism.SEAT_WITH_LID; the owner, 30 Oct): a seat that came
up with the lid is closed again by Close the seat right after, its own check that the seat is up asked even in a
sequence; when the next lid or seat task is Open the seat, both are skipped; a seat the camera cannot judge leaves the
next task to its own start check. S2 links its stages the same way (workflow.link_after_lid)."""
from alix_studio_lite.server import mechanism, workflow

from test_mechanism import FakeExec

OPENED = ["lid_closed", "lid_lifted", "lid_open"]          # Open the lid on its own: closed, lifted, open at its end


def ran(out):
    return [(p["plan"], p["outcome"]) for p in out["plans"]]


def asked(ex):
    return [(c[1], c[3]) for c in ex.calls if c[0] == "ask"]


def test_a_seat_that_came_up_with_the_lid_is_closed_right_after():
    ex = FakeExec(OPENED + [{"also": "seat_lifted"}, "seat_open", "seat_closed"])
    out = mechanism.Runner(ex).run("lid_open")
    assert ran(out) == [("lid_open", "done"), ("seat_close", "done")] and out["seat_after_lid"] == "up"
    assert asked(ex)[3:] == [("verify_seat_with_lid", "seat_lifted"), ("verify_seat_open", "seat_open"),
                             ("verify_seat_closed", "seat_closed")]
    assert [c["verdict"] for c in out["checks"] if c["check"] == "verify_seat_with_lid"] == ["up"]


def test_a_seat_left_down_needs_nothing_more():
    ex = FakeExec(OPENED + [{"also": "seat_closed"}])
    out = mechanism.Runner(ex).run("lid_open")
    assert ran(out) == [("lid_open", "done")] and out["seat_after_lid"] == "down"


def test_in_a_sequence_the_follow_up_still_asks_that_the_seat_is_up():
    """S2 runs its tasks without their start checks (the sequence's order made the state): not the follow-up's, whose
    state the order did not make."""
    ex = FakeExec(["lid_lifted", "lid_open", {"also": "seat_lifted"}, "seat_open", "seat_closed"])
    out = mechanism.Runner(ex, start_checks=False).run("lid_open")
    assert ran(out) == [("lid_open", "done"), ("seat_close", "done")]
    assert ("verify_seat_open", "seat_open") in asked(ex)


def test_an_unknown_seat_runs_close_the_seat_which_skips_itself_when_the_seat_is_down():
    ex = FakeExec(OPENED + [{"also": None}, "seat_closed"])
    out = mechanism.Runner(ex, start_checks=False).run("lid_open")
    assert out["seat_after_lid"] == "unknown"
    assert ran(out) == [("lid_open", "done"), ("seat_close", "skipped")]          # its start check: NOT seat_open -> skip


def test_open_the_seat_next_skips_both():
    """The owner, 30 Oct: the seat came up and the sequence opens it next: neither Close the seat nor Open the seat."""
    ex = FakeExec(OPENED + [{"also": "seat_lifted"}])
    out = mechanism.Runner(ex, next_task="seat_open").run("lid_open")
    assert ran(out) == [("lid_open", "done")] and out["seat_after_lid"] == "up"
    nxt = dict(kind="mechanism", task="seat_open", title="Open the seat")
    plan = [dict(kind="mechanism", task="lid_open", title="Open the lid"), dict(kind="parallel"), nxt]
    assert workflow.next_lid_seat_task(plan, 0) is nxt
    assert workflow.link_after_lid(nxt, out["seat_after_lid"]).startswith("Open the seat: not run, the seat came up")
    assert nxt["held"] == "seat_open"
    unknown = dict(kind="mechanism", task="seat_open", title="Open the seat")
    workflow.link_after_lid(unknown, "unknown")
    assert unknown.get("ask_start") and not unknown.get("held")                   # it asks its own start checks
    for seat in ("down", None):
        other = dict(kind="mechanism", task="seat_open", title="Open the seat")
        assert workflow.link_after_lid(other, seat) == "" and not other.get("held")


def test_open_the_seat_after_its_own_prerequisite_is_skipped_when_the_seat_came_up():
    """Open the seat on its own finds the lid closed: Open the lid runs first (its prerequisite), lifts the seat with the
    lid, and Open the seat is not run again (its outcome holds)."""
    ex = FakeExec(["lid_closed",                                   # Open the seat: lid open? no -> Open the lid first
                   "lid_closed", "lid_lifted", "lid_open", {"also": "seat_lifted"}])
    out = mechanism.Runner(ex).run("seat_open")
    assert ran(out) == [("lid_open", "done"), ("seat_open", "skipped")]


def test_simulations_assume_the_seat_stayed_down():
    class Sim:
        def __init__(self):
            self.n = 0

        def ask(self, action, expect, observe=False):
            return mechanism.assumed("no_" + expect) if observe else mechanism.assumed(expect)

    r = mechanism.Runner(Sim())
    r.queue, r.k = ["lid_open"], 0
    assert r._seat_with_lid() == "down"
