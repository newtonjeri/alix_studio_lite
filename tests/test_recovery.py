"""The controller's errors, read and cleared as the operator's tools do them (butterfli-realman-alix's
dual_arm_common, vendored as it is; the owner, 16 Oct: "Lite should be able to clear errors of any kind,
similar to how the vendor web GUI does"), and what Arm Drag Mode refuses: on ALIX's emulator, in this
process, with its own fault injection."""
from contextlib import contextmanager
from types import SimpleNamespace

IP = "192.168.1.10"


class FakeLink:
    """What RealArm needs of robot_link.ArmLink: the side, the address, the handle, and busy()."""

    def __init__(self, robot):
        self.side, self.ip, self.handle = "left", IP, SimpleNamespace(robot=robot)

    @contextmanager
    def busy(self, what):
        yield


def test_errors_are_read_and_cleared_as_the_operators_tools_do():
    from alix_studio_lite import vendor_paths

    vendor_paths.activate()
    from alix_dispatch import emulated_program
    from alix_emulator import rm_emulator

    from alix_studio_lite.server.robot_real import RealArm

    raw = rm_emulator.RoboticArm()
    raw.rm_create_robot_arm(IP, 8080)
    real = RealArm(FakeLink(emulated_program.attach(raw, IP)))
    ctrl = rm_emulator.emu_controller(IP)
    try:
        assert real.errors()["clean"] and real.drag_ready() == []
        ctrl.joint_err_flags[1] = 16384                  # a Position Command Step warning on J2
        ctrl.joint_en[5] = 0                             # J6 not enabled (under-voltage, as on 8 Aug)
        e = real.errors()
        assert not e["clean"] and e["disabled"] == [6] and e["joints"][0]["joint"] == 2
        assert "Position Command Step" in e["joints"][0]["text"]
        assert any("J6 is not enabled" in why for why in real.drag_ready())
        out = real.clear_errors()                        # the system's, then each flagged joint's
        assert out["joints"] == [] and out["disabled"] == [6] and not out["clean"]   # enabling is not a clear
        ctrl.joint_en[5] = 1
        assert real.errors()["clean"] and real.drag_ready() == []
        assert real.read_sensitivity() is None           # ALIX's emulator has no such call: not reported
    finally:
        ctrl.joint_err_flags[:] = [0] * 7
        ctrl.joint_en[:] = [1] * 7
