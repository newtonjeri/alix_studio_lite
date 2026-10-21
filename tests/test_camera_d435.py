"""The wrist D435 as Lite reads it (server/camera_d435.py), against a stand-in of pyrealsense2: the native depth as it
comes, the depth filtered (spatial, then temporal) and aligned to the colour frame as ALIX's driver does, each answer a
frame that arrived after the question, and the camera kept open while a task or a check uses it."""
import threading
import time
from types import SimpleNamespace

import numpy as np

from alix_studio_lite.server.camera_d435 import D435


class _Frame:
    def __init__(self, arr, ts):
        self.arr, self.ts = arr, ts

    def __bool__(self):
        return self.arr is not None

    def get_data(self):
        return self.arr

    def get_timestamp(self):
        return self.ts


class _Set:
    def __init__(self, depth, colour, tag=""):
        self.depth, self.colour, self.tag = depth, colour, tag

    def get_depth_frame(self):
        return self.depth

    def get_color_frame(self):
        return self.colour

    def as_frameset(self):
        return self


class FakeRS:
    """Just what camera_d435 calls: a device, a pipeline making framesets at 30 Hz, the filters, the aligner."""
    stream = SimpleNamespace(depth="depth", color="color")
    format = SimpleNamespace(z16="z16", bgr8="bgr8")
    camera_info = SimpleNamespace(name="name", serial_number="serial")
    option = SimpleNamespace(enable_auto_exposure="auto_exposure")

    def __init__(self):
        self.applied, self.resets, self.n = [], 0, 0
        self.streams, self.options = [], []
        rs = self

        class Sensor:
            def supports(self, opt):
                return opt == "auto_exposure"

            def set_option(self, opt, value):
                rs.options.append((opt, value))

        class Device:
            def get_info(self, what):
                return {"name": "Intel RealSense D435", "serial": "123"}[what]

            def hardware_reset(self):
                rs.resets += 1

            def first_depth_sensor(self):
                return SimpleNamespace(get_depth_scale=lambda: 0.001)

            def query_sensors(self):
                return [Sensor(), Sensor()]                     # the depth module and the RGB camera

        def intr(w, h, f):
            return SimpleNamespace(fx=f, fy=f, ppx=w / 2, ppy=h / 2, width=w, height=h)

        class Profile:
            def get_device(self):
                return Device()

            def get_stream(self, s):
                i = intr(848, 480, 425.0) if s == "depth" else intr(1280, 720, 910.0)
                return SimpleNamespace(as_video_stream_profile=lambda: SimpleNamespace(get_intrinsics=lambda: i))

        class Pipeline:
            def start(self, cfg):
                self.on = True
                return Profile()

            def try_wait_for_frames(self, ms):
                time.sleep(1 / 30)
                rs.n += 1
                d = np.full((480, 848), rs.n, dtype=np.uint16)
                c = np.zeros((720, 1280, 3), dtype=np.uint8)
                return True, _Set(_Frame(d, rs.n * 33.3), _Frame(c, rs.n * 33.3))

            def stop(self):
                self.on = False

        def filt(name):
            class F:
                def process(self, fs):
                    rs.applied.append(name)
                    return _Set(_Frame(fs.depth.arr + 1000, fs.depth.ts), fs.colour, fs.tag + name)
            return F

        class Align:
            def __init__(self, to):
                assert to == "color"

            def process(self, fs):
                rs.applied.append("align")
                return _Set(_Frame(np.full((720, 1280), int(fs.depth.arr[0, 0]), dtype=np.uint16), fs.depth.ts), fs.colour)

        self.pipeline = Pipeline
        self.config = lambda: SimpleNamespace(enable_device=lambda s: None, enable_stream=lambda *a: rs.streams.append(a))
        self.context = lambda: SimpleNamespace(query_devices=lambda: [Device()])
        self.spatial_filter, self.temporal_filter, self.align = filt("spatial"), filt("temporal"), Align


def test_the_depth_as_it_comes_and_aligned_to_the_colour_filtered_as_alixs_driver(monkeypatch):
    monkeypatch.setattr("alix_studio_lite.server.camera_d435.WARM_UP_S", 0.0)
    rs = FakeRS()
    cam = D435(rs=rs)
    assert cam.available()["ok"]
    with cam.using():
        first = cam.shot()
        assert rs.resets == 1                                   # once, as ALIX's bringup (initial_reset)
        assert rs.streams == [("depth", 848, 480, "z16", 30), ("color", 1280, 720, "bgr8", 30)]   # the spec's profiles
        assert rs.options == [("auto_exposure", 1.0)] * 2       # auto exposure on both sensors, as the spec asks
        assert first.depth.shape == (480, 848) and first.depth[0, 0] < 1000          # native: not filtered
        assert first.aligned.shape == (720, 1280) and first.aligned[0, 0] == first.depth[0, 0] + 2000   # spatial + temporal
        assert first.colour.shape == (720, 1280, 3)
        assert (first.intrinsics.fx, first.colour_intrinsics.fx) == (425.0, 910.0)
        assert rs.applied[:3] == ["spatial", "temporal", "align"]
        second = cam.shot()
        assert second.depth[0, 0] > first.depth[0, 0]          # a frame that came after the question
        assert second.asked >= first.arrived
        with cam.using():                                       # a camera check while a task holds it
            cam.shot()
        assert cam._pipe is not None                           # still open for the task
    assert cam._pipe is None                                   # closed after the last user
    with cam.using():
        cam.shot()
    assert rs.resets == 1                                      # not reset again in the session


def test_the_profiles_and_settings_are_alix_s_camera_specification():
    """What Lite starts the camera with is what ALIX's bringup starts it with: the specification's active profiles
    and its hardware extra_args (read through ALIX's own loader, not copied)."""
    import yaml

    from alix_studio_lite import vendor_paths
    from alix_studio_lite.server.camera_d435 import spec

    doc = yaml.safe_load((vendor_paths.ALIX / "core/python/alix_perception_core/src/alix_perception_core/config/cameras/d435.yaml").read_text())
    s = spec()
    assert s["depth"] == tuple(doc["streams"]["depth"]["active_profile"])
    assert s["colour"] == tuple(doc["streams"]["color"]["active_profile"])
    extra = doc["targets"]["hardware"]["extra_args"]
    assert (s["reset"], s["align"], s["spatial"], s["temporal"]) == tuple(
        extra[k] == "true" for k in ("initial_reset", "align_depth.enable", "spatial_filter.enable", "temporal_filter.enable"))
    assert s["auto_exposure"] == (extra["rgb_camera.enable_auto_exposure"] == "true")
    assert s["args"] == {k: str(v) for k, v in extra.items()} | {
        "depth_module.depth_profile": "%dx%dx%d" % s["depth"], "rgb_camera.color_profile": "%dx%dx%d" % s["colour"]}


def test_no_frame_in_time_is_said(monkeypatch):
    monkeypatch.setattr("alix_studio_lite.server.camera_d435.WARM_UP_S", 0.0)
    rs = FakeRS()
    cam = D435(rs=rs)
    stall = threading.Event()
    real = rs.pipeline.try_wait_for_frames

    def slow(self, ms):
        if stall.is_set():
            time.sleep(ms / 1000.0)
            return False, None
        return real(self, ms)

    rs.pipeline.try_wait_for_frames = slow
    with cam.using():
        cam.shot()
        stall.set()
        time.sleep(0.15)                                        # the frame already on its way lands first
        try:
            cam.shot(wait_s=0.3)
            raise AssertionError("a frame was invented")
        except Exception as exc:                                # noqa: BLE001
            assert "sent no frame within 0.3 s" in str(exc)


def test_the_top_bars_camera_circle(monkeypatch):
    """The owner, 21 Oct: the camera's circle beside the arms' (status, for status.js): green with a D435 plugged in, or
    open with its frames coming; red without one, or with pyrealsense2 missing, the reason said."""
    from alix_studio_lite.server.jobs import UserError

    monkeypatch.setattr("alix_studio_lite.server.camera_d435.WARM_UP_S", 0.0)
    monkeypatch.setattr("alix_studio_lite.server.camera_d435.STATUS_EVERY_S", 0.05)
    rs = FakeRS()
    cam = D435(rs=rs)
    try:
        st = cam.status()
        assert st == {"level": "good", "text": "Intel RealSense D435 (serial 123) plugged in"}
        with cam.using():
            cam.shot()
            time.sleep(0.2)
            assert cam.status() == {"level": "good", "text": "Intel RealSense D435 (serial 123), streaming"}
        rs.context = lambda: SimpleNamespace(query_devices=lambda: [])               # unplugged
        time.sleep(0.2)
        assert cam.status() == {"level": "none", "text": "no D435 is plugged into this computer"}
    finally:
        cam.stop_status()
    bare = D435()

    def missing():
        raise UserError("The wrist camera cannot be read: pyrealsense2 is not installed on this computer.")

    monkeypatch.setattr(bare, "_module", missing)
    try:
        st = bare.status()
        assert st["level"] == "none" and "pyrealsense2 is not installed" in st["text"]
    finally:
        bare.stop_status()

