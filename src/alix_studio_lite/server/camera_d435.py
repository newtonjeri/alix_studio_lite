"""The right wrist's RealSense D435, read by Lite itself (pyrealsense2), with no ROS.

The owner, 18 Oct: the camera is plugged into the computer Lite runs on, and Lite is ROS 2 independent.
ALIX's bringup reads it with the realsense2_camera node (ALIX's alix_perception_core
config/cameras/d435.yaml: depth 848 x 480 z16 at 30 fps, colour 1280 x 720 at 30 fps, a hardware
reset at start); the gap check uses the NATIVE depth stream with its own intrinsics. Lite opens the
same two streams. Only one program can hold the camera: ALIX's camera node must not be running.

``shot()`` answers with a frame that arrived AFTER it was asked (the node's ``frame_max_age_s`` rule: a
frame from before the question is not the pose the question is about), waiting at most ``wait_s``.
"""
from __future__ import annotations

import threading
import time

import numpy as np

from .gap_check import Intrinsics
from .jobs import UserError

DEPTH = (848, 480, 30)                 # ALIX's d435.yaml: the native depth profile
COLOUR = (1280, 720, 30)               # colour at 720p since 2026-10-10 (USB frame corruption at 1080p)
WARM_UP_S = 1.0                        # auto exposure settles before the first check is answered


class Shot:
    """One frameset: the depth (z16 millimetres, or metres when the unit's scale is not 1 mm), its intrinsics,
    the colour frame (BGR), when they arrived and how old they were when used."""

    def __init__(self, depth, intrinsics: Intrinsics, colour, arrived: float, asked: float, camera_ms: float | None):
        self.depth, self.intrinsics, self.colour = depth, intrinsics, colour
        self.arrived, self.asked, self.camera_ms = arrived, asked, camera_ms

    @property
    def age_s(self) -> float:
        return max(0.0, time.time() - self.arrived)


class D435:
    """The camera, opened on first use and kept open while a lid or seat task runs (``close`` after it)."""

    def __init__(self, serial: str | None = None, log=None, rs=None):
        self.serial = serial
        self._log = log or (lambda _t: None)
        self._rs = rs                                    # pyrealsense2, or a stand-in in the tests
        self._pipe = None
        self._intr: Intrinsics | None = None
        self._scale = 0.001
        self._opened_at = 0.0
        self._reset_done = False
        self._lock = threading.Lock()
        self.device = None                              # name and serial, for the page and the logs

    def _module(self):
        if self._rs is None:
            try:
                import pyrealsense2 as rs                # noqa: PLC0415 - optional: only the lid and seat tasks need it
            except ImportError:
                raise UserError("The wrist camera cannot be read: pyrealsense2 is not installed on this computer.",
                                "Install it (python3 tools/setup_lite.py installs it where it can), then try again.") from None
            self._rs = rs
        return self._rs

    def available(self) -> dict:
        """What is plugged in, without opening it: {ok, devices: [{name, serial}], why}."""
        try:
            rs = self._module()
            devs = [dict(name=d.get_info(rs.camera_info.name), serial=d.get_info(rs.camera_info.serial_number))
                    for d in rs.context().query_devices()]
        except UserError as exc:
            return dict(ok=False, devices=[], why=str(exc))
        except Exception as exc:                                                    # noqa: BLE001
            return dict(ok=False, devices=[], why=f"the camera library could not list devices ({exc})")
        d435 = [d for d in devs if "D435" in d["name"] and (self.serial is None or d["serial"] == self.serial)]
        return dict(ok=bool(d435), devices=devs, why=None if d435 else "no D435 is plugged into this computer")

    def open(self) -> None:
        with self._lock:
            if self._pipe is not None:
                return
            rs = self._module()
            if not self._reset_done:                    # once per session, as ALIX's bringup (initial_reset)
                self._hardware_reset(rs)
            pipe, cfg = rs.pipeline(), rs.config()
            if self.serial:
                cfg.enable_device(self.serial)
            cfg.enable_stream(rs.stream.depth, DEPTH[0], DEPTH[1], rs.format.z16, DEPTH[2])
            cfg.enable_stream(rs.stream.color, COLOUR[0], COLOUR[1], rs.format.bgr8, COLOUR[2])
            try:
                try:
                    profile = pipe.start(cfg)
                except Exception:                                                   # noqa: BLE001 - just back from its reset
                    time.sleep(2.0)
                    profile = pipe.start(cfg)
            except Exception as exc:                                                # noqa: BLE001
                why = str(exc)
                busy = "busy" in why.lower() or "resource" in why.lower()
                raise UserError(f"The wrist camera could not be opened ({why}).",
                                "Close any other program using it (ALIX's camera node) and check its USB cable."
                                if busy else "Check its USB cable (a USB 3 port), then try again.") from None
            dev = profile.get_device()
            self._scale = float(dev.first_depth_sensor().get_depth_scale())
            i = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
            self._intr = Intrinsics(float(i.fx), float(i.fy), float(i.ppx), float(i.ppy), int(i.width), int(i.height))
            self.device = dict(name=dev.get_info(rs.camera_info.name), serial=dev.get_info(rs.camera_info.serial_number))
            self._pipe, self._opened_at = pipe, time.monotonic()
            self._log(f"camera open: {self.device['name']} {self.device['serial']}, depth {DEPTH[0]}x{DEPTH[1]} "
                      f"(fx {self._intr.fx:.1f}, scale {self._scale * 1000:g} mm), colour {COLOUR[0]}x{COLOUR[1]}")

    def _hardware_reset(self, rs) -> None:
        devs = [d for d in rs.context().query_devices()
                if self.serial is None or d.get_info(rs.camera_info.serial_number) == self.serial]
        if not devs:
            raise UserError("The wrist camera (D435) is not plugged into this computer.",
                            "Plug its USB cable into a USB 3 port, then try again.")
        devs[0].hardware_reset()
        t_end = time.monotonic() + 10.0
        time.sleep(2.0)                                  # it leaves the bus, then comes back
        while time.monotonic() < t_end and not list(rs.context().query_devices()):
            time.sleep(0.25)
        self._reset_done = True

    def shot(self, wait_s: float = 2.0) -> Shot:
        """The first frameset that arrives after this call (stale ones are dropped), within ``wait_s``."""
        self.open()
        asked = time.time()
        settle = WARM_UP_S - (time.monotonic() - self._opened_at)
        if settle > 0:
            time.sleep(settle)
        with self._lock:
            pipe = self._pipe
            while pipe.poll_for_frames():                # whatever waited in the queue is from before the question
                pass
            t_end = time.monotonic() + max(0.1, float(wait_s))
            while True:
                left = t_end - time.monotonic()
                if left <= 0:
                    raise UserError(f"The wrist camera sent no frame within {wait_s:g} s.", "Check its USB cable, then try again.")
                ok, frames = pipe.try_wait_for_frames(int(left * 1000))
                if ok:
                    break
            arrived = time.time()
            d, c = frames.get_depth_frame(), frames.get_color_frame()
            if not d:
                raise UserError("The wrist camera's frame carried no depth.", "Try again.")
            depth = np.array(d.get_data(), copy=True)    # z16, in the unit's depth scale
            if abs(self._scale - 0.001) > 1e-9:
                depth = depth.astype(np.float64) * self._scale
            colour = np.array(c.get_data(), copy=True) if c else None
            return Shot(depth, self._intr, colour, arrived, asked, float(d.get_timestamp()))

    def close(self) -> None:
        with self._lock:
            pipe, self._pipe = self._pipe, None
        if pipe is not None:
            try:
                pipe.stop()
            except Exception:                                                       # noqa: BLE001
                pass
            self._log("camera closed")
