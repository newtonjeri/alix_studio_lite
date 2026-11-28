#!/usr/bin/env python3
"""Re-read the lab's 27 Nov IMU captures (Resources/Logs/2026-11-27/imu) with the current tools, printing only: the
lab's own report files stay as they were written. Run from Lite's root:

    PYTHONPATH=src .venv/bin/python docs/imu/checks/2026-11-27_lab_step_a/recheck.py
"""
import json
import sys
from pathlib import Path

import numpy as np
from scipy.signal import find_peaks

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "tools"))
import imu_n100 as n  # noqa: E402
import imu_runs as r  # noqa: E402

LOGS = ROOT / "Resources/Logs/2026-11-27/imu"


def main() -> int:
    r._lite()
    for d in sorted(LOGS.iterdir()):
        if not (d / "raw.bin").exists():
            continue
        meta = json.loads((d / "meta.json").read_text())
        buf, chunks = n.load(d)
        lines, rep = n.analyse(buf, chunks, meta.get("test") or "frames")
        imu = [n.decode_imu(f.payload) for f in n.scan(buf)[0] if f.type == n.T_IMU]
        g = np.abs(np.array([x["gyro"] for x in imu])) * n.RAD
        a = np.abs(np.array([x["acc"] for x in imu]))
        print(f"== {d.name} ({meta.get('note')}): ok={rep['ok']}, powered {imu[1]['ts_us'] / 1e6:.0f} s, "
              f"max gyro {g.max():.1f} deg/s, max accel {a.max():.2f} m/s^2")
        for line in lines:
            if line.startswith(("FAIL", "CHECK")) or "frame counter" in line or "left over" in line:
                print("   " + line)

    nc = LOGS / "163559_noise_c_arms_holding"
    for fc in (30.0, 8.0):
        s = r.signals(r.load_imu(nc), (180, 0, 0), 1e9, fc)
        print(f"noise floor (c) at {fc:g} Hz: {np.sqrt(np.mean(s['rock'] ** 2)):.4f} deg/s RMS")

    for tap in ("162455_mounted_tap", "162707_mounted_tap"):
        s = r.signals(r.load_imu(LOGS / tap), (180, 0, 0), 2.0, 190.0)
        w, dt = s["rates"], s["dt"]
        pk, _ = find_peaks(s["rock"], height=2.0, distance=int(1.5 / dt))
        for k in pk:
            ax = int(np.argmax(np.abs(w[k, :2])))
            x = w[k + int(0.03 / dt):k + int(0.53 / dt), ax]
            x = x - x.mean()
            f = np.fft.rfftfreq(len(x), dt)
            p = np.abs(np.fft.rfft(x * np.hanning(len(x)))) ** 2
            hi = f >= 30
            print(f"{tap} {s['t'][k] - s['t'][0]:5.2f} s peak {s['rock'][k]:6.1f} deg/s: "
                  f"{100 * p[(f > 1) & (f < 30)].sum() / p[f > 1].sum():4.1f} % of the 30-530 ms after it below 30 Hz; "
                  f"above, strongest {f[hi][np.argmax(p[hi])]:.0f} Hz")
    return 0


if __name__ == "__main__":
    sys.exit(main())
