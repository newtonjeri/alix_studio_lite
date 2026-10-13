"""Per file: min margin to RM75 joint limits (deg), max per-tick joint rate at 5 ms and
at the 5.5545 ms player clock vs ratings, and how many 'spikes' (second-difference
outliers) appear. Read-only."""
import json, sys, os

LIM = [178, 130, 178, 135, 178, 128, 360]
RATE = [180, 180, 225, 225, 225, 225, 225]

def load(p):
    out = []
    with open(p, 'rb') as fh:
        for line in fh.read().replace(b'\r', b'\n').split(b'\n'):
            s = line.strip()
            if s:
                out.append(json.loads(s)['point'])
    return out

for p in sys.argv[1:]:
    pts = load(p)
    n = len(pts)
    margin = 1e9
    mj = -1
    for r in pts:
        for j in range(7):
            m = LIM[j] - abs(r[j] / 1000.0)
            if m < margin:
                margin, mj = m, j + 1
    # rates
    maxrate_ratio = 0.0
    rj = -1
    for i in range(1, n):
        for j in range(7):
            d = abs(pts[i][j] - pts[i - 1][j]) / 1000.0
            ratio = (d / 0.005) / RATE[j]
            if ratio > maxrate_ratio:
                maxrate_ratio, rj = ratio, j + 1
    # second difference (accel proxy) in deg/s^2 at 5 ms
    maxacc = 0.0
    aj = -1
    for i in range(1, n - 1):
        for j in range(7):
            a = abs(pts[i + 1][j] - 2 * pts[i][j] + pts[i - 1][j]) / 1000.0 / (0.005 ** 2)
            if a > maxacc:
                maxacc, aj = a, j + 1
    print('%-60s n=%6d  min_limit_margin=%7.2f deg (J%d)  peak_rate=%5.1f%% of rating (J%d) at 5ms  peak_acc=%8.0f deg/s^2 (J%d)' % (
        os.path.basename(p), n, margin, mj, maxrate_ratio * 100, rj, maxacc, aj))
