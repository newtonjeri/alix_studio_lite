"""Per file: ticks over the joint rating (at 5 ms and 5.5545 ms), longest run of
over-rating ticks, and top isolated single-sample jumps (a step much larger than both
neighbours = a spike). Read-only."""
import json, sys, os

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
    res = {}
    for dt in (0.005, 0.0055545):
        over = 0
        run = 0
        best = 0
        for i in range(1, n):
            o = any(abs(pts[i][j] - pts[i - 1][j]) / 1000.0 / dt > RATE[j] for j in range(7))
            if o:
                over += 1
                run += 1
                best = max(best, run)
            else:
                run = 0
        res[dt] = (over, best)
    # isolated spikes: step_i > 3x median(neighbour steps) and > 0.5 deg
    spikes = []
    for i in range(2, n - 1):
        for j in range(7):
            s_prev = abs(pts[i - 1][j] - pts[i - 2][j])
            s_cur = abs(pts[i][j] - pts[i - 1][j])
            s_next = abs(pts[i + 1][j] - pts[i][j])
            if s_cur > 500 and s_cur > 3 * max(s_prev, s_next, 1):
                spikes.append((s_cur / 1000.0, i, j + 1))
    spikes.sort(reverse=True)
    print('%-58s over@5ms=%5d (run %4d)  over@5.55ms=%5d (run %4d)  isolated_spikes=%d top=%s' % (
        os.path.basename(p), res[0.005][0], res[0.005][1], res[0.0055545][0], res[0.0055545][1],
        len(spikes), [(round(a, 3), b, c) for a, b, c in spikes[:3]]))
