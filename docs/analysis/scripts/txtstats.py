import json, sys, os

DT = 0.005  # assumed 5 ms cadence

def load(path):
    pts = []
    bad = 0
    crlf = False
    keys = set()
    with open(path, 'rb') as fh:
        raw = fh.read()
    if b'\r\n' in raw[:2000]:
        crlf = True
    for line in raw.decode('utf-8', 'replace').splitlines():
        s = line.strip()
        if not s:
            continue
        try:
            obj = json.loads(s)
        except Exception:
            bad += 1
            continue
        if isinstance(obj, dict):
            keys.update(obj.keys())
            p = obj.get('point')
        else:
            p = obj
        if not isinstance(p, list):
            bad += 1
            continue
        pts.append(p)
    return pts, bad, crlf, keys

def stats(path):
    pts, bad, crlf, keys = load(path)
    n = len(pts)
    if n == 0:
        return dict(file=path, n=0)
    nj = set(len(p) for p in pts)
    allint = all(isinstance(v, int) for p in pts for v in p)
    J = len(pts[0])
    mins = [min(p[j] for p in pts) for j in range(J)]
    maxs = [max(p[j] for p in pts) for j in range(J)]
    maxd = 0.0
    maxd_idx = -1
    maxd_j = -1
    perj_maxd = [0.0] * J
    dup = 0
    for i in range(1, n):
        a, b = pts[i - 1], pts[i]
        if a == b:
            dup += 1
        for j in range(J):
            d = abs(b[j] - a[j]) / 1000.0
            if d > perj_maxd[j]:
                perj_maxd[j] = d
            if d > maxd:
                maxd = d
                maxd_idx = i
                maxd_j = j
    # longest run of identical points (idle)
    run = 1
    best = 1
    for i in range(1, n):
        if pts[i] == pts[i - 1]:
            run += 1
            best = max(best, run)
        else:
            run = 1
    return dict(
        file=os.path.relpath(path, '/home/newtonjeri/alix_studio_lite_ws'),
        bytes=os.path.getsize(path),
        n=n, bad=bad, crlf=crlf, keys=sorted(keys), joints=sorted(nj), allint=allint,
        dur_s=round(n * DT, 3),
        max_step_deg=round(maxd, 3), max_step_at=maxd_idx, max_step_joint=maxd_j + 1,
        perj_max_step_deg=[round(x, 3) for x in perj_maxd],
        min_deg=[round(x / 1000.0, 2) for x in mins],
        max_deg=[round(x / 1000.0, 2) for x in maxs],
        dup_pairs=dup, longest_identical_run=best,
        first=pts[0], last=pts[-1],
    )

if __name__ == '__main__':
    for p in sys.argv[1:]:
        s = stats(p)
        print(json.dumps(s))
