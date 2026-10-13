import json, sys, os, glob

base = sys.argv[1]

def load(p):
    out = []
    with open(p, 'r', encoding='utf-8') as fh:
        for line in fh:
            s = line.strip()
            if s:
                out.append(tuple(json.loads(s)['point']))
    return out

merged = {}
for side in ('LEFT', 'RIGHT'):
    for f in sorted(glob.glob(os.path.join(base, side, '*.txt'))):
        merged[os.path.relpath(f, base)] = load(f)

for b in sorted(glob.glob(os.path.join(base, 'BRIDGES_TEST_ONLY', '*.txt'))):
    bp = load(b)
    found = []
    for name, pts in merged.items():
        # search for the first point then check whole sequence
        L = len(bp)
        for i in range(0, len(pts) - L + 1):
            if pts[i] == bp[0] and pts[i:i + L] == bp:
                found.append((name, i, i + L - 1, len(pts)))
                break
    if not found:
        # try approx: find nearest point to first and last
        best = None
        for name, pts in merged.items():
            for i, p in enumerate(pts):
                d = max(abs(a - c) for a, c in zip(p, bp[0]))
                if best is None or d < best[0]:
                    best = (d, name, i)
        found = [('NOT_EXACT nearest-to-first', best)]
    print(os.path.basename(b), len(bp), found)
