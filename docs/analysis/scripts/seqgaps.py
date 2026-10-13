"""Walk each FINAL workflow JSON and report the joint gap between the last point of
one program file and the first point of the next (the controller's own transit move,
not collision-checked), plus the cumulative modelled timeline. Read-only."""
import json, sys, os

base = sys.argv[1]
POLE_UP = 0.059
POLE_DN = 0.097
CREEP = 1.5

def load(p):
    out = []
    with open(p, 'rb') as fh:
        for line in fh.read().replace(b'\r', b'\n').split(b'\n'):
            s = line.strip()
            if s:
                out.append(json.loads(s)['point'])
    return out

for side, jf in (('LEFT', 'LEFT_V6_70SERIES_FINAL.json'), ('RIGHT', 'RIGHT_V6_70SERIES_FINAL.json')):
    doc = json.load(open(os.path.join(base, jf)))
    print('=====', side, jf, 'format', doc['format'], 'keys', sorted(doc.keys()))
    prev = None
    prev_name = None
    t5 = 0.0
    t55 = 0.0
    pole = None
    for k, st in enumerate(doc['steps'], 1):
        typ = st['type']
        if typ == 'program':
            pts = load(os.path.join(base, side, st['name']))
            gap = None
            if prev is not None:
                d = [abs(a - b) / 1000.0 for a, b in zip(prev, pts[0])]
                gap = (round(max(d), 2), 'J%d' % (d.index(max(d)) + 1))
            dur5 = len(pts) * 0.005
            dur55 = (len(pts) - 1) * 0.0055545
            print('  %2d PROGRAM id=%s %-16s pts=%5d dur@5ms=%7.3f dur@5.55ms=%7.3f rule=%-10s gap_from_prev(%s)=%s  t_start@5ms=%7.2f' % (
                k, st['id'], st['name'], len(pts), dur5, dur55, st.get('dual_rule'), prev_name, gap, t5))
            t5 += dur5
            t55 += dur55
            prev = pts[-1]
            prev_name = st['name']
        elif typ == 'pole':
            mm = st['mm']
            if pole is None:
                est = None
            else:
                dist = abs(mm - pole) / 1000.0
                up = mm > pole
                est = CREEP + dist / (POLE_UP if up else POLE_DN)
            print('  %2d POLE %s mm (from %s) pole.py-model %s s ; V6 model %s s' % (
                k, mm, pole, None if est is None else round(est, 2),
                None if pole is None else round(abs(mm - pole) / 72.5, 3)))
            pole = mm
        else:
            print('  %2d %s %s' % (k, typ.upper(), {kk: vv for kk, vv in st.items() if kk != 'type'}))
    print('  total program time @5ms=%.2f s  @5.5545ms=%.2f s' % (t5, t55))
