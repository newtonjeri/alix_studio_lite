"""Rough FK of the V6 trajectory files to see WHERE each program works (read-only).
Usage: python3 -I v6_fk_segments.py <urdf> <v6_dir>
- joint values in the .txt are 0.001 deg (RealMan convention)
- segments split at the BRIDGES_TEST_ONLY sequences found inside merged files
- pole heights taken from the *_FINAL.json workflow (mm -> sliding plate m)
- frame reported: <side>_glove_frame_2 in base_link (x fwd to fixture, y left, z up)
"""
import sys, os, json, glob, math
import xml.etree.ElementTree as ET
import numpy as np

urdf, base = sys.argv[1], sys.argv[2]
root = ET.parse(urdf).getroot()
joints = {}
child_of = {}
for j in root.findall('joint'):
    o = j.find('origin')
    xyz = [float(v) for v in (o.get('xyz', '0 0 0').split() if o is not None else '0 0 0'.split())]
    rpy = [float(v) for v in (o.get('rpy', '0 0 0').split() if o is not None else '0 0 0'.split())]
    a = j.find('axis')
    axis = [float(v) for v in a.get('xyz').split()] if a is not None else [1, 0, 0]
    rec = dict(name=j.get('name'), type=j.get('type'), parent=j.find('parent').get('link'),
               child=j.find('child').get('link'), xyz=xyz, rpy=rpy, axis=axis)
    joints[rec['name']] = rec
    child_of[rec['child']] = rec


def rot_rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def rot_axis(ax, q):
    ax = np.array(ax, float)
    ax = ax / np.linalg.norm(ax)
    K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    return np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * (K @ K)


def chain(link):
    out = []
    while link in child_of:
        j = child_of[link]
        out.append(j)
        link = j['parent']
    return list(reversed(out))


def fk(target, qmap):
    T = np.eye(4)
    for j in chain(target):
        A = np.eye(4)
        A[:3, :3] = rot_rpy(*j['rpy'])
        A[:3, 3] = j['xyz']
        T = T @ A
        q = qmap.get(j['name'], 0.0)
        if j['type'] in ('revolute', 'continuous'):
            M = np.eye(4)
            M[:3, :3] = rot_axis(j['axis'], q)
            T = T @ M
        elif j['type'] == 'prismatic':
            M = np.eye(4)
            M[:3, 3] = np.array(j['axis']) * q
            T = T @ M
    return T


def load(p):
    out = []
    with open(p) as fh:
        for line in fh:
            s = line.strip()
            if s:
                out.append(tuple(json.loads(s)['point']))
    return out


bridges = {}
for b in glob.glob(os.path.join(base, 'BRIDGES_TEST_ONLY', '*.txt')):
    bridges[os.path.basename(b)] = load(b)


def find_sub(pts, sub):
    L = len(sub)
    for i in range(0, len(pts) - L + 1):
        if pts[i] == sub[0] and pts[i:i + L] == sub:
            return i
    return -1


wf = {}
for side in ('LEFT', 'RIGHT'):
    data = json.load(open(os.path.join(base, f'{side}_V6_70SERIES_FINAL.json')))
    pole = None
    for st in data['steps']:
        if st['type'] == 'pole':
            pole = st['mm']
        if st['type'] == 'program':
            wf[(side, st['name'])] = (st['id'], pole, st.get('dual_rule'))

STRIDE = 50  # 0.25 s
for side, pre in (('LEFT', 'L'), ('RIGHT', 'R')):
    tgt = f'{pre}_glove_frame_2'
    for f in sorted(glob.glob(os.path.join(base, side, '*.txt'))):
        name = os.path.basename(f)
        pts = load(f)
        pid, pole, rule = wf.get((side, name), (None, None, None))
        # split at bridges belonging to this side
        cuts = []
        for bn, bp in bridges.items():
            if not bn.startswith(pre + '_'):
                continue
            i = find_sub(pts, bp)
            if i >= 0:
                cuts.append((i, i + len(bp), bn))
        cuts.sort()
        segs = []
        start = 0
        for (a, b, bn) in cuts:
            segs.append(('prog', start, a))
            segs.append(('bridge:' + bn.split('_BRIDGE')[0], a, b))
            start = b
        segs.append(('prog', start, len(pts)))
        print(f'=== {side} {name} id={pid} pole={pole}mm rule={rule} points={len(pts)} ({len(pts)*0.005:.2f}s)')
        k = 0
        for kind, a, b in segs:
            if kind == 'prog':
                k += 1
            sl = pts[a:b]
            if not sl:
                continue
            P = []
            for p in sl[::STRIDE] + [sl[-1]]:
                q = {f'{pre}_joint{i+1}': math.radians(v / 1000.0) for i, v in enumerate(p)}
                q[f'{pre}_sliding_plate_joint'] = (pole or 0) / 1000.0
                T = fk(tgt, q)
                P.append(T[:3, 3])
            P = np.array(P)
            lab = f'seg{k}' if kind == 'prog' else kind
            mn, mx, me = P.min(0), P.max(0), P.mean(0)
            print(f'  {lab:28} {a:6d}-{b:6d} {len(sl)*0.005:6.2f}s  x[{mn[0]:.2f},{mx[0]:.2f}] y[{mn[1]:+.2f},{mx[1]:+.2f}] z[{mn[2]:.2f},{mx[2]:.2f}]  mean=({me[0]:.2f},{me[1]:+.2f},{me[2]:.2f})')
