"""Offline FK (no robot, no network): where the glove frame is, in base_link, for every
V6 segment, at the pole height the FINAL workflow sets before that program.
Also the commode mesh bounding boxes at the scene.yaml pose. Read-only."""
import json, math, os, struct, sys
import warnings
warnings.filterwarnings('ignore')

ALIX = '/home/newtonjeri/alix_ws/src/alix'
import numpy as np
import xml.etree.ElementTree as ET


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


class _J:
    pass


class UrdfModel:
    # minimal numpy-only FK (scipy in this environment is ABI-broken against numpy 2.x)
    @classmethod
    def from_file(cls, path):
        m = cls()
        root = ET.parse(path).getroot()
        m.by_child = {}
        for je in root.findall('joint'):
            j = _J()
            j.name = je.get('name')
            j.type = je.get('type')
            j.parent = je.find('parent').get('link')
            j.child = je.find('child').get('link')
            o = je.find('origin')
            xyz = [float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]
            rpy = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
            T = np.eye(4)
            T[:3, :3] = _rpy(*rpy)
            T[:3, 3] = xyz
            j.origin = T
            a = je.find('axis')
            ax = np.array([float(v) for v in (a.get('xyz') if a is not None else '1 0 0').split()])
            j.axis = ax / np.linalg.norm(ax)
            j.movable = j.type in ('revolute', 'continuous', 'prismatic')
            m.by_child[j.child] = j
        return m

    def chain(self, base, tip):
        out = []
        cur = tip
        while cur != base:
            j = self.by_child[cur]
            out.append(j)
            cur = j.parent
        out.reverse()
        return out

    def movable_joints(self, ch):
        return [j for j in ch if j.movable]

    def fk_batch(self, ch, Q):
        n = Q.shape[0]
        T = np.broadcast_to(np.eye(4), (n, 4, 4)).copy()
        qi = 0
        for j in ch:
            T = T @ j.origin
            if j.movable:
                t = Q[:, qi]
                M = np.broadcast_to(np.eye(4), (n, 4, 4)).copy()
                if j.type == 'prismatic':
                    M[:, :3, 3] = j.axis[None, :] * t[:, None]
                else:
                    k = j.axis
                    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
                    M[:, :3, :3] = (np.eye(3)[None] + np.sin(t)[:, None, None] * K[None]
                                    + (1 - np.cos(t))[:, None, None] * (K @ K)[None])
                T = T @ M
                qi += 1
        return T

V6 = '/home/newtonjeri/alix_studio_lite_ws/Trajectories/DUAL_ARM_V6_PROGRAM_IDS_70_SERIES'
URDF = os.path.join(ALIX, 'core/python/alix_workspace/src/alix_workspace/urdf/alix.urdf')
MODELS = os.path.join(ALIX, 'ros/orchestration/alix_tasks/config/commode_cleaning/commode_c/models')
FIX_POSE = np.array([1.005, 0.0, 0.42])

model = UrdfModel.from_file(URDF)
chains = {}
for side, tip in (('L', 'L_glove_frame_2'), ('R', 'R_glove_frame_2')):
    ch = model.chain('base_link', tip)
    mv = [j.name for j in model.movable_joints(ch)]
    chains[side] = (ch, mv)
    print(side, 'movable joints in chain:', mv)

def stl_bounds(path):
    with open(path, 'rb') as fh:
        data = fh.read()
    if data[:5].lower() == b'solid' and b'facet' in data[:400]:
        vs = []
        for line in data.decode('ascii', 'replace').splitlines():
            line = line.strip()
            if line.startswith('vertex'):
                vs.append([float(x) for x in line.split()[1:4]])
        v = np.array(vs)
    else:
        n = struct.unpack('<I', data[80:84])[0]
        arr = np.frombuffer(data[84:84 + n * 50], dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
        v = arr['v'].reshape(-1, 3).astype(float)
    return v.min(0), v.max(0)

for comp in ('body', 'lid', 'seat'):
    p = os.path.join(MODELS, comp + '.stl')
    if os.path.exists(p):
        lo, hi = stl_bounds(p)
        print('commode %-4s bbox (base_link, closed pose, standoff 0.25): x %.3f..%.3f  y %.3f..%.3f  z %.3f..%.3f' % (
            comp, lo[0] + FIX_POSE[0], hi[0] + FIX_POSE[0], lo[1] + FIX_POSE[1], hi[1] + FIX_POSE[1], lo[2] + FIX_POSE[2], hi[2] + FIX_POSE[2]))

def load(p):
    out = []
    with open(p, 'rb') as fh:
        for line in fh.read().replace(b'\r', b'\n').split(b'\n'):
            s = line.strip()
            if s:
                out.append(json.loads(s)['point'])
    return np.array(out, dtype=float)

SEGS = {
    'LEFT/L_P01P02.txt': [('P1', 0, 3274), ('br', 3275, 3526), ('P2', 3527, 7004)],
    'LEFT/L_P03P05.txt': [('P3', 0, 3145), ('br', 3146, 3423), ('P4', 3424, 7172), ('br', 7173, 7450), ('P5', 7451, 10982)],
    'LEFT/L_P06P07.txt': [('P6', 0, 3542), ('br', 3543, 3864), ('P7', 3865, 6743)],
    'LEFT/M3_OPNST01.txt': [('P8', 0, None)],
    'LEFT/M3_OPNST02.txt': [('P9', 0, None)],
    'LEFT/L_P10P12.txt': [('P10', 0, 3224), ('br', 3225, 3276), ('P11', 3277, 6694), ('br', 6695, 6736), ('P12', 6737, 10213)],
    'LEFT/R15_BUPFUL.txt': [('P13', 0, None)],
    'RIGHT/R_P01P03.txt': [('P1', 0, 3274), ('br', 3275, 3526), ('P2', 3527, 7003), ('br', 7004, 7281), ('P3', 7282, 10427)],
    'RIGHT/R2_LIDTOP.txt': [('P4', 0, None)],
    'RIGHT/R10_HNGR.txt': [('P5', 0, None)],
    'RIGHT/R_P06P07.txt': [('P6', 0, 3542), ('br', 3543, 3864), ('P7', 3865, 6743)],
    'RIGHT/M3_ASOPN01.txt': [('P8', 0, None)],
    'RIGHT/M3_ASOPN02.txt': [('P9', 0, None)],
    'RIGHT/R_P10P12.txt': [('P10', 0, 3224), ('br', 3225, 3276), ('P11', 3277, 6694), ('br', 6695, 6736), ('P12', 6737, 10213)],
    'RIGHT/R_SPILLAGE.txt': [('P13', 0, None)],
}
POLE = {
    'LEFT/L_P01P02.txt': 0.150, 'LEFT/L_P03P05.txt': 0.050, 'LEFT/L_P06P07.txt': 0.150,
    'LEFT/M3_OPNST01.txt': 0.150, 'LEFT/M3_OPNST02.txt': 0.150, 'LEFT/L_P10P12.txt': 0.150,
    'LEFT/R15_BUPFUL.txt': 0.050,
    'RIGHT/R_P01P03.txt': 0.050, 'RIGHT/R2_LIDTOP.txt': 0.150, 'RIGHT/R10_HNGR.txt': 0.050,
    'RIGHT/R_P06P07.txt': 0.150, 'RIGHT/M3_ASOPN01.txt': 0.150, 'RIGHT/M3_ASOPN02.txt': 0.150,
    'RIGHT/R_P10P12.txt': 0.150, 'RIGHT/R_SPILLAGE.txt': 0.050,
}

extra = sys.argv[1:]  # optional extra files as side:pole_m:path
for spec in extra:
    side, pole, path = spec.split(':', 2)
    SEGS[path] = [('all', 0, None)]
    POLE[path] = float(pole)

for rel, segs in SEGS.items():
    side = 'L' if rel.startswith('LEFT') or rel.startswith('L:') else ('R' if rel.startswith('RIGHT') else None)
    path = rel if os.path.isabs(rel) else os.path.join(V6, rel)
    if side is None:
        side = 'L' if '/left' in rel.lower() else 'R'
    ch, mv = chains[side]
    Q = load(path)
    print('=== %s (pole %.3f m, %d pts)' % (rel, POLE[rel], len(Q)))
    for name, a, b in segs:
        b = len(Q) - 1 if b is None else b
        idx = np.arange(a, b + 1, 10)
        if idx[-1] != b:
            idx = np.append(idx, b)
        q = np.radians(Q[idx] / 1000.0)
        full = np.zeros((len(idx), len(mv)))
        for k, jn in enumerate(mv):
            if 'sliding_plate' in jn:
                full[:, k] = POLE[rel]
            else:
                jnum = int(jn.split('joint')[-1])
                full[:, k] = q[:, jnum - 1]
        T = model.fk_batch(ch, full)
        P = T[:, :3, 3]
        print('  %-3s rows %5d-%5d (%6.2f s)  start (%.3f, %.3f, %.3f)  end (%.3f, %.3f, %.3f)  x %.3f..%.3f  y %.3f..%.3f  z %.3f..%.3f  path %.2f m' % (
            name, a, b, (b - a + 1) * 0.005, *P[0], *P[-1], P[:, 0].min(), P[:, 0].max(), P[:, 1].min(), P[:, 1].max(),
            P[:, 2].min(), P[:, 2].max(), float(np.linalg.norm(np.diff(P, axis=0), axis=1).sum())))
