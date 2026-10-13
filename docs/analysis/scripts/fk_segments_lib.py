import math
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

