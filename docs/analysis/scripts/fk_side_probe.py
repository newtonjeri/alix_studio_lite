"""FK of .txt files under BOTH arm chains at a given pole height. Read-only."""
import json, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fk_segments_lib import UrdfModel
URDF = '/home/newtonjeri/alix_ws/src/alix/core/python/alix_workspace/src/alix_workspace/urdf/alix.urdf'
m = UrdfModel.from_file(URDF)
pole = float(sys.argv[1])
def load(p):
    out = []
    for line in open(p, 'rb').read().replace(b'\r', b'\n').split(b'\n'):
        s = line.strip()
        if s:
            out.append(json.loads(s)['point'])
    return np.array(out, dtype=float)
for path in sys.argv[2:]:
    Q = load(path)
    idx = np.arange(0, len(Q), 10)
    for side in ('L', 'R'):
        ch = m.chain('base_link', side + '_glove_frame_2')
        mv = [j.name for j in m.movable_joints(ch)]
        full = np.zeros((len(idx), len(mv)))
        for k, jn in enumerate(mv):
            full[:, k] = pole if 'sliding' in jn else np.radians(Q[idx, int(jn.split('joint')[-1]) - 1] / 1000.0)
        P = m.fk_batch(ch, full)[:, :3, 3]
        print('%-60s %s-chain pole %.3f: x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f  start (%.3f,%.3f,%.3f)' % (
            os.path.basename(path), side, pole, P[:,0].min(), P[:,0].max(), P[:,1].min(), P[:,1].max(), P[:,2].min(), P[:,2].max(), *P[0]))
