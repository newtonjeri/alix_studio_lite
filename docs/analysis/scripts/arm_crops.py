"""Per-arm crop boxes for the task slides of the regions deck (slides 5-25).
Prints the box in slide inches and in pixels of a <dpi> render; optionally renders crops with pdftoppm.
Usage: python3 -I arm_crops.py <slides_dir> <pdf> <out_dir> <dpi> [--render]"""
import sys, os, re, glob, subprocess
import xml.etree.ElementTree as ET

A = 'http://schemas.openxmlformats.org/drawingml/2006/main'
P = 'http://schemas.openxmlformats.org/presentationml/2006/main'
ns = {'a': A, 'p': P}
EMU = 914400.0
SW, SH = 13.333, 7.5
d, pdf, out, dpi = sys.argv[1], sys.argv[2], sys.argv[3], int(sys.argv[4])
render = '--render' in sys.argv


def bbox(el):
    x = el.find('.//a:xfrm', ns)
    off, ext = x.find('a:off', ns), x.find('a:ext', ns)
    bx, by, bw, bh = int(off.get('x')) / EMU, int(off.get('y')) / EMU, int(ext.get('cx')) / EMU, int(ext.get('cy')) / EMU
    rot = int(x.get('rot', '0')) / 60000.0
    if rot in (90.0, 270.0):  # rotated picture: swap extents around centre
        cx, cy = bx + bw / 2, by + bh / 2
        bx, by, bw, bh = cx - bh / 2, cy - bw / 2, bh, bw
    return [bx, by, bx + bw, by + bh]


def union(boxes):
    return [min(b[0] for b in boxes), min(b[1] for b in boxes), max(b[2] for b in boxes), max(b[3] for b in boxes)]


def task_of(t):
    return [' '.join((x.text or '') for x in sp.iter('{%s}t' % A)).strip() for sp in t.iter('{%s}sp' % P)
            if ' '.join((x.text or '') for x in sp.iter('{%s}t' % A)).strip().startswith('TASK')]


for f in sorted(glob.glob(os.path.join(d, 'slide*.xml')), key=lambda s: int(re.findall(r'(\d+)', os.path.basename(s))[0])):
    n = int(re.findall(r'(\d+)', os.path.basename(f))[0])
    if n < 5:
        continue
    t = ET.parse(f).getroot()
    pics, items = [], {'L': [], 'R': []}
    for pic in t.iter('{%s}pic' % P):
        pics.append(bbox(pic))
    for sp in t.iter('{%s}sp' % P):
        txt = ' '.join((x.text or '') for x in sp.iter('{%s}t' % A)).strip()
        if txt.startswith('TASK'):
            continue
        spPr = sp.find('p:spPr', ns)
        raw = ET.tostring(spPr).decode() if spPr is not None else ''
        b = bbox(sp)
        if 'LEFT ARM' in txt and 'RIGHT' not in txt or '4472C4' in raw or '7A73FF' in raw or txt.startswith('L-'):
            items['L'].append(b)
        elif 'RIGHT ARM' in txt or 'ED7D31' in raw or 'accent2' in raw or txt.startswith('R-'):
            items['R'].append(b)
    # attach pictures to the arm(s) whose overlay/label centre falls in the picture's x-range
    owner = []
    for pb in pics:
        arms = set()
        for arm in 'LR':
            for b in items[arm]:
                cx, cy = (b[0] + b[2]) / 2, (b[1] + b[3]) / 2
                if pb[0] <= cx <= pb[2] and pb[1] - 0.8 <= cy <= pb[3] + 1.2:
                    arms.add(arm)
        owner.append(arms)
    shared = any(len(o) == 2 for o in owner) or any(len(o) == 0 for o in owner)
    names = []
    if shared:
        boxes = pics + items['L'] + items['R']
        names.append(('LR', union(boxes)))
    else:
        for arm in 'LR':
            boxes = [pb for pb, o in zip(pics, owner) if arm in o] + items[arm]
            if boxes:
                names.append((arm, union(boxes)))
    tasks = '+'.join(x.replace('TASK:', '').strip() for x in task_of(t))
    for arm, b in names:
        m = 0.08
        x0, y0 = max(0, b[0] - m), max(0, b[1] - m)
        x1, y1 = min(SW, b[2] + m), min(SH, b[3] + 0.4)  # captions wrap to 2 lines below their box
        px = [round(v * dpi) for v in (x0, y0, x1 - x0, y1 - y0)]
        fn = f'crop-s{n:02d}-T{tasks.replace("+", "_")}-{arm}.png'
        print(f's{n:02d} task {tasks:6} {arm:2} {"shared-image" if shared else "own-image   "} in[{x0:.2f},{y0:.2f},{x1:.2f},{y1:.2f}] px@{dpi}(x,y,w,h)={px} -> {fn}')
        if render:
            stem = os.path.join(out, fn[:-4])
            subprocess.run(['pdftoppm', '-r', str(dpi), '-f', str(n), '-l', str(n), '-x', str(px[0]), '-y', str(px[1]),
                            '-W', str(px[2]), '-H', str(px[3]), '-png', '-singlefile', pdf, stem], check=True)
