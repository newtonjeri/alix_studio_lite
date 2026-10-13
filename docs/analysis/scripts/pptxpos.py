"""Print every text shape (and picture) of each slide with its position (EMU -> cm),
so left/right attribution can be decided by x-position. Read-only."""
import os, re, sys
import xml.etree.ElementTree as ET

base = sys.argv[1]
NS = {
    'p': 'http://schemas.openxmlformats.org/presentationml/2006/main',
    'a': 'http://schemas.openxmlformats.org/drawingml/2006/main',
    'r': 'http://schemas.openxmlformats.org/officeDocument/2006/relationships',
}
EMU_CM = 360000.0

def slide_num(fn):
    return int(re.findall(r'\d+', fn)[0])

def rels(slide_fn):
    rp = os.path.join(base, 'ppt', 'slides', '_rels', slide_fn + '.rels')
    out = {}
    if os.path.exists(rp):
        t = ET.parse(rp).getroot()
        for r in t:
            out[r.get('Id')] = r.get('Target')
    return out

def xfrm_of(el):
    x = el.find('.//a:xfrm', NS)
    if x is None:
        return None
    off = x.find('a:off', NS)
    ext = x.find('a:ext', NS)
    if off is None or ext is None:
        return None
    return (int(off.get('x')) / EMU_CM, int(off.get('y')) / EMU_CM,
            int(ext.get('cx')) / EMU_CM, int(ext.get('cy')) / EMU_CM)

files = sorted([f for f in os.listdir(os.path.join(base, 'ppt', 'slides')) if f.endswith('.xml')], key=slide_num)
for fn in files:
    root = ET.parse(os.path.join(base, 'ppt', 'slides', fn)).getroot()
    rl = rels(fn)
    print(f'=== slide {slide_num(fn)} ({fn})')
    items = []
    for sp in root.iter('{%s}sp' % NS['p']):
        texts = []
        for para in sp.findall('.//a:p', NS):
            t = ''.join((r.text or '') for r in para.findall('.//a:t', NS)).strip()
            if t:
                texts.append(t)
        pos = xfrm_of(sp)
        name = sp.find('.//p:cNvPr', NS)
        nm = name.get('name') if name is not None else ''
        if texts:
            items.append(('TXT', pos, nm, ' / '.join(texts)))
    for pic in root.iter('{%s}pic' % NS['p']):
        blip = pic.find('.//a:blip', NS)
        rid = blip.get('{%s}embed' % NS['r']) if blip is not None else None
        pos = xfrm_of(pic)
        name = pic.find('.//p:cNvPr', NS)
        nm = name.get('name') if name is not None else ''
        items.append(('PIC', pos, nm, rl.get(rid, rid)))
    items.sort(key=lambda it: ((it[1] or (0, 0, 0, 0))[1], (it[1] or (0, 0, 0, 0))[0]))
    for kind, pos, nm, txt in items:
        ps = 'pos=?' if pos is None else 'x=%.1f y=%.1f w=%.1f h=%.1f' % pos
        print(f'  {kind} {ps} [{nm}] {txt}')
