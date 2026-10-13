import sys, re, glob, os
import xml.etree.ElementTree as ET
ns = {'a':'http://schemas.openxmlformats.org/drawingml/2006/main',
      'p':'http://schemas.openxmlformats.org/presentationml/2006/main',
      'r':'http://schemas.openxmlformats.org/officeDocument/2006/relationships'}
d = sys.argv[1]
files = sorted(glob.glob(os.path.join(d, 'slide*.xml')), key=lambda f:int(re.findall(r'(\d+)', os.path.basename(f))[0]))
EMU = 914400.0
for f in files:
    n = int(re.findall(r'(\d+)', os.path.basename(f))[0])
    t = ET.parse(f).getroot()
    items = []
    for sp in t.iter('{%s}sp' % ns['p']):
        off = sp.find('.//a:xfrm/a:off', ns)
        ext = sp.find('.//a:xfrm/a:ext', ns)
        txt = ' '.join((x.text or '') for x in sp.iter('{%s}t' % ns['a'])).strip()
        if not txt: continue
        x = int(off.get('x'))/EMU if off is not None else -1
        y = int(off.get('y'))/EMU if off is not None else -1
        w = int(ext.get('cx'))/EMU if ext is not None else -1
        items.append((round(y,2), round(x,2), round(w,2), txt))
    pics = []
    for pic in t.iter('{%s}pic' % ns['p']):
        off = pic.find('.//a:xfrm/a:off', ns)
        ext = pic.find('.//a:xfrm/a:ext', ns)
        blip = pic.find('.//a:blip', ns)
        rid = blip.get('{%s}embed' % ns['r']) if blip is not None else None
        if off is not None:
            pics.append((round(int(off.get('x'))/EMU,2), round(int(off.get('y'))/EMU,2), round(int(ext.get('cx'))/EMU,2), rid))
    print(f'--- slide {n}')
    for it in sorted(items, key=lambda z:(z[1], z[0])):
        print(f'   x={it[1]:5} y={it[0]:5} w={it[2]:5} | {it[3]}')
    print(f'   pictures: {len(pics)} ' + '; '.join(f'x={p[0]} y={p[1]} w={p[2]} {p[3]}' for p in sorted(pics)))
