"""Dump every shape/picture/connector of each slide with position, size, fill, line colour, text and text colour.
Usage: python3 -I pptx_detail.py <slides_dir> [slide_numbers...]
Positions in inches (slide is 13.333 x 7.5 in)."""
import sys, re, glob, os
import xml.etree.ElementTree as ET

A = 'http://schemas.openxmlformats.org/drawingml/2006/main'
P = 'http://schemas.openxmlformats.org/presentationml/2006/main'
R = 'http://schemas.openxmlformats.org/officeDocument/2006/relationships'
ns = {'a': A, 'p': P, 'r': R}
EMU = 914400.0

d = sys.argv[1]
only = set(int(x) for x in sys.argv[2:])


def colour(el):
    if el is None:
        return None
    s = el.find('a:solidFill', ns)
    if s is None:
        if el.find('a:noFill', ns) is not None:
            return 'none'
        if el.find('a:gradFill', ns) is not None:
            return 'grad'
        return None
    c = s.find('a:srgbClr', ns)
    if c is not None:
        alpha = c.find('a:alpha', ns)
        return '#' + c.get('val') + (f"@{int(alpha.get('val'))//1000}%" if alpha is not None else '')
    c = s.find('a:schemeClr', ns)
    if c is not None:
        mods = ','.join(f"{m.tag.split('}')[1]}={m.get('val')}" for m in c)
        return 'scheme:' + c.get('val') + (f'({mods})' if mods else '')
    c = s.find('a:prstClr', ns)
    if c is not None:
        return 'prst:' + c.get('val')
    return 'solid?'


def rels(n):
    f = os.path.join(d, '_rels', f'slide{n}.xml.rels')
    out = {}
    if os.path.exists(f):
        t = ET.parse(f).getroot()
        for r in t:
            out[r.get('Id')] = os.path.basename(r.get('Target'))
    return out


def xf(el):
    off = el.find('.//a:xfrm/a:off', ns)
    ext = el.find('.//a:xfrm/a:ext', ns)
    xfrm = el.find('.//a:xfrm', ns)
    rot = xfrm.get('rot') if xfrm is not None else None
    flip = ''
    if xfrm is not None:
        if xfrm.get('flipH') == '1':
            flip += 'H'
        if xfrm.get('flipV') == '1':
            flip += 'V'
    if off is None or ext is None:
        return None
    x = int(off.get('x')) / EMU
    y = int(off.get('y')) / EMU
    w = int(ext.get('cx')) / EMU
    h = int(ext.get('cy')) / EMU
    extra = ''
    if rot:
        extra += f' rot={int(rot)/60000:.0f}'
    if flip:
        extra += f' flip={flip}'
    return f'x={x:5.2f} y={y:5.2f} w={w:5.2f} h={h:5.2f}{extra}'


files = sorted(glob.glob(os.path.join(d, 'slide*.xml')), key=lambda f: int(re.findall(r'(\d+)', os.path.basename(f))[0]))
for f in files:
    n = int(re.findall(r'(\d+)', os.path.basename(f))[0])
    if only and n not in only:
        continue
    rl = rels(n)
    t = ET.parse(f).getroot()
    print(f'=== slide {n}')
    tree = t.find('.//p:cSld/p:spTree', ns)
    for el in tree.iter():
        tag = el.tag.split('}')[1]
        if tag not in ('sp', 'pic', 'cxnSp', 'grpSp'):
            continue
        nv = el.find('.//p:cNvPr', ns)
        name = nv.get('name') if nv is not None else ''
        pos = xf(el)
        spPr = el.find('p:spPr', ns)
        geom = None
        fill = line = None
        if spPr is not None:
            g = spPr.find('a:prstGeom', ns)
            geom = g.get('prst') if g is not None else ('custGeom' if spPr.find('a:custGeom', ns) is not None else None)
            fill = colour(spPr)
            ln = spPr.find('a:ln', ns)
            line = colour(ln) if ln is not None else None
            if ln is not None and ln.get('w'):
                line = f"{line} w={int(ln.get('w'))/12700:.1f}pt"
            if ln is not None:
                he = ln.find('a:headEnd', ns)
                te = ln.find('a:tailEnd', ns)
                if he is not None and he.get('type') not in (None, 'none'):
                    line += f" head={he.get('type')}"
                if te is not None and te.get('type') not in (None, 'none'):
                    line += f" tail={te.get('type')}"
        # style-based fill
        style = el.find('p:style', ns)
        sfill = None
        if style is not None:
            fr = style.find('a:fillRef', ns)
            if fr is not None:
                c = fr.find('a:schemeClr', ns)
                sfill = 'style:' + (c.get('val') if c is not None else '?')
        if tag == 'pic':
            blip = el.find('.//a:blip', ns)
            rid = blip.get('{%s}embed' % R) if blip is not None else None
            src = el.find('.//p:blipFill/a:srcRect', ns)
            crop = ''
            if src is not None and src.attrib:
                crop = ' crop=' + ','.join(f'{k}={int(v)/1000:.1f}%' for k, v in src.attrib.items())
            print(f'  PIC  {name!r:28} {pos} media={rl.get(rid)}{crop}')
            continue
        if tag == 'grpSp':
            print(f'  GRP  {name!r:28} {pos}')
            continue
        txts = []
        tcols = set()
        sizes = set()
        bolds = set()
        for para in el.iter('{%s}p' % A):
            ptxt = ''.join((r.text or '') for r in para.iter('{%s}t' % A))
            if ptxt.strip():
                txts.append(ptxt.strip())
        for rpr in el.iter('{%s}rPr' % A):
            c = colour(rpr)
            if c:
                tcols.add(c)
            if rpr.get('sz'):
                sizes.add(int(rpr.get('sz')) / 100)
            if rpr.get('b') == '1':
                bolds.add('B')
        txt = ' / '.join(txts)
        meta = f'geom={geom} fill={fill or sfill} line={line}'
        tinfo = f' tcol={sorted(tcols)} sz={sorted(sizes)}{"B" if bolds else ""}' if txt else ''
        print(f'  {tag.upper():4} {name!r:28} {pos} {meta}{tinfo} | {txt}')
