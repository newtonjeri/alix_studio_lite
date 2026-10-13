"""Map every coloured overlay rectangle of the regions deck onto the media image it sits on.
Output: slide, overlay text, arm colour, media file, normalised box (u0,v0,u1,v1) in the SOURCE image
(0..1, crop + flipH applied; picture rotation <=4 deg ignored), overlay rotation.
Usage: python3 -I overlay_norm.py <slides_dir>"""
import sys, os, re, glob
import xml.etree.ElementTree as ET

A = 'http://schemas.openxmlformats.org/drawingml/2006/main'
P = 'http://schemas.openxmlformats.org/presentationml/2006/main'
R = 'http://schemas.openxmlformats.org/officeDocument/2006/relationships'
ns = {'a': A, 'p': P, 'r': R}
EMU = 914400.0
d = sys.argv[1]


def rels(n):
    t = ET.parse(os.path.join(d, '_rels', f'slide{n}.xml.rels')).getroot()
    return {r.get('Id'): os.path.basename(r.get('Target')) for r in t}


def box(el):
    x = el.find('.//a:xfrm', ns)
    off, ext = x.find('a:off', ns), x.find('a:ext', ns)
    return (int(off.get('x')) / EMU, int(off.get('y')) / EMU, int(ext.get('cx')) / EMU, int(ext.get('cy')) / EMU,
            int(x.get('rot', '0')) / 60000.0, x.get('flipH') == '1')


for f in sorted(glob.glob(os.path.join(d, 'slide*.xml')), key=lambda s: int(re.findall(r'(\d+)', os.path.basename(s))[0])):
    n = int(re.findall(r'(\d+)', os.path.basename(f))[0])
    if n < 5:
        continue
    rl = rels(n)
    t = ET.parse(f).getroot()
    pics = []
    for pic in t.iter('{%s}pic' % P):
        bx = box(pic)
        blip = pic.find('.//a:blip', ns)
        src = pic.find('.//p:blipFill/a:srcRect', ns)
        crop = {k: int(v) / 100000.0 for k, v in (src.attrib.items() if src is not None else [])}
        pics.append((bx, rl.get(blip.get('{%s}embed' % R)), crop))
    for sp in t.iter('{%s}sp' % P):
        spPr = sp.find('p:spPr', ns)
        g = spPr.find('a:prstGeom', ns) if spPr is not None else None
        if g is None or g.get('prst') != 'rect':
            continue
        name = sp.find('.//p:cNvPr', ns).get('name')
        if name.startswith('TextBox'):
            continue
        txt = ' '.join((x.text or '') for x in sp.iter('{%s}t' % A)).strip()
        fill = spPr.find('a:solidFill/a:srgbClr', ns)
        lnc = spPr.find('a:ln/a:solidFill/*', ns)
        col = (fill.get('val') if fill is not None else 'nofill') + '/' + (lnc.get('val') if lnc is not None else '-')
        arm = 'L(blue)' if ('4472C4' in col or '7A73FF' in col) else ('R(orange)' if ('ED7D31' in col or 'accent2' in col) else '?')
        x, y, w, h, rot, _ = box(sp)
        cx, cy = x + w / 2, y + h / 2
        best = None
        for (px, py, pw, ph, prot, pflip), media, crop in pics:
            inside = px <= cx <= px + pw and py <= cy <= py + ph
            dist = 0 if inside else min(abs(cx - (px + pw / 2)), 99)
            if best is None or dist < best[0]:
                best = (dist, (px, py, pw, ph, prot, pflip), media, crop)
        dist, (px, py, pw, ph, prot, pflip), media, crop = best
        l, tp, r, b = crop.get('l', 0), crop.get('t', 0), crop.get('r', 0), crop.get('b', 0)

        def u_of(xx):
            fr = (xx - px) / pw
            if pflip:
                fr = 1 - fr
            return l + fr * (1 - l - r)

        def v_of(yy):
            return tp + (yy - py) / ph * (1 - tp - b)

        u0, u1 = sorted((u_of(x), u_of(x + w)))
        v0, v1 = v_of(y), v_of(y + h)
        flag = '' if dist == 0 else ' (label box outside picture)'
        print(f's{n:02d} {arm:9} {txt[:42]:42} on {media:11} u[{u0:.3f},{u1:.3f}] v[{v0:.3f},{v1:.3f}] rot={rot:.0f}{" picflipH" if pflip else ""}{flag}')
