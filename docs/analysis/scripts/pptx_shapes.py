import sys, re, glob, os
import xml.etree.ElementTree as ET
A='http://schemas.openxmlformats.org/drawingml/2006/main'
P='http://schemas.openxmlformats.org/presentationml/2006/main'
d=sys.argv[1]
files = sorted(glob.glob(os.path.join(d, 'slide*.xml')), key=lambda f:int(re.findall(r'(\d+)', os.path.basename(f))[0]))
for f in files:
    n = int(re.findall(r'(\d+)', os.path.basename(f))[0])
    t = ET.parse(f).getroot()
    kinds = {}
    for sp in t.iter('{%s}sp' % P):
        txt = ''.join((x.text or '') for x in sp.iter('{%s}t' % A)).strip()
        geom = sp.find('.//{%s}prstGeom' % A)
        cust = sp.find('.//{%s}custGeom' % A)
        g = geom.get('prst') if geom is not None else ('custom' if cust is not None else 'none')
        fill = sp.find('.//{%s}spPr/{%s}solidFill' % (P, A))
        key = ('text' if txt else 'shape') + ':' + g
        kinds[key] = kinds.get(key, 0) + 1
    grp = len(list(t.iter('{%s}grpSp' % P)))
    cx = len(list(t.iter('{%s}cxnSp' % P)))
    print(f'slide{n}: ' + ', '.join(f'{k}x{v}' for k,v in sorted(kinds.items())) + f' | groups {grp} | connectors {cx}')
