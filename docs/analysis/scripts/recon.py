"""Try to reproduce zip/02 (30pct) from zip/01 (idle_trimmed) using the HTML's
mean smoothing (centered, edge-padded, mean recentred, endpoints restored) and
different resampling conventions. Read-only; prints error statistics."""
import json, sys

src_path, dst_path = sys.argv[1], sys.argv[2]
pct = float(sys.argv[3]) / 100.0
windows = [int(w) for w in sys.argv[4].split(',')] if len(sys.argv) > 4 else [101]

def load(p):
    out = []
    with open(p) as fh:
        for line in fh:
            s = line.strip()
            if s:
                out.append(json.loads(s)['point'])
    return out

src = load(src_path)
dst = load(dst_path)

def crm(vals, window):
    n = len(vals)
    half = window // 2
    prefix = [0.0] * (n + 1)
    for i in range(n):
        prefix[i + 1] = prefix[i] + vals[i]
    first, last = vals[0], vals[-1]
    out = [0.0] * n
    for i in range(n):
        rs, re_ = i - half, i + half
        s, e = max(0, rs), min(n - 1, re_)
        sm = prefix[e + 1] - prefix[s]
        if rs < 0:
            sm += (-rs) * first
        if re_ >= n:
            sm += (re_ - (n - 1)) * last
        out[i] = sm / window
    return out

def smooth(data, window, rnd=True):
    n = len(data)
    out = [[0] * 7 for _ in range(n)]
    for j in range(7):
        y = [r[j] for r in data]
        m0 = sum(y) / n
        ys = crm(y, window)
        corr = m0 - sum(ys) / n
        for i in range(n):
            out[i][j] = round(ys[i] + corr) if rnd else ys[i] + corr
        out[0][j] = data[0][j]
        out[-1][j] = data[-1][j]
    return out

def resample(data, new_len):
    old = len(data)
    out = []
    for k in range(new_len):
        t = k / (new_len - 1)
        pos = t * (old - 1)
        i0 = int(pos)
        i1 = min(old - 1, i0 + 1)
        a = pos - i0
        out.append([round(data[i0][j] + a * (data[i1][j] - data[i0][j])) for j in range(7)])
    out[0] = list(data[0])
    out[-1] = list(data[-1])
    return out

print('src', len(src), 'dst', len(dst), 'ratio', round(len(dst) / len(src), 5))
for w in windows:
    sm = smooth(src, w, rnd=False)
    for label, new_len in (('round(N*p)', round(len(src) * pct)), ('round((N-1)*p)+1', round((len(src) - 1) * pct) + 1)):
        if new_len != len(dst):
            print('w', w, label, new_len, '!= dst len; skip')
            continue
        rs = resample(sm, new_len)
        err = max(abs(a - b) for r1, r2 in zip(rs, dst) for a, b in zip(r1, r2))
        mean_err = sum(abs(a - b) for r1, r2 in zip(rs, dst) for a, b in zip(r1, r2)) / (7 * len(dst))
        print('w', w, label, 'max_abs_err_mdeg', err, 'mean_abs_err_mdeg', round(mean_err, 2))
