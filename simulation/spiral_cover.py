# -*- coding: utf-8 -*-
"""Coverage of the square-spiral search (advisor review 2026-11-04: table 3-12 gave 99.2 % without a method).
Camera axis moves over the travel area; a marker counts as seen when it can lie fully inside the frame while the axis is on
the path.  Values from maincamera.py: CENTER 125/133, SPIRAL 20/30, range(MAX_LOOPS-1) = 4 rounds, TRAVEL 262.0 x 255.5,
scale 8.40 px/mm, frame 640 x 360 px; marker 9 mm (book).  Result is written to sim/spiral_cover.json for the book."""
import json, os
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
TX, TY = 262.0, 255.5; CX, CY = 125.0, 133.0; START, INC, ROUNDS = 20.0, 30.0, 4
S = 8.40; A = 9.0; FW, FH = 640 / S - A, 360 / S - A
def path():
    x, y, s = CX, CY, START; P = [(x, y)]
    for _ in range(ROUNDS):
        y += s; P.append((x, y)); x += s; P.append((x, y)); s += INC
        y -= s; P.append((x, y)); x -= s; P.append((x, y)); s += INC
    return P
P = path()
L = sum(abs(a[0] - b[0]) + abs(a[1] - b[1]) for a, b in zip(P, P[1:]))
step = 0.1
gx = np.arange(step / 2, TX, step); gy = np.arange(step / 2, TY, step); m = np.zeros((len(gx), len(gy)), bool)
for a, b in zip(P, P[1:]):
    x0, x1 = min(a[0], b[0]) - FW / 2, max(a[0], b[0]) + FW / 2; y0, y1 = min(a[1], b[1]) - FH / 2, max(a[1], b[1]) + FH / 2
    ix = np.where((gx >= x0) & (gx <= x1))[0]; iy = np.where((gy >= y0) & (gy <= y1))[0]
    m[np.ix_(ix, iy)] = True
# rectangles missed, for the worked example
miss_x = gx[np.where(~m.all(axis=1))[0]]; miss_y = gy[np.where(~m.all(axis=0))[0]]
res = dict(travel=[TX, TY], start=[CX, CY], points=P, length=L, frame=[FW, FH], cover=float(m.mean() * 100), area=TX * TY,
           missed=float((~m).sum() * step * step), xmin=min(p[0] for p in P), xmax=max(p[0] for p in P), ymin=min(p[1] for p in P), ymax=max(p[1] for p in P))
os.makedirs(os.path.join(HERE, "sim"), exist_ok=True)
json.dump(res, open(os.path.join(HERE, "sim", "spiral_cover.json"), "w"), indent=1)
print({k: v for k, v in res.items() if k != "points"}); print(P)
# where the holes are
from scipy import ndimage
lab, n = ndimage.label(~m); print("holes", n)
for i, sl in enumerate(ndimage.find_objects(lab)):
    print(i + 1, "x %.1f-%.1f y %.1f-%.1f" % (gx[sl[0].start], gx[sl[0].stop - 1], gy[sl[1].start], gy[sl[1].stop - 1]), "area %.1f" % ((lab[sl] == i + 1).sum() * step * step))
