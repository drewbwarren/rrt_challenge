#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as mp
from matplotlib.path import Path
import matplotlib.patches as patches
# let's plot a triangle:
tree = [
    [-1,0],
    [1, 0],
    [0,1.5]
    ]
verts = []
codes = []
tree.append(tree[0])
for i,t in enumerate(tree[:-1]):
    verts.append(t)
    verts.append(tree[i+1])
    codes.append(Path.MOVETO)
    codes.append(Path.LINETO)
    print i
    print t
    print verts
    print codes
fig = mp.figure()
path = Path(verts, codes)
patch = patches.PathPatch(path)
ax = fig.add_subplot(111)
ax.add_patch(patch)
ax.set_xlim([-1.5,1.5])
ax.set_ylim([-1.5,1.5])
mp.show()
