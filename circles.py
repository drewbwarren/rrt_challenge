#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches

# how big is our world?
SIZE = 100

def generate_circles(num, mean, std):
    """
    This function generates /num/ random circles with a radius mean defined by
    /mean/ and a standard deviation of /std/.

    The circles are stored in a num x 3 sized array. The first column is the
    circle radii and the second two columns are the circle x and y locations.
    """
    circles = np.zeros((num,3))
    # generate circle locations using a uniform distribution:
    circles[:,1:] = np.random.uniform(mean, SIZE-mean, size=(num,2))
    # generate radii using a normal distribution:
    circles[:,0] = np.random.normal(mean, std, size=(num,))
    return circles

# generate circles:
world = generate_circles(10, 8, 3)
print world

# now let's plot the circles:
ax = plt.subplot(111)
fcirc = lambda x: patches.Circle((x[1],x[2]), radius=x[0], fill=True, alpha=1, fc='k', ec='k')
circs = [fcirc(x) for x in world]
for c in circs:
    ax.add_patch(c)
plt.xlim([0,SIZE])
plt.ylim([0,SIZE])
plt.xticks(range(0,SIZE + 1))
plt.yticks(range(0,SIZE + 1))
ax.set_aspect('equal')
ax.set_xticklabels([])
ax.set_yticklabels([])
plt.show()
