#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
from math import atan2

# Workspace
D = [
    [0,100],
    [0,100]
    ]

# Initial configuration
q_init = [50,50]

dq = 1

# Limit to the loop
K = 1000

# G is the list of vertices and edges
G = []

def init_G():
    G.append([q_init, None])

def rand_conf():
    return [np.random.rand() * 100, np.random.rand() * 100]

def nearest_vertex(q_rand): # returns the index of the vertex in G that is the closest to q_rand
    min_length = [100,0]
    for i,vertex in enumerate(G):
        parent = vertex[1]
        vertex= vertex[0]
        d = np.sqrt((q_rand[0]-vertex[0])**2 + (q_rand[1]-vertex[1])**2)
        if d < min_length[0]:
            min_length = [d,i]
    return min_length[1]

def new_conf(q_rand, q_near):
    angle = atan2(q_rand[1] - q_near[1], q_rand[0] - q_near[0])
    x_new = q_near[0] + dq*np.cos(angle)
    y_new = q_near[1] + dq*np.sin(angle)
    return [x_new, y_new]

def G_plot():
    verts = []
    codes = []
    for vertex,parent in G[1:]:
        verts.append(vertex)
        verts.append(G[parent][0])
        codes.append(Path.MOVETO)
        codes.append(Path.LINETO)
    fig = plt.figure()
    path = Path(verts, codes)
    patch = patches.PathPatch(path)
    ax = fig.add_subplot(111)
    ax.add_patch(patch)
    ax.set_xlim([0,100])
    ax.set_ylim([0,100])
    plt.show()

def main():
    init_G()
    for i in range(K):
        # print "new q"
        # Obtain a random configuration
        q_rand = rand_conf() # Line 1

        # Find the closest vertex
        vertex_ind = nearest_vertex(q_rand) # Line 2

        # Determine new configuration - just use q_rand
        q_new = new_conf(q_rand, G[vertex_ind][0]) # Line 3

        # Add q_new to vertex_list and edge_list
        # print i+1, vertex_ind, q_new
        G.append([q_new, vertex_ind])

    # Plot the vertices
    G_plot()



if __name__ == '__main__':
    main()
