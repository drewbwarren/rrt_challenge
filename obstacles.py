#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
from math import atan2

# how big is our world?
SIZE = 100

# Start
q_init = [10,10]

# Goal
q_final = [75,75]

dq = .5

# Limit to the loop
K = 1000

# G is the list of vertices and edges
G = []

world = []

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

def init():
    # generate circles:
    world = []
    generation_flag = True
    circle_count = 0
    while generation_flag:
        world = generate_circles(10, 8, 3)
        print "new world"
        for c in world:
            print "checking a circle"
            if dist(q_init, c[1:]) < c[0]:
                print "start inside circle"
                circle_count = 0
                break
            elif dist(q_final, c[1:]) < c[0]:
                print "goal inside circle"
                circle_count = 0
                break
            else:
                circle_count += 1
        if circle_count == len(world):
            print "all circles good"
            generation_flag = False

    return world


def init_G():
    G.append([q_init, None])

def rand_conf():
    return [np.random.rand() * 100, np.random.rand() * 100]

def nearest_vertex(q_rand): # returns the index of the vertex in G that is the closest to q_rand
    min_length = [100,0]
    for i,vertex in enumerate(G):
        parent = vertex[1]
        vertex= vertex[0]
        d = dist(q_rand, vertex)
        if d < min_length[0]:
            min_length = [d,i]
    return min_length[1]

def G_plot(world):
    verts = []
    codes = []
    for vertex,parent in G[1:]:
        verts.append(vertex)
        verts.append(G[parent][0])
        codes.append(Path.MOVETO)
        codes.append(Path.LINETO)
    verts_goal = []
    codes_goal = []

    fig = plt.figure()
    path = Path(verts, codes)
    patch = patches.PathPatch(path)
    ax = fig.add_subplot(111)
    ax.add_patch(patch)
    ax.set_xlim([0,100])
    ax.set_ylim([0,100])
    plt.hold(True)

    # Indicate the start and goal - from Miaoding
    ax.plot(q_init[0],q_init[1],'go')
    ax.text(q_init[0]-5,q_init[1]-5,'init',fontsize=25,style='italic',color='green')
    ax.plot(q_final[0],q_final[1],'ro')
    ax.text(q_final[0]+1,q_final[1]+1,'goal',fontsize=25,style='italic',color='red')

    # Draw a path from start to goal
    final_path = []
    final_codes = []
    final_path.append(G[-1][0]) # place q_final in this list
    final_path.append((G[G[-1][1]][0])) # place parent of q_final in list
    final_codes.append(Path.MOVETO)
    final_codes.append(Path.LINETO)
    current_index = G[G[-1][1]][1] # parent of q_final
    parent_index = G[current_index][1]
    while parent_index != 0:
        parent_index = G[current_index][1]
        final_path.append(G[current_index][0])
        final_path.append(G[parent_index][0])
        final_codes.append(Path.MOVETO)
        final_codes.append(Path.LINETO)
        current_index = parent_index
    path_2 = Path(final_path, final_codes)
    patch_2 = patches.PathPatch(path_2, color='red', linewidth=2)
    ax.add_patch(patch_2)

    # Plot the circles
    fcirc = lambda x: patches.Circle((x[1], x[2]), radius=x[0], fill=True, alpha=1, fc='k', ec='k')
    circs = [fcirc(x) for x in world]
    for c in circs:
        ax.add_patch(c)
    plt.xlim([0, SIZE])
    plt.ylim([0, SIZE])
    plt.xticks(range(0, SIZE + 1))
    plt.yticks(range(0, SIZE + 1))
    ax.set_aspect('equal')
    ax.set_xticklabels([])
    ax.set_yticklabels([])

    plt.show()


def check_collision(new_pt, near_pt, world):
    '''
    Use this function to check for a collision between two points
    '''
    flag = False
    for c in world:

        # check first if the new point is inside the circle
        if dist(new_pt, c[1:]) < c[0]:
            flag = True
        # check if the line to the nearest vertex collides
        elif line_thru_circ(new_pt, near_pt, c):
            # TODO
            # check if the nearest vertex is on the other side of the obstacle
            if circ_between_pts(new_pt, near_pt, c):
                flag = True
        else:
            continue
    return flag


def line_thru_circ(new_pt, near_pt, circ):
    slope = (new_pt[1]-near_pt[1])/(new_pt[0]-near_pt[0])
    A = -slope
    B = 1
    C = -new_pt[1] + (slope*new_pt[0])
    x = circ[1]
    y = circ[2]
    d = np.abs(A*x + B*y + C)/np.sqrt(A**2 + B**2)
    if d < circ[0]:
        return True
    else:
        return False

# Given that the line collides with the circle, find out if the circle is between the points or not - from Miaoding
def circ_between_pts(new_pt, near_pt, circ):
    C1 = (new_pt[1] - near_pt[1])*(near_pt[1] - circ[2]) + (new_pt[0] - near_pt[0])*(near_pt[0] - circ[1])
    C2 = (new_pt[1] - near_pt[1])*(new_pt[1] - circ[2]) + (new_pt[0] - near_pt[0])*(new_pt[0] - circ[1])
    if C1*C2 < 0:
        return True
    else:
        return False


def dist(p1, p2):
    return np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def new_conf(q_rand, q_near):
    angle = atan2(q_rand[1] - q_near[1], q_rand[0] - q_near[0])
    x_new = q_near[0] + dq*np.cos(angle)
    y_new = q_near[1] + dq*np.sin(angle)
    return [x_new, y_new]


def main():
    world = init()
    init_G()
    goal = False
    # while len(G) < K:
    while not goal:
        # print "new q"
        # Obtain a random configuration
        q_rand = rand_conf() # Line 1

        # Find the closest vertex
        vertex_ind = nearest_vertex(q_rand) # Line 2

        # Check for straight line collisionsa
        collision = check_collision(q_rand, G[vertex_ind][0], world)

        # Determine new configuration - just use q_rand
        q_new = new_conf(q_rand, G[vertex_ind][0]) # Line 3

        # Add q_new to vertex_list and edge_list
        # print i+1, vertex_ind, q_new
        if not collision:
            G.append([q_new, vertex_ind])

        # Check if the new configuration is close to the goal
        if dist(q_new, q_final) < 2:
            # print vertex_ind, len(G)
            G.append([q_final, len(G)-1])
            goal = True

    G_plot(world)


if __name__ == '__main__':
    main()
