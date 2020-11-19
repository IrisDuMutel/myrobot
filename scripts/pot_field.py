#!/usr/bin/env python

"""
Potential Field based path planner
based on code from Atsushi Sakai
"""


from collections import deque
import numpy as np
import matplotlib.pyplot as plt


kp = 5 #attractive pottential gain
eta = 100.0 # repulsive potential gain
AREA_WIDTH = 30 # potential area width [m]
OSCILLATIONS_DETECTION_LENGTH = 3

show_animation = True
# to see image, comment return in pot_field() and remove from __main__ the outcome

def get_motion_model():
    # dx, dy
    # Defining 8 connection for the grid.
    motion = [[1, 0],
              [0, 1],
              [-1, 0],
              [0, -1],
              [-1, -1],
              [-1, 1],
              [1, -1],
              [1, 1]]

    return motion


def calc_potential_field(sx, sy, gx, gy, ox, oy, grid_size, robot_radius):
    # Limit the range in which the potential field will be computed
    # Reference frame tranformation so i can center put the points
    # at AREA_WIDTH from the limit of the map (minimum)

    minx = min(min(ox),sx,gx)   - AREA_WIDTH/2
    miny = min(min(oy), sy, gy) - AREA_WIDTH/2
    maxx = max(max(ox),sx,gx)   + AREA_WIDTH/2
    maxy = max(max(oy),sy,gy)   + AREA_WIDTH/2
    xw   = int(round((maxx-minx)/grid_size))
    yw   = int(round((maxy-miny)/grid_size))

    # initialize a matrix of vectors
    # [[0.0 0.0 ... 0.0·yw],...[0.0 0.0 ... 0.0]·xw]
    pmap = [[0.0 for i in range(yw)] for i in range(xw)]
    
    for ix in range(xw):
        # get x coordinate in originar RF
        x = ix*grid_size + minx

        for iy in range(yw):
            # get y coordinate in originar RF
            y = iy*grid_size + miny
            u_att = calc_att_pot(x,y,gx,gy)
            u_rep = calc_rep_pot(x,y,ox,oy,robot_radius)
            u_tot = u_att + u_rep
            pmap[ix][iy] = u_tot
    return pmap,minx,miny

def calc_att_pot(x,y,gx,gy):
    return kp*np.hypot(x-gx,y-gy)

def calc_rep_pot(x,y,ox,oy,robot_radius):
    # search nearest obstacle
    minod = -1 # obstacle index
    dmin = float("inf") # initialize distance as inf
    for i, _ in enumerate(ox):
        d = np.hypot(x - ox[i], y - oy[i])
        if dmin >= d:
            dmin = d
            minod = i
    
    DQ = np.hypot(x-ox[minod],y-oy[minod])

    if DQ<=robot_radius: # if inside or touching the robots limits
        if DQ<=0.1:
            DQ = 0.1
        return 0.5*eta*(1/DQ - 1/robot_radius)**2
    else:
        return 0.0

def draw_heatmap(data,ax1): 
    data = np.array(data).T
    ax1.pcolor(data, vmax=100.0, cmap=plt.cm.Blues)


def oscillations_detection(previous_ids,ix,iy):
    previous_ids.append((ix,iy)) # add this couple to the list on nodes

    if (len(previous_ids)>OSCILLATIONS_DETECTION_LENGTH):
        previous_ids.popleft() #remove the initial pair of coordinates
    
    # check if there are duplicates: 
    previous_ids_set = set()
    for index in previous_ids:
        if index in previous_ids_set:
            return True
        else:
            previous_ids_set.add(index)
    return False
        
def potential_field_planning(sx, sy, gx, gy, ox, oy, grid_size, robot_radius):
    

    # calculate potential field
    pmap, minx, miny = calc_potential_field(sx, sy, gx, gy, ox, oy, grid_size, robot_radius)

    # resolve path
    d = np.hypot(sx-gx, sy-gy) # initial distance
    ix = round((sx-minx)/grid_size)
    iy = round((sy-miny)/grid_size)
    gix = round((gx-minx)/grid_size)
    giy = round((gy-miny)/grid_size)
    f, (ax1, ax2, ax3) = plt.subplots(1, 3, sharey=False)

    if show_animation:
        # draw the potential field with the starting and ending points
        draw_heatmap(pmap,ax1)
        
        ax1.plot(ix, iy, "*k")
        ax1.plot(gix, giy, "*m")


    rx, ry = [sx], [sy] # initializing position vectors
    vx, th = [0],[0]    # Initializing velocity and angle vectors
    motion = get_motion_model() # 8 connection
    previous_ids = deque() # we initialize a list-like object optimized to be accessed through
    # its endpoints

    while d >= grid_size:
        minp = float("inf")
        minix, miniy = -1,-1
        for i, _ in enumerate(motion): # for the amount of connections in one node
            inx = int(ix+motion[i][0])
            iny = int(iy+motion[i][1])
            if inx >= len(pmap) or iny >= len(pmap[0]) or inx<0 or iny<0:
                p = float("inf")
                print("Outside the potential area")
            else:
                p = pmap[inx][iny] # what is the pot in this connection?
            if minp > p:
                minp = p
                minix = inx
                miniy = iny
        ix = minix
        iy = miniy
        xp = ix*grid_size + minx # back to original RF
        yp = iy*grid_size +miny
        d = np.hypot(gx-xp,gy-yp)
        rx.append(xp)
        ry.append(yp)

        if (oscillations_detection(previous_ids, ix, iy)):
            print("Oscillation detected at ({},{})!".format(ix, iy))
            break
        

        if show_animation:
            ax1.plot(ix, iy, ".r")
            plt.pause(0.01)
       
    print("Goal reached, path computed")

    return rx, ry, ax1, ax2, ax3
def pot_field():
    print("Starting APF algortihm")

    # Positions definition

    sx = 0.0 # start x position [m]
    sy = 0.0 # start y position [m]
    gx = 2.0 # goal x position [m]
    gy = 2.0 #goal y position [m]
    grid_size = 0.3 # potential grid size [m]
    robot_radius = 0.5 # robot dimensions [m]
    ox = [0.5] # obstacle x positions list [m]
    oy = [0.55] # obstacles y positions list [m]


    rx, ry, ax1, ax2, ax3 = potential_field_planning(sx, sy, gx, gy, ox, oy, grid_size, robot_radius)
    
    vx, th = trajectory_generation(rx,ry)
    
    print("vel prof:", vx)
    print("th prof:", th)
    print("rx:", rx)
    print("ry:", ry)
    if show_animation:
        ax2.plot(vx)
        ax2.grid()
        ax2.axes.set_ylabel("Vel x")
        ax2.axes.set_xlabel("points")
        ax3.plot(th)
        ax3.grid()
        ax3.axes.set_ylabel("Heading")
        ax3.axes.set_xlabel("points")
        plt.show()
    return vx,th,rx,ry


def trajectory_generation(rx,ry):
    t = 1 # time for segment i
    i = 0
    vx,th = [0],[0]
    while i < len(rx)-1:
        dist  = np.hypot(rx[i+1]-rx[i], ry[i+1]-ry[i])
        vx.append(dist/t)
        th.append(np.arctan2( ry[i+1]-ry[i],rx[i+1]-rx[i]))
        i += 1

    return vx,th
    
if __name__=='__main__':
    print(__file__ + "start!")
    pot_field()
    print(__file__ + "Done!")