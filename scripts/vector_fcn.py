#!usr/bin/env python
#Code Property of Matteo Scanavino - matteo.scanavino@polito.it
import matplotlib.pyplot as plt
import numpy as np
import random
import os
# os.environ['MAVLINK20']='1' #set mavlink2 for odometry message
# from pymavlink import mavutil
import time


def sum_vect(vectA,vectB):
    # This function implements the sum of two vector in polar coordinates
    # Formulas taken from: https://math.stackexchange.com/questions/1365622/adding-two-polar-vectors
    phiA = vectA[0]*np.pi/180   # angle vector A
    phiB = vectB[0]*np.pi/180   # agnle vector B
    rA = vectA[1]               # magnitude vector A
    rB = vectB[1]               # magnitude vector B
    # Compute magnitude
    r = np.sqrt(rA*rA+rB*rB+2*rA*rB*np.cos(phiB-phiA)) 
    # Compute angle
    phi = phiA + np.arctan2(rB*np.sin(phiB-phiA),rA+rB*np.cos(phiB-phiA))*180/np.pi
    # sum of the input vector
    vect = [phi,r]
    return vect

def plt_vect(vect):
    print(vect)
    # This function plots n vectors in vect 
    nvect = len(vect)
    plt.figure(1)
    ax = plt.subplot(111, projection='polar')
    # RMAX = max([row[1] for row in vect])*1.2
    RMAX = 4
    ax.set_rmax(RMAX)
    ax.grid(True)
    ax.set_title("Polar vector plot", va='bottom')

    # Define colors
    r = np.linspace(0,1,nvect)
    g = np.roll(r,1) 
    b = np.roll(g,1)
    
    for i in range(0,nvect):
        color = (r[i], g[i], b[i])
        plt.arrow(vect[i][0]*np.pi/180, 0, 0*np.pi/180, vect[i][1], alpha = 0.5, width = 0.015,edgecolor = color, facecolor = color, lw = 2, zorder = 5)
    plt.show(block=False)


def send_vect(vect,connection):
    # vect is a vector in polar coordinates
    #       vect[0]: vector angle
    #       vect[1]: vector module
    # connection is a mavlink connection object
    
    angle = vect[0]     # vector angle 
    module = vect[1]    # vector module
    
    # TODO implement a custom mavlink message to send a vector in polar coordinates instead of using Mavlink Odometry Msg    
    
    # # Preparign Mavlink message Odometry
    # time_usec = 0
    # frame_id = 0
    # child_frame_id = 0
    # x = angle
    # y = module
    # z = 0
    # q = [0,0,0,0]
    # vx = 0
    # vy = 0
    # vz = 0
    # rollspeed = 0
    # pitchspeed = 0
    # yawspeed = 0
    # pose_covariance=[]
    # velocity_covariance=[]    
    # for i in range (21): 
    #     pose_covariance.append(0)
    #     velocity_covariance.append(0)
    # reset_counter=0
    # estimator_type=0

    # Send mavlink message
    # connection.mav.odometry_send(time_usec,frame_id,child_frame_id,x,y,z,q,vx,vy,vz,rollspeed,pitchspeed,yawspeed,pose_covariance,velocity_covariance,reset_counter,estimator_type)
