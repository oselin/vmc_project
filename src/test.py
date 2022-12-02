#! /usr/bin/env python3
from dataclasses import dataclass
from http.client import IM_USED
from re import I
from BurgerRobot import BurgerRobot
from racer import ACTIVE_REGION
import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from rospy.numpy_msg import numpy_msg
from vmc_project.msg import Floats
from scipy.ndimage.filters import convolve as conv2
from utils import gaussian2, strong_avoid,exp_moving_average, gaussian_kernel, \
                         moving_average, get_front, PID_controller, occupancy_map
from VFH import polar_histogram
from sensor_msgs.msg import LaserScan

def data_processing(data):
    data = data.ranges
    global myhistogram, occmap
    dist, ang = get_front(data)

    # Rescale data to consider only the closest
    dist = 4*dist

    idx = dist == float("inf")
    for i,j in  enumerate(idx):
        if i == 0 or i == len(idx) - 1: dist[i] = 0.120
        else: 
            if j: dist[i] = np.amin([dist[i-1], dist[i+1]])


    # Get Cartesian coordinates
    ox = dist*np.cos(ang)
    oy = dist*np.sin(ang)
    
    # Create occupancy map and Gaussian filter
    occmap = occupancy_map([ox,oy])
    g =  gaussian_kernel(5, 0.5)

    # Apply uncertainty
    occmap= conv2(occmap,g,mode ="reflect")

    myhistogram = polar_histogram(dist, occmap, active_region=ACTIVE_REGION)    



#plt.axis("equal")
#plt.grid(True, which="minor", color="w", linewidth = .6, alpha = 0.5)
#tab2 = ax[1].bar(np.arange(180/ACTIVE_REGION)*ACTIVE_REGION, np.zeros([int(180/ACTIVE_REGION)]))



rospy.init_node('listener', anonymous=True)
rospy.Subscriber('/scan', LaserScan, data_processing)


def animate(frameno):
    #occmap = hp.reshape(70,70)
    ax[0].imshow(occmap,cmap = "PiYG_r")

    ax[1].cla()
    ax[1].set_xlim(0,200)
    ax[1].set_ylim(0,200)   
    ax[1].bar(np.arange(180/ACTIVE_REGION)*ACTIVE_REGION+2,myhistogram)


fig, ax = plt.subplots(3, clear=True, figsize=(4, 14))
plt.axis("equal")
plt.grid(True, which="minor", color="w", linewidth = .6, alpha = 0.5)


ani = FuncAnimation(fig, animate, interval=100)

plt.show()
