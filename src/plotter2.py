#! /usr/bin/env python3
from racer import ACTIVE_REGION
import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from scipy.ndimage.filters import convolve as conv2
from utils import gaussian_kernel, get_front, occupancy_map, moving_average
from VFH import polar_histogram
from sensor_msgs.msg import LaserScan

def data_processing(data):
    data = data.ranges
    global myhistogram, occmap, myhistogram2
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
    myhistogram2 = moving_average(myhistogram,6)


rospy.init_node('listener', anonymous=True)
rospy.Subscriber('/scan', LaserScan, data_processing)


def animate(frameno):
    try:
        ax[0].cla()
        ax[0].imshow(occmap,cmap = "PiYG_r")

        ax[1].cla()
        ax[1].set_xlim([0,200])
        ax[1].set_ylim([0,200])   
        ax[1].bar(np.arange(180/ACTIVE_REGION)*ACTIVE_REGION,myhistogram, color="gray")
        ax[1].plot(np.arange(180/ACTIVE_REGION)*ACTIVE_REGION,myhistogram2, color="orange")

    except:
        pass


fig, ax = plt.subplots(2, clear=True, figsize=(4, 8))
plt.axis("equal")
plt.grid(True, which="minor", color="w", linewidth = .6, alpha = 0.5)

ani = FuncAnimation(fig, animate, frames=10000)

plt.show()
