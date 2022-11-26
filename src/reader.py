#! /usr/bin/env python3
import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, radians, pi
import lidar_to_grid_map as lg
from numpy import inf


from sensor_msgs.msg import LaserScan  #import of the message from the package

rospy.init_node('reader') #initialization of the node

def plotter(ranges):
    dist = np.array(ranges)
    dist[dist == inf] = 0.0
    print(dist)
    ang = np.array(list(range(0, 360)))
    ox = dist * np.cos(np.pi/180*ang)
    oy = -dist*np.sin(np.pi/180*ang)
    
    xyreso = 0.02  # x-y grid resolution
    yawreso = math.radians(3.1)  # yaw angle resolution [rad]
    pmap, minx, maxx, miny, maxy, xyreso = lg.generate_ray_casting_grid_map(ox, oy, xyreso, True)
    xyres = np.array(pmap).shape
    plt.figure(figsize=(20,8))
    plt.subplot(122)
    plt.imshow(pmap, cmap = "PiYG_r") 
    plt.clim(-0.4, 1.4)
    plt.gca().set_xticks(np.arange(-.5, xyres[1], 1), minor = True)
    plt.gca().set_yticks(np.arange(-.5, xyres[0], 1), minor = True)
    plt.grid(True, which="minor", color="w", linewidth = .6, alpha = 0.5)
    plt.colorbar()
    plt.show()



def callback(msg):
    #print(msg.ranges)
    plotter(msg.ranges)


sub = rospy.Subscriber('/scan' , LaserScan , callback)

rate = rospy.Rate(1)
while not rospy.is_shutdown():
    rate.sleep()