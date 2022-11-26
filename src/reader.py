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


plt.ion()
fig, ax = plt.subplots()

plt.axis("equal")
plt.grid(True, which="minor", color="w", linewidth = .6, alpha = 0.5)



def get_front(data):
    data   = np.array(data)
    angles = np.array(list(range(-90,90))) * np.pi/180
    dist   = np.concatenate(
                          (np.flip(data[:90]), np.flip(data[270:])),
                          axis = 0
                          )
    return dist, angles

def plotter(ranges):
    dist, ang = get_front(ranges)
    dist[dist == inf] = 0.0

    ox = dist * np.cos(ang)
    oy = dist*np.sin(ang)
    
    xyreso = 0.05 # x-y grid resolution
    yawreso = math.radians(3.1)  # yaw angle resolution [rad]
    pmap, minx, maxx, miny, maxy, xyreso = lg.generate_ray_casting_grid_map(ox, oy, xyreso, True)
    xyres = np.array(pmap).shape
    plt.imshow(pmap, cmap = "PiYG_r") 
    plt.clim(-0.4, 1.4)
    plt.gca().set_xticks(np.arange(-.5, xyres[1], 1), minor = True)
    plt.gca().set_yticks(np.arange(-.5, xyres[0], 1), minor = True)
    #plt.colorbar()
    fig.canvas.draw_idle()
    #plt.show()


def callback(msg):
    #print(msg.ranges)
    plotter(msg.ranges)


sub = rospy.Subscriber('/scan' , LaserScan , callback)

rate = rospy.Rate(1)

plt.show(block=True)
while not rospy.is_shutdown():
    rate.sleep()