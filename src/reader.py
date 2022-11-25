#! /usr/bin/env python3
import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, radians, pi


from sensor_msgs.msg import LaserScan  #import of the message from the package

rospy.init_node('reader') #initialization of the node

def plotter(ranges):
    dist = np.array(ranges)
    ang = np.array(list(range(0, 360)))
    ox = []
    oy = []
    #for i in range(360):
    #    ox.append(np.sin(ang[i]) * dist[i])
    #    oy.append(np.cos(ang[i]) * dist[i])
    ox = np.sin(ang) * dist
    oy = np.cos(ang) * dist
    #ox = np.array(ox)
    #oy = np.array(oy)
    print(ox.size)
    plt.figure(figsize=(6,10))
    plt.plot([oy, np.zeros(np.size(oy))], [ox, np.zeros(np.size(oy))], "ro-") # lines from 0,0 to the point
    plt.axis("equal")
    bottom, top = plt.ylim()  # return the current ylim
    plt.ylim((top, bottom)) # rescale y axis, to match the grid orientation
    plt.grid(True)
    plt.show()




def callback(msg):
    print("The front LiDAR reading is: %f" %msg.ranges[359])
    plotter(msg.ranges)

sub = rospy.Subscriber('/scan' , LaserScan , callback)

rate = rospy.Rate(1)
while not rospy.is_shutdown():
    rate.sleep()