#! /usr/bin/env python3
import rospy
import math
import numpy as np
import matplotlib.pyplot as plt

import matplotlib.pyplot as plt
import matplotlib.animation


from sensor_msgs.msg import LaserScan  #import of the message from the package

rospy.init_node('reader') #initialization of the node


plt.ion()
fig, ax = plt.subplots()

ox, oy = [],[]
sc = ax.scatter(ox,oy)
plt.axis("equal")
plt.ylim([-5, 5]) 
plt.xlim([-5,5])
plt.grid(True)


def get_front(data):
    data   = np.array(data)
    angles = np.array(list(range(-90,90))) * np.pi/180
    dist   = np.concatenate(
                          (np.flip(data[:90]), data[270:]),
                          axis = 0
                          )
    return dist, angles

def plotter(ranges):
    global ox, oy

    dist, ang = get_front(ranges)
    ox, oy = dist * np.cos(ang), -dist*np.sin(ang)

    sc.set_offsets(np.c_[ox,oy])
    bottom, top = plt.ylim()  # return the current ylim
    
    fig.canvas.draw_idle()



def callback(msg):
    #print("The front LiDAR reading is: %f" %msg.ranges[359])
    plotter(msg.ranges)

sub = rospy.Subscriber('/scan' , LaserScan , callback)
rate = rospy.Rate(1)

plt.show(block=True)
# rescale y axis, to match the grid orientation


while not rospy.is_shutdown():
    rate.sleep()