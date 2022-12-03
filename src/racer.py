#! /usr/bin/env python3
from BurgerRobot import BurgerRobot
import rospy
import numpy as np
from scipy.ndimage.filters import convolve as conv2
from sensor_msgs.msg import LaserScan  
from geometry_msgs.msg import Twist
from rospy.numpy_msg import numpy_msg
from vmc_project.msg import Floats

import time
import signal

import os

from VFH import polar_histogram
from utils import gaussian_kernel, \
    moving_average, get_front, PID_controller, occupancy_map


myhistogram, myhistogram2, occmap = [], [], []
plt_msg = Floats()

PAUSE         = 0
TIME          = time.time()
ACTIVE_REGION = 9
vehicle = BurgerRobot()


def handler(signum, frame):
    """
    Event handler for debugging the code
    """
    global PAUSE, TIME
    if time.time() - TIME < 1:
        print("CLOSING")
        exit(1)
    
    if not PAUSE:
        vehicle.vel.linear.x = 0
        vehicle.vel.angular.z = 0
        PAUSE = 1
        TIME = time.time()

        pub.publish(vehicle.vel)
    else:
        PAUSE = 0


def cost_function(myhistogram,LIDAR):
    """
    Cost function that will choose the best direction
    """
    global myhistogram2
    myhistogram2 = moving_average(myhistogram,6)     

    goal_direction = np.argmin(myhistogram2)*ACTIVE_REGION
    goal_direction_rad = goal_direction*np.pi/180

    long_vel = np.amin([0.3,(LIDAR[90])/10])
    
    return goal_direction_rad, long_vel



def plotter(ranges):
    global myhistogram, occmap

    # Filter the data and take only the one within [0, 180]
    dist, ang = get_front(ranges)

    if np.sum(dist == float("inf")) == len(dist):
        vehicle.stop()
        pub.publish(vehicle.vel)
        exit(1)

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

    # Calculate Vector Field Histograms
    myhistogram = polar_histogram(dist, occmap, active_region=ACTIVE_REGION)    
    
    # Estimate direction and desired velocities
    vehicle.goal_dir,vehicle.vel.linear.x = cost_function(myhistogram,dist)
    
    # Apply PID control
    PID_controller(vehicle)
    
    if not PAUSE: 
        pub.publish(vehicle.vel)
        os.system("clear")
        print(vehicle.goal_dir*180/np.pi)
        print(np.argmin(myhistogram))
    vehicle.prev_dir = vehicle.goal_dir




def callback(msg):
    plotter(msg.ranges)


def main():
    global sub, pub, datapub
    
    #initialization of the node
    rospy.init_node('racer') 
    sub = rospy.Subscriber('/scan' , LaserScan , callback)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
    datapub = rospy.Publisher('/data', numpy_msg(Floats) , queue_size=1)

    rate = rospy.Rate(100)

    # Enable the debug tool by pressing CTRL+C
    signal.signal(signal.SIGINT, handler)


    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()
