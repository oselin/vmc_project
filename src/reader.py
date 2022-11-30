#! /usr/bin/env python3
from re import M
import rospy
import numpy as np
import matplotlib.pyplot as plt
from numpy import inf
import os, sys
from scipy.ndimage.filters import convolve as conv2
from sensor_msgs.msg import LaserScan  
from geometry_msgs.msg import Twist
import time
import signal

sys.path.append(".")
from src.VFH import polar_histogram


#initialization of the node
rospy.init_node('reader') 




myhistogram = []
myhistogram2 = []
occmpa_uncert = []

def moving_average(x, w):
    bfr = []

    i = 0
    while i < len(x) - w :
    
        window = x[i : i + w]
        window_average = sum(window) / w
        bfr.append(window_average)

        i += 1 
    bfr = np.concatenate(([700,700,700], bfr, [700,700,700]), axis=0)

    return bfr




class myTurtle():
    def __init__(self):
        self.prev_dir = np.pi/2
        self.goal_dir = np.pi/2

        # Sample time for data collection
        self.sample_time = 0.1

        # PID variables
        self.previous_error = 0
        self.error = 0
        self.delta_error = 0
        self.integr_error = 0

        # PID coefficients
        self.Kp = 1.5 #0.67
        self.Ki = 0.05 #0.0045
        self.Kd = 0.5# 0.097

        # Initialize the Twist object for publishing velocities
        self.vel = Twist()

        # Define default initial values for linear and angular velocities
        self.vel.linear.x = 0
        self.vel.angular.z = 0

        # Distances for PID
        self.dr, self.dl = 0, 0

        # Status of the simulation
        self.is_running = 0
    
    def stop(self):
        self.vel.angular.z = 0
        self.vel.linear.x = 0
        self.is_running = 0


def dnorm(x, mu, sd):
    return 1 / (np.sqrt(2 * np.pi) * sd) * np.e ** (-np.power((x - mu) / sd, 2) / 2)


def gaussian2(sigma, N=None):

    if N is None:
        N = 2*np.maximum(4, np.ceil(6*sigma))+1
    k = (N - 1) / 2.
    xv, yv = np.meshgrid(np.arange(-k, k+1), np.arange(-k, k+1))

    # 2D gaussian filter    
    g = 1/(2 * np.pi * sigma**2) * np.exp(-(xv**2 + yv**2) / (2 * sigma**2))

    # 1st order derivatives
    gx = -xv / (2 * np.pi * sigma**4) * np.exp(-(xv**2 + yv**2) / (2 * sigma**2))
    gy = -yv / (2 * np.pi * sigma**4) * np.exp(-(xv**2 + yv**2) / (2 * sigma**2)) 

    # 2nd order derivatives
    gxx = (-1 + xv**2 / sigma**2) * np.exp(-(xv**2 + yv**2) / (2*sigma**2)) / (2 * np.pi * sigma**4)
    gyy = (-1 + yv**2 / sigma**2) * np.exp(-(xv**2 + yv**2) / (2*sigma**2)) / (2 * np.pi * sigma**4)
    gxy = (xv * yv) / (2 * np.pi * sigma**6) * np.exp(-(xv**2 + yv**2) / (2*sigma**2))    

    return g, gx, gy, gxx, gyy, gxy


def gaussian_kernel(size, sigma=1, verbose=False):
    kernel_1D = np.linspace(-(size // 2), size // 2, size)
    for i in range(size):
        kernel_1D[i] = dnorm(kernel_1D[i], 0, sigma)
    kernel_2D = np.outer(kernel_1D.T, kernel_1D.T)
    kernel_2D *= 1.0 / kernel_2D.max()
 
    if verbose:
        plt.imshow(kernel_2D, interpolation='none',cmap='gray')
        plt.title("Image")
        plt.show()
    return kernel_2D


def get_front(data):
    data   = np.array(data)
    angles = np.array(list(range(0,180))) * np.pi/180
    dist   = np.concatenate((data[270:], data[:90]),
                          axis = 0
                          )
    return dist, angles


def occupancy_map(data, resolution=0.1):

    grid = np.zeros([70,70])

    X = data[0]
    Y = data[1]
    for i in range(len(X)):
        indx = 35 + int(np.floor(X[i]/resolution))
        indy = 70 - int(np.floor(Y[i]/resolution))
        if indx < grid.shape[0] and indy < grid.shape[1] and indx > 0 and indy > 0: grid[indy,indx] = 1

    return grid

def cost_function(myhistogram, prev_dir, LIDAR, a, b, c):
    
    bfr = 1
    
    global myhistogram2
    #while bfr:
    myhistogram2 = moving_average(myhistogram,6)     

    heading = np.abs(myhistogram2)
    change = np.abs(myhistogram2*10*np.pi/180 - prev_dir)
    cost = b*heading + c*change

    goal_direction = np.argmin(myhistogram2)*10
    goal_direction_rad = goal_direction*np.pi/180

    
    return goal_direction_rad

def PID_controller(robot):

    robot.error = robot.goal_dir - np.pi/2
    robot.delta_error = robot.error - robot.previous_error
    robot.integr_error += robot.error
    
    P = robot.Kp*robot.error
    I = robot.Ki*robot.integr_error*robot.sample_time
    D = robot.Kd*robot.delta_error/robot.sample_time
    
    robot.previous_error = robot.error

    gas = P+I+D
    #print("gas: ", gas)

    robot.vel.angular.z = gas


def plotter(ranges):
    global myhistogram, occmpa_uncert
    dist, ang = get_front(ranges)
    dist = 4*dist
    dist[dist == inf] = 100

    ox = dist*np.cos(ang)
    oy = dist*np.sin(ang)
   
    occmap = occupancy_map([ox,oy])

    #g, _,_,_,_,_ = gaussian2(0.5)
    g = gaussian_kernel(5,1)
    occmpa_uncert = conv2(occmap,g,mode ="reflect")

    myhistogram = polar_histogram(dist, occmpa_uncert, active_region=10)    

    thr_up = 20
    thr_down = 0.1
    #myhistgram[myhistgram > thr_up] = 40
    #myhistgram[myhistgram < thr_down] = 40
    
    turtle.goal_dir = cost_function(myhistogram,turtle.prev_dir, dist, 1,1,2)

    PID_controller(turtle)
    if not PAUSE: 
        pub.publish(turtle.vel)
        os.system("clear")
        print(turtle.goal_dir*180/np.pi)
    turtle.prev_dir = turtle.goal_dir

    #os.system("clear")
    #print(myhistgram)
    """ax.clear()
    ax.bar(np.arange(36),myhistgram)
    ax.set_xlim(0,36)
    ax.set_ylim(0,100)
    ax.autoscale(False)"""
    #plt.imshow(occmpa_uncert,cmap = "PiYG_r")
    
    
    #fig.canvas.flush_events()



def callback(msg):
    #print(msg.ranges)
    plotter(msg.ranges)


sub = rospy.Subscriber('/scan' , LaserScan , callback)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

rate = rospy.Rate(10)

turtle = myTurtle()
turtle.vel.linear.x = 0.05

PAUSE = 0
TIME = time.time()

def handler(signum, frame):
    global PAUSE, TIME
    if time.time() - TIME < 1:
        print("CHIDUO")
        exit(1)
    
    if not PAUSE:
        turtle.vel.linear.x = 0
        turtle.vel.angular.z = 0
        PAUSE = 1
        TIME = time.time()

        pub.publish(turtle.vel)

        fig, ax = plt.subplots()
        plt.axis("equal")
        plt.grid(True, which="minor", color="w", linewidth = .6, alpha = 0.5)

        #plt.imshow(occmpa_uncert,cmap = "PiYG_r")
        print(myhistogram2.shape)
        plt.bar(np.arange(18)*10,myhistogram)
        plt.bar(np.arange(18)*10+2,myhistogram2)
        plt.show()
    else:
        turtle.vel.linear.x = 0.1
        PAUSE = 0
        pub.publish(turtle.vel)


signal.signal(signal.SIGINT, handler)


while not rospy.is_shutdown():
    rate.sleep()