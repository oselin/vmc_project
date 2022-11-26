#! /usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from numpy import inf
import os, sys
from scipy.ndimage.filters import convolve as conv2
from sensor_msgs.msg import LaserScan  

sys.path.append(".")
from src.VFH import polar_histogram


#initialization of the node
rospy.init_node('reader') 


## Plot Settings
plt.ion()
fig, ax = plt.subplots()
plt.axis("equal")
plt.grid(True, which="minor", color="w", linewidth = .6, alpha = 0.5)


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



def plotter(ranges):
    dist, ang = get_front(ranges)
    dist = 4*dist
    dist[dist == inf] = 100

    ox = dist*np.cos(ang)
    oy = dist*np.sin(ang)
   
    occmap = occupancy_map([ox,oy])

    #g, _,_,_,_,_ = gaussian2(0.5)
    g = gaussian_kernel(5,0.5)
    occmpa_uncert = conv2(occmap,g,mode ="reflect")

    myhistgram = polar_histogram(dist, occmpa_uncert)    

    os.system("clear")
    print(myhistgram)
    """ax.clear()
    ax.bar(np.arange(36),myhistgram)
    ax.set_xlim(0,36)
    ax.set_ylim(0,100)
    ax.autoscale(False)"""
    #plt.clf()
    plt.bar(np.arange(36),myhistgram)
    #plt.imshow(occmpa_uncert,cmap = "PiYG_r")
    plt.show()
    
    #fig.canvas.flush_events()



def callback(msg):
    #print(msg.ranges)
    plotter(msg.ranges)


sub = rospy.Subscriber('/scan' , LaserScan , callback)
rate = rospy.Rate(100)


plt.show(block=True)

while not rospy.is_shutdown():
    rate.sleep()