import imp
import numpy as np
from BurgerRobot import BurgerRobot
def label_data(data: np.array):
    text = ""

    text += f"STD:  {np.std(data)}\n"
    text += f"MEAN: {np.mean(data)}\n"
    text += f"MIN:  {np.amin(data)}\n"
    
    return text


def strong_avoid(occ_map,r):
    strong_occ_map = np.zeros(occ_map.shape)
    for i in range(r,occ_map.shape[0]-r):
        for j in range(r,occ_map.shape[1]-r):
            if occ_map[i,j] == 1:
                for a in range(-r,r):
                    for b in range(-r,r):
                        strong_occ_map[i+a,j+b] = 1  
    return strong_occ_map


def moving_average(x, w):
    bfr = []
    i = 0
    while i < len(x) - w :
    
        window = x[i : i + w]
        window_average = sum(window) / w
        bfr.append(window_average)

        i += 1 
    bfr = np.concatenate(([bfr[i] for i in range(int(w/2)-1,-1,-1)], 
                           bfr, 
                           [bfr[-i] for i in range(1,int(w/2)+1)]), 
                        axis=0)
    
    return bfr


def gaussian2(sigma, N=None):

    if N is None:
        N = 2*np.maximum(4, np.ceil(6*sigma))+1
    k = (N - 1) / 2.
    xv, yv = np.meshgrid(np.arange(-k, k+1), np.arange(-k, k+1))

    # 2D gaussian filter    
    g = 1/(2 * np.pi * sigma**2) * np.exp(-(xv**2 + yv**2) / (2 * sigma**2))

    # 1st order derivatives
    #gx = -xv / (2 * np.pi * sigma**4) * np.exp(-(xv**2 + yv**2) / (2 * sigma**2))
    #gy = -yv / (2 * np.pi * sigma**4) * np.exp(-(xv**2 + yv**2) / (2 * sigma**2)) 

    # 2nd order derivatives
    #gxx = (-1 + xv**2 / sigma**2) * np.exp(-(xv**2 + yv**2) / (2*sigma**2)) / (2 * np.pi * sigma**4)
    #gyy = (-1 + yv**2 / sigma**2) * np.exp(-(xv**2 + yv**2) / (2*sigma**2)) / (2 * np.pi * sigma**4)
    #gxy = (xv * yv) / (2 * np.pi * sigma**6) * np.exp(-(xv**2 + yv**2) / (2*sigma**2))    

    # Normalize the filter
    g /= g[int(np.floor(N/2)),int(np.floor(N/2))]

    return g #, gx, gy, gxx, gyy, gxy


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

def PID_controller(robot):

    robot.error = robot.goal_dir - np.pi/2
    robot.delta_error = robot.error - robot.previous_error
    robot.integr_error += robot.error
    
    P = robot.Kp*robot.error
    I = robot.Ki*robot.integr_error*robot.sample_time
    D = robot.Kd*robot.delta_error/robot.sample_time
    
    robot.previous_error = robot.error

    gas = P+I+D
 
    robot.vel.angular.z = gas