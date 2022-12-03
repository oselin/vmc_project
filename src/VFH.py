import numpy as np
import os


def magnitude_matrix(LIDAR_readings, confidence_matrix):

    # Simplify the notation
    shape = confidence_matrix.shape

    # Create an empty confidence matrix
    m = np.zeros(shape)

    # Determine the a-coefficient according to its definition
    a = np.amin(LIDAR_readings)**2

    # Calculate the discretization ratio
    discr_ratio = LIDAR_readings[0]/(shape[1]/2)

    for i in range(shape[0]):
        for j in range(shape[1]):
            distance = discr_ratio * np.sqrt( (shape[0] - i)**2 + (shape[1]/2 - j)**2 )
            m[i,j] = a*confidence_matrix[i,j]**2/distance**2
    return m


def polar_histogram(LIDAR_readings, CARTESIAN_readings, angle_range=180, active_region=5):

    # Convert angle range in radians
    angle_range = angle_range/180*np.pi

    # Convert active region in radians
    active_region = active_region/180*np.pi

    # Estimate the magnitude matrix
    magnitudes = magnitude_matrix(LIDAR_readings,CARTESIAN_readings)

    # Initialization of the histogram array
    histogram = np.zeros([int(np.floor(angle_range/active_region))])

    # Definition of the robot position in the magnitude matrix
    robot_pos = [magnitudes.shape[0],magnitudes.shape[1]/2]

    # Filling the histogram array
    for i in range(magnitudes.shape[0]):
        for j in range(magnitudes.shape[1]):
            angle = np.arctan2(robot_pos[0]-i, j - robot_pos[1])
            histogram[int(np.floor(angle/active_region))] += magnitudes[i,j]
    return histogram