import numpy as np


def magnitude_matrix(LIDAR_readings, CARTESIAN_readings):

    # Simplify the notation
    shape = CARTESIAN_readings.shape

    # Create an empty confidence matrix
    m = np.zeros([shape])

    # Determine the a-coefficient according to its definition
    a = np.amin(LIDAR_readings)

    # Calculate the discretization ratio
    discr_ratio = LIDAR_readings[0]/(shape[1]/2)

    for i in range(shape[0]):
        for j in range(shape[1]):
            
            distance = discr_ratio * np.sqrt( (shape(0) - i)**2 + (shape(1) - j)**2 )
            m[i,j] = a*CARTESIAN_readings[i,j]**2/distance**2
    return m


def polar_histogram(LIDAR_readings, CARTESIAN_readings, angle_range=180, active_region=5):

    magnitudes = magnitude_matrix(LIDAR_readings,CARTESIAN_readings)
    histogram = np.zeros([angle_range/active_region])
    robot_pos = [magnitudes.shape[0],magnitudes[1]/2]

    idx = 0
    for i in range(magnitudes.shape[0]):
        for j in range(magnitudes.shape[1]):
            angle = np.arctan2((j-robot_pos[1]),i-robot_pos[0])
            histogram[np.floor(angle/active_region)] += magnitudes[i,j]
            
    return histogram


