from geometry_msgs.msg import Twist
import numpy as np

class BurgerRobot():
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
        self.Kp = 1.2 #0.67
        self.Ki = 0.0#0.0045
        self.Kd = 0.56# 0.097

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