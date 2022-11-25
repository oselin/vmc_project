#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import os
import math


# Class for colored stdout on terminal
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class myTurtle():
    def __init__(self):

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
    

    def PID_controller(self):
        
        if self.is_running:
            if not self.dl == float("inf")  and not self.dr == float("inf"):        
                
                self.error = self.dl - self.dr
                self.delta_error = self.error - self.previous_error
                self.integr_error += self.error
                
                P = self.Kp*self.error
                I = self.Ki*self.integr_error*self.sample_time
                D = self.Kd*self.delta_error/self.sample_time
                
                self.previous_error = self.error

                gas = P+I+D
                
            # if the vehicle is so rotated to see infinity
            elif self.dl == float("inf")  and not self.dr == float("inf"): 
                gas = 1
            elif not self.dl == float("inf")  and self.dr == float("inf"): 
                gas = -1
            else:
                self.stop()
                gas = 0

            self.vel.angular.z = gas
            print_data(self)


def print_data(vehicle: myTurtle):
    os.system('clear')

    print("Vehicle information:")
    print(vehicle.vel)
    print(".............................")
    print("RIGHT:", vehicle.dr)
    print("LEFT :", vehicle.dl)
    print(".............................")
    if vehicle.error == 0:
        print("ERROR:", vehicle.error)
    else:
        if vehicle.vel.angular.z/vehicle.error > 0:
            print(f"{bcolors.WARNING}ERROR: {vehicle.error}{bcolors.ENDC}")
        else:
            print("ERROR:", vehicle.error)
    print("DERIV:",vehicle.delta_error)
    print("INTEG:",vehicle.integr_error)
    

# Read data from input and generate a steering command
def callback(msg, turtle):
    turtle.dr, turtle.dl = msg.ranges[295], msg.ranges[65]
    turtle.PID_controller()


def main():

    # Initialize the myTurtle object. It will be the "digital twin" of the turtle robot
    turtle = myTurtle()

    # Initialize the node as publisher and subscriber
    rospy.init_node('PIDcontroller')
    
    # Initialize Subscriber and Publisher
    sub = rospy.Subscriber('/scan', LaserScan, callback,(turtle))
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)

    

    # Define the update rate for collecting data from sensors and update steering inputs
    rate = rospy.Rate(1/turtle.sample_time)

    print_data(turtle)

    turtle.is_running = 1
    turtle.vel.linear.x = 0.3
    
    while not rospy.is_shutdown():

        pub.publish(turtle.vel)
        if not turtle.is_running: exit()
        rate.sleep()


if __name__ == '__main__':
    main()