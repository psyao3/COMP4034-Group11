#!/usr/bin/env python

## Closed Loop Control

import rospy
from math import pi, sqrt
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

class closed_loop_control:
   
    # Constructor
    def __init__(self):
       
        # Instantiate x, y and theta, the current positions that will be updated by odom_callback
        self.x = 0
        self.y = 0
        self.theta = 0
        
        # Intialise numpy arrays to record pose
        self.xs = []
        self.ys = []
        self.time = []

        # Define publisher to send Twist messages on /cmd_vel topic.
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(100) # hz

        # Define subscriber to listen to messages on /odom topic.
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # Draw the square
        self.draw_square()

    def odom_callback(self, msg):
        # Get (x, y, theta) specification from odometry topic
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)

        self.theta = yaw
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.pRatex)
        np.append(self.ys, self.y)
        np.append(self.time, rospy.Time.now().to_sec())


            
    def draw_square(self):
      
        # Define twist messages for moving forward, and rotating
        move = Twist()
        move.linear.x = 0.25

        turn = Twist()
        turn.angular.z = pi/8

        # Make sure it isn't moving initially
        stop = Twist()
        self.pub.publish(stop)

        # Count the number of edges so far.
        edges = 0     

        # Loop to draw the square
        while not rospy.is_shutdown() and edges < 4:
            
            # Get the previous target (inital position for this iteration)
            x_p = self.x
            y_p = self.y
            # Get the target co-ordinatestrajectory
            # Move forward until target reached/exceeded
            while self.pythagoras(x_p, y_p) < 1:
                # Move forward 
                self.pub.publish(move)
                rospy.loginfo("Moving forward, currently %f from last objective" % self.pythagoras(x_p, y_p) )
                #self.rate.sleep()
            
            # Stop moving
            self.pub.publish(stop)

            # Get current theta
            theta_p = self.theta

            while self.angular_distance(theta_p) < pi/2:
                self.pub.publish(turn)
                rospy.loginfo("Turning, current angle %f" % self.theta)
                #self.rate.sleep()
            
            # Stop turning
            self.pub.publish(stop)

            # Increment number of edges
            edges += 1

    def angular_distance(self, theta_prime):
        # Angle between current theta from odom and theta_prime
        # Basically theta_prime is the new 'origin', and must turn pi/2 radians from this

        # Convert to the scale with 2pi
        if self.theta < 0:
            theta_cur = 2*pi + self.theta
            theta_prime = 2*pi + theta_prime
            return abs(theta_cur - theta_prime)

        ang = abs(self.theta - theta_prime)
        return ang


    def pythagoras(self, x_prime, y_prime):
        # Distance between current position updated by odom and x',y'
        dist = sqrt((x_prime -  self.x)**2 + (y_prime - self.y)**2)
        return dist

    # Returns true if target angle has not yet been reached (so bot keeps turning)
    def check_angle (self, theta_t):
        
        # Convert odometry reading to 2pi scale if it is negative
        if self.theta < 0:
            theta_cur = 2*pi + self.theta
        else:
            theta_cur = self.theta

        # Comparison
        if theta_cur < theta_t:
            return True
        else:
            return False
        

if __name__ == '__main__':
    try:
        # Initiate node
        rospy.init_node('closed_loop_control', anonymous=True)
        clc = closed_loop_control()

    except rospy.ROSInterruptException:
        pass
