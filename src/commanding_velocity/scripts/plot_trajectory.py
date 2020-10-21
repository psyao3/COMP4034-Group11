#!/usr/bin/env python

## Plot trajectory

import rospy
from math import pi, sqrt
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

class plot_trajectory:
   
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

        
        self.rate = rospy.Rate(5) # hz

        # Define subscriber to listen to messages on /odom topic for the turtlebot pose
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # Intitalise and display the graph
        ani = FuncAnimation(plt.gcf(), self.animate, interval = 50)
        plt.show()



    def animate(self, i):

        # Add new pose information
        self.xs.append(self.x)
        self.ys.append(self.y)
        self.time.append(rospy.Time.now().to_sec())

        # Clear current graph
        plt.cla()

        # Plot new graph with updated xs, ys, time.
        plt.plot(self.time, self.xs, label = "x Position")
        plt.plot(self.time, self.ys, label = "y Position")

        # Set constant legend position
        plt.legend(loc='upper left')
        plt.tight_layout()



    def odom_callback(self, msg):
        # Get (x, y, theta) specification from odometry topic
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)

        self.theta = yaw
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y


if __name__ == '__main__':
    try:
        # Initiate node
        rospy.init_node('trajectory', anonymous=True)
        pt = plot_trajectory()
        #rospy.spin()

    except rospy.ROSInterruptException:
        pass

