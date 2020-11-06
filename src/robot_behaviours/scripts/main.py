#!/usr/bin/env python

import rospy
import tf
import math
import random
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from math import cos, sqrt, pi
import numpy as np
from calculator import *

class robot_behaviours:
   
    # Constructor
    def __init__(self):
       
        self.rate = rospy.Rate(10) # hz

        # Speeds to use        
        self.linear_speed = 0.4
        self.angular_speed = 0.4

        # Flag to determine if a random walk is already in progress
        self.in_progress = False

        # Initalize pose objects for tracking distance/angle moved in the random walk.
        self.pose = Pose2D()
        self.initial_pose = Pose2D()
        self.random_angle = 0.0

        # Define publisher to issue Twist messages depending on the current perception (scan data)
        # Messages will be sent within the scan_callback
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Define subscribers to listen to messages on /scan  and /odom topics 
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)


    def odom_callback(self, msg):
        # Get (x, y, theta) specification from odometry topic
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)

        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y
    

    def scan_callback(self, msg):
        # The ranges array contains the distance to an obstacle in those directions.
        # I.e. ranges[0] is the distance to the obstance directly in front.
        #      ranges[90] is the distance to the obstacle directly left.
        #      ranges[270] is the distance to the obstacle directly right.
        # Need to work within a range e.g. +- 10 degrees of a direction and compute the average.

        ranges = np.array(msg.ranges)

        # Decide on the behaviour to use

        #### OBSTACLE AVOIDANCE

             #### OBSTACLE AVOIDANCE
        
        # Intitialize empty list
        obst_dists = []

        # Get the average distance to obstacles around the turtlebot in specified ranges (in x degree segments)
        # Observes a frontal cone in front of the turtlebot e.g (90 -> 0 -> 270)
        # TODO: fix overlap between different ranges
        for i in range(-45, 45, 8):
            obst_dists.append(self.average_distance(ranges, i, 8))

        # Check obst_dists list to check if any are closer than 0.25
        # Returns true if any values are <= 0.25, indicating a close obstacle.
        #is_obstacle = any(x <= 0.25 for x in obst_dists)
        #print(len(obst_dists))
        #threshold = 0.25
        right_obst = min(obst_dists[0:5])
        front_obst = min(obst_dists[5:7])
        left_obst = min(obst_dists[7:])

        
        twist_msg = Twist()

        # Note: positive angular speed is anticlockwise,
        #       negative is clockwise.
        # Left side 
        if left_obst <= 0.5:
            rospy.loginfo("Obstacle left; Turning clockwise")
            twist_msg.angular.z = -self.angular_speed
            is_obstacle = True
        # Right side
        elif right_obst <= 0.5:
            rospy.loginfo("Obstacle right; Turning anticlockwise")
            twist_msg.angular.z = self.angular_speed
            is_obstacle = True
        # Ahead
        elif front_obst <= 0.5:
            rospy.loginfo("Obstacle ahead; Turning anticlockwise")
            twist_msg.angular.z = self.angular_speed
            is_obstacle = True
        else: 
            # No obstacle
            twist_msg.angular.z = 0 
            is_obstacle = False


       #left_obst, right_obst, front_obst, front_left_obst, front_right_obst = self.get_obstacle_distances(ranges)
     
        #### RIGHT HAND WALL FOLLOWING

       

        #### GET TWIST MESSAGE
        # May need to swap obstacle/wall behaviour

        if is_obstacle:

            #twist_msg = Twist()


            self.in_progress = False
            # Use obstacle avoidance behaviour
           
        else:
            # Random walk
            #rospy.loginfo("Behaviour: Random walk")
            twist_msg = self.random_walk(ranges, min(front_obst, left_obst, right_obst))
               
       
        self.pub.publish(twist_msg)
       
        # Sleep according to the rate
        self.rate.sleep()
        




    # Walk 3m, rotate a random angle, repeat.
    def random_walk(self, ranges, closest_obst):
        twist_msg = Twist()

        # If not currently in progress, start fresh with current position as the 'origin'/initial
        if not self.in_progress:
            rospy.loginfo("Starting random walk")
            self.initial_pose = Pose2D(self.pose.x, self.pose.y, self.pose.theta)
            self.in_progress = True
            while abs(self.random_angle) < 1:
                self.random_angle = random.uniform(-math.pi, math.pi)

        # Issue stop command if there is a close obstacle
        if(closest_obst <= 0.25):
            self.in_progress = False
            return Twist()

        # Move 3m
        if calculate_linear_distance(self.initial_pose, self.pose) < 3:
            rospy.loginfo("Walked %3fm out of 3m" % calculate_linear_distance(self.initial_pose, self.pose))

            #front_obst = np.mean(ranges[range(-20, 20)])

            # Need to set the speed proportional to the distance ahead - slow it down if approaching an object            
            twist_msg.linear.x = min(self.linear_speed * closest_obst, self.linear_speed)
        # Turn a random angle
        elif calculate_angular_distance(self.initial_pose, self.pose) < self.random_angle:
            rospy.loginfo("Turned %3f radians out of %3f" % (calculate_angular_distance(self.initial_pose, self.pose), self.random_angle))
            twist_msg.angular.z = self.angular_speed
        # Reset
        else:
            self.in_progress = False

        return twist_msg


    # Calculate the average distance around the index given, to specified degree
    # Need a special case for values around 0.
    def average_distance(self, ranges, index, degree):
        
        # Get range of angles to compute average distance over
        angles = range(index - degree, index + degree - 1)

        # enumerate gives the index and value at the same time.
        # this is to allow wrap around if index + degrees exceeds the array bounds
        for i, ang in enumerate(angles):
            if ang > 360:
                angles[i] = ang - 360
            # If index - degrees is negative, it indexes from the end of the array, e.g
            # -1 is 359, so conversion should not be needed.

        # Convert ranges to np array so list indexing can be used (using angles list)
        arr = np.array(ranges)
        avg = np.mean(arr[angles])
        
        # Return average distance
        return avg


if __name__ == '__main__':
    try:
        # Initiate node
        rospy.init_node('robot_behaviours', anonymous=True)
        robot_behaviours()

        # Continue until terminated
        rospy.spin()

    except rospy.ROSInterruptException:
        pass