#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import cos, sqrt
import numpy as np

class robot_behaviours:
   
    # Constructor
    def __init__(self):
       
        # Instantiate any instance variables that will be needed for each of the behaviors

        # Cone of view divided up into x degree segments.
        self.degrees = 10 # Define average distance to be calculated over +- x degrees in a direction.

        self.rate = rospy.Rate(10) # hz
         
        self.linear_speed = 0.4
        self.angular_speed = 0.4

        # Define publisher to issue Twist messages depending on the current perception (scan data)
        # Messages will be sent within the scan_callback
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Define subscriber to listen to messages on /scan topic 
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)


    def scan_callback(self, msg):
        # The ranges array contains the distance to an obstacle in those directions.
        # I.e. ranges[0] is the distance to the obstance directly in front.
        #      ranges[90] is the distance to the obstacle directly left.
        #      ranges[270] is the distance to the obstacle directly right.
        # Need to work within a range e.g. +- 10 degrees of a direction and compute the average.

        ranges = msg.ranges
        #rospy.loginfo(ranges)

        # Decide on the behaviour to use

        #### OBSTACLE AVOIDANCE
        
        # Intitialize empty list
        obst_dists = []

        # Get the average distance to obstacles around the turtlebot in specified ranges (in x degree segments)
        # Observes an x degree cone in front of the turtlebot e.g (90 -> 0 -> 270)
        # TODO: fix overlap between different ranges
        for i in range(-80, 80, self.degrees):
            obst_dists.append(self.average_distance(ranges, i, self.degrees))

        # Check obst_dists list to check if any are closer than 0.25
        # Returns true if any values are <= 0.25, indicating a close obstacle.
        is_obstacle = any(x <= 0.25 for x in obst_dists)
       


        #### RIGHT HAND WALL FOLLOWING

        # Compute if there is a straight line to the right by looking at the distances observed by scan
        # If so, there is a wall to follow

        # Look at a range e.g. 20 degrees range about 270
        # Assuming there is a wall nearby, a and b form two edges of the triangle
        a = ranges[289] # 0 index, so -1
        b = ranges[250]

        # Use law of cosines to get unknown side, the length of the wall:
        c = sqrt(a**2 + b**2 - (2 * a * b * cos(40)))

        # Could set a threshold value, e.g. if c < threshold,
        # then is_wall = True
        # it can be assumed it is part of a nearby wall
        if c <= 1.25:
            is_wall = True
        else:
            is_wall = False


        #### GET TWIST MESSAGE
        # May need to swap obstacle/wall behaviour

        if is_obstacle:
            # Use obstacle avoidance behaviour
            rospy.loginfo("Behaviour: Obstacle Avoidance")
            twist_msg = self.obstacle_avoidance(ranges_array = ranges)
        elif is_wall:
            # Use right hand wall following behaviour
            rospy.loginfo("Behaviour: Right hand wall following")
            twist_msg = self.wall_following()
        else:
            # Random walk
            rospy.loginfo("Behaviour: Random walk")
            twist_msg = self.random_walk()
               
        # Remove later
        twist_msg = Twist()
        
        # Publish the twist message
        self.pub.publish(twist_msg)

        # Sleep according to the rate
        self.rate.sleep()
        

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


    # Basic method signatures, which need to return a twist message.
    # Add extra parameters as needed.
    def obstacle_avoidance(self, ranges_array):
        
        window_size = 4
        threshold = 0.25
        front_right = [45-window_size/2, 45+window_size/2]
        front_left = [315-window_size/2, 315+window_size/2]

        twist_msg = Twist()

        if np.mean(ranges_array[front_right[0]:front_right[1]]) < threshold:
            twist_msg.angular.z = -self.angular_speed
        elif np.mean(ranges_array[front_left[0]:front_left[1]]) < threshold:
            twist_msg.angular.z = self.angular_speed
        else: 
            twist_msg.angular.z = self.angular_speed
            
        return twist_msg
    
    def wall_following(self):
        return
    
    def random_walk(self):
        return



if __name__ == '__main__':
    try:
        # Initiate node
        rospy.init_node('robot_behaviours', anonymous=True)
        robot_behaviours()

        # Continue until terminated
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
