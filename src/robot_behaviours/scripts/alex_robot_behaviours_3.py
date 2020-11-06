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

class RobotBehaviours():
   
    # Constructor
    def __init__(self):
       
        # Instantiate any instance variables that will be needed for each of the behaviors

        # Cone of view divided up into x degree segments.
        self.degrees = 10 # Define average distance to be calculated over +- x degrees in a direction.

        self.rate = rospy.Rate(10) # hz
                  
        self.linear_speed = 0.4
        self.angular_speed = 0.4

        self.walking = True
        self.random_moving = False

        self.dist_to_wall = 0.5
        self.states = 0
        self.position_window = {
            'right': 0,
            'front_right': 0,
            'front': 0,
            'front_left': 0,
            'left': 0,
        }

        self.pose = Pose2D()
        self.pose.x, self.pose.y, self.pose.theta = 0.0, 0.0, 0.0
        self.initial_pose = Pose2D()
        self.initial_pose.x, self.initial_pose.y, self.initial_pose.theta = 0.0, 0.0, 0.0
        self.random_angle = 0.0

        self.trajectory = []
        self.logging_counter = 0

        # Define publisher to issue Twist messages depending on the current perception (scan data)
        # Messages will be sent within the scan_callback
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Define subscribers to listen to messages on /scan  and /odom topics 
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)


    def odom_callback(self, msg):
        # Get (x, y, theta) specification from odometry topic
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)

        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y
        
        # Save values for trajectory
        self.logging_counter += 1
        if self.logging_counter == 50:
            self.logging_counter = 0
            self.trajectory.append([self.pose.x, self.pose.y])
    

    def scan_callback(self, msg):
        # The ranges array contains the distance to an obstacle in those directions.
        # I.e. ranges[0] is the distance to the obstance directly in front.
        #      ranges[90] is the distance to the obstacle directly left.
        #      ranges[270] is the distance to the obstacle directly right.
        # Need to work within a range e.g. +- 10 degrees of a direction and compute the average.

        ranges = msg.ranges
        rospy.loginfo(ranges)

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

        # Look at a range e.g. 90 degrees range about 270
        # Assuming there is a wall nearby, a and b form two edges of the triangle
        a = ranges[314] # Since indexing starts at 0 
        b = ranges[225]

        # Use law of cosines to get unknown side, the length of the wall:
        c = sqrt(a**2 + b**2 - (2 * a * b * cos(60)))

        # Could set a threshold value, e.g. if c < threshold,
        # then is_wall = True
        # it can be assumed it is part of a nearby wall

        # TODO:
        # Determine if there is a wall on the right, if so set is_wall to true.
        if c > 0.3:
            is_wall = True
        else:
            is_wall = False

        self.position_window = {
            'right': min(ranges[270:306]),
            'front_right': min(ranges[307:342]),
            'front': min(min(ranges[0:16]),min(ranges[343:359])),
            'front_left': min(ranges[17:53]),
            'left': min(ranges[54:90]),
        }


        #### GET TWIST MESSAGE
        # May need to swap obstacle/wall behaviour

        if is_obstacle:

            twist_msg = Twist()
            self.pub.publish(twist_msg)



            # Use obstacle avoidance behaviour
            rospy.loginfo("Behaviour: Obstacle Avoidance")
            twist_msg = self.obstacle_avoidance(ranges_array = ranges)
        elif is_wall:
            # Use right hand wall following behaviour
            rospy.loginfo("Behaviour: Right hand wall following")
            self.position_decider()
            twist_msg = self.wall_following()
        else:
            # Random walk
            rospy.loginfo("Behaviour: Random walk")
            twist_msg = self.random_walk()
               
        # Remove later
        #twist_msg = Twist()
        
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
    
    def position_decider(self):

        if self.position_window['front'] > self.dist_to_wall and self.position_window['front_right'] > self.dist_to_wall \
            and self.position_window['front_right'] > self.dist_to_wall and self.position_window['right'] > self.dist_to_wall:
            # random_wander maybe?
            self.states = 0
        elif self.position_window['front'] < self.dist_to_wall and self.position_window['front_right'] > self.dist_to_wall \
            and self.position_window['front_left'] > self.dist_to_wall:
            self.states = 1
        elif self.position_window['front'] > self.dist_to_wall and self.position_window['front_right'] < self.dist_to_wall \
            and self.position_window['front_left'] > self.dist_to_wall:
            self.states = 2
        elif self.position_window['front'] > self.dist_to_wall and self.position_window['front_right'] > self.dist_to_wall \
            and self.position_window['front_left'] < self.dist_to_wall:
            # random_wander again
            self.states = 0
        elif self.position_window['front'] < self.dist_to_wall and self.position_window['front_right'] < self.dist_to_wall \
            and self.position_window['front_left'] > self.dist_to_wall:
            self.states = 1
        elif self.position_window['front'] < self.dist_to_wall and self.position_window['front_right'] > self.dist_to_wall \
            and self.position_window['front_left'] < self.dist_to_wall:
            self.states = 1
        elif self.position_window['front'] < self.dist_to_wall and self.position_window['front_right'] < self.dist_to_wall \
            and self.position_window['front_left'] < self.dist_to_wall:
            self.states = 1
        elif self.position_window['front'] > self.dist_to_wall and self.position_window['front_right'] < self.dist_to_wall \
            and self.position_window['front_left'] < self.dist_to_wall:
            self.states = 0   
        else:
            rospy.loginfo("Error locating position")

    def wall_following(self):
        msg = Twist()
        if self.states == 0:
            rospy.loginfo("finding wall")
            msg = self.random_walk()
        elif self.states == 1:
            rospy.loginfo("rotating left")
            msg = self.move(0.2, 0)
        elif self.states == 2:
            rospy.loginfo("following wall")
            msg = self.move(0, 0.2)
        else:
            rospy.loginfo("Error")

        return msg
    
    def move(self, angle, speed):
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = abs(angle)

        return msg
    
    def random_walk(self):
        twist_msg = Twist()
        

        #if the robot was perfoming a different action previously 
        #set the current positions. so in the next call if the robot is still randomly moving,
        # the distance calculations aren't reset
        if (self.random_moving == False):

            self.initial_pose = Pose2D()
            self.initial_pose.x, self.initial_pose.y, self.initial_pose.theta = self.pose.x, self.pose.y, self.pose.theta
            #only runs when randomly moving again for the first time
            self.random_moving = True


            #temporary random angle between 5 and 10
            self.random_angle = random.uniform(5,10)

        #2 states walking or rotating. 
        #if walking is current random movement then calculate the current position based on the initial position and compare to see if the robot has travelled 3m
        if (self.walking == True):
            if((math.sqrt((self.initial_pose.x - self.pose.x)**2 + (self.initial_pose.x - self.pose.y)**2) < 3)):
                twist_msg.linear.x = self.linear_speed
            #if the else statement is called then the robot must have travelled its 3m
            #change current movement mode to not walking
            #set random_moving to false in order to re-initialise the pose variables 
            else:
                self.random_moving = False
                self.walking = False
        #if the current movement mode is rotating
        else:
            if (self.initial_pose.theta - self.pose.theta) > pi:
                angle = (self.initial_pose.theta - self.pose.theta) - (2*pi)             
            else:
                angle = (self.initial_pose.theta - self.pose.theta) + (2*pi)

            
            if(angle  < self.random_angle):
                twist_msg.angular.z = self.angular_speed
            else:
                self.random_moving = False
                self.walking = True


        return twist_msg

if __name__ == '__main__':
    try:
        # Initiate node
        rospy.init_node('robot_behaviours', anonymous=True)
        RobotBehaviours()

        # Continue until terminated
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

