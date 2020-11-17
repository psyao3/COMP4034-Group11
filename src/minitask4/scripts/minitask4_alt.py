#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import cv2, cv_bridge
import numpy as np
import math
import random
import tf

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
import random
import grid_methods as grid

# note to self make sure the navigation file is running
# roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=`rospack find minitask4`/maps/mymap.yaml

class Waypoints:

    def __init__(self):

        # Rooms 1 to 6. Found using rostopic echo /amcl_pose, and setting the 2d Pose
        # estimate in RViz to locations of interest (each room).
        self.rooms = [Point(4.6,0.8,0),
                      Point(5.9,-4.5,0),
                      Point(1.1,3.8,0),
                      Point(-3.3,4.2,0),
                      Point(-5.5, 4.78, 0),
                      Point(-5.74, -3.13, 0)]



        while not rospy.is_shutdown():

            # Lists evaluate to false if empty, so this will run when finished.
            if not self.rooms:
                rospy.loginfo("Visited all destinations.")
                rospy.signal_shutdown("Finished!")
            # Pop the next target from the list.
            else:        
                self.move_to_target(self.rooms.pop(0))

   


    def move_to_target(self, coordinates):
        # Create a client to send goal requests to the move_base server through
        # a SimpleActionClient
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

        while not client.wait_for_server(rospy.Duration.from_sec(5.0)):
            rospy.loginfo("Waiting for the move_base action server")

        goal = MoveBaseGoal()

        # important because "map" defines we are using map coordinates 
        # and not coordinates relative to the robot
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        #set positions of the goal location
        goal.target_pose.pose.position =  coordinates
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo("Sending goal location")
        client.send_goal(goal)

        client.wait_for_result(rospy.Duration(60))

        
        if(client.get_state() ==  GoalStatus.SUCCEEDED):
            rospy.loginfo("You have reached the destination")
        else:
            rospy.loginfo("The robot failed to reach the destination")
            

       



if __name__ == '__main__':
    try:
        rospy.init_node('waypoint')
        waypoint = Waypoints()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print("An exception was caught.")
        raise e
