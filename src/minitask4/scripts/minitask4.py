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

# note to self make sure the navigation file is running
# roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=`rospack find minitask4`/maps/mymap.yaml

class Waypoints:

    def __init__(self):

        #only have coordinates set up for 4 rooms at the moment
        self.rooms = []
        
        room1 =  Point(4.6,0.8,0)
        self.rooms.append(room1)

        room2 =  Point(5.9,-4.5,0)
        self.rooms.append(room2)

        room3 =  Point(1.1,3.8,0)
        self.rooms.append(room3)
 
        room4 =  Point(-3.3,4.2,0)
        self.rooms.append(room4)

        counter = -1

        while not rospy.is_shutdown():

            #find the next waypoint to move to
            if counter+1 == len(self.rooms):
                counter = -1
            counter+=1

            rospy.loginfo("counter = " + str(counter))
        
            self.move_to_target(self.rooms[counter])

    


    def move_to_target(self, coordinates):
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

        rospy.loginfo("sending goal location")
        client.send_goal(goal)

        wait = client.wait_for_result(rospy.Duration(60))

        
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
