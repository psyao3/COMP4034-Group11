#!/usr/bin/env python

import rospy
from smach import StateMachine
from smach_ros import SimpleActionState
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from geometry_msgs.msg import Twist, Pose2D, Point
from nav_msgs.msg import Odometry

from callbacks import  *
from actionlib_msgs.msg import *
import actionlib

waypoints = [
    ["a", Point(-1.1, 3.8, 0)],
    ["b", Point(-0.88, -3.63, 0)],
    ["c", Point(-0.13, -0.73, 0)],
    ["d", Point(0.19, 4.25, 0)],
    ["e", Point(4.23, 3.96, 0)],
    ["f", Point(3.93, 2.43, 0)],
    ["g", Point(1.48, 2.38, 0)],
    ["h", Point(0.9, -1.44, 0)],
    ["i", Point(0.79, -3.77, 0)],
    ["j", Point(3.48, -3.9, 0)],
    ["k", Point(2.53, -1.42, 0)],
    ["l", Point(2.89, 0.98, 0)],
    ["m", Point(4.86, 0.85, 0)],
    ["n", Point(5.85, 3.48, 0)],
    ["o", Point(5.34, -2.33, 0)]
]



def move_to_target(coordinates):
    # Create a client to send goal requests to the move_base server through
    # a SimpleActionClient
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    while not client.wait_for_server(rospy.Duration.from_sec(10.0)):
        rospy.loginfo("Waiting for the move_base action server")

    goal = MoveBaseGoal()

    # important because "map" defines we are using map coordinates 
    # and not coordinates relative to the robot
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    #set positions of the goal location
    goal.target_pose.pose.position = coordinates
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.0
    goal.target_pose.pose.orientation.w = 1.0

    rospy.loginfo("Sending goal location")
    client.send_goal(goal)

    client.wait_for_result(rospy.Duration(60))
    
    if (client.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.loginfo("You have reached the destination")
        return True
    else:
        rospy.loginfo("The robot failed to reach the destination")
        return False


# When this file is run:
if __name__ == '__main__':
    # Initialize node.
    rospy.init_node('tour')


    for i, data in enumerate(waypoints):
        node = data[0]
        location = data[1]
        move_to_target(location)
        

