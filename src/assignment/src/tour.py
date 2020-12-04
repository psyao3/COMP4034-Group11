#!/usr/bin/env python

import rospy
import Graph
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from geometry_msgs.msg import Twist, Pose2D, Point
from nav_msgs.msg import Odometry

from callbacks import  *
from actionlib_msgs.msg import *
import actionlib

graph = { "a" : ["b"],
          "b" : ["a","c"],
          "c" : ["b","d"],
          "d" : ["c", "e", "k"],
          "e" : ["d", "f", "n"],
          "f" : ["e", "g"],
          "g" : ["f", "h", "j"],
          "h" : ["g", "i"],
          "i" : ["h", "j"],
          "j" : ["g", "i"],
          "k" : ["d", "l"],
          "l" : ["k", "m"],
          "m" : ["l", "n"],
          "n" : ["m", "e", "o"],
          "o" : ["n", "p", "q"],
          "p" : ["o", "q"],
          "q" : ["o", "p", "r"],
          "r" : ["q"]
        }

node_locations = {
          "a" : Point(-1.1, 3.8, 0),
          "b" : Point(-1.1, 1.36, 0),
          "c" : Point(-1.1, -1.22, 0),
          "d" : Point(-0.01, -0.34, 0),
          "e" : Point(0.07, 0.98, 0),
          "f" : Point(0.18, 4.17, 0),
          "g" : Point(1.62, 4.11, 0),
          "h" : Point(4.14, 3.93, 0),
          "i" : Point(3.82, 2.32, 0),
          "j" : Point(1.45, 2.5, 0),
          "k" : Point(0.77, -1.63, 0),
          "l" : Point(0.612, -3.53, 0),
          "m" : Point(3.07, -3.6, 0),
          "n" : Point(2.66, 1.04, 0),
          "o" : Point(4.65, 0.88, 0),
          "p" : Point(5.77, 3.51, 0),
          "q" : Point(5.77, 0.78, 0),
          "r" : Point(5.77, -2.55, 0)
}

visited_nodes = {}
unvisited_nodes = node_locations.copy()

def patrol():
    # Empty dictionaries {} evaluate to False
    while not unvisited_nodes:
        # Get next node/key
        node = list(unvisited_nodes)[0]
        target = unvisited_nodes.pop(node)

        # Need to navigate to the target via defined edges

# Take the start/end node and existing path. If this is the first
# function call, then path = []
def find_path(graph, start_node, end_node, path):
    # Add the current start node to the path list.
    path.append(start_node)

    # Destination is reached, so the entire path is found
    if start_node == end_node:
        return path


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
        

