#!/usr/bin/env python

import rospy
import tf

from geometry_msgs.msg import Twist, Pose2D, Point
from costmap_2d import *
from nav_msgs.msg import Odometry, OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from sensor_msgs.msg import Image, PointCloud2, LaserScan
from visualization_msgs.msg import MarkerArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
import actionlib


class FrontierExploration:

    # Test class from attempted implementation of frontier_exploration
    # Not in use

    def __init__(self):

        print("Enter init")

        self.counter = 0

        self.rate = rospy.Rate(10)  # Hz

        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.map_updates_sub = rospy.Subscriber('/map_updates',  OccupancyGridUpdate, self.map_updates_callback)
        self.explore_sub = rospy.Subscriber('/explore/frontiers', MarkerArray, self.explore_callback)

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.pose = Pose2D()

        self.number_of_cells_x = None
        self.number_of_cells_y = None

        self.objects = [Point(4.6, 0.8, 0),
                        Point(5.9, -4.5, 0)]

        while not rospy.is_shutdown():

            if len(self.objects) != 0:
                current_target = self.objects.pop(0)
                while not self.move_to_target(current_target):
                    rospy.loginfo("Trying again.")
            else:
                pass

        rospy.spin()

    def move_to_target(self, coordinates):
        # Create a client to send goal requests to the move_base server through
        # a SimpleActionClient
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        while not client.wait_for_server(rospy.Duration.from_sec(5.0)):
            rospy.loginfo("Waiting for the move_base action server")

        goal = MoveBaseGoal()

        # important because "map" defines we are using map coordinates
        # and not coordinates relative to the robot
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # set positions of the goal location
        goal.target_pose.pose.position = coordinates
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo("Sending goal location")
        client.send_goal(goal)

        client.wait_for_result(rospy.Duration(60))

        if (client.get_state() == GoalStatus.SUCCEEDED):
            rospy.loginfo("You have reached the destination")
            return True
        else:
            rospy.loginfo("The robot failed to reach the destination")
            return False

    def odom_callback(self, msg):
        #print("Odom callback")
        # Get (x, y, theta) specification from odometry topic
        quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)

        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y



    def scan_callback(self, msg):
        #print("Scan callback")
        pass

    def map_callback(self, msg):
        print("Map callback")
        self.counter += 1
        print(self.counter)
        header = msg.header
        info = msg.info

        global_frame = header.frame_id

        self.number_of_cells_x = info.width
        self.number_of_cells_y = info.height

        resolution = info.resolution

        origin_x = info.origin.position.x
        origin_y = info.origin.position.y

        self.occ_grid = msg.data

    def explore_callback(self, msg):
        print('Explore callback')
        print(msg)

    def map_updates_callback(self, msg):
        # https://github.com/DLu/navigation/blob/92b5a4f2a2c6788bd1d7616c21694b1e49c548c0/costmap_2d/src/occupancy_grid_subscriber.cpp
        print("Update")

        def get_index(x, y):
            return x * self.number_of_cells_x + y

        index = 0

        for y in range(msg.y, msg.height):
            for x in range(msg.x, msg.width):
                index += 1
                self.occ_grid[get_index(x, y)] = msg.data[index]
                pass

    def stop(self):
        self.pub.publish(Twist())


if __name__ == '__main__':

    rospy.init_node('frontier_exploration')
    FrontierExploration()
    print("Success")

