#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Twist, Pose2D, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
import cv2, cv_bridge
import numpy as np
import math
import random
import tf
import tf2_ros
import tf2_geometry_msgs
import struct
import actionlib
from darknet_ros_msgs.msg import BoundingBoxes
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from calculator import calculate_linear_distance, calculate_angular_distance, average_distance
from image_processing import generate_mask, convert_to_hsv, show_image, find_closest_centroid
from actionlib_msgs.msg import *

from callbacks import *

class Follower:

    def __init__(self):

        # Initialising rospy behaviour
        rospy.on_shutdown(self.stop)
        self.rate = rospy.Rate(10)  # Hz

        # Speeds to use
        self.linear_speed = 0.4
        self.angular_speed = 0.05

        # Publishers and Subscribers
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.image_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, box_callback, (self))
        #self.depth_sub = rospy.Subscriber('camera/depth/image_raw', Image, depth_callback, (self))
        #self.cam_info = rospy.Subscribe('camera/depth/camera_info', CameraInfo, self.camera_info_callback)
        #self.cloud_sub = rospy.Subscriber('camera/depth/points', PointCloud2, self.cloud_callback)

        self.odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback, (self))
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, scan_callback, (self))

        # Initialise robot state
        self.state = 'Random Walk'

        # Initalise pose objects for tracking distance/angle moved in the random walk.
        self.pose = Pose2D()
        self.initial_pose = Pose2D()
        self.random_angle = 0.0
        self.beginning_of_random_walk = True

        # Initialise image processing
        self.bridge = cv_bridge.CvBridge()
        self.mask, self.masked_image, self.target = None, None, None

        # Initialise ranges
        self.ranges = None
        self.closest_obstacle = None
        self.right_obst, self.front_obst, self.left_obst = None, None, None

        # Bounding box/depth related variables
        self.goal_x =0
        self.goal_y =0
        self.objects = []
        self.facing_object = False

        self.current_tgt_class = None
        self.is_target = False
        # Keep track of targets that have been reached.
        self.complete = []

        # Start main control function
        self.main()

    # Publish a stop message
    def stop(self):
        self.pub.publish(Twist())


    def main(self):
        # Main control here. Control logic extracted from callbacks.

        # Set up move base action client
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        # Define h,w of image input
        w = 1920
        h = 1080

        # Stop if all 4 targets have been found.
        while not rospy.is_shutdown() and not len(self.complete) > 3:

            # Target reached
            # We need to set distance closer than obstacle avoidance
            # so that it takes precedence i.e. it only gets this close
            # if its a target, so we know its a target and not a wall
        
            # Alternatively use ymax from the image, but doesn't work
            # for mailbox or number 5
            if self.front_obst < 0.25 and self.is_target:
                # Stop
                self.stop()

                # Add target to list of those already found
                self.complete.append(self.current_tgt_class)
                self.is_target = False

                # Log info
                rospy.loginfo(self.current_tgt_class + " target reached.")
                rospy.loginfo("Stopping.")
                rospy.loginfo(self.complete) # Print obstacles found so far

            # Beacon towards target
            elif self.is_target:
                self.beacon_to_target()

            # If theres an obstacle closer than threshold do obstacle avoidance
            elif self.closest_obstacle <= 0.5:
                self.obstacle_avoidance(0.5)    

            # Stop otherwise, no random walk atm.
            else:
                self.stop()
            
            
            # Sleep
            self.rate.sleep()

        # We did it! All targets reached and acknowledged :)
        rospy.loginfo("All targets reached, shutting down.")



    def obstacle_avoidance(self, threshold):
        # This is just copied from elsewhere
        # Needs refactoring though
        twist_msg = Twist()
        if self.left_obst <= threshold and self.right_obst <= threshold:
            rospy.loginfo("Obstacle left and right; Turning 90 degrees")
            self.state = 'Obstacle'
            twist_msg.angular.z = math.pi/3
            self.pub.publish(twist_msg)
            self.beginning_of_random_walk = True
            rospy.sleep(1.5)
            # 90 degrees
        elif self.left_obst <= threshold:
            rospy.loginfo("Obstacle left; Turning clockwise")
            self.state = 'Obstacle'
            twist_msg.angular.z = -self.angular_speed
            self.pub.publish(twist_msg)
            self.beginning_of_random_walk = True
        elif self.right_obst <= threshold:
            rospy.loginfo("Obstacle right; Turning anticlockwise")
            self.state = 'Obstacle'
            twist_msg.angular.z = self.angular_speed
            self.pub.publish(twist_msg)
            self.beginning_of_random_walk = True
        elif self.front_obst <= threshold:
            rospy.loginfo("Obstacle ahead; Turning anticlockwise")
            self.state = 'Obstacle'
            twist_msg.angular.z = self.angular_speed
            self.pub.publish(twist_msg)
            self.beginning_of_random_walk = True

    # Beacon towards target object based on goal_x value.
    def beacon_to_target(self):
        
        # Get x centroid of bounding box of target
        box_centroid = self.goal_x

        # And width of the camera image (provided by darknet_ros)
        # it seems to be 1920x1080, so im hard coding it for now.
        w = 1920

        # Calculate the error for prop. control
        err = box_centroid - w/2

        # Construct twist message and publish it
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_speed
        twist_msg.angular.z = -float(err) / 400

        rospy.loginfo("Beaconing towards " + self.current_tgt_class)
        self.pub.publish(twist_msg)


    ### NOT IN USE ATM
    def random_walk(self):
        if self.state == 'Random Walk' and not rospy.is_shutdown():

            # If it's the beginning of the random walk, initialise the starting pose
            if self.beginning_of_random_walk:
                rospy.loginfo("Begin Random Walk")
                self.initial_pose = Pose2D(self.pose.x, self.pose.y, self.pose.theta)
                self.random_angle = random.uniform(-math.pi, math.pi)
                while abs(self.random_angle) < 1:
                    self.random_angle = random.uniform(-math.pi, math.pi)
                self.beginning_of_random_walk = False

            # If there is an obstacle stop and restart the random walk
            # If closest_obstacle has not been initiated yet, wait until it has
            if self.closest_obstacle is not None and (self.state == "Obstacle" or self.closest_obstacle <= 0.25):
                self.state = "Obstacle"
                self.stop()
                self.beginning_of_random_walk = True
                return

            twist_msg = Twist()
            n_of_meters = 3

            if calculate_linear_distance(self.initial_pose, self.pose) < n_of_meters:
                rospy.loginfo("Walked %.2f m out of %i m" % (
                    calculate_linear_distance(self.initial_pose, self.pose), n_of_meters))
                twist_msg.linear.x = min(self.linear_speed * self.closest_obstacle, self.linear_speed)
            elif calculate_angular_distance(self.initial_pose, self.pose) < self.random_angle:
                rospy.loginfo("Turned %.3f radians out of %.2f" % (
                    calculate_angular_distance(self.initial_pose, self.pose), self.random_angle))
                twist_msg.angular.z = self.angular_speed
            else:
                # If random walk is completed restart
                self.beginning_of_random_walk = True

            self.pub.publish(twist_msg)

        else:
            self.stop()
            self.beginning_of_random_walk = True


if __name__ == '__main__':
    try:
        rospy.init_node('follower')
        follower = Follower()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print("An exception was caught.")
        raise e


'''
def cloud_callback(self, data):
    width = data.width
    height = data.height
    point_step = data.point_step
    row_step = data.row_step
    array_pos = self.goal_y*row_step + self.goal_x*point_step
    (X, Y, Z) = struct.unpack_from('fff', data.data, offset=array_pos)
    # rospy.loginfo("x: {}, y: {}, z: {}".format(X,Y,Z))
'''
