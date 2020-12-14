#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist, Pose2D, Point, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
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
from darknet_ros_msgs.msg import BoundingBoxes, ObjectCount
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from calculator import calculate_linear_distance, calculate_angular_distance, average_distance
from image_processing import generate_mask, convert_to_hsv, show_image, find_closest_centroid
from actionlib_msgs.msg import *
import matplotlib.pyplot as plt

from callbacks import *
from frontier_methods import *

class Follower:

    def __init__(self):

        # Initialising rospy behaviour
        rospy.on_shutdown(self.stop)
        self.rate = rospy.Rate(10)  # Hz

        # Speeds to use
        self.linear_speed = 0.4
        self.angular_speed = math.pi/10

        # Occupancy grid initialisation
        self.occ_grid = None
        self.occ_grid_info = None

        # Pose for odom
        self.pose = Pose2D()

        # Initialise image processing
        self.bridge = cv_bridge.CvBridge()
        self.mask, self.masked_image, self.target = None, None, None

        # Initialise ranges
        self.ranges = None
        self.closest_obstacle = None
        self.right_obst, self.front_obst, self.left_obst = None, None, None
        self.any_obst = None

        # Bounding box related variables
        self.goal_x = 0
        self.goal_y = 0
    
        self.current_tgt_class = None
        self.is_target = False

        # Keep track of targets that have been reached.
        self.complete = []

        # For number 5 special case
        self.box_height = None 
        self.box_width = None


        # Variables for evaluation
        self.start_time = rospy.get_rostime()
        self.time_limit = 300 # 5 minutes

        # Initialise move_base goal status
        self.goal_status = -1

        # Publishers and Subscribers
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.occ_grid_publisher = rospy.Publisher('/occ_grid', Float32MultiArray, queue_size=10)

        # Darknet
        # This only updates when bounding boxes are found, so need another subscriber.
        self.box_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, box_callback, (self))
        # Updates constantly with number of bounding boxes
        self.count_obj_sub = rospy.Subscriber('/darknet_ros/found_object', ObjectCount, count_boxes_callback, (self) )
        self.box_count = 0 # Initialise


        self.odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback, (self))
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, scan_callback, (self))

        # Subscriber to get the move_base result
        rospy.Subscriber('move_base/result', MoveBaseActionResult, status_callback, (self))
        # Subsribers for occupancy grids - global costmap and updates
        # self.occ_grid_ingo needs to be initialised before it's updated
        rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, grid_callback, (self))
        while self.occ_grid_info is None:
            rospy.loginfo("Waiting for occupancy to be initialised.")
            rospy.sleep(0.5)
        rospy.Subscriber('/move_base/global_costmap/costmap_updates', OccupancyGridUpdate, update_callback, (self))

        # Publisher to cancel current move base command to use in box callback.
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)

        # Start main control function
        self.main()


    # Publish a stop message
    def stop(self):
        self.pub.publish(Twist())


    def main(self):

        # Main control here. Control logic extracted from callbacks.
    
        # Define h,w of image input
        w = 1920
        h = 1080

        # Stop if all 4 targets have been found. TODO: add condition to stop after 5 mins (demo timer)
        while not rospy.is_shutdown() and not len(self.complete) > 3:

            # Check if target reached
            # IF the bottom of the bounding box (goal_y) is close to the bottom
            # of the camera view in pixels (>1080 * 0.8), then object reached.

            # FOR NUMBER 5 ONLY:
            # If the box width is 1.5 * box height, goal reached. May need to tweak
            # this, but chose this based on observed box behaviour when closing in on
            # the number 5 object.
            if self.is_target and (self.goal_y >= h * 0.8 or 
             ((self.current_tgt_class == "number 5") and 
             (self.box_width > 1.5 * self.box_height))):
                # Stop
                self.stop()

                # Add target to list of those already found
                self.complete.append(self.current_tgt_class)
                self.is_target = False

                # Log info
                rospy.loginfo(self.current_tgt_class + " target reached.")
                rospy.loginfo("Stopping.")
                rospy.loginfo(self.complete) # Print obstacles found so far


            # If theres an obstacle closer than threshold (0.5) do obstacle avoidance
            elif self.closest_obstacle <= 0.3:
                self.obstacle_avoidance(0.3) 

              # Beacon towards target
            elif self.is_target:
                self.beacon_to_target()
                self.rate.sleep()   

            else:

                ### I think a 360 degree spin should go here, before frontier exploration.

                '''
                Frontier Exploration logic
                '''
                rospy.loginfo("Start exploration.")

                while self.pose is None:
                    rospy.loginfo("Waiting for pose to be initialised.")

                # Frontiers include the ones outside the arena
                frontiers = get_frontiers(self)
                # Removing frontiers outside the arena - need to add two sides
                # point[0] is x and point[1] is y
                valid_frontiers = [point for point in frontiers if point[0] > 165 and point[0] < 330 and point[1] > 110 and point[1] < 295]

                while len(frontiers) != 0 and not self.is_target:
                    # Visualise frontiers
                    grid_target = np.ones((384, 384))
                    for x, y in valid_frontiers:
                        grid_target[y][x] = 0
                    grid_target = [[grid_target[y][x] for y in range(len(grid_target))] for x in
                                   range(len(grid_target[0]) - 1, -1, -1)]
                    grid_target = np.flip(grid_target, 1)
                    plt.imshow(grid_target, cmap='hot', interpolation='nearest')
                    plt.show()

                    # Get closest frontiers (point distance - not path)
                    current_x_grid, current_y_grid = world_to_occ_grid(self, self.pose.x, self.pose.y)
                    target = get_closest_frontier(self, current_x_grid, current_y_grid, valid_frontiers)
                    rospy.loginfo(target)
                    # Move to target
                    target_x, target_y = occ_grid_to_world(self, target[0], target[1])
                    target_pose = Pose2D()
                    target_pose.x = target_x
                    target_pose.y = target_y
                    target_point = Point(target_x, target_y, 0)
                    rospy.loginfo("Target: {}".format(target_pose))
                    move_to_target_alt(self, target_point)

                    # Frontiers include the ones outside the arena
                    frontiers = get_frontiers(self)
                    # Removing frontiers outside the arena - need to add two sides
                    valid_frontiers = [point for point in frontiers if point[0] > 165 and point[1] < 295]
            
            
            # Sleep
            self.rate.sleep()

        # We did it! All targets reached and acknowledged :)
        rospy.loginfo("All targets reached, shutting down.")
        self.evaluate()


    def evaluate(self):

        run_time = rospy.get_rostime() - self.start_time

        rospy.loginfo("Statistics of this run:")
        rospy.loginfo("Runtime in seconds: %f" % run_time)
        rospy.loginfo("Targets found: %d" % len(self.complete))
        rospy.loginfo(self.complete) # The targets

        # Extra stats can be added w.r.t to frontiers
        '''
            number of frontiers explored?
            number of move_base commands sent?
            % of map explored?
        '''



    def obstacle_avoidance(self, threshold):
        # TODO: This is just copied from elsewhere
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



if __name__ == '__main__':
    try:
        rospy.init_node('follower')
        follower = Follower()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print("An exception was caught.")
        raise e

