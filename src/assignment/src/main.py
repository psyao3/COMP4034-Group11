#!/usr/bin/env python

import rospy
import cv2, cv_bridge
import numpy as np
import math
import random
import tf
import struct
import ctypes
import actionlib
import grid_methods as grid
from calculator import calculate_linear_distance, calculate_angular_distance, average_distance
from image_processing import generate_mask, convert_to_hsv, show_image, find_closest_centroid
from sensor_msgs.msg import Image, PointCloud2, LaserScan
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *


class ObjectFinder:

    def __init__(self):

        # Initialising rospy behaviour
        rospy.on_shutdown(self.stop)
        self.rate = rospy.Rate(10)  # Hz

        # Speeds to use
        self.linear_speed = 0.4
        self.angular_speed = 0.4

        # Publishers and Subscribers
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.image_sub = rospy.Subscriber('camera/depth/points', PointCloud2, self.depth_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)

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

        # Obstacles and objects
        self.obstacles = []
        self.objects = []

        # Count number of grid squares visited.
        self.visit_count = 0


        # Initialize occupancy grid; -1 means unknown/unvisited
        # Length is size_x * size_y (variable imported from grid_methods)
        # So for this map, 20 * 20, 400 grid squares total.
        self.occ_grid = np.full(np.product(grid.size), -1)


        while not rospy.is_shutdown():

            if len(self.objects) != 0:
                current_target = self.objects.pop(0)
                while not self.move_to_target(current_target):
                    rospy.loginfo("Trying again.")
            else:
                pass
                # Search strategy

    def stop(self):
        self.pub.publish(Twist())

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
        # Get (x, y, theta) specification from odometry topic
        quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)

        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

        # Get grid position
        g_xy = grid.to_grid([self.pose.x, self.pose.y], grid.origin, grid.size, grid.resolution)


        # Set current grid position on occ_grid to visited
        index = grid.to_index(g_xy[0], g_xy[1], grid.size[0])


        # 0 indicates a grid square has been visited.
        if self.occ_grid[index] == -1:
            self.occ_grid[index] = 0
            self.visit_count += 1
            total_grid_cells = grid.size[0]*grid.size[1]

            rospy.loginfo("Visited a new grid square [%i, %i]." % (g_xy[0], g_xy[1]))
            rospy.loginfo("Visited %i squares - %.1f %% of total."
                          % (self.visit_count, float(self.visit_count)/float(total_grid_cells)*100))
            # Print 1d grid
            print("\n\n")
            print(np.fliplr(self.occ_grid.reshape(grid.size[0], grid.size[1])))
            print("\n\n")


    def scan_callback(self, msg):
        self.ranges = np.array(msg.ranges)

        obst_dists = []
        for i in range(-45, 45, 8):
            obst_dists.append(average_distance(self.ranges, i, 8))

        self.right_obst = min(obst_dists[0:5])
        self.front_obst = min(obst_dists[5:7])
        self.left_obst = min(obst_dists[7:])

        self.closest_obstacle = min(self.front_obst, self.left_obst, self.right_obst)


    def image_callback(self, msg):

        hsv = convert_to_hsv(msg, self.bridge)

        mask = generate_mask(hsv)

        target = cv2.bitwise_and(hsv, hsv, mask=mask)
        show_image(mask=mask, masked_image=target)

        # Label objects in image (0 is background), matrix of labels and stats
        num_targets, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)

        threshold = 0.5

        # If theres an obstacle closer than threshold do obstacle avoidance
        if self.closest_obstacle <= threshold:
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

        # If no obstacle but at least one target beacon towards
        elif num_targets > 1:
            # If there is more than one label, find the closest target and move towards it
            rospy.loginfo("Target(s) in sight.")
            self.state = 'Target'
            self.beacon_towards_closest_target(centroids, hsv, labels)

        # Else random walk
        else:
            # If there is only one label, then there is only background; Initiate Random Walk
            rospy.loginfo("No targets  - Random Walk.")
            self.state = 'Random Walk'
            self.random_walk()

    def beacon_towards_closest_target(self, centroids, hsv, targets):
        if self.state == 'Target' and not rospy.is_shutdown():
            # If there are multiple green objects/targets, only follow one.
            # Remove the one further away from the image.

            # Need Image width to get centroid of turtlebot's view.       
            _, w, _ = hsv.shape
            twist_msg = Twist()

            # Iterate over targets, keep largest one.
            largest_target = find_closest_centroid(centroids, w)

            # Keep only largest_target, remove others from image.
            #mask = np.where(targets == largest_target, np.uint8(255), np.uint8(0))
            obj_centroid = centroids[largest_target, 0]

            # Implement a proportional controller to beacon towards it
            err = obj_centroid - w / 2
            twist_msg.linear.x = self.linear_speed
            twist_msg.angular.z = -float(err) / 400

            self.pub.publish(twist_msg)
        else:
            self.stop()

    def depth_callback(self, data):

        if self.state == 'Target' and not rospy.is_shutdown():
        
            # ignore non-numbers
            generator = point_cloud2.read_points(data, skip_nans=True)
            int_data = list(generator)

            rospy.loginfo("point cloud")

            current_max = []
            tracker = 0
            '''
            ok, so my idea here is that the point cloud is able to give relative xyz values of the surrounding points from the depth scan
            this temporary function just finds the greenest object in the view and updates grid with a number 2 
            just as a temporary way to identify the object locations are correctly found

            I will try and replace this 'find the greenest value' with something which identifies whether a point is a part of an object 
            then update the grid or obstacles list with the position of the object maybe using the to_world method to add to the move_base

            i've found this really hard to test at the moment though because its almost impossible to find a green treshold which doesnt get confused with the walls and bricks in certain areas
            when specifically trying to find the green box
            '''
            for p in int_data:

                rgb = p[3]  
                   
                
                # cast float32 to int 
                s = struct.pack('>f' ,rgb)
                i = struct.unpack('>l',s)[0]

                # extract the individaul rgb values from the float32
                pack = ctypes.c_uint32(i).value
                r = (pack & 0x00FF0000)>> 16
                g = (pack & 0x0000FF00)>> 8
                b = (pack & 0x000000FF)

                # very temporary find greatest geeen value in the view
                if g > tracker:
                    current_max = p


                    
            #then convert the relative position to the grid based on robots odom values
            g_xy = grid.to_grid([self.pose.x + current_max[0], self.pose.y + current_max[1]], grid.origin, grid.size, grid.resolution)
            index = grid.to_index(g_xy[0], g_xy[1], grid.size[0])
            self.occ_grid[index] = 2

            rospy.loginfo("updated grid")
       


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
        rospy.init_node('object_finder')
        object_finder = ObjectFinder()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print("An exception was caught.")
        raise e
