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
from calculator import calculate_linear_distance, calculate_angular_distance, average_distance
from image_processing import generate_mask, convert_to_hsv, show_image, find_closest_centroid


class Follower:

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

    def stop(self):
        self.pub.publish(Twist())

    def odom_callback(self, msg):
        # Get (x, y, theta) specification from odometry topic
        quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)

        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

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