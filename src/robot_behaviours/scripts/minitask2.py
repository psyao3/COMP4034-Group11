#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist, Pose, Pose2D
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf
import random

import calculator
import numpy as np


class RobotHandler():

    def __init__(self, robot_name='robot'):

        # Initialise rospy and the node
        rospy.on_shutdown(self._stop)
        rospy.init_node(robot_name, anonymous=bool)

        # Create publisher and subscribers
        self._pub = rospy.Publisher('/cmd_vel', Twist, queue_size=50)
        self._sub = rospy.Subscriber('/scan', LaserScan, self._callback)
        self._sub_odom = rospy.Subscriber('/odom', Odometry, self._odom_callback)

        # Set the frequency
        self._rate = rospy.Rate(10)

        # Set initial pose
        self.pose = Pose2D()
        self.pose.x, self.pose.y, self.pose.theta = 0.0, 0.0, 0.0

        # Set initial ranges
        self.ranges = None

        self.linear_speed = 0.4
        self.rotational_speed = 0.4

        self.should_move_forward = True
        self.rotation_sense = 'clockwise'

        self.state = "Start"

        while not rospy.is_shutdown():

            if self.state == 'Random_walk' and self.should_move_forward:
                print("Random Walk")
                self.should_move_forward = not self.should_move_forward
                self._move_forward(distance=1, linear_speed=self.linear_speed)
            elif self.state == 'Random_walk' and not self.should_move_forward:
                angle = random.uniform(-math.pi, math.pi)
                print(angle)
                self.should_move_forward = not self.should_move_forward
                self._rotate(angle_in_rad=angle, angular_speed=self.rotational_speed)
            elif self.state == 'Obstacle':
                twist_msg = Twist()
                if self.rotation_sense == 'clockwise':
                    twist_msg.angular.z = self.rotational_speed
                elif self.rotation_sense == 'anticlockwise':
                    twist_msg.angular.z = -self.rotational_speed
                self._pub.publish(twist_msg)
                self.should_move_forward = True

    def _avoid_obstacle(self):

        window_size = 4
        threshold = 0.25
        front = [0, 0+window_size/2, 360-window_size/2, 360]
        front_right = [45-window_size/2, 45+window_size/2]
        front_left = [315-window_size/2, 315+window_size/2]

        twist_msg = Twist()

        while self.state == 'Obstacle':
            if np.mean(self.ranges[front[0]:front[1]] + self.ranges[front[2]:front[3]]) <= threshold or \
                    np.mean(self.ranges[front_right[0]:front_right[1]]) < threshold or \
                    np.mean(self.ranges[front_left[0]:front_left[1]]) < threshold:
                twist_msg.angular.z = self.rotational_speed
                self._pub.publish(twist_msg)
            else:
                self._stop()
                self.state = 'Random Walk'

    def _calculate_distance(self, type, pose1, pose2):
        if type == 'linear':
            distance = calculator.calculate_linear_distance(pose1, pose2)
        elif type == 'angular':
            distance = calculator.calculate_angular_distance(pose1, pose2)
        return distance

    def _stop(self):
        twist_msg = Twist()
        self._pub.publish(twist_msg)

    def _move_forward(self, distance, linear_speed=0.5):
        print("Moving forward!")
        self._move(movement_type="linear", target=distance, speed=linear_speed)

    def _rotate(self, angle_in_rad, angular_speed):
        print("Rotating!")
        self._move(movement_type="angular", target=angle_in_rad, speed=angular_speed)

    def _move(self, movement_type, target, speed):

        # Generate Twist message and set the linear (m/s) or angular (rad/s) speed
        twist_msg = Twist()
        if movement_type == 'linear':
            twist_msg.linear.x = speed
        elif movement_type == 'angular':
            if target < 0:
                target = target*-1
                speed = speed*-1
            twist_msg.angular.z = speed

        # Calculate stopping criteria
        initial_pose = Pose2D()
        initial_pose.x, initial_pose.y, initial_pose.theta = self.pose.x, self.pose.y, self.pose.theta

        # Publish the message
        while self._calculate_distance(movement_type, self.pose, initial_pose) < target and not rospy.is_shutdown():
            self._pub.publish(twist_msg)
            if self.state == "Obstacle":
                self._stop()
                break
        if movement_type == 'angular':
            print(target)
        self._stop()

    def _callback(self, msg):
        self.ranges = msg.ranges

        window_size = 4
        threshold = 0.25
        front = [0, 0+window_size/2, 360-window_size/2, 360]
        front_right = [45-window_size/2, 45+window_size/2]
        front_left = [315-window_size/2, 315+window_size/2]

        if np.mean(self.ranges[front[0]:front[1]] + self.ranges[front[2]:front[3]]) <= threshold or \
                np.mean(self.ranges[front_right[0]:front_right[1]]) < threshold or \
                np.mean(self.ranges[front_left[0]:front_left[1]]) < threshold:
            self.state = "Obstacle"
            if np.mean(self.ranges[front_right[0]:front_right[1]]) < threshold:
                self.rotation_sense = 'anticlockwise'
            elif np.mean(self.ranges[front_left[0]:front_left[1]]) < threshold:
                self.rotation_sense = 'clockwise'
        else:
            self.state = "Random_walk"

    def _odom_callback(self, msg):
        # Get (x, y, theta) specification from odometry topic
        quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, \
                       msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)

        # self.pose.theta = yaw
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y


if __name__ == '__main__':
    try:
        robot = RobotHandler()
    except rospy.ROSInterruptException as e:
        print("An exception was caught.")
        raise e
