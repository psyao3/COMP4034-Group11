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
        rospy.Subscriber('/scan', LaserScan, self._callback)
        rospy.Subscriber('/odom', Odometry, self._odom_callback)

        # Set the frequency
        self._rate = rospy.Rate(10)

        # Set initial pose
        self.pose = Pose2D()
        self.pose.x, self.pose.y, self.pose.theta = 0.0, 0.0, 0.0

        # Set initial ranges
        self.ranges = None

        self.linear_speed = 0.4
        self.rotational_speed = 0.4

        self.threshold = 0.5

        self.should_move_forward = True
        self.rotation_sense = 'clockwise'
        self.previous_rotation_sense = self.rotation_sense

        self.state = "Start"

        while not rospy.is_shutdown():

            if self.state == 'Random_walk' and self.should_move_forward:
                rospy.loginfo("Random Walk - Walking")
                self.should_move_forward = not self.should_move_forward
                self._move_forward(distance=1, linear_speed=self.linear_speed)

            elif self.state == 'Random_walk' and not self.should_move_forward:
                rospy.loginfo("Random Walk - Rotating")
                angle = random.uniform(-math.pi, math.pi)
                rospy.loginfo(angle)
                self.should_move_forward = not self.should_move_forward
                self._rotate(angle_in_rad=angle, angular_speed=self.rotational_speed)

            elif self.state == 'Obstacle':
                twist_msg = Twist()
                if self.rotation_sense == 'clockwise':
                    twist_msg.angular.z = -self.rotational_speed
                elif self.rotation_sense == 'anticlockwise':
                    twist_msg.angular.z = self.rotational_speed
                self._pub.publish(twist_msg)
                self.should_move_forward = True

            elif self.state == 'Wall Following':
                twist_msg = Twist()
                twist_msg.linear.x = self.linear_speed
                self._pub.publish(twist_msg)

        rospy.sleep(1)

    def average_distance(self, ranges, index, degree):

        angles = range(index - degree, index + degree - 1)

        for i, ang in enumerate(angles):
            if ang > 360:
                angles[i] = ang - 360

        arr = np.array(ranges)
        avg = np.mean(arr[angles])

        return avg

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
        self._move(movement_type="linear", target=distance, speed=linear_speed)

    def _rotate(self, angle_in_rad, angular_speed):
        self._move(movement_type="angular", target=angle_in_rad, speed=angular_speed)

    def _move(self, movement_type, target, speed):

        # Generate Twist message and set the linear (m/s) or angular (rad/s) speed
        twist_msg = Twist()
        if movement_type == 'linear':
            twist_msg.linear.x = speed
        elif movement_type == 'angular':
            if target < 0:
                target = target * -1
                speed = speed * -1
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

        window_size = 5
        threshold = self.threshold

        front = []
        front_right = []
        front_left = []

        for i in range(-10, 10, window_size):
            front.append(self.average_distance(self.ranges, i, window_size))
        for i in range(-45, -10, window_size):
            front_right.append(self.average_distance(self.ranges, i, window_size))
        for i in range(10, 45, window_size):
            front_left.append(self.average_distance(self.ranges, i, window_size))

        obstacle_range = []
        for i in range(-45, 45, window_size):
            obstacle_range.append(self.average_distance(self.ranges, i, window_size))

        if self._check_if_wall():
            self.state = "Wall Following"
            rospy.loginfo("Wall Following")

        if any(distance <= threshold for distance in obstacle_range):

            if self._check_if_wall():
                self.state = "Wall Following"
                rospy.loginfo("Wall Following")
            else:
                self.state = "Obstacle"
                rospy.loginfo("Obstacle")
                if any(distance <= threshold for distance in front_right):
                    self.rotation_sense, self.previous_rotation_sense = 'anticlockwise', 'anticlockwise'
                elif any(distance <= threshold for distance in front_left):
                    self.rotation_sense, self.previous_rotation_sense = 'clockwise', 'clockwise'
                else:
                    self.rotation_sense = self.previous_rotation_sense
        else:
            self.state = "Random_walk"

    def _check_if_wall(self):

        obstacle_range = []
        window_size = 5
        for i in range(-45, 45, window_size):
            obstacle_range.append(self.average_distance(self.ranges, i, window_size))

        # Get the distance to the right (270) and 45 degrees above that (315)
        a = np.mean(self.ranges[315])
        b = np.mean(self.ranges[270])
        # If there is a wall to the right, the angle should be 90 degrees and the other two 45 degrees
        # the value of a can be calculated as b / sin(B) where B is the angle opposite to b (45 degrees)
        calc_a = b / np.sin(math.pi / 4)
        # Checks that a and the calculated a are similar and we are close to the wall
        # and that we are not too close to any obstacle
        if abs(calc_a - a) < 0.05 and b < self.threshold \
                and b >= 0.2 and not any(
            distance <= 0.15 for distance in obstacle_range):  # and that we are not too close to any obstacle
            is_wall = True
        else:
            is_wall = False

        return is_wall

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
