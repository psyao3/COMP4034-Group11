#!/usr/bin/env python

import rospy
import tf
import math
from geometry_msgs.msg import Twist, Pose2D
import visualiser 

class RobotHandler():

    def __init__(self, robot_name = 'robot'):

        # Initialise rospy and the node
        rospy.on_shutdown(self._stop)
        rospy.init_node(node_name=robot_name, anonymous=True)

        # Create publisher and subscriber
        self._pub = rospy.Publisher(name='/cmd_vel', type=Twist, queue_size=100)
        self._sub = rospy.Subscriber(name='/odom', type=Odometry, callback=self._odom_callback)

        # Set the frequency
        self._rate = rospy.Rate(100)

        # Set initial pose
        self.pose = Pose2D()
        self.pose.x, self.pose.y, self.pose.theta = 0.0, 0.0, 0.0

        # Initialise trajectory logging
        self.trajectory = []
        self.logging_counter = 0

    def _move(self, movement_type, target, speed):

        # Generate Twist message and set the linear (m/s) or angular (rad/s) speed 
        twist_msg = Twist()
        if movement_type == 'linear':
            twist_msg.linear.x = speed
        elif movement_type == 'angular':
            twist_msg.angular.z = speed

        # Calculate stopping criteria
        initial_pose = Pose2D()
        initial_pose.x, initial_pose.y, initial_pose.theta = self.pose.x, self.pose.y, self.pose.theta

        # Publish the message
        while self._calculate_distance(movement_type, self.pose, initial_pose) < target and not rospy.is_shutdown(): 
            self._pub.publish(twist_msg)
       
        self._stop()
        
    def _move_forward(self, distance, linear_speed = 0.5):
        print "Moving forward!"
        self._move(movement_type="linear", target=distance, speed=linear_speed)

    def _rotate(self, angle_in_rad, angular_speed):
        print "Rotating!"
        self._move(movement_type="angular", target=angle_in_rad, speed=angular_speed)

    def _stop(self):
        twist_msg = Twist()
        self._pub.publish(twist_msg)

    def _calculate_linear_distance(self, pose1, pose2):
        x1, y1 = pose1.x, pose1.y
        x2, y2 = pose2.x, pose2.y
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2) 

    def _calculate_angular_distance(self, pose1, pose2):
        theta1, theta2 = pose1.theta, pose2.theta
        angle = theta2 - theta1
        if angle > math.pi:
            angle -= 2*math.pi
        elif angle < -math.pi:
            angle += 2*math.pi
        return abs(angle)

    def _calculate_distance(self, type, pose1, pose2):
        if type == 'linear':
            distance = self._calculate_linear_distance(pose1, pose2)
        elif type == 'angular':
            distance  = self._calculate_angular_distance(pose1, pose2)
        return distance 

    def _odom_callback(self, msg):
        # Get (x, y, theta) specification from odometry topic
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)

        # self.pose.theta = yaw
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y
        
        # Save values for trajectory
        self.logging_counter += 1
        if self.logging_counter == 50:
            self.logging_counter = 0
            self.trajectory.append([self.pose.x, self.pose.y]) 

    def move_in_a_square(self, edge_size_in_m):
        rospy.sleep(1)
        for x in range(0,4):
            self._move_forward(distance=edge_size_in_m, linear_speed=0.2)
            self._rotate(angle_in_rad=math.pi/2, angular_speed=0.2)
            self._stop()
        self._stop()

if __name__ == '__main__':
    try:
        robot = RobotHandler()
        robot.move_in_a_square(edge_size_in_m=1)
        visualiser.plot_trajectory(trajectory=robot.trajectory)
    except rospy.ROSInterruptException as e:
        print("An exception was caught.")
        raise e
