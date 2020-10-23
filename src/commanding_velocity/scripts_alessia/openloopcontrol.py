#!/usr/bin/env python

import rospy
import tf
import math
from geometry_msgs.msg import Twist, Pose, Pose2D
from nav_msgs.msg import Odometry

import publisher as pub
import subscriber as sub
import visualiser 

class RobotHandler():

    def __init__(self, robot_name = 'robot'):

        # Initialise rospy and the node
        rospy.on_shutdown(self._stop)
        self._initialise_node(node_name=robot_name)

        # Create publisher and subscriber
        self._pub = pub._create_publisher(name='/cmd_vel', type=Twist)
        self._sub = sub._create_subscriber(name='/odom', type=Odometry, callback=self._odom_callback)

        # Set the frequency
        self._rate = rospy.Rate(100)

        # Set initial pose
        self.pose = Pose2D()
        self.pose.x, self.pose.y, self.pose.theta = 0.0, 0.0, 0.0

        # Initialise trajectory logging
        self.trajectory = []
        self.logging_counter = 0

    def _initialise_node(self, node_name, anonymous = True):
        rospy.init_node(node_name, anonymous=bool)

    def _move_forward(self, distance, linear_speed = 0.5):

        print "Moving forward!"
        self._move(movement_type="move_forward", amount=distance, speed=linear_speed)

    def _rotate(self, angle_in_rad, angular_speed):

        print "Rotating!"
        self._move(movement_type="rotate", amount=angle_in_rad, speed=angular_speed)

    def _move(self, movement_type, amount, speed):

        # Generate Twist message and set the linear (m/s) or angular (rad/s) speed 
        twist_msg = Twist()

        if movement_type == "move_forward":
            twist_msg.linear.x = speed
        elif movement_type == "rotate":
            twist_msg.angular.z = speed
        else:
            print("Invalid value.")
            return 1

        # Calculate duration
        initial_time = rospy.Time.now().to_sec()
        time_necessary_in_s = amount / speed

        # Publish the message
        while rospy.Time.now().to_sec() - initial_time < rospy.Duration(time_necessary_in_s).to_sec() and not rospy.is_shutdown(): 
            self._pub.publish(twist_msg)
        
        self._stop()

    def _stop(self):
        twist_msg = Twist()
        self._pub.publish(twist_msg)

    def _odom_callback(self, msg):
        # Get (x, y, theta) specification from odometry topic
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)

        # self.pose.theta = yaw
        self.pose.theta = yaw if yaw >= 0 else yaw+6.283
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
