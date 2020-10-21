#!/usr/bin/env python

import rospy
import tf
import math
from geometry_msgs.msg import Twist, Pose, Pose2D
from nav_msgs.msg import Odometry

from pylab import *
import matplotlib.pyplot as plt

class RobotHandler():

    def __init__(self, robot_name = 'robot'):
        rospy.on_shutdown(self._stop)
        self._initialise_node(node_name=robot_name)
        self._pub = self._create_publisher(name='/cmd_vel', type=Twist)
        self._sub = self._create_subscriber(name='/odom', type=Odometry, callback=self._odom_callback)
        self._pub_odo = self._create_publisher(name='/odom', type=Odometry)
        self._odo_broadcaster = tf.TransformBroadcaster()
        self._rate = rospy.Rate(20)

        self.des_x, self.des_y, self.des_theta = 5,5,0

        self.pose = Pose2D()
        self.pose.x = 0.0
        self.pose.y = 0.0
        self.pose.theta = 0.0

        self.trajectory = []
        self.logging_counter = 0


    def _create_publisher(self, name, type, size_of_queue = 10):
        return rospy.Publisher(name, type, queue_size=size_of_queue)

    def _create_subscriber(self, name, type, callback):
        return rospy.Subscriber(name, type, callback)

    def _initialise_node(self, node_name, anonymous = True):
        rospy.init_node(node_name, anonymous=bool)

    def _move(self, distance, speed = 0.5):
        print "Moving forward!"
        twist_msg = Twist()

        initial_time = rospy.Time.now().to_sec()
        distance_driven = 0

        while distance_driven < distance and not rospy.is_shutdown():  
            # m/s
            twist_msg.linear.x = speed
            twist_msg.linear.y = 0.0
            twist_msg.linear.z = 0.0

            # rad/s
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0
            twist_msg.angular.z = 0.0

            self._pub.publish(twist_msg)
            #self._rate.sleep()
            current_time = rospy.Time.now().to_sec()
            distance_driven = speed*(current_time-initial_time)
        
        self._stop()


    def _rotate(self, duration, angular_speed):
        print "Rotating!"
        
        # 90 degrees = 1.5708 radians
        twist_msg = Twist()

        initial_time = rospy.Time.now().to_sec()
        orient = 0

        self.des_theta = self.des_theta + (math.pi/2)
        previous_theta = 0

        #while orient < math.pi/2 and not rospy.is_shutdown():
        while self.pose.theta < self.des_theta and not rospy.is_shutdown():

            if previous_theta > 6 and self.pose.theta < 1:
                break
            previous_theta = self.pose.theta

            speed = angular_speed

            # m/s
            twist_msg.linear.x = 0.0
            twist_msg.linear.y = 0.0
            twist_msg.linear.z = 0.0

            # rad/s
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0
            twist_msg.angular.z = speed

            self._pub.publish(twist_msg)
            #self._rate.sleep()
            current_time = rospy.Time.now().to_sec()
            #orient = speed*(current_time-initial_time)
        
        self._stop()

    def _stop(self):
        twist_msg = Twist()
        self._pub.publish(twist_msg)

    def move_in_a_square(self, edge_size_in_m):
        rospy.sleep(1)
        for x in [1, 2, 3, 4]:
            self._move(distance=edge_size_in_m, speed=0.2)
            self._rotate(duration=1, angular_speed=math.pi/2/2/2)
            self._stop()
            print(self.pose.theta, self.des_theta)
        self._stop()

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
        if self.logging_counter == 100:
            self.logging_counter = 0
            self.trajectory.append([self.pose.x, self.pose.y]) 


    def plot_trajectory(self):
        coordinates = np.array(self.trajectory)
        x_coordinates = coordinates[:,0]
        y_coordinates = coordinates[:,1]
        plt.plot(x_coordinates,y_coordinates, color='lightblue', linewidth=3)
        grid()
        plt.show()


if __name__ == '__main__':
    try:
        robot = RobotHandler()
        robot.move_in_a_square(edge_size_in_m=1)
        robot.plot_trajectory()
    except rospy.ROSInterruptException:
        pass
