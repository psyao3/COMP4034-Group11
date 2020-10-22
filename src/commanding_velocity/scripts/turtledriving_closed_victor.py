#!/usr/bin/env python2

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as ani
import rospy
import tf

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from math import pi, sqrt


class Closed_Loop:

    def __init__(self):

        self.pose = Pose2D()
        self.pose.x = 0
        self.pose.y = 0
        self.pose.theta = 0

        self.distance = 1
        self.desired_angle = pi/2

        rospy.init_node('turtlebotclosedloop', anonymous = True)

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.rate = rospy.Rate(100)
    
    def odom_callback(self, msg):
        # Get (x, y, theta) specification from odometry topic
        quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)

        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

    def move(self, speed):

        msgForward = Twist()
        msgForward.linear.x = speed
        msgForward.angular.z = 0

        initial_x = self.pose.x
        initial_y = self.pose.y

        while sqrt((self.pose.x - initial_x)**2 + (self.pose.y - initial_y)**2) < self.distance and not rospy.is_shutdown():
            rospy.loginfo("Moving forward")
            self.pub.publish(msgForward)
        
        self.stop_moving()

    def rotate(self, speed):

        msgTurn = Twist()
        msgTurn.linear.x = 0
        msgTurn.angular.z = speed

        initial_theta = self.pose.theta

        while self.cal_angle(initial_theta) < self.desired_angle and not rospy.is_shutdown():
            rospy.loginfo("Rotating 90 degrees")
            self.pub.publish(msgTurn)
        

        self.stop_moving()

    def cal_angle(self, theta):

        if self.pose.theta < 0:
            initial_theta = 2*pi + self.pose.theta
            theta = 2*pi + theta
            angle = abs(initial_theta - theta)
        else:
            angle = abs(self.pose.theta - theta)
        
        return angle
            

    def stop_moving(self):

        msgStop = Twist()
        self.pub.publish(msgStop)


    def draw_square(self):

        self.move(0.2)
        self.rotate(pi/8)
        self.move(0.2)
        self.rotate(pi/8)
        self.move(0.2)
        self.rotate(pi/8)
        self.move(0.2)
        self.rotate(pi/8)
        self.stop_moving()



if __name__ == "__main__":
    try:
        cl = Closed_Loop()
        cl.draw_square()
    except rospy.ROSInterruptException: pass    