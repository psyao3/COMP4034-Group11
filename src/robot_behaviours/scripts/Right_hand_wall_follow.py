#!/usr/bin/env python

import math
import rospy
import tf
import visualiser 

from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class WallFollower():

    def __init__(self, robot_name = 'robot'):

        # Initialise rospy and the node
        rospy.on_shutdown(self.stop_moving)
        rospy.init_node('wall_follower', anonymous=True)

        # Create publisher and subscriber
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/scan', LaserScan, self.laserscan_callback )

        # Set the frequency
        rospy.Rate(100)

        # Set initial pose
        self.pose = Pose2D()
        self.pose.x, self.pose.y, self.pose.theta = 0.0, 0.0, 0.0

        self.ranges = None
        self.dist_to_wall = 0.25
        self.angle_min = -math.pi/2
        self.angle_max = math.pi/2
        self.range_size = None
        self.states = 0
        self.position_window = {
            'right': 0,
            'front_right': 0,
            'front': 0,
            'front_left': 0,
            'left': 0,
        }

    def odom_callback(self, msg):
            # Get (x, y, theta) specification from odometry topic
            quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)

            # self.pose.theta = yaw
            self.pose.theta = yaw
            self.pose.x = msg.pose.pose.position.x
            self.pose.y = msg.pose.pose.position.y
        
    def laserscan_callback(self, msg):
        self.ranges = msg.ranges
        self.range_size = len(msg.ranges)
        print(self.range_size)
        self.ranges.angle_min = self.angle_min
        self.ranges.angle_max = self.angle_max
        self.ranges.range_min = self.dist_to_wall
        # Ranges array of size 720
        # 0-719 (5 positional ranges) equals 144 for each 'window' of ranges
        self.position_window = {
            'right': min(min(self.range_size[0:143]),25),
            'front_right': min(min(self.range_size[144:287]),25),
            'front': min(min(self.range_size[288:431]),25),
            'front_left': min(min(self.range_size[432:575]),25),
            'left': min(min(self.range_size[576:719]),25),
        }

        self.position_decider()

    def position_decider(self):

        if self.position_window['front'] > self.dist_to_wall and self.position_window['front_right'] > self.dist_to_wall \
            and self.position_window['front_right'] > self.dist_to_wall:
            # random_wander maybe?
            self.states = 0
        elif self.position_window['front'] < self.dist_to_wall and self.position_window['front_right'] > self.dist_to_wall \
            and self.position_window['front_left'] > self.dist_to_wall:
            self.states = 1
        elif self.position_window['front'] > self.dist_to_wall and self.position_window['front_right'] < self.dist_to_wall \
            and self.position_window['front_left'] > self.dist_to_wall:
            self.states = 2
        elif self.position_window['front'] > self.dist_to_wall and self.position_window['front_right'] > self.dist_to_wall \
            and self.position_window['front_left'] < self.dist_to_wall:
            # random_wander again
            self.states = 0
        elif self.position_window['front'] < self.dist_to_wall and self.position_window['front_right'] < self.dist_to_wall \
            and self.position_window['front_left'] > self.dist_to_wall:
            self.states = 1
        elif self.position_window['front'] < self.dist_to_wall and self.position_window['front_right'] > self.dist_to_wall \
            and self.position_window['front_left'] < self.dist_to_wall:
            self.states = 1
        elif self.position_window['front'] < self.dist_to_wall and self.position_window['front_right'] < self.dist_to_wall \
            and self.position_window['front_left'] < self.dist_to_wall:
            self.states = 1
        elif self.position_window['front'] > self.dist_to_wall and self.position_window['front_right'] < self.dist_to_wall \
            and self.position_window['front_left'] < self.dist_to_wall:
            self.states = 0   
        else:
            rospy.loginfo("Error locating position")


    def move(self, angle, speed=0.2):
        msgForward = Twist()
        msgForward.linear.x = speed
        msgForward.angular.z = angle

        self.pub.publish(msgForward)
        self.stop_moving()

    def rotate(self, speed=0.1):
        msgTurn = Twist()
        msgTurn.linear.x = 0
        msgTurn.angular.z = speed

        self.pub.publish(msgTurn)
        self.stop_moving()
         
    def stop_moving(self):
        msgStop = Twist()
        self.pub.publish(msgStop)

    def find_a_wall(self):

        if self.states == 0:
            self.move(-0.3)
        elif self.states == 1:
            self.rotate()
        elif self.states == 2:
            self.move(0)
        else:
            rospy.loginfo("Error")


if __name__ == '__main__':
    try:
        robot = WallFollower()
        robot.find_a_wall()
    except rospy.ROSInterruptException as e:
        print("An exception was caught.")
        raise e