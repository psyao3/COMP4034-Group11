#!/usr/bin/env python

import math
import rospy
import tf
import visualiser 

from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class WallFollower():

    def __init__(self):

        # Initialise rospy and the node
        rospy.on_shutdown(self.stop_moving)
        rospy.init_node('wall_follower', anonymous=True)
        self.dist_to_wall = 0.6
        self.states = 0
        self.position_window = {
            'right': 0,
            'front_right': 0,
            'front': 0,
            'front_left': 0,
            'left': 0,
        }
        # Create publisher and subscriber
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.laserscan_callback )

        # Set the frequency
        rospy.Rate(10)
        
    def laserscan_callback(self, msg):
        ranges = msg.ranges

        self.position_window = {
            'right': min(ranges[270:306]),
            'front_right': min(ranges[307:342]),
            'front': min(min(ranges[0:16]),min(ranges[343:359])),
            'front_left': min(ranges[17:53]),
            'left': min(ranges[54:90]),
        }

        self.position_decider()

    def position_decider(self):

        if self.position_window['front'] > self.dist_to_wall and self.position_window['front_right'] > self.dist_to_wall \
            and self.position_window['front_right'] > self.dist_to_wall and self.position_window['right'] > self.dist_to_wall:
            # random_wander maybe?
            self.states = 0
        elif self.position_window['front'] < self.dist_to_wall and self.position_window['front_right'] > self.dist_to_wall \
            and self.position_window['front_left'] > self.dist_to_wall:
            self.states = 1
        elif self.position_window['front'] > self.dist_to_wall and self.position_window['front_right'] < self.dist_to_wall \
            and self.position_window['front_left'] > self.dist_to_wall:
            self.states = 3
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


    def move(self, angle, speed):
        msgForward = Twist()
        msgForward.linear.x = speed
        msgForward.angular.z = abs(angle)

        return msgForward

    def rotate(self, direction, speed):
        msgTurn = Twist()
        msgTurn.linear.x = 0
        if direction == 'left':
            msgTurn.angular.z = abs(speed)
        elif direction == 'stuck':
            msgTurn.linear.x = 0.1
            msgTurn.angular.z = abs(speed)

        return msgTurn
         
    def stop_moving(self):
        msgStop = Twist()
        
        return msgStop

    def wall_following(self):

        msg = Twist()

        while not rospy.is_shutdown():
            if self.states == 0:
                rospy.loginfo("finding wall")
                msg = self.move(0,0.2)
            elif self.states == 1:
                rospy.loginfo("rotating left")
                msg = self.rotate('left', 0.2)
            elif self.states == 2:
                rospy.loginfo("moving away from wall")
                msg = self.rotate('stuck', 0.2)
            elif self.states == 3:
                rospy.loginfo("following wall")
                msg = self.move(0, 0.2)
            else:
                rospy.loginfo("Error")
            
            self.pub.publish(msg)

if __name__ == '__main__':
    try:
        robot = WallFollower()
        robot.wall_following()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print("An exception was caught.")
        raise e
