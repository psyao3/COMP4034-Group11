#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class square:
    def __init__(self):

        rospy.on_shutdown(self.cleanup)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.sleep(1)
        r = rospy.Rate(5.0)

        while not rospy.is_shutdown():
            twist = Twist()
            twist.linear.x = 0.15
            for i in range(10):         # 10*5hz = 2sec
                self.pub.publish(twist)
                r.sleep()
          
            twist = Twist()
            twist.angular.z = 1.57/2    
            for i in range(10):         # 10*5hz = 2sec
                self.pub.publish(twist)
                r.sleep()
 
    def cleanup(self):
        # stop the robot!
        twist = Twist()
        self.pub.publish(twist)
 
if __name__=="__main__":
    rospy.init_node('square')
    square()
 