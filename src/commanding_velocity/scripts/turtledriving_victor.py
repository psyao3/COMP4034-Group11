#!/usr/bin/env python2
import math
import rospy
from geometry_msgs.msg import Twist


def turtlebotdriving():

    rospy.init_node('turtlebotdriving', anonymous=True)

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    forward = Twist()
    forward.linear.x = 0.1

    rotate = Twist()
    rotate.linear.x = 0
    rotate.angular.z = (math.pi/2)/10

    postion = True
    while not rospy.is_shutdown():
        r = rospy.Rate(10)
        t = rospy.Time.now().to_sec()

        while rospy.Time.now().to_sec() - t < rospy.Duration(10).to_sec():
            if postion:
                rospy.loginfo("Moving straight")
                pub.publish(forward)
                r.sleep()
            else:
                rospy.loginfo("Rotating 45 degrees")
                pub.publish(rotate)
                r.sleep()
        pub.publish(Twist())        
        postion = not postion
    
    pub.publish(Twist())

if __name__ == "__main__":
    try:
        turtlebotdriving()
    except rospy.ROSInterruptException: pass    