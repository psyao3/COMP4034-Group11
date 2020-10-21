#!/usr/bin/env python

## Open loop control
# Make the turtlebot go around in a square of 1m edge length in open-loop fashion
# 1. Drive forward a distance d (1m).
# 2. Rotate by angle alpha (90 degrees, but as a quaternion).

import rospy
import math
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def talker():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(100) # 10hz

    # Define the twist messages to send
    # Forward at 1 m/s
    forward = Twist()
    forward.linear.x = 0.1

    # Angular at pi/2 radians/s
    angular = Twist()
    angular.angular.z = (math.pi/2)/10
  
    # Flag to alternate moving forward/ rotating
    flag = True

    # Count actions so we can stop after 8; 4 moves and 4 turns, so the square is complete
    actions = 0
    time.sleep(5)

    while not rospy.is_shutdown() and actions < 8:
        
        t = rospy.Time.now().to_sec()
        
        while rospy.Time.now().to_sec() - t < rospy.Duration(10).to_sec():
            if flag:
                # Move forward
                pub.publish(forward)
                rospy.loginfo("moving forward")
                rate.sleep()
            else:
                # Rotate
                pub.publish(angular)
                rospy.loginfo("rotating")
                rate.sleep()
        # Stop
        pub.publish(Twist())  
        # Alternate
        flag = not flag
        actions += 1  
    # Finish
    pub.publish(Twist())  


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
