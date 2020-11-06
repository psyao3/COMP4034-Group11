#!/usr/bin/env python
import rospy
import tf
from pylab import *
from math import pi, sqrt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Pose2D

class Square_movement:

    def __init__(self):
      
        
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.angular_subscriber = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.vel_msg = Twist()

        self.speed = 0.15
        self.distance = 1
        self.angle = 45
        self.angular_speed = 0.565*2*pi/360
        self.relative_angle = self.angle*2*pi/360

        self.pose = Pose2D()
        self.pose.x=5
        self.pose.y=5
        self.thetaa=0
        self.pose.theta=0

        self.move()


    def move(self):
        while not rospy.is_shutdown():

            #Setting the current time for distance calculations
            t0 = rospy.Time.now().to_sec()
            current_distance = 0
            edges = 1
  
            #Loop to move the turtle for the specified distance
            while(current_distance < self.distance ):
                print("robot moving")
                self.vel_msg.linear.x = abs(self.speed)

                self.vel_msg.linear.y = 0
                self.vel_msg.linear.z = 0

                self.vel_msg.angular.x = 0
                self.vel_msg.angular.y = 0
                self.vel_msg.angular.z = 0

                print(str(current_distance) + "   "  + str(self.distance))
                #Publish the velocity
                self.velocity_publisher.publish(self.vel_msg)
                #Takes actual time
                t1=rospy.Time.now().to_sec()
                #Calculates distance travelled
                current_distance= self.speed*(t1-t0)

            self.vel_msg = Twist()
            self.velocity_publisher.publish(self.vel_msg)

            current_angle = 0 

            self.thetaa = self.thetaa +(math.pi/2)
            prev_theta = 0

            while(current_angle < self.relative_angle/2/2/2):

                self.vel_msg.linear.x = 0
                self.vel_msg.linear.y = 0
                self.vel_msg.linear.z = 0

                self.vel_msg.angular.x = 0
                self.vel_msg.angular.y = 0
                self.vel_msg.angular.z = abs(0.5)

                print(str(current_angle) + "   "  + str(self.relative_angle))
                self.velocity_publisher.publish(self.vel_msg)
                t1 = rospy.Time.now().to_sec()
                current_angle = self.angular_speed*(t1-t0)
        
        #After the loop, stops the robot
        vel_msg.linear.x = 0
        #Force the robot to stop
        velocity_publisher.publish(vel_msg)
    
    def odom_callback(self, msg):
        # Get (x, y, theta) specification from odometry topic
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)

        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

if __name__ == '__main__':
    try:
           
        rospy.init_node('Square_movement', anonymous=True)
        sm = Square_movement()
   
    except rospy.ROSInterruptException: pass

