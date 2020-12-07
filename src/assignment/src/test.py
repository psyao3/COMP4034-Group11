#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Twist, Pose2D, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
import cv2, cv_bridge
import numpy as np
import math
import random
import tf
import tf2_ros
import tf2_geometry_msgs
import struct
import actionlib
from darknet_ros_msgs.msg import BoundingBoxes
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from calculator import calculate_linear_distance, calculate_angular_distance, average_distance
from image_processing import generate_mask, convert_to_hsv, show_image, find_closest_centroid
from actionlib_msgs.msg import *


class Follower:

    def __init__(self):

        # Initialising rospy behaviour
        rospy.on_shutdown(self.stop)
        self.rate = rospy.Rate(10)  # Hz

        # Speeds to use
        self.linear_speed = 0.4
        self.angular_speed = 0.05

        # Publishers and Subscribers
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.detection_img = rospy.Subscriber('/darknet_ros/detection_image', Image, self.image_callback)
        self.image_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.box_callback)
        self.depth_sub = rospy.Subscriber('camera/depth/image_raw', Image, self.depth_callback)
        #self.cloud_sub = rospy.Subscriber('camera/depth/points', PointCloud2, self.cloud_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        # Initialise robot state
        self.state = 'Random Walk'

        # Initalise pose objects for tracking distance/angle moved in the random walk.
        self.pose = Pose2D()
        self.initial_pose = Pose2D()
        self.random_angle = 0.0
        self.beginning_of_random_walk = True

        # Initialise image processing
        self.bridge = cv_bridge.CvBridge()
        self.mask, self.masked_image, self.target = None, None, None

        # Initialise ranges
        self.ranges = None
        self.closest_obstacle = None
        self.right_obst, self.front_obst, self.left_obst = None, None, None


        self.goal_x =0
        self.goal_y =0
        self.current_goal = False
        self.objects = []
        self.facing_object = False

        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        while not rospy.is_shutdown():

            if len(self.objects) >0:
                # If there are targets, visit them
                rospy.loginfo("Go to target.")
                current_target = self.objects.pop(0)
                

                rospy.loginfo("Sending goal location")
                client.send_goal(current_target)

                client.wait_for_result(rospy.Duration(100))

                if (client.get_state() == GoalStatus.SUCCEEDED):
                    rospy.loginfo("You have reached the destination")
                    
                    twist = Twist()
                    twist.angular.z = 1.57/2    
                    for i in range(5):         
                        self.pub.publish(twist)
                        self.rate.sleep()
                    
                    self.pub.publish(Twist())

                    self.state == 'Random Walk'
                    self.current_goal = False
                    self.facing_object = False
                
                else:
                    rospy.loginfo("The robot failed to reach the destination")
     
            else:
                pass


    def stop(self):
        self.pub.publish(Twist())


    def depth_callback(self, data):
        '''
        centralises the bounding box on the robot vision and creates a move base goal 
        '''
        
        if self.state == 'Sending Target':

            depth_image = self.bridge.imgmsg_to_cv2(data, "passthrough")

            _, w = depth_image.shape # width
            twist = Twist()
            

            #distance to object
            depth = depth_image[self.goal_y,self.goal_x]
            
            if not self.facing_object:
                rospy.loginfo ("depth: {} ".format(depth))
                if not ((w//2) -100  <=  self.goal_x and self.goal_x <= (w//2) +100):

                    #currently just rotate left
                    twist.angular.z = self.angular_speed    
                                 
                    self.pub.publish(twist)
                    self.rate.sleep()
                else:
                    self.facing_object = True



            else:


                goal = MoveBaseGoal()

                # using base_link
                
                goal.target_pose.header.frame_id = "base_link"
                goal.target_pose.header.stamp = rospy.Time.now()

                # set positions of the goal location
                goal.target_pose.pose.position = Point(depth-0.5, 0, 0)
                goal.target_pose.pose.orientation.x = 0.0
                goal.target_pose.pose.orientation.y = 0.0
                goal.target_pose.pose.orientation.z = 0.0
                goal.target_pose.pose.orientation.w = 1.0

            
                self.state = 'To Target'
                self.objects.append(goal)


    def box_callback(self, data):

        if not self.facing_object:

            for box in data.bounding_boxes:
                rospy.loginfo(
                    "Xmin: {}, Xmax: {} Ymin: {}, Ymax: {}".format(
                        box.xmin, box.xmax, box.ymin, box.ymax
                    )
                )
                # update the x and y values for the bounding box
                x = (box.xmax + box.xmin)//2
                y = (box.ymax + box.ymin)//2

                self.goal_x = x
                self.goal_y = y

                rospy.loginfo("X: {},  Y: {}".format( x, y))
                
                self.state = 'Sending Target'

                #self.current_goal = True

                

            

                break


    def odom_callback(self, msg):
        # Get (x, y, theta) specification from odometry topic
        quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)

        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

    def scan_callback(self, msg):
        self.ranges = np.array(msg.ranges)

        obst_dists = []
        for i in range(-45, 45, 8):
            obst_dists.append(average_distance(self.ranges, i, 8))

        self.right_obst = min(obst_dists[0:5])
        self.front_obst = min(obst_dists[5:7])
        self.left_obst = min(obst_dists[7:])

        self.closest_obstacle = min(self.front_obst, self.left_obst, self.right_obst)


    def image_callback(self, msg):
        pass

        # Convert the image message to openCV type, scale size.
        #image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        

        #cv2.imshow("window",image)        
        #cv2.waitKey(1)

    def beacon_towards_closest_target(self, centroids, hsv, targets):
        if self.state == 'Target' and not rospy.is_shutdown():


            # If there are multiple green objects/targets, only follow one.
            # Remove the one further away from the image.

            # Need Image width to get centroid of turtlebot's view.
            _, w, _ = hsv.shape
            twist_msg = Twist()

            # Iterate over targets, keep largest one.
            largest_target = find_closest_centroid(centroids, w)

            # Keep only largest_target, remove others from image.
            #mask = np.where(targets == largest_target, np.uint8(255), np.uint8(0))
            obj_centroid = centroids[largest_target, 0]

            # Implement a proportional controller to beacon towards it
            
            twist_msg.linear.x = self.linear_speed
            twist_msg.angular.z = -float(err) / 400

            self.pub.publish(twist_msg)
        else:
            self.stop()

    def random_walk(self):
        if self.state == 'Random Walk' and not rospy.is_shutdown():

            # If it's the beginning of the random walk, initialise the starting pose
            if self.beginning_of_random_walk:
                rospy.loginfo("Begin Random Walk")
                self.initial_pose = Pose2D(self.pose.x, self.pose.y, self.pose.theta)
                self.random_angle = random.uniform(-math.pi, math.pi)
                while abs(self.random_angle) < 1:
                    self.random_angle = random.uniform(-math.pi, math.pi)
                self.beginning_of_random_walk = False

            # If there is an obstacle stop and restart the random walk
            # If closest_obstacle has not been initiated yet, wait until it has
            if self.closest_obstacle is not None and (self.state == "Obstacle" or self.closest_obstacle <= 0.25):
                self.state = "Obstacle"
                self.stop()
                self.beginning_of_random_walk = True
                return

            twist_msg = Twist()
            n_of_meters = 3

            if calculate_linear_distance(self.initial_pose, self.pose) < n_of_meters:
                rospy.loginfo("Walked %.2f m out of %i m" % (
                    calculate_linear_distance(self.initial_pose, self.pose), n_of_meters))
                twist_msg.linear.x = min(self.linear_speed * self.closest_obstacle, self.linear_speed)
            elif calculate_angular_distance(self.initial_pose, self.pose) < self.random_angle:
                rospy.loginfo("Turned %.3f radians out of %.2f" % (
                    calculate_angular_distance(self.initial_pose, self.pose), self.random_angle))
                twist_msg.angular.z = self.angular_speed
            else:
                # If random walk is completed restart
                self.beginning_of_random_walk = True

            self.pub.publish(twist_msg)

        else:
            self.stop()
            self.beginning_of_random_walk = True


if __name__ == '__main__':
    try:
        rospy.init_node('follower')
        follower = Follower()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print("An exception was caught.")
        raise e


'''
def cloud_callback(self, data):


    width = data.width
    height = data.height
    point_step = data.point_step
    row_step = data.row_step

    array_pos = self.goal_y*row_step + self.goal_x*point_step

    (X, Y, Z) = struct.unpack_from('fff', data.data, offset=array_pos)


    # rospy.loginfo("x: {}, y: {}, z: {}".format(X,Y,Z))
'''
        