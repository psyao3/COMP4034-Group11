#!/usr/bin/env python

import tf
import numpy as np
from calculator import *

import rospy

from geometry_msgs.msg import Twist, Pose2D, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, PointCloud2, LaserScan
from darknet_ros_msgs.msg import BoundingBoxes
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from calculator import calculate_linear_distance, calculate_angular_distance, average_distance
from image_processing import generate_mask, convert_to_hsv, show_image, find_closest_centroid
from actionlib_msgs.msg import *

## NOTE: to extract the callbacks from the main class, the data/msg variable is
# now the first parameter, and 'self' (the instance of the main class) is passed
# in second. This is because self needs to be passed in as an extra parameter
# when creating the subscriber, rather than calling self.callback.
  

def depth_callback(data, self):
    '''
    centralises the bounding box on the robot vision and creates a move base goal 
    '''
    
    if self.state == 'Sending Target':

        depth_image = self.bridge.imgmsg_to_cv2(data, "passthrough")

        _, w = depth_image.shape # width
        twist = Twist()
        

        # distance to object
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
            goal.target_pose.pose.position = Point(depth, 0, 0)
            goal.target_pose.pose.orientation.x = 0.0
            goal.target_pose.pose.orientation.y = 0.0
            goal.target_pose.pose.orientation.z = 0.0
            goal.target_pose.pose.orientation.w = 1.0

        
            self.state = 'To Target'
            self.objects.append(goal)


#def camera_info_callback(self, data):

    


def box_callback(data, self):

    # We need to ignore targets or boxes belonging to a class
    # we've already found.
    ignore = self.complete

    # If no boxes set values and return early.
    # Check each bounding box found, ignore if already reached that class
    boxes = []

    for box in data.bounding_boxes:
        if box.Class not in ignore:
            boxes.append(box)

    # If boxes is still empty, it means no new targets found,
    # set variables and return early. (empty list is false)
    if not boxes:
        self.is_target = False
        self.current_tgt_class = None
        return


    # Break after variables set.
    for box in data.bounding_boxes:
       
        # Get the class
        obj_class = box.Class
        
        # Get the centers of the bounding box (x is needed for prop controller)
        x = ((box.xmax - box.xmin)/2)+box.xmin
        y = ((box.ymax - box.ymin)/2)+box.ymin

        # x for beaconing
        self.goal_x = x

        # y for identifying when we've reached an object (and stopping)
        self.goal_y = box.ymax

       # rospy.loginfo("Class: {}, X: {},  Y: {}".format(obj_class, x, y))
        
        self.state = 'Sending Target'
        self.current_tgt_class = obj_class

        self.is_target = True

        # Break loop now we have target
        break




def odom_callback(msg, self):
        # Get (x, y, theta) specification from odometry topic
        quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)

        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

       
def scan_callback(msg, self):
    self.ranges = np.array(msg.ranges)

    obst_dists = []
    for i in range(-45, 45, 8):
        obst_dists.append(average_distance(self.ranges, i, 8))

    self.right_obst = min(obst_dists[0:5])
    self.front_obst = min(obst_dists[5:7])
    self.left_obst = min(obst_dists[7:])

    self.closest_obstacle = min(self.front_obst, self.left_obst, self.right_obst)

'''
# alternate implementation that checks for smallest. ignore atm
def box_callback(data, self):

    # We need to ignore targets or boxes belonging to a class
    # we've already found.
    ignore = self.complete

    ## TODO: if there are multiple boxes in view
    # then we pick the largest one as the current target
    # as it makes sense that one is closest.

    # Check each bounding box found, ignore if already reached that class
    boxes = []

    for box in data.bounding_boxes:
        if box.Class not in ignore:
            boxes.append(box)

    # If boxes is still empty, it means no new targets found,
    # set variables and return early. (empty list is false)
    if not boxes:
        self.is_target = False
        self.current_tgt_class = None
        return
    
    # Now iterate through boxes to find one with the largest area
    # although most of the time there will only be 1 (and at most 2)
    best_size = 0
    closest_box = None
    for box in boxes:
        box_area = (box.xmax - box.xmin) * (box.ymax - box.ymin)
        if box_area >= best_size:
            best_size = box_area
            closest_box = box

    # Now we know we have the closest box (or at least the largest)
    # we can continue
        
    # Get the centers (centroids) of the bounding box (x is needed for prop controller)
    x = (closest_box.xmax - closest_box.xmin)/2 + closest_box.xmin
    y = (closest_box.ymax - closest_box.ymin)/2 + closest_box.ymin

    # x for beaconing
    self.goal_x = x

    # y for identifying when we've reached an object (and stopping)
    self.goal_y = box.ymax

    # rospy.loginfo("Class: {}, X: {},  Y: {}".format(obj_class, x, y))
    
    self.state = 'Sending Target'
    self.current_tgt_class = closest_box.Class

    self.is_target = True
'''