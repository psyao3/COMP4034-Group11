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
from actionlib_msgs.msg import *
from std_msgs.msg import Float32MultiArray
from frontier_methods import *

import cv2, cv_bridge


## NOTE: to extract the callbacks from the main class, the data/msg variable is
# now the first parameter, and 'self' (the instance of the main class) is passed
# in second. This is because self needs to be passed in as an extra parameter
# when creating the subscriber, rather than calling self.callback.
     
# Updates the number of bounding boxes currently seen     
def count_boxes_callback(msg, self):
    self.box_count = msg.count

    if self.box_count == 0:
        #rospy.loginfo("No target found, resetting values.")
        self.is_target = False
        self.current_tgt_class = None
        self.goal_x = None
        self.goal_y = None
        return



    '''def box_callback(data, self):

    # We need to ignore targets or boxes belonging to a class
    # we've already found.
    ignore = self.complete

    # If no boxes set values and return early.
    # Check each bounding box found, ignore if already reached that class
    boxes = []

    for box in data.bounding_boxes:
        # ignore mailbox for now, beacon only to legs
        if box.Class not in ignore and box.Class != "mailbox":
            boxes.append(box)

   

    # Break after variables set.
    for box in boxes:
       
        # Get the class
        obj_class = box.Class
        
        # Get the centers of the bounding box (x is needed for prop controller)
        x = ((box.xmax - box.xmin)/2)+box.xmin
        y = ((box.ymax - box.ymin)/2)+box.ymin

        # x for beaconing
        self.goal_x = x

        # y for identifying when we've reached an object (and stopping)
        # This works mostly, but need special case for number 5.
        self.goal_y = box.ymax

        # Use height and width for telling when we've arrived at number 5.
        self.box_height = box.ymax - box.ymin 
        self.box_width = box.xmax - box.xmin


       # rospy.loginfo("Class: {}, X: {},  Y: {}".format(obj_class, x, y))
        
        self.state = 'Sending Target'
        self.current_tgt_class = obj_class

        self.is_target = True

        # Cancel current move_base goal if there is one.
        self.cancel_pub.publish(GoalID()) # {} empty ID should cancel all goals.

        # Break loop now we have target
        break'''



def odom_callback(msg, self):
        # Get (x, y, theta) specification from odometry topic
        quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)

        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y
       #rospy.loginfo("Odom positon: %f, %f" % (self.pose.x, self.pose.y))

       
def scan_callback(msg, self):
    self.ranges = np.array(msg.ranges)

    obst_dists = []
    for i in range(-84, 84, 10):
        obst_dists.append(average_distance(self.ranges, i, 10))

    self.right_obst = min(obst_dists[0:5])
    self.front_obst = min(obst_dists[5:7])
    self.left_obst = min(obst_dists[7:])

    self.any_obst = min(min(self.ranges[0:75]), min(self.ranges[285:]))

    self.closest_obstacle = min(self.front_obst, self.left_obst, self.right_obst)


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
        if box.Class not in ignore and box.Class != "mailbox":
            boxes.append(box)

    # If boxes is still empty, it means no new targets found,
    # set variables and return early. (empty list is false)
    if not boxes:
        return
    '''if not boxes:
        self.is_target = False
        self.current_tgt_class = None
        return'''
    
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

    # Use height and width for telling when we've arrived at number 5.
    self.box_height = box.ymax - box.ymin 
    self.box_width = box.xmax - box.xmin

    # rospy.loginfo("Class: {}, X: {},  Y: {}".format(obj_class, x, y))
    
    self.state = 'Sending Target'
    self.current_tgt_class = closest_box.Class


    self.is_target = True
   
    # Cancel current move_base goal if there is one.
    self.cancel_pub.publish(GoalID()) # {} empty ID should cancel all goals.



def grid_callback(msg, self):
    # Initialises the occupancy grid with values of -1 from the global costmap which gets published once
    # The shape is 384x384
    self.occ_grid = np.array([-1 for cell in msg.data], dtype=np.int8).reshape(msg.info.height, msg.info.width)
    self.occ_grid_info = msg.info
    grid_to_pub = Float32MultiArray()
    grid_to_pub.data = self.occ_grid.flatten()
    self.occ_grid_publisher.publish(grid_to_pub)


def update_callback(msg, self):
    # Updates the occupancy grid with the values obtained in the update
    # The update is a subset of the global costmap
    update = np.array(msg.data, dtype=np.int8).reshape(msg.height, msg.width)
    self.occ_grid[msg.y:msg.y + msg.height, msg.x:msg.x + msg.width] = update
    # Creates a float array to publish the map on the /occ_grid topic
    grid_to_pub = Float32MultiArray()
    grid_to_pub.data = self.occ_grid.flatten()
    self.occ_grid_publisher.publish(grid_to_pub)


def status_callback(msg, self):
    self.goal_status = msg.status.status



def depth_callback(data, self):
    '''
    centralises the bounding box on the robot vision and creates a move base goal 
    '''  
    if self.get_depth == True:

        rospy.loginfo ("Starting depth targeting")

        depth_image = self.bridge.imgmsg_to_cv2(data, "passthrough")

        _, w = depth_image.shape # width
        twist = Twist()

        # distance to object
        depth = depth_image[self.goal_y,self.goal_x]

        x_region = range(self.goal_x - self.box_width/4, self.goal_x+ self.box_width/4)
        y_region = range(self.goal_y - self.box_height/4, self.goal_y + self.box_height/4)
        
        depth_avg = 0
        for x in x_region:
            for y in y_region:
                depth_avg += depth_image[y][x]
        depth_avg = depth_avg/(len(x_region)+ len(y_region))
        
        '''if not self.facing_object:
            rospy.loginfo ("depth: {} ".format(depth))
            if not ((w//2) - (self.box_width/2)  <=  self.goal_x and self.goal_x <= (w//2) + (self.box_width/2)):

                if (self.goal_x > w//2):
            
                    twist.angular.z = -self.angular_speed    
                else:
                    twist.angular.z = self.angular_speed  
                                 
                self.pub.publish(twist)
                self.rate.sleep()
            else:
                self.facing_object = True
        else:'''


        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        while not client.wait_for_server(rospy.Duration.from_sec(5.0)):
            rospy.loginfo("Waiting for the move_base action server")


        goal = MoveBaseGoal()

        # using base_link
        
        goal.target_pose.header.frame_id = "base_link"
        goal.target_pose.header.stamp = rospy.Time.now()

        # set positions of the goal location
        goal.target_pose.pose.position.x = depth -0.3
        goal.target_pose.pose.orientation.w = 1.0


        rospy.loginfo("Sending goal location on base_link")
        client.send_goal(goal)

        client.wait_for_result(rospy.Duration(60))
    
        if (client.get_state() ==  GoalStatus.SUCCEEDED):
            rospy.loginfo("You have reached the destination")        
        else:
            rospy.loginfo("The robot failed to reach the destination")


        self.complete.append(self.current_tgt_class)
        self.is_target = False
        self.facing_object = False
        self.depth = False